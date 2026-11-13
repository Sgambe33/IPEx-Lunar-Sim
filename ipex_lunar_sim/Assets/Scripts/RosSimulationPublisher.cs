using System.Collections.Generic;
using UnityEngine;
using ROS2;

/// <summary>
/// Class tasked with publishing useful data for visualization and navigation.
/// Publishes:
///   /ground_truth/odom    true pose of base_link
///   /dig_map              where regolith was dug or dumped on the ground
///   /obstacles            rocks taller than a threshold, hopper and the lander
/// </summary>
public class RosSimulationPublisher : RosBehaviour
{
    [SerializeField] private IPEXController controller;
    [SerializeField] private RegolithDeposit deposit;
    [SerializeField] private float groundTruthHz = 20f;
    [SerializeField] private float heightHz = 0.5f;

    [Header("Dig map")]
    [SerializeField] private Vector2 gridCenter = Vector2.zero;
    [SerializeField] private float gridSize = 30f;
    [SerializeField] private float gridResolution = 0.1f;
    [SerializeField] private float digMapMaxDepth = 0.08f;

    [Header("Obstacles")]
    [SerializeField] private Transform[] rockRoots;
    [SerializeField] private Transform lander;
    [SerializeField] private Hopper hopper;

    private IPublisher<nav_msgs.msg.Odometry> groundTruthPublisher;
    private nav_msgs.msg.Odometry groundTruthMsg;
    private IPublisher<nav_msgs.msg.OccupancyGrid> digMapPublisher;
    private IPublisher<visualization_msgs.msg.MarkerArray> obstaclePublisher;

    private float minObstacleHeight = 0.05f;
    private float obstacleRadius = 30f;
    private ArticulationBody baseBody;
    private float nextGt, nextHeight;
    private bool obstaclesSent;
    private int rows, cols;
    private float gridx0, gridy0;
    private sbyte[] gridData;


    protected override void Awake()
    {
        base.Awake();
        if (controller == null || deposit == null || lander == null || controller.BaseBody == null)
        {
            Debug.LogError("RosSimulationPublisher: assign controller, deposit and lander.", this);
            enabled = false;
            return;
        }

        baseBody = controller.BaseBody;
        if (rockRoots == null) rockRoots = new Transform[0];

        rows = Mathf.FloorToInt(gridSize / gridResolution) + 1;
        cols = rows;
        gridx0 = gridCenter.x - gridSize * 0.5f;
        gridy0 = gridCenter.y - gridSize * 0.5f;
        gridData = new sbyte[rows * cols];
    }

    protected override void OnRosReady()
    {
        groundTruthPublisher = Advertise<nav_msgs.msg.Odometry>("/ground_truth/odom", RosQos.Reliable());
        groundTruthMsg = new nav_msgs.msg.Odometry();
        groundTruthMsg.Header.Frame_id = "map";
        groundTruthMsg.Child_frame_id = "base_link";
        for (int i = 0; i < 36; i++) groundTruthMsg.Pose.Covariance[i] = 0.0;
        // small covariance to avoid zero division
        groundTruthMsg.Pose.Covariance[0] = groundTruthMsg.Pose.Covariance[7] = groundTruthMsg.Pose.Covariance[14] = 1e-6;
        groundTruthMsg.Pose.Covariance[21] = groundTruthMsg.Pose.Covariance[28] = groundTruthMsg.Pose.Covariance[35] = 1e-6;
        digMapPublisher = Advertise<nav_msgs.msg.OccupancyGrid>("/dig_map", RosQos.Latched());
        obstaclePublisher = Advertise<visualization_msgs.msg.MarkerArray>("/obstacles", RosQos.Latched());
    }

    void FixedUpdate()
    {
        if (groundTruthPublisher == null) return;
        float t = Time.time;

        if (t >= nextGt && baseBody != null)
        {
            nextGt = t + 1f / groundTruthHz;
            PublishGroundTruth();
        }

        if (deposit != null && deposit.IsReady)
        {
            if (t >= nextHeight)
            {
                nextHeight = t + 1f / heightHz;
                PublishDigMap();
            }

            if (!obstaclesSent)
            {
                PublishObstacles();
                obstaclesSent = true;
            }
        }
    }

    private void PublishGroundTruth()
    {
        var msg = groundTruthMsg;
        Stamp(msg);

        Transform roverTransform = baseBody.transform;
        Vector3 roverPosition = roverTransform.position.Unity2Ros();
        Quaternion roverRotation = roverTransform.rotation.Unity2Ros();
        msg.Pose.Pose.Position.X = roverPosition.x;
        msg.Pose.Pose.Position.Y = roverPosition.y;
        msg.Pose.Pose.Position.Z = roverPosition.z;
        msg.Pose.Pose.Orientation.X = roverRotation.x;
        msg.Pose.Pose.Orientation.Y = roverRotation.y;
        msg.Pose.Pose.Orientation.Z = roverRotation.z;
        msg.Pose.Pose.Orientation.W = roverRotation.w;

        Vector3 roverLinearVelocity = roverTransform.InverseTransformDirection(baseBody.linearVelocity).Unity2Ros();
        Vector3 roverAngularVelocity = -roverTransform.InverseTransformDirection(baseBody.angularVelocity).Unity2Ros();

        msg.Twist.Twist.Linear.X = roverLinearVelocity.x;
        msg.Twist.Twist.Linear.Y = roverLinearVelocity.y;
        msg.Twist.Twist.Linear.Z = roverLinearVelocity.z;

        msg.Twist.Twist.Angular.X = roverAngularVelocity.x;
        msg.Twist.Twist.Angular.Y = roverAngularVelocity.y;
        msg.Twist.Twist.Angular.Z = roverAngularVelocity.z;

        groundTruthPublisher.Publish(msg);
    }

    private void PublishDigMap()
    {
        for (int r = 0; r < rows; r++)
        {
            float y = gridy0 + r * gridResolution;
            for (int c = 0; c < cols; c++)
            {
                float x = gridx0 + c * gridResolution;
                float qty = deposit.QuantityChange(-y, x);
                int v = 0;
                if (qty < -1e-3f) v = Mathf.Clamp(Mathf.RoundToInt(-qty / digMapMaxDepth * 100f), 1, 100);
                else if (qty > 1e-3f) v = 1;
                gridData[r * cols + c] = (sbyte)v;
            }
        }

        var msg = new nav_msgs.msg.OccupancyGrid();
        msg.Header.Frame_id = "map";
        Stamp(msg);
        msg.Info.Resolution = gridResolution;
        msg.Info.Width = (uint)cols;
        msg.Info.Height = (uint)rows;
        msg.Info.Origin.Position.X = gridx0 - 0.5f * gridResolution;
        msg.Info.Origin.Position.Y = gridy0 - 0.5f * gridResolution;
        msg.Info.Origin.Orientation.W = 1.0;
        msg.Data = (sbyte[])gridData.Clone();
        digMapPublisher.Publish(msg);
    }

    private void PublishObstacles()
    {
        List<visualization_msgs.msg.Marker> markers = new List<visualization_msgs.msg.Marker>();
        int id = 0;

        if (lander != null)
        {
            Bounds landerBounds;
            if (TryBounds(lander, out landerBounds))
            {
                markers.Add(Cylinder("lander", id++, landerBounds.center, Mathf.Max(landerBounds.extents.x, landerBounds.extents.z), landerBounds.size.y, new Color(0.9f, 0.7f, 0.2f)));
            }
        }

        if (hopper != null && TryBounds(hopper.transform, out Bounds hopperBounds))
        {
            markers.Add(Cylinder("hopper", id++, hopperBounds.center, Mathf.Max(hopperBounds.extents.x, hopperBounds.extents.z), hopperBounds.size.y, new Color(0.9f, 0.9f, 0.9f)));
        }

        foreach (Transform root in rockRoots)
        {
            if (root == null) continue;
            foreach (Transform rock in root)
            {
                if (!TryBounds(rock, out Bounds rockBounds)) continue;
                Vector2 rp = new Vector2(rockBounds.center.z, -rockBounds.center.x);
                if (rp.magnitude > obstacleRadius) continue;

                float ground = deposit.GroundHeightAt(rockBounds.center.x, rockBounds.center.z);
                if (float.IsNaN(ground)) ground = rockBounds.min.y;

                float height = rockBounds.max.y - ground;
                if (height < minObstacleHeight) continue;
                markers.Add(Cylinder("rock", id++, rockBounds.center, Mathf.Max(rockBounds.extents.x, rockBounds.extents.z), height, new Color(0.6f, 0.6f, 0.6f)));
            }
        }

        visualization_msgs.msg.MarkerArray obstaclesArray = new visualization_msgs.msg.MarkerArray { Markers = markers.ToArray() };
        obstaclePublisher.Publish(obstaclesArray);
    }

    private visualization_msgs.msg.Marker Cylinder(string objName, int id, Vector3 unityCenter, float objRadius, float objHeight, Color color)
    {
        visualization_msgs.msg.Marker m = new visualization_msgs.msg.Marker();
        m.Header.Frame_id = "map";
        Stamp(m);
        m.Ns = objName;
        m.Id = id;
        m.Type = 3; // cylinder
        m.Action = 0; // add
        Vector3 p = unityCenter.Unity2Ros();
        m.Pose.Position.X = p.x;
        m.Pose.Position.Y = p.y;
        m.Pose.Position.Z = p.z;
        m.Pose.Orientation.W = 1.0;
        m.Scale.X = 2f * objRadius;
        m.Scale.Y = 2f * objRadius;
        m.Scale.Z = Mathf.Max(objHeight, 0.01f);
        m.Color.R = color.r;
        m.Color.G = color.g;
        m.Color.B = color.b;
        m.Color.A = 0.8f;
        return m;
    }

    private static bool TryBounds(Transform t, out Bounds bounds)
    {
        bounds = default;
        bool foundAny = false;
        foreach (Renderer rend in t.GetComponentsInChildren<Renderer>())
        {
            if (!foundAny)
            {
                bounds = rend.bounds;
                foundAny = true;
            }
            else bounds.Encapsulate(rend.bounds);
        }
        return foundAny;
    }
}