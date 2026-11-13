using UnityEngine;
using ROS2;

/// <summary>
/// Handles and publishes odometry dead reckoning on /odom the odom -> base_link transform.
/// </summary>
public class RosOdometryPublisher : RosBehaviour
{
    [SerializeField] private Transform baseLinkTransform;
    [SerializeField] private string odomFrameId = "odom";
    [SerializeField] private string baseFrameId = "base_link";
    [SerializeField] private float publishRateHz = 30f;

    [Header("Simulated drift")]
    [SerializeField] private bool enableNoise = true;
    [SerializeField] private float linearNoiseScaleStdDev = 0.02f;
    [SerializeField] private float angularNoiseScaleStdDev = 0.03f;
    [SerializeField] private float linearScaleError = 0.01f;
    [SerializeField] private float angularScaleError = 0.05f;

    [Header("Covariance")]
    [SerializeField] private double positionVariance = 0.001;
    [SerializeField] private double orientationVariance = 0.001;
    [SerializeField] private double linearVelVariance = 0.0005;
    [SerializeField] private double angularVelVariance = 0.0005;

    private ArticulationBody baseBody;
    private IPublisher<nav_msgs.msg.Odometry> odomPub;
    private IPublisher<tf2_msgs.msg.TFMessage> tfPub;
    private nav_msgs.msg.Odometry odomMsg;
    private geometry_msgs.msg.TransformStamped odomTf;
    private tf2_msgs.msg.TFMessage tfMsg;
    private Vector3 odomPos = Vector3.zero;
    private Quaternion odomRot = Quaternion.identity;
    private float publishInterval;
    private float nextPublishTime;

    protected override void Awake()
    {
        base.Awake();

        baseBody = baseLinkTransform != null ? baseLinkTransform.GetComponent<ArticulationBody>() : null;
        if (baseBody == null)
        {
            Debug.LogError("RosOdometryPublisher: assign baseLinkTransform.", this);
            enabled = false;
            return;
        }

        publishInterval = 1f / Mathf.Max(publishRateHz, 1f);
    }

    protected override void OnRosReady()
    {
        odomMsg = new nav_msgs.msg.Odometry();
        odomMsg.Header.Frame_id = odomFrameId;
        odomMsg.Child_frame_id = baseFrameId;
        FillDiagonalCovariance(odomMsg.Pose.Covariance, positionVariance, orientationVariance);
        FillDiagonalCovariance(odomMsg.Twist.Covariance, linearVelVariance, angularVelVariance);

        odomTf = new geometry_msgs.msg.TransformStamped();
        odomTf.Header.Frame_id = odomFrameId;
        odomTf.Child_frame_id = baseFrameId;
        tfMsg = new tf2_msgs.msg.TFMessage { Transforms = new[] { odomTf } };

        odomPub = Advertise<nav_msgs.msg.Odometry>("/odom", RosQos.Reliable(50));
        tfPub = Advertise<tf2_msgs.msg.TFMessage>("/tf", RosQos.Reliable(50));
    }

    void FixedUpdate()
    {
        if (baseBody == null) return;

        float dt = Time.fixedDeltaTime;

        Vector3 heading = Vector3.ProjectOnPlane(baseLinkTransform.forward, Vector3.up).normalized;
        Vector3 rightHand = Vector3.Cross(Vector3.up, heading);
        float forwardSpeed = Vector3.Dot(baseBody.linearVelocity, heading);
        float lateralSpeed = Vector3.Dot(baseBody.linearVelocity, rightHand);
        float yawRate = baseBody.angularVelocity.y;

        Vector3 deltaPos = new Vector3(lateralSpeed, 0f, forwardSpeed) * dt;
        float deltaYaw = yawRate * dt;

        if (enableNoise)
        {
            float linearNoiseScale = 1f + linearScaleError + RandomGaussian() * linearNoiseScaleStdDev;
            float angularNoiseScale = 1f + angularScaleError + RandomGaussian() * angularNoiseScaleStdDev;

            deltaPos *= linearNoiseScale;
            deltaYaw *= angularNoiseScale;
        }
        odomPos += odomRot * deltaPos;

        if (Mathf.Abs(deltaYaw) > 1e-9f)
        {
            odomRot = (odomRot * Quaternion.AngleAxis(deltaYaw * Mathf.Rad2Deg, Vector3.up)).normalized;
        }

        Vector3 reportedLinVel = deltaPos / dt;
        Vector3 reportedAngVel = new Vector3(0f, deltaYaw / dt, 0f);

        if (Time.time >= nextPublishTime)
        {
            nextPublishTime = Time.time + publishInterval;
            Publish(reportedLinVel, reportedAngVel);
        }
    }

    private void Publish(Vector3 localLinVel, Vector3 localAngVel)
    {
        if (odomPub == null) return;

        Stamp(odomMsg);

        Vector3 rosPos = odomPos.Unity2Ros();
        Quaternion rosRot = odomRot.Unity2Ros();
        Vector3 rosLinVel = localLinVel.Unity2Ros();
        Vector3 rosAngVel = -localAngVel.Unity2Ros(); //Unity2Ros does not handle angular speeds

        var pose = odomMsg.Pose.Pose;
        pose.Position.X = rosPos.x;
        pose.Position.Y = rosPos.y;
        pose.Position.Z = rosPos.z;
        pose.Orientation.X = rosRot.x;
        pose.Orientation.Y = rosRot.y;
        pose.Orientation.Z = rosRot.z;
        pose.Orientation.W = rosRot.w;

        var twist = odomMsg.Twist.Twist;
        twist.Linear.X = rosLinVel.x;
        twist.Linear.Y = rosLinVel.y;
        twist.Linear.Z = rosLinVel.z;
        twist.Angular.X = rosAngVel.x;
        twist.Angular.Y = rosAngVel.y;
        twist.Angular.Z = rosAngVel.z;

        odomPub.Publish(odomMsg);
        PublishTf(rosPos, rosRot);
    }

    private void PublishTf(Vector3 rosPos, Quaternion rosRot)
    {
        if (tfPub == null) return;

        var ts = odomTf;
        Stamp(ts);

        ts.Transform.Translation.X = rosPos.x;
        ts.Transform.Translation.Y = rosPos.y;
        ts.Transform.Translation.Z = rosPos.z;
        ts.Transform.Rotation.X = rosRot.x;
        ts.Transform.Rotation.Y = rosRot.y;
        ts.Transform.Rotation.Z = rosRot.z;
        ts.Transform.Rotation.W = rosRot.w;

        tfPub.Publish(tfMsg);
    }

    private static void FillDiagonalCovariance(double[] covariance, double linearVariance, double angularVariance)
    {
        for (int i = 0; i < 36; i++) covariance[i] = 0.0;
        covariance[0 * 6 + 0] = linearVariance;
        covariance[1 * 6 + 1] = linearVariance;
        covariance[2 * 6 + 2] = linearVariance;
        covariance[3 * 6 + 3] = angularVariance;
        covariance[4 * 6 + 4] = angularVariance;
        covariance[5 * 6 + 5] = angularVariance;
    }

    private static float RandomGaussian()
    {
        // https://en.wikipedia.org/wiki/Box%E2%80%93Muller_transform
        float u = Mathf.Max(Random.value, 1e-7f);
        float v = Random.value;
        return Mathf.Sqrt(-2f * Mathf.Log(u)) * Mathf.Cos(2f * Mathf.PI * v);
    }
}
