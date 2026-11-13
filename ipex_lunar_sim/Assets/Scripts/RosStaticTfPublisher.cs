using UnityEngine;
using ROS2;
using System;

/// <summary>
/// Publishes on /tf_static the static transforms of rover components relative to base_link.
/// </summary>
public class RosStaticTfPublisher : RosBehaviour
{
    [Serializable]
    public struct StaticLink
    {
        public string parentFrame;
        public string childFrame;
        public Transform transform;
    }

    [SerializeField] private StaticLink[] links;
    [SerializeField] private Transform baseLinkTransform;

    // rotation needed for correct visualization of images in Rviz2/OpenCV
    private static readonly Quaternion opticalFrameCorrection = new Quaternion(-0.5f, 0.5f, -0.5f, 0.5f);

    private IPublisher<tf2_msgs.msg.TFMessage> tfStaticPub;

    protected override void Awake()
    {
        base.Awake();
        if (baseLinkTransform == null)
        {
            Debug.LogError("RosStaticTfPublisher: assign baseLinkTransform.", this);
            enabled = false;
        }
    }

    protected override void OnRosReady()
    {
        tfStaticPub = Advertise<tf2_msgs.msg.TFMessage>("/tf_static", RosQos.Latched());
        PublishTransforms();
    }

    private void PublishTransforms()
    {
        var msg = new tf2_msgs.msg.TFMessage
        {
            Transforms = new geometry_msgs.msg.TransformStamped[links.Length]
        };

        for (int i = 0; i < links.Length; i++)
        {
            var link = links[i];
            var ts = new geometry_msgs.msg.TransformStamped();

            ts.Header.Frame_id = link.parentFrame;
            Stamp(ts);
            ts.Child_frame_id = link.childFrame;

            Transform parent = ParentTransform(link.parentFrame);
            Vector3 relativePos = parent.InverseTransformPoint(link.transform.position);
            Quaternion relativeRot = Quaternion.Inverse(parent.rotation) * link.transform.rotation;

            Vector3 rosPos = relativePos.Unity2Ros();
            Quaternion rosRot = relativeRot.Unity2Ros();

            if (link.childFrame.EndsWith("_camera"))
            {
                rosRot = rosRot * opticalFrameCorrection;
            }

            ts.Transform.Translation.X = rosPos.x;
            ts.Transform.Translation.Y = rosPos.y;
            ts.Transform.Translation.Z = rosPos.z;

            ts.Transform.Rotation.X = rosRot.x;
            ts.Transform.Rotation.Y = rosRot.y;
            ts.Transform.Rotation.Z = rosRot.z;
            ts.Transform.Rotation.W = rosRot.w;

            msg.Transforms[i] = ts;
        }

        tfStaticPub.Publish(msg);
    }

    private Transform ParentTransform(string frame)
    {
        foreach (StaticLink link in links)
        {
            if (link.childFrame == frame && link.transform != null)
            {
                return link.transform;
            }
        }
        return baseLinkTransform;
    }
}