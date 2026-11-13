using UnityEngine;
using ROS2;
using System;
using System.Threading;

/// <summary>
/// ROS2-for-unity entry point. Contains the only ROS2Node istance and handles sim clock.
/// </summary>
[DefaultExecutionOrder(-1000)] // this ensures that at every update, RosBridge is updated first
public class RosBridge : MonoBehaviour
{
    [SerializeField] private string nodeName = "unity_node";
    [SerializeField] private bool publishClock = true;
    [SerializeField] private ROS2UnityComponent ros2Unity;
    private IPublisher<rosgraph_msgs.msg.Clock> clockPub;
    private rosgraph_msgs.msg.Clock clockMsg;
    private readonly SimClockTimeSource simTime = new SimClockTimeSource();
    private bool warnedMissingRos;

    public ROS2Node RosNode { get; private set; }
    public bool IsReady => RosNode != null;
    public event Action<ROS2Node> NodeReady;

    void Update()
    {
        if (RosNode == null)
        {
            TryCreateNode();
            return;
        }
    }

    void FixedUpdate()
    {
        simTime.Set(Time.fixedTimeAsDouble);
        if (clockPub == null) return;

        simTime.GetTime(out int sec, out uint nsec);
        clockMsg.Clock_.Sec = sec;
        clockMsg.Clock_.Nanosec = nsec;
        clockPub.Publish(clockMsg);
    }

    void TryCreateNode()
    {
        if (ros2Unity == null)
        {
            if (!warnedMissingRos)
            {
                Debug.LogError("RosBridge: assign ros2Unity.", this);
                warnedMissingRos = true;
            }
            return;
        }

        if (!ros2Unity.Ok()) return;

        RosNode = ros2Unity.CreateNode(nodeName);

        simTime.Set(Time.fixedTimeAsDouble);
        RosNode.clock = new ROS2Clock(simTime);

        if (publishClock)
        {
            clockMsg = new rosgraph_msgs.msg.Clock();
            clockPub = RosNode.CreatePublisher<rosgraph_msgs.msg.Clock>("/clock", RosQos.SensorData());
        }

        Debug.Log($"RosBridge: node '{nodeName}' created.", this);
        NodeReady?.Invoke(RosNode);
    }

    void OnDestroy()
    {
        if (RosNode != null && ros2Unity != null)
        {
            try { ros2Unity.RemoveNode(RosNode); }
            catch (Exception) { }
        }
        RosNode = null;
    }

    /// <summary>
    /// Custom ITimeSource implementation
    /// </summary>
    private class SimClockTimeSource : ITimeSource
    {
        private long nanoseconds;

        public void Set(double seconds) => Interlocked.Exchange(ref nanoseconds, (long)(seconds * 1e9));

        public void GetTime(out int seconds, out uint nanosec)
        {
            long ns = Interlocked.Read(ref nanoseconds);
            seconds = (int)(ns / 1000000000);
            nanosec = (uint)(ns % 1000000000);
        }
    }
}