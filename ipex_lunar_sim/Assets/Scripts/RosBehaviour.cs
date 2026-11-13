using System;
using UnityEngine;
using ROS2;

/// <summary>
/// Base class for every concrete class that wants to interface with ROS.
/// </summary>
public abstract class RosBehaviour : MonoBehaviour
{
    [SerializeField] protected RosBridge bridge;
    private bool subscribed;
    private bool warnedNoBridge;

    protected ROS2Node RosNode { get; private set; }
    protected bool RosReady => RosNode != null;

    protected abstract void OnRosReady();

    protected virtual void Awake()
    {
        ResolveBridge();
    }

    protected virtual void OnEnable()
    {
        if (RosReady || !ResolveBridge()) return;
        if (bridge.IsReady)
        {
            HandleNodeReady(bridge.RosNode);
        }
        else if (!subscribed)
        {
            bridge.NodeReady += HandleNodeReady;
            subscribed = true;
        }
    }

    protected virtual void OnDisable()
    {
        Unsubscribe();
    }

    private bool ResolveBridge()
    {
        if (bridge == null && !warnedNoBridge)
        {
            Debug.LogError($"{GetType().Name}: assign RosBridge.", this);
            warnedNoBridge = true;
        }
        return bridge != null;
    }

    private void HandleNodeReady(ROS2Node node)
    {
        Unsubscribe();
        if (RosReady) return;
        RosNode = node;
        try
        {
            OnRosReady();
        }
        catch (Exception e)
        {
            Debug.LogException(e, this);
        }
    }

    private void Unsubscribe()
    {
        if (subscribed && bridge != null)
        {
            bridge.NodeReady -= HandleNodeReady;
        }
        subscribed = false;
    }

    // T must inherit from ROS.Message class and must have a constructor without parameters
    protected IPublisher<T> Advertise<T>(string topic, QualityOfServiceProfile qos = null) where T : Message, new()
    {
        return RosNode.CreatePublisher<T>(RosQos.Topic(topic), qos);
    }

    protected ISubscription<T> Subscribe<T>(string topic, Action<T> callback, QualityOfServiceProfile qos = null) where T : Message, new()
    {
        return RosNode.CreateSubscription<T>(RosQos.Topic(topic), callback, qos);
    }

    protected void Stamp(MessageWithHeader msg)
    {
        RosNode.clock.UpdateROSTimestamp(ref msg);
    }
}

public static class RosQos
{
    public static QualityOfServiceProfile SensorData() => new QualityOfServiceProfile(QosPresetProfile.SENSOR_DATA);

    public static QualityOfServiceProfile Reliable(int depth = 10)
    {
        var qos = new QualityOfServiceProfile();
        qos.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_VOLATILE);
        qos.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE);
        qos.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, depth);
        return qos;
    }

    public static QualityOfServiceProfile Latched()
    {
        var qos = new QualityOfServiceProfile();
        qos.SetDurability(DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
        qos.SetReliability(ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE);
        qos.SetHistory(HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST, 1);
        return qos;
    }

    public static string Topic(string name)
    {
        if (string.IsNullOrEmpty(name) || name[0] == '/') return name;
        return "/" + name;
    }
}