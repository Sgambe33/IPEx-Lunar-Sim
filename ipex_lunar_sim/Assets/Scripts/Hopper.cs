using UnityEngine;
using ROS2;

/// <summary>
/// Handles hopper behaviour and acts as a ROS node that publishes hopper status.
/// </summary>
public class Hopper : RosBehaviour
{
    [SerializeField] private float length = 0.5f;
    [SerializeField] private float width = 0.7f;
    [SerializeField] private float wallThickness = 0.03f;
    [SerializeField] private float height = 0.25f;
    [SerializeField] private float publishHz = 2f;

    private IPublisher<ipex_interfaces.msg.HopperState> publisher;
    private ipex_interfaces.msg.HopperState hopperState;
    private float nextPublish;

    public float Length => length;
    public float Width => width;
    public float WallThickness => wallThickness;
    public float Height => height;
    public float DeliveredKg { get; private set; }

    protected override void OnRosReady()
    {
        hopperState = new ipex_interfaces.msg.HopperState();
        publisher = Advertise<ipex_interfaces.msg.HopperState>("/isru/hopper", RosQos.Reliable());
    }

    void FixedUpdate()
    {
        if (publisher == null || Time.time < nextPublish) return;
        nextPublish = Time.time + 1f / publishHz;
        hopperState.Delivered_kg = DeliveredKg;
        publisher.Publish(hopperState);
    }

    public bool Contains(Vector3 world)
    {
        Vector3 local = transform.InverseTransformPoint(world);
        return Mathf.Abs(local.z) <= 0.5f * length && Mathf.Abs(local.x) <= 0.5f * width;
    }

    public float Receive(float kg)
    {
        float accepted = Mathf.Max(kg, 0f);
        DeliveredKg += accepted;
        return accepted;
    }
}