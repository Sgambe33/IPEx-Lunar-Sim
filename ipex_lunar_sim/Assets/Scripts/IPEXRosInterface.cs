using ROS2;
using UnityEngine;

/// <summary>
/// Handles command inputs received via ROS topics.
/// <list type="bullet">
/// <item>/cmd_vel for linear and angular speed</item>
/// <item>/excavation/drum_{front,rear}_cmd  to control drums rotation</item>
/// <item>/excavation/arm_{front,rear}_cmd to control arms elevation</item>
/// </list>
/// </summary>
[RequireComponent(typeof(IPEXController))]
public class IPEXRosInterface : RosBehaviour
{
    [SerializeField] private string cmdVelTopic = "/cmd_vel";
    [SerializeField] private string drumFrontTopic = "/excavation/drum_front_cmd";
    [SerializeField] private string drumRearTopic = "/excavation/drum_rear_cmd";
    [SerializeField] private string armFrontTopic = "/excavation/arm_front_cmd";
    [SerializeField] private string armRearTopic = "/excavation/arm_rear_cmd";

    private IPEXController controller;
    private ISubscription<geometry_msgs.msg.Twist> cmdVelSub;
    private ISubscription<std_msgs.msg.Float32> drumFrontSub, drumRearSub, armFrontSub, armRearSub;

    protected override void Awake()
    {
        base.Awake();
        controller = GetComponent<IPEXController>();
    }

    protected override void OnRosReady()
    {
        cmdVelSub = Subscribe<geometry_msgs.msg.Twist>(cmdVelTopic, m => controller.SetCmdVel((float)m.Linear.X, (float)m.Angular.Z), RosQos.Reliable());
        drumFrontSub = Subscribe<std_msgs.msg.Float32>(drumFrontTopic, m => controller.SetDrumCommands(m.Data, float.NaN));
        drumRearSub = Subscribe<std_msgs.msg.Float32>(drumRearTopic, m => controller.SetDrumCommands(float.NaN, m.Data));
        armFrontSub = Subscribe<std_msgs.msg.Float32>(armFrontTopic, m => controller.SetArmCommands(m.Data, float.NaN));
        armRearSub = Subscribe<std_msgs.msg.Float32>(armRearTopic, m => controller.SetArmCommands(float.NaN, m.Data));
    }
}