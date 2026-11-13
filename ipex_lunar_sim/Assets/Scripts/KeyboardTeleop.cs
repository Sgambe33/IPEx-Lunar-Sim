using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Allows to control the rover using the keyboard. For debug.
/// </summary>
[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(IPEXController))]
public class KeyboardTeleop : MonoBehaviour
{
    [SerializeField] private float linearSpeed = 0.30f;
    [SerializeField] private float angularSpeed = 0.8f;
    [SerializeField] private float armSpeed = 17.25f;

    private IPEXController controller;

    void Awake()
    {
        controller = GetComponent<IPEXController>();
    }

    void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null || !controller.PowerAvailable) return;

        float move = Axis(keyboard.wKey.isPressed, keyboard.sKey.isPressed);
        float turn = Axis(keyboard.aKey.isPressed, keyboard.dKey.isPressed);
        if (move != 0f || turn != 0f) controller.SetManualTwist(move * linearSpeed, turn * angularSpeed);

        float drum = Axis(keyboard.spaceKey.isPressed, keyboard.xKey.isPressed);
        if (drum != 0f) controller.SetManualDrums(drum);

        float arm = Axis(keyboard.iKey.isPressed, keyboard.kKey.isPressed);
        if (arm != 0f)
        {
            float step = arm * armSpeed * Time.deltaTime;
            controller.SetArmCommands(controller.ArmTargetFront + step, controller.ArmTargetRear + step);
        }
    }

    static float Axis(bool positive, bool negative) => (positive ? 1f : 0f) - (negative ? 1f : 0f);
}
