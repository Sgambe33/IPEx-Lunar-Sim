using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// ROS independent controller. Consumes commands received from ROS thread every frame and applies them if possible.
/// </summary>
public class IPEXController : MonoBehaviour
{
    [Serializable]
    private class Arm
    {
        public ArticulationBody body;
        public float jointDirection = 1f;
        [NonSerialized] public float targetAngle; // commanded angle
        [NonSerialized] public float driveTarget; // what is being actually applied to the arm

    }

    [Serializable]
    private class Drum
    {
        public ArticulationBody body;
        public float digSign = 1f;
        [NonSerialized] public float rosCmd;
    }

    public const float ArmMin = -10f, ArmMax = 55f;
    private const float DriveDamping = 10f;
    private const float BrakeDamping = 500f;
    private const float BrakeTorque = 25f;
    private const float BrakeStiffness = 5000f;
    private const float ArmStiffness = 1000f;
    private const float ArmDamping = 500f;
    private const float WheelJointFriction = 0.5f;

    [SerializeField] private ArticulationBody baseBody;

    [Header("Mobility")]
    [SerializeField] private ArticulationBody[] leftWheels = Array.Empty<ArticulationBody>();
    [SerializeField] private ArticulationBody[] rightWheels = Array.Empty<ArticulationBody>();
    [SerializeField] private float maxWheelVelocity = 115f;
    [SerializeField] private float wheelMaxTorque = 7.5f;
    [SerializeField] private float wheelRadius = 0.16f;
    [SerializeField] private float trackWidth = 0.406f;
    [SerializeField] private float cmdVelTimeout = 0.5f;

    [Header("Implements")]
    [SerializeField] private Arm armFront = new Arm { jointDirection = -1f };
    [SerializeField] private Arm armRear = new Arm { jointDirection = 1f };
    [SerializeField] private float armVelocity = 17.25f;
    [SerializeField] private float armMaxTorque = 25.0f;
    [SerializeField, Range(ArmMin, ArmMax)] private float initialArmAngle;
    [SerializeField] private Drum drumFront = new Drum { digSign = 1f };
    [SerializeField] private Drum drumRear = new Drum { digSign = -1f };
    [SerializeField] private float drumDigVelocity = 40f;
    [SerializeField] private float drumMaxTorque = 7.5f;
    [SerializeField] private float drumCmdTimeout = 1.0f;


    // lock required for exchanging messages between ROS2-for-Unity thread and Unity main thread
    private readonly object cmdLock = new object();
    private bool pendingTwist, pendingDrums, pendingArms;
    private float pendingLinear, pendingAngular;
    private float pendingDrumFront, pendingDrumRear;
    private float pendingArmFront, pendingArmRear;

    // actual commands touched only by unity main thread
    private float cmdVelLinear, cmdVelAngular;
    private float lastCmdVelTime;
    private bool cmdVelActive;
    private bool braking;
    private float lastDrumCmdTime = -1000f;

    private bool manualTwist, manualDrums;
    private float manualLinear, manualAngular, manualDrumCmd;
    private Arm[] arms;

    public float DrumCommandFront { get; private set; }
    public float DrumCommandRear { get; private set; }
    public float ArmTargetFront => armFront.targetAngle;
    public float ArmTargetRear => armRear.targetAngle;
    public bool PowerAvailable { get; set; } = true;
    public float DesiredLinear { get; private set; }
    public float DesiredAngular { get; private set; }
    public ArticulationBody BaseBody => baseBody;
    public ArticulationBody DrumFront => drumFront.body;
    public ArticulationBody DrumRear => drumRear.body;
    public float TrackWidth => trackWidth;

    public IEnumerable<ArticulationBody> Wheels
    {
        get
        {
            foreach (ArticulationBody w in leftWheels) yield return w;
            foreach (ArticulationBody w in rightWheels) yield return w;
        }
    }

    public bool IsArmMoving(bool front)
    {
        Arm arm = front ? armFront : armRear;
        bool hasNotReachedTarget = !Mathf.Approximately(arm.driveTarget, arm.targetAngle);
        return hasNotReachedTarget || (arm.body != null && arm.body.dofCount > 0 && Mathf.Abs(arm.body.jointVelocity[0]) > 0.02f);
    }

    public void SetCmdVel(float linearX, float angularZ)
    {
        lock (cmdLock)
        {
            pendingLinear = linearX;
            pendingAngular = angularZ;
            pendingTwist = true;
        }
    }

    public void SetArmCommands(float front, float rear)
    {
        lock (cmdLock)
        {
            if (!pendingArms)
            {
                pendingArmFront = armFront.targetAngle;
                pendingArmRear = armRear.targetAngle;
            }

            if (!float.IsNaN(front)) pendingArmFront = Mathf.Clamp(front, ArmMin, ArmMax);
            if (!float.IsNaN(rear)) pendingArmRear = Mathf.Clamp(rear, ArmMin, ArmMax);
            pendingArms = true;
        }
    }

    public void SetDrumCommands(float front, float rear)
    {
        lock (cmdLock)
        {
            if (!pendingDrums)
            {
                pendingDrumFront = drumFront.rosCmd;
                pendingDrumRear = drumRear.rosCmd;
            }

            if (!float.IsNaN(front)) pendingDrumFront = Mathf.Clamp(front, -1f, 1f);
            if (!float.IsNaN(rear)) pendingDrumRear = Mathf.Clamp(rear, -1f, 1f);
            pendingDrums = true;
        }
    }

    public void SetManualTwist(float linear, float angular)
    {
        manualTwist = true;
        manualLinear = linear;
        manualAngular = angular;
    }

    public void SetManualDrums(float cmd)
    {
        manualDrums = true;
        manualDrumCmd = Mathf.Clamp(cmd, -1f, 1f);
    }

    void SetDrums(float front, float rear)
    {
        DrumCommandFront = front;
        DrumCommandRear = rear;
        SetVelocity(drumFront.body, Mathf.Sign(drumFront.digSign) * drumDigVelocity * front, DriveDamping, drumMaxTorque);
        SetVelocity(drumRear.body, Mathf.Sign(drumRear.digSign) * drumDigVelocity * rear, DriveDamping, drumMaxTorque);
    }

    void RampArms(float dt)
    {
        foreach (Arm arm in arms)
        {
            if (Mathf.Approximately(arm.driveTarget, arm.targetAngle)) continue;
            arm.driveTarget = Mathf.MoveTowards(arm.driveTarget, arm.targetAngle, armVelocity * dt);
            ApplyArmDrive(arm);
        }
    }

    void ApplyArmDrive(Arm arm)
    {
        if (arm.body == null) return;
        ArticulationDrive drive = arm.body.xDrive;
        drive.lowerLimit = arm.jointDirection >= 0f ? ArmMin : -ArmMax;
        drive.upperLimit = arm.jointDirection >= 0f ? ArmMax : -ArmMin;
        drive.stiffness = ArmStiffness;
        drive.damping = ArmDamping;
        drive.forceLimit = armMaxTorque;
        drive.target = Mathf.Sign(arm.jointDirection) * Mathf.Clamp(arm.driveTarget, ArmMin, ArmMax);
        arm.body.xDrive = drive;
    }

    static void SetVelocity(ArticulationBody ab, float targetVelocityDegPerSec, float damping, float forceLimit)
    {
        if (ab == null) return;
        ArticulationDrive drive = ab.xDrive;
        drive.stiffness = 0f;
        drive.damping = damping;
        drive.forceLimit = forceLimit;
        drive.targetVelocity = targetVelocityDegPerSec;
        ab.xDrive = drive;
    }

    void Awake()
    {
        if (baseBody == null)
        {
            Debug.LogError("IPEXController: assign baseBody.", this);
            enabled = false;
        }
    }

    void Start()
    {
        foreach (ArticulationBody w in Wheels)
        {
            if (w == null) continue;
            w.jointFriction = WheelJointFriction;
            SetVelocity(w, 0f, DriveDamping, wheelMaxTorque);
        }

        SetVelocity(drumFront.body, 0f, DriveDamping, drumMaxTorque);
        SetVelocity(drumRear.body, 0f, DriveDamping, drumMaxTorque);

        arms = new[] { armFront, armRear };
        foreach (Arm arm in arms)
        {
            arm.targetAngle = arm.driveTarget = Mathf.Clamp(initialArmAngle, ArmMin, ArmMax);
            ApplyArmDrive(arm);
        }
    }

    void Update()
    {
        ConsumeRosCommands();

        if (!PowerAvailable)
        {
            cmdVelActive = false;
            Brake();
            SetDrums(0f, 0f);
        }
        else
        {
            bool cmdVelNew = cmdVelActive && (cmdVelTimeout <= 0f || Time.time - lastCmdVelTime <= cmdVelTimeout);
            if (manualTwist)
            {
                cmdVelActive = false;
                Drive(manualLinear, manualAngular);
            }
            else if (cmdVelNew && (Mathf.Abs(cmdVelLinear) > 1e-3f || Mathf.Abs(cmdVelAngular) > 1e-3f))
            {
                Drive(cmdVelLinear, cmdVelAngular);
            }
            else
            {
                cmdVelActive = false;
                Brake();
            }

            if (manualDrums)
            {
                SetDrums(manualDrumCmd, manualDrumCmd);
            }
            else
            {
                bool drumCmdFresh = drumCmdTimeout <= 0f || Time.time - lastDrumCmdTime <= drumCmdTimeout;
                SetDrums(drumCmdFresh ? drumFront.rosCmd : 0f, drumCmdFresh ? drumRear.rosCmd : 0f);
            }
        }

        manualTwist = manualDrums = false;
    }

    void FixedUpdate()
    {
        RampArms(Time.fixedDeltaTime);
    }

    void ConsumeRosCommands()
    {
        lock (cmdLock)
        {
            if (pendingTwist)
            {
                pendingTwist = false;
                cmdVelLinear = pendingLinear;
                cmdVelAngular = pendingAngular;
                lastCmdVelTime = Time.time;
                cmdVelActive = true;
            }

            if (pendingDrums)
            {
                pendingDrums = false;
                drumFront.rosCmd = pendingDrumFront;
                drumRear.rosCmd = pendingDrumRear;
                lastDrumCmdTime = Time.time;
            }

            if (pendingArms)
            {
                pendingArms = false;
                armFront.targetAngle = pendingArmFront;
                armRear.targetAngle = pendingArmRear;
            }
        }
    }

    /// <summary>
    /// Differential drive  https://matwbn.icm.edu.pl/ksiazki/amc/amc14/amc1445.pdf 2,9,10,11
    /// </summary>
    void Drive(float linear, float angular)
    {
        braking = false;
        float halfTrack = trackWidth * 0.5f;
        float linearToDegPerSec = Mathf.Rad2Deg / wheelRadius;

        //v_L = v_x - w * (b/2)
        float leftSpeed = (linear - angular * halfTrack) * linearToDegPerSec;

        //v_R = v_x + w * (b/2)
        float rightSpeed = (linear + angular * halfTrack) * linearToDegPerSec;

        float maxMag = Mathf.Max(Mathf.Abs(leftSpeed), Mathf.Abs(rightSpeed));
        float scale = maxMag > maxWheelVelocity ? maxWheelVelocity / maxMag : 1f;

        DesiredLinear = linear * scale;
        DesiredAngular = angular * scale;
        foreach (ArticulationBody w in leftWheels) SetVelocity(w, leftSpeed * scale, DriveDamping, wheelMaxTorque);
        foreach (ArticulationBody w in rightWheels) SetVelocity(w, rightSpeed * scale, DriveDamping, wheelMaxTorque);
    }

    void Brake()
    {
        DesiredLinear = DesiredAngular = 0f;
        foreach (ArticulationBody w in Wheels)
        {
            if (w == null) continue;
            ArticulationDrive drive = w.xDrive;
            if (!braking) drive.target = w.jointPosition[0] * Mathf.Rad2Deg;
            drive.stiffness = BrakeStiffness;
            drive.damping = BrakeDamping;
            drive.forceLimit = BrakeTorque;
            drive.targetVelocity = 0f;
            w.xDrive = drive;
        }
        braking = true;
    }
}