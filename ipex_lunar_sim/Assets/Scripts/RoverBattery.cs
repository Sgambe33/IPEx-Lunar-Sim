using UnityEngine;
using ROS2;

/// <summary>
/// Simplified battery consumption model. 
/// Publishes /rover/battery with soc, energy_wh, capacity_wh, power_out_w, docked, charging, consumed_wh, charged_wh
/// </summary>
public class RoverBattery : RosBehaviour
{
    [SerializeField] private IPEXController controller;
    [SerializeField] private RegolithExcavator excavator;
    [SerializeField] private float publishHz = 5f;

    [Header("Battery")]
    [SerializeField] private float capacityWh = 283f;
    [SerializeField, Range(0f, 1f)] private float initialSoc = 1f;

    [Header("Power config")]
    [SerializeField] private float basePowerW = 86.8f;   // 10 + 6 cameras x 3 + 6 lights x 9.8
    [SerializeField] private float wheelsPowerW = 50f;
    [SerializeField] private float wheelsReferenceSpeed = 0.3f;
    [SerializeField] private float armMovingPowerW = 250f;
    [Header("Drums")]
    [SerializeField] private float digSpecificEnergyJPerKg = 60f;
    [SerializeField, Range(0.1f, 1f)] private float drumMotorEfficiency = 0.6f;

    [Header("Charging")]
    [SerializeField] private Transform charger;
    [SerializeField] private float chargingRadius = 1.6f;
    [SerializeField] private float maxDockSpeed = 0.05f;
    [SerializeField] private float chargingPowerW = 1000f;

    private ArticulationBody baseBody;
    private IPublisher<ipex_interfaces.msg.BatteryState> publisher;
    private ipex_interfaces.msg.BatteryState batteryState;
    private float nextPublish;

    public float CapacityWh => capacityWh;
    public float EnergyWh { get; private set; }
    public float Soc => capacityWh > 0f ? EnergyWh / capacityWh : 0f;
    public float PowerOutW { get; private set; }
    public float PowerInW { get; private set; }
    public bool IsCharging => PowerInW > 0f;
    public bool IsDocked { get; private set; }
    public bool IsDepleted => EnergyWh <= 0f;
    public float TotalConsumedWh { get; private set; }
    public float TotalChargedWh { get; private set; }

    protected override void Awake()
    {
        base.Awake();
        if (controller == null || controller.BaseBody == null)
        {
            Debug.LogError("RoverBattery: assign controller.", this);
            enabled = false;
            return;
        }
        baseBody = controller.BaseBody;
        EnergyWh = capacityWh * initialSoc;
    }

    protected override void OnRosReady()
    {
        batteryState = new ipex_interfaces.msg.BatteryState();
        publisher = Advertise<ipex_interfaces.msg.BatteryState>("/rover/battery", RosQos.Reliable());
    }

    void FixedUpdate()
    {
        float dt = Time.fixedDeltaTime;
        PowerOutW = ConsumptionW();

        Vector3 distanceToCharger = charger != null ? baseBody.transform.position - charger.position : Vector3.positiveInfinity;
        distanceToCharger.y = 0f;
        IsDocked = charger != null && distanceToCharger.magnitude <= chargingRadius && baseBody.linearVelocity.magnitude <= maxDockSpeed;
        PowerInW = IsDocked && Soc < 1f ? chargingPowerW : 0f;

        TotalConsumedWh += PowerOutW * dt / 3600f;
        TotalChargedWh += PowerInW * dt / 3600f;
        EnergyWh = Mathf.Clamp(EnergyWh + (PowerInW - PowerOutW) * dt / 3600f, 0f, capacityWh);
        controller.PowerAvailable = !IsDepleted;

        if (publisher != null && Time.time >= nextPublish)
        {
            nextPublish = Time.time + 1f / publishHz;
            batteryState.Soc = Soc;
            batteryState.Energy_wh = EnergyWh;
            batteryState.Capacity_wh = capacityWh;
            batteryState.Power_out_wh = PowerOutW;
            batteryState.Docked = IsDocked;
            batteryState.Charging = IsCharging;
            batteryState.Consumed_wh = TotalConsumedWh;
            batteryState.Charged_wh = TotalChargedWh;
            publisher.Publish(batteryState);
        }
    }

    private float ConsumptionW()
    {
        float p = basePowerW;

        // wheels: ground speed of the wheels = forward speed + turning speed at the half track
        //https://hades.mech.northwestern.edu/images/2/25/MR-v2.pdf pg.528 chp. 13.3.1.4
        Transform t = baseBody.transform;
        float v = Mathf.Abs(Vector3.Dot(baseBody.linearVelocity, t.forward));
        float w = Mathf.Abs(Vector3.Dot(baseBody.angularVelocity, t.up));
        float wheelSpeed = v + w * 0.5f * controller.TrackWidth;
        p += wheelsPowerW * (wheelSpeed / Mathf.Max(wheelsReferenceSpeed, 1e-3f));

        foreach (bool front in new[] { true, false })
        {
            if (controller.IsArmMoving(front))
            {
                p += armMovingPowerW;
            }
        }

        if (excavator != null)
        {   
            p += (excavator.CurrentDigRateKgPerSec * digSpecificEnergyJPerKg) / drumMotorEfficiency;
        }
        return p;
    }
}
