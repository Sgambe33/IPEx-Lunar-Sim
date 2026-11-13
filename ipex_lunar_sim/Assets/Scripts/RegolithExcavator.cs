using UnityEngine;
using ROS2;
using UnityEngine.Serialization;

/// <summary>
/// Handles implements commands.
/// Publishes /rover/excavator which contains excavator status.
/// </summary>
public class RegolithExcavator : RosBehaviour
{
    [System.Serializable]
    public class Drum
    {
        public string name = "front";
        [System.NonSerialized] public ArticulationBody body;
        public Transform center;
        public float side = 1f; //+1 if in front, -1 if in rear

        [HideInInspector] public float loadKg;
        [HideInInspector] public float baseMass;
        [HideInInspector] public float lastMassLoad;
        [HideInInspector] public bool digging, dumping;
    }

    [SerializeField] private IPEXController controller;
    [SerializeField] private RegolithDeposit deposit;
    [SerializeField] private Hopper hopper;
    [SerializeField] private Drum front = new Drum { name = "front", side = 1f };
    [SerializeField] private Drum rear = new Drum { name = "rear", side = -1f };

    [Header("Drums")]
    [SerializeField] private float drumRadius = 0.14f;
    [SerializeField] private float capacityKgPerDrum = 15f;
    [SerializeField] private float maxDigRateKgPerSec = 1.5f;
    [SerializeField] private float minDistanceToStartDig = 0.01f;
    [SerializeField] private float minSpinRadPerSec = 0.2f;
    [SerializeField] private float commandThreshold = 0.1f;
    [SerializeField] private float dumpRateKgPerSec = 3f;
    [SerializeField] private float dumpOffset = 0.12f;
    [SerializeField] private float publishHz = 5f;

    private Transform roverBase;
    private IPublisher<ipex_interfaces.msg.ExcavatorState> publisher;
    private ipex_interfaces.msg.ExcavatorState excavatorState;
    private float nextPublish;

    public float FrontLoadKg => front.loadKg;
    public float RearLoadKg => rear.loadKg;
    public float TotalLoadKg => front.loadKg + rear.loadKg;
    public float TotalCapacityKg => 2f * capacityKgPerDrum;
    public float TotalDugKg { get; private set; }


    public float TotalDumpedKg { get; private set; }
    public Hopper Hopper => hopper;
    public float CurrentDigRateKgPerSec { get; private set; }
    public bool IsDigging => front.digging || rear.digging;
    public bool IsDumping => front.dumping || rear.dumping;


    protected override void Awake()
    {
        base.Awake();
        if (controller == null || deposit == null || controller.BaseBody == null)
        {
            Debug.LogError("RegolithExcavator: assign controller and deposit.", this);
            enabled = false;
            return;
        }

        front.body = controller.DrumFront;
        rear.body = controller.DrumRear;
        roverBase = controller.BaseBody.transform;

        foreach (var drum in new[] { front, rear })
        {
            if (drum.body != null) drum.baseMass = drum.body.mass;
        }
    }

    protected override void OnRosReady()
    {
        excavatorState = new ipex_interfaces.msg.ExcavatorState();
        publisher = Advertise<ipex_interfaces.msg.ExcavatorState>("/rover/excavator", RosQos.Reliable());
    }

    void FixedUpdate()
    {
        if (deposit == null || !deposit.IsReady || controller == null) return;
        float dt = Time.fixedDeltaTime;
        float dug = 0f;

        foreach (Drum drum in new[] { front, rear })
        {
            float cmd = drum == front ? controller.DrumCommandFront : controller.DrumCommandRear;
            drum.digging = drum.dumping = false;
            if (drum.body == null) continue;

            bool spinning = drum.body.jointVelocity.dofCount > 0 && Mathf.Abs(drum.body.jointVelocity[0]) > minSpinRadPerSec;
            if (!spinning)
            {
                UpdateMass(drum);
                continue;
            }

            Vector3 drumCenter = drum.center != null ? drum.center.position : drum.body.transform.position;
            Vector3 fwd = Vector3.ProjectOnPlane(roverBase.forward, Vector3.up);

            if (cmd > commandThreshold && drum.loadKg < capacityKgPerDrum)
            {
                dug += Dig(drum, drumCenter, cmd, dt);
            }
            else if (cmd < -commandThreshold && drum.loadKg > 0f)
            {
                Dump(drum, drumCenter, fwd, -cmd, dt);
            }

            UpdateMass(drum);
        }

        CurrentDigRateKgPerSec = dug / dt;

        if (publisher != null && Time.time >= nextPublish)
        {
            nextPublish = Time.time + 1f / publishHz;
            excavatorState.Front_load_kg = FrontLoadKg;
            excavatorState.Rear_load_kg = RearLoadKg;
            excavatorState.Load_kg = TotalLoadKg;
            excavatorState.Capacity_kg = TotalCapacityKg;
            excavatorState.Dug_kg = TotalDugKg;
            excavatorState.Dumped_kg = TotalDumpedKg;
            publisher.Publish(excavatorState);
        }
    }

    private float Dig(Drum drum, Vector3 drumCenter, float cmd, float dt)
    {
        float ground = deposit.GroundHeightAt(drumCenter.x, drumCenter.z);
        if (float.IsNaN(ground) || drumCenter.y - drumRadius > ground + minDistanceToStartDig) return 0f; // not touching

        float wantedKg = Mathf.Min(maxDigRateKgPerSec * Mathf.Clamp01(cmd) * dt, capacityKgPerDrum - drum.loadKg);
        float takenKg = deposit.Take(drumCenter.x, drumCenter.z, wantedKg);
        drum.loadKg += takenKg;
        TotalDugKg += takenKg;
        drum.digging = takenKg > 0f;
        return takenKg;
    }

    private void Dump(Drum drum, Vector3 drumCenter, Vector3 fwd, float cmd, float dt)
    {
        float kg = Mathf.Min(drum.loadKg, dumpRateKgPerSec * Mathf.Clamp01(cmd) * dt);
        if (kg <= 0f) return;
        Vector3 point = drumCenter + fwd.normalized * (drum.side * (drumRadius * 0.5f + dumpOffset));
        float unloadedKg;
        
        if (hopper != null && hopper.Contains(point))
        {
            unloadedKg = hopper.Receive(kg);
        }
        else
        {
            unloadedKg = deposit.Put(point.x, point.z, kg);
        }

        drum.loadKg -= unloadedKg;
        TotalDumpedKg += unloadedKg;
        drum.dumping = unloadedKg > 0f;
    }
    
    private void UpdateMass(Drum drum)
    {
        if (drum.body == null) return;
        if (Mathf.Abs(drum.loadKg - drum.lastMassLoad) < 0.05f) return;
        drum.body.mass = drum.baseMass + drum.loadKg;
        drum.lastMassLoad = drum.loadKg;
    }
}