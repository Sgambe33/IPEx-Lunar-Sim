using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// Handles UI for displaying rover/mission status.
/// </summary>
public class SimulationHud : MonoBehaviour
{
    [SerializeField] private RegolithExcavator excavator;
    [SerializeField] private RoverBattery battery;
    [SerializeField] private IPEXController controller;

    [Header("UI")]
    [SerializeField] private GameObject panel;
    [SerializeField] private Text fpsText;
    [SerializeField] private Text titleText;
    [SerializeField] private Text loadText;
    [SerializeField] private Text digText;
    [SerializeField] private Text implementsText;
    [SerializeField] private Text batteryText;
    [SerializeField] private Text hopperText;
    [SerializeField] private float refreshRate = 0.1f;

    private int frames;
    private float t0;
    private float period = 0.25f;
    private float nextRefresh;

    void Awake()
    {
        if (panel == null) Debug.LogWarning("SimulationHud: no panel assigned.", this);
    }

    void Update()
    {
        var keyboard = UnityEngine.InputSystem.Keyboard.current;
        if (keyboard != null && keyboard.f1Key.wasPressedThisFrame && panel != null) panel.SetActive(!panel.activeSelf);

        frames++;
        float dt = Time.unscaledTime - t0;
        if (dt < period) return;
        if (fpsText) fpsText.text = $"FPS: {Mathf.RoundToInt(frames / dt)}";
        frames = 0;
        t0 = Time.unscaledTime;

        if (panel == null || !panel.activeInHierarchy || Time.unscaledTime < nextRefresh) return;
        nextRefresh = Time.unscaledTime + refreshRate;
        Refresh();
    }

    private void Refresh()
    {
        Set(titleText, $"IPEx mission       t = {Time.timeAsDouble:F0} s");
        if (excavator != null)
        {
            Set(loadText, $"Load  front {excavator.FrontLoadKg,5:F1} kg   rear {excavator.RearLoadKg,5:F1} kg   / {excavator.TotalCapacityKg:F0} kg");
            Set(digText, $"Dug {excavator.TotalDugKg,7:F1} kg   Dumped {excavator.TotalDumpedKg,7:F1} kg   " + (excavator.IsDigging ? "DIGGING" : excavator.IsDumping ? "DUMPING" : ""));
        }
        if (controller != null)
        {
            Set(implementsText, $"Drums F {controller.DrumCommandFront:+0.0;-0.0} R {controller.DrumCommandRear:+0.0;-0.0}  Arms F {controller.ArmTargetFront:F0} R {controller.ArmTargetRear:F0} deg");
        }
        if (battery != null)
        {
            Set(batteryText, $"Battery {battery.Soc * 100f:F1}% ({battery.EnergyWh:F1}/{battery.CapacityWh:F0} Wh)  out {battery.PowerOutW:F0} W  in {battery.PowerInW:F0} W" + (battery.IsDocked ? " DOCKED" : ""));
        }
        var hopper = excavator != null ? excavator.Hopper : null;
        if (hopper != null)
        {
            Set(hopperText, $"Hopper {hopper.DeliveredKg,6:F1} kg");
        }
    }

    private static void Set(Text t, string s)
    {
        if (t != null && t.text != s) t.text = s;
    }
}
