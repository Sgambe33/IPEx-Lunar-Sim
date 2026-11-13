using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Allows to switch cameras while in Play mode without interfearing with ROS.
/// </summary>
public class CameraSwitcher : MonoBehaviour
{
    [SerializeField] private Camera cameraFL, cameraFR, cameraLS, cameraRS, cameraRL, cameraRR;

    private Camera[] cameras;

    void Start()
    {
        cameras = new[] { cameraFL, cameraFR, cameraLS, cameraRS, cameraRL, cameraRR };
        Show(cameraFL);
    }

    void Update()
    {
        Keyboard keyboard = Keyboard.current;
        if (keyboard == null) return;

        if (keyboard.numpad7Key.wasPressedThisFrame) Show(cameraFL);
        if (keyboard.numpad9Key.wasPressedThisFrame) Show(cameraFR);
        if (keyboard.numpad4Key.wasPressedThisFrame) Show(cameraLS);
        if (keyboard.numpad6Key.wasPressedThisFrame) Show(cameraRS);
        if (keyboard.numpad1Key.wasPressedThisFrame) Show(cameraRL);
        if (keyboard.numpad3Key.wasPressedThisFrame) Show(cameraRR);
    }

    private void Show(Camera target)
    {
        foreach (Camera cam in cameras)
        {
            if (cam != null) cam.enabled = cam == target;
        }
    }
}
