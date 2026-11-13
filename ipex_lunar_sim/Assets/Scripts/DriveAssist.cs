using UnityEngine;

/// <summary>
/// Tractions system that allows for better control of the rover
/// </summary>
[RequireComponent(typeof(IPEXController))]
public class DriveAssist : MonoBehaviour
{
    [SerializeField] private float maxLinearSpeed = 0.30f;
    [SerializeField] private float maxAngularSpeed = 0.8f;
    [SerializeField] private float linearSpeedCoeff = 3f;
    [SerializeField] private float angularSpeedCoeff = 5f;
    [SerializeField] private float angularSpeedIntegralCoeff = 4f;
    [SerializeField] private float maxAngularSpeedIntegral = 0.6f;
    [SerializeField] private float sideSlipDamping = 6f;
    [SerializeField] private float maxLinearAccel = 1.0f;
    [SerializeField] private float maxAngularAccel = 3.0f;
    
    private float angularSpeedErrorIntegral;
    private IPEXController controller;
    private ArticulationBody baseBody;
    private ArticulationBody[] allBodies;

    void Awake()
    {
        controller = GetComponent<IPEXController>();
        baseBody = controller != null ? controller.BaseBody : null;
        if (baseBody == null)
        {
            Debug.LogError("DriveAssist: needs an IPEXController.", this);
            enabled = false;
            return;
        }
        allBodies = baseBody.GetComponentsInChildren<ArticulationBody>();
    }

    void FixedUpdate()
    {
        if (baseBody == null) return;

        Vector3 fwd = Vector3.ProjectOnPlane(baseBody.transform.forward, Vector3.up);
        fwd.Normalize();
        Vector3 right = Vector3.Cross(Vector3.up, fwd);

        if (!ComputeMassAndYawInertia(out float mass, out float iyaw)) return;

        Vector3 vel = baseBody.linearVelocity;
        float vFwd = Vector3.Dot(vel, fwd);
        float vLat = Vector3.Dot(vel, right);

        float w = baseBody.angularVelocity.y;
        float desiredV = Mathf.Clamp(controller.DesiredLinear, -maxLinearSpeed, maxLinearSpeed);
        float desiredW = -Mathf.Clamp(controller.DesiredAngular, -maxAngularSpeed, maxAngularSpeed);

        float aFwd = Mathf.Clamp(linearSpeedCoeff * (desiredV - vFwd), -maxLinearAccel, maxLinearAccel);
        float aLat = Mathf.Clamp(-sideSlipDamping * vLat, -maxLinearAccel, maxLinearAccel);
        float yawError = desiredW - w;

        if (Mathf.Abs(desiredW) < 1e-3f)
        {
            angularSpeedErrorIntegral = 0f;
        }
        else
        {
            angularSpeedErrorIntegral = Mathf.Clamp(angularSpeedErrorIntegral + yawError * Time.fixedDeltaTime, -maxAngularSpeedIntegral, maxAngularSpeedIntegral);
        }

        float alpha = Mathf.Clamp(angularSpeedCoeff * yawError + angularSpeedIntegralCoeff * angularSpeedErrorIntegral, -maxAngularAccel, maxAngularAccel);

        baseBody.AddForce((fwd * aFwd + right * aLat) * mass);
        baseBody.AddTorque(Vector3.up * (alpha * Mathf.Max(iyaw, 0.05f)));
    }

    /// <summary>Calculates the rover mass and yaw intertia.</summary>
    bool ComputeMassAndYawInertia(out float mass, out float iyaw)
    {
        mass = 0f;
        iyaw = 0f;
        Vector3 centerOfMass = Vector3.zero;

        foreach (ArticulationBody ab in allBodies)
        {
            if (ab == null) continue;
            mass += ab.mass;
            centerOfMass += ab.worldCenterOfMass * ab.mass;
        }

        if (mass <= 0f) return false;
        centerOfMass /= mass;

        foreach (ArticulationBody ab in allBodies)
        {
            if (ab == null) continue;
            //distance factor
            Vector3 distance = ab.worldCenterOfMass - centerOfMass;
            distance.y = 0f;
            iyaw += ab.mass * distance.sqrMagnitude;
            //shape factor
            Quaternion r = ab.transform.rotation * ab.inertiaTensorRotation;
            Vector3 it = ab.inertiaTensor;
            Vector3 ax = Quaternion.Inverse(r) * Vector3.up;
            iyaw += it.x * ax.x * ax.x + it.y * ax.y * ax.y + it.z * ax.z * ax.z;
        }

        return true;
    }
}