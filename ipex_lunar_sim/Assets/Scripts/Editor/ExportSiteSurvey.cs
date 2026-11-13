using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text.RegularExpressions;
using UnityEditor;
using UnityEngine;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;

/// <summary>
/// Exports site buildings and fiducials tag positions
/// </summary>
public static class ExportSiteSurvey
{
    [MenuItem("IPEx Mission/Export Site Survey")]
    public static void Export()
    {
        Hopper hopper = Object.FindFirstObjectByType<Hopper>();
        RoverBattery battery = Object.FindFirstObjectByType<RoverBattery>();
        RosSimulationPublisher ros = Object.FindFirstObjectByType<RosSimulationPublisher>();
        
        var lander = ros != null ? new SerializedObject(ros).FindProperty("lander").objectReferenceValue as Transform : null;
        var charger = battery != null ? new SerializedObject(battery).FindProperty("charger").objectReferenceValue as Transform : null;
        
        if (hopper == null || lander == null || charger == null)
        {
            Debug.LogError("ExportSiteSurvey: needs Hopper, RosSimulationPublisher and RoverBattery in the scene.");
            return;
        }
        float radius = new SerializedObject(battery).FindProperty("chargingRadius").floatValue;


        Transform h = hopper.transform;
        Vector3 f = h.forward;

        JObject output = new JObject();
        output["hopper"] = new JObject()
        {
            ["x"] = F(h.position.z),
            ["y"] = F(-h.position.x),
            ["yaw"] = F(Mathf.Atan2(-f.x, f.z)),
            ["length"] = F(hopper.Length),
            ["width"] = F(hopper.Width),
            ["height"] = F(hopper.Height),
            ["wall"] = F(hopper.WallThickness)
        };

        output["charger"] = new JObject()
        {
            ["x"] = F(charger.position.z),
            ["y"] = F(-charger.position.x),
            ["radius"] = F(radius)
        };


        output["lander"] = new JObject()
        {
            ["x"] = F(lander.position.z),
            ["y"] = F(-lander.position.x)
        };

        JArray tagsArray = new JArray();
        output["fiducials"] = new JObject()
        {
            ["tags"] = tagsArray
        };

        Regex rx = new Regex(@"_ID_(\d+)$");
        var sources = new List<(MeshFilter mf, bool unityQuad)>();
        foreach (MeshFilter mf in lander.GetComponentsInChildren<MeshFilter>(true)) sources.Add((mf, false));
        foreach (MeshFilter mf in hopper.GetComponentsInChildren<MeshFilter>(true)) sources.Add((mf, true));
        foreach (var (mf, unityQuad) in sources)
        {
            Match m = rx.Match(mf.gameObject.name);
            if (!m.Success || mf.sharedMesh == null || !mf.gameObject.activeInHierarchy) continue;

            Vector3[] world = unityQuad ? QuadCorners(mf.transform) : TagCorners(mf);

            JArray cornersArray = new JArray();
            for (int i = 0; i < 4; i++)
            {
                Vector3 c = world[i];
                cornersArray.Add(new JArray(F(c.z), F(-c.x), F(c.y)));
            }

            JObject tagObj = new JObject()
            {
                ["id"] = int.Parse(m.Groups[1].Value),
                ["size"] = F(Vector3.Distance(world[0], world[1])),
                ["corners"] = cornersArray
            };

            tagsArray.Add(tagObj);
        }
        string path = Path.GetFullPath(Path.Combine(Application.dataPath, "ExportedData", "site_survey.json"));
        File.WriteAllText(path, output.ToString(Formatting.Indented));
    }

    static double F(float v) => System.Math.Round(v, 5);

    static Vector3[] QuadCorners(Transform q)
    {
        return new[]
        {
            q.TransformPoint(new Vector3(-0.5f, 0.5f, 0f)), q.TransformPoint(new Vector3(0.5f, 0.5f, 0f)),
            q.TransformPoint(new Vector3(0.5f, -0.5f, 0f)), q.TransformPoint(new Vector3(-0.5f, -0.5f, 0f)),
        };
    }

    static Vector3[] TagCorners(MeshFilter mf)
    {
        Vector3[] world = new Vector3[4];
        Mesh mesh = mf.sharedMesh;
        Vector2[] targets = new[] { new Vector2(1, 0), new Vector2(0, 0), new Vector2(0, 1), new Vector2(1, 1) };
        Vector3[] local = new Vector3[4];
        if (mesh.isReadable && mesh.uv != null && mesh.uv.Length == mesh.vertexCount && mesh.vertexCount >= 4)
        {
            Vector3[] vertices = mesh.vertices;
            Vector2[] uv = mesh.uv;
            for (int k = 0; k < 4; k++)
            {
                int best = 0;
                for (int i = 1; i < uv.Length; i++)
                {
                    if ((uv[i] - targets[k]).sqrMagnitude < (uv[best] - targets[k]).sqrMagnitude) best = i;
                }
                local[k] = vertices[best];
            }
        }
        else
        {
            Vector3 extents = mesh.bounds.extents;
            float h = Mathf.Max(extents.x, extents.y);
            local[0] = new Vector3(-h, -h, 0); local[1] = new Vector3(h, -h, 0);
            local[2] = new Vector3(h, h, 0); local[3] = new Vector3(-h, h, 0);
        }
        for (int k = 0; k < 4; k++) world[k] = mf.transform.TransformPoint(local[k]);
        return world;
    }
}
