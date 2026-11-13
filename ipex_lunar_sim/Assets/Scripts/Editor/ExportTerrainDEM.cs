using System;
using System.IO;
using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using UnityEditor;
using UnityEngine;

/// <summary>
/// Editor tool used to export terrain heights to then be used inside ROS package for decision making.
/// </summary>
public static class ExportTerrainDEM
{
    const float TerrainHalfSize = 50f;
    const float DEMResolution = 1f;

    [MenuItem("IPEx Mission/Export Terrain DEM")]
    public static void Export()
    {
        var terrainObject = Selection.activeGameObject;
        var terrainCollider = terrainObject != null ? terrainObject.GetComponentInChildren<MeshCollider>() : null;
        if (terrainCollider == null)
        {
            Debug.LogError("ExportTerrainDEM: select the terrain.");
            return;
        }

        int n = Mathf.RoundToInt(2f * TerrainHalfSize / DEMResolution) + 1;
        float top = terrainCollider.bounds.max.y + 10f;
        float depth = terrainCollider.bounds.size.y + 20f;

        JArray dataArray = new JArray();
        int hits = 0;

        for (int row = 0; row < n; row++)
        {
            float y = -TerrainHalfSize + row * DEMResolution;
            for (int col = 0; col < n; col++)
            {
                float x = -TerrainHalfSize + col * DEMResolution;
                Ray ray = new Ray(new Vector3(-y, top, x), Vector3.down);
                
                if (terrainCollider.Raycast(ray, out RaycastHit hit, depth))
                {
                    dataArray.Add(Math.Round(hit.point.y, 4));
                    hits++;
                }
                else
                {
                    dataArray.Add(JValue.CreateNull());
                }
            }
        }

        JObject output = new JObject()
        {
            ["frame"] = "map",
            ["rows"] = n,
            ["cols"] = n,
            ["origin_x"] = -TerrainHalfSize,
            ["origin_y"] = -TerrainHalfSize,
            ["resolution"] = DEMResolution,
            ["source"] = terrainObject.name,
            ["data"] = dataArray
        };

        string path = Path.GetFullPath(Path.Combine(Application.dataPath, "ExportedData", "terrain_heights.json"));
        File.WriteAllText(path, output.ToString(Formatting.None));
    }
}