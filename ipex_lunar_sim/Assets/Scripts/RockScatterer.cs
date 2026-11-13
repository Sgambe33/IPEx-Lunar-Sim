using System.Collections.Generic;
using UnityEngine;

public class RockScatterer : MonoBehaviour
{
    [SerializeField] private List<GameObject> rockPrefabs;
    [SerializeField] private int count = 2000;
    [SerializeField] private int seed;

    [SerializeField] private Transform areaCenter;
    [SerializeField] private float areaSize = 30f;
    [SerializeField] private float minScale = 0.2f;
    [SerializeField] private float maxScale = 4f;
    [SerializeField, Range(1f, 5f)] private float sizeCoeff = 3f;

    [SerializeField] private int clusterCount = 10;
    [SerializeField] private float clusterRadius = 15f;
    [SerializeField, Range(0f, 1f)] private float clusterChance = 0.7f;

    [SerializeField, HideInInspector] private GameObject rocksRoot;

    [ContextMenu("Clear Rocks")]
    public void ClearRocks()
    {
        if (rocksRoot != null)
        {
            DestroyImmediate(rocksRoot);
            rocksRoot = null;
        }
    }

    [ContextMenu("Generate Rocks")]
    public void GenerateRocks()
    {
        if (rockPrefabs == null || rockPrefabs.Count == 0) return;

        ClearRocks();
        if (seed != 0) Random.InitState(seed);

        Vector3 center = areaCenter != null ? areaCenter.position : transform.position;
        float half = areaSize * 0.5f;

        rocksRoot = new GameObject(name + " Rocks");

        Vector3[] clusters = new Vector3[clusterCount];
        for (int i = 0; i < clusterCount; i++)
        {
            clusters[i] = center + new Vector3(Random.Range(-half, half), 0, Random.Range(-half, half));
        }

        for (int i = 0; i < count; i++)
        {
            Vector3 pos;
            if (clusterCount > 0 && Random.value < clusterChance)
            {
                Vector2 offset = Random.insideUnitCircle * clusterRadius;
                pos = clusters[Random.Range(0, clusterCount)] + new Vector3(offset.x, 0, offset.y);
            }
            else
            {
                pos = center + new Vector3(Random.Range(-half, half), 0, Random.Range(-half, half));
            }

            Vector3 rayStart = new Vector3(pos.x, center.y + 1000f, pos.z);
            if (!Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, 2000f))
                continue;

            if (!hit.collider.CompareTag("Terrain"))
                continue;

            float t = Mathf.Pow(Random.value, sizeCoeff);
            float scale = Mathf.Lerp(minScale, maxScale, t);

            GameObject prefab = rockPrefabs[Random.Range(0, rockPrefabs.Count)];

            Quaternion rot = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);

            GameObject rock = Instantiate(prefab, hit.point, rot, rocksRoot.transform);
            rock.transform.localScale = prefab.transform.localScale * scale;
        }
    }
}