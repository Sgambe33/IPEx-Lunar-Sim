using UnityEngine;

/// <summary>
/// Handles the representation of regolith deposit as a grid.
/// </summary>
public class RegolithDeposit : MonoBehaviour
{
    [SerializeField] private MeshCollider terrain;
    [SerializeField] private float cellSize = 0.5f;
    [SerializeField] private float regolithDensity = 1500f; //kg/m3
    [SerializeField] private float minableDepth = 0.08f;

    private float originX, originZ;
    private int cellsX, cellsZ;
    private float[] actualKg;
    private float[] initialKg;
    private bool ready;

    public bool IsReady => ready;

    void Awake()
    {
        if (terrain == null)
        {
            Debug.LogError("RegolithDeposit: no terrain MeshCollider assigned.", this);
            enabled = false;
            return;
        }

        Bounds terrainBounds = terrain.bounds;
        originX = terrainBounds.min.x;
        originZ = terrainBounds.min.z;
        cellsX = Mathf.Max(1, Mathf.CeilToInt(terrainBounds.size.x / cellSize));
        cellsZ = Mathf.Max(1, Mathf.CeilToInt(terrainBounds.size.z / cellSize));

        float fullKg = regolithDensity * cellSize * cellSize * minableDepth;
        initialKg = new float[cellsX * cellsZ];
        System.Array.Fill(initialKg, fullKg);

        actualKg = (float[])initialKg.Clone();
        ready = true;
    }

    public float GroundHeightAt(float x, float z)
    {
        Bounds terrainBounds = terrain.bounds;
        Ray ray = new Ray(new Vector3(x, terrainBounds.max.y + 5f, z), Vector3.down);
        return terrain.Raycast(ray, out RaycastHit hit, terrainBounds.size.y + 10f) ? hit.point.y : float.NaN;
    }

    public float Take(float x, float z, float kg)
    {
        int i = CellIndex(x, z);
        if (i < 0 || kg <= 0f) return 0f;
        float taken = Mathf.Min(kg, actualKg[i]);
        actualKg[i] -= taken;
        return taken;
    }

    public float Put(float x, float z, float kg)
    {
        int i = CellIndex(x, z);
        if (i < 0 || kg <= 0f) return 0f;
        actualKg[i] += kg;
        return kg;
    }

    public float QuantityChange(float x, float z)
    {
        int i = CellIndex(x, z);
        if (i < 0) return 0f;
        return (actualKg[i] - initialKg[i]) / (regolithDensity * cellSize * cellSize);
    }

    private int CellIndex(float x, float z)
    {
        int ix = Mathf.FloorToInt((x - originX) / cellSize);
        int iz = Mathf.FloorToInt((z - originZ) / cellSize);
        if (ix < 0 || iz < 0 || ix >= cellsX || iz >= cellsZ) return -1;
        return iz * cellsX + ix;
    }
}