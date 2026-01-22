using UnityEngine;
using System.Collections.Generic;
using System;
using System.Threading.Tasks;
using Sirenix.OdinInspector;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Jobs;
using Unity.Burst;

/// <summary>
/// Advanced Volumetric Soft Body Physics using Position Based Dynamics (PBD) and Verlet Integration.
/// Uses SoftBodyTruss ScriptableObject for data storage, allowing editor-time editing.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class AdvancedVolumetricSoftBody : MonoBehaviour
{
    #region Editor Settings Classes
    
    [Serializable]
    public class SelectedPointSettings
    {
        [LabelText("Inverse Mass")]
        [Range(0f, 10f)]
        public float inverseMass = 1f;
        
        [LabelText("Is Pinned")]
        public bool isPinned = false;
    }
    
    [Serializable]
    public class SelectedBeamSettings
    {
        [LabelText("Rest Length")]
        [MinValue(0.001f)]
        public float restLength = 1f;
        
        [LabelText("Stiffness Override (-1 = global)")]
        [Range(-1f, 1f)]
        public float stiffnessOverride = -1f;
    }
    
    #endregion
    
    #region Truss Data Reference
    
    [TitleGroup("Truss Data")]
    [Required("Assign a SoftBodyTruss asset to store the physics mesh")]
    [InlineEditor(InlineEditorModes.GUIOnly)]
    public SoftBodyTruss trussData;
    
    #endregion
    
    #region Danger Zone Buttons
    
    [TitleGroup("Danger Zone")]
    [HorizontalGroup("Danger Zone/Buttons", Width = 0.5f)]
    [GUIColor(1f, 0.4f, 0.4f)]
    [Button("Clear Beams", ButtonSizes.Medium)]
    [DisableInPlayMode]
    [PropertyOrder(-110)]
    public void EditorClearBeams()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        RecordUndoIfNotPlaying("Clear Beams");
        trussData.ClearBeams();
        MarkTrussDirty();
    }
    
    [HorizontalGroup("Danger Zone/Buttons")]
    [GUIColor(1f, 0.4f, 0.4f)]
    [Button("Clear Grid", ButtonSizes.Medium)]
    [DisableInPlayMode]
    [PropertyOrder(-109)]
    public void EditorClearGrid()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        RecordUndoIfNotPlaying("Clear Grid");
        trussData.ClearGrid();
        MarkTrussDirty();
    }
    
    #endregion
    
    #region Editor Tools
    
    [TitleGroup("Tools")]
    [HorizontalGroup("Tools/Buttons", Width = 0.25f)]
    [GUIColor(0.4f, 0.9f, 0.4f)]
    [Button("Generate From Mesh", ButtonSizes.Medium)]
    [DisableInPlayMode]
    [PropertyOrder(-100)]
    public void EditorGenerateFromMesh()
    {
        if (trussData == null)
        {
            Debug.LogError("Assign a SoftBodyTruss asset first!");
            return;
        }
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        RecordUndoIfNotPlaying("Generate From Mesh");
        GenerateFromMesh(false); // Don't clear existing data
        MarkTrussDirty();
    }
    
    [HorizontalGroup("Tools/Buttons")]
    [Button("New Point", ButtonSizes.Medium)]
    [PropertyOrder(-99)]
    public void EditorCreatePoint()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        #if UNITY_EDITOR
        var sceneView = UnityEditor.SceneView.lastActiveSceneView;
        if (sceneView != null)
        {
            RecordUndoIfNotPlaying("Create Point");
            
            Vector3 worldPos = sceneView.camera.transform.position + sceneView.camera.transform.forward * newPointDistance;
            Vector3 localPos = transform.InverseTransformPoint(worldPos);
            
            trussData.DeselectAllNodes();
            trussData.DeselectAllBeams();
            
            int newIndex = trussData.AddNode(localPos, 1f, -1);
            trussData.nodes[newIndex].isSelected = true;
            
            MarkTrussDirty();
        }
        #endif
    }
    
    [HorizontalGroup("Tools/Buttons")]
    [Button("New Beam(s)", ButtonSizes.Medium)]
    [EnableIf("HasMultiplePointsSelected")]
    [PropertyOrder(-98)]
    public void EditorCreateBeams()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        var selectedIndices = trussData.GetSelectedNodeIndices();
        if (selectedIndices.Count < 2) return;
        
        RecordUndoIfNotPlaying("Create Beams");
        
        trussData.DeselectAllNodes();
        
        // Create beams between all pairs
        for (int i = 0; i < selectedIndices.Count; i++)
        {
            for (int j = i + 1; j < selectedIndices.Count; j++)
            {
                int a = selectedIndices[i];
                int b = selectedIndices[j];
                
                Vector3 posA = transform.TransformPoint(trussData.nodes[a].localPosition);
                Vector3 posB = transform.TransformPoint(trussData.nodes[b].localPosition);
                float length = (posA - posB).magnitude;
                
                trussData.AddBeam(a, b, length);
            }
        }
        
        // If tetrahedra are disabled and we have 3+ nodes, create triangles
        if (!generateTetrahedra && selectedIndices.Count >= 3)
        {
            // Create triangles for all triplets
            for (int i = 0; i < selectedIndices.Count - 2; i++)
            {
                for (int j = i + 1; j < selectedIndices.Count - 1; j++)
                {
                    for (int k = j + 1; k < selectedIndices.Count; k++)
                    {
                        int a = selectedIndices[i];
                        int b = selectedIndices[j];
                        int c = selectedIndices[k];
                        
                        float area = CalculateTriangleArea(
                            trussData.nodes[a].localPosition,
                            trussData.nodes[b].localPosition,
                            trussData.nodes[c].localPosition
                        );
                        
                        if (area > 0.00001f)
                        {
                            trussData.triangles.Add(new SoftBodyTruss.TrussTriangle(a, b, c, area));
                        }
                    }
                }
            }
        }
        
        // Select the newly created beams
        trussData.DeselectAllBeams();
        foreach (var beam in trussData.beams)
        {
            if (selectedIndices.Contains(beam.nodeA) && selectedIndices.Contains(beam.nodeB))
                beam.isSelected = true;
        }
        
        MarkTrussDirty();
    }
    
    [HorizontalGroup("Tools/Buttons")]
    [Button("Reset Deformation", ButtonSizes.Medium)]
    [PropertyOrder(-97)]
    public void EditorResetDeformation()
    {
        if (!Application.isPlaying) return;
        ResetDeformation();
    }
    
    [PropertySpace(5)]
    [TitleGroup("Tools")]
    [LabelText("New Point Distance")]
    [Range(1f, 20f)]
    public float newPointDistance = 5f;
    
    #endregion
    
    #region Selection UI
    
    [TitleGroup("Selection")]
    [ShowIf("HasPointsSelected")]
    [BoxGroup("Selection/SelectedPoint")]
    [HideLabel]
    [InlineProperty]
    public SelectedPointSettings selectedPointSettings = new SelectedPointSettings();
    
    [TitleGroup("Selection")]
    [ShowIf("HasPointsSelected")]
    [BoxGroup("Selection/SelectedPoint")]
    [Button("Apply to Selected Points")]
    public void ApplyPointSettings()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        RecordUndoIfNotPlaying("Apply Point Settings");
        
        foreach (var node in trussData.nodes)
        {
            if (node.isSelected)
            {
                node.inverseMass = selectedPointSettings.inverseMass;
                node.isPinned = selectedPointSettings.isPinned;
            }
        }
        
        MarkTrussDirty();
    }
    
    [TitleGroup("Selection")]
    [ShowIf("HasBeamsSelected")]
    [BoxGroup("Selection/SelectedBeam")]
    [HideLabel]
    [InlineProperty]
    public SelectedBeamSettings selectedBeamSettings = new SelectedBeamSettings();
    
    [TitleGroup("Selection")]
    [ShowIf("HasBeamsSelected")]
    [BoxGroup("Selection/SelectedBeam")]
    [Button("Apply to Selected Beams")]
    public void ApplyBeamSettings()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        RecordUndoIfNotPlaying("Apply Beam Settings");
        
        foreach (var beam in trussData.beams)
        {
            if (beam.isSelected)
            {
                beam.restLength = selectedBeamSettings.restLength;
                beam.stiffnessOverride = selectedBeamSettings.stiffnessOverride;
            }
        }
        
        MarkTrussDirty();
    }
    
    // Condition methods for ShowIf
    private bool HasPointsSelected() => trussData != null && trussData.HasSelectedNodes();
    private bool HasBeamsSelected() => trussData != null && trussData.HasSelectedBeams();
    private bool HasMultiplePointsSelected() => trussData != null && trussData.SelectedNodeCount() >= 2;
    
    #endregion
    
    #region Generation Settings
    
    [TitleGroup("Generation")]
    [LabelText("Use Tetrahedra (3D Volume)")]
    [Tooltip("If true, generates internal tetrahedra. If false, uses surface triangles only.")]
    public bool generateTetrahedra = true;
    
    [TitleGroup("Generation")]
    [LabelText("Resolution")]
    [Range(0f, 1f)]
    [Tooltip("1 = node per unique vertex. <1 = fewer nodes via voxel clustering.")]
    public float GenerationResolution = 1f;
    
    [TitleGroup("Generation")]
    [ShowIf("generateTetrahedra")]
    [LabelText("Internal Point Count")]
    [Range(0, 100)]
    public int internalPointCount = 20;
    
    [TitleGroup("Generation")]
    [ShowIf("generateTetrahedra")]
    [LabelText("Point Spacing")]
    [Range(0.05f, 0.5f)]
    public float pointSpacing = 0.15f;
    
    #endregion
    
    #region Linked Meshes
    
    /// <summary>
    /// Mesh that deforms based on nearby soft body nodes.
    /// </summary>
    [Serializable]
    public class LinkedMesh
    {
        [Required]
        public MeshFilter meshFilter;
        
        [LabelText("Influence Radius")]
        [Min(0.0001f)]
        [Tooltip("Vertices within this distance from nodes will be affected.")]
        public float influenceRadius = 0.25f;
        
        [LabelText("Max Influences")]
        [Range(1, 8)]
        [Tooltip("Maximum number of nodes affecting each vertex.")]
        public int maxInfluences = 4;
        
        [LabelText("Strength")]
        [Range(0f, 5f)]
        public float strength = 1f;
        
        [LabelText("Recalc Normals")]
        public bool recalcNormals = false;
        
        [LabelText("Recalc Bounds")]
        public bool recalcBounds = true;
        
        // Runtime data (NonSerialized)
        [NonSerialized] public Mesh runtimeMesh;
        [NonSerialized] public NativeArray<float3> restVertsLocal;
        [NonSerialized] public NativeArray<float3> deformedVertsLocal;
        [NonSerialized] public NativeArray<int> bindNodeIdx;     // len = vCount * maxInfluences
        [NonSerialized] public NativeArray<float> bindNodeW;     // len = vCount * maxInfluences
        [NonSerialized] public NativeArray<byte> bindCount;      // len = vCount
        [NonSerialized] public bool isInitialized;
    }
    
    [TitleGroup("Linked Meshes")]
    [Tooltip("Additional meshes that deform based on this soft body's node grid.")]
    public List<LinkedMesh> linkedMeshes = new List<LinkedMesh>();
    
    #endregion
    
    #region Physics Settings
    
    [TitleGroup("Physics")]
    [LabelText("Gravity")]
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    
    [TitleGroup("Physics")]
    [LabelText("Damping")]
    [Range(0f, 1f)]
    public float damping = 0.01f;
    
    [TitleGroup("Physics")]
    [LabelText("Distance Stiffness")]
    [Range(0f, 1f)]
    public float distanceStiffness = 0.9f;
    
    [TitleGroup("Physics")]
    [LabelText("Volume Stiffness")]
    [Range(0f, 1f)]
    public float volumeStiffness = 0.5f;

    // ... (rest of file)

    // In FixedUpdate
    // float stiffness = 1f - Mathf.Pow(1f - distanceStiffness, 1f / solverIterations);
    // Becomes:
    // float stiffness = 1f - Mathf.Pow(1f - distanceStiffness, 1f / SoftBodySimulationManager.Instance.globalSolverIterations);

    // ... (rest of file)

    /// <summary>
    /// Estimate node count based on current resolution settings (Editor only).
    /// </summary>
    public int GetEstimatedNodeCount()
    {
        if (meshFilter == null) meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null) return 0;
        
        Vector3[] verts = meshFilter.sharedMesh.vertices;
        if (GenerationResolution >= 0.99f) return verts.Length;
        
        Bounds bounds = meshFilter.sharedMesh.bounds;
        float voxelSize = bounds.size.magnitude * (1f - GenerationResolution) * 0.1f;
        if (voxelSize <= 0.0001f) return verts.Length;

        HashSet<int3> voxelSet = new HashSet<int3>();
        int count = 0;
        
        foreach (var v in verts)
        {
            int3 voxel = new int3(
                (int)math.floor(v.x / voxelSize),
                (int)math.floor(v.y / voxelSize),
                (int)math.floor(v.z / voxelSize)
            );
            
            if (voxelSet.Add(voxel))
            {
                count++;
            }
        }
        
        // Add estimate for internal nodes if tetrahedra enabled
        if (generateTetrahedra)
        {
            count += internalPointCount;
        }
        
        return count;
    }
    
    #endregion
    
    #region Plasticity Settings
    
    [TitleGroup("Plasticity")]
    [LabelText("Enable Plasticity")]
    public bool enablePlasticity = true;
    
    [TitleGroup("Plasticity")]
    [ShowIf("enablePlasticity")]
    [LabelText("Threshold")]
    [Range(0f, 1f)]
    public float plasticityThreshold = 0.3f;
    
    [TitleGroup("Plasticity")]
    [ShowIf("enablePlasticity")]
    [LabelText("Amount")]
    [Range(0f, 1f)]
    public float plasticityAmount = 0.5f;
    
    #endregion
    
    #region Collision Settings
    
    [TitleGroup("Collision")]
    [LabelText("Enable Collision")]
    [Tooltip("If disabled, object will pass through surfaces")]
    public bool enableCollision = true;
    
    [TitleGroup("Collision")]
    [FoldoutGroup("Collision/Settings")]
    [EnableIf("enableCollision")]
    [LabelText("Collision Radius")]
    public float collisionRadius = 0.05f;
    
    [FoldoutGroup("Collision/Settings")]
    [EnableIf("enableCollision")]
    [LabelText("Friction")]
    [Range(0f, 1f)]
    public float friction = 0.3f;
    
    [FoldoutGroup("Collision/Settings")]
    [EnableIf("enableCollision")]
    [LabelText("Collision Layers")]
    public LayerMask collisionLayers = ~0;
    
    #endregion
    
    #region Deformation Settings
    
    [TitleGroup("Deformation")]
    [LabelText("Enable Contact Deformation")]
    [Tooltip("If disabled, collisions won't cause deformation")]
    public bool enableDeformation = true;
    
    #endregion
    
    #region Debug Settings
    
    [TitleGroup("Debug")]
    [LabelText("Show Gizmos")]
    public bool showDebugGizmos = false;
    
    #endregion
    
    #region Private Fields
    
    private MeshFilter meshFilter;
    private Mesh originalMesh;
    private Mesh deformedMesh;
    
    private Vector3[] originalMeshVertices;
    private int[] meshTriangles;
    
    private Collider[] colliderBuffer = new Collider[32];
    private bool isRuntimeInitialized = false;
    
    // Cloned truss data for runtime (so we don't modify the SO)
    private SoftBodyTruss runtimeTruss;
    
    /// <summary>
    /// Returns the runtime truss data when playing, or the SO truss data in editor.
    /// Use this property in editor scripts to get correct node positions at runtime.
    /// </summary>
    public SoftBodyTruss RuntimeTruss => Application.isPlaying && runtimeTruss != null ? runtimeTruss : trussData;
    
    // ========== NativeArrays for Jobs ==========
    // Positions (float3 for math)
    private NativeArray<float3> nativePositions;
    private NativeArray<float3> nativePrevPositions;
    private NativeArray<float3> nativeDeltaPositions; // Jacobi accumulator
    private NativeArray<int> nativeDeltaCounts;       // Count of constraints per node
    
    // Node properties
    private NativeArray<float> nativeInverseMass;
    private NativeArray<byte> nativePinned;  // 0 = not pinned, 1 = pinned
    
    // Beam constraints
    private NativeArray<int2> nativeBeamNodes;
    private NativeArray<float> nativeBeamRestLengths;
    private NativeArray<float> nativeBeamStiffness;
    
    // Mesh vertex mapping
    private NativeArray<int> nativeVertexToNode;
    private NativeArray<float3> nativeDeformedVertices;
    
    // ========== Offset-based deformation ==========
    // Node rest positions (for delta computation)
    private NativeArray<float3> nativeNodeRestWorld;
    private NativeArray<float3> nativeNodeDeltaWorld;
    
    // Main mesh binding (for GenerationResolution < 1)
    private NativeArray<float3> nativeMainMeshRestLocal;
    private NativeArray<int> nativeMainMeshBindIdx;     // vertexCount * maxInfluences
    private NativeArray<float> nativeMainMeshBindW;     // vertexCount * maxInfluences
    private NativeArray<byte> nativeMainMeshBindCount;  // vertexCount
    private const int MAIN_MESH_MAX_INFLUENCES = 4;
    
    // Job handles for synchronization
    private JobHandle physicsJobHandle;
    
    #endregion
    
    #region Burst Jobs
    
    /// <summary>
    /// Verlet integration - moves nodes based on velocity and gravity
    /// </summary>
    [BurstCompile]
    private struct VerletIntegrationJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        public NativeArray<float3> prevPositions;
        [Unity.Collections.ReadOnly] public NativeArray<float> inverseMass;
        [Unity.Collections.ReadOnly] public NativeArray<byte> pinned;
        public float3 gravity;
        public float damping;
        public float dt;
        
        public void Execute(int i)
        {
            if (pinned[i] == 1 || inverseMass[i] <= 0f) return;
            
            float3 pos = positions[i];
            float3 prev = prevPositions[i];
            float3 velocity = pos - prev;
            float3 acceleration = gravity * dt;
            
            float3 newPos = pos + velocity * (1f - damping) + acceleration * dt;
            
            prevPositions[i] = pos;
            positions[i] = newPos;
        }
    }
    
    /// <summary>
    /// Clear delta accumulator before constraint solving
    /// </summary>
    [BurstCompile]
    private struct ClearDeltaJob : IJobParallelFor
    {
        public NativeArray<float3> deltaPositions;
        public NativeArray<int> deltaCounts;
        
        public void Execute(int i)
        {
            deltaPositions[i] = float3.zero;
            deltaCounts[i] = 0;
        }
    }
    
    /// <summary>
    /// Distance constraint solving - Jacobi style (accumulates deltas)
    /// </summary>
    [BurstCompile]
    private struct DistanceConstraintJob : IJobParallelFor
    {
        [Unity.Collections.ReadOnly] public NativeArray<float3> positions;
        [Unity.Collections.ReadOnly] public NativeArray<float> inverseMass;
        [Unity.Collections.ReadOnly] public NativeArray<byte> pinned;
        [Unity.Collections.ReadOnly] public NativeArray<int2> beamNodes;
        [Unity.Collections.ReadOnly] public NativeArray<float> restLengths;
        [Unity.Collections.ReadOnly] public NativeArray<float> stiffness;
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> deltaPositions;
        [NativeDisableParallelForRestriction]
        public NativeArray<int> deltaCounts;
        public float globalStiffness;
        
        public void Execute(int b)
        {
            int2 nodes = beamNodes[b];
            int idxA = nodes.x;
            int idxB = nodes.y;
            
            float3 posA = positions[idxA];
            float3 posB = positions[idxB];
            
            float3 delta = posB - posA;
            float currentLength = math.length(delta);
            
            if (currentLength < 0.0001f) return;
            
            float restLength = restLengths[b];
            float beamStiffness = stiffness[b] >= 0 ? stiffness[b] : globalStiffness;
            float error = currentLength - restLength;
            float3 correction = (delta / currentLength) * error * beamStiffness;
            
            // Exclude pinned from denominator
            float wA = pinned[idxA] == 1 ? 0f : inverseMass[idxA];
            float wB = pinned[idxB] == 1 ? 0f : inverseMass[idxB];
            float totalW = wA + wB;
            
            if (totalW <= 0f) return;
            
            // Accumulate deltas (Jacobi style - will be averaged later)
            if (wA > 0f)
            {
                // Atomic-like accumulation via NativeDisableParallelForRestriction
                deltaPositions[idxA] = deltaPositions[idxA] + correction * (wA / totalW);
                deltaCounts[idxA] = deltaCounts[idxA] + 1;
            }
            
            if (wB > 0f)
            {
                deltaPositions[idxB] = deltaPositions[idxB] - correction * (wB / totalW);
                deltaCounts[idxB] = deltaCounts[idxB] + 1;
            }
        }
    }
    
    /// <summary>
    /// Apply accumulated deltas (Jacobi resolution)
    /// </summary>
    [BurstCompile]
    private struct ApplyDeltaJob : IJobParallelFor
    {
        public NativeArray<float3> positions;
        [Unity.Collections.ReadOnly] public NativeArray<float3> deltaPositions;
        [Unity.Collections.ReadOnly] public NativeArray<int> deltaCounts;
        [Unity.Collections.ReadOnly] public NativeArray<byte> pinned;
        public float relaxation; // Usually 1.0 - 1.8
        
        public void Execute(int i)
        {
            if (pinned[i] == 1 || deltaCounts[i] == 0) return;
            
            // Average the accumulated deltas
            float3 avgDelta = deltaPositions[i] / deltaCounts[i];
            positions[i] = positions[i] + avgDelta * relaxation;
        }
    }
    
    /// <summary>
    /// Update mesh vertices from physics positions
    /// </summary>
    [BurstCompile]
    private struct UpdateMeshVerticesJob : IJobParallelFor
    {
        [Unity.Collections.ReadOnly] public NativeArray<float3> positions;
        [Unity.Collections.ReadOnly] public NativeArray<int> vertexToNode;
        public NativeArray<float3> deformedVertices;
        [Unity.Collections.ReadOnly] public NativeArray<float3> originalVertices;
        public float4x4 worldToLocal;
        
        public void Execute(int i)
        {
            int nodeIdx = vertexToNode[i];
            if (nodeIdx >= 0 && nodeIdx < positions.Length)
            {
                float3 worldPos = positions[nodeIdx];
                deformedVertices[i] = math.transform(worldToLocal, worldPos);
            }
            else
            {
                deformedVertices[i] = originalVertices[i];
            }
        }
    }
    
    /// <summary>
    /// Compute node deltas (current - rest) for offset-based deformation
    /// </summary>
    [BurstCompile]
    private struct ComputeNodeDeltaJob : IJobParallelFor
    {
        [Unity.Collections.ReadOnly] public NativeArray<float3> positions;
        [Unity.Collections.ReadOnly] public NativeArray<float3> restPositions;
        public NativeArray<float3> deltaWorld;
        
        public void Execute(int i)
        {
            deltaWorld[i] = positions[i] - restPositions[i];
        }
    }
    
    /// <summary>
    /// Deform mesh vertices using binding weights (offset-based)
    /// </summary>
    [BurstCompile]
    private struct DeformFromBindingJob : IJobParallelFor
    {
        [Unity.Collections.ReadOnly] public NativeArray<float3> restVertsLocal;
        [Unity.Collections.ReadOnly] public NativeArray<float3> nodeDeltaWorld;
        [Unity.Collections.ReadOnly] public NativeArray<int> bindNodeIdx;
        [Unity.Collections.ReadOnly] public NativeArray<float> bindNodeW;
        [Unity.Collections.ReadOnly] public NativeArray<byte> bindCount;
        public NativeArray<float3> deformedVertsLocal;
        public float4x4 worldToLocal;
        public float strength;
        public int maxInfluences;
        
        public void Execute(int v)
        {
            int k = bindCount[v];
            if (k == 0)
            {
                deformedVertsLocal[v] = restVertsLocal[v];
                return;
            }
            
            float3 sumDelta = float3.zero;
            for (int j = 0; j < k; j++)
            {
                int idx = v * maxInfluences + j;
                sumDelta += nodeDeltaWorld[bindNodeIdx[idx]] * bindNodeW[idx];
            }
            
            // Transform delta from world to local
            float3 deltaLocal = math.mul(worldToLocal, new float4(sumDelta, 0)).xyz;
            deformedVertsLocal[v] = restVertsLocal[v] + deltaLocal * strength;
        }
    }
    
    #endregion
    
    #region Internal Structures
    
    private struct Circumsphere
    {
        public Vector3 center;
        public float radiusSqr;
        
        public bool Contains(Vector3 point)
        {
            return (point - center).sqrMagnitude <= radiusSqr + 0.0001f;
        }
    }
    
    // Cached original vertices as NativeArray for job
    private NativeArray<float3> nativeOriginalVertices;
    
    #endregion
    
    #region Unity Lifecycle
    
    private void Start()
    {
        // Ensure simulation manager exists in this scene
        SoftBodySimulationManager.EnsureInstance();

        if (trussData == null)
        {
            Debug.LogError("AdvancedVolumetricSoftBody: No SoftBodyTruss assigned!");
            enabled = false;
            return;
        }
        
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            Debug.LogError("AdvancedVolumetricSoftBody: No MeshFilter or mesh found!");
            enabled = false;
            return;
        }
        
        // Clone truss data to prevent sharing between instances
        // This is critical - without this, multiple instances would conflict
        runtimeTruss = Instantiate(trussData);
        
        originalMesh = meshFilter.sharedMesh;
        deformedMesh = Instantiate(originalMesh);
        deformedMesh.MarkDynamic();
        meshFilter.mesh = deformedMesh;
        
        originalMeshVertices = originalMesh.vertices;
        meshTriangles = originalMesh.triangles;
        
        // Initialize runtime state arrays
        InitializeRuntimeArrays();
        RebuildVertexMapping();
        
        isRuntimeInitialized = true;
    }
    
    private void FixedUpdate()
    {
        if (!isRuntimeInitialized || !nativePositions.IsCreated || nativePositions.Length == 0) return;
        
        float dt = Time.fixedDeltaTime;
        int nodeCount = nativePositions.Length;
        int beamCount = nativeBeamNodes.Length;
        
        if (SoftBodySimulationManager.Instance == null) return;
        
        // Calculate stiffness for this frame
        // Use global solver iterations
        int iterations = SoftBodySimulationManager.Instance.globalSolverIterations;
        float stiffness = 1f - Mathf.Pow(1f - distanceStiffness, 1f / iterations);
        
        // ========== 1. Verlet Integration ==========
        var integrationJob = new VerletIntegrationJob
        {
            positions = nativePositions,
            prevPositions = nativePrevPositions,
            inverseMass = nativeInverseMass,
            pinned = nativePinned,
            gravity = (float3)gravity,
            damping = damping,
            dt = dt
        };
        physicsJobHandle = integrationJob.Schedule(nodeCount, 64);
        
        // ========== 2. Constraint Solving (Jacobi) ==========
        for (int iter = 0; iter < iterations; iter++)
        {
            // Clear delta accumulators
            var clearJob = new ClearDeltaJob
            {
                deltaPositions = nativeDeltaPositions,
                deltaCounts = nativeDeltaCounts
            };
            physicsJobHandle = clearJob.Schedule(nodeCount, 128, physicsJobHandle);
            
            // Distance constraints
            var distanceJob = new DistanceConstraintJob
            {
                positions = nativePositions,
                inverseMass = nativeInverseMass,
                pinned = nativePinned,
                beamNodes = nativeBeamNodes,
                restLengths = nativeBeamRestLengths,
                stiffness = nativeBeamStiffness,
                deltaPositions = nativeDeltaPositions,
                deltaCounts = nativeDeltaCounts,
                globalStiffness = stiffness
            };
            physicsJobHandle = distanceJob.Schedule(beamCount, 64, physicsJobHandle);
            
            // Apply accumulated deltas
            var applyJob = new ApplyDeltaJob
            {
                positions = nativePositions,
                deltaPositions = nativeDeltaPositions,
                deltaCounts = nativeDeltaCounts,
                pinned = nativePinned,
                relaxation = 1.2f  // SOR relaxation for faster convergence
            };
            physicsJobHandle = applyJob.Schedule(nodeCount, 64, physicsJobHandle);
        }
        
        // Complete physics jobs before collision (PhysX needs main thread)
        physicsJobHandle.Complete();
        
        // ========== 3. Collisions (main thread - PhysX) ==========
        if (enableCollision)
            HandleCollisions();
        
        // ========== 4. Plasticity ==========
        // if (enablePlasticity)
        //     UpdatePlasticity(); // Replaced by Burst job (TODO: Implement Burst Plasticity)
    }
    
    private void LateUpdate()
    {
        if (!isRuntimeInitialized || !nativePositions.IsCreated) return;
        
        // Sync physics positions back to trussData for editor visualization
        SyncPositionsToTrussData();
        
        // Schedule mesh update job
        var meshJob = new UpdateMeshVerticesJob
        {
            positions = nativePositions,
            vertexToNode = nativeVertexToNode,
            deformedVertices = nativeDeformedVertices,
            originalVertices = nativeOriginalVertices,
            worldToLocal = (float4x4)transform.worldToLocalMatrix
        };
        
        JobHandle meshHandle = meshJob.Schedule(nativeDeformedVertices.Length, 128);
        meshHandle.Complete();
        
        // Apply to mesh using SetVertexBufferData (faster than vertices = ...)
        deformedMesh.SetVertexBufferData(nativeDeformedVertices, 0, 0, nativeDeformedVertices.Length);
        deformedMesh.RecalculateNormals();
        deformedMesh.RecalculateBounds();
    }
    
    /// <summary>
    /// Sync native physics positions back to TrussData for editor visualization.
    /// This allows Editor scene handles to display correct node positions during play mode.
    /// </summary>
    private void SyncPositionsToTrussData()
    {
        if (runtimeTruss == null || !nativePositions.IsCreated) return;
        
        int nodeCount = Mathf.Min(runtimeTruss.nodes.Count, nativePositions.Length);
        for (int i = 0; i < nodeCount; i++)
        {
            runtimeTruss.nodes[i].worldPosition = nativePositions[i];
        }
    }
    
    private void OnDisable()
    {
        // Ensure any running job is finished immediately
        physicsJobHandle.Complete();
    }
    
    private void OnDestroy()
    {
        // Double safety: complete jobs before disposing
        physicsJobHandle.Complete();
        isGenerating = false;
        
        if (deformedMesh != null)
            Destroy(deformedMesh);
        if (runtimeTruss != null)
            Destroy(runtimeTruss);
        
        // Dispose all NativeArrays
        DisposeNativeArrays();
    }
    
    private void DisposeNativeArrays()
    {
        if (nativePositions.IsCreated) nativePositions.Dispose();
        if (nativePrevPositions.IsCreated) nativePrevPositions.Dispose();
        if (nativeDeltaPositions.IsCreated) nativeDeltaPositions.Dispose();
        if (nativeDeltaCounts.IsCreated) nativeDeltaCounts.Dispose();
        if (nativeInverseMass.IsCreated) nativeInverseMass.Dispose();
        if (nativePinned.IsCreated) nativePinned.Dispose();
        if (nativeBeamNodes.IsCreated) nativeBeamNodes.Dispose();
        if (nativeBeamRestLengths.IsCreated) nativeBeamRestLengths.Dispose();
        if (nativeBeamStiffness.IsCreated) nativeBeamStiffness.Dispose();
        if (nativeVertexToNode.IsCreated) nativeVertexToNode.Dispose();
        if (nativeDeformedVertices.IsCreated) nativeDeformedVertices.Dispose();
        if (nativeOriginalVertices.IsCreated) nativeOriginalVertices.Dispose();
        
        // Offset deformation arrays
        if (nativeNodeRestWorld.IsCreated) nativeNodeRestWorld.Dispose();
        if (nativeNodeDeltaWorld.IsCreated) nativeNodeDeltaWorld.Dispose();
        
        // Main mesh binding arrays
        if (nativeMainMeshRestLocal.IsCreated) nativeMainMeshRestLocal.Dispose();
        if (nativeMainMeshBindIdx.IsCreated) nativeMainMeshBindIdx.Dispose();
        if (nativeMainMeshBindW.IsCreated) nativeMainMeshBindW.Dispose();
        if (nativeMainMeshBindCount.IsCreated) nativeMainMeshBindCount.Dispose();
        
        // LinkedMesh arrays
        foreach (var lm in linkedMeshes)
        {
            if (lm.restVertsLocal.IsCreated) lm.restVertsLocal.Dispose();
            if (lm.deformedVertsLocal.IsCreated) lm.deformedVertsLocal.Dispose();
            if (lm.bindNodeIdx.IsCreated) lm.bindNodeIdx.Dispose();
            if (lm.bindNodeW.IsCreated) lm.bindNodeW.Dispose();
            if (lm.bindCount.IsCreated) lm.bindCount.Dispose();
            if (lm.runtimeMesh != null) Destroy(lm.runtimeMesh);
            lm.isInitialized = false;
        }
    }
    
    #endregion
    
    #region Initialization
    

    /// <summary>
    /// Initialize all NativeArrays from the cloned truss data.
    /// This keeps all runtime state separate from the ScriptableObject.
    /// </summary>
    private void InitializeRuntimeArrays()
    {
        int nodeCount = runtimeTruss.nodes.Count;
        int beamCount = runtimeTruss.beams.Count;
        int vertexCount = originalMeshVertices.Length;
        
        // Dispose any existing arrays first
        DisposeNativeArrays();
        
        // ========== Node arrays ==========
        nativePositions = new NativeArray<float3>(nodeCount, Allocator.Persistent);
        nativePrevPositions = new NativeArray<float3>(nodeCount, Allocator.Persistent);
        nativeDeltaPositions = new NativeArray<float3>(nodeCount, Allocator.Persistent);
        nativeDeltaCounts = new NativeArray<int>(nodeCount, Allocator.Persistent);
        nativeInverseMass = new NativeArray<float>(nodeCount, Allocator.Persistent);
        nativePinned = new NativeArray<byte>(nodeCount, Allocator.Persistent);
        
        // Offset deformation arrays
        nativeNodeRestWorld = new NativeArray<float3>(nodeCount, Allocator.Persistent);
        nativeNodeDeltaWorld = new NativeArray<float3>(nodeCount, Allocator.Persistent);
        
        for (int i = 0; i < nodeCount; i++)
        {
            var node = runtimeTruss.nodes[i];
            float3 worldPos = (float3)transform.TransformPoint(node.localPosition);
            nativePositions[i] = worldPos;
            nativePrevPositions[i] = worldPos;
            nativeInverseMass[i] = node.inverseMass;
            nativePinned[i] = node.isPinned ? (byte)1 : (byte)0;
            
            nativeNodeRestWorld[i] = worldPos;
            nativeNodeDeltaWorld[i] = float3.zero;
        }
        
        // ========== Beam constraint arrays ==========
        nativeBeamNodes = new NativeArray<int2>(beamCount, Allocator.Persistent);
        nativeBeamRestLengths = new NativeArray<float>(beamCount, Allocator.Persistent);
        nativeBeamStiffness = new NativeArray<float>(beamCount, Allocator.Persistent);
        
        for (int i = 0; i < beamCount; i++)
        {
            var beam = runtimeTruss.beams[i];
            nativeBeamNodes[i] = new int2(beam.nodeA, beam.nodeB);
            nativeBeamRestLengths[i] = beam.restLength;
            nativeBeamStiffness[i] = beam.stiffnessOverride;
        }
        
        // ========== Mesh vertex arrays ==========
        nativeVertexToNode = new NativeArray<int>(vertexCount, Allocator.Persistent);
        nativeDeformedVertices = new NativeArray<float3>(vertexCount, Allocator.Persistent);
        nativeOriginalVertices = new NativeArray<float3>(vertexCount, Allocator.Persistent);
        
        for (int i = 0; i < vertexCount; i++)
        {
            nativeOriginalVertices[i] = (float3)originalMeshVertices[i];
            nativeDeformedVertices[i] = nativeOriginalVertices[i];
        }
        
        // ========== Binding System ==========
        InitializeBindings();
    }
    
    /// <summary>
    /// Calculate binding weights for main mesh and LinkedMeshes using Spatial Hash.
    /// </summary>
    private void InitializeBindings()
    {
        int nodeCount = runtimeTruss.nodes.Count;
        float3[] nodePos = new float3[nodeCount];
        for (int i = 0; i < nodeCount; i++)
            nodePos[i] = (float3)trussData.nodes[i].localPosition; // Use local position for binding
        
        // 1. Build Spatial Hash for nodes
        // Use max influence radius across all meshes to ensure coverage
        float maxRadius = 0.5f; // Base radius
        if (linkedMeshes != null)
        {
            foreach (var lm in linkedMeshes)
                maxRadius = Mathf.Max(maxRadius, lm.influenceRadius);
        }
        
        // Simple bucket grid
        // Key: tuple int3 -> Value: List<int> node indices
        Dictionary<int3, List<int>> buckets = new Dictionary<int3, List<int>>();
        float cellSize = maxRadius;
        
        for (int i = 0; i < nodeCount; i++)
        {
            float3 p = nodePos[i];
            int3 cell = (int3)math.floor(p / cellSize);
            
            // Add to cell and neighbors (to handle boundary cases easily)
            // Actually, inserting into just the cell is enough if we scan neighbors during query
            if (!buckets.TryGetValue(cell, out var list))
            {
                list = new List<int>();
                buckets[cell] = list;
            }
            list.Add(i);
        }
        
        // Helper to query nodes
        List<int> nearbyNodes = new List<int>();
        void QueryNodes(float3 pos, float radius, List<int> result)
        {
            result.Clear();
            int3 centerCell = (int3)math.floor(pos / cellSize);
            int cellRange = Mathf.CeilToInt(radius / cellSize);
            
            for (int x = -cellRange; x <= cellRange; x++)
            for (int y = -cellRange; y <= cellRange; y++)
            for (int z = -cellRange; z <= cellRange; z++)
            {
                int3 cell = centerCell + new int3(x, y, z);
                if (buckets.TryGetValue(cell, out var list))
                {
                    foreach (int idx in list)
                    {
                        if (math.distancesq(nodePos[idx], pos) <= radius * radius)
                            result.Add(idx);
                    }
                }
            }
        }
        
        // 2. Bind Main Mesh
        // Allocations
        int mainVertexCount = originalMeshVertices.Length;
        int maxInfluences = 4; // Standard skinning limit
        
        nativeMainMeshRestLocal = new NativeArray<float3>(mainVertexCount, Allocator.Persistent);
        nativeMainMeshBindIdx = new NativeArray<int>(mainVertexCount * maxInfluences, Allocator.Persistent);
        nativeMainMeshBindW = new NativeArray<float>(mainVertexCount * maxInfluences, Allocator.Persistent);
        nativeMainMeshBindCount = new NativeArray<byte>(mainVertexCount, Allocator.Persistent);
        
        // Binding loop
        for (int v = 0; v < mainVertexCount; v++)
        {
            float3 vertPos = (float3)originalMeshVertices[v];
            nativeMainMeshRestLocal[v] = vertPos;
            
            float radius = 0.5f; // TODO: Expose main mesh influence radius?
            QueryNodes(vertPos, radius, nearbyNodes);
            
            // Sort by distance (closest first)
            nearbyNodes.Sort((a, b) => math.distancesq(nodePos[a], vertPos).CompareTo(math.distancesq(nodePos[b], vertPos)));
            
            int count = math.min(nearbyNodes.Count, maxInfluences);
            nativeMainMeshBindCount[v] = (byte)count;
            
            float totalWeight = 0f;
            for (int k = 0; k < count; k++)
            {
                int nodeIdx = nearbyNodes[k];
                float dist = math.distance(nodePos[nodeIdx], vertPos);
                float weight = Mathf.Max(0f, 1f - (dist / radius)); // Linear falloff
                weight = weight * weight; // Quadratic falloff looks smoother
                
                int flatIdx = v * maxInfluences + k;
                nativeMainMeshBindIdx[flatIdx] = nodeIdx;
                nativeMainMeshBindW[flatIdx] = weight;
                totalWeight += weight;
            }
            
            // Normalize
            if (totalWeight > 0.0001f)
            {
                for (int k = 0; k < count; k++)
                    nativeMainMeshBindW[v * maxInfluences + k] /= totalWeight;
            }
            else if (count > 0)
            {
                // Fallback: mostly rigid to closest
                nativeMainMeshBindW[v * maxInfluences] = 1f;
            }
        }
        
        // 3. Bind Linked Meshes
        if (linkedMeshes != null)
        {
            foreach (var lm in linkedMeshes)
            {
                if (lm.meshFilter == null || lm.meshFilter.sharedMesh == null) continue;
                
                Mesh lmMesh = lm.meshFilter.sharedMesh;
                Vector3[] lmVerts = lmMesh.vertices;
                int lmVertCount = lmVerts.Length;
                
                // Create runtime mesh for deformation
                lm.runtimeMesh = Instantiate(lmMesh);
                lm.runtimeMesh.MarkDynamic();
                lm.meshFilter.mesh = lm.runtimeMesh;
                
                // Allocate native arrays
                lm.restVertsLocal = new NativeArray<float3>(lmVertCount, Allocator.Persistent);
                lm.deformedVertsLocal = new NativeArray<float3>(lmVertCount, Allocator.Persistent);
                lm.bindNodeIdx = new NativeArray<int>(lmVertCount * lm.maxInfluences, Allocator.Persistent);
                lm.bindNodeW = new NativeArray<float>(lmVertCount * lm.maxInfluences, Allocator.Persistent);
                lm.bindCount = new NativeArray<byte>(lmVertCount, Allocator.Persistent);
                
                // Binding loop
                for (int v = 0; v < lmVertCount; v++)
                {
                    // Convert linked mesh vertex to local space of the soft body
                    // Assuming linked mesh is a child or we need to handle transforms?
                    // Implementation Plan implies they move with the soft body, so local space binding is assumed relative to the SoftBody root.
                    // If LinkedMesh is a separate object, we must InverseTransformPoint from its world pos.
                    
                    Vector3 worldVert = lm.meshFilter.transform.TransformPoint(lmVerts[v]);
                    Vector3 localVert = transform.InverseTransformPoint(worldVert); // To SoftBody local space
                    
                    lm.restVertsLocal[v] = localVert;
                    lm.deformedVertsLocal[v] = localVert; // Initial state
                    
                    QueryNodes(localVert, lm.influenceRadius, nearbyNodes);
                    
                    // Sort by distance
                    nearbyNodes.Sort((a, b) => math.distancesq(nodePos[a], (float3)localVert).CompareTo(math.distancesq(nodePos[b], (float3)localVert)));
                    
                    int count = math.min(nearbyNodes.Count, lm.maxInfluences);
                    lm.bindCount[v] = (byte)count;
                    
                    float totalWeight = 0f;
                    for (int k = 0; k < count; k++)
                    {
                        int nodeIdx = nearbyNodes[k];
                        float dist = math.distance(nodePos[nodeIdx], (float3)localVert);
                        float weight = Mathf.Max(0f, 1f - (dist / lm.influenceRadius));
                        weight = weight * weight;
                        
                        int flatIdx = v * lm.maxInfluences + k;
                        lm.bindNodeIdx[flatIdx] = nodeIdx;
                        lm.bindNodeW[flatIdx] = weight;
                        totalWeight += weight;
                    }
                    
                    if (totalWeight > 0.0001f)
                    {
                        for (int k = 0; k < count; k++)
                            lm.bindNodeW[v * lm.maxInfluences + k] /= totalWeight;
                    }
                    else if (count > 0)
                    {
                        lm.bindNodeW[v * lm.maxInfluences] = 1f;
                    }
                }
                
                lm.isInitialized = true;
            }
        }
    }
    
    /// <summary>
    /// Build vertex-to-node mapping NativeArray for O(1) lookup in jobs.
    /// </summary>
    private void RebuildVertexMapping()
    {
        // Initialize with -1 (no mapping)
        for (int i = 0; i < nativeVertexToNode.Length; i++)
            nativeVertexToNode[i] = -1;
        
        for (int i = 0; i < runtimeTruss.nodes.Count; i++)
        {
            int vertIdx = runtimeTruss.nodes[i].originalVertexIndex;
            if (vertIdx >= 0 && vertIdx < nativeVertexToNode.Length)
            {
                nativeVertexToNode[vertIdx] = i;
            }
        }
    }
    
    /// <summary>
    /// Status flag for async generation.
    /// </summary>
    [HideInInspector]
    public bool isGenerating = false;
    
    /// <summary>
    /// Async mesh generation - runs Bowyer-Watson on background thread.
    /// </summary>
    public async void GenerateFromMesh(bool clearExisting = true, Action onComplete = null)
    {
        if (trussData == null || isGenerating) return;
        
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            Debug.LogError("No MeshFilter found!");
            return;
        }
        
        isGenerating = true;
        Debug.Log("Starting async mesh generation...");
        
        // ===== STEP 1: Copy mesh data on main thread =====
        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        Bounds bounds = mesh.bounds;
        
        if (clearExisting)
        {
            trussData.Clear();
        }
        
        // ===== Create nodes with voxel clustering based on GenerationResolution =====
        Dictionary<int, int> vertexToNewNode = new Dictionary<int, int>();
        
        if (GenerationResolution >= 0.99f)
        {
            // Resolution = 1: one node per unique vertex (original behavior)
            Dictionary<Vector3, int> uniquePositions = new Dictionary<Vector3, int>();
            
            for (int i = 0; i < vertices.Length; i++)
            {
                Vector3 rounded = RoundVector(vertices[i], 5);
                
                if (!uniquePositions.TryGetValue(rounded, out int nodeIndex))
                {
                    nodeIndex = trussData.AddNode(vertices[i], 1f, i);
                    uniquePositions[rounded] = nodeIndex;
                }
                
                vertexToNewNode[i] = nodeIndex;
            }
        }
        else
        {
            // Resolution < 1: voxel clustering to reduce node count
            int uniqueVertCount = CountUniqueVertices(vertices);
            int targetNodes = Mathf.Max(4, Mathf.RoundToInt(uniqueVertCount * GenerationResolution));
            
            // Compute cell size from volume
            float volume = bounds.size.x * bounds.size.y * bounds.size.z;
            float cellSize = Mathf.Pow(volume / Mathf.Max(1, targetNodes), 1f / 3f);
            cellSize = Mathf.Max(cellSize, 0.001f); // Minimum cell size
            
            // Voxel grid: cell key → list of vertex indices
            Dictionary<long, List<int>> voxelCells = new Dictionary<long, List<int>>();
            
            for (int i = 0; i < vertices.Length; i++)
            {
                Vector3 p = vertices[i];
                int cx = Mathf.FloorToInt((p.x - bounds.min.x) / cellSize);
                int cy = Mathf.FloorToInt((p.y - bounds.min.y) / cellSize);
                int cz = Mathf.FloorToInt((p.z - bounds.min.z) / cellSize);
                
                // Pack cell coords into long key
                long key = ((long)cx << 40) | ((long)(cy & 0xFFFFF) << 20) | (long)(cz & 0xFFFFF);
                
                if (!voxelCells.TryGetValue(key, out var cellVerts))
                {
                    cellVerts = new List<int>();
                    voxelCells[key] = cellVerts;
                }
                cellVerts.Add(i);
            }
            
            // Create one node per cell, position = average of vertices in cell
            foreach (var kvp in voxelCells)
            {
                List<int> cellVerts = kvp.Value;
                Vector3 avgPos = Vector3.zero;
                foreach (int vi in cellVerts)
                    avgPos += vertices[vi];
                avgPos /= cellVerts.Count;
                
                // First vertex index for originalVertexIndex (for binding)
                int nodeIndex = trussData.AddNode(avgPos, 1f, cellVerts[0]);
                
                // Map all vertices in this cell to this node
                foreach (int vi in cellVerts)
                    vertexToNewNode[vi] = nodeIndex;
            }
            
            Debug.Log($"Voxel clustering: {uniqueVertCount} vertices → {trussData.nodes.Count} nodes (resolution={GenerationResolution:F2})");
        }
        
        if (!generateTetrahedra)
        {
            // Surface constraints don't need background thread
            CreateSurfaceTriangleConstraints(triangles, vertexToNewNode);
            trussData.usedTetrahedra = false;
            isGenerating = false;
            Debug.Log($"Generated (sync): {trussData.nodes.Count} nodes, {trussData.beams.Count} beams");
            onComplete?.Invoke();
            return;
        }
        
        // ===== STEP 2: Copy node positions to plain array for thread =====
        int nodeCount = trussData.nodes.Count;
        float3[] nodePositions = new float3[nodeCount];
        for (int i = 0; i < nodeCount; i++)
        {
            nodePositions[i] = (float3)trussData.nodes[i].localPosition;
        }
        
        float3 boundsMin = (float3)bounds.min;
        float3 boundsMax = (float3)bounds.max;
        float3 boundsSize = boundsMax - boundsMin;
        float minSpacing = math.min(boundsSize.x, math.min(boundsSize.y, boundsSize.z)) * pointSpacing;
        int internalCount = internalPointCount;
        
        // Capture vertices/triangles for thread
        float3[] verticesF3 = new float3[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            verticesF3[i] = (float3)vertices[i];
        
        // ===== STEP 3: Run tetrahedralization in background thread =====
        // Use semaphore to limit concurrent generation (prevents CPU saturation with 5+ bodies)
        await SoftBodySimulationManager.AcquireGenerationSlot();
        TetraResult result;
        try
        {
            result = await Task.Run(() => 
            {
                return ComputeTetrahedraThreaded(nodePositions, verticesF3, triangles, 
                                                  boundsMin, boundsMax, minSpacing, internalCount);
            });
        }
        finally
        {
            SoftBodySimulationManager.ReleaseGenerationSlot();
        }
        
        // Safety check: Object might have been destroyed while waiting
        if (this == null || transform == null) return;
        
        // ===== STEP 4: Write results back on main thread =====
        // Add internal nodes
        foreach (var pos in result.internalNodes)
        {
            trussData.AddNode((Vector3)(float3)pos, 1f, -1);
        }
        
        // Add tetrahedra
        foreach (var tet in result.tetrahedra)
        {
            if (tet.w >= 0) // Valid tet
            {
                float volume = CalculateTetrahedronVolume(
                    trussData.nodes[tet.x].localPosition,
                    trussData.nodes[tet.y].localPosition,
                    trussData.nodes[tet.z].localPosition,
                    trussData.nodes[tet.w].localPosition
                );
                
                if (Mathf.Abs(volume) > 0.0001f)
                {
                    trussData.tetrahedra.Add(new SoftBodyTruss.TrussTetrahedron(
                        tet.x, tet.y, tet.z, tet.w, Mathf.Abs(volume)));
                }
            }
        }
        
        CreateTetrahedralConstraints();
        trussData.usedTetrahedra = true;
        
        isGenerating = false;
        Debug.Log($"Generated (async): {trussData.nodes.Count} nodes, {trussData.beams.Count} beams, " +
                  $"{trussData.tetrahedra.Count} tetrahedra");
        
        onComplete?.Invoke();
    }
    
    /// <summary>
    /// Result from background thread computation.
    /// </summary>
    private struct TetraResult
    {
        public List<float3> internalNodes;
        public List<int4> tetrahedra;
    }
    
    /// <summary>
    /// Pure C# computation - no Unity API calls. Runs on worker thread.
    /// </summary>
    private TetraResult ComputeTetrahedraThreaded(float3[] nodePositions, float3[] vertices, int[] triangles,
                                                   float3 boundsMin, float3 boundsMax, float minSpacing, int internalCount)
    {
        TetraResult result = new TetraResult
        {
            internalNodes = new List<float3>(),
            tetrahedra = new List<int4>()
        };
        
        // Working positions list (copy of original + internal + super)
        List<float3> workingPositions = new List<float3>(nodePositions);
        
        // Generate internal points (simplified - uses System.Random instead of UnityEngine.Random)
        System.Random rng = new System.Random();
        int attempts = 0;
        int maxAttempts = internalCount * 20;
        
        while (result.internalNodes.Count < internalCount && attempts < maxAttempts)
        {
            attempts++;
            
            float3 randomPoint = new float3(
                boundsMin.x + minSpacing + (float)rng.NextDouble() * (boundsMax.x - boundsMin.x - 2 * minSpacing),
                boundsMin.y + minSpacing + (float)rng.NextDouble() * (boundsMax.y - boundsMin.y - 2 * minSpacing),
                boundsMin.z + minSpacing + (float)rng.NextDouble() * (boundsMax.z - boundsMin.z - 2 * minSpacing)
            );
            
            // Check distance from existing nodes
            bool tooClose = false;
            float minSpacingSqr = minSpacing * minSpacing;
            
            foreach (var pos in workingPositions)
            {
                if (math.distancesq(pos, randomPoint) < minSpacingSqr)
                {
                    tooClose = true;
                    break;
                }
            }
            
            foreach (var pos in result.internalNodes)
            {
                if (math.distancesq(pos, randomPoint) < minSpacingSqr)
                {
                    tooClose = true;
                    break;
                }
            }
            
            if (!tooClose)
            {
                result.internalNodes.Add(randomPoint);
                workingPositions.Add(randomPoint);
            }
        }
        
        // Bowyer-Watson (pure math, no Unity API)
        if (workingPositions.Count < 4)
            return result;
        
        List<int4> workingTetrahedra = new List<int4>();
        
        // Create super-tetrahedron
        float3 center = float3.zero;
        foreach (var pos in workingPositions)
            center += pos;
        center /= workingPositions.Count;
        
        float maxDistSqr = 0;
        foreach (var pos in workingPositions)
            maxDistSqr = math.max(maxDistSqr, math.distancesq(pos, center));
        
        float radius = math.sqrt(maxDistSqr) * 3f;
        
        float3 sv0 = center + new float3(0, radius * 2, 0);
        float3 sv1 = center + new float3(radius * 1.732f, -radius, 0);
        float3 sv2 = center + new float3(-radius * 0.866f, -radius, radius * 1.5f);
        float3 sv3 = center + new float3(-radius * 0.866f, -radius, -radius * 1.5f);
        
        int superStart = workingPositions.Count;
        workingPositions.Add(sv0);
        workingPositions.Add(sv1);
        workingPositions.Add(sv2);
        workingPositions.Add(sv3);
        
        workingTetrahedra.Add(new int4(superStart, superStart + 1, superStart + 2, superStart + 3));
        
        int originalNodeCount = superStart;
        
        for (int i = 0; i < originalNodeCount; i++)
        {
            float3 point = workingPositions[i];
            List<int4> badTetrahedra = new List<int4>();
            
            foreach (var tet in workingTetrahedra)
            {
                float4 cs = CalculateCircumsphereMath(
                    workingPositions[tet.x],
                    workingPositions[tet.y],
                    workingPositions[tet.z],
                    workingPositions[tet.w]
                );
                
                if (math.distancesq(point, cs.xyz) <= cs.w + 0.0001f)
                    badTetrahedra.Add(tet);
            }
            
            List<int3> boundaryFaces = new List<int3>();
            
            foreach (var tet in badTetrahedra)
            {
                int3[] faces = new int3[]
                {
                    new int3(tet.x, tet.y, tet.z),
                    new int3(tet.x, tet.y, tet.w),
                    new int3(tet.x, tet.z, tet.w),
                    new int3(tet.y, tet.z, tet.w)
                };
                
                foreach (var face in faces)
                {
                    bool isShared = false;
                    
                    foreach (var otherTet in badTetrahedra)
                    {
                        if (tet.Equals(otherTet)) continue;
                        
                        if (TetContainsFaceMath(otherTet, face))
                        {
                            isShared = true;
                            break;
                        }
                    }
                    
                    if (!isShared)
                        boundaryFaces.Add(face);
                }
            }
            
            foreach (var tet in badTetrahedra)
                workingTetrahedra.Remove(tet);
            
            foreach (var face in boundaryFaces)
            {
                workingTetrahedra.Add(new int4(face.x, face.y, face.z, i));
            }
        }
        
        // Remove tetrahedra containing super-tetrahedron vertices
        workingTetrahedra.RemoveAll(tet =>
            tet.x >= superStart || tet.y >= superStart ||
            tet.z >= superStart || tet.w >= superStart);
        
        result.tetrahedra = workingTetrahedra;
        
        return result;
    }
    
    /// <summary>
    /// Math-only circumsphere calculation (no Unity API).
    /// Returns float4: xyz = center, w = radiusSqr
    /// </summary>
    private float4 CalculateCircumsphereMath(float3 a, float3 b, float3 c, float3 d)
    {
        float3 ba = b - a;
        float3 ca = c - a;
        float3 da = d - a;
        
        float baLen = math.lengthsq(ba);
        float caLen = math.lengthsq(ca);
        float daLen = math.lengthsq(da);
        
        float3 cross = math.cross(ca, da) * baLen +
                       math.cross(da, ba) * caLen +
                       math.cross(ba, ca) * daLen;
        
        float denom = 2f * math.dot(ba, math.cross(ca, da));
        
        if (math.abs(denom) < 0.0001f)
            return new float4(a, float.MaxValue);
        
        float3 centerOffset = cross / denom;
        float3 center = a + centerOffset;
        
        return new float4(center, math.lengthsq(centerOffset));
    }
    
    /// <summary>
    /// Check if tetrahedron contains face (pure math).
    /// </summary>
    private bool TetContainsFaceMath(int4 tet, int3 face)
    {
        int matches = 0;
        if (tet.x == face.x || tet.y == face.x || tet.z == face.x || tet.w == face.x) matches++;
        if (tet.x == face.y || tet.y == face.y || tet.z == face.y || tet.w == face.y) matches++;
        if (tet.x == face.z || tet.y == face.z || tet.z == face.z || tet.w == face.z) matches++;
        return matches == 3;
    }
    
    private Vector3 RoundVector(Vector3 v, int decimals)
    {
        float mult = Mathf.Pow(10, decimals);
        return new Vector3(
            Mathf.Round(v.x * mult) / mult,
            Mathf.Round(v.y * mult) / mult,
            Mathf.Round(v.z * mult) / mult
        );
    }
    
    /// <summary>
    /// Count unique vertices (for voxel clustering calculation)
    /// </summary>
    private int CountUniqueVertices(Vector3[] vertices)
    {
        HashSet<Vector3> unique = new HashSet<Vector3>();
        foreach (var v in vertices)
            unique.Add(RoundVector(v, 5));
        return unique.Count;
    }
    
    private void GenerateInternalPoints(Mesh mesh)
    {
        if (internalPointCount <= 0) return;
        
        Bounds bounds = mesh.bounds;
        Vector3 size = bounds.size;
        float minSpacing = Mathf.Min(size.x, size.y, size.z) * pointSpacing;
        
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        
        int attempts = 0;
        int maxAttempts = internalPointCount * 20;
        int generated = 0;
        
        while (generated < internalPointCount && attempts < maxAttempts)
        {
            attempts++;
            
            Vector3 randomPoint = new Vector3(
                UnityEngine.Random.Range(bounds.min.x + minSpacing, bounds.max.x - minSpacing),
                UnityEngine.Random.Range(bounds.min.y + minSpacing, bounds.max.y - minSpacing),
                UnityEngine.Random.Range(bounds.min.z + minSpacing, bounds.max.z - minSpacing)
            );
            
            if (!IsPointInsideMesh(randomPoint, vertices, triangles, minSpacing))
                continue;
            
            bool tooClose = false;
            foreach (var node in trussData.nodes)
            {
                if ((node.localPosition - randomPoint).sqrMagnitude < minSpacing * minSpacing)
                {
                    tooClose = true;
                    break;
                }
            }
            
            if (!tooClose)
            {
                trussData.AddNode(randomPoint, 1f, -1);
                generated++;
            }
        }
    }
    
    private bool IsPointInsideMesh(Vector3 point, Vector3[] vertices, int[] triangles, float padding)
    {
        Mesh mesh = meshFilter.sharedMesh;
        Bounds bounds = mesh.bounds;
        bounds.Expand(-padding * 2);
        
        if (!bounds.Contains(point))
            return false;
        
        int intersections = 0;
        Vector3[] directions = { Vector3.right, Vector3.up, Vector3.forward };
        
        foreach (var dir in directions)
        {
            if (RayMeshIntersectionCount(point, dir, vertices, triangles) % 2 == 1)
                intersections++;
        }
        
        return intersections >= 2;
    }
    
    private int RayMeshIntersectionCount(Vector3 origin, Vector3 direction, Vector3[] vertices, int[] triangles)
    {
        int count = 0;
        
        for (int i = 0; i < triangles.Length; i += 3)
        {
            Vector3 v0 = vertices[triangles[i]];
            Vector3 v1 = vertices[triangles[i + 1]];
            Vector3 v2 = vertices[triangles[i + 2]];
            
            if (RayTriangleIntersect(origin, direction, v0, v1, v2, out float t) && t > 0)
                count++;
        }
        
        return count;
    }
    
    private bool RayTriangleIntersect(Vector3 origin, Vector3 dir, Vector3 v0, Vector3 v1, Vector3 v2, out float t)
    {
        t = 0;
        const float EPSILON = 0.0000001f;
        
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 h = Vector3.Cross(dir, edge2);
        float a = Vector3.Dot(edge1, h);
        
        if (a > -EPSILON && a < EPSILON)
            return false;
        
        float f = 1.0f / a;
        Vector3 s = origin - v0;
        float u = f * Vector3.Dot(s, h);
        
        if (u < 0.0f || u > 1.0f)
            return false;
        
        Vector3 q = Vector3.Cross(s, edge1);
        float v = f * Vector3.Dot(dir, q);
        
        if (v < 0.0f || u + v > 1.0f)
            return false;
        
        t = f * Vector3.Dot(edge2, q);
        return t > EPSILON;
    }
    
    #endregion
    
    #region Bowyer-Watson Tetrahedralization
    
    private void BowyerWatsonTetrahedralization()
    {
        if (trussData.nodes.Count < 4)
        {
            Debug.LogWarning("Not enough nodes for tetrahedralization");
            return;
        }
        
        List<int[]> workingTetrahedra = new List<int[]>();
        int[] superTetNodes = CreateSuperTetrahedron();
        workingTetrahedra.Add(new int[] { superTetNodes[0], superTetNodes[1], superTetNodes[2], superTetNodes[3] });
        
        int originalNodeCount = trussData.nodes.Count - 4;
        
        for (int i = 0; i < originalNodeCount; i++)
        {
            Vector3 point = trussData.nodes[i].localPosition;
            List<int[]> badTetrahedra = new List<int[]>();
            
            foreach (var tet in workingTetrahedra)
            {
                Circumsphere cs = CalculateCircumsphere(
                    trussData.nodes[tet[0]].localPosition,
                    trussData.nodes[tet[1]].localPosition,
                    trussData.nodes[tet[2]].localPosition,
                    trussData.nodes[tet[3]].localPosition
                );
                
                if (cs.Contains(point))
                    badTetrahedra.Add(tet);
            }
            
            List<int[]> boundaryFaces = new List<int[]>();
            
            foreach (var tet in badTetrahedra)
            {
                int[][] faces = GetTetrahedronFaces(tet);
                
                foreach (var face in faces)
                {
                    bool isShared = false;
                    
                    foreach (var otherTet in badTetrahedra)
                    {
                        if (tet == otherTet) continue;
                        
                        if (TetrahedronContainsFace(otherTet, face))
                        {
                            isShared = true;
                            break;
                        }
                    }
                    
                    if (!isShared)
                        boundaryFaces.Add(face);
                }
            }
            
            foreach (var tet in badTetrahedra)
                workingTetrahedra.Remove(tet);
            
            foreach (var face in boundaryFaces)
            {
                workingTetrahedra.Add(new int[] { face[0], face[1], face[2], i });
            }
        }
        
        int superStart = originalNodeCount;
        workingTetrahedra.RemoveAll(tet =>
            tet[0] >= superStart || tet[1] >= superStart ||
            tet[2] >= superStart || tet[3] >= superStart);
        
        // Remove super-tetrahedron nodes
        for (int i = 0; i < 4; i++)
            trussData.nodes.RemoveAt(superStart);
        
        // Convert to tetrahedra
        foreach (var tet in workingTetrahedra)
        {
            float volume = CalculateTetrahedronVolume(
                trussData.nodes[tet[0]].localPosition,
                trussData.nodes[tet[1]].localPosition,
                trussData.nodes[tet[2]].localPosition,
                trussData.nodes[tet[3]].localPosition
            );
            
            if (Mathf.Abs(volume) > 0.0001f)
            {
                trussData.tetrahedra.Add(new SoftBodyTruss.TrussTetrahedron(
                    tet[0], tet[1], tet[2], tet[3], Mathf.Abs(volume)));
            }
        }
    }
    
    private int[] CreateSuperTetrahedron()
    {
        Vector3 center = Vector3.zero;
        foreach (var node in trussData.nodes)
            center += node.localPosition;
        center /= trussData.nodes.Count;
        
        float maxDistSqr = 0;
        foreach (var node in trussData.nodes)
            maxDistSqr = Mathf.Max(maxDistSqr, (node.localPosition - center).sqrMagnitude);
        
        float radius = Mathf.Sqrt(maxDistSqr) * 3f;
        
        Vector3 v0 = center + new Vector3(0, radius * 2, 0);
        Vector3 v1 = center + new Vector3(radius * 1.732f, -radius, 0);
        Vector3 v2 = center + new Vector3(-radius * 0.866f, -radius, radius * 1.5f);
        Vector3 v3 = center + new Vector3(-radius * 0.866f, -radius, -radius * 1.5f);
        
        int startIndex = trussData.nodes.Count;
        trussData.AddNode(v0, 0f, -1);
        trussData.AddNode(v1, 0f, -1);
        trussData.AddNode(v2, 0f, -1);
        trussData.AddNode(v3, 0f, -1);
        
        return new int[] { startIndex, startIndex + 1, startIndex + 2, startIndex + 3 };
    }
    
    private Circumsphere CalculateCircumsphere(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        Vector3 ba = b - a;
        Vector3 ca = c - a;
        Vector3 da = d - a;
        
        float baLen = ba.sqrMagnitude;
        float caLen = ca.sqrMagnitude;
        float daLen = da.sqrMagnitude;
        
        Vector3 cross = Vector3.Cross(ca, da) * baLen +
                        Vector3.Cross(da, ba) * caLen +
                        Vector3.Cross(ba, ca) * daLen;
        
        float denom = 2f * Vector3.Dot(ba, Vector3.Cross(ca, da));
        
        Circumsphere cs = new Circumsphere();
        
        if (Mathf.Abs(denom) < 0.0000001f)
        {
            cs.center = (a + b + c + d) / 4f;
            cs.radiusSqr = float.MaxValue;
        }
        else
        {
            cs.center = a + cross / denom;
            cs.radiusSqr = (cs.center - a).sqrMagnitude;
        }
        
        return cs;
    }
    
    private int[][] GetTetrahedronFaces(int[] tet)
    {
        return new int[][]
        {
            new int[] { tet[0], tet[1], tet[2] },
            new int[] { tet[0], tet[1], tet[3] },
            new int[] { tet[0], tet[2], tet[3] },
            new int[] { tet[1], tet[2], tet[3] }
        };
    }
    
    private bool TetrahedronContainsFace(int[] tet, int[] face)
    {
        int matches = 0;
        foreach (int fi in face)
        {
            foreach (int ti in tet)
            {
                if (fi == ti)
                {
                    matches++;
                    break;
                }
            }
        }
        return matches == 3;
    }
    
    private float CalculateTetrahedronVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ad = d - a;
        return Vector3.Dot(ab, Vector3.Cross(ac, ad)) / 6f;
    }
    
    private void CreateTetrahedralConstraints()
    {
        HashSet<long> addedEdges = new HashSet<long>();
        
        foreach (var tet in trussData.tetrahedra)
        {
            int[] pairs = { 0, 1, 0, 2, 0, 3, 1, 2, 1, 3, 2, 3 };
            
            for (int i = 0; i < pairs.Length; i += 2)
            {
                int a = tet.nodes[pairs[i]];
                int b = tet.nodes[pairs[i + 1]];
                
                long edgeKey = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
                
                if (!addedEdges.Contains(edgeKey))
                {
                    addedEdges.Add(edgeKey);
                    float length = (trussData.nodes[a].localPosition - trussData.nodes[b].localPosition).magnitude;
                    trussData.beams.Add(new SoftBodyTruss.TrussBeam(a, b, length));
                }
            }
        }
    }
    
    #endregion
    
    #region Surface Triangle Mode
    
    private void CreateSurfaceTriangleConstraints(int[] triangles, Dictionary<int, int> vertexToNewNode)
    {
        HashSet<long> addedEdges = new HashSet<long>();
        
        for (int i = 0; i < triangles.Length; i += 3)
        {
            int a = vertexToNewNode[triangles[i]];
            int b = vertexToNewNode[triangles[i + 1]];
            int c = vertexToNewNode[triangles[i + 2]];
            
            if (a == b || b == c || a == c)
                continue;
            
            float area = CalculateTriangleArea(
                trussData.nodes[a].localPosition,
                trussData.nodes[b].localPosition,
                trussData.nodes[c].localPosition
            );
            
            if (area > 0.00001f)
                trussData.triangles.Add(new SoftBodyTruss.TrussTriangle(a, b, c, area));
            
            AddEdgeConstraint(a, b, addedEdges);
            AddEdgeConstraint(b, c, addedEdges);
            AddEdgeConstraint(c, a, addedEdges);
        }
    }
    
    private void AddEdgeConstraint(int a, int b, HashSet<long> addedEdges)
    {
        long edgeKey = a < b ? ((long)a << 32) | (uint)b : ((long)b << 32) | (uint)a;
        
        if (!addedEdges.Contains(edgeKey))
        {
            addedEdges.Add(edgeKey);
            float length = (trussData.nodes[a].localPosition - trussData.nodes[b].localPosition).magnitude;
            trussData.beams.Add(new SoftBodyTruss.TrussBeam(a, b, length));
        }
    }
    
    private float CalculateTriangleArea(Vector3 a, Vector3 b, Vector3 c)
    {
        return Vector3.Cross(b - a, c - a).magnitude * 0.5f;
    }
    
    #endregion
    
    #region Physics Simulation
    

    

    

    
    #endregion
    
    #region Visual Mesh Update
    

    
    #endregion
    
    #region Public API
    
    /// <summary>
    /// Pin nodes near a world position. Works at runtime.
    /// </summary>
    public void PinNodeAt(Vector3 worldPosition, float radius = 0.1f)
    {
        if (!nativePositions.IsCreated || !nativePinned.IsCreated) return;
        
        float radiusSqr = radius * radius;
        for (int i = 0; i < nativePositions.Length; i++)
        {
            if (math.distancesq(nativePositions[i], (float3)worldPosition) < radiusSqr)
            {
                nativePinned[i] = 1;
            }
        }
    }
    
    /// <summary>
    /// Unpin all nodes. Works at runtime.
    /// </summary>
    public void UnpinAllNodes()
    {
        if (!nativePinned.IsCreated) return;
        
        for (int i = 0; i < nativePinned.Length; i++)
            nativePinned[i] = 0;
    }
    
    /// <summary>
    /// Apply an impulse to nodes near a world position.
    /// </summary>
    public void ApplyImpulse(Vector3 worldPosition, Vector3 impulse, float radius = 0.5f)
    {
        if (!nativePositions.IsCreated || !nativePrevPositions.IsCreated || !nativePinned.IsCreated) return;
        
        float radiusSqr = radius * radius;
        float3 impulseF3 = (float3)impulse;
        float3 worldPosF3 = (float3)worldPosition;
        
        for (int i = 0; i < nativePositions.Length; i++)
        {
            if (nativePinned[i] == 1 || nativeInverseMass[i] <= 0f)
                continue;
            
            float distSqr = math.distancesq(nativePositions[i], worldPosF3);
            if (distSqr < radiusSqr)
            {
                float falloff = 1f - (distSqr / radiusSqr);
                nativePrevPositions[i] = nativePrevPositions[i] - impulseF3 * falloff * nativeInverseMass[i];
            }
        }
    }
    
    /// <summary>
    /// Reset all positions and rest values to initial state.
    /// </summary>

    public void ResetDeformation()
    {
        // Re-initialize all native arrays from the SO data
        if (runtimeTruss != null && isRuntimeInitialized)
        {
            InitializeRuntimeArrays();
        }
    }
    
    public void CreateMidpointOnSelectedBeams()
    {
        if (trussData == null) return;
        if (Application.isPlaying) return; // Don't modify SO during runtime
        
        RecordUndoIfNotPlaying("Create Midpoints");
        
        List<SoftBodyTruss.TrussBeam> selectedBeams = new List<SoftBodyTruss.TrussBeam>();
        foreach (var beam in trussData.beams)
            if (beam.isSelected) selectedBeams.Add(beam);
        
        if (selectedBeams.Count == 0) return;
        
        trussData.DeselectAllBeams();
        
        foreach (var beam in selectedBeams)
        {
            Vector3 midpoint = (trussData.nodes[beam.nodeA].localPosition + trussData.nodes[beam.nodeB].localPosition) * 0.5f;
            
            int newIndex = trussData.AddNode(midpoint, 1f, -1);
            trussData.nodes[newIndex].isSelected = true;
            
            float lengthA = (trussData.nodes[beam.nodeA].localPosition - midpoint).magnitude;
            float lengthB = (trussData.nodes[beam.nodeB].localPosition - midpoint).magnitude;
            
            // Use AddBeam() for duplicate prevention
            trussData.AddBeam(beam.nodeA, newIndex, lengthA);
            trussData.AddBeam(newIndex, beam.nodeB, lengthB);
        }
        
        trussData.beams.RemoveAll(b => selectedBeams.Contains(b));
        
        MarkTrussDirty();
    }
    
    #region Utility
    
    /// <summary>
    /// Mark truss dirty only in editor mode, not during play mode.
    /// Runtime changes should be temporary and not saved to the SO.
    /// </summary>
    private void MarkTrussDirty()
    {
        #if UNITY_EDITOR
        if (trussData != null && !Application.isPlaying)
            UnityEditor.EditorUtility.SetDirty(trussData);
        #endif
    }
    
    /// <summary>
    /// Record undo only in editor mode, not during play mode.
    /// </summary>
    private void RecordUndoIfNotPlaying(string actionName)
    {
        #if UNITY_EDITOR
        if (trussData != null && !Application.isPlaying)
            UnityEditor.Undo.RecordObject(trussData, actionName);
        #endif
    }
    
    #endregion
    

    

    /// <summary>
    /// Handle collisions with standard Unity colliders (Main Thread).
    /// </summary>
    private void HandleCollisions()
    {
        if (!nativePositions.IsCreated) return;
        
        int nodeCount = nativePositions.Length;
        for (int i = 0; i < nodeCount; i++)
        {
            if (nativePinned[i] == 1) continue;
            
            float3 pos = nativePositions[i];
            
            // Simple point collision
            int hitCount = Physics.OverlapSphereNonAlloc(pos, collisionRadius, colliderBuffer, collisionLayers);
            
            for (int k = 0; k < hitCount; k++)
            {
                Collider col = colliderBuffer[k];
                if (col.transform.root == transform.root) continue; // Ignore self
                
                Vector3 closestPoint = col.ClosestPoint(pos);
                float3 closest = (float3)closestPoint;
                float dist = math.distance(pos, closest);
                
                if (dist < collisionRadius)
                {
                    // Push out
                    float3 normal = math.normalize(pos - closest);
                    // If center is inside collider, normal might be zero/weird
                    if (math.lengthsq(normal) < 0.0001f) normal = new float3(0, 1, 0);
                    
                    float penetration = collisionRadius - dist;
                    float3 correction = normal * penetration;
                    
                    // Apply position correction
                    nativePositions[i] = pos + correction;
                    
                    // Friction (modify prevPosition)
                    float3 velocity = nativePositions[i] - nativePrevPositions[i];
                    
                    // Tangent velocity
                    float3 tangent = velocity - math.dot(velocity, normal) * normal;
                    nativePrevPositions[i] -= tangent * friction * math.min(1f, penetration * 100f); // Friction proportional to penetration
                }
            }
        }
    }
    
    private void OnDrawGizmosSelected()
    {
        if (!showDebugGizmos || trussData == null || trussData.nodes.Count == 0)
            return;
        
        // Draw nodes
        int nodeCount = trussData.nodes.Count;
        for (int i = 0; i < nodeCount; i++)
        {
            Vector3 pos;
            bool pinned = trussData.nodes[i].isPinned;
            bool selected = trussData.nodes[i].isSelected;
            
            if (Application.isPlaying && nativePositions.IsCreated && i < nativePositions.Length)
            {
                pos = nativePositions[i];
                pinned = nativePinned.IsCreated ? nativePinned[i] == 1 : pinned;
            }
            else
            {
                pos = transform.TransformPoint(trussData.nodes[i].localPosition);
            }
            
            Gizmos.color = pinned ? Color.red : (selected ? Color.yellow : Color.cyan);
            float size = selected ? 0.04f : 0.02f;
            Gizmos.DrawWireSphere(pos, size);
        }
        
        // Draw beams
        if (trussData.beams != null)
        {
            foreach (var beam in trussData.beams)
            {
                if (beam.nodeA >= nodeCount || beam.nodeB >= nodeCount)
                    continue;
                
                Vector3 posA, posB;
                
                if (Application.isPlaying && nativePositions.IsCreated && 
                    beam.nodeA < nativePositions.Length && beam.nodeB < nativePositions.Length)
                {
                    posA = nativePositions[beam.nodeA];
                    posB = nativePositions[beam.nodeB];
                }
                else
                {
                    posA = transform.TransformPoint(trussData.nodes[beam.nodeA].localPosition);
                    posB = transform.TransformPoint(trussData.nodes[beam.nodeB].localPosition);
                }
                
                Gizmos.color = beam.isSelected ? Color.green : Color.yellow;
                Gizmos.DrawLine(posA, posB);
            }
        }
    }
    
    #endregion
}
