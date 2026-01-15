using UnityEngine;
using System.Collections.Generic;
using System;
using Sirenix.OdinInspector;

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
    
    #region Physics Settings
    
    [TitleGroup("Physics")]
    [LabelText("Gravity")]
    public Vector3 gravity = new Vector3(0, -9.81f, 0);
    
    [TitleGroup("Physics")]
    [LabelText("Damping")]
    [Range(0f, 1f)]
    public float damping = 0.01f;
    
    [TitleGroup("Physics")]
    [LabelText("Solver Iterations")]
    [Range(1, 50)]
    public int solverIterations = 10;
    
    [TitleGroup("Physics")]
    [LabelText("Distance Stiffness")]
    [Range(0f, 1f)]
    public float distanceStiffness = 0.9f;
    
    [TitleGroup("Physics")]
    [LabelText("Volume Stiffness")]
    [Range(0f, 1f)]
    public float volumeStiffness = 0.5f;
    
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
    private Dictionary<int, int> vertexToNode = new Dictionary<int, int>();
    
    private Collider[] colliderBuffer = new Collider[10];
    private bool isRuntimeInitialized = false;
    
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
    
    #endregion
    
    #region Unity Lifecycle
    
    private void Start()
    {
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
        
        originalMesh = meshFilter.sharedMesh;
        deformedMesh = Instantiate(originalMesh);
        deformedMesh.MarkDynamic();
        meshFilter.mesh = deformedMesh;
        
        originalMeshVertices = originalMesh.vertices;
        meshTriangles = originalMesh.triangles;
        
        // Initialize runtime positions from local positions
        InitializeRuntimePositions();
        RebuildVertexMapping();
        
        isRuntimeInitialized = true;
    }
    
    private void FixedUpdate()
    {
        if (!isRuntimeInitialized || trussData == null || trussData.nodes.Count == 0) return;
        
        float dt = Time.fixedDeltaTime;
        
        IntegrateVerlet(dt);
        
        for (int iter = 0; iter < solverIterations; iter++)
        {
            SolveDistanceConstraints();
            
            if (trussData.usedTetrahedra && trussData.tetrahedra.Count > 0)
                SolveVolumeConstraints();
            else if (trussData.triangles.Count > 0)
                SolveAreaConstraints();
        }
        
        if (enableCollision)
            HandleCollisions();
        
        if (enablePlasticity)
            UpdatePlasticity();
    }
    
    private void LateUpdate()
    {
        if (isRuntimeInitialized)
            UpdateVisualMesh();
    }
    
    private void OnDestroy()
    {
        if (deformedMesh != null)
            Destroy(deformedMesh);
    }
    
    #endregion
    
    #region Initialization
    
    private void InitializeRuntimePositions()
    {
        foreach (var node in trussData.nodes)
        {
            node.worldPosition = transform.TransformPoint(node.localPosition);
            node.oldPosition = node.worldPosition;
        }
    }
    
    private void RebuildVertexMapping()
    {
        vertexToNode.Clear();
        
        for (int i = 0; i < trussData.nodes.Count; i++)
        {
            int vertIdx = trussData.nodes[i].originalVertexIndex;
            if (vertIdx >= 0)
            {
                vertexToNode[vertIdx] = i;
            }
        }
    }
    
    public void GenerateFromMesh(bool clearExisting = true)
    {
        if (trussData == null) return;
        
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null || meshFilter.sharedMesh == null)
        {
            Debug.LogError("No MeshFilter found!");
            return;
        }
        
        Mesh mesh = meshFilter.sharedMesh;
        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        
        if (clearExisting)
        {
            trussData.Clear();
        }
        
        // Remember existing node count for offset
        int nodeOffset = trussData.nodes.Count;
        
        // Create nodes from mesh vertices (merge duplicates)
        Dictionary<Vector3, int> uniquePositions = new Dictionary<Vector3, int>();
        Dictionary<int, int> vertexToNewNode = new Dictionary<int, int>();
        
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
        
        if (generateTetrahedra)
        {
            GenerateInternalPoints(mesh);
            BowyerWatsonTetrahedralization();
            CreateTetrahedralConstraints();
            trussData.usedTetrahedra = true;
        }
        else
        {
            CreateSurfaceTriangleConstraints(triangles, vertexToNewNode);
            trussData.usedTetrahedra = false;
        }
        
        Debug.Log($"Generated: {trussData.nodes.Count} nodes, {trussData.beams.Count} beams, " +
                  $"{trussData.tetrahedra.Count} tetrahedra, {trussData.triangles.Count} triangles");
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
    
    private void IntegrateVerlet(float dt)
    {
        Vector3 acceleration = gravity * dt;
        
        for (int i = 0; i < trussData.nodes.Count; i++)
        {
            var node = trussData.nodes[i];
            if (node.isPinned || node.inverseMass <= 0f)
                continue;
            
            Vector3 velocity = node.Velocity;
            Vector3 newPos = node.worldPosition + velocity * (1f - damping) + acceleration * dt;
            
            node.oldPosition = node.worldPosition;
            node.worldPosition = newPos;
        }
    }
    
    private void SolveDistanceConstraints()
    {
        float stiffness = 1f - Mathf.Pow(1f - distanceStiffness, 1f / solverIterations);
        
        foreach (var beam in trussData.beams)
        {
            var nodeA = trussData.nodes[beam.nodeA];
            var nodeB = trussData.nodes[beam.nodeB];
            
            Vector3 delta = nodeB.worldPosition - nodeA.worldPosition;
            float currentLength = delta.magnitude;
            
            if (currentLength < 0.0001f)
                continue;
            
            float beamStiffness = beam.stiffnessOverride >= 0 ? beam.stiffnessOverride : stiffness;
            float error = currentLength - beam.restLength;
            Vector3 correction = (delta / currentLength) * error * beamStiffness;
            
            float totalInvMass = nodeA.inverseMass + nodeB.inverseMass;
            if (totalInvMass <= 0f)
                continue;
            
            if (!nodeA.isPinned && nodeA.inverseMass > 0f)
                nodeA.worldPosition += correction * (nodeA.inverseMass / totalInvMass);
            
            if (!nodeB.isPinned && nodeB.inverseMass > 0f)
                nodeB.worldPosition -= correction * (nodeB.inverseMass / totalInvMass);
        }
    }
    
    private void SolveVolumeConstraints()
    {
        float stiffness = 1f - Mathf.Pow(1f - volumeStiffness, 1f / solverIterations);
        
        foreach (var tet in trussData.tetrahedra)
        {
            Vector3 p0 = trussData.nodes[tet.nodes[0]].worldPosition;
            Vector3 p1 = trussData.nodes[tet.nodes[1]].worldPosition;
            Vector3 p2 = trussData.nodes[tet.nodes[2]].worldPosition;
            Vector3 p3 = trussData.nodes[tet.nodes[3]].worldPosition;
            
            float currentVolume = CalculateTetrahedronVolume(p0, p1, p2, p3);
            float volumeError = currentVolume - tet.restVolume;
            
            tet.currentStress = Mathf.Abs(volumeError) / Mathf.Max(tet.restVolume, 0.0001f);
            
            if (Mathf.Abs(volumeError) < 0.0001f)
                continue;
            
            Vector3[] gradients = new Vector3[4];
            gradients[0] = Vector3.Cross(p2 - p1, p3 - p1) / 6f;
            gradients[1] = Vector3.Cross(p3 - p0, p2 - p0) / 6f;
            gradients[2] = Vector3.Cross(p1 - p0, p3 - p0) / 6f;
            gradients[3] = Vector3.Cross(p2 - p0, p1 - p0) / 6f;
            
            float denominator = 0f;
            for (int i = 0; i < 4; i++)
            {
                if (trussData.nodes[tet.nodes[i]].inverseMass > 0f)
                    denominator += trussData.nodes[tet.nodes[i]].inverseMass * gradients[i].sqrMagnitude;
            }
            
            if (denominator < 0.0001f)
                continue;
            
            float lambda = -volumeError * stiffness / denominator;
            
            for (int i = 0; i < 4; i++)
            {
                var node = trussData.nodes[tet.nodes[i]];
                if (!node.isPinned && node.inverseMass > 0f)
                {
                    node.worldPosition += gradients[i] * lambda * node.inverseMass;
                }
            }
        }
    }
    
    private void SolveAreaConstraints()
    {
        float stiffness = 1f - Mathf.Pow(1f - volumeStiffness, 1f / solverIterations);
        
        foreach (var tri in trussData.triangles)
        {
            Vector3 p0 = trussData.nodes[tri.nodes[0]].worldPosition;
            Vector3 p1 = trussData.nodes[tri.nodes[1]].worldPosition;
            Vector3 p2 = trussData.nodes[tri.nodes[2]].worldPosition;
            
            Vector3 edge1 = p1 - p0;
            Vector3 edge2 = p2 - p0;
            Vector3 cross = Vector3.Cross(edge1, edge2);
            float currentArea = cross.magnitude * 0.5f;
            float areaError = currentArea - tri.restArea;
            
            tri.currentStress = Mathf.Abs(areaError) / Mathf.Max(tri.restArea, 0.0001f);
            
            if (Mathf.Abs(areaError) < 0.0001f || currentArea < 0.0001f)
                continue;
            
            Vector3 normal = cross.normalized;
            
            Vector3[] gradients = new Vector3[3];
            gradients[0] = Vector3.Cross(normal, p2 - p1) * 0.5f;
            gradients[1] = Vector3.Cross(normal, p0 - p2) * 0.5f;
            gradients[2] = Vector3.Cross(normal, p1 - p0) * 0.5f;
            
            float denominator = 0f;
            for (int i = 0; i < 3; i++)
            {
                if (trussData.nodes[tri.nodes[i]].inverseMass > 0f)
                    denominator += trussData.nodes[tri.nodes[i]].inverseMass * gradients[i].sqrMagnitude;
            }
            
            if (denominator < 0.0001f)
                continue;
            
            float lambda = -areaError * stiffness / denominator;
            
            for (int i = 0; i < 3; i++)
            {
                var node = trussData.nodes[tri.nodes[i]];
                if (!node.isPinned && node.inverseMass > 0f)
                {
                    node.worldPosition += gradients[i] * lambda * node.inverseMass;
                }
            }
        }
    }
    
    #endregion
    
    #region Collision Detection
    
    private void HandleCollisions()
    {
        for (int i = 0; i < trussData.nodes.Count; i++)
        {
            var node = trussData.nodes[i];
            if (node.isPinned || node.inverseMass <= 0f)
                continue;
            
            int hitCount = Physics.OverlapSphereNonAlloc(node.worldPosition, collisionRadius, colliderBuffer, collisionLayers);
            
            for (int j = 0; j < hitCount; j++)
            {
                Collider col = colliderBuffer[j];
                if (col.transform == transform || col.isTrigger)
                    continue;
                
                Vector3 closestPoint = col.ClosestPoint(node.worldPosition);
                Vector3 toNode = node.worldPosition - closestPoint;
                float distance = toNode.magnitude;
                
                if (distance < collisionRadius && distance > 0.0001f)
                {
                    Vector3 normal = toNode / distance;
                    float penetration = collisionRadius - distance;
                    
                    node.worldPosition += normal * penetration;
                    
                    if (enableDeformation)
                    {
                        Vector3 velocity = node.Velocity;
                        Vector3 normalVel = Vector3.Dot(velocity, normal) * normal;
                        Vector3 tangentVel = velocity - normalVel;
                        
                        node.oldPosition = node.worldPosition - tangentVel * (1f - friction);
                    }
                    else
                    {
                        node.oldPosition = node.worldPosition;
                    }
                }
            }
        }
    }
    
    #endregion
    
    #region Plasticity
    
    private void UpdatePlasticity()
    {
        if (trussData.usedTetrahedra)
        {
            foreach (var tet in trussData.tetrahedra)
            {
                if (tet.currentStress > plasticityThreshold)
                {
                    float currentVolume = CalculateTetrahedronVolume(
                        trussData.nodes[tet.nodes[0]].worldPosition,
                        trussData.nodes[tet.nodes[1]].worldPosition,
                        trussData.nodes[tet.nodes[2]].worldPosition,
                        trussData.nodes[tet.nodes[3]].worldPosition
                    );
                    
                    float t = (tet.currentStress - plasticityThreshold) * plasticityAmount * Time.fixedDeltaTime;
                    tet.restVolume = Mathf.Lerp(tet.restVolume, Mathf.Abs(currentVolume), t);
                }
            }
        }
        else
        {
            foreach (var tri in trussData.triangles)
            {
                if (tri.currentStress > plasticityThreshold)
                {
                    float currentArea = CalculateTriangleArea(
                        trussData.nodes[tri.nodes[0]].worldPosition,
                        trussData.nodes[tri.nodes[1]].worldPosition,
                        trussData.nodes[tri.nodes[2]].worldPosition
                    );
                    
                    float t = (tri.currentStress - plasticityThreshold) * plasticityAmount * Time.fixedDeltaTime;
                    tri.restArea = Mathf.Lerp(tri.restArea, currentArea, t);
                }
            }
        }
        
        foreach (var beam in trussData.beams)
        {
            float currentLength = (trussData.nodes[beam.nodeA].worldPosition - trussData.nodes[beam.nodeB].worldPosition).magnitude;
            float strain = Mathf.Abs(currentLength - beam.restLength) / beam.restLength;
            
            if (strain > plasticityThreshold)
            {
                float t = (strain - plasticityThreshold) * plasticityAmount * Time.fixedDeltaTime;
                beam.restLength = Mathf.Lerp(beam.restLength, currentLength, t);
            }
        }
    }
    
    #endregion
    
    #region Visual Mesh Update
    
    private void UpdateVisualMesh()
    {
        if (deformedMesh == null || originalMeshVertices == null) return;
        
        Vector3[] newVertices = new Vector3[originalMeshVertices.Length];
        
        for (int i = 0; i < originalMeshVertices.Length; i++)
        {
            if (vertexToNode.TryGetValue(i, out int nodeIndex) && nodeIndex < trussData.nodes.Count)
            {
                newVertices[i] = transform.InverseTransformPoint(trussData.nodes[nodeIndex].worldPosition);
            }
            else
            {
                newVertices[i] = originalMeshVertices[i];
            }
        }
        
        deformedMesh.vertices = newVertices;
        deformedMesh.RecalculateNormals();
        deformedMesh.RecalculateBounds();
    }
    
    #endregion
    
    #region Public API
    
    public void PinNodeAt(Vector3 worldPosition, float radius = 0.1f)
    {
        if (trussData == null) return;
        
        float radiusSqr = radius * radius;
        foreach (var node in trussData.nodes)
        {
            if ((node.worldPosition - worldPosition).sqrMagnitude < radiusSqr)
            {
                node.isPinned = true;
            }
        }
    }
    
    public void UnpinAllNodes()
    {
        if (trussData == null) return;
        
        foreach (var node in trussData.nodes)
            node.isPinned = false;
    }
    
    public void ApplyImpulse(Vector3 worldPosition, Vector3 impulse, float radius = 0.5f)
    {
        if (trussData == null) return;
        
        float radiusSqr = radius * radius;
        foreach (var node in trussData.nodes)
        {
            if (node.isPinned || node.inverseMass <= 0f)
                continue;
            
            float distSqr = (node.worldPosition - worldPosition).sqrMagnitude;
            if (distSqr < radiusSqr)
            {
                float falloff = 1f - (distSqr / radiusSqr);
                node.oldPosition -= impulse * falloff * node.inverseMass;
            }
        }
    }
    
    public void ResetDeformation()
    {
        if (trussData == null || originalMeshVertices == null) return;
        
        foreach (var node in trussData.nodes)
        {
            if (node.originalVertexIndex >= 0 && node.originalVertexIndex < originalMeshVertices.Length)
            {
                node.worldPosition = transform.TransformPoint(originalMeshVertices[node.originalVertexIndex]);
                node.oldPosition = node.worldPosition;
            }
        }
        
        // Reset rest lengths
        foreach (var beam in trussData.beams)
        {
            beam.restLength = (trussData.nodes[beam.nodeA].worldPosition - trussData.nodes[beam.nodeB].worldPosition).magnitude;
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
            
            trussData.beams.Add(new SoftBodyTruss.TrussBeam(beam.nodeA, newIndex, lengthA));
            trussData.beams.Add(new SoftBodyTruss.TrussBeam(newIndex, beam.nodeB, lengthB));
        }
        
        trussData.beams.RemoveAll(b => selectedBeams.Contains(b));
        
        MarkTrussDirty();
    }
    
    #endregion
    
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
    
    #region Debug Gizmos
    
    private void OnDrawGizmosSelected()
    {
        if (!showDebugGizmos || trussData == null || trussData.nodes.Count == 0)
            return;
        
        // Draw nodes in local space (editor) or world space (runtime)
        foreach (var node in trussData.nodes)
        {
            Vector3 pos = Application.isPlaying ? node.worldPosition : transform.TransformPoint(node.localPosition);
            
            Gizmos.color = node.isPinned ? Color.red : (node.isSelected ? Color.yellow : Color.cyan);
            float size = node.isSelected ? 0.04f : 0.02f;
            Gizmos.DrawWireSphere(pos, size);
        }
        
        // Draw beams
        foreach (var beam in trussData.beams)
        {
            if (beam.nodeA >= trussData.nodes.Count || beam.nodeB >= trussData.nodes.Count)
                continue;
            
            Vector3 posA = Application.isPlaying 
                ? trussData.nodes[beam.nodeA].worldPosition 
                : transform.TransformPoint(trussData.nodes[beam.nodeA].localPosition);
            Vector3 posB = Application.isPlaying 
                ? trussData.nodes[beam.nodeB].worldPosition 
                : transform.TransformPoint(trussData.nodes[beam.nodeB].localPosition);
            
            Gizmos.color = beam.isSelected ? Color.green : Color.yellow;
            Gizmos.DrawLine(posA, posB);
        }
    }
    
    #endregion
}
