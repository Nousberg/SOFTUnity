using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

/// <summary>
/// Custom editor for AdvancedVolumetricSoftBody with scene handles for node/beam manipulation.
/// Works with SoftBodyTruss ScriptableObject for data persistence.
/// </summary>
[CustomEditor(typeof(AdvancedVolumetricSoftBody))]
public class AdvancedVolumetricSoftBodyEditor : Editor
{
    private AdvancedVolumetricSoftBody softBody;
    private SoftBodyTruss truss;
    
    // Selection state
    private bool isRegionSelecting = false;
    private Vector2 selectionStart;
    private Rect selectionRect;
    
    // Colors
    private static readonly Color nodeColor = new Color(0.2f, 0.8f, 1f, 1f);
    private static readonly Color selectedNodeColor = Color.yellow;
    private static readonly Color pinnedNodeColor = Color.red;
    private static readonly Color beamColor = new Color(1f, 0.8f, 0.2f, 0.8f);
    private static readonly Color selectedBeamColor = Color.green;
    
    private void OnEnable()
    {
        softBody = (AdvancedVolumetricSoftBody)target;
        Tools.hidden = true;
    }
    
    private void OnDisable()
    {
        Tools.hidden = false;
    }
    
    /// <summary>
    /// Record undo only in editor mode, not during play mode
    /// </summary>
    private void RecordUndoIfNotPlaying(string name)
    {
        if (!Application.isPlaying && truss != null)
        {
            Undo.RecordObject(truss, name);
        }
    }
    
    /// <summary>
    /// Mark truss dirty only in editor mode, not during play mode
    /// </summary>
    private void MarkDirtyIfNotPlaying()
    {
        if (!Application.isPlaying && truss != null)
        {
            EditorUtility.SetDirty(truss);
        }
    }
    
    private void OnSceneGUI()
    {
        if (softBody == null)
            return;
        
        truss = softBody.trussData;
        if (truss == null || truss.nodes == null)
            return;
        
        int controlID = GUIUtility.GetControlID(FocusType.Passive);
        
        HandleKeyboardInput();
        DrawNodes();
        DrawBeams();
        HandleSelection(controlID);
        DrawMoveHandles();
        
        if (isRegionSelecting)
        {
            DrawSelectionRectangle();
        }
    }
    
    private void HandleKeyboardInput()
    {
        Event e = Event.current;
        
        if (e.type == EventType.KeyDown)
        {
            switch (e.keyCode)
            {
                case KeyCode.Delete:
                case KeyCode.Backspace:
                    DeleteSelection();
                    e.Use();
                    break;
                    
                case KeyCode.Escape:
                    DeselectAll();
                    e.Use();
                    break;
            }
        }
    }
    
    private void DrawNodes()
    {
        for (int i = 0; i < truss.nodes.Count; i++)
        {
            var node = truss.nodes[i];
            
            // Use local position in editor, world position in runtime
            Vector3 pos = Application.isPlaying 
                ? node.worldPosition 
                : softBody.transform.TransformPoint(node.localPosition);
            
            float size = HandleUtility.GetHandleSize(pos) * 0.08f;
            
            Color color = nodeColor;
            if (node.isPinned) color = pinnedNodeColor;
            else if (node.isSelected) color = selectedNodeColor;
            
            Handles.color = color;
            
            if (Handles.Button(pos, Quaternion.identity, size, size * 1.2f, Handles.SphereHandleCap))
            {
                HandleNodeClick(i);
            }
        }
    }
    
    private void DrawBeams()
    {
        foreach (var beam in truss.beams)
        {
            if (beam.nodeA >= truss.nodes.Count || beam.nodeB >= truss.nodes.Count)
                continue;
            
            Vector3 posA, posB;
            
            if (Application.isPlaying)
            {
                posA = truss.nodes[beam.nodeA].worldPosition;
                posB = truss.nodes[beam.nodeB].worldPosition;
            }
            else
            {
                posA = softBody.transform.TransformPoint(truss.nodes[beam.nodeA].localPosition);
                posB = softBody.transform.TransformPoint(truss.nodes[beam.nodeB].localPosition);
            }
            
            Handles.color = beam.isSelected ? selectedBeamColor : beamColor;
            Handles.DrawLine(posA, posB, beam.isSelected ? 3f : 1.5f);
        }
    }
    
    private void HandleNodeClick(int nodeIndex)
    {
        Event e = Event.current;
        RecordUndoIfNotPlaying("Select Node");
        
        // Check if any beams are selected - if so, deselect them
        bool hadBeamsSelected = truss.HasSelectedBeams();
        if (hadBeamsSelected)
        {
            truss.DeselectAllBeams();
        }
        
        if (e.control || e.command)
        {
            truss.nodes[nodeIndex].isSelected = !truss.nodes[nodeIndex].isSelected;
        }
        else if (e.shift)
        {
            truss.nodes[nodeIndex].isSelected = true;
        }
        else
        {
            truss.DeselectAllNodes();
            truss.nodes[nodeIndex].isSelected = true;
        }
        
        UpdateSelectedPointSettings();
        MarkDirtyIfNotPlaying();
    }
    
    private void HandleSelection(int controlID)
    {
        Event e = Event.current;
        
        switch (e.type)
        {
            case EventType.MouseDown:
                if (e.button == 0 && !e.alt && HandleUtility.nearestControl == controlID)
                {
                    if (TrySelectBeam(e))
                    {
                        e.Use();
                        return;
                    }
                    
                    selectionStart = e.mousePosition;
                    selectionRect = new Rect(selectionStart, Vector2.zero);
                    isRegionSelecting = true;
                    e.Use();
                }
                break;
                
            case EventType.MouseDrag:
                if (isRegionSelecting)
                {
                    selectionRect.xMin = Mathf.Min(selectionStart.x, e.mousePosition.x);
                    selectionRect.xMax = Mathf.Max(selectionStart.x, e.mousePosition.x);
                    selectionRect.yMin = Mathf.Min(selectionStart.y, e.mousePosition.y);
                    selectionRect.yMax = Mathf.Max(selectionStart.y, e.mousePosition.y);
                    HandleUtility.Repaint();
                    e.Use();
                }
                break;
                
            case EventType.MouseUp:
                if (isRegionSelecting)
                {
                    PerformRegionSelection();
                    isRegionSelecting = false;
                    e.Use();
                }
                break;
                
            case EventType.Layout:
                HandleUtility.AddDefaultControl(controlID);
                break;
        }
    }
    
    private bool TrySelectBeam(Event e)
    {
        float minDist = 5f;
        int closestBeamIndex = -1;
        
        for (int i = 0; i < truss.beams.Count; i++)
        {
            var beam = truss.beams[i];
            if (beam.nodeA >= truss.nodes.Count || beam.nodeB >= truss.nodes.Count)
                continue;
            
            Vector3 posA, posB;
            if (Application.isPlaying)
            {
                posA = truss.nodes[beam.nodeA].worldPosition;
                posB = truss.nodes[beam.nodeB].worldPosition;
            }
            else
            {
                posA = softBody.transform.TransformPoint(truss.nodes[beam.nodeA].localPosition);
                posB = softBody.transform.TransformPoint(truss.nodes[beam.nodeB].localPosition);
            }
            
            Vector2 screenA = HandleUtility.WorldToGUIPoint(posA);
            Vector2 screenB = HandleUtility.WorldToGUIPoint(posB);
            
            float dist = HandleUtility.DistancePointToLineSegment(e.mousePosition, screenA, screenB);
            
            if (dist < minDist)
            {
                minDist = dist;
                closestBeamIndex = i;
            }
        }
        
        if (closestBeamIndex >= 0)
        {
            RecordUndoIfNotPlaying("Select Beam");
            
            truss.DeselectAllNodes();
            
            if (e.control || e.command)
            {
                truss.beams[closestBeamIndex].isSelected = !truss.beams[closestBeamIndex].isSelected;
            }
            else if (e.shift)
            {
                truss.beams[closestBeamIndex].isSelected = true;
            }
            else
            {
                truss.DeselectAllBeams();
                truss.beams[closestBeamIndex].isSelected = true;
            }
            
            UpdateSelectedBeamSettings();
            MarkDirtyIfNotPlaying();
            return true;
        }
        
        return false;
    }
    
    private void PerformRegionSelection()
    {
        if (selectionRect.width < 5 && selectionRect.height < 5)
        {
            if (!Event.current.control && !Event.current.shift)
            {
                RecordUndoIfNotPlaying("Deselect All");
                truss.DeselectAllNodes();
                truss.DeselectAllBeams();
                MarkDirtyIfNotPlaying();
            }
            return;
        }
        
        RecordUndoIfNotPlaying("Region Select");
        
        truss.DeselectAllBeams();
        
        Event e = Event.current;
        if (!e.control && !e.shift)
        {
            truss.DeselectAllNodes();
        }
        
        for (int i = 0; i < truss.nodes.Count; i++)
        {
            var node = truss.nodes[i];
            
            Vector3 pos = Application.isPlaying 
                ? node.worldPosition 
                : softBody.transform.TransformPoint(node.localPosition);
            
            Vector2 screenPos = HandleUtility.WorldToGUIPoint(pos);
            
            if (selectionRect.Contains(screenPos))
            {
                if (e.control)
                    node.isSelected = !node.isSelected;
                else
                    node.isSelected = true;
            }
        }
        
        UpdateSelectedPointSettings();
        MarkDirtyIfNotPlaying();
    }
    
    private void DrawMoveHandles()
    {
        var selectedIndices = truss.GetSelectedNodeIndices();
        if (selectedIndices.Count == 0) return;
        
        Vector3 center = Vector3.zero;
        foreach (int i in selectedIndices)
        {
            Vector3 pos = Application.isPlaying 
                ? truss.nodes[i].worldPosition 
                : softBody.transform.TransformPoint(truss.nodes[i].localPosition);
            center += pos;
        }
        center /= selectedIndices.Count;
        
        EditorGUI.BeginChangeCheck();
        Vector3 newCenter = Handles.PositionHandle(center, Quaternion.identity);
        
        if (EditorGUI.EndChangeCheck())
        {
            RecordUndoIfNotPlaying("Move Nodes");
            Vector3 delta = newCenter - center;
            
            foreach (int i in selectedIndices)
            {
                if (Application.isPlaying)
                {
                    // Runtime: only update world positions, don't save to SO
                    truss.nodes[i].worldPosition += delta;
                    truss.nodes[i].oldPosition += delta;
                }
                else
                {
                    // Editor: move in local space and save
                    Vector3 localDelta = softBody.transform.InverseTransformVector(delta);
                    truss.nodes[i].localPosition += localDelta;
                }
            }
            
            MarkDirtyIfNotPlaying();
        }
    }
    
    private void DrawSelectionRectangle()
    {
        Handles.BeginGUI();
        
        Color fillColor = new Color(0.2f, 0.5f, 1f, 0.1f);
        Color borderColor = new Color(0.2f, 0.5f, 1f, 0.8f);
        
        EditorGUI.DrawRect(selectionRect, fillColor);
        
        Handles.color = borderColor;
        Vector3[] corners = new Vector3[]
        {
            new Vector3(selectionRect.xMin, selectionRect.yMin, 0),
            new Vector3(selectionRect.xMax, selectionRect.yMin, 0),
            new Vector3(selectionRect.xMax, selectionRect.yMax, 0),
            new Vector3(selectionRect.xMin, selectionRect.yMax, 0),
            new Vector3(selectionRect.xMin, selectionRect.yMin, 0)
        };
        Handles.DrawPolyLine(corners);
        
        Handles.EndGUI();
    }
    
    private void DeleteSelection()
    {
        // Don't allow deletion during play mode
        if (Application.isPlaying)
        {
            Debug.LogWarning("Cannot delete nodes/beams during play mode");
            return;
        }
        
        RecordUndoIfNotPlaying("Delete Selection");
        
        truss.RemoveSelectedBeams();
        truss.RemoveSelectedNodes();
        
        MarkDirtyIfNotPlaying();
    }
    
    private void DeselectAll()
    {
        RecordUndoIfNotPlaying("Deselect All");
        truss.DeselectAllNodes();
        truss.DeselectAllBeams();
        MarkDirtyIfNotPlaying();
    }
    
    private void UpdateSelectedPointSettings()
    {
        var selectedIndices = truss.GetSelectedNodeIndices();
        if (selectedIndices.Count > 0)
        {
            var firstNode = truss.nodes[selectedIndices[0]];
            softBody.selectedPointSettings.inverseMass = firstNode.inverseMass;
            softBody.selectedPointSettings.isPinned = firstNode.isPinned;
        }
    }
    
    private void UpdateSelectedBeamSettings()
    {
        foreach (var beam in truss.beams)
        {
            if (beam.isSelected)
            {
                softBody.selectedBeamSettings.restLength = beam.restLength;
                softBody.selectedBeamSettings.stiffnessOverride = beam.stiffnessOverride;
                break;
            }
        }
    }
    
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        truss = softBody.trussData;
        
        // Header
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Soft Body Designer", EditorStyles.boldLabel);
        EditorGUILayout.Space();
        
        // Truss data reference
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUI.BeginChangeCheck();
        softBody.trussData = (SoftBodyTruss)EditorGUILayout.ObjectField(
            "Truss Data", softBody.trussData, typeof(SoftBodyTruss), false);
        if (EditorGUI.EndChangeCheck())
        {
            EditorUtility.SetDirty(softBody);
            truss = softBody.trussData;
        }
        EditorGUILayout.EndVertical();
        
        if (truss == null)
        {
            EditorGUILayout.HelpBox("Assign a SoftBodyTruss asset to begin editing.\nCreate one via: Assets → Create → Physics → Soft Body Truss", MessageType.Warning);
            return;
        }
        
        // Tab Toolbar
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        string[] tabs = { "Truss", "Nodes", "Beams", "Sets" };
        currentTab = GUILayout.Toolbar(currentTab, tabs, GUILayout.Height(25));
        EditorGUILayout.EndVertical();
        
        // Draw current tab
        switch (currentTab)
        {
            case TAB_TRUSS: DrawTrussTab(); break;
            case TAB_NODES: DrawNodesTab(); break;
            case TAB_BEAMS: DrawBeamsTab(); break;
            case TAB_SETS: DrawNodeSetsTab(); break;
        }
        
        if (GUI.changed)
        {
            serializedObject.ApplyModifiedProperties();
            SceneView.RepaintAll();
        }
    }
    
    #region Tab Constants
    private const int TAB_TRUSS = 0;
    private const int TAB_NODES = 1;
    private const int TAB_BEAMS = 2;
    private const int TAB_SETS = 3;
    private int currentTab = 0;
    private string newNodeSetName = "";
    #endregion
    
    #region Tab Drawing Methods
    
    private void DrawTrussTab()
    {
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Truss Information", EditorStyles.boldLabel);
        EditorGUILayout.Separator();
        
        GUI.enabled = false;
        EditorGUILayout.IntField("Nodes", truss.nodes?.Count ?? 0);
        EditorGUILayout.IntField("Beams", truss.beams?.Count ?? 0);
        EditorGUILayout.IntField("Tetrahedra", truss.tetrahedra?.Count ?? 0);
        EditorGUILayout.IntField("Triangles", truss.triangles?.Count ?? 0);
        GUI.enabled = true;
        
        EditorGUILayout.EndVertical();
        
        // Danger Zone
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Danger Zone", EditorStyles.boldLabel);
        
        EditorGUILayout.BeginHorizontal();
        GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
        if (GUILayout.Button("Clear Beams", GUILayout.Height(25)))
        {
            if (EditorUtility.DisplayDialog("Clear Beams", "Remove all beams? This cannot be undone.", "Clear", "Cancel"))
            {
                softBody.EditorClearBeams();
            }
        }
        if (GUILayout.Button("Clear Grid", GUILayout.Height(25)))
        {
            if (EditorUtility.DisplayDialog("Clear Grid", "Remove all nodes, beams, and elements? This cannot be undone.", "Clear", "Cancel"))
            {
                softBody.EditorClearGrid();
            }
        }
        GUI.backgroundColor = Color.white;
        EditorGUILayout.EndHorizontal();
        
        EditorGUILayout.EndVertical();
        
        // Generation
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Auto Generation", EditorStyles.boldLabel);
        
        MeshFilter meshFilter = softBody.GetComponent<MeshFilter>();
        if (meshFilter != null && meshFilter.sharedMesh != null)
        {
            EditorGUILayout.LabelField($"Source Mesh: {meshFilter.sharedMesh.name}");
            EditorGUILayout.LabelField($"Vertices: {meshFilter.sharedMesh.vertexCount}");
            
            EditorGUILayout.Space();
            softBody.generateTetrahedra = EditorGUILayout.Toggle("Use Tetrahedra", softBody.generateTetrahedra);
            if (softBody.generateTetrahedra)
            {
                softBody.internalPointCount = EditorGUILayout.IntSlider("Internal Points", softBody.internalPointCount, 0, 100);
                softBody.pointSpacing = EditorGUILayout.Slider("Point Spacing", softBody.pointSpacing, 0.05f, 0.5f);
            }
            
            EditorGUILayout.Space();
            GUI.backgroundColor = new Color(0.5f, 1f, 0.5f);
            if (GUILayout.Button("🔧 Generate from Mesh", GUILayout.Height(30)))
            {
                if (EditorUtility.DisplayDialog("Generate Truss from Mesh",
                    "This will create nodes and beams from the mesh.\nExisting data will NOT be cleared.\nContinue?",
                    "Generate", "Cancel"))
                {
                    softBody.EditorGenerateFromMesh();
                }
            }
            GUI.backgroundColor = Color.white;
        }
        else
        {
            EditorGUILayout.HelpBox("No MeshFilter found on this object.", MessageType.Warning);
        }
        
        EditorGUILayout.EndVertical();
    }
    
    private void DrawNodesTab()
    {
        int count = truss.SelectedNodeCount();
        
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        
        string selInfo = count == 0 ? "No Nodes Selected" : $"{count} Nodes Selected";
        var style = new GUIStyle(EditorStyles.boldLabel) { alignment = TextAnchor.MiddleRight };
        EditorGUILayout.LabelField(selInfo, style);
        
        if (count > 0)
        {
            EditorGUILayout.Separator();
            
            // Edit selected nodes
            softBody.selectedPointSettings.inverseMass = EditorGUILayout.FloatField("Inverse Mass", softBody.selectedPointSettings.inverseMass);
            softBody.selectedPointSettings.isPinned = EditorGUILayout.Toggle("Is Pinned", softBody.selectedPointSettings.isPinned);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Apply to Selected"))
            {
                softBody.ApplyPointSettings();
            }
            EditorGUILayout.EndHorizontal();
        }
        else
        {
            EditorGUILayout.HelpBox("Select nodes in Scene View to edit.", MessageType.Info);
        }
        
        EditorGUILayout.EndVertical();
        
        EditorGUILayout.Space();
        
        // Node tools
        EditorGUILayout.BeginHorizontal();
        if (GUILayout.Button("New Point"))
        {
            softBody.EditorCreatePoint();
        }
        GUI.enabled = count > 0;
        if (GUILayout.Button("Delete"))
        {
            DeleteSelection();
        }
        GUI.enabled = true;
        if (GUILayout.Button("Select All"))
        {
            RecordUndoIfNotPlaying("Select All Nodes");
            for (int i = 0; i < truss.nodes.Count; i++)
                truss.nodes[i].isSelected = true;
            MarkDirtyIfNotPlaying();
        }
        if (GUILayout.Button("Clear"))
        {
            DeselectAll();
        }
        EditorGUILayout.EndHorizontal();
        
        // New Point distance
        softBody.newPointDistance = EditorGUILayout.Slider("New Point Distance", softBody.newPointDistance, 1f, 20f);
    }
    
    private void DrawBeamsTab()
    {
        int count = 0;
        foreach (var beam in truss.beams)
            if (beam.isSelected) count++;
        
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        
        string selInfo = count == 0 ? "No Beams Selected" : $"{count} Beams Selected";
        var style = new GUIStyle(EditorStyles.boldLabel) { alignment = TextAnchor.MiddleRight };
        EditorGUILayout.LabelField(selInfo, style);
        
        if (count > 0)
        {
            EditorGUILayout.Separator();
            
            // Edit selected beams
            softBody.selectedBeamSettings.restLength = EditorGUILayout.FloatField("Rest Length", softBody.selectedBeamSettings.restLength);
            softBody.selectedBeamSettings.stiffnessOverride = EditorGUILayout.Slider("Stiffness Override", softBody.selectedBeamSettings.stiffnessOverride, -1f, 1f);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Apply to Selected"))
            {
                softBody.ApplyBeamSettings();
            }
            if (GUILayout.Button("Create Midpoints"))
            {
                softBody.CreateMidpointOnSelectedBeams();
            }
            EditorGUILayout.EndHorizontal();
        }
        else
        {
            EditorGUILayout.HelpBox("Select beams in Scene View to edit.", MessageType.Info);
        }
        
        EditorGUILayout.EndVertical();
        
        EditorGUILayout.Space();
        
        // Beam tools
        EditorGUILayout.BeginHorizontal();
        GUI.enabled = truss.SelectedNodeCount() >= 2;
        if (GUILayout.Button("New Beam(s)"))
        {
            softBody.EditorCreateBeams();
        }
        GUI.enabled = count > 0;
        if (GUILayout.Button("Delete"))
        {
            RecordUndoIfNotPlaying("Delete Beams");
            truss.RemoveSelectedBeams();
            MarkDirtyIfNotPlaying();
        }
        GUI.enabled = true;
        if (GUILayout.Button("Select All"))
        {
            RecordUndoIfNotPlaying("Select All Beams");
            foreach (var beam in truss.beams)
                beam.isSelected = true;
            MarkDirtyIfNotPlaying();
        }
        if (GUILayout.Button("Clear"))
        {
            RecordUndoIfNotPlaying("Deselect Beams");
            truss.DeselectAllBeams();
            MarkDirtyIfNotPlaying();
        }
        EditorGUILayout.EndHorizontal();
        
        // Physics settings
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Physics Settings", EditorStyles.boldLabel);
        softBody.distanceStiffness = EditorGUILayout.Slider("Distance Stiffness", softBody.distanceStiffness, 0f, 1f);
        softBody.volumeStiffness = EditorGUILayout.Slider("Volume Stiffness", softBody.volumeStiffness, 0f, 1f);
        softBody.damping = EditorGUILayout.Slider("Damping", softBody.damping, 0f, 1f);
        softBody.solverIterations = EditorGUILayout.IntSlider("Solver Iterations", softBody.solverIterations, 1, 50);
        EditorGUILayout.EndVertical();
    }
    
    private void DrawNodeSetsTab()
    {
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Node Sets", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox("Node sets are named groups of nodes used for constraints.", MessageType.Info);
        EditorGUILayout.EndVertical();
        
        EditorGUILayout.Space();
        
        // Create from selection
        int selectedCount = truss.SelectedNodeCount();
        
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Create From Selection", EditorStyles.boldLabel);
        
        EditorGUILayout.BeginHorizontal();
        newNodeSetName = EditorGUILayout.TextField("Name", newNodeSetName);
        GUI.enabled = selectedCount > 0 && !string.IsNullOrEmpty(newNodeSetName);
        if (GUILayout.Button("Create", GUILayout.Width(60)))
        {
            RecordUndoIfNotPlaying("Create Node Set");
            truss.AddNodeSet(newNodeSetName, truss.GetSelectedNodeIndices());
            MarkDirtyIfNotPlaying();
            newNodeSetName = "";
        }
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();
        
        if (selectedCount > 0)
        {
            EditorGUILayout.LabelField($"Selected: {selectedCount} nodes");
        }
        else
        {
            EditorGUILayout.LabelField("Select nodes in Scene view first", EditorStyles.miniLabel);
        }
        
        EditorGUILayout.EndVertical();
        
        EditorGUILayout.Space();
        
        // Existing sets
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField($"Existing Sets ({truss.nodeSets.Count})", EditorStyles.boldLabel);
        
        if (truss.nodeSets.Count == 0)
        {
            EditorGUILayout.LabelField("No node sets defined", EditorStyles.miniLabel);
        }
        else
        {
            for (int i = 0; i < truss.nodeSets.Count; i++)
            {
                var nodeSet = truss.nodeSets[i];
                
                EditorGUILayout.BeginHorizontal();
                
                EditorGUILayout.LabelField(nodeSet.name, GUILayout.Width(100));
                EditorGUILayout.LabelField($"({nodeSet.nodeIndices.Count} nodes)", EditorStyles.miniLabel);
                
                if (GUILayout.Button("Select", EditorStyles.miniButton, GUILayout.Width(50)))
                {
                    RecordUndoIfNotPlaying("Select Node Set");
                    truss.DeselectAllNodes();
                    foreach (int idx in nodeSet.nodeIndices)
                    {
                        if (idx >= 0 && idx < truss.nodes.Count)
                            truss.nodes[idx].isSelected = true;
                    }
                    MarkDirtyIfNotPlaying();
                }
                
                GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
                if (GUILayout.Button("×", EditorStyles.miniButton, GUILayout.Width(20)))
                {
                    RecordUndoIfNotPlaying("Remove Node Set");
                    truss.nodeSets.RemoveAt(i);
                    MarkDirtyIfNotPlaying();
                    break;
                }
                GUI.backgroundColor = Color.white;
                
                EditorGUILayout.EndHorizontal();
            }
        }
        
        EditorGUILayout.EndVertical();
    }
    
    #endregion
}

