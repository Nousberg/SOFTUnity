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
    private static readonly Color selectedNodeColor = Color.green;
    private static readonly Color pinnedNodeColor = Color.red;
    private static readonly Color beamColor = new Color(1f, 0.8f, 0.2f, 0.8f);
    private static readonly Color selectedBeamColor = Color.green;

    // Node Creation State
    private bool isCreatingNode = false;
    private Vector3 ghostNodePosition;
    private bool showGhostNode = false;

    // Handle state for continuous tools (Rotation/Scale)
    private static Quaternion s_LastCmdRotation = Quaternion.identity;
    private static Vector3 s_LastCmdScale = Vector3.one;
    
    private void OnEnable()
    {
        softBody = (AdvancedVolumetricSoftBody)target;
        Tools.hidden = true;
    }
    
    private void OnDisable()
    {
        SetEditorCollider(false);
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

    private void SetEditorCollider(bool active)
    {
        if (softBody == null) return;
        
        MeshCollider col = softBody.GetComponent<MeshCollider>();

        if (active)
        {
            if (col == null)
            {
                // We use AddComponent; Unity will handle the refresh.
                col = softBody.gameObject.AddComponent<MeshCollider>();

                // Try to grab the mesh so the collider actually works for raycasting
                MeshFilter mf = softBody.GetComponent<MeshFilter>();
                if (mf != null) col.sharedMesh = mf.sharedMesh;
                
                // Hide it so it doesn't clutter inspector too much or look like part of the setup
                col.hideFlags = HideFlags.DontSave | HideFlags.HideInInspector; 
            }
        }
        else
        {
            if (col != null)
            {
                // Use DestroyImmediate for Editor-time removal
                // Only destroy if it seems to be our temporary one (e.g. check hideFlags or just assume)
                // Designer logic just destroys it.
                if ((col.hideFlags & HideFlags.DontSave) != 0)
                    DestroyImmediate(col);
            }
        }
    }
    

    private void OnSceneGUI()
    {
        if (softBody == null)
            return;
        
        // Use RuntimeTruss to get correct positions at runtime
        truss = softBody.RuntimeTruss;
        if (truss == null || truss.nodes == null)
            return;
        
        int controlID = GUIUtility.GetControlID(FocusType.Passive);
        
        // Handle node creation input first
        if (isCreatingNode)
        {
            HandleUtility.AddDefaultControl(controlID);
            
            Event e = Event.current;
            
            // Always try to get position to update visibility
            bool isValidPos = TryGetNodeCreationPosition(e.mousePosition, out Vector3 worldPos);
            
            if (isValidPos)
            {
                ghostNodePosition = worldPos;
                showGhostNode = true;
            }
            else
            {
                showGhostNode = false;
            }

            if (e.type == EventType.MouseMove)
            {
                e.Use();
            }
            else if (e.type == EventType.MouseDown && e.button == 0 && isValidPos)
            {
                CreateNodeAtPosition(ghostNodePosition);
                e.Use();
            }
            else if (e.type == EventType.KeyDown && e.keyCode == KeyCode.Escape)
            {
                isCreatingNode = false;
                showGhostNode = false;
                SetEditorCollider(false);
                e.Use();
            }
            
            if (showGhostNode)
            {
                DrawGhostNode(ghostNodePosition);
            }
            
            // Force continuous repaint for smooth pulse animation and responsive updates
            SceneView.RepaintAll();
            
            // Draw existing nodes/beams so we can see where we are placing
            DrawNodes();
            DrawBeams();
            
            // If in creation mode, don't process other scene GUI interactions (selection, etc.)
            return;
        }

        HandleKeyboardInput();
        DrawNodes();
        DrawBeams();
        HandleSelection(controlID);
        DrawMoveHandles();
        DrawSelectionLabels(); // Stats text above selected nodes/beams
        
        if (isRegionSelecting)
        {
            DrawSelectionRectangle();
        }
        
        // Force repaint for pulsating animation
        if (truss.SelectedNodeCount() > 0)
        {
            HandleUtility.Repaint();
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
            
            float baseSize = HandleUtility.GetHandleSize(pos) * 0.06f;
            
            // Pulsating size for selected nodes
            float size = baseSize;
            if (node.isSelected)
            {
                float pulse = Mathf.Sin((float)EditorApplication.timeSinceStartup * 6f) * 0.5f + 1.5f;
                size = baseSize * pulse;
            }
            
            Color color = nodeColor;
            if (node.isPinned) color = pinnedNodeColor;
            else if (node.isSelected) color = selectedNodeColor;
            
            // Draw solid sphere
            Handles.color = color;
            Handles.SphereHandleCap(0, pos, Quaternion.identity, size * 2f, EventType.Repaint);
            
            // Invisible button for click detection (smaller for precision)
            Handles.color = Color.clear;
            if (Handles.Button(pos, Quaternion.identity, size, size * 1.5f, Handles.SphereHandleCap))
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
        Repaint();
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
            Repaint();
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
                Repaint();
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
        Repaint();
    }
    
    private void DrawMoveHandles()
    {
        var selectedIndices = truss.GetSelectedNodeIndices();
        if (selectedIndices.Count == 0) return;
        
        // Reset handle state if not interacting
        if (GUIUtility.hotControl == 0)
        {
            s_LastCmdRotation = Quaternion.identity;
            s_LastCmdScale = Vector3.one;
        }
        
        // Calculate center of selection
        Vector3 center = Vector3.zero;
        foreach (int i in selectedIndices)
        {
            Vector3 pos = Application.isPlaying 
                ? truss.nodes[i].worldPosition 
                : softBody.transform.TransformPoint(truss.nodes[i].localPosition);
            center += pos;
        }
        center /= selectedIndices.Count;
        
        // Handle based on current tool
        switch (Tools.current)
        {
            case Tool.Move:
                HandleMoveTool(selectedIndices, center);
                break;
            case Tool.Rotate:
                HandleRotateTool(selectedIndices, center);
                break;
            case Tool.Scale:
                HandleScaleTool(selectedIndices, center);
                break;
        }
    }
    
    private void HandleMoveTool(List<int> selectedIndices, Vector3 center)
    {
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
                    truss.nodes[i].worldPosition += delta;
                    truss.nodes[i].oldPosition += delta;
                }
                else
                {
                    Vector3 localDelta = softBody.transform.InverseTransformVector(delta);
                    truss.nodes[i].localPosition += localDelta;
                }
            }
            
            MarkDirtyIfNotPlaying();
        }
    }
    
    private void HandleRotateTool(List<int> selectedIndices, Vector3 center)
    {
        if (selectedIndices.Count < 2) return; // Need at least 2 nodes to rotate
        
        EditorGUI.BeginChangeCheck();
        
        // Track cumulative rotation in the handle, but apply delta to object
        Quaternion newRotation = Handles.RotationHandle(s_LastCmdRotation, center);
        
        if (EditorGUI.EndChangeCheck())
        {
            RecordUndoIfNotPlaying("Rotate Nodes");
            
            // Calculate delta from last frame
            Quaternion delta = newRotation * Quaternion.Inverse(s_LastCmdRotation);
            s_LastCmdRotation = newRotation;
            
            foreach (int i in selectedIndices)
            {
                Vector3 pos = Application.isPlaying 
                    ? truss.nodes[i].worldPosition 
                    : softBody.transform.TransformPoint(truss.nodes[i].localPosition);
                
                Vector3 relativePos = pos - center;
                Vector3 rotatedPos = delta * relativePos; // Apply delta to current position relative Vector
                Vector3 newPos = center + rotatedPos;
                
                if (Application.isPlaying)
                {
                    truss.nodes[i].worldPosition = newPos;
                    truss.nodes[i].oldPosition = newPos;
                }
                else
                {
                    truss.nodes[i].localPosition = softBody.transform.InverseTransformPoint(newPos);
                }
            }
            
            MarkDirtyIfNotPlaying();
        }
    }
    
    private void HandleScaleTool(List<int> selectedIndices, Vector3 center)
    {
        if (selectedIndices.Count < 2) return; // Need at least 2 nodes to scale
        
        EditorGUI.BeginChangeCheck();
        
        // Track cumulative scale in the handle, but apply delta to object
        Vector3 newScale = Handles.ScaleHandle(s_LastCmdScale, center, Quaternion.identity, HandleUtility.GetHandleSize(center));
        
        if (EditorGUI.EndChangeCheck())
        {
            RecordUndoIfNotPlaying("Scale Nodes");
            
            // Calculate delta factor (safe division)
            Vector3 delta = new Vector3(
                s_LastCmdScale.x == 0 ? 1 : newScale.x / s_LastCmdScale.x,
                s_LastCmdScale.y == 0 ? 1 : newScale.y / s_LastCmdScale.y,
                s_LastCmdScale.z == 0 ? 1 : newScale.z / s_LastCmdScale.z
            );
            
            s_LastCmdScale = newScale;
            
            foreach (int i in selectedIndices)
            {
                Vector3 pos = Application.isPlaying 
                    ? truss.nodes[i].worldPosition 
                    : softBody.transform.TransformPoint(truss.nodes[i].localPosition);
                
                Vector3 relativePos = pos - center;
                Vector3 scaledPos = Vector3.Scale(relativePos, delta); // Apply delta factor
                Vector3 newPos = center + scaledPos;
                
                if (Application.isPlaying)
                {
                    truss.nodes[i].worldPosition = newPos;
                    truss.nodes[i].oldPosition = newPos;
                }
                else
                {
                    truss.nodes[i].localPosition = softBody.transform.InverseTransformPoint(newPos);
                }
            }
            
            MarkDirtyIfNotPlaying();
        }
    }
    
    private void DrawSelectionLabels()
    {
        // Create a style for labels
        GUIStyle labelStyle = new GUIStyle(EditorStyles.boldLabel)
        {
            alignment = TextAnchor.MiddleCenter,
            fontSize = 10,
            normal = { textColor = Color.white }
        };
        
        // Draw stats for all nodes (only on Nodes tab)
        if (currentTab == TAB_NODES)
        {
            for (int i = 0; i < truss.nodes.Count; i++)
            {
                var node = truss.nodes[i];
                
                Vector3 pos = Application.isPlaying 
                    ? node.worldPosition 
                    : softBody.transform.TransformPoint(node.localPosition);
                
                Vector3 labelPos = pos + Vector3.up * HandleUtility.GetHandleSize(pos) * 0.3f;
                
                // Build stats string
                string stats = $"#{i}";
                if (node.isPinned) stats += " [P]";
                stats += $"\nm:{(node.inverseMass > 0 ? (1f / node.inverseMass).ToString("F2") : "∞")}";
                
                Handles.Label(labelPos, stats, labelStyle);
            }
        }
        
        // Draw stats for beams (only on Beams tab)
        if (currentTab == TAB_BEAMS)
        {
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
                
                Vector3 mid = (posA + posB) * 0.5f;
                Vector3 labelPos = mid + Vector3.up * HandleUtility.GetHandleSize(mid) * 0.2f;
                
                float currentLength = Vector3.Distance(posA, posB);
                
                // Build stats string
                string stats = $"B#{i}\nL:{currentLength:F2}\nR:{beam.restLength:F2}";
                if (beam.stiffnessOverride >= 0) stats += $"\nS:{beam.stiffnessOverride:F1}";
                
                Handles.Label(labelPos, stats, labelStyle);
            }
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
        GUIStyle headerStyle = new GUIStyle(EditorStyles.boldLabel)
        {
            fontSize = 14,
            alignment = TextAnchor.MiddleCenter
        };
        EditorGUILayout.LabelField("Soft Body Designer", headerStyle);
        
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
        
        // Manager Warning (Between Truss Data and Tabs)
        if (SoftBodySimulationManager.Instance == null && FindObjectOfType<SoftBodySimulationManager>() == null)
        {
             EditorGUILayout.HelpBox("SoftBodySimulationManager not found in scene! Physics will not simulate without it.", MessageType.Error);
        }
        
        if (truss == null)
        {
            EditorGUILayout.HelpBox("Assign a SoftBodyTruss asset to begin editing.\nCreate one via: Assets → Create → Physics → Soft Body Truss", MessageType.Warning);
            return;
        }
        
        // Tab Toolbar
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        string[] tabs = { "Truss", "Nodes", "Beams", "Sets", "Settings" };
        currentTab = GUILayout.Toolbar(currentTab, tabs, GUILayout.Height(25));
        EditorGUILayout.EndVertical();
        
        // Draw current tab
        switch (currentTab)
        {
            case TAB_TRUSS: DrawTrussTab(); break;
            case TAB_NODES: DrawNodesTab(); break;
            case TAB_BEAMS: DrawBeamsTab(); break;
            case TAB_SETS: DrawNodeSetsTab(); break;
            case TAB_SETTINGS: DrawSettingsTab(); break;
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
    private const int TAB_SETTINGS = 4;
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
        
        // Disable danger buttons in Play Mode
        EditorGUI.BeginDisabledGroup(Application.isPlaying);
        
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
        
        EditorGUI.EndDisabledGroup(); 
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
            EditorGUI.BeginChangeCheck();
            softBody.GenerationResolution = EditorGUILayout.Slider("Resolution", softBody.GenerationResolution, 0.1f, 1f);
            if (EditorGUI.EndChangeCheck())
            {
                // Force repaint to update estimate
                SceneView.RepaintAll();
            }
            
            // Show estimated nodes
            int estNodes = softBody.GetEstimatedNodeCount();
            EditorGUILayout.LabelField($"Estimated Nodes: ~{estNodes}", EditorStyles.miniLabel);
            
            softBody.generateTetrahedra = EditorGUILayout.Toggle("Use Tetrahedra", softBody.generateTetrahedra);
            
            EditorGUI.BeginDisabledGroup(!softBody.generateTetrahedra);
            if (softBody.generateTetrahedra)
            {
                softBody.internalPointCount = EditorGUILayout.IntSlider("Internal Points", softBody.internalPointCount, 0, 100);
                softBody.pointSpacing = EditorGUILayout.Slider("Point Spacing", softBody.pointSpacing, 0.05f, 0.5f);
            }
            EditorGUI.EndDisabledGroup();
            
            EditorGUILayout.Space();
            
            // Runtime Reset Button
            EditorGUI.BeginDisabledGroup(!Application.isPlaying);
            GUI.backgroundColor = Application.isPlaying ? new Color(0.3f, 0.6f, 1f) : Color.white;
            if (GUILayout.Button(Application.isPlaying ? "↺ Reset Deformation" : "Reset Deformation (Runtime Only)", GUILayout.Height(25)))
            {
                if (Application.isPlaying) softBody.ResetDeformation();
            }
            GUI.backgroundColor = Color.white;
            EditorGUI.EndDisabledGroup();
            
            // Disable generation button in Play Mode or if generating
            EditorGUI.BeginDisabledGroup(Application.isPlaying || softBody.isGenerating);
            
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
            
            EditorGUI.EndDisabledGroup();
        }
        else
        {
            EditorGUILayout.HelpBox("No MeshFilter found on this object.", MessageType.Warning);
        }
        
        EditorGUILayout.EndVertical();
        
        // Linked Meshes
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        SerializedProperty linkedMeshesProp = serializedObject.FindProperty("linkedMeshes");
        if (linkedMeshesProp != null)
        {
            EditorGUILayout.PropertyField(linkedMeshesProp, true);
        }
        EditorGUILayout.EndVertical();
    }
    
    private void DrawNodesTab()
    {
        int count = truss.SelectedNodeCount();
        
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        
        string selInfo = count == 0 ? "No Nodes Selected" : $"{count} Nodes Selected";
        var style = new GUIStyle(EditorStyles.boldLabel) 
        { 
            alignment = TextAnchor.MiddleLeft,
            normal = { textColor = count > 0 ? new Color(0.29f, 1f, 0.1f) : Color.gray }
        };
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
        int selectedBeamCount = 0;
        foreach (var beam in truss.beams)
            if (beam.isSelected) selectedBeamCount++;
        
        EditorGUILayout.BeginHorizontal();
        
        string btnText = isCreatingNode ? "Cancel Creation" : "New Point";
        Color originalColor = GUI.backgroundColor;
        if (isCreatingNode) GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
        
        if (GUILayout.Button(btnText))
        {
            if (selectedBeamCount > 0)
            {
                // Create midpoints on selected beams
                softBody.CreateMidpointOnSelectedBeams();
                isCreatingNode = false; 
            }
            else
            {
                // Toggle interactive creation
                isCreatingNode = !isCreatingNode;
                showGhostNode = isCreatingNode;
                SetEditorCollider(isCreatingNode);
                SceneView.RepaintAll();
            }
        }
        GUI.backgroundColor = originalColor;
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
        
        if (selectedBeamCount > 0)
        {
            EditorGUILayout.LabelField($"({selectedBeamCount} beams selected - New Point creates midpoints)", EditorStyles.miniLabel);
        }
    }
    
    private void DrawBeamsTab()
    {
        int count = 0;
        foreach (var beam in truss.beams)
            if (beam.isSelected) count++;
        
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        
        string selInfo = count == 0 ? "No Beams Selected" : $"{count} Beams Selected";
        var style = new GUIStyle(EditorStyles.boldLabel) 
        { 
            alignment = TextAnchor.MiddleLeft,
            normal = { textColor = count > 0 ? new Color(0.29f, 1f, 0.1f) : Color.gray }
        };
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

    private void DrawSettingsTab()
    {
        // Physics
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Physics Settings", EditorStyles.boldLabel);
        
        softBody.gravity = EditorGUILayout.Vector3Field("Gravity", softBody.gravity);
        softBody.damping = EditorGUILayout.Slider("Damping", softBody.damping, 0f, 1f);
        softBody.distanceStiffness = EditorGUILayout.Slider("Distance Stiffness", softBody.distanceStiffness, 0f, 1f);
        softBody.volumeStiffness = EditorGUILayout.Slider("Volume Stiffness", softBody.volumeStiffness, 0f, 1f);
        
        EditorGUILayout.EndVertical();
        
        // Plasticity
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Plasticity", EditorStyles.boldLabel);
        
        softBody.enablePlasticity = EditorGUILayout.Toggle("Enable Plasticity", softBody.enablePlasticity);
        if (softBody.enablePlasticity)
        {
            softBody.plasticityThreshold = EditorGUILayout.Slider("Threshold", softBody.plasticityThreshold, 0f, 1f);
            softBody.plasticityAmount = EditorGUILayout.Slider("Amount", softBody.plasticityAmount, 0f, 1f);
        }
        EditorGUILayout.EndVertical();
        
        // Collision
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Collision", EditorStyles.boldLabel);
        
        softBody.enableCollision = EditorGUILayout.Toggle("Enable Collision", softBody.enableCollision);
        if (softBody.enableCollision)
        {
            softBody.collisionRadius = EditorGUILayout.FloatField("Collision Radius", softBody.collisionRadius);
            softBody.friction = EditorGUILayout.Slider("Friction", softBody.friction, 0f, 1f);
            
            SerializedProperty colLayersProp = serializedObject.FindProperty("collisionLayers");
            if (colLayersProp != null)
            {
                EditorGUILayout.PropertyField(colLayersProp);
            }
            else
            {
                // Fallback manual drawing if property lookup fails
                // softBody.collisionLayers = LayerMaskField... (Unity doesn't have standard specific one, usually uses property)
            }
        }
        EditorGUILayout.EndVertical();
        
        // Deformation
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Deformation", EditorStyles.boldLabel);
        softBody.enableDeformation = EditorGUILayout.Toggle("Enable Deformation", softBody.enableDeformation);
        EditorGUILayout.EndVertical();
        
        // Debug
        EditorGUILayout.Space();
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        EditorGUILayout.LabelField("Debug", EditorStyles.boldLabel);
        softBody.showDebugGizmos = EditorGUILayout.Toggle("Show Gizmos", softBody.showDebugGizmos);
        EditorGUILayout.EndVertical();
    }
    
    #endregion
    
    #region Helper Methods
    
    private void DrawGhostNode(Vector3 worldPos)
    {
        float pulse = Mathf.Sin((float)EditorApplication.timeSinceStartup * 10f) * 0.2f + 1.2f; // Pulse between 1.0 and 1.4
        float size = HandleUtility.GetHandleSize(worldPos) * 0.4f * pulse;
        
        Handles.color = new Color(1f, 1f, 0f, 0.6f);
        Handles.SphereHandleCap(0, worldPos, Quaternion.identity, size, EventType.Repaint);
        
        GUIStyle labelStyle = new GUIStyle(EditorStyles.miniLabel) { normal = { textColor = Color.green }, fontSize = 12, fontStyle = FontStyle.Bold };
        Handles.Label(worldPos + Vector3.up * size * 1.2f, "Click to Place", labelStyle);
    }
    
    private bool TryGetNodeCreationPosition(Vector2 mousePosition, out Vector3 worldPos)
    {
        Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);
        worldPos = Vector3.zero;
        
        // 1. Prioritize hitting OUR soft body collider (which we added appropriately)
        // We can do this by checking if the hit collider belongs to our softBody
        RaycastHit[] hits = Physics.RaycastAll(ray);
        foreach (var hit in hits)
        {
            if (hit.collider.gameObject == softBody.gameObject)
            {
                worldPos = hit.point;
                return true;
            }
        }

        // If 'SetEditorCollider' is working, the softBody should have a MeshCollider.
        // If we want to strictly limit to the mesh, and we failed to hit it, return false.
        
        return false;
    }

    private void CreateNodeAtPosition(Vector3 worldPos)
    {
        RecordUndoIfNotPlaying("Create Node");
        
        Vector3 localPos = Application.isPlaying 
            ? softBody.transform.InverseTransformPoint(worldPos) 
            : softBody.transform.InverseTransformPoint(worldPos);

        // Add node
        SoftBodyTruss.TrussNode newNode = new SoftBodyTruss.TrussNode 
        {
            localPosition = localPos,
            worldPosition = worldPos,
            inverseMass = softBody.selectedPointSettings.inverseMass > 0 ? softBody.selectedPointSettings.inverseMass : 2f 
        };
        
        truss.nodes.Add(newNode);
        
        MarkDirtyIfNotPlaying();
    }
    
    #endregion
}

