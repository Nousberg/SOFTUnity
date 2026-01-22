/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */
using UnityEngine;
using UnityEditor;
using System.Collections.Generic;
using System.Linq;

namespace DynamicEngine
{
    [CustomEditor(typeof(SoftBody))]
    public class SoftBodyEditor : UnityEditor.Editor
    {
        #region Constants
        const int TAB_TRUSS = 0;
        const int TAB_NODES = 1;
        const int TAB_BEAMS = 2;
        const int TAB_FACES = 3;
        const int TAB_SETS = 4;
        const int TAB_SETTINGS = 5;
        #endregion

        #region References
        private SoftBody softBody;
        private SoftBody targetSoftBody;
        private Truss workingTruss;
        #endregion

        #region Editor State (Local - Moved from Runtime)
        // Selections
        private List<int> selectedNodes = new List<int>();
        private List<int> selectedBeams = new List<int>();
        private List<int> copiedNodes = new List<int>();
        private List<int> selectedFaces = new List<int>();


        // Node Settings
        private float nodeCreationMass = 0.5f;

        // Beam Settings
        private float defaultBeamCompliance = 0.01f;
        private float defaultBeamDamping = 0.3f;


        // Face Settings
        private float selectedFacePlasticityThreshold = 0.1f;
        private float selectedFacePlasticityRate = 0.1f;
        private float selectedFaceMaxDeformation = 0.2f;

        // Sets State
        private int selectedNodeSetIndex = -1;
        private string nodeSetCreationName = "New Node Set";
        private Color nodeSetCreationColor = Color.cyan;
        private bool showNodeSetLabels = true;
        private float nodeSetLabelOffset = 0.3f;

        private int selectedLinkSetIndex = -1;
        private string linkSetCreationName = "New Link Set";
        private Color linkSetCreationColor = Color.magenta;
        private bool showLinkSetLabels = true;
        private float linkSetLabelOffset = 0.3f;
        
        // Settings Tab Foldouts
        private bool showEnvironment = true;
        private bool showDeformation = true;
        private bool showCollision = true;
        private bool showDebug = true;
        private bool showVisualization = true;

        // Visualization Settings
        private bool showNodeIndices = true;
        private bool showBeamIndices = false;

        private float nodeDisplaySize = 0.15f; // Hit detection radius
        private float beamLineThickness = 2.0f;
        private Color nodeColor = Color.green;
        private Color selectedNodeColor = Color.yellow;
        private Color pinnedNodeColor = Color.red;
        private Color beamColor = Color.cyan;
        private Color selectedBeamColor = Color.magenta;

        private bool enableDepthCulling = true;
        private bool showOccludedElements = true;
        private float occludedAlpha = 0.5f;
        private bool showGizmos = true;
        private bool showRestAreaGizmos = true;
        
        // Render Distance Optimization
        private float maxNodesRenderDistance = 8f;
        private float maxBeamsRenderDistance = 8f;
        private float maxFacesRenderDistance = 8f;
        private float maxTextInfoRenderDistance = 1.3f;
        private float autoLinkDistance = 1.0f; // New setting for beam generation
        private bool enableDistanceCulling = true;
        private bool enableOcclusionCulling = true;

        
        // Cached visible nodes for beam optimization (populated by DrawNodes)
        private HashSet<int> visibleNodesAfterFilters = new HashSet<int>();

        // Generation
        private float GenerationResolution = 1f;
        private int interpolationResolution = 1;
        #endregion

        #region Editor State
        private int currentTab = 0;
        private GUIStyle roundedButtonStyle;
        private GUIStyle roundedButtonActiveStyle;
        private GUIStyle sectionStyle;
        
        // Selection state (from UniSoft)
        private bool isRegionSelecting = false;
        private Vector2 selectionStart;
        private Rect selectionRect;
        
        // Visualization Settings are now stored in this editor class (see above region)
        #endregion

        #region Tool State
        private bool isCreatingNode = false;
        private bool isCreatingBeam = false;
        private int beamStartNode = -1;
        #endregion

        #region Transform State
        private bool isTransformingNode = false;
        private int transformingNodeIndex = -1;
        private Vector3 selectionCenter = Vector3.zero;
        
        // Tool state tracking (from UniSoft)
        private static Quaternion s_LastCmdRotation = Quaternion.identity;
        private static Vector3 s_LastCmdScale = Vector3.one;
        private Vector3 ghostNodePosition = Vector3.zero;
        private bool showGhostNode = false;
        
        // Editor Buffers (for UniSoft layout parity)
        private float selectedMass = 1f;
        private bool selectedIsPinned = false;
        private float massToApply = 10f;
        private float selectedCompliance = 0.001f;
        private bool useChainLinking = false;

        private enum MatterPreset
        {
            Custom,
            Metal,
            Rubber
        }
        private MatterPreset currentPreset = MatterPreset.Custom;


        #endregion

        #region Optimization Helpers
        private static Mesh cachedSphereMesh;
        private static Material cachedSphereMaterial;

        private Mesh GetCachedSphereMesh()
        {
            if (cachedSphereMesh == null)
            {
                GameObject temp = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                cachedSphereMesh = temp.GetComponent<MeshFilter>().sharedMesh;
                DestroyImmediate(temp);
            }
            return cachedSphereMesh;
        }

        private Material GetCachedSphereMaterial()
        {
            if (cachedSphereMaterial == null)
            {
                // Simple colored shader that works in editor
                Shader shader = Shader.Find("Hidden/Internal-Colored");
                cachedSphereMaterial = new Material(shader);
                cachedSphereMaterial.hideFlags = HideFlags.HideAndDontSave;
            }
            return cachedSphereMaterial;
        }
        #endregion

        #region Unity Callbacks
        private void OnEnable()
        {
            softBody = (SoftBody)target;
            targetSoftBody = softBody;

            if (targetSoftBody != null)
            {
                workingTruss = targetSoftBody.GetTrussAsset();
                if (workingTruss != null)
                {
                    LoadDefaultsFromTruss(workingTruss);
                }
            }

            if (selectedNodes == null) selectedNodes = new List<int>();
            if (selectedBeams == null) selectedBeams = new List<int>();
            if (selectedBeams == null) selectedBeams = new List<int>();
            if (selectedFaces == null) selectedFaces = new List<int>();
            
            // Hide Unity's default transform gizmo (like UniSoft)
            Tools.hidden = true;

            LoadVisualizationPrefs();
        }
        private void OnDisable()
        {
            // Restore Unity's transform tools when editor is disabled
            Tools.hidden = false;
            SaveVisualizationPrefs();
        }

        public override void OnInspectorGUI()
        {
            // Ensure styles are initialized before drawing
            InitializeStyles();

            serializedObject.Update();

            if (targetSoftBody == null)
            {
                EditorGUILayout.HelpBox("No SoftBody component found on this GameObject.", MessageType.Error);
                return;
            }

            if (workingTruss != targetSoftBody.GetTrussAsset())
            {
                workingTruss = targetSoftBody.GetTrussAsset();
                if (workingTruss != null)
                {
                    LoadDefaultsFromTruss(workingTruss);
                }
            }

            // Header
            EditorGUILayout.Space();
            GUIStyle headerStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                fontSize = 14,
                alignment = TextAnchor.MiddleCenter
            };
            EditorGUILayout.LabelField("Soft Body Designer", headerStyle);
            
            EditorGUILayout.Space();

            // Truss Data
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUI.BeginDisabledGroup(Application.isPlaying);
            EditorGUI.BeginChangeCheck();
            Truss newTruss = (Truss)EditorGUILayout.ObjectField("Truss Data", workingTruss, typeof(Truss), false);
            if (EditorGUI.EndChangeCheck())
            {
                targetSoftBody.truss = newTruss;
                workingTruss = newTruss;
                if (workingTruss != null) LoadDefaultsFromTruss(workingTruss);
            }

            SerializedProperty matterProp = serializedObject.FindProperty("matter");
            EditorGUILayout.PropertyField(matterProp, new GUIContent("Matter Data"));
            EditorGUI.EndDisabledGroup();
            
            EditorGUILayout.EndVertical();
            if (matterProp.objectReferenceValue == null)
            {
                EditorGUILayout.HelpBox("Matter Data is required to edit the Soft Body.", MessageType.Error);
                serializedObject.ApplyModifiedProperties();
                return;
            }

            if (workingTruss == null)
            {
                EditorGUILayout.HelpBox("Assign a Soft Body Truss asset to begin editing.", MessageType.Error);
                serializedObject.ApplyModifiedProperties();
                return;
            }

            // Tabs
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            string[] tabs = { "Truss", "Nodes", "Beams", "Faces", "Sets", "Settings" };
            
            EditorGUI.BeginChangeCheck();
            currentTab = GUILayout.Toolbar(currentTab, tabs, GUILayout.Height(25));
            if (EditorGUI.EndChangeCheck())
            {
                // Deselect when switching to Truss (0) or Settings (4)
                if (currentTab == TAB_TRUSS || currentTab == TAB_SETTINGS)
                {
                    selectedNodes.Clear();
                    selectedBeams.Clear();
                    selectedFaces.Clear();
                    
                    isCreatingNode = false;
                    isCreatingBeam = false;
                    
                    SceneView.RepaintAll();
                }
                else if (currentTab == TAB_FACES)
                {
                    selectedNodes.Clear();
                    selectedBeams.Clear();
                }
                else
                {
                    selectedFaces.Clear();
                }
            }
            EditorGUILayout.EndVertical();

            EditorGUILayout.Space(10);

            UpdateDesignerModeFromTab();

            switch (currentTab)
            {
                case TAB_TRUSS: DrawTrussTab(); break;
                case TAB_NODES: DrawNodesTab(); break;
                case TAB_BEAMS: DrawBeamsTab(); break;
                case TAB_FACES: DrawFacesTab(); break;
                case TAB_SETS: DrawSetsTab(); break;
                case TAB_SETTINGS: DrawSettingsTab(); break;
            }

            if (GUI.changed)
            {
                serializedObject.ApplyModifiedProperties();
                SceneView.RepaintAll();
            }
        }

        public void OnSceneGUI()
        {
            if (softBody == null || targetSoftBody == null || workingTruss == null || workingTruss.NodePositions == null) return;
            
            // Allow transforming the object itself if nothing is selected
            bool hasSelection = selectedNodes.Count > 0 || selectedBeams.Count > 0 || selectedFaces.Count > 0;
            bool isModeActive = isCreatingNode || isCreatingBeam;
            
            // Hide default tools only when we are editing sub-objects
            Tools.hidden = hasSelection || isModeActive;
            
            int controlID = GUIUtility.GetControlID(FocusType.Passive);
            
            // Handle node creation input first
            if (isCreatingNode)
            {
                HandleUtility.AddDefaultControl(controlID);
                
                Event e = Event.current;
                Vector3? worldPosNullable = GetWorldPositionFromMouse(e.mousePosition);
                
                // Only show ghost and allow creation when mouse is over mesh
                if (worldPosNullable.HasValue)
                {
                    ghostNodePosition = worldPosNullable.Value;
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
                else if (e.type == EventType.MouseDown && e.button == 0 && showGhostNode)
                {
                    CreateNodeAtPosition(ghostNodePosition);
                    e.Use();
                }
                else if (e.type == EventType.KeyDown && e.keyCode == KeyCode.Escape)
                {
                    isCreatingNode = false;
                    showGhostNode = false;
                    e.Use();
                }
                
                if (showGhostNode)
                {
                    DrawGhostNode(ghostNodePosition);
                }
                
                SceneView.RepaintAll();
                if (currentTab == TAB_NODES || currentTab == TAB_BEAMS || currentTab == TAB_SETS)
                {
                    DrawNodes();
                    DrawBeams();
                }
                return; // Don't process other interactions in creation mode
            }

            // Standard mode
            HandleKeyboardInput();
            
            if (currentTab == TAB_NODES || currentTab == TAB_BEAMS || currentTab == TAB_SETS)
            {
                DrawNodes();
                DrawBeams();
            }
            if (currentTab == TAB_FACES)
            {
                DrawFaces();
            }
            HandleSelection(controlID);
            DrawMoveHandles();
            if (currentTab != TAB_FACES)
            {
                DrawSelectionLabels();
            }
            UpdateRuntimeRestLengthsForSelectedNodes();

            
            if (isRegionSelecting)
            {
                DrawSelectionRectangle();
            }
            
            // Force repaint for pulsating animation
            if (selectedNodes.Count > 0 || selectedFaces.Count > 0)
            {
                HandleUtility.Repaint();
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
            
            // Draw stats for all nodes (only on Nodes tab or Truss tab (Node mode))
            if (currentTab == TAB_NODES)
            {
                for (int i = 0; i < workingTruss.NodePositions.Length; i++)
                {
                    // Optimization: Skip label if node is not visible
                    if (!visibleNodesAfterFilters.Contains(i))
                        continue;

                    Vector3 localPos = workingTruss.NodePositions[i];
                    Vector3 pos = targetSoftBody.transform.TransformPoint(localPos);
                    
                    // Culling: check distance
                    if (enableDistanceCulling && Camera.current != null)
                    {
                        if (Vector3.Distance(pos, Camera.current.transform.position) > maxTextInfoRenderDistance)
                            continue;
                    }
                    
                    Vector3 labelPos = pos + Vector3.up * HandleUtility.GetHandleSize(pos) * 0.3f;
                    
                    // Build stats string
                    string stats = $"#{i}";
                    if (workingTruss.PinnedNodes.Contains(i)) stats += " [P]";
                    
                    // Mass from truss
                    float m = (i < workingTruss.NodeMasses.Count) ? workingTruss.NodeMasses[i] : 1f;
                    
                    stats += $"\nm:{m.ToString("F2")}";
                    
                    Handles.Label(labelPos, stats, labelStyle);
                }
            }
            
            // Draw stats for beams (only on Beams tab)
            if (currentTab == TAB_BEAMS)
            {
                var beams = workingTruss.GetTrussBeams();
                for (int i = 0; i < beams.Count; i++)
                {
                    var beam = beams[i];
                    
                    if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length)
                        continue;
                    
                    // Optimization: Skip label if beam is not visible (neither node is visible)
                    if (!visibleNodesAfterFilters.Contains(beam.nodeA) && !visibleNodesAfterFilters.Contains(beam.nodeB))
                        continue;

                    Vector3 posA = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeA]);
                    Vector3 posB = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeB]);
                    
                    Vector3 mid = (posA + posB) * 0.5f;

                    // Culling: check distance
                    if (enableDistanceCulling && Camera.current != null)
                    {
                        if (Vector3.Distance(mid, Camera.current.transform.position) > maxTextInfoRenderDistance)
                            continue;
                    }

                    Vector3 labelPos = mid + Vector3.up * HandleUtility.GetHandleSize(mid) * 0.2f;
                    
                    float currentLength = Vector3.Distance(posA, posB);
                    
                    // Build stats string
                    string stats = $"B#{i}\nL:{currentLength:F2}\nR:{beam.restLength:F2}";
                    if (beam.compliance > 0) stats += $"\nC:{beam.compliance:F3}";
                    
                    Handles.Label(labelPos, stats, labelStyle);
                }
            }
        }
        
        private void HandleKeyboardInput()
        {
            Event e = Event.current;
            
            if (e.type == EventType.KeyDown)
            {
                if (e.control && !e.shift && !e.alt && e.keyCode == KeyCode.C)
                {
                    copiedNodes.Clear();
                    if (selectedNodes.Count > 0)
                    {
                        copiedNodes.AddRange(selectedNodes);
                    }
                    e.Use();
                    return;
                }

                if (e.control && !e.shift && !e.alt && e.keyCode == KeyCode.V)
                {
                    if (copiedNodes.Count > 0)
                    {
                        selectedNodes.Clear();
                        selectedNodes.AddRange(copiedNodes);
                        DuplicateSelectedNodes();
                    }
                    e.Use();
                    return;
                }

                switch (e.keyCode)
                {
                    case KeyCode.Delete:
                    case KeyCode.Backspace:
                    case KeyCode.X:
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
        
        private void HandleSelection(int controlID)
        {
            Event e = Event.current;
            
            switch (e.type)
            {
                case EventType.MouseDown:
                    if (e.button == 0 && !e.alt && HandleUtility.nearestControl == controlID)
                    {
                        if (currentTab == TAB_FACES)
                        {
                        if (TrySelectFace(e))
                        {
                            e.Use();
                        }
                        else if (!e.shift && !e.control)
                        {
                            selectedFaces.Clear();
                            UpdateSelectedFaceValues();
                            Repaint();
                            SceneView.RepaintAll();
                        }
                        return;
                    }

                        // Try selecting a node first
                        if (TrySelectNode(e))
                        {
                            e.Use();
                            return;
                        }
                        
                        // Then try selecting a beam
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
        

        private bool TrySelectNode(Event e)
        {
            if (workingTruss.NodePositions == null) return false;

            Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            float minDist = float.MaxValue;
            int closestNodeIndex = -1;

            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 nodeWorldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                float handleSize = HandleUtility.GetHandleSize(nodeWorldPos) * 0.12f;

                float distance = Vector3.Cross(ray.direction, nodeWorldPos - ray.origin).magnitude;
                if (distance <= handleSize && distance < minDist)
                {
                    float t = Vector3.Dot(nodeWorldPos - ray.origin, ray.direction);
                    if (t > 0f)
                    {
                        minDist = distance;
                        closestNodeIndex = i;
                    }
                }
            }

            if (closestNodeIndex >= 0)
            {
                HandleNodeClick(closestNodeIndex);
                return true;
            }

            return false;
        }

        private bool TrySelectBeam(Event e)
        {
            var beams = workingTruss.GetTrussBeams();
            float minDist = 5f;
            int closestBeamIndex = -1;
            
            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length)
                    continue;
                
                Vector3 posA = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeA]);
                Vector3 posB = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeB]);
                
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
                selectedNodes.Clear();
                
                if (e.control)
                {
                    if (selectedBeams.Contains(closestBeamIndex))
                        selectedBeams.Remove(closestBeamIndex);
                    else
                        selectedBeams.Add(closestBeamIndex);
                }
                else if (e.shift)
                {
                    if (!selectedBeams.Contains(closestBeamIndex))
                        selectedBeams.Add(closestBeamIndex);
                }
                else
                {
                    selectedBeams.Clear();
                    selectedBeams.Add(closestBeamIndex);
                }
                
                UpdateSelectedBeamValues();
                EditorUtility.SetDirty(softBody);
                Repaint();
                return true;
            }
            
            return false;
        }

        private bool TrySelectFace(Event e)
        {
            if (workingTruss == null) return false;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return false;

            Ray ray = HandleUtility.GUIPointToWorldRay(e.mousePosition);
            float closestT = float.MaxValue;
            int closestFace = -1;

            for (int i = 0; i < faces.Count; i++)
            {
                var f = faces[i];
                if (f.nodeA < 0 || f.nodeB < 0 || f.nodeC < 0) continue;
                if (f.nodeA >= workingTruss.NodePositions.Length ||
                    f.nodeB >= workingTruss.NodePositions.Length ||
                    f.nodeC >= workingTruss.NodePositions.Length)
                    continue;

                Vector3 a = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[f.nodeA]);
                Vector3 b = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[f.nodeB]);
                Vector3 c = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[f.nodeC]);

                if (RayTriangleIntersect(ray, a, b, c, out float t, out _))
                {
                    if (t > 0f && t < closestT)
                    {
                        closestT = t;
                        closestFace = i;
                    }
                }
            }

            if (closestFace >= 0)
            {
                selectedNodes.Clear();
                selectedBeams.Clear();

                if (e.control)
                {
                    if (selectedFaces.Contains(closestFace))
                        selectedFaces.Remove(closestFace);
                    else
                        selectedFaces.Add(closestFace);
                }
                else if (e.shift)
                {
                    if (!selectedFaces.Contains(closestFace))
                        selectedFaces.Add(closestFace);
                }
                else
                {
                    selectedFaces.Clear();
                    selectedFaces.Add(closestFace);
                }

                UpdateSelectedFaceValues();
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
                    selectedNodes.Clear();
                    selectedBeams.Clear();
                    UpdateSelectedBeamValues();
                    EditorUtility.SetDirty(softBody);
                    Repaint();
                }
                return;
            }
            
            selectedBeams.Clear();
            
            Event e = Event.current;
            if (!e.control && !e.shift)
            {
                selectedNodes.Clear();
            }
            
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                Vector2 screenPos = HandleUtility.WorldToGUIPoint(pos);
                
                if (selectionRect.Contains(screenPos))
                {
                    if (e.control)
                    {
                        if (selectedNodes.Contains(i))
                            selectedNodes.Remove(i);
                        else
                            selectedNodes.Add(i);
                    }
                    else
                    {
                        if (!selectedNodes.Contains(i))
                            selectedNodes.Add(i);
                    }
                }
            }
            
            UpdateSelectionCenter();
            EditorUtility.SetDirty(softBody);
            Repaint();
        }
        
        private List<int> GetSelectedFaceNodes()
        {
            var result = new List<int>();
            if (workingTruss == null) return result;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return result;

            HashSet<int> unique = new HashSet<int>();
            foreach (int faceIndex in selectedFaces)
            {
                if (faceIndex < 0 || faceIndex >= faces.Count) continue;
                var f = faces[faceIndex];
                unique.Add(f.nodeA);
                unique.Add(f.nodeB);
                unique.Add(f.nodeC);
            }
            result.AddRange(unique);
            return result;
        }

        private void DrawMoveHandles()
        {
            List<int> activeNodes = currentTab == TAB_FACES ? GetSelectedFaceNodes() : selectedNodes;
            if (activeNodes.Count == 0) return;

            // FIX: Ensure matrix is identity so handles aren't squashed by object scale
            Handles.matrix = Matrix4x4.identity;
            
            // Reset handle state if not interacting
            if (GUIUtility.hotControl == 0)
            {
                s_LastCmdRotation = Quaternion.identity;
                s_LastCmdScale = Vector3.one;
            }
            
            // Calculate center of selection
            Vector3 center = Vector3.zero;
            int count = 0;
            foreach (int i in activeNodes)
            {
                if (i >= 0 && i < workingTruss.NodePositions.Length)
                {
                    Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                    center += pos;
                    count++;
                }
            }
            if (count == 0) return;
            center /= count;
            
            // Handle based on current tool
            var prevZTest = Handles.zTest;
            Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
            
            try
            {
                switch (Tools.current)
                {
                    case Tool.Move:
                        HandleMoveTool(center, activeNodes);
                        break;
                    case Tool.Rotate:
                        HandleRotateTool(center, activeNodes);
                        break;
                    case Tool.Scale:
                        HandleScaleTool(center, activeNodes);
                        break;
                }
            }
            finally
            {
                Handles.zTest = prevZTest;
            }
        }
        
        private void HandleMoveTool(Vector3 center, List<int> activeNodes)
        {
            EditorGUI.BeginChangeCheck();
            Vector3 newCenter = Handles.PositionHandle(center, Quaternion.identity);
            
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(workingTruss, "Move Nodes");
                Vector3 delta = newCenter - center;
                Vector3 localDelta = targetSoftBody.transform.InverseTransformVector(delta);
                
                foreach (int i in activeNodes)
                {
                    if (i >= 0 && i < workingTruss.NodePositions.Length)
                    {
                        workingTruss.NodePositions[i] += localDelta;
                    }
                }
                
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
                RecalculateRestLengthsForNodes(activeNodes);
            }
        }
        
        private void HandleRotateTool(Vector3 center, List<int> activeNodes)
        {
            if (activeNodes.Count < 2) return;
            
            EditorGUI.BeginChangeCheck();
            
            // Track cumulative rotation in the handle
            Quaternion newRotation = Handles.RotationHandle(s_LastCmdRotation, center);
            
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(workingTruss, "Rotate Nodes");
                
                Quaternion delta = newRotation * Quaternion.Inverse(s_LastCmdRotation);
                s_LastCmdRotation = newRotation;
                
                foreach (int i in activeNodes)
                {
                    if (i >= 0 && i < workingTruss.NodePositions.Length)
                    {
                        Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                        Vector3 relativePos = pos - center;
                        Vector3 rotatedPos = delta * relativePos;
                        Vector3 newPos = center + rotatedPos;
                        
                        workingTruss.NodePositions[i] = targetSoftBody.transform.InverseTransformPoint(newPos);
                    }
                }
                
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
                RecalculateRestLengthsForNodes(activeNodes);
            }
        }
        
        private void HandleScaleTool(Vector3 center, List<int> activeNodes)
        {
            if (activeNodes.Count < 2) return;
            
            EditorGUI.BeginChangeCheck();
            
            Vector3 newScale = Handles.ScaleHandle(s_LastCmdScale, center, Quaternion.identity, HandleUtility.GetHandleSize(center));
            
            if (EditorGUI.EndChangeCheck())
            {
                Undo.RecordObject(workingTruss, "Scale Nodes");
                
                Vector3 delta = new Vector3(
                    s_LastCmdScale.x == 0 ? 1 : newScale.x / s_LastCmdScale.x,
                    s_LastCmdScale.y == 0 ? 1 : newScale.y / s_LastCmdScale.y,
                    s_LastCmdScale.z == 0 ? 1 : newScale.z / s_LastCmdScale.z
                );
                
                s_LastCmdScale = newScale;
                
                foreach (int i in activeNodes)
                {
                    if (i >= 0 && i < workingTruss.NodePositions.Length)
                    {
                        Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                        Vector3 relativePos = pos - center;
                        Vector3 scaledPos = Vector3.Scale(relativePos, delta);
                        Vector3 newPos = center + scaledPos;
                        
                        workingTruss.NodePositions[i] = targetSoftBody.transform.InverseTransformPoint(newPos);
                    }
                }
                
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
                RecalculateRestLengthsForNodes(activeNodes);
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
            // Priority based on current tab
            if (currentTab == TAB_FACES && selectedFaces.Count > 0)
            {
                DeleteSelectedFaces();
                return;
            }
            else if (currentTab == TAB_BEAMS && selectedBeams.Count > 0)
            {
                if (EditorUtility.DisplayDialog("Delete Beams", 
                    $"Are you sure you want to delete {selectedBeams.Count} selected beam(s)?", 
                    "Delete", "Cancel"))
                {
                    DeleteSelectedBeams();
                }
                return;
            }
            else if (currentTab == TAB_NODES && selectedNodes.Count > 0)
            {
                if (EditorUtility.DisplayDialog("Delete Nodes", 
                    $"Are you sure you want to delete {selectedNodes.Count} selected node(s)?\nThis will also delete connected beams.", 
                    "Delete", "Cancel"))
                {
                    DeleteSelectedNodes();
                }
                return;
            }
            
            // Fallback: delete whatever is selected
            if (selectedNodes.Count > 0)
            {
                if (EditorUtility.DisplayDialog("Delete Nodes", 
                    $"Are you sure you want to delete {selectedNodes.Count} selected node(s)?\nThis will also delete connected beams.", 
                    "Delete", "Cancel"))
                {
                    DeleteSelectedNodes();
                }
            }
            else if (selectedBeams.Count > 0)
            {
                if (EditorUtility.DisplayDialog("Delete Beams", 
                    $"Are you sure you want to delete {selectedBeams.Count} selected beam(s)?", 
                    "Delete", "Cancel"))
                {
                    DeleteSelectedBeams();
                }
            }
            else if (selectedFaces.Count > 0)
            {
                DeleteSelectedFaces();
            }
        }
        
        private void DeselectAll()
        {
            selectedNodes.Clear();
            selectedBeams.Clear();
            selectedNodes.Clear();
            selectedBeams.Clear();
            EditorUtility.SetDirty(softBody);
        }
        
        #region Styles
        private void InitializeStyles()
        {
            if (roundedButtonStyle != null && roundedButtonStyle.normal.background != null)
                return;

            roundedButtonStyle = new GUIStyle(EditorStyles.miniButton)
            {
                padding = new RectOffset(10, 10, 5, 5),
                margin = new RectOffset(4, 4, 4, 4),
                fontSize = 11,
                fontStyle = FontStyle.Normal,
                alignment = TextAnchor.MiddleCenter,
                fixedHeight = 24 
            };

            roundedButtonActiveStyle = new GUIStyle(roundedButtonStyle)
            {
                fontStyle = FontStyle.Bold,
                normal = { textColor = new Color(0.3f, 0.8f, 0.3f) }
            };

            roundedButtonActiveStyle.normal.background = EditorStyles.miniButton.active.background;

            if (sectionStyle == null)
            {
                sectionStyle = new GUIStyle()
                {
                    padding = new RectOffset(8, 8, 8, 8),
                    margin = new RectOffset(0, 0, 4, 4)
                };
            }
        }

        private GUIStyle GetButtonStyle(bool isActive)
        {
            InitializeStyles();
            return isActive ? roundedButtonActiveStyle : roundedButtonStyle;
        }

        private GUIStyle GetRoundedButtonStyle()
        {
            InitializeStyles();
            return roundedButtonStyle;
        }
        #endregion

        #region Helper Methods
        private bool IsRuntimeEditing()
        {
            return Application.isPlaying && targetSoftBody != null && targetSoftBody.solver != null;
        }

        private int GetBaseBeamCount()
        {
            return workingTruss != null ? workingTruss.GetTrussBeams().Count : 0;
        }

        private void RecalculateRestLengthsForNodes(IEnumerable<int> nodeIndices)
        {
            if (workingTruss == null || workingTruss.NodePositions == null) return;
            var beams = workingTruss.GetTrussBeams();
            if (beams == null || beams.Count == 0) return;

            HashSet<int> nodes = nodeIndices as HashSet<int> ?? new HashSet<int>(nodeIndices);
            bool changed = false;

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (nodes.Contains(beam.nodeA) || nodes.Contains(beam.nodeB))
                {
                    Vector3 a = workingTruss.NodePositions[beam.nodeA];
                    Vector3 b = workingTruss.NodePositions[beam.nodeB];
                    beam.restLength = Vector3.Distance(a, b);
                    changed = true;
                }
            }

            if (changed)
            {
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
            }
        }

        private void ApplyPointSettings()
        {
            if (IsRuntimeEditing())
            {
                var solver = targetSoftBody.solver;
                for (int n = 0; n < selectedNodes.Count; n++)
                {
                    int i = selectedNodes[n];
                    if (i < 0) continue;

                    if (i < solver.nodeMasses.Count)
                    {
                        solver.nodeMasses[i] = Mathf.Max(0.01f, selectedMass);
                    }

                    solver.nodeManager.SetPinned(i, selectedIsPinned);
                }
                return;
            }

            Undo.RecordObject(workingTruss, "Apply Point Settings");
            foreach (int i in selectedNodes)
            {
                if (i >= 0 && i < workingTruss.NodeMasses.Count)
                {
                    // Apply the entered mass directly to each selected node
                    workingTruss.NodeMasses[i] = Mathf.Max(0.01f, selectedMass);
                }
                
                if (i >= 0 && i < workingTruss.NodePositions.Length)
                {
                    if (selectedIsPinned)
                    {
                        if (!workingTruss.PinnedNodes.Contains(i)) workingTruss.PinnedNodes.Add(i);
                    }
                    else
                    {
                        workingTruss.PinnedNodes.Remove(i);
                    }
                }
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void ApplyBeamSettings()
        {
            if (IsRuntimeEditing())
            {
                var solver = targetSoftBody.solver;
                var solverBeams = solver.beams;
                int baseBeamCount = GetBaseBeamCount();

                foreach (int i in selectedBeams)
                {
                    if (i >= 0 && i < baseBeamCount && i < solverBeams.Count)
                    {
                        solverBeams[i].compliance = selectedCompliance;
                    }
                }
                return;
            }

            Undo.RecordObject(workingTruss, "Apply Beam Settings");
            var beams = workingTruss.GetTrussBeams();
            foreach (int i in selectedBeams)
            {
                if (i >= 0 && i < beams.Count)
                {
                    beams[i].compliance = selectedCompliance;
                }
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void ApplyFullBeamSettings()
        {
            if (selectedBeams.Count == 0) return;

            float perBeamCompliance = selectedCompliance;
            float perBeamDamping = defaultBeamDamping;

            if (IsRuntimeEditing())
            {
                var solverBeams = targetSoftBody.solver.beams;
                int baseBeamCount = GetBaseBeamCount();

                foreach (int i in selectedBeams)
                {
                    if (i >= 0 && i < baseBeamCount && i < solverBeams.Count)
                    {
                        solverBeams[i].compliance = perBeamCompliance;
                        solverBeams[i].damping = perBeamDamping;
                    }
                }
            }
            else
            {
                Undo.RecordObject(workingTruss, "Apply Full Beam Settings");
                var beams = workingTruss.GetTrussBeams();
                foreach (int i in selectedBeams)
                {
                    if (i >= 0 && i < beams.Count)
                    {
                        beams[i].compliance = perBeamCompliance;
                        beams[i].damping = perBeamDamping;
                    }
                }
                EditorUtility.SetDirty(workingTruss);
            }

            // Refresh displayed values
            UpdateSelectedBeamValues();
        }

        private void UpdateSelectedBeamValues()
        {
            if (workingTruss == null || selectedBeams.Count == 0) return;

            List<Beam> beams = IsRuntimeEditing() ? targetSoftBody.solver.beams : workingTruss.GetTrussBeams();
            int maxIndex = IsRuntimeEditing() ? Mathf.Min(GetBaseBeamCount(), beams.Count) : beams.Count;
            
            // Calculate average of all selected beams' values
            float sumCompliance = 0f;
            float sumDamping = 0f;
            int count = 0;
            
            foreach (int i in selectedBeams)
            {
                if (i >= 0 && i < maxIndex)
                {
                    sumCompliance += beams[i].compliance;
                    sumDamping += beams[i].damping;
                    count++;
                }
            }
            
            // Update displayed values
            if (count > 0)
            {
                selectedCompliance = sumCompliance / count;
                defaultBeamDamping = sumDamping / count;
            }
        }

        private void UpdateSelectedFaceValues()
        {
            if (workingTruss == null || selectedFaces.Count == 0) return;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;

            float sumThreshold = 0f;
            float sumRate = 0f;
            float sumMaxDef = 0f;
            int count = 0;

            foreach (int i in selectedFaces)
            {
                if (i < 0 || i >= faces.Count) continue;
                var f = faces[i];
                sumThreshold += f.areaPlasticityThreshold;
                sumRate += f.areaPlasticityRate;
                sumMaxDef += f.maxAreaDeformation;
                count++;
            }

            if (count > 0)
            {
                selectedFacePlasticityThreshold = sumThreshold / count;
                selectedFacePlasticityRate = sumRate / count;
                selectedFaceMaxDeformation = sumMaxDef / count;
            }
        }

        private void ApplyFaceSettingsToSelected()
        {
            if (workingTruss == null || selectedFaces.Count == 0) return;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;

            Undo.RecordObject(workingTruss, "Apply Face Settings");
            foreach (int i in selectedFaces)
            {
                if (i < 0 || i >= faces.Count) continue;
                var f = faces[i];
                f.areaPlasticityThreshold = selectedFacePlasticityThreshold;
                f.areaPlasticityRate = selectedFacePlasticityRate;
                f.maxAreaDeformation = selectedFaceMaxDeformation;
                faces[i] = f;
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void AddBeamIfMissing(List<Beam> beams, int a, int b)
        {
            if (beams.Exists(x => (x.nodeA == a && x.nodeB == b) || (x.nodeA == b && x.nodeB == a)))
                return;

            Vector3 pA = workingTruss.NodePositions[a];
            Vector3 pB = workingTruss.NodePositions[b];
            beams.Add(new Beam(a, b, defaultBeamCompliance, defaultBeamDamping, Vector3.Distance(pA, pB)));
        }

        private void DuplicateSelectedFaces()
        {
            if (workingTruss == null || selectedFaces.Count == 0) return;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;

            Undo.RecordObject(workingTruss, "Duplicate Faces");

            List<Vector3> nodePositions = workingTruss.NodePositions.ToList();
            List<float> nodeMasses = new List<float>(workingTruss.NodeMasses);
            List<int> pinnedNodes = new List<int>(workingTruss.PinnedNodes);
            var beams = workingTruss.GetTrussBeams();

            List<int> newFaceIndices = new List<int>();
            List<int> facesToDuplicate = new List<int>(selectedFaces);

            foreach (int faceIndex in facesToDuplicate)
            {
                if (faceIndex < 0 || faceIndex >= faces.Count) continue;
                var f = faces[faceIndex];

                int[] oldNodes = new int[] { f.nodeA, f.nodeB, f.nodeC };
                int[] newNodes = new int[3];

                for (int n = 0; n < 3; n++)
                {
                    int oldIdx = oldNodes[n];
                    if (oldIdx < 0 || oldIdx >= workingTruss.NodePositions.Length) continue;

                    Vector3 pos = workingTruss.NodePositions[oldIdx];
                    newNodes[n] = nodePositions.Count;
                    nodePositions.Add(pos);

                    float mass = oldIdx < workingTruss.NodeMasses.Count ? workingTruss.NodeMasses[oldIdx] : nodeCreationMass;
                    nodeMasses.Add(mass);

                    if (workingTruss.PinnedNodes.Contains(oldIdx))
                        pinnedNodes.Add(newNodes[n]);
                }

                AddBeamIfMissing(beams, newNodes[0], newNodes[1]);
                AddBeamIfMissing(beams, newNodes[1], newNodes[2]);
                AddBeamIfMissing(beams, newNodes[2], newNodes[0]);

                var newFace = f;
                newFace.nodeA = newNodes[0];
                newFace.nodeB = newNodes[1];
                newFace.nodeC = newNodes[2];
                faces.Add(newFace);
                newFaceIndices.Add(faces.Count - 1);
            }

            workingTruss.SetNodePositions(nodePositions.ToArray());
            workingTruss.SetNodeMasses(nodeMasses);
            workingTruss.SetPinnedNodes(pinnedNodes);
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();

            selectedFaces.Clear();
            selectedFaces.AddRange(newFaceIndices);
            UpdateSelectedFaceValues();
            SceneView.RepaintAll();
            Repaint();
        }

        private void DeleteSelectedFaces()
        {
            if (workingTruss == null || selectedFaces.Count == 0) return;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;

            // Collect all nodes used by the selected faces
            HashSet<int> nodesToDelete = new HashSet<int>();
            foreach (int faceIdx in selectedFaces)
            {
                if (faceIdx >= 0 && faceIdx < faces.Count)
                {
                    var face = faces[faceIdx];
                    nodesToDelete.Add(face.nodeA);
                    nodesToDelete.Add(face.nodeB);
                    nodesToDelete.Add(face.nodeC);
                }
            }

            // Count beams that will be deleted (connected to these nodes)
            var beams = workingTruss.GetTrussBeams();
            int beamsToDeleteCount = 0;
            foreach (var beam in beams)
            {
                if (nodesToDelete.Contains(beam.nodeA) || nodesToDelete.Contains(beam.nodeB))
                    beamsToDeleteCount++;
            }

            if (!EditorUtility.DisplayDialog("Delete Faces",
                $"Delete {selectedFaces.Count} face(s)?\n\n" +
                "This will also remove:\n" +
                $" - {nodesToDelete.Count} nodes\n" +
                $" - {beamsToDeleteCount} beams\n\n" +
                "THIS CANNOT BE UNDONE",
                "Delete", "Cancel"))
            {
                return;
            }

            Undo.RecordObject(workingTruss, "Delete Faces with Nodes and Beams");

            // Use the existing node deletion infrastructure
            // First, select the nodes
            selectedNodes.Clear();
            selectedNodes.AddRange(nodesToDelete);

            // Build index mapping
            Dictionary<int, int> oldToNewIndex = BuildIndexMapping(nodesToDelete);

            // Delete nodes
            RemoveNodesFromTruss(nodesToDelete);
            UpdateNodeMasses(nodesToDelete);
            UpdatePinnedNodes(nodesToDelete, oldToNewIndex);
            var removedBeamIndices = UpdateBeams(nodesToDelete, oldToNewIndex);
            UpdateFaces(nodesToDelete, oldToNewIndex);
            UpdateNodeSetsAfterDeletion(nodesToDelete, oldToNewIndex);
            UpdateLinkSetsAfterDeletion(removedBeamIndices);

            selectedNodes.Clear();
            selectedBeams.Clear();
            selectedFaces.Clear();

            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
            UpdateSelectedFaceValues();
            SceneView.RepaintAll();
            Repaint();
        }

        private void GenerateFacesFromBeams(bool append)
        {
            if (workingTruss == null || workingTruss.NodePositions == null) return;
            var beams = workingTruss.GetTrussBeams();
            if (beams == null || beams.Count == 0) return;

            int nodeCount = workingTruss.NodePositions.Length;
            if (nodeCount == 0) return;

            Undo.RecordObject(workingTruss, "Generate Faces From Beams");

            // Build adjacency list
            var adj = new HashSet<int>[nodeCount];
            for (int i = 0; i < nodeCount; i++) adj[i] = new HashSet<int>();
            foreach (var b in beams)
            {
                if (b.nodeA < 0 || b.nodeB < 0 || b.nodeA >= nodeCount || b.nodeB >= nodeCount) continue;
                adj[b.nodeA].Add(b.nodeB);
                adj[b.nodeB].Add(b.nodeA);
            }

            var faces = workingTruss.GetTrussFaces();
            if (!append)
            {
                faces.Clear();
            }

            HashSet<(int, int, int)> existing = new HashSet<(int, int, int)>();
            foreach (var f in faces)
            {
                int a = f.nodeA, b = f.nodeB, c = f.nodeC;
                int x = Mathf.Min(a, Mathf.Min(b, c));
                int z = Mathf.Max(a, Mathf.Max(b, c));
                int y = a + b + c - x - z;
                existing.Add((x, y, z));
            }

            int created = 0;
            for (int i = 0; i < nodeCount; i++)
            {
                foreach (int j in adj[i])
                {
                    if (j <= i) continue;
                    foreach (int k in adj[j])
                    {
                        if (k <= j) continue;
                        if (!adj[i].Contains(k)) continue;

                        int x = i, y = j, z = k;
                        if (existing.Contains((x, y, z))) continue;

                        Vector3 p0 = workingTruss.NodePositions[x];
                        Vector3 p1 = workingTruss.NodePositions[y];
                        Vector3 p2 = workingTruss.NodePositions[z];
                        float area = Vector3.Cross(p1 - p0, p2 - p0).magnitude * 0.5f;
                        if (area <= 1e-6f) continue;

                        Face f = new Face(x, y, z, true);
                        f.areaPlasticityThreshold = selectedFacePlasticityThreshold;
                        f.areaPlasticityRate = selectedFacePlasticityRate;
                        f.maxAreaDeformation = selectedFaceMaxDeformation;
                        f.InitializeRestArea(p0, p1, p2);

                        faces.Add(f);
                        existing.Add((x, y, z));
                        created++;
                    }
                }
            }

            if (created > 0)
            {
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
                Debug.Log($"Generated {created} faces from beams (append={append})");
            }
            else
            {
                Debug.LogWarning("No new faces generated from beams.");
            }
        }

        private void ApplySettingsToAllBeams()
        {
            if (IsRuntimeEditing())
            {
                var solverBeams = targetSoftBody.solver.beams;
                int baseBeamCount = GetBaseBeamCount();
                for (int i = 0; i < baseBeamCount && i < solverBeams.Count; i++)
                {
                    solverBeams[i].compliance = defaultBeamCompliance;
                    solverBeams[i].damping = defaultBeamDamping;
                }
                return;
            }

            if (workingTruss == null) return;

            Undo.RecordObject(workingTruss, "Apply Settings To All Beams");
            var beams = workingTruss.GetTrussBeams();
            foreach (var beam in beams)
            {
                beam.compliance = defaultBeamCompliance;
                beam.damping = defaultBeamDamping;
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private string GetVizPrefKey(string suffix)
        {
            int id = targetSoftBody != null ? targetSoftBody.GetInstanceID() : 0;
            return $"DE3D.SoftBodyEditor.Viz.{id}.{suffix}";
        }

        private void LoadVisualizationPrefs()
        {
            if (targetSoftBody == null) return;

            enableDepthCulling = EditorPrefs.GetBool(GetVizPrefKey("EnableDepthCulling"), enableDepthCulling);
            occludedAlpha = EditorPrefs.GetFloat(GetVizPrefKey("OccludedAlpha"), occludedAlpha);
            enableDistanceCulling = EditorPrefs.GetBool(GetVizPrefKey("EnableDistanceCulling"), enableDistanceCulling);
            maxNodesRenderDistance = EditorPrefs.GetFloat(GetVizPrefKey("MaxNodesRenderDistance"), maxNodesRenderDistance);
            maxBeamsRenderDistance = EditorPrefs.GetFloat(GetVizPrefKey("MaxBeamsRenderDistance"), maxBeamsRenderDistance);
            maxFacesRenderDistance = EditorPrefs.GetFloat(GetVizPrefKey("MaxFacesRenderDistance"), maxFacesRenderDistance);
            maxTextInfoRenderDistance = EditorPrefs.GetFloat(GetVizPrefKey("MaxTextInfoRenderDistance"), maxTextInfoRenderDistance);
            enableOcclusionCulling = EditorPrefs.GetBool(GetVizPrefKey("EnableOcclusionCulling"), enableOcclusionCulling);
            showGizmos = EditorPrefs.GetBool(GetVizPrefKey("ShowGizmos"), showGizmos);
            showRestAreaGizmos = EditorPrefs.GetBool(GetVizPrefKey("ShowRestAreaGizmos"), showRestAreaGizmos);

            // Clamp to safe ranges after load
            occludedAlpha = Mathf.Clamp01(occludedAlpha);
            maxNodesRenderDistance = Mathf.Max(0f, maxNodesRenderDistance);
            maxBeamsRenderDistance = Mathf.Clamp(maxBeamsRenderDistance, 0f, maxNodesRenderDistance);
            maxFacesRenderDistance = Mathf.Clamp(maxFacesRenderDistance, 0f, maxNodesRenderDistance);
            maxTextInfoRenderDistance = Mathf.Max(0f, maxTextInfoRenderDistance);
        }

        private void SaveVisualizationPrefs()
        {
            if (targetSoftBody == null) return;

            EditorPrefs.SetBool(GetVizPrefKey("EnableDepthCulling"), enableDepthCulling);
            EditorPrefs.SetFloat(GetVizPrefKey("OccludedAlpha"), occludedAlpha);
            EditorPrefs.SetBool(GetVizPrefKey("EnableDistanceCulling"), enableDistanceCulling);
            EditorPrefs.SetFloat(GetVizPrefKey("MaxNodesRenderDistance"), maxNodesRenderDistance);
            EditorPrefs.SetFloat(GetVizPrefKey("MaxBeamsRenderDistance"), maxBeamsRenderDistance);
            EditorPrefs.SetFloat(GetVizPrefKey("MaxFacesRenderDistance"), maxFacesRenderDistance);
            EditorPrefs.SetFloat(GetVizPrefKey("MaxTextInfoRenderDistance"), maxTextInfoRenderDistance);
            EditorPrefs.SetBool(GetVizPrefKey("EnableOcclusionCulling"), enableOcclusionCulling);
            EditorPrefs.SetBool(GetVizPrefKey("ShowGizmos"), showGizmos);
            EditorPrefs.SetBool(GetVizPrefKey("ShowRestAreaGizmos"), showRestAreaGizmos);
        }

        private void UpdateRuntimeRestLengthsForSelectedNodes()
        {
            if (!Application.isPlaying) return;
            if (targetSoftBody == null || targetSoftBody.solver == null) return;
            if (selectedNodes.Count == 0) return;

            var solver = targetSoftBody.solver;
            var solverBeams = solver.beams;
            int baseBeamCount = GetBaseBeamCount();
            if (baseBeamCount == 0 || solverBeams == null || solverBeams.Count == 0) return;

            var positions = solver.nodeManager.PredictedPositions;
            if (positions == null) return;

            HashSet<int> selected = new HashSet<int>(selectedNodes);
            int count = Mathf.Min(baseBeamCount, solverBeams.Count);

            for (int i = 0; i < count; i++)
            {
                var beam = solverBeams[i];
                if (!selected.Contains(beam.nodeA) && !selected.Contains(beam.nodeB)) continue;
                if (beam.nodeA < 0 || beam.nodeA >= positions.Count) continue;
                if (beam.nodeB < 0 || beam.nodeB >= positions.Count) continue;
                beam.restLength = Vector3.Distance(positions[beam.nodeA], positions[beam.nodeB]);
            }
        }

        private void LinkSelectedNodes()
        {
            if (selectedNodes.Count < 2) return;
            Undo.RecordObject(workingTruss, "Link Selected Nodes");
             
            var nodes = workingTruss.NodePositions;
            var beams = workingTruss.GetTrussBeams();

            if (useChainLinking)
            {
                // Chain Link: Link nodes sequentially (0-1, 1-2, 2-3...)
                for (int i = 0; i < selectedNodes.Count - 1; i++)
                {
                    int idxA = selectedNodes[i];
                    int idxB = selectedNodes[i + 1];
                    
                    float dist = Vector3.Distance(nodes[idxA], nodes[idxB]);
                    if (autoLinkDistance > 0f && dist > autoLinkDistance) continue;
                    
                    // Avoid duplicates
                    if (!beams.Exists(b => (b.nodeA == idxA && b.nodeB == idxB) || (b.nodeA == idxB && b.nodeB == idxA)))
                    {
                        beams.Add(new Beam(idxA, idxB, 0.01f, 0.3f, dist));
                    }
                }
            }
            else
            {
                // Standard Link: All-to-All
                for (int i = 0; i < selectedNodes.Count; i++)
                {
                    for (int j = i + 1; j < selectedNodes.Count; j++)
                    {
                        int idxA = selectedNodes[i];
                        int idxB = selectedNodes[j];
                         
                        float dist = Vector3.Distance(nodes[idxA], nodes[idxB]);
                        if (autoLinkDistance > 0f && dist > autoLinkDistance) continue;
                        
                        if (!beams.Exists(b => (b.nodeA == idxA && b.nodeB == idxB) || (b.nodeA == idxB && b.nodeB == idxA)))
                        {
                             beams.Add(new Beam(idxA, idxB, 0.01f, 0.3f, dist));
                        }
                    }
                }
            }
            EditorUtility.SetDirty(workingTruss);
        }




        #endregion





        // ... existing region References, Editor State, Tool State, Transform State, Unity Callbacks ...

        private void DrawTrussTab()
        {
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Truss Information", EditorStyles.boldLabel);
            EditorGUILayout.Separator();
            
            GUI.enabled = false;
            int nodeCount = workingTruss != null ? (workingTruss.NodePositions?.Length ?? 0) : 0;
            int beamCount = workingTruss != null ? (workingTruss.GetTrussBeams()?.Count ?? 0) : 0;
            int triCount = workingTruss != null ? CountImplicitTriangles(workingTruss) : 0;
            
            EditorGUILayout.IntField("Nodes", nodeCount);
            EditorGUILayout.IntField("Beams", beamCount);
            EditorGUILayout.IntField("Triangles", triCount);
            GUI.enabled = true;
            
            EditorGUILayout.EndVertical();
            
            // Reset Deformation (Runtime Only)
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Runtime Actions", EditorStyles.boldLabel);
            
            EditorGUI.BeginDisabledGroup(!Application.isPlaying);
            GUI.backgroundColor = new Color(0.5f, 0.8f, 1f);
            if (GUILayout.Button("Reset Deformation", GUILayout.Height(25)))
            {
                targetSoftBody.ResetDeformation();
                SceneView.RepaintAll();
            }
            GUI.backgroundColor = Color.white;
            EditorGUI.EndDisabledGroup();
            
            if (!Application.isPlaying)
            {
                EditorGUILayout.HelpBox("Reset Deformation is only available during Play mode.", MessageType.Info);
            }
            EditorGUILayout.EndVertical();
            
            // Danger Zone
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Danger Zone", EditorStyles.boldLabel);
            
            // Disable danger buttons in Play Mode
            EditorGUI.BeginDisabledGroup(Application.isPlaying);
            
            EditorGUILayout.BeginHorizontal();
            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            
            // Disable Clear Beams if no beams
            bool hasBeams = beamCount > 0;
            EditorGUI.BeginDisabledGroup(!hasBeams);
            if (GUILayout.Button("Clear Beams", GUILayout.Height(25)))
            {
                if (EditorUtility.DisplayDialog("Clear Beams", "Remove all beams? This cannot be undone.", "Clear", "Cancel"))
                {
                    EditorClearBeams();
                    SceneView.RepaintAll();
                }
            }
            EditorGUI.EndDisabledGroup();
            
            // Disable Clear Grid if no nodes
            bool hasNodes = nodeCount > 0;
            EditorGUI.BeginDisabledGroup(!hasNodes);
            if (GUILayout.Button("Clear Grid", GUILayout.Height(25)))
            {
                if (EditorUtility.DisplayDialog("Clear Grid", "Remove all nodes, beams, and elements? This cannot be undone.", "Clear", "Cancel"))
                {
                    EditorClearGrid();
                    SceneView.RepaintAll();
                }
            }
            EditorGUI.EndDisabledGroup();
            
            GUI.backgroundColor = Color.white;
            EditorGUILayout.EndHorizontal();
            
            EditorGUI.EndDisabledGroup(); 
            EditorGUILayout.EndVertical();
            
        }

        private void DrawSetsTab()
        {
            DrawNodeSetsTab();
            EditorGUILayout.Space(20);
            DrawLinkSetsTab();
        }

        // ... existing DrawDangerZone (now called from DrawTrussTab), DrawNodesTab, DrawBeamsTab ...
        
        // Removed DrawFacesTab logic for brevity in this specific replacement, 
        // assuming DrawFacesTab method definition still exists below or needs to be removed if tab is gone.
        // Wait, "Faces" tab is not in the new list. I should check if I need to delete DrawFacesTab or just not call it.
        // I'll keep the methods but not call them for now, or just leave it.
        // The instruction says "Update tabs", so I am removing TAB_FACES.
        







        private void DrawNodesTab()
        {
            int count = selectedNodes.Count;
            
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
                selectedMass = EditorGUILayout.FloatField("Mass", selectedMass);
                selectedIsPinned = EditorGUILayout.Toggle("Is Pinned", selectedIsPinned);
                
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply to Selected"))
                {
                    ApplyPointSettings();
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.HelpBox("Select nodes in Scene View to edit.", MessageType.Warning);
            }
            
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space();
            
            // Mass Application
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Mass Correction", EditorStyles.boldLabel);
            EditorGUILayout.BeginHorizontal();
            massToApply = EditorGUILayout.FloatField("Total Mass", massToApply);
            
            // Disable if no nodes exist
            bool hasNodesForMass = workingTruss != null && workingTruss.NodePositions != null && workingTruss.NodePositions.Length > 0;
            EditorGUI.BeginDisabledGroup(!hasNodesForMass);
            if (GUILayout.Button("Apply to ALL Nodes"))
            {
                if (massToApply > 0.0001f && workingTruss.NodePositions.Length > 0)
                {
                    float massPerNode = massToApply / (float)workingTruss.NodePositions.Length;
                    
                    if (Application.isPlaying && targetSoftBody != null && targetSoftBody.solver != null)
                    {
                        // RUNTIME: Only update solver masses (don't modify ScriptableObject)
                        var solverMasses = targetSoftBody.solver.nodeMasses;
                        while (solverMasses.Count < workingTruss.NodePositions.Length)
                            solverMasses.Add(massPerNode);
                        
                        for (int j = 0; j < solverMasses.Count && j < workingTruss.NodePositions.Length; j++)
                        {
                            solverMasses[j] = massPerNode;
                        }
                        Debug.Log($"[Runtime] Applied total mass {massToApply} ({massPerNode} per node) to solver. ScriptableObject unchanged.");
                    }
                    else
                    {
                        // EDITOR: Update ScriptableObject
                        Undo.RecordObject(workingTruss, "Apply Mass to All");
                        
                        var masses = workingTruss.NodeMasses;
                        while (masses.Count < workingTruss.NodePositions.Length) masses.Add(1f);

                        for (int i = 0; i < masses.Count; i++)
                        {
                            masses[i] = massPerNode;
                        }
                        
                        EditorUtility.SetDirty(workingTruss);
                        Debug.Log($"Applied total mass {massToApply} ({massPerNode} per node) to {workingTruss.NodePositions.Length} nodes.");
                    }
                    
                    // Update inspector display
                    if (massPerNode > 0.0001f)
                    {
                        selectedMass = massPerNode;
                    }
                }
            }
            EditorGUI.EndDisabledGroup();
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space();
            
            // Node tools
            int selectedBeamCount = selectedBeams.Count;
            
            EditorGUILayout.BeginHorizontal();
            
            string btnText = isCreatingNode ? "Cancel Creation" : "New Point";
            Color originalColor = GUI.backgroundColor;
            if (isCreatingNode) GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            
            if (GUILayout.Button(btnText))
            {
                if (selectedBeamCount > 0)
                {
                    CreateMidpointsOnSelectedBeams();
                }
                else
                {
                    ToggleNodeCreation();
                }
            }
            GUI.backgroundColor = originalColor;
            
            GUI.enabled = count > 0;
            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            if (GUILayout.Button("Delete"))
            {
                DeleteSelection();
            }
            GUI.backgroundColor = Color.white;
            GUI.enabled = true;

            bool allNodesSelected = workingTruss.NodePositions.Length > 0 && selectedNodes.Count == workingTruss.NodePositions.Length;
    GUI.enabled = workingTruss.NodePositions.Length > 0 && !allNodesSelected;
            if (GUILayout.Button("Select All"))
            {
                selectedNodes.Clear();
                selectedBeams.Clear(); // Clear beams when selecting all nodes
                for (int i = 0; i < workingTruss.NodePositions.Length; i++)
                    selectedNodes.Add(i);
                Repaint();
            }
            GUI.enabled = true;
            
            // Disable Clear if no nodes are selected
            GUI.enabled = count > 0;
            if (GUILayout.Button("Clear"))
            {
                DeselectAll();
            }
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();
            
            if (selectedBeamCount > 0)
            {
                EditorGUILayout.LabelField($"({selectedBeamCount} beams selected - New Point creates midpoints)", EditorStyles.miniLabel);
            }
            
            // Generation section
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Generate from Mesh", EditorStyles.boldLabel);
            MeshFilter meshFilter = softBody.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
            {
                EditorGUILayout.LabelField($"Source: {meshFilter.sharedMesh.name} ({meshFilter.sharedMesh.vertexCount} verts)", EditorStyles.miniLabel);
                
                EditorGUI.BeginChangeCheck();
                GenerationResolution = EditorGUILayout.Slider("Resolution", GenerationResolution, 0.01f, 1f);
                if (EditorGUI.EndChangeCheck())
                {
                    RecalculateEstimatedNodes();
                }
                
                // Ensure estimation is calculated if missing (e.g. first open)
                if (cachedEstimatedNodes == 0 && GenerationResolution > 0) RecalculateEstimatedNodes();
                
                EditorGUILayout.LabelField($"Estimated Nodes: ~{cachedEstimatedNodes}", EditorStyles.miniLabel);
                
                EditorGUI.BeginDisabledGroup(Application.isPlaying);
                GUI.backgroundColor = new Color(0.5f, 1f, 0.5f);
                if (GUILayout.Button("Merge Nodes from Mesh", GUILayout.Height(25)))
                {
                    if (EditorUtility.DisplayDialog("Merge Nodes from Mesh",
                        "This will add new nodes from mesh vertices, preserving existing nodes and connections.\nContinue?",
                        "Merge", "Cancel"))
                    {
                        EditorGenerateFromMesh();
                        SceneView.RepaintAll();
                    }
                }
                GUI.backgroundColor = Color.white;
                EditorGUI.EndDisabledGroup();
            }
            else
            {
                EditorGUILayout.HelpBox("No MeshFilter found.", MessageType.Warning);
            }
            EditorGUILayout.EndVertical();

            // Interpolation (Upscaler) section
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Generate Interpolation (Upscaler)", EditorStyles.boldLabel);
            
            interpolationResolution = EditorGUILayout.IntSlider("Resolution", interpolationResolution, 1, 10);
            if (interpolationResolution < 1) interpolationResolution = 1;
            EditorGUILayout.LabelField($"Subdivides beams {interpolationResolution} times.", EditorStyles.miniLabel);
            
            if (GUILayout.Button("Interpolate Nodes"))
            {
                if (interpolationResolution >= 1)
                {
                    if (EditorUtility.DisplayDialog("Interpolate Nodes", 
                        $"This will subdivide all beams {interpolationResolution} times. This increases node count significantly.\nContinue?", "Interpolate", "Cancel"))
                    {
                        GenerateInterpolatedNodes(interpolationResolution);
                        SceneView.RepaintAll();
                    }
                }
            }
            EditorGUILayout.EndVertical();
        }

        private void DrawBeamsTab()
        {
            int count = selectedBeams.Count;
            
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
                
                // Basic beam properties
                selectedCompliance = EditorGUILayout.FloatField("Compliance", selectedCompliance);
                
                EditorGUILayout.Separator();
                EditorGUILayout.LabelField("Deformation", EditorStyles.boldLabel);
                
                // Deformation parameters for selected beams
                defaultBeamDamping = EditorGUILayout.Slider("Damping", defaultBeamDamping, 0f, 1f);


                
                // NOTE: Area-based plasticity is now used instead of beam plasticity.
                // See ApplyAreaPlasticityPostCollision in Solver.cs
                
                EditorGUILayout.Space();
                
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply to Selected"))
                {
                    ApplyFullBeamSettings();
                }
                if (GUILayout.Button("Create Midpoints"))
                {
                    CreateMidpointsOnSelectedBeams();
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.HelpBox("Select beams in Scene View to edit.", MessageType.Warning);
            }
            
            EditorGUILayout.EndVertical();
            

            
            // Pressure
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);

            
            SerializedProperty pressureProp = serializedObject.FindProperty("internalPressure");
            if (pressureProp != null)
            {
                EditorGUI.BeginChangeCheck();
                EditorGUILayout.PropertyField(pressureProp, new GUIContent("Internal Pressure"));
                if (EditorGUI.EndChangeCheck())
                {
                    serializedObject.ApplyModifiedProperties();
                }
            }
            
            EditorGUILayout.EndVertical();
            
            EditorGUILayout.Space();
            
            // Beam tools
            EditorGUILayout.BeginHorizontal();
            
            GUI.enabled = count > 0;
            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            if (GUILayout.Button("Delete", GetRoundedButtonStyle())) DeleteSelectedBeams();
            GUI.backgroundColor = Color.white;
            GUI.enabled = true;

            // Disable Select All if no beams exist or if ALL behave selected
    var beams = workingTruss.GetTrussBeams();
    bool hasBeams = beams != null && beams.Count > 0;
    bool allBeamsSelected = hasBeams && selectedBeams.Count == beams.Count;

    GUI.enabled = hasBeams && !allBeamsSelected;
            if (GUILayout.Button("Select All", GetRoundedButtonStyle()))
            {
                 selectedBeams.Clear();
                 selectedNodes.Clear(); // Clear nodes when selecting all beams
                 for (int i = 0; i < beams.Count; i++) selectedBeams.Add(i);
                 UpdateSelectedBeamValues();
                 SceneView.RepaintAll();
            }
            GUI.enabled = true;
            
            GUI.enabled = count > 0;
            if (GUILayout.Button("Clear", GetRoundedButtonStyle())) { selectedBeams.Clear(); SceneView.RepaintAll(); }
            GUI.enabled = true;
            
            EditorGUILayout.EndHorizontal();
            
            if (isCreatingBeam)
            {
                EditorGUILayout.Space(5);
                EditorGUILayout.HelpBox("Click two nodes to connect them.", MessageType.None);
            }
            
            // Link Selected Nodes section
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Nodes linking", EditorStyles.boldLabel);
            useChainLinking = EditorGUILayout.ToggleLeft("Chain Link", useChainLinking);
            
            autoLinkDistance = EditorGUILayout.Slider("Max Distance", autoLinkDistance, 0f, 10f);
            
            GUI.enabled = selectedNodes.Count >= 2;
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Link Selected Nodes", GetRoundedButtonStyle()))
            {
                LinkSelectedNodes();
            }
            GUI.enabled = true;
            
            // Green Auto Link button - enabled when object has at least 2 nodes total
            bool hasEnoughNodes = workingTruss != null && workingTruss.NodePositions != null && workingTruss.NodePositions.Length >= 2;
            GUI.enabled = hasEnoughNodes;
            GUI.backgroundColor = new Color(0.3f, 0.9f, 0.3f);
            if (GUILayout.Button("Auto Link Nodes", GetRoundedButtonStyle()))
            {
                AutoConnectSelectedNodes();
            }
            GUI.backgroundColor = Color.white;
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();
            EditorGUILayout.EndVertical();
        }

        private void DrawFacesTab()
        {
            int count = selectedFaces.Count;

            // Selected face panel
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            string selInfo = count == 0 ? "No Faces Selected" : $"{count} Faces Selected";
            var style = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleLeft,
                normal = { textColor = count > 0 ? new Color(0.29f, 1f, 0.1f) : Color.gray }
            };
            EditorGUILayout.LabelField(selInfo, style);

            if (count > 0)
            {
                EditorGUILayout.Separator();

                selectedFacePlasticityThreshold = EditorGUILayout.Slider("Plasticity Threshold", selectedFacePlasticityThreshold, 0f, 1f);
                selectedFacePlasticityRate = EditorGUILayout.Slider("Plasticity Rate", selectedFacePlasticityRate, 0f, 2f);
                selectedFaceMaxDeformation = EditorGUILayout.Slider("Max Deformation", selectedFaceMaxDeformation, 0f, 1f);

                EditorGUILayout.Space();
                EditorGUILayout.BeginHorizontal();
                if (GUILayout.Button("Apply to Selected"))
                {
                    ApplyFaceSettingsToSelected();
                }
                EditorGUILayout.EndHorizontal();
            }
            else
            {
                EditorGUILayout.HelpBox("Select faces in Scene View to edit.", MessageType.Warning);
            }

            EditorGUILayout.EndVertical();

            // Face tools (Select All, Delete, Clear)
            EditorGUILayout.Space();
            EditorGUILayout.BeginHorizontal();

            var allFaces = workingTruss != null ? workingTruss.GetTrussFaces() : null;
            bool hasFaces = allFaces != null && allFaces.Count > 0;
            bool allFacesSelected = hasFaces && selectedFaces.Count == allFaces.Count;

            GUI.enabled = hasFaces && !allFacesSelected;
            if (GUILayout.Button("Select All", GetRoundedButtonStyle()))
            {
                selectedFaces.Clear();
                for (int i = 0; i < allFaces.Count; i++) selectedFaces.Add(i);
                UpdateSelectedFaceValues();
                SceneView.RepaintAll();
            }
            GUI.enabled = true;

            GUI.enabled = selectedFaces.Count > 0;
            GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
            if (GUILayout.Button("Delete", GetRoundedButtonStyle()))
            {
                DeleteSelectedFaces();
            }
            GUI.backgroundColor = Color.white;
            GUI.enabled = true;

            GUI.enabled = selectedFaces.Count > 0;
            if (GUILayout.Button("Clear", GetRoundedButtonStyle()))
            {
                selectedFaces.Clear();
                UpdateSelectedFaceValues();
                SceneView.RepaintAll();
            }
            GUI.enabled = true;

            EditorGUILayout.EndHorizontal();

            // Face generation tools
            EditorGUILayout.Space();
            EditorGUILayout.BeginVertical(EditorStyles.helpBox);
            EditorGUILayout.LabelField("Generate Faces", EditorStyles.boldLabel);
            EditorGUI.BeginDisabledGroup(Application.isPlaying || workingTruss == null);

            GUI.backgroundColor = new Color(0.3f, 0.9f, 0.3f);
            if (GUILayout.Button("Generate Faces from Beams", GUILayout.Height(25)))
            {
                if (EditorUtility.DisplayDialog("Generate Faces",
                    "This will generate new faces from existing beams.\n\nContinue?",
                    "Generate", "Cancel"))
                {
                    GenerateFacesFromBeams(false);
                    SceneView.RepaintAll();
                }
            }
            GUI.backgroundColor = Color.white;

            EditorGUI.EndDisabledGroup();
            if (Application.isPlaying)
            {
                EditorGUILayout.HelpBox("Face generation is only available in Edit Mode.", MessageType.Info);
            }
            EditorGUILayout.EndVertical();
        }



        private void DrawNodeSetsTab()
        {
            if (workingTruss == null) return;

            // Clean up invalid indices before display
            workingTruss.ValidateNodeSets();
            var nodeSets = workingTruss.GetNodeSets();

            EditorGUILayout.LabelField("Node Sets", EditorStyles.boldLabel);
            EditorGUILayout.Space(5);

            if (nodeSets.Count == 0)
            {
                EditorGUILayout.HelpBox("No node sets created yet.", MessageType.Info);
            }
            else
            {
                DrawNodeSetList(nodeSets);
            }

            EditorGUILayout.Space(10);
            DrawNodeSetCreation();

            EditorGUILayout.Space(10);
            DrawNodeSetVisualization();
        }

        private void DrawNodeSetList(List<NodeSet> nodeSets)
        {
            for (int i = 0; i < nodeSets.Count; i++)
            {
                var nodeSet = nodeSets[i];

                EditorGUILayout.BeginHorizontal();

                // Draw color indicator box
                Rect colorRect = GUILayoutUtility.GetRect(20, 20, GUILayout.Width(20), GUILayout.Height(20));
                EditorGUI.DrawRect(colorRect, nodeSet.color);

                EditorGUILayout.BeginVertical();
                EditorGUILayout.LabelField(nodeSet.name, EditorStyles.boldLabel);
                EditorGUILayout.LabelField($"{nodeSet.nodeIndices.Count} nodes", EditorStyles.miniLabel);
                EditorGUILayout.EndVertical();

                if (GUILayout.Button(selectedNodeSetIndex == i ? "Selected" : "Select", GetRoundedButtonStyle(), GUILayout.Width(80)))
                {
                    SelectNodeSet(i, nodeSet);
                }

                GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
                if (GUILayout.Button("X", GUILayout.Width(30)))
                {
                    DeleteNodeSet(nodeSet);
                }
                GUI.backgroundColor = Color.white;

                EditorGUILayout.EndHorizontal();

                if (selectedNodeSetIndex == i)
                {
                    DrawNodeSetEditor(nodeSet);
                }

                EditorGUILayout.Space(5);
            }
        }

        private void DrawNodeSetEditor(NodeSet nodeSet)
        {
            EditorGUI.indentLevel++;

            EditorGUI.BeginChangeCheck();
            nodeSet.name = EditorGUILayout.TextField("Name", nodeSet.name);
            nodeSet.color = EditorGUILayout.ColorField("Color", nodeSet.color);
            nodeSet.isVisible = EditorGUILayout.Toggle("Visible", nodeSet.isVisible);

            if (EditorGUI.EndChangeCheck())
            {
                EditorUtility.SetDirty(workingTruss);
            }

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add Selected", GetRoundedButtonStyle()) && selectedNodes.Count > 0)
            {
                AddNodesToSet(nodeSet);
            }

            if (GUILayout.Button("Remove Selected", GetRoundedButtonStyle()) && selectedNodes.Count > 0)
            {
                RemoveNodesFromSet(nodeSet);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUI.indentLevel--;
        }

        private void DrawNodeSetCreation()
        {
            EditorGUILayout.LabelField("Create New Node Set", EditorStyles.boldLabel);
            nodeSetCreationName = EditorGUILayout.TextField("Name", nodeSetCreationName);
            nodeSetCreationColor = EditorGUILayout.ColorField("Color", nodeSetCreationColor);

            GUI.enabled = selectedNodes.Count > 0 && !string.IsNullOrEmpty(nodeSetCreationName);
            if (GUILayout.Button($"Create from Selected ({selectedNodes.Count} nodes)", GetRoundedButtonStyle(), GUILayout.Height(30)))
            {
                CreateNodeSetFromSelection();
            }
            GUI.enabled = true;
        }

        private void DrawNodeSetVisualization()
        {
            EditorGUILayout.LabelField("Visualization", EditorStyles.boldLabel);
            showNodeSetLabels = EditorGUILayout.Toggle("Show Labels", showNodeSetLabels);
            if (showNodeSetLabels)
            {
                nodeSetLabelOffset = EditorGUILayout.Slider("Label Offset", nodeSetLabelOffset, 0f, 1f);
            }
        }

        private void DrawLinkSetsTab()
        {
            if (workingTruss == null) return;

            var linkSets = workingTruss.GetLinkSets();

            EditorGUILayout.LabelField("Link Sets", EditorStyles.boldLabel);
            EditorGUILayout.Space(5);

            if (linkSets.Count == 0)
            {
                EditorGUILayout.HelpBox("No link sets created yet.", MessageType.Info);
            }
            else
            {
                DrawLinkSetList(linkSets);
            }

            EditorGUILayout.Space(10);
            DrawLinkSetCreation();
        }

        private void DrawLinkSetList(List<LinkSet> linkSets)
        {
            for (int i = 0; i < linkSets.Count; i++)
            {
                var linkSet = linkSets[i];

                EditorGUILayout.BeginHorizontal();

                // Draw color indicator box
                Rect colorRect = GUILayoutUtility.GetRect(20, 20, GUILayout.Width(20), GUILayout.Height(20));
                EditorGUI.DrawRect(colorRect, linkSet.color);

                EditorGUILayout.BeginVertical();
                EditorGUILayout.LabelField(linkSet.name, EditorStyles.boldLabel);
                EditorGUILayout.LabelField($"{linkSet.linkIndices.Count} links", EditorStyles.miniLabel);
                EditorGUILayout.EndVertical();

                if (GUILayout.Button(selectedLinkSetIndex == i ? "Selected" : "Select", GetRoundedButtonStyle(), GUILayout.Width(80)))
                {
                    SelectLinkSet(i, linkSet);
                }

                GUI.backgroundColor = new Color(1f, 0.5f, 0.5f);
                if (GUILayout.Button("X", GUILayout.Width(30)))
                {
                    DeleteLinkSet(linkSet);
                }
                GUI.backgroundColor = Color.white;

                EditorGUILayout.EndHorizontal();

                if (selectedLinkSetIndex == i)
                {
                    DrawLinkSetEditor(linkSet);
                }

                EditorGUILayout.Space(5);
            }
        }

        private void DrawLinkSetEditor(LinkSet linkSet)
        {
            EditorGUI.indentLevel++;

            EditorGUI.BeginChangeCheck();
            linkSet.name = EditorGUILayout.TextField("Name", linkSet.name);
            linkSet.color = EditorGUILayout.ColorField("Color", linkSet.color);
            linkSet.isVisible = EditorGUILayout.Toggle("Visible", linkSet.isVisible);

            if (EditorGUI.EndChangeCheck())
            {
                EditorUtility.SetDirty(workingTruss);
            }

            EditorGUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add Selected", GetRoundedButtonStyle()) && selectedBeams.Count > 0)
            {
                AddLinksToSet(linkSet);
            }

            if (GUILayout.Button("Remove Selected", GetRoundedButtonStyle()) && selectedBeams.Count > 0)
            {
                RemoveLinksFromSet(linkSet);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUI.indentLevel--;
        }

        private void DrawLinkSetCreation()
        {
            EditorGUILayout.LabelField("Create New Link Set", EditorStyles.boldLabel);
            linkSetCreationName = EditorGUILayout.TextField("Name", linkSetCreationName);
            linkSetCreationColor = EditorGUILayout.ColorField("Color", linkSetCreationColor);

            GUI.enabled = selectedBeams.Count > 0 && !string.IsNullOrEmpty(linkSetCreationName);
            if (GUILayout.Button($"Create from Selected ({selectedBeams.Count} links)", GetRoundedButtonStyle(), GUILayout.Height(30)))
            {
                CreateLinkSetFromSelection();
            }
            GUI.enabled = true;
        }
        #endregion

        #region Scene Visualization
        private void DrawSceneVisualization()
        {
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;

            DrawNodes();
            DrawBeams();
            DrawNodeLabels(); // Draw labels
            if (showGhostNode && isCreatingNode) DrawGhostNode(ghostNodePosition);
        }

        private void DrawFaces()
        {
            if (workingTruss == null) return;
            var faces = workingTruss.GetTrussFaces();
            if (faces == null || faces.Count == 0) return;
            if (workingTruss.NodePositions == null || workingTruss.NodePositions.Length == 0) return;

            Camera sceneCamera = SceneView.lastActiveSceneView?.camera;
            Vector3 cameraPos = sceneCamera != null ? sceneCamera.transform.position : Vector3.zero;

            Color faceColor = new Color(0.2f, 0.6f, 1f, 0.15f);
            Color faceSelected = new Color(1f, 0.7f, 0.2f, 0.35f);
            Color outlineColor = new Color(0.2f, 0.6f, 1f, 0.6f);
            Color outlineSelected = new Color(1f, 0.7f, 0.2f, 0.8f);

            bool useRuntimePositions = Application.isPlaying &&
                                       targetSoftBody != null &&
                                       targetSoftBody.solver != null &&
                                       targetSoftBody.solver.nodeManager?.PredictedPositions != null;
            List<Vector3> runtimePositions = useRuntimePositions
                ? targetSoftBody.solver.nodeManager.PredictedPositions
                : null;

            var prevZTest = Handles.zTest;

            for (int i = 0; i < faces.Count; i++)
            {
                var f = faces[i];
                if (f.nodeA < 0 || f.nodeB < 0 || f.nodeC < 0) continue;
                if (useRuntimePositions)
                {
                    if (f.nodeA >= runtimePositions.Count ||
                        f.nodeB >= runtimePositions.Count ||
                        f.nodeC >= runtimePositions.Count)
                        continue;
                }
                else
                {
                    if (f.nodeA >= workingTruss.NodePositions.Length ||
                        f.nodeB >= workingTruss.NodePositions.Length ||
                        f.nodeC >= workingTruss.NodePositions.Length)
                        continue;
                }

                Vector3 a = useRuntimePositions
                    ? runtimePositions[f.nodeA]
                    : targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[f.nodeA]);
                Vector3 b = useRuntimePositions
                    ? runtimePositions[f.nodeB]
                    : targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[f.nodeB]);
                Vector3 c = useRuntimePositions
                    ? runtimePositions[f.nodeC]
                    : targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[f.nodeC]);

                Vector3 centroid = (a + b + c) / 3f;

                if (enableDistanceCulling)
                {
                    float distance = Vector3.Distance(centroid, cameraPos);
                    if (distance > maxFacesRenderDistance)
                        continue;
                }

                if (enableOcclusionCulling && sceneCamera != null)
                {
                    Vector3 viewportPoint = sceneCamera.WorldToViewportPoint(centroid);
                    if (viewportPoint.z < 0 || viewportPoint.x < 0 || viewportPoint.x > 1 || viewportPoint.y < 0 || viewportPoint.y > 1)
                        continue;
                }

                bool isSelected = selectedFaces.Contains(i);
                Color fill = isSelected ? faceSelected : faceColor;

                if (enableDepthCulling)
                {
                    Handles.zTest = UnityEngine.Rendering.CompareFunction.LessEqual;
                    Handles.color = fill;
                    Handles.DrawAAConvexPolygon(a, b, c);

                    if (showOccludedElements)
                    {
                        Handles.zTest = UnityEngine.Rendering.CompareFunction.Greater;
                        Color occludedFill = fill;
                        occludedFill.a *= occludedAlpha;
                        Handles.color = occludedFill;
                        Handles.DrawAAConvexPolygon(a, b, c);
                    }
                }
                else
                {
                    Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
                    Handles.color = fill;
                    Handles.DrawAAConvexPolygon(a, b, c);
                }

                // Always draw outlines on top for visibility
                Handles.zTest = UnityEngine.Rendering.CompareFunction.Always;
                Handles.color = isSelected ? outlineSelected : outlineColor;
                Handles.DrawLine(a, b);
                Handles.DrawLine(b, c);
                Handles.DrawLine(c, a);

                if (showGizmos && showRestAreaGizmos && Application.isPlaying && f.originalRestArea > 0f)
                {
                    float deformation = (f.restArea - f.originalRestArea) / f.originalRestArea;
                    if (Mathf.Abs(deformation) > 0.001f)
                    {
                        Vector3 normal = Vector3.Cross(b - a, c - a);
                        if (normal.sqrMagnitude > 1e-8f)
                        {
                            normal.Normalize();
                            float magnitude = Mathf.Clamp01(Mathf.Abs(deformation));
                            float size = HandleUtility.GetHandleSize(centroid) * 0.2f;
                            float lineLength = size * Mathf.Max(0.1f, magnitude);
                            Vector3 lineDir = normal * Mathf.Sign(deformation);

                            Color deformColor = deformation >= 0f
                                ? new Color(1f, 0.2f, 0.2f, 0.2f + 0.8f * magnitude)
                                : new Color(0.2f, 0.6f, 1f, 0.2f + 0.8f * magnitude);

                            Handles.color = deformColor;
                            Handles.DrawLine(centroid, centroid + lineDir * lineLength);

                            if (isSelected)
                            {
                                GUIStyle labelStyle = new GUIStyle(EditorStyles.miniLabel)
                                {
                                    normal = { textColor = deformColor }
                                };
                                Handles.Label(centroid + lineDir * lineLength * 1.1f,
                                    $"{deformation * 100f:F1}%", labelStyle);
                            }
                        }
                    }
                }
            }

            Handles.zTest = prevZTest;
        }

        private void DrawNodeLabels()
        {
            if (workingTruss.NodePositions == null) return;
            
            Camera sceneCamera = SceneView.lastActiveSceneView?.camera;
            Vector3 cameraPos = sceneCamera != null ? sceneCamera.transform.position : Vector3.zero;
            
            // Save current matrix and reset to identity to draw in World Space
            Matrix4x4 oldMatrix = Handles.matrix;
            Handles.matrix = Matrix4x4.identity;

            // Only draw labels for visible nodes that are close enough
            GUIStyle style = new GUIStyle(EditorStyles.miniLabel);
            style.normal.textColor = Color.white;
            style.alignment = TextAnchor.MiddleCenter;
            
            foreach (int i in visibleNodesAfterFilters)
            {
                // Extra distance check for text (usually much shorter than nodes)
                Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                
                if (enableDistanceCulling)
                {
                    if (Vector3.Distance(pos, cameraPos) > maxTextInfoRenderDistance)
                        continue;
                }
                
                // Depth occlusion check: skip labels for nodes hidden behind geometry
                if (!enableDepthCulling && sceneCamera != null)
                {
                    Vector3 dirToNode = pos - cameraPos;
                    float distToNode = dirToNode.magnitude;
                    if (Physics.Raycast(cameraPos, dirToNode.normalized, distToNode - 0.01f))
                        continue; // Something is blocking the view
                }
                
                Handles.Label(pos, i.ToString(), style);
            }
            
            // Restore matrix
            Handles.matrix = oldMatrix;
        }



        private void DrawNodes()
        {
            if (workingTruss.NodePositions == null || workingTruss.NodePositions.Length == 0) return;
            var pinnedNodes = workingTruss.PinnedNodes;
            
            // Clear visible nodes cache at the start
            visibleNodesAfterFilters.Clear();
            
            // Get camera position for distance culling
            Camera sceneCamera = SceneView.lastActiveSceneView?.camera;
            Vector3 cameraPos = sceneCamera != null ? sceneCamera.transform.position : Vector3.zero;
            
            // Pre-calculate which nodes pass the filters
            List<int> nodesToRender = new List<int>();
            
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                
                // Distance culling
                if (enableDistanceCulling)
                {
                    float distance = Vector3.Distance(pos, cameraPos);
                    if (distance > maxNodesRenderDistance)
                        continue;
                }
                
                // Occlusion culling (frustum check)
                if (enableOcclusionCulling && sceneCamera != null)
                {
                    Vector3 viewportPoint = sceneCamera.WorldToViewportPoint(pos);
                    // Check if point is in front of camera and within viewport bounds
                    if (viewportPoint.z < 0 || viewportPoint.x < 0 || viewportPoint.x > 1 || viewportPoint.y < 0 || viewportPoint.y > 1)
                        continue;
                }
                
                nodesToRender.Add(i);
                visibleNodesAfterFilters.Add(i);
            }

            // Helper to draw nodes with specific alpha (EXACT UniSoft style)
            void DrawNodePass(float alphaMultiplier)
            {
                foreach (int i in nodesToRender)
                {
                    Vector3 pos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                    float baseSize = HandleUtility.GetHandleSize(pos) * 0.12f; // Increased from 0.06 for better visibility
                    float size = baseSize;
                    
                    Color color = nodeColor;
                    if (pinnedNodes.Contains(i)) color = pinnedNodeColor;
                    else if (selectedNodes.Contains(i)) 
                    {
                        color = selectedNodeColor;
                        float pulse = Mathf.Sin((float)EditorApplication.timeSinceStartup * 6f) * 0.5f + 1.5f;
                        size *= pulse;
                    }
                    else
                    {
                        foreach (var set in workingTruss.GetNodeSets())
                        {
                            if (set.isVisible && set.ContainsNode(i))
                            {
                                color = set.color;
                                break;
                            }
                        }
                    }
                    color.a *= alphaMultiplier;
                    
                    // Use Handles.SphereHandleCap which properly respects Handles.zTest
                    Handles.color = color;
                    Handles.SphereHandleCap(0, pos, Quaternion.identity, size, EventType.Repaint);
                }
            }

            // Pass 1: Visible
            DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.LessEqual, () => DrawNodePass(1f));

            // Pass 2: Occluded (only when depth culling is enabled)
            if (enableDepthCulling)
            {
                DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.Greater, () => DrawNodePass(occludedAlpha));
            }

            // Hit detection is handled via Raycast in HandleSceneInput, so we don't need invisible Handles.Button calls.
            // This saves thousands of draw calls.
        }

        private void DrawNode(int index, List<int> pinnedNodes, float size, float alpha)
        {
            Vector3 position = workingTruss.NodePositions[index];
            Color color = GetNodeColor(index, pinnedNodes);
            color.a = alpha;

            if (isTransformingNode && transformingNodeIndex == index)
            {
                color = Color.yellow;
                size *= 1.2f;
            }

            Handles.color = color;
            Handles.SphereHandleCap(0, position, Quaternion.identity, size, EventType.Repaint);
        }

        private Color GetNodeColor(int index, List<int> pinnedNodes)
        {
            if (selectedNodes.Contains(index)) return selectedNodeColor;
            if (pinnedNodes.Contains(index)) return pinnedNodeColor;
            return nodeColor;
        }

        private void DrawBeams()
        {
            var beams = workingTruss.GetTrussBeams();
            if (beams == null || beams.Count == 0 || workingTruss.NodePositions == null) return;
            
            // Get camera position for distance culling
            Camera sceneCamera = SceneView.lastActiveSceneView?.camera;
            Vector3 cameraPos = sceneCamera != null ? sceneCamera.transform.position : Vector3.zero;
            
            // Clamp maxBeamsRenderDistance to not exceed maxNodesRenderDistance
            float effectiveBeamDistance = Mathf.Min(maxBeamsRenderDistance, maxNodesRenderDistance);
            
            // Pre-filter beams
            List<int> beamsToRender = new List<int>();
            
            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length)
                    continue;
                
                // Check if at least one node passed all filters (visible in the scene)
                bool nodeAVisible = visibleNodesAfterFilters.Contains(beam.nodeA);
                bool nodeBVisible = visibleNodesAfterFilters.Contains(beam.nodeB);
                
                if (!nodeAVisible && !nodeBVisible)
                    continue;
                
                // Distance culling for beams (check midpoint)
                if (enableDistanceCulling)
                {
                    Vector3 posA = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeA]);
                    Vector3 posB = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeB]);
                    Vector3 midpoint = (posA + posB) * 0.5f;
                    
                    float distance = Vector3.Distance(midpoint, cameraPos);
                    if (distance > effectiveBeamDistance)
                        continue;
                }
                
                beamsToRender.Add(i);
            }

            void DrawBeamPass(float alphaMultiplier)
            {
                foreach (int i in beamsToRender)
                {
                    var beam = beams[i];
                    
                    // Use TransformPoint to convert local to world (EXACTLY like UniSoft)
                    Vector3 posA = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeA]);
                    Vector3 posB = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[beam.nodeB]);

                    bool isSelected = selectedBeams.Contains(i);
                    Color color = isSelected ? selectedBeamColor : beamColor;

                    if (!isSelected)
                    {
                        foreach (var set in workingTruss.GetLinkSets())
                        {
                            if (set.isVisible && set.ContainsLink(i))
                            {
                                color = set.color;
                                break;
                            }
                        }
                    }
                    color.a *= alphaMultiplier;

                    Handles.color = color;
                    Handles.DrawLine(posA, posB, isSelected ? 3f : 1.5f);
                }
            }

            // Pass 1: Visible
            DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.LessEqual, () => DrawBeamPass(1f));

            // Pass 2: Occluded
            if (enableDepthCulling && showOccludedElements)
            {
                DrawWithDepthTest(UnityEngine.Rendering.CompareFunction.Greater, () => DrawBeamPass(occludedAlpha));
            }
        }
        
        private void HandleNodeClick(int nodeIndex)
        {
            Event e = Event.current;
            
            // Clear beams when selecting a node
            if (!e.shift && !e.control)
            {
                selectedBeams.Clear();
                selectedBeams.Clear();
            }
            
            if (e.control)
            {
                // Toggle selection
                if (selectedNodes.Contains(nodeIndex))
                    selectedNodes.Remove(nodeIndex);
                else
                    selectedNodes.Add(nodeIndex);
            }
            else if (e.shift)
            {
                // Add to selection
                if (!selectedNodes.Contains(nodeIndex))
                    selectedNodes.Add(nodeIndex);
            }
            else
            {
                // Single select
                selectedNodes.Clear();
                selectedNodes.Add(nodeIndex);
            }
            
            UpdateSelectionCenter();
            EditorUtility.SetDirty(softBody);
            Repaint();
        }



        private void DrawGhostNode(Vector3 worldPos)
        {
            // Draw in world space directly (like UniSoft) - no Handles.matrix manipulation!
            float pulse = Mathf.Sin((float)EditorApplication.timeSinceStartup * 10f) * 0.2f + 1.2f;
                    float size = HandleUtility.GetHandleSize(worldPos) * 0.16f * pulse;
            
            Handles.color = new Color(1f, 1f, 0f, 0.6f);
            Handles.SphereHandleCap(0, worldPos, Quaternion.identity, size, EventType.Repaint);
            
            GUIStyle labelStyle = new GUIStyle(EditorStyles.miniLabel) 
            { 
                normal = { textColor = Color.green }, 
                fontSize = 12, 
                fontStyle = FontStyle.Bold 
            };
            Handles.Label(worldPos + Vector3.up * size * 1.2f, "Click to Place", labelStyle);
        }

        private void DrawWithDepthTest(UnityEngine.Rendering.CompareFunction test, System.Action drawAction)
        {
            Handles.zTest = test;
            drawAction();
        }
        #endregion

        #region Scene Interaction
        private void HandleSceneInput(Event e, Vector3 mouseWorldPos)
        {
            // Don't handle input if transform gizmo is being used
            bool isUsingTransformHandle = currentTab == TAB_NODES && selectedNodes.Count > 0 &&
                                           !isCreatingNode && !isCreatingBeam &&
                                           (e.type == EventType.MouseDown || e.type == EventType.MouseDrag) &&
                                           GUIUtility.hotControl != 0;

            if (isUsingTransformHandle)
                return;

            switch (e.type)
            {
                case EventType.MouseMove:
                    if (isCreatingNode) e.Use();
                    break;

                case EventType.MouseDown:
                    if (e.button == 0 && !e.alt)
                    {
                        HandleMouseDown(e, mouseWorldPos);
                    }
                    break;

                case EventType.MouseDrag:
                    if (isTransformingNode && e.button == 0)
                    {
                        UpdateNodeTransform(mouseWorldPos);
                        e.Use();
                    }
                    break;

                case EventType.MouseUp:
                    if (e.button == 0)
                    {
                        EndNodeTransform();
                        e.Use();
                    }
                    break;

                case EventType.KeyDown:
                    if (e.keyCode == KeyCode.Escape)
                    {
                        CancelCurrentOperation();
                        e.Use();
                    }
                    else if (e.keyCode == KeyCode.Delete || e.keyCode == KeyCode.Backspace || e.keyCode == KeyCode.X)
                    {
                        ExecuteDeleteShortcut();
                        e.Use();
                    }
                    break;
            }
        }
        private void ExecuteDeleteShortcut()
        {
            // Delete based on current tab first, then fall back to any selection
            if (currentTab == TAB_FACES && selectedFaces.Count > 0)
            {
                DeleteSelectedFaces();
            }
            else if (currentTab == TAB_BEAMS && selectedBeams.Count > 0)
            {
                DeleteSelectedBeams();
            }
            else if (currentTab == TAB_NODES && selectedNodes.Count > 0)
            {
                DeleteSelectedNodes();
            }
            // Fallback: delete whatever is selected
            else if (selectedNodes.Count > 0) DeleteSelectedNodes();
            else if (selectedBeams.Count > 0) DeleteSelectedBeams();
            else if (selectedFaces.Count > 0) DeleteSelectedFaces();
        }

        private void HandleMouseDown(Event e, Vector3 mouseWorldPos)
        {
            bool multi = e.shift || e.control;

            if (isCreatingNode)
            {
                CreateNodeAtPosition(mouseWorldPos);
            }
            else if (isCreatingBeam)
            {
                HandleBeamCreation(mouseWorldPos);
            }
            else if (isCreatingBeam)
            {
                HandleBeamCreation(mouseWorldPos);
            }
            else
            {
                HandleSelection(mouseWorldPos, multi);
            }

            e.Use();
        }

        private void HandleSelection(Vector3 mouseWorldPos, bool multi)
        {
            bool hitNode = SelectNodeAtPosition(mouseWorldPos, multi);
            if (hitNode) return;

            bool hitBeam = SelectBeamAtPosition(mouseWorldPos, multi);
            if (hitBeam) return;

            // Optional: Face selection if desired global, or keep specific?
            // User only asked for Nodes and Beams global.
            // If we are in Face mode, we might want to prioritize Faces?
            // But for now, let's treat Faces as separate or secondary. 
            // If nothing hit:
            if (!multi)
            {
                selectedNodes.Clear();
                selectedBeams.Clear();
            }
        }

        private void HandleTransformGizmo()
        {
            Handles.matrix = Matrix4x4.identity;

            Vector3 center = targetSoftBody.transform.TransformPoint(selectionCenter);
            EditorGUI.BeginChangeCheck();
            center = Handles.PositionHandle(center, Quaternion.identity);
            if (EditorGUI.EndChangeCheck())
            {
                Vector3 localOffset = targetSoftBody.transform.InverseTransformPoint(center) - selectionCenter;
                MoveSelectedNodes(localOffset);
                UpdateSelectionCenter();
            }
            Handles.matrix = targetSoftBody.transform.localToWorldMatrix;
        }
        
        private void MoveSelectedNodes(Vector3 localOffset)
        {
            if (selectedNodes.Count == 0) return;
            
            foreach (int i in selectedNodes)
            {
                if (i >= 0 && i < workingTruss.NodePositions.Length)
                {
                    workingTruss.NodePositions[i] += localOffset;
                }
            }
            
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
            RecalculateRestLengthsForNodes(selectedNodes);
        }
        
        private void UpdateSelectionCenter()
        {
            if (selectedNodes.Count == 0)
            {
                selectionCenter = Vector3.zero;
                selectedMass = 1f; // Default value when nothing selected
                return;
            }
            
            Vector3 sum = Vector3.zero;
            int count = 0;
            float totalMass = 0f;
            
            foreach (int i in selectedNodes)
            {
                if (i >= 0 && i < workingTruss.NodePositions.Length)
                {
                    sum += workingTruss.NodePositions[i];
                    count++;
                    
                    // Get mass from truss
                    if (i < workingTruss.NodeMasses.Count)
                    {
                        totalMass += workingTruss.NodeMasses[i];
                    }
                    else
                    {
                        totalMass += 0.5f; // Default mass
                    }
                }
            }
            
            selectionCenter = count > 0 ? sum / count : Vector3.zero;
            selectedMass = count > 0 ? totalMass / count : 1f;
        }
        #endregion

        #region Core Operations
        private void UpdateDesignerModeFromTab()
        {
            // Cancel node creation if switching away from Nodes tab
            if (currentTab != TAB_NODES && isCreatingNode)
            {
                isCreatingNode = false;
            }
            if (currentTab != TAB_BEAMS && isCreatingBeam)
            {
                isCreatingBeam = false;
                beamStartNode = -1;
            }
        }

        private void ApplyTrussToTarget()
        {
            if (targetSoftBody == null || workingTruss == null) return;

            if (targetSoftBody.truss != workingTruss)
            {
                targetSoftBody.truss = workingTruss;
                EditorUtility.SetDirty(targetSoftBody);
            }

            if (Application.isPlaying && targetSoftBody.solver != null)
            {
                targetSoftBody.ApplyTrussAsset(workingTruss);
            }
        }


        #endregion

        #region Node Operations
        private void DuplicateSelectedNodes()
        {
            if (selectedNodes.Count == 0) return;

            // Store old indices before adding new nodes
            List<int> oldIndices = new List<int>(selectedNodes);
            List<int> newIndices = new List<int>();

            // Duplicate each selected node
            foreach (int oldIndex in oldIndices)
            {
                if (oldIndex >= workingTruss.NodePositions.Length) continue;

                // Get original node data
                Vector3 originalPos = workingTruss.NodePositions[oldIndex];
                float mass = oldIndex < workingTruss.NodeMasses.Count ? workingTruss.NodeMasses[oldIndex] : nodeCreationMass;
                bool isPinned = workingTruss.PinnedNodes.Contains(oldIndex);

                Vector3 newPos = originalPos;

                // Add to truss
                var positions = new List<Vector3>(workingTruss.NodePositions);
                positions.Add(newPos);
                workingTruss.SetNodePositions(positions.ToArray());

                var masses = new List<float>(workingTruss.NodeMasses);
                masses.Add(mass);
                workingTruss.SetNodeMasses(masses);

                // Pin the new node if original was pinned
                if (isPinned)
                {
                    var pinnedNodes = new List<int>(workingTruss.PinnedNodes);
                    pinnedNodes.Add(positions.Count - 1);
                    workingTruss.SetPinnedNodes(pinnedNodes);
                }

                newIndices.Add(positions.Count - 1);
            }

            // Select the newly created nodes
            selectedNodes.Clear();
            selectedNodes.AddRange(newIndices);
            UpdateSelectionCenter();

            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();

            SceneView.RepaintAll();
            Repaint();
        }
        private void CreateNodeAtPosition(Vector3 worldPos)
        {
            Vector3 local = targetSoftBody.transform.InverseTransformPoint(worldPos);
            var list = new List<Vector3>(workingTruss.NodePositions ?? new Vector3[0]);
            list.Add(local);
            workingTruss.SetNodePositions(list.ToArray());

            var masses = new List<float>(workingTruss.NodeMasses);
            masses.Add(nodeCreationMass);
            workingTruss.SetNodeMasses(masses);

            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
        }

        private bool SelectNodeAtPosition(Vector3 worldPos, bool multi)
        {
            int idx = FindClosestNode(worldPos);
            if (idx == -1)
            {
                return false;
            }

            if (!multi)
            {
                selectedNodes.Clear();
                selectedBeams.Clear(); // Clear beams when selecting a node
            }

            if (selectedNodes.Contains(idx))
                selectedNodes.Remove(idx);
            else
                selectedNodes.Add(idx);

            UpdateSelectionCenter();
            return true;
        }

        private void SetNodePosition(int idx, Vector3 pos)
        {
            workingTruss.NodePositions[idx] = pos;
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
            RecalculateRestLengthsForNodes(new int[] { idx });
        }

        private void UpdateNodeTransform(Vector3 worldPos)
        {
            if (transformingNodeIndex != -1)
            {
                workingTruss.NodePositions[transformingNodeIndex] = targetSoftBody.transform.InverseTransformPoint(worldPos);
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
                RecalculateRestLengthsForNodes(new int[] { transformingNodeIndex });
            }
        }

        private void EndNodeTransform()
        {
            isTransformingNode = false;
            transformingNodeIndex = -1;
        }

        private void PinSelectedNodes(bool pin)
        {
            var list = new List<int>(workingTruss.PinnedNodes);
            foreach (int i in selectedNodes)
            {
                if (pin && !list.Contains(i))
                    list.Add(i);
                else if (!pin && list.Contains(i))
                    list.Remove(i);
            }
            workingTruss.SetPinnedNodes(list);
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
        }

        private void SetSelectedNodesMass(float mass)
        {
            foreach (int i in selectedNodes)
            {
                if (i < workingTruss.NodeMasses.Count)
                    workingTruss.NodeMasses[i] = mass;
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void DeleteSelectedNodes()
        {
            if (selectedNodes.Count == 0) return;

            if (!EditorUtility.DisplayDialog("Delete Nodes",
                $"Delete {selectedNodes.Count} node(s)?\n\n" +
                "This will also remove:\n" +
                " - All beams connected to these nodes\n" +
                " - All faces using these nodes\n" +
                " - References in node/link sets\n\n" +
                "This operation cannot be undone.",
                "Delete", "Cancel"))
            {
                return;
            }

            var nodesToDelete = new HashSet<int>(selectedNodes);
            Dictionary<int, int> oldToNewIndex = BuildIndexMapping(nodesToDelete);

            RemoveNodesFromTruss(nodesToDelete);
            UpdateNodeMasses(nodesToDelete);
            UpdatePinnedNodes(nodesToDelete, oldToNewIndex);
            var removedBeamIndices = UpdateBeams(nodesToDelete, oldToNewIndex);
            UpdateFaces(nodesToDelete, oldToNewIndex);
            UpdateNodeSetsAfterDeletion(nodesToDelete, oldToNewIndex);
            UpdateLinkSetsAfterDeletion(removedBeamIndices);

            selectedNodes.Clear();
            selectedBeams.Clear();

            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();

            SceneView.RepaintAll();
            Repaint();
        }

        private Dictionary<int, int> BuildIndexMapping(HashSet<int> nodesToDelete)
        {
            Dictionary<int, int> oldToNewIndex = new Dictionary<int, int>();
            int offset = 0;
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                if (nodesToDelete.Contains(i))
                {
                    offset++;
                }
                else
                {
                    oldToNewIndex[i] = i - offset;
                }
            }
            return oldToNewIndex;
        }

        private void RemoveNodesFromTruss(HashSet<int> nodesToDelete)
        {
            var newPositions = new List<Vector3>();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                if (!nodesToDelete.Contains(i))
                {
                    newPositions.Add(workingTruss.NodePositions[i]);
                }
            }
            workingTruss.SetNodePositions(newPositions.ToArray());
        }

        private void UpdateNodeMasses(HashSet<int> nodesToDelete)
        {
            var newMasses = new List<float>();
            for (int i = 0; i < workingTruss.NodeMasses.Count; i++)
            {
                if (!nodesToDelete.Contains(i))
                {
                    newMasses.Add(workingTruss.NodeMasses[i]);
                }
            }
            workingTruss.SetNodeMasses(newMasses);
        }

        private void UpdatePinnedNodes(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var newPinnedNodes = new List<int>();
            foreach (int pinnedNode in workingTruss.PinnedNodes)
            {
                if (!nodesToDelete.Contains(pinnedNode))
                {
                    newPinnedNodes.Add(oldToNewIndex[pinnedNode]);
                }
            }
            workingTruss.SetPinnedNodes(newPinnedNodes);
        }

        private HashSet<int> UpdateBeams(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var beams = workingTruss.GetTrussBeams();
            var validBeams = new List<Beam>();
            var removedBeamIndices = new HashSet<int>();

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                bool nodeADeleted = nodesToDelete.Contains(beam.nodeA);
                bool nodeBDeleted = nodesToDelete.Contains(beam.nodeB);

                if (!nodeADeleted && !nodeBDeleted)
                {
                    beam.nodeA = oldToNewIndex[beam.nodeA];
                    beam.nodeB = oldToNewIndex[beam.nodeB];
                    validBeams.Add(beam);
                }
                else
                {
                    removedBeamIndices.Add(i);
                }
            }
            workingTruss.SetBeams(validBeams);
            return removedBeamIndices;
        }

        private void UpdateFaces(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var faces = workingTruss.GetTrussFaces();
            for (int i = faces.Count - 1; i >= 0; i--)
            {
                var face = faces[i];
                bool anyNodeDeleted = nodesToDelete.Contains(face.nodeA) ||
                                     nodesToDelete.Contains(face.nodeB) ||
                                     nodesToDelete.Contains(face.nodeC);

                if (anyNodeDeleted)
                {
                    faces.RemoveAt(i);
                }
                else
                {
                    face.nodeA = oldToNewIndex[face.nodeA];
                    face.nodeB = oldToNewIndex[face.nodeB];
                    face.nodeC = oldToNewIndex[face.nodeC];
                    faces[i] = face;
                }
            }
        }

        private void UpdateNodeSetsAfterDeletion(HashSet<int> nodesToDelete, Dictionary<int, int> oldToNewIndex)
        {
            var nodeSets = workingTruss.GetNodeSets();
            foreach (var nodeSet in nodeSets)
            {
                var updatedIndices = new List<int>();
                foreach (int nodeIndex in nodeSet.nodeIndices)
                {
                    if (!nodesToDelete.Contains(nodeIndex))
                    {
                        updatedIndices.Add(oldToNewIndex[nodeIndex]);
                    }
                }
                nodeSet.nodeIndices = updatedIndices;
            }
            workingTruss.ValidateNodeSets();
        }

        private void UpdateLinkSetsAfterDeletion(HashSet<int> removedBeamIndices)
        {
            var linkSets = workingTruss.GetLinkSets();
            foreach (var linkSet in linkSets)
            {
                var updatedIndices = new List<int>();
                foreach (int linkIndex in linkSet.linkIndices)
                {
                    if (!removedBeamIndices.Contains(linkIndex))
                    {
                        int newIndex = linkIndex - removedBeamIndices.Count(removed => removed < linkIndex);
                        updatedIndices.Add(newIndex);
                    }
                }
                linkSet.linkIndices = updatedIndices;
            }
            workingTruss.ValidateLinkSets();
        }

        private void SelectAllNodes()
        {
            selectedNodes.Clear();
            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
                selectedNodes.Add(i);
        }

        #endregion

        #region Beam Operations
        private void HandleBeamCreation(Vector3 worldPos)
        {
            int idx = FindClosestNode(worldPos);
            if (idx == -1) return;

            if (beamStartNode == -1)
            {
                beamStartNode = idx;
            }
            else if (beamStartNode != idx)
            {
                CreateBeam(beamStartNode, idx);
                beamStartNode = -1;
                isCreatingBeam = false;
            }
        }

        private void CreateBeam(int a, int b)
        {
            Vector3 pA = workingTruss.NodePositions[a];
            Vector3 pB = workingTruss.NodePositions[b];
            workingTruss.GetTrussBeams().Add(new Beam(
                a, b,
                defaultBeamCompliance,
                defaultBeamDamping,
                Vector3.Distance(pA, pB)
            ));
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
        }

        private bool SelectBeamAtPosition(Vector3 worldPos, bool multi)
        {
            var beams = workingTruss.GetTrussBeams();

            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            float minDistance = 0.5f;
            int closestBeam = -1;

            for (int i = 0; i < beams.Count; i++)
            {
                var beam = beams[i];
                if (beam.nodeA >= workingTruss.NodePositions.Length || beam.nodeB >= workingTruss.NodePositions.Length)
                    continue;

                Vector3 localA = workingTruss.NodePositions[beam.nodeA];
                Vector3 localB = workingTruss.NodePositions[beam.nodeB];

                Vector3 worldA = targetSoftBody.transform.TransformPoint(localA);
                Vector3 worldB = targetSoftBody.transform.TransformPoint(localB);

                Vector3 lineDir = worldB - worldA;
                float lineLength = lineDir.magnitude;
                // Simple distance check
                float distance = HandleUtility.DistancePointToLineSegment(Event.current.mousePosition, 
                    HandleUtility.WorldToGUIPoint(worldA), HandleUtility.WorldToGUIPoint(worldB));

                // Previous implementation used DistanceFromRayToLineSegment which was custom. 
                // Let's stick to previous custom method if it worked, or use HandleUtility for better UX?
                // The previous method was using 'ray', but signature takes 'worldPos' which isn't used much except by caller.
                // Caller passed mouseWorldPos... wait.
                // Let's revert to using the custom DistanceFromRayToLineSegment to ensure behavior is consistent unless broken.
                // Re-reading previous code... it used DistanceFromRayToLineSegment(ray, worldA, worldB).
                
                distance = DistanceFromRayToLineSegment(ray, worldA, worldB);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestBeam = i;
                }
            }

            if (closestBeam != -1)
            {
                if (!multi) 
                {
                    selectedBeams.Clear();
                    selectedNodes.Clear(); // Clear nodes when selecting a beam

                }

                if (selectedBeams.Contains(closestBeam))
                    selectedBeams.Remove(closestBeam);
                else
                    selectedBeams.Add(closestBeam);
                
                // Update displayed values to show sum of selected beams
                UpdateSelectedBeamValues();
                
                return true;
            }
            
            return false;
        }

        private float DistanceFromRayToLineSegment(Ray ray, Vector3 lineStart, Vector3 lineEnd)
        {
            Vector3 lineDir = lineEnd - lineStart;
            float lineLength = lineDir.magnitude;

            if (lineLength < 0.0001f)
            {
                return Vector3.Cross(ray.direction, lineStart - ray.origin).magnitude;
            }

            lineDir /= lineLength;

            Vector3 rayToLineStart = lineStart - ray.origin;
            Vector3 rayToLineEnd = lineEnd - ray.origin;

            float rayDotLine = Vector3.Dot(ray.direction, lineDir);
            float t1 = Vector3.Dot(rayToLineStart, ray.direction);
            float t2 = Vector3.Dot(rayToLineStart, lineDir);

            float denom = 1.0f - rayDotLine * rayDotLine;

            float s, t;
            if (Mathf.Abs(denom) < 0.0001f)
            {
                s = 0;
                t = t2;
            }
            else
            {
                s = (t1 - t2 * rayDotLine) / denom;
                t = (t1 * rayDotLine - t2) / denom;
            }

            t = Mathf.Clamp(t, 0, lineLength);

            Vector3 closestPointOnRay = ray.origin + ray.direction * s;
            Vector3 closestPointOnLine = lineStart + lineDir * t;

            return Vector3.Distance(closestPointOnRay, closestPointOnLine);
        }

        private void AutoConnectSelectedNodes()
        {
            var nodes = workingTruss.NodePositions;
            var beams = workingTruss.GetTrussBeams();
            int created = 0;

            // Always work on ALL nodes as requested
            int nodeCount = nodes.Length;
            if (nodeCount < 2) return;

            Undo.RecordObject(workingTruss, "Auto Link Nodes");

            if (useChainLinking)
            {
                // Chain Link: Sequential 0-1, 1-2, ...
                for (int i = 0; i < nodeCount - 1; i++)
                {
                    int nodeA = i;
                    int nodeB = i + 1;
                    
                    Vector3 pA = nodes[nodeA];
                    Vector3 pB = nodes[nodeB];
                    float dist = Vector3.Distance(pA, pB);
                    
                    // Distance check! (0 = infinite)
                    if (autoLinkDistance > 0f && dist > autoLinkDistance) continue;
                    
                    bool exists = beams.Any(b => (b.nodeA == nodeA && b.nodeB == nodeB) ||
                                                 (b.nodeA == nodeB && b.nodeB == nodeA));
                    if (!exists)
                    {
                        beams.Add(new Beam(nodeA, nodeB, defaultBeamCompliance,
                                         defaultBeamDamping, dist));
                        created++;
                    }
                }
            }
            else
            {
                // Standard: All-to-All (Warning: Heavy!)
                for (int i = 0; i < nodeCount; i++)
                {
                    for (int j = i + 1; j < nodeCount; j++)
                    {
                        int nodeA = i;
                        int nodeB = j;
                        
                        Vector3 pA = nodes[nodeA];
                        Vector3 pB = nodes[nodeB];
                        float dist = Vector3.Distance(pA, pB);
                        
                        // Distance check! (0 = infinite)
                        if (autoLinkDistance > 0f && dist > autoLinkDistance) continue;
                        
                        bool exists = beams.Any(b => (b.nodeA == nodeA && b.nodeB == nodeB) ||
                                                     (b.nodeA == nodeB && b.nodeB == nodeA));
                        if (!exists)
                        {
                            beams.Add(new Beam(nodeA, nodeB, defaultBeamCompliance,
                                             defaultBeamDamping, dist));
                            created++;
                        }
                    }
                }
            }

            if (created > 0)
            {
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
                Debug.Log($"Auto-linked {created} beams (Chain: {useChainLinking})");
            }
        }
        private void DeleteSelectedBeams()
    {
        if (selectedBeams.Count == 0) return;

        if (!EditorUtility.DisplayDialog("Delete Beams", 
            $"Delete {selectedBeams.Count} beam(s)?\nTHIS CANNOT BE UNDONE", "Delete", "Cancel"))
        {
            return;
        }

        Undo.RecordObject(workingTruss, "Delete Beams");
        var beams = workingTruss.GetTrussBeams();
            selectedBeams.Sort();
            for (int i = selectedBeams.Count - 1; i >= 0; i--)
            {
                int idx = selectedBeams[i];
                if (idx < beams.Count) beams.RemoveAt(idx);
            }
            selectedBeams.Clear();
            UpdateSelectedBeamValues();
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
        }

        private void SelectAllBeams()
        {
            selectedBeams.Clear();
            for (int i = 0; i < workingTruss.GetTrussBeams().Count; i++)
                selectedBeams.Add(i);
        }
        #endregion



        #region Node Set Operations
        private void CreateNodeSetFromSelection()
        {
            if (selectedNodes.Count == 0 || string.IsNullOrEmpty(nodeSetCreationName))
                return;

            var nodeSet = new NodeSet(nodeSetCreationName, selectedNodes)
            {
                color = nodeSetCreationColor,
                isVisible = true
            };

            workingTruss.AddNodeSet(nodeSet);
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();

            nodeSetCreationName = "New Node Set";
            selectedNodeSetIndex = workingTruss.GetNodeSets().Count - 1;
        }

        private void SelectNodeSet(int index, NodeSet nodeSet)
        {
            selectedNodeSetIndex = index;
            selectedLinkSetIndex = -1; // Deselect Link Set index
            selectedBeams.Clear(); // Deselect selected beams
            selectedNodes.Clear();
            selectedNodes.AddRange(nodeSet.nodeIndices);
            SceneView.RepaintAll();
        }

        private void DeleteNodeSet(NodeSet nodeSet)
        {
            if (EditorUtility.DisplayDialog("Delete Node Set",
                $"Delete node set '{nodeSet.name}'?", "Delete", "Cancel"))
            {
                workingTruss.RemoveNodeSet(nodeSet.name);
                selectedNodeSetIndex = -1;
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
            }
        }

        private void AddNodesToSet(NodeSet nodeSet)
        {
            foreach (int nodeIdx in selectedNodes)
            {
                if (!nodeSet.nodeIndices.Contains(nodeIdx))
                    nodeSet.nodeIndices.Add(nodeIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void RemoveNodesFromSet(NodeSet nodeSet)
        {
            foreach (int nodeIdx in selectedNodes)
            {
                nodeSet.nodeIndices.Remove(nodeIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }
        #endregion

        #region Link Set Operations
        private void CreateLinkSetFromSelection()
        {
            if (selectedBeams.Count == 0 || string.IsNullOrEmpty(linkSetCreationName))
                return;

            var linkSet = new LinkSet(linkSetCreationName, selectedBeams)
            {
                color = linkSetCreationColor,
                isVisible = true
            };

            workingTruss.AddLinkSet(linkSet);
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();

            linkSetCreationName = "New Link Set";
            selectedLinkSetIndex = workingTruss.GetLinkSets().Count - 1;
        }

        private void SelectLinkSet(int index, LinkSet linkSet)
        {
            selectedLinkSetIndex = index;
            selectedNodeSetIndex = -1; // Deselect Node Set index
            selectedNodes.Clear(); // Deselect selected nodes
            selectedBeams.Clear();
            selectedBeams.AddRange(linkSet.linkIndices);
            SceneView.RepaintAll();
        }

        private void DeleteLinkSet(LinkSet linkSet)
        {
            if (EditorUtility.DisplayDialog("Delete Link Set",
                $"Delete link set '{linkSet.name}'?", "Delete", "Cancel"))
            {
                workingTruss.RemoveLinkSet(linkSet.name);
                selectedLinkSetIndex = -1;
                EditorUtility.SetDirty(workingTruss);
                ApplyTrussToTarget();
            }
        }

        private void AddLinksToSet(LinkSet linkSet)
        {
            foreach (int linkIdx in selectedBeams)
            {
                if (!linkSet.linkIndices.Contains(linkIdx))
                    linkSet.linkIndices.Add(linkIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }

        private void RemoveLinksFromSet(LinkSet linkSet)
        {
            foreach (int linkIdx in selectedBeams)
            {
                linkSet.linkIndices.Remove(linkIdx);
            }
            EditorUtility.SetDirty(workingTruss);
        }
        #endregion

        #region Utility Methods
        private Vector3? GetWorldPositionFromMouse(Vector2 mousePosition)
        {
            Ray ray = HandleUtility.GUIPointToWorldRay(mousePosition);

            // Try to hit the mesh directly (works without collider)
            MeshFilter meshFilter = targetSoftBody.GetComponent<MeshFilter>();
            if (meshFilter != null && meshFilter.sharedMesh != null)
            {
                // Transform ray to local space for mesh intersection
                Matrix4x4 worldToLocal = targetSoftBody.transform.worldToLocalMatrix;
                Ray localRay = new Ray(
                    worldToLocal.MultiplyPoint(ray.origin),
                    worldToLocal.MultiplyVector(ray.direction).normalized
                );
                
                if (IntersectRayMesh(localRay, meshFilter.sharedMesh, out RaycastHit meshHit))
                {
                    return targetSoftBody.transform.TransformPoint(meshHit.point);
                }
            }

            // Fallback: try collider if available
            if (isCreatingNode)
            {
                Collider softBodyCollider = targetSoftBody.GetComponent<Collider>();
                if (softBodyCollider != null && softBodyCollider.Raycast(ray, out RaycastHit hit, Mathf.Infinity))
                {
                    return hit.point;
                }
            }

            // No valid hit on mesh - return null to indicate ghost should not be shown
            return null;
        }

        private bool IntersectRayMesh(Ray ray, Mesh mesh, out RaycastHit hit)
        {
            hit = new RaycastHit();
            
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            
            float closestDist = float.MaxValue;
            bool hitFound = false;
            
            for (int i = 0; i < triangles.Length; i += 3)
            {
                Vector3 v0 = vertices[triangles[i]];
                Vector3 v1 = vertices[triangles[i + 1]];
                Vector3 v2 = vertices[triangles[i + 2]];
                
                if (RayTriangleIntersect(ray, v0, v1, v2, out float t, out Vector3 hitPoint))
                {
                    if (t > 0 && t < closestDist)
                    {
                        closestDist = t;
                        hit.point = hitPoint;
                        hit.distance = t;
                        hit.normal = Vector3.Cross(v1 - v0, v2 - v0).normalized;
                        hitFound = true;
                    }
                }
            }
            
            return hitFound;
        }

        private bool RayTriangleIntersect(Ray ray, Vector3 v0, Vector3 v1, Vector3 v2, out float t, out Vector3 hitPoint)
        {
            t = 0;
            hitPoint = Vector3.zero;
            
            Vector3 edge1 = v1 - v0;
            Vector3 edge2 = v2 - v0;
            Vector3 h = Vector3.Cross(ray.direction, edge2);
            float a = Vector3.Dot(edge1, h);
            
            if (a > -0.00001f && a < 0.00001f)
                return false;
            
            float f = 1.0f / a;
            Vector3 s = ray.origin - v0;
            float u = f * Vector3.Dot(s, h);
            
            if (u < 0.0f || u > 1.0f)
                return false;
            
            Vector3 q = Vector3.Cross(s, edge1);
            float v = f * Vector3.Dot(ray.direction, q);
            
            if (v < 0.0f || u + v > 1.0f)
                return false;
            
            t = f * Vector3.Dot(edge2, q);
            
            if (t > 0.00001f)
            {
                hitPoint = ray.origin + ray.direction * t;
                return true;
            }
            
            return false;
        }

        private int FindClosestNode(Vector3 worldPos)
        {
            if (workingTruss.NodePositions == null) return -1;

            Ray ray = HandleUtility.GUIPointToWorldRay(Event.current.mousePosition);
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < workingTruss.NodePositions.Length; i++)
            {
                Vector3 nodeWorldPos = targetSoftBody.transform.TransformPoint(workingTruss.NodePositions[i]);
                float distance = Vector3.Cross(ray.direction, nodeWorldPos - ray.origin).magnitude;
                
                // Use fixed hit radius (matching beam selection behavior)
                float hitRadius = 0.5f;

                if (distance < hitRadius && distance < minDistance)
                {
                    float t = Vector3.Dot(nodeWorldPos - ray.origin, ray.direction);
                    if (t > 0 && t < 1000f)
                    {
                        minDistance = distance;
                        closestNode = i;
                    }
                }
            }

            return closestNode;
        }

        private void ToggleNodeCreation()
        {
            // If beams are selected, create midpoints on them instead of entering creation mode
            if (selectedBeams.Count > 0)
            {
                CreateMidpointsOnSelectedBeams();
                return;
            }
            
            isCreatingNode = !isCreatingNode;
        }

        private void CreateMidpointsOnSelectedBeams()
        {
            if (workingTruss == null || selectedBeams.Count == 0) return;
            
            Undo.RecordObject(workingTruss, "Split Beams at Midpoint");
            
            var beams = workingTruss.GetTrussBeams();
            var nodePositions = workingTruss.NodePositions.ToList();
            var nodeMasses = workingTruss.NodeMasses.ToList();
            List<int> newNodeIndices = new List<int>();
            List<Beam> newBeams = new List<Beam>();
            
            // Sort in descending order to safely remove beams from the end
            var sortedBeamIndices = selectedBeams.OrderByDescending(x => x).ToList();
            
            foreach (int beamIdx in sortedBeamIndices)
            {
                if (beamIdx < 0 || beamIdx >= beams.Count) continue;
                
                var originalBeam = beams[beamIdx];
                if (originalBeam.nodeA >= nodePositions.Count || originalBeam.nodeB >= nodePositions.Count) continue;
                
                // Calculate midpoint position
                Vector3 posA = nodePositions[originalBeam.nodeA];
                Vector3 posB = nodePositions[originalBeam.nodeB];
                Vector3 midpoint = (posA + posB) * 0.5f;
                
                // Add new node at midpoint
                int midpointNodeIdx = nodePositions.Count;
                nodePositions.Add(midpoint);
                nodeMasses.Add(nodeCreationMass);
                newNodeIndices.Add(midpointNodeIdx);
                
                // Create two new beams: Aв†’midpoint and midpointв†’B
                float halfRestLength = originalBeam.restLength * 0.5f;
                
                Beam beamAtoMid = new Beam(
                    originalBeam.nodeA, 
                    midpointNodeIdx, 
                    originalBeam.compliance, 
                    originalBeam.damping, 
                    halfRestLength
                );
                
                Beam beamMidtoB = new Beam(
                    midpointNodeIdx, 
                    originalBeam.nodeB, 
                    originalBeam.compliance, 
                    originalBeam.damping, 
                    halfRestLength
                );
                
                newBeams.Add(beamAtoMid);
                newBeams.Add(beamMidtoB);
                
                // Remove original beam
                beams.RemoveAt(beamIdx);
            }
            
            // Add new beams
            beams.AddRange(newBeams);
            
            // Apply changes
            workingTruss.SetNodePositions(nodePositions.ToArray());
            workingTruss.SetNodeMasses(nodeMasses);
            
            // Select the newly created nodes
            selectedNodes.Clear();
            selectedNodes.AddRange(newNodeIndices);
            selectedBeams.Clear();
            
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
            
            SceneView.RepaintAll();
            Repaint();
        }

        private void ToggleBeamCreation()
        {
            isCreatingBeam = !isCreatingBeam;
        }

        private void CancelCurrentOperation()
        {
            isCreatingNode = false;
            isCreatingBeam = false;

            beamStartNode = -1;
        }
        private void DrawSettingsTab()
        {
            // Environment (Gravity)
            showEnvironment = EditorGUILayout.Foldout(showEnvironment, "Environment", true, EditorStyles.boldLabel);
            if (showEnvironment)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                softBody.gravity = EditorGUILayout.Vector3Field("Gravity", softBody.gravity);
                EditorGUILayout.EndVertical();
            }
            EditorGUILayout.Space();

            // Deformation Section
            showDeformation = EditorGUILayout.Foldout(showDeformation, "Deformation", true, EditorStyles.boldLabel);
            if (showDeformation)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                // Preset
                MatterPreset newPreset = (MatterPreset)EditorGUILayout.EnumPopup("Preset", currentPreset);
                
                if (newPreset != currentPreset)
                {
                    if (newPreset == MatterPreset.Metal)
                    {
                        softBody.enableDeformation = true;
                        softBody.enablePlasticity = true;
                        defaultBeamCompliance = 0.0001f;
                        defaultBeamDamping = 0.1f;
                        currentPreset = MatterPreset.Custom;
                    }
                    else if (newPreset == MatterPreset.Rubber)
                    {
                        softBody.enableDeformation = true;
                        softBody.enablePlasticity = false;
                        defaultBeamCompliance = 0.05f;
                        defaultBeamDamping = 0.8f;
                        currentPreset = MatterPreset.Custom;
                    }
                    else
                    {
                        currentPreset = MatterPreset.Custom;
                    }
                    EditorUtility.SetDirty(targetSoftBody);
                }
                EditorGUILayout.Space();

                softBody.enableDeformation = EditorGUILayout.Toggle("Enable Deformation", softBody.enableDeformation);
                softBody.enablePlasticity = EditorGUILayout.Toggle("Enable Plasticity", softBody.enablePlasticity);
                softBody.areaCompliance = EditorGUILayout.Slider("Area Compliance", softBody.areaCompliance, 0f, 0.1f);

                
                EditorGUILayout.Separator();
                EditorGUILayout.LabelField("Beam Defaults", EditorStyles.boldLabel);
                defaultBeamCompliance = EditorGUILayout.Slider("Compliance", defaultBeamCompliance, 0f, 2f);
                defaultBeamDamping = EditorGUILayout.Slider("Damping", defaultBeamDamping, 0f, 1f);
                
                EditorGUILayout.Space();
                GUI.backgroundColor = new Color(0.5f, 0.8f, 1f);
                if (GUILayout.Button("Apply To All Beams", GUILayout.Height(25)))
                {
                    ApplySettingsToAllBeams();
                }
                GUI.backgroundColor = Color.white;

                EditorGUILayout.EndVertical();
            }
            EditorGUILayout.Space();

            // Collision
            showCollision = EditorGUILayout.Foldout(showCollision, "Collision", true, EditorStyles.boldLabel);
            if (showCollision)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                softBody.enableCollision = EditorGUILayout.Toggle("Enable Collision", softBody.enableCollision);
                
                EditorGUI.BeginDisabledGroup(!softBody.enableCollision);
                softBody.collisionRadius = EditorGUILayout.FloatField("Collision Radius", softBody.collisionRadius);
                
                LayerMask tempMask = EditorGUILayout.MaskField("Collision Layers", UnityEditorInternal.InternalEditorUtility.LayerMaskToConcatenatedLayersMask(softBody.collisionLayers), UnityEditorInternal.InternalEditorUtility.layers);
                softBody.collisionLayers = UnityEditorInternal.InternalEditorUtility.ConcatenatedLayersMaskToLayerMask(tempMask);
                
                EditorGUILayout.Separator();
                // Self-Collision (V2V within same body)
                softBody.enableSelfCollision = EditorGUILayout.Toggle("Enable Self-Collision", softBody.enableSelfCollision);
                if (softBody.enableSelfCollision)
                {
                    EditorGUILayout.HelpBox("Nodes will collide with triangles of the same body.", MessageType.None);
                }
                
                EditorGUI.EndDisabledGroup();
                EditorGUILayout.EndVertical();
            }
            EditorGUILayout.Space();

            // Debug
            showDebug = EditorGUILayout.Foldout(showDebug, "Debug", true, EditorStyles.boldLabel);
            if (showDebug)
            {
                showGizmos = EditorGUILayout.Toggle("Show Gizmos", showGizmos);
                EditorGUI.BeginDisabledGroup(!showGizmos);
                showRestAreaGizmos = EditorGUILayout.Toggle("Show Rest Area Change", showRestAreaGizmos);
                EditorGUI.EndDisabledGroup();
            }
            EditorGUILayout.Space();

            // Visualization
            showVisualization = EditorGUILayout.Foldout(showVisualization, "Visualization Settings", true, EditorStyles.boldLabel);
            if (showVisualization)
            {
                EditorGUILayout.BeginVertical(EditorStyles.helpBox);
                

                EditorGUI.BeginChangeCheck();
                // Depth Culling (existing)
                enableDepthCulling = EditorGUILayout.Toggle("Enable Depth Culling", enableDepthCulling);
                
                EditorGUI.BeginDisabledGroup(!enableDepthCulling);
                occludedAlpha = EditorGUILayout.Slider("Occluded Alpha", occludedAlpha, 0f, 1f);
                EditorGUI.EndDisabledGroup();
                
                EditorGUILayout.Space(10);
                EditorGUILayout.LabelField("Render Distance Optimization", EditorStyles.boldLabel);
                
                // Distance Culling
                enableDistanceCulling = EditorGUILayout.Toggle("Enable Distance Culling", enableDistanceCulling);
                
                EditorGUI.BeginDisabledGroup(!enableDistanceCulling);
                EditorGUI.indentLevel++;
                
                maxNodesRenderDistance = EditorGUILayout.FloatField("Max Nodes Distance", maxNodesRenderDistance);
                maxNodesRenderDistance = Mathf.Max(0f, maxNodesRenderDistance);
                
                maxBeamsRenderDistance = EditorGUILayout.FloatField("Max Beams Distance", maxBeamsRenderDistance);
                // Clamp beams distance to not exceed nodes distance
                maxBeamsRenderDistance = Mathf.Clamp(maxBeamsRenderDistance, 0f, maxNodesRenderDistance);
                
                maxFacesRenderDistance = EditorGUILayout.FloatField("Max Faces Distance", maxFacesRenderDistance);
                maxFacesRenderDistance = Mathf.Clamp(maxFacesRenderDistance, 0f, maxNodesRenderDistance);
                
                maxBeamsRenderDistance = Mathf.Clamp(maxBeamsRenderDistance, 0f, maxNodesRenderDistance);
                
                maxTextInfoRenderDistance = EditorGUILayout.FloatField("Max Text Distance", maxTextInfoRenderDistance);
                maxTextInfoRenderDistance = Mathf.Max(0f, maxTextInfoRenderDistance);
                

                


                if (maxBeamsRenderDistance > maxNodesRenderDistance)
                {
                    EditorGUILayout.HelpBox("Beams distance cannot exceed Nodes distance", MessageType.Info);
                }
                
                EditorGUI.indentLevel--;
                EditorGUI.EndDisabledGroup();
                
                EditorGUILayout.Space(5);
                
                // Occlusion Culling
                enableOcclusionCulling = EditorGUILayout.Toggle("Enable Occlusion Culling", enableOcclusionCulling);
                if (enableOcclusionCulling)
                {
                    EditorGUILayout.HelpBox("Only nodes visible in camera frustum will render.", MessageType.None);
                }
                
                if (EditorGUI.EndChangeCheck())
                {
                    SaveVisualizationPrefs();
                }
                EditorGUILayout.EndVertical();
            }
        }

        #endregion

        #region Localized Helper Methods (Moved from Runtime)

        private void LoadDefaultsFromTruss(Truss truss)
        {
            if (truss == null) return;
            
            var beams = truss.GetTrussBeams();
            if (beams != null && beams.Count > 0)
            {
                var first = beams[0];
                defaultBeamCompliance = first.compliance;
                defaultBeamDamping = first.damping;
            }
        }

        private void EditorClearBeams()
        {
            if (workingTruss == null) return;
            Undo.RecordObject(workingTruss, "Clear Beams");
            workingTruss.ClearBeams();
            EditorUtility.SetDirty(workingTruss);
        }

        private void EditorClearGrid()
        {
            if (workingTruss == null) return;
            Undo.RecordObject(workingTruss, "Clear Grid");
            workingTruss.ClearGrid();
            EditorUtility.SetDirty(workingTruss);
        }

        private void EditorGenerateFromMesh()
        {
            if (workingTruss == null || targetSoftBody == null) return;
            
            MeshFilter mf = targetSoftBody.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) return;

            Vector3[] vertices = mf.sharedMesh.vertices;
            
            // Start with existing nodes
            List<Vector3> finalNodes = new List<Vector3>();
            if (workingTruss.NodePositions != null)
                finalNodes.AddRange(workingTruss.NodePositions);
                
            int originalCount = finalNodes.Count;

            const float STRICT_EPSILON = 0.00001f; // 1e-5
            // If Resolution is 1.0, distance should be min (Epsilon) to allow all vertices
            float densityDistance = Mathf.Max(STRICT_EPSILON, (1.0f - GenerationResolution) * 0.5f);
            
            int addedCount = 0;

            foreach (var v in vertices)
            {
                // 1. Check against EXISTING nodes (Strict Equality)
                bool matchesExisting = false;
                for (int i = 0; i < originalCount; i++)
                {
                    if (finalNodes[i] == v) // Direct vector comparison
                    {
                        matchesExisting = true;
                        break; 
                    }
                }

                if (matchesExisting) continue; // Found exact match, skip (don't add, don't touch existing)

                // 2. Check against NEWLY ADDED nodes (Density Check)
                // We filter the mesh itself to avoid adding too many nodes for dense meshes
                bool duplicateNew = false;
                for (int i = originalCount; i < finalNodes.Count; i++)
                {
                    if (Vector3.Distance(v, finalNodes[i]) < densityDistance)
                    {
                        duplicateNew = true;
                        break;
                    }
                }
                
                if (!duplicateNew)
                {
                    // Unique enough -> Add as new node
                    finalNodes.Add(v);
                    addedCount++;
                }
            }

            if (addedCount == 0)
            {
                EditorUtility.DisplayDialog("Merge Result", "No new nodes found (all mesh vertices matched existing nodes exactly or were filtered by resolution).", "OK");
                return;
            }

            Undo.RecordObject(workingTruss, "Merge Nodes From Mesh");
            
            // Save old masses
            List<float> oldMasses = new List<float>(workingTruss.NodeMasses);
            
            workingTruss.SetNodePositions(finalNodes.ToArray());
            
            // Restore masses (fills defaults for new)
            workingTruss.SetNodeMasses(oldMasses);
            
            EditorUtility.SetDirty(workingTruss);
            
            ApplyTrussToTarget();
            Debug.Log($"Merged Mesh: {addedCount} new nodes added. No existing nodes were modified.");
        }

        private int cachedEstimatedNodes = 0;
        private float lastCachedResolution = -1f;

        private void RecalculateEstimatedNodes()
        {
            // Force update
            cachedEstimatedNodes = GetEstimatedNodeCount();
            lastCachedResolution = GenerationResolution;
        }

        private int GetEstimatedNodeCount()
        {
            if (targetSoftBody == null) return 0;
            MeshFilter mf = targetSoftBody.GetComponent<MeshFilter>();
            if (mf == null || mf.sharedMesh == null) return 0;
            
            // Count actual unique positions (same logic as generation)
            Vector3[] vertices = mf.sharedMesh.vertices;
            List<Vector3> unique = new List<Vector3>();
            
            float densityDistance = Mathf.Max(0.00001f, (1.0f - GenerationResolution) * 0.5f);
            
            foreach (var v in vertices)
            {
                bool isDuplicate = false;
                foreach (var u in unique)
                {
                    if (v == u || Vector3.Distance(v, u) < densityDistance)
                    {
                        isDuplicate = true;
                        break;
                    }
                }
                if (!isDuplicate) unique.Add(v);
            }
            
            return unique.Count;
        }



        #endregion
        private void GenerateInterpolatedNodes(int resolution)
        {
            if (resolution < 1) return;
            if (workingTruss.GetTrussBeams() == null || workingTruss.GetTrussBeams().Count == 0) return;

            Undo.RecordObject(workingTruss, "Interpolate Nodes");

            // We perform subdivisions iteratively
            for (int r = 0; r < resolution; r++)
            {
                var beams = workingTruss.GetTrussBeams();
                var positions = new List<Vector3>(workingTruss.NodePositions);
                var masses = new List<float>(workingTruss.NodeMasses);
                var pinned = workingTruss.PinnedNodes;
                
                // Existing nodes remain indices 0..N-1
                // New nodes will restart at N
                
                // Low-level validation
                while(masses.Count < positions.Count) masses.Add(1f);

                // We will rebuild the beams list completely for this step
                var newBeams = new List<Beam>();
                
                // Map (NodeA, NodeB) -> NewNodeIndex to handle shared edges
                // Key: smallerIndex << 32 | largerIndex
                var midpointCache = new Dictionary<long, int>();
                
                foreach (var beam in beams)
                {
                    int a = beam.nodeA;
                    int b = beam.nodeB;
                    
                    if (a >= positions.Count || b >= positions.Count) continue;

                    // Order for cache key
                    int min = Mathf.Min(a, b);
                    int max = Mathf.Max(a, b);
                    long key = ((long)min << 32) | (long)max;
                    
                    int midIndex;
                    if (midpointCache.TryGetValue(key, out midIndex))
                    {
                         // Use existing midpoint
                    }
                    else
                    {
                        // Create new node
                        Vector3 pA = positions[a];
                        Vector3 pB = positions[b];
                        Vector3 mid = (pA + pB) * 0.5f;
                        
                        float massA = masses[a];
                        float massB = masses[b];
                        float massMid = (massA + massB) * 0.5f;
                        
                        midIndex = positions.Count;
                        positions.Add(mid);
                        masses.Add(massMid);
                        
                        midpointCache.Add(key, midIndex);
                    }
                    
                    // Create two new beams
                    float dist1 = Vector3.Distance(positions[a], positions[midIndex]);
                    float dist2 = Vector3.Distance(positions[midIndex], positions[b]);

                    Beam b1 = new Beam(a, midIndex, beam.compliance, beam.damping, dist1);
                    Beam b2 = new Beam(midIndex, b, beam.compliance, beam.damping, dist2);

                    newBeams.Add(b1);
                    newBeams.Add(b2);
                }
                
                // Apply changes for this iteration
                workingTruss.SetNodePositions(positions.ToArray());
                workingTruss.SetNodeMasses(masses);
                workingTruss.SetBeams(newBeams);
            }
            // Note: Pinned nodes indices remain valid for original nodes. New nodes are not pinned by default.
            
            EditorUtility.SetDirty(workingTruss);
            ApplyTrussToTarget();
        }

        private int CountImplicitTriangles(Truss truss)
        {
            if (truss == null || truss.NodePositions == null || truss.GetTrussBeams() == null) return 0;

            var beams = truss.GetTrussBeams();
            int nodeCount = truss.NodePositions.Length;
            if (nodeCount == 0) return 0;

            // Build adjacency list using HashSets for fast lookup
            var adj = new HashSet<int>[nodeCount];
            for (int i = 0; i < nodeCount; i++) adj[i] = new HashSet<int>();

            foreach (var beam in beams)
            {
                if (beam.nodeA < nodeCount && beam.nodeB < nodeCount)
                {
                    adj[beam.nodeA].Add(beam.nodeB);
                    adj[beam.nodeB].Add(beam.nodeA);
                }
            }

            int count = 0;
            // Iterate all unique triplets to find cycles of length 3
            for (int i = 0; i < nodeCount; i++)
            {
                foreach (int j in adj[i])
                {
                    if (j > i) // Check edge (i, j)
                    {
                        foreach (int k in adj[j])
                        {
                            if (k > j) // Check edge (j, k)
                            {
                                // Check closing edge (i, k)
                                if (adj[i].Contains(k))
                                {
                                    count++;
                                }
                            }
                        }
                    }
                }
            }
            return count;
        }
    }
}
