/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */
using UnityEditor;
using UnityEngine;
using System.Linq;
using System.Collections.Generic;

namespace DynamicEngine
{
    [CustomEditor(typeof(Constraint))]
    [CanEditMultipleObjects]
    public class ConstraintEditor : UnityEditor.Editor
    {
        private Constraint[] m_targets;
        private SerializedProperty softBodyBProp;
        private SerializedProperty disableCollisionProp;
        private SerializedProperty showLinksProp;
        private SerializedProperty linkConfigsProp;
        private SerializedProperty enableMotorProp;
        private SerializedProperty axisNodeSetProp;
        private SerializedProperty targetRateProp;
        private SerializedProperty maxTorqueProp;

        // Track previous transform state to detect scene changes
        private Vector3 lastBodyAPosition;
        private Quaternion lastBodyARotation;
        private Vector3 lastBodyAScale;
        private Vector3 lastBodyBPosition;
        private Quaternion lastBodyBRotation;
        private Vector3 lastBodyBScale;
        private bool transformsInitialized = false;

        protected virtual void OnEnable()
        {
            m_targets = new Constraint[targets.Length];
            for (int i = 0; i < targets.Length; ++i) m_targets[i] = (Constraint)targets[i];
            softBodyBProp = serializedObject.FindProperty("m_baseBody");
            disableCollisionProp = serializedObject.FindProperty("m_disableCollision");
            showLinksProp = serializedObject.FindProperty("m_showLinks");
            linkConfigsProp = serializedObject.FindProperty("m_pointLinks");
            enableMotorProp = serializedObject.FindProperty("m_enableMotor");
            axisNodeSetProp = serializedObject.FindProperty("m_axisNodeSet");
            targetRateProp = serializedObject.FindProperty("m_targetRate");
            maxTorqueProp = serializedObject.FindProperty("m_maxTorque");

            // Subscribe to scene view updates
            SceneView.duringSceneGui += OnSceneUpdate;
            transformsInitialized = false;
        }

        private void OnDisable()
        {
            // Unsubscribe from scene view updates
            SceneView.duringSceneGui -= OnSceneUpdate;
        }

        private void OnSceneUpdate(SceneView sceneView)
        {
            // Only check for single target
            if (m_targets.Length != 1) return;

            Constraint constraint = m_targets[0];
            if (constraint == null) return;

            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            SoftBody bodyB = constraint.baseBody;

            if (bodyA == null) return;

            // Initialize transform tracking
            if (!transformsInitialized)
            {
                lastBodyAPosition = bodyA.transform.position;
                lastBodyARotation = bodyA.transform.rotation;
                lastBodyAScale = bodyA.transform.localScale;

                if (bodyB != null)
                {
                    lastBodyBPosition = bodyB.transform.position;
                    lastBodyBRotation = bodyB.transform.rotation;
                    lastBodyBScale = bodyB.transform.localScale;
                }

                transformsInitialized = true;
                return;
            }

            // Check if transforms have changed
            bool transformChanged = false;

            if (bodyA.transform.position != lastBodyAPosition ||
                bodyA.transform.rotation != lastBodyARotation ||
                bodyA.transform.localScale != lastBodyAScale)
            {
                lastBodyAPosition = bodyA.transform.position;
                lastBodyARotation = bodyA.transform.rotation;
                lastBodyAScale = bodyA.transform.localScale;
                transformChanged = true;
            }

            if (bodyB != null)
            {
                if (bodyB.transform.position != lastBodyBPosition ||
                    bodyB.transform.rotation != lastBodyBRotation ||
                    bodyB.transform.localScale != lastBodyBScale)
                {
                    lastBodyBPosition = bodyB.transform.position;
                    lastBodyBRotation = bodyB.transform.rotation;
                    lastBodyBScale = bodyB.transform.localScale;
                    transformChanged = true;
                }
            }

            // If transforms changed, update rest lengths
            if (transformChanged)
            {
                UpdateRestLengths();
                Repaint(); // Force inspector repaint
            }
        }

        private void UpdateRestLengths()
        {
            if (m_targets.Length != 1) return;

            Constraint constraint = m_targets[0];
            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            SoftBody bodyB = constraint.baseBody;

            if (bodyA == null || bodyA.truss == null) return;
            if (bodyB == null || bodyB.truss == null) return;

            serializedObject.Update();

            // Update rest lengths for point links
            for (int i = 0; i < linkConfigsProp.arraySize; i++)
            {
                SerializedProperty link = linkConfigsProp.GetArrayElementAtIndex(i);
                SerializedProperty sourceIdx = link.FindPropertyRelative("sourceNodeIndex");
                SerializedProperty targetIdx = link.FindPropertyRelative("targetNodeIndex");
                SerializedProperty restLength = link.FindPropertyRelative("restLength");
                SerializedProperty attachType = link.FindPropertyRelative("attachmentType");

                if (sourceIdx.intValue < 0 || targetIdx.intValue < 0) continue;
                if (sourceIdx.intValue >= bodyA.truss.NodePositions.Length) continue;
                if (targetIdx.intValue >= bodyB.truss.NodePositions.Length) continue;

                Vector3 posA = bodyA.transform.TransformPoint(bodyA.truss.NodePositions[sourceIdx.intValue]);
                Vector3 posB = bodyB.transform.TransformPoint(bodyB.truss.NodePositions[targetIdx.intValue]);
                
                float calculatedLength = Vector3.Distance(posA, posB);
                if (calculatedLength > 0 && restLength.floatValue == 0)
                {
                    restLength.floatValue = calculatedLength;
                }
            }

            serializedObject.ApplyModifiedProperties();
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();
            EditorGUIUtility.labelWidth = 120f;
            bool isPlaying = Application.isPlaying;
            SoftBody softBodyB = (SoftBody)softBodyBProp.objectReferenceValue;
            GUI.enabled = true && !isPlaying;
            
            EditorGUILayout.PropertyField(softBodyBProp, new GUIContent("Base Body", "The base soft body to connect to."));
            EditorGUILayout.PropertyField(disableCollisionProp, new GUIContent("Disable Collision", "Disable collision between connected bodies."));
            
            if (m_targets.Length > 1)
            {
                GUI.enabled = false;
                EditorGUILayout.LabelField("Select single body to edit links");
                GUI.enabled = true;
                serializedObject.ApplyModifiedProperties();
                return;
            }

            Constraint targetConstraint = m_targets[0];
            SoftBody softBodyA = targetConstraint.GetComponent<SoftBody>();
            if (softBodyA == null)
            {
                serializedObject.ApplyModifiedProperties();
                return;
            }
            
            SerializedProperty sourceNodeSetProp = serializedObject.FindProperty("m_sourceNodeSet");
            SerializedProperty pointLinksProp = serializedObject.FindProperty("m_pointLinks");

            // Source Node Set Selection
            if (softBodyA.truss != null)
            {
                var nodeSets = softBodyA.truss.GetNodeSets();
                string[] sets = nodeSets.Select(s => s.name).ToArray();
                if (sets.Length > 0)
                {
                    int currentIndex = System.Array.IndexOf(sets, sourceNodeSetProp.stringValue);
                    if (currentIndex == -1) currentIndex = 0;
                    
                    int newIndex = EditorGUILayout.Popup("Source Node Set", currentIndex, sets);
                    string newSet = sets[newIndex];
                    
                    if (newSet != sourceNodeSetProp.stringValue)
                    {
                        sourceNodeSetProp.stringValue = newSet;
                    }
                }
                else
                {
                    EditorGUILayout.HelpBox("No Node Sets found in Source Body Truss.", MessageType.Warning);
                }
            }
            
            // Sync Logic
            if (!string.IsNullOrEmpty(sourceNodeSetProp.stringValue) && softBodyA.truss != null)
            {
                int[] sourceIndices = softBodyA.truss.GetNodeSetIndices(sourceNodeSetProp.stringValue);
                if (sourceIndices != null)
                {
                    if (pointLinksProp.arraySize != sourceIndices.Length)
                    {
                        pointLinksProp.arraySize = sourceIndices.Length;
                    }
                    
                    for (int i = 0; i < sourceIndices.Length; i++)
                    {
                        SerializedProperty element = pointLinksProp.GetArrayElementAtIndex(i);
                        element.FindPropertyRelative("sourceNodeIndex").intValue = sourceIndices[i];
                    }
                }
            }

            if (softBodyB == null)
            {
                EditorGUILayout.HelpBox("Assign a Base Body to configure constraints.", MessageType.Error);
            }
            else
            {
                EditorGUILayout.BeginVertical("box");
                EditorGUILayout.LabelField($"Points ({pointLinksProp.arraySize})", EditorStyles.boldLabel);
                
                for (int i = 0; i < pointLinksProp.arraySize; i++)
                {
                    SerializedProperty link = pointLinksProp.GetArrayElementAtIndex(i);
                    int sourceIdx = link.FindPropertyRelative("sourceNodeIndex").intValue;
                    SerializedProperty show = link.FindPropertyRelative("show");
                    
                    show.boolValue = EditorGUILayout.Foldout(show.boolValue, $"Point {sourceIdx}", true);
                    
                    if (show.boolValue)
                    {
                        EditorGUI.indentLevel++;
                        
                        SerializedProperty attachType = link.FindPropertyRelative("attachmentType");
                        EditorGUILayout.PropertyField(attachType, new GUIContent("Type"));

                        int attachTypeValue = attachType.intValue;
                        if (attachTypeValue == (int)AttachmentType.Node)
                        {
                            SerializedProperty targetNode = link.FindPropertyRelative("targetNodeIndex");
                            targetNode.intValue = EditorGUILayout.IntField("Target Node Index", targetNode.intValue);
                        }
                        else if (attachTypeValue == (int)AttachmentType.Edge)
                        {
                            SerializedProperty targetNodeA = link.FindPropertyRelative("targetNodeIndex");
                            SerializedProperty targetNodeB = link.FindPropertyRelative("targetNodeIndexB");
                            targetNodeA.intValue = EditorGUILayout.IntField("Edge Node A", targetNodeA.intValue);
                            targetNodeB.intValue = EditorGUILayout.IntField("Edge Node B", targetNodeB.intValue);
                        }
                        
                        EditorGUILayout.PropertyField(link.FindPropertyRelative("restLength"), new GUIContent("Rest Length", "0 = Auto"));
                        EditorGUILayout.PropertyField(link.FindPropertyRelative("minLength"));
                        EditorGUILayout.PropertyField(link.FindPropertyRelative("maxLength"));
                        EditorGUILayout.PropertyField(link.FindPropertyRelative("strength"));
                        
                        EditorGUI.indentLevel--;
                    }
                }
                EditorGUILayout.EndVertical();
            }

            // Motor Section
            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(enableMotorProp, new GUIContent("Enable Motor"));

            if (enableMotorProp.boolValue)
            {
                EditorGUI.indentLevel++;

                if (softBodyA.truss != null)
                {
                    string[] axisNodes = GetNodes(softBodyA, 2, 100); // Allow 2+ nodes for axis
                    int index = System.Array.IndexOf(axisNodes, axisNodeSetProp.stringValue);
                    index = EditorGUILayout.Popup("Axis Nodes", index, axisNodes);
                    if (index > -1 && index < axisNodes.Length)
                        axisNodeSetProp.stringValue = axisNodes[index];
                    else if (axisNodes.Length > 0 && string.IsNullOrEmpty(axisNodeSetProp.stringValue))
                        axisNodeSetProp.stringValue = axisNodes[0];
                }

                EditorGUILayout.PropertyField(targetRateProp, new GUIContent("Target Rate (deg/s)", "Target angular velocity in degrees per second"));
                EditorGUILayout.PropertyField(maxTorqueProp, new GUIContent("Max Torque", "Maximum torque the motor can apply"));

                // Show warning if axis nodes not set
                if (string.IsNullOrEmpty(axisNodeSetProp.stringValue))
                {
                    EditorGUILayout.HelpBox("Motor requires axis nodes to be set! Select a NodeSet with 2+ nodes from the source body that define the rotation axis.", MessageType.Warning);
                }

                EditorGUI.indentLevel--;
            }

            if (GUI.changed) serializedObject.ApplyModifiedProperties();
        }

        private string[] GetNodes(SoftBody body, int minNodes = 1, int maxNodes = 1)
        {
            List<string> nodes = new List<string>();
            if (body != null && body.truss != null)
            {
                var nodeSets = body.truss.GetNodeSets();
                foreach (var nodeSet in nodeSets)
                {
                    if (nodeSet.nodeIndices.Count >= minNodes && nodeSet.nodeIndices.Count <= maxNodes)
                    {
                        nodes.Add(nodeSet.name);
                    }
                }
            }
            return nodes.ToArray();
        }
        private void OnSceneGUI()
        {
            if (Application.isPlaying) return;

            Constraint constraint = target as Constraint;
            if (constraint == null) return;

            SoftBody bodyA = constraint.GetComponent<SoftBody>();
            if (bodyA == null || bodyA.truss == null) return;

            SoftBody bodyB = constraint.baseBody;
            if (bodyB == null) return; // Base body mandatory now

            SerializedObject so = new SerializedObject(constraint);
            SerializedProperty pointLinksProp = so.FindProperty("m_pointLinks");

            for (int i = 0; i < pointLinksProp.arraySize; i++)
            {
                SerializedProperty linkProp = pointLinksProp.GetArrayElementAtIndex(i);
                if (!linkProp.FindPropertyRelative("show").boolValue) continue;

                int sourceIdx = linkProp.FindPropertyRelative("sourceNodeIndex").intValue;
                int targetIdx = linkProp.FindPropertyRelative("targetNodeIndex").intValue;
                int attachType = linkProp.FindPropertyRelative("attachmentType").intValue;

                Vector3 posA = GetNodePosition(bodyA, sourceIdx);
                if (posA == Vector3.zero) continue;

                Vector3 posB = Vector3.zero;
                if (attachType == (int)AttachmentType.Node)
                {
                    posB = GetNodePosition(bodyB, targetIdx);
                }
                else if (attachType == (int)AttachmentType.Edge)
                {
                    int targetIdxB = linkProp.FindPropertyRelative("targetNodeIndexB").intValue;
                    Vector3 p1 = GetNodePosition(bodyB, targetIdx);
                    Vector3 p2 = GetNodePosition(bodyB, targetIdxB);
                    if (p1 != Vector3.zero && p2 != Vector3.zero)
                    {
                        // Draw Edge
                        Handles.color = new Color(0.5f, 0.5f, 0.5f, 0.5f);
                        Handles.DrawLine(p1, p2);
                        
                        // Project
                        Vector3 edge = (p2 - p1);
                        float len = edge.magnitude;
                        if (len > 0.001f)
                        {
                            edge /= len;
                            float t = Mathf.Clamp(Vector3.Dot(posA - p1, edge), 0, len);
                            posB = p1 + edge * t;
                        }
                        else posB = p1;
                    }
                }

                if (posB != Vector3.zero)
                {
                    Handles.color = Color.green;
                    Handles.DrawLine(posA, posB);
                    
                    float size = HandleUtility.GetHandleSize(posA) * 0.05f;
                    Handles.SphereHandleCap(0, posA, Quaternion.identity, size, EventType.Repaint);
                    Handles.SphereHandleCap(0, posB, Quaternion.identity, size, EventType.Repaint);
                    
                    // Optional: Draw text?
                }
            }
        }

        private Vector3 GetNodePosition(SoftBody body, int index)
        {
            if (body == null || body.truss == null || index < 0 || index >= body.truss.NodePositions.Length)
                return Vector3.zero;
            return body.transform.TransformPoint(body.truss.NodePositions[index]);
        }
    }
}
