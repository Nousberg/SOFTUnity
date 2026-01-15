using UnityEngine;
using UnityEditor;
using System.Collections.Generic;

/// <summary>
/// Custom editor for SoftBodyConstraint with link management and scene visualization.
/// </summary>
[CustomEditor(typeof(SoftBodyConstraint))]
public class SoftBodyConstraintEditor : Editor
{
    private SoftBodyConstraint m_constraint;
    private SerializedProperty m_baseBodyProp;
    private SerializedProperty m_collideWithParentProp;
    private SerializedProperty m_showLinksProp;
    private SerializedProperty m_linksProp;
    
    private void OnEnable()
    {
        m_constraint = (SoftBodyConstraint)target;
        m_baseBodyProp = serializedObject.FindProperty("m_baseBody");
        m_collideWithParentProp = serializedObject.FindProperty("m_collideWithParent");
        m_showLinksProp = serializedObject.FindProperty("m_showLinks");
        m_linksProp = serializedObject.FindProperty("m_links");
    }
    
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        
        bool isPlaying = Application.isPlaying;
        
        // Header
        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Soft Body Constraint", EditorStyles.boldLabel);
        EditorGUILayout.Space();
        
        // Base Body
        GUI.enabled = !isPlaying;
        EditorGUILayout.PropertyField(m_baseBodyProp, new GUIContent("Base Body", "The other soft body to connect to. Leave null for world constraints."));
        EditorGUILayout.PropertyField(m_collideWithParentProp, new GUIContent("Collide With Parent", "If true, collision between bodies is enabled. If false, they pass through each other."));
        GUI.enabled = true;
        
        AdvancedVolumetricSoftBody baseBody = (AdvancedVolumetricSoftBody)m_baseBodyProp.objectReferenceValue;
        AdvancedVolumetricSoftBody attachedBody = m_constraint.GetComponent<AdvancedVolumetricSoftBody>();
        
        EditorGUILayout.Space();
        
        // Links section
        EditorGUILayout.BeginVertical(EditorStyles.helpBox);
        
        EditorGUILayout.BeginHorizontal();
        m_showLinksProp.boolValue = EditorGUILayout.Foldout(m_showLinksProp.boolValue, 
            $"Links ({m_linksProp.arraySize})");
        
        GUI.enabled = !isPlaying;
        if (GUILayout.Button("Add", EditorStyles.miniButton, GUILayout.MaxWidth(50)))
        {
            m_linksProp.InsertArrayElementAtIndex(m_linksProp.arraySize);
            SerializedProperty newLink = m_linksProp.GetArrayElementAtIndex(m_linksProp.arraySize - 1);
            newLink.FindPropertyRelative("typeA").enumValueIndex = (int)SoftBodyConstraint.AttachmentType.Node;
            newLink.FindPropertyRelative("nodeSetA").stringValue = "";
            newLink.FindPropertyRelative("typeB").enumValueIndex = baseBody != null ? 
                (int)SoftBodyConstraint.AttachmentType.Node : (int)SoftBodyConstraint.AttachmentType.World;
            newLink.FindPropertyRelative("nodeSetB").stringValue = "";
            newLink.FindPropertyRelative("restLength").floatValue = 0f;
            newLink.FindPropertyRelative("stiffness").floatValue = 1000f;
            newLink.FindPropertyRelative("damping").floatValue = 0.1f;
            newLink.FindPropertyRelative("show").boolValue = true;
        }
        GUI.enabled = true;
        EditorGUILayout.EndHorizontal();
        
        if (m_showLinksProp.boolValue)
        {
            EditorGUI.indentLevel++;
            
            for (int i = 0; i < m_linksProp.arraySize; i++)
            {
                SerializedProperty linkProp = m_linksProp.GetArrayElementAtIndex(i);
                SerializedProperty showProp = linkProp.FindPropertyRelative("show");
                
                EditorGUILayout.BeginHorizontal();
                
                string nodeSetA = linkProp.FindPropertyRelative("nodeSetA").stringValue;
                string nodeSetB = linkProp.FindPropertyRelative("nodeSetB").stringValue;
                string linkLabel = $"Link {i + 1}: {(string.IsNullOrEmpty(nodeSetA) ? "?" : nodeSetA)} → {(string.IsNullOrEmpty(nodeSetB) ? "World" : nodeSetB)}";
                
                showProp.boolValue = EditorGUILayout.Foldout(showProp.boolValue, linkLabel);
                
                GUI.enabled = !isPlaying;
                if (GUILayout.Button("×", EditorStyles.miniButton, GUILayout.MaxWidth(20)))
                {
                    m_linksProp.DeleteArrayElementAtIndex(i);
                    break;
                }
                GUI.enabled = true;
                EditorGUILayout.EndHorizontal();
                
                if (showProp.boolValue)
                {
                    EditorGUI.indentLevel++;
                    GUI.enabled = !isPlaying;
                    
                    // Node Set A (attached body)
                    DrawNodeSetDropdown(linkProp.FindPropertyRelative("nodeSetA"), "Node (Attached)", attachedBody);
                    
                    // Attachment type B
                    SerializedProperty typeBProp = linkProp.FindPropertyRelative("typeB");
                    
                    if (baseBody != null)
                    {
                        string[] typeOptions = { "Node", "World" };
                        int typeIndex = typeBProp.enumValueIndex == (int)SoftBodyConstraint.AttachmentType.World ? 1 : 0;
                        typeIndex = EditorGUILayout.Popup("Attach To", typeIndex, typeOptions);
                        typeBProp.enumValueIndex = typeIndex == 1 ? 
                            (int)SoftBodyConstraint.AttachmentType.World : (int)SoftBodyConstraint.AttachmentType.Node;
                        
                        if (typeBProp.enumValueIndex == (int)SoftBodyConstraint.AttachmentType.Node)
                        {
                            DrawNodeSetDropdown(linkProp.FindPropertyRelative("nodeSetB"), "Node (Base)", baseBody);
                        }
                    }
                    else
                    {
                        EditorGUILayout.LabelField("Attach To", "World (no base body)");
                        typeBProp.enumValueIndex = (int)SoftBodyConstraint.AttachmentType.World;
                    }
                    
                    // Properties
                    EditorGUILayout.PropertyField(linkProp.FindPropertyRelative("restLength"), 
                        new GUIContent("Rest Length", "0 = auto-calculate"));
                    EditorGUILayout.PropertyField(linkProp.FindPropertyRelative("stiffness"),
                        new GUIContent("Stiffness", "Higher = stiffer"));
                    EditorGUILayout.PropertyField(linkProp.FindPropertyRelative("damping"));
                    
                    GUI.enabled = true;
                    EditorGUI.indentLevel--;
                }
            }
            
            EditorGUI.indentLevel--;
        }
        
        EditorGUILayout.EndVertical();
        
        if (GUI.changed)
        {
            serializedObject.ApplyModifiedProperties();
        }
    }
    
    private void DrawNodeSetDropdown(SerializedProperty nodeSetProp, string label, AdvancedVolumetricSoftBody body)
    {
        if (body == null || body.trussData == null)
        {
            EditorGUILayout.LabelField(label, "No truss data");
            return;
        }
        
        string[] nodeSetNames = body.trussData.GetNodeSetNames();
        
        if (nodeSetNames.Length == 0)
        {
            EditorGUILayout.HelpBox($"No node sets defined in {body.name}'s truss", MessageType.Warning);
            return;
        }
        
        int currentIndex = System.Array.IndexOf(nodeSetNames, nodeSetProp.stringValue);
        int newIndex = EditorGUILayout.Popup(label, currentIndex, nodeSetNames);
        
        if (newIndex >= 0 && newIndex < nodeSetNames.Length)
        {
            nodeSetProp.stringValue = nodeSetNames[newIndex];
        }
    }
    
    private void OnSceneGUI()
    {
        if (m_constraint == null || !m_constraint.showLinks) return;
        
        AdvancedVolumetricSoftBody attachedBody = m_constraint.GetComponent<AdvancedVolumetricSoftBody>();
        AdvancedVolumetricSoftBody baseBody = m_constraint.baseBody;
        
        if (attachedBody == null || attachedBody.trussData == null) return;
        
        foreach (var link in m_constraint.links)
        {
            if (!link.show) continue;
            
            Vector3 pointA = GetAttachmentPoint(attachedBody, link.nodeSetA);
            if (pointA == Vector3.zero) continue;
            
            if (link.typeB == SoftBodyConstraint.AttachmentType.World)
            {
                // Draw sphere for world anchor
                Handles.color = Color.green;
                Handles.SphereHandleCap(0, pointA, Quaternion.identity, 
                    HandleUtility.GetHandleSize(pointA) * 0.1f, EventType.Repaint);
            }
            else if (baseBody != null && baseBody.trussData != null)
            {
                // Draw line to other body
                Vector3 pointB = GetAttachmentPoint(baseBody, link.nodeSetB);
                if (pointB != Vector3.zero)
                {
                    Handles.color = Color.cyan;
                    Handles.DrawDottedLine(pointA, pointB, 2f);
                }
            }
        }
    }
    
    private Vector3 GetAttachmentPoint(AdvancedVolumetricSoftBody body, string nodeSetName)
    {
        if (body == null || body.trussData == null || string.IsNullOrEmpty(nodeSetName)) 
            return Vector3.zero;
        
        int[] indices = body.trussData.GetNodeSetIndices(nodeSetName);
        if (indices == null || indices.Length == 0) return Vector3.zero;
        
        Vector3 center = Vector3.zero;
        int count = 0;
        
        foreach (int idx in indices)
        {
            if (idx >= 0 && idx < body.trussData.nodes.Count)
            {
                var node = body.trussData.nodes[idx];
                Vector3 pos = Application.isPlaying ? node.worldPosition : 
                    body.transform.TransformPoint(node.localPosition);
                center += pos;
                count++;
            }
        }
        
        return count > 0 ? center / count : Vector3.zero;
    }
}
