/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */

using UnityEditor;
using UnityEngine;

namespace DynamicEngine
{
    [CustomEditor(typeof(SceneSettings))]
    public class SceneSettingsEditor : UnityEditor.Editor
    {
        SceneSettings[] m_targets;


        SerializedProperty simulationTimeScale;
        SerializedProperty constraintIterations;
        SerializedProperty baseSubSteps;
        SerializedProperty minSubSteps;
        SerializedProperty workerThreads;
        SerializedProperty antiClipDistance;
        SerializedProperty triangleActivationRadius;
        SerializedProperty externalCollisionBias;
        SerializedProperty v2vSolverIterations;
        SerializedProperty v2vBaumgarteFactor;
        SerializedProperty v2vRestitution;
        SerializedProperty v2vFriction;

        protected virtual void OnEnable()
        {
            m_targets = new SceneSettings[targets.Length];
            for (int i = 0; i < targets.Length; ++i) m_targets[i] = (SceneSettings)targets[i];


            simulationTimeScale = serializedObject.FindProperty("m_simulationTimeScale");
            constraintIterations = serializedObject.FindProperty("m_constraintIterations");
            baseSubSteps = serializedObject.FindProperty("m_baseSubSteps");
            minSubSteps = serializedObject.FindProperty("m_minSubSteps");
            workerThreads = serializedObject.FindProperty("m_workerThreads");
            antiClipDistance = serializedObject.FindProperty("m_antiClipDistance");
            triangleActivationRadius = serializedObject.FindProperty("m_triangleActivationRadius");
            externalCollisionBias = serializedObject.FindProperty("m_externalCollisionBias");
            v2vSolverIterations = serializedObject.FindProperty("m_v2vSolverIterations");
            v2vBaumgarteFactor = serializedObject.FindProperty("m_v2vBaumgarteFactor");
            v2vRestitution = serializedObject.FindProperty("m_v2vRestitution");
            v2vFriction = serializedObject.FindProperty("m_v2vFriction");
        }

        public override void OnInspectorGUI()
        {
            serializedObject.Update();

            EditorGUILayout.LabelField("Simulation");

            EditorGUI.indentLevel++;
            EditorGUILayout.Slider(simulationTimeScale, 0.1f, 1f);
            EditorGUILayout.PropertyField(constraintIterations);
            EditorGUILayout.PropertyField(baseSubSteps);
            EditorGUILayout.PropertyField(minSubSteps);
            EditorGUI.indentLevel--;



            EditorGUILayout.Separator();

            EditorGUILayout.LabelField("Optimization");

            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(workerThreads);
            EditorGUI.indentLevel--;

            EditorGUILayout.Separator();
            EditorGUILayout.Separator();

            EditorGUILayout.LabelField("V2V Collision Settings");
            EditorGUI.indentLevel++;
            EditorGUILayout.PropertyField(antiClipDistance);
            EditorGUILayout.PropertyField(triangleActivationRadius);
            EditorGUILayout.PropertyField(externalCollisionBias);
            EditorGUILayout.PropertyField(v2vSolverIterations);
            EditorGUILayout.Slider(v2vBaumgarteFactor, 0f, 1f);
            EditorGUILayout.Slider(v2vRestitution, 0f, 1f);
            EditorGUILayout.Slider(v2vFriction, 0f, 1f);
            EditorGUI.indentLevel--;

            EditorGUILayout.Separator();
            EditorGUILayout.Separator();

            if (GUILayout.Button("Remove Scene Settings"))
            {
                EditorApplication.delayCall += () => Undo.DestroyObjectImmediate(m_targets[0].gameObject);
            }

            if (GUI.changed) serializedObject.ApplyModifiedProperties();
        }
    }
}
