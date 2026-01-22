/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */
using UnityEngine;

namespace DynamicEngine
{
    public class SceneSettings : MonoBehaviour
    {
        #region Singleton

        private static SceneSettings instance;
        public static SceneSettings Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = FindFirstObjectByType<SceneSettings>();
                    if (instance == null)
                    {
                        GameObject settingsObj = new GameObject("SceneSettings");
                        instance = settingsObj.AddComponent<SceneSettings>();
                        Debug.LogWarning("No SceneSettings found in scene. Created new instance.", settingsObj);
                    }
                }
                return instance;
            }
        }

        void Awake()
        {
            if (instance != null && instance != this)
            {
                Debug.LogWarning("Multiple SceneSettings instances detected. Destroying duplicate.", this);
                Destroy(gameObject);
                return;
            }
            instance = this;

            // Ensure the GameObject is a root object before applying DontDestroyOnLoad
            if (transform.parent != null)
            {
                Debug.LogWarning("SceneSettings must be a root GameObject to persist across scenes. Moving to root.", this);
                transform.SetParent(null);
            }
            DontDestroyOnLoad(gameObject);
        }

        #endregion

        #region Properties



        public float SimulationTimeScale
        {
            get { return m_simulationTimeScale; }
            set { m_simulationTimeScale = value; }
        }

        public int ConstraintIterations
        {
            get { return m_constraintIterations; }
            set { m_constraintIterations = value; }
        }

        public int WorkerThreads
        {
            get { return m_workerThreads; }
            set { m_workerThreads = value; }
        }

        public int BaseSubSteps
        {
            get { return m_baseSubSteps; }
            set { m_baseSubSteps = value; }
        }

        public int MinSubSteps
        {
            get { return m_minSubSteps; }
            set { m_minSubSteps = value; }
        }

        // V2V Collision Settings
        public float AntiClipDistance
        {
            get { return m_antiClipDistance; }
            set { m_antiClipDistance = Mathf.Max(0.001f, value); }
        }

        public float TriangleActivationRadius
        {
            get { return m_triangleActivationRadius; }
            set { m_triangleActivationRadius = Mathf.Max(0.01f, value); }
        }

        public float ExternalCollisionBias
        {
            get { return m_externalCollisionBias; }
            set { m_externalCollisionBias = value; }
        }

        public int V2VSolverIterations
        {
            get { return m_v2vSolverIterations; }
            set { m_v2vSolverIterations = Mathf.Max(1, value); }
        }

        public float V2VBaumgarteFactor
        {
            get { return m_v2vBaumgarteFactor; }
            set { m_v2vBaumgarteFactor = Mathf.Clamp01(value); }
        }

        public float V2VRestitution
        {
            get { return m_v2vRestitution; }
            set { m_v2vRestitution = Mathf.Clamp01(value); }
        }

        public float V2VFriction
        {
            get { return m_v2vFriction; }
            set { m_v2vFriction = Mathf.Clamp01(value); }
        }

        #endregion

        #region Unity

        void OnValidate()
        {
            m_simulationTimeScale = Mathf.Clamp(m_simulationTimeScale, 0.1f, 1.0f);
            m_constraintIterations = Mathf.Max(m_constraintIterations, 1);
            m_workerThreads = Mathf.Max(m_workerThreads, 1);
            m_baseSubSteps = Mathf.Max(m_baseSubSteps, 1);
            m_minSubSteps = Mathf.Max(m_minSubSteps, 1);
        }

        void OnGUI()
        {
            GUIStyle style = new GUIStyle();
            style.normal.textColor = Color.red;
            GUIStyle size = new GUIStyle();
            size.fontSize = 20;
            GUI.Label(new Rect(10, 10, 200, 40), "Info:", size);
            GUI.Label(new Rect(10, 40, 200, 40), "Iterations: " + m_constraintIterations, style);
            GUI.Label(new Rect(10, 60, 200, 40), "Simulation Speed: " + m_simulationTimeScale, style);
            GUI.Label(new Rect(10, 80, 200, 40), "Base Substeps: " + m_baseSubSteps, style);

        }

        #endregion

        #region Private



        [SerializeField]
        private float m_simulationTimeScale = 1.0f;

        [SerializeField]
        private int m_constraintIterations = 5;

        [SerializeField]
        private int m_workerThreads = 1;

        [SerializeField]
        private int m_baseSubSteps = 1;

        [SerializeField]
        private int m_minSubSteps = 1;

        [Header("V2V Collision Settings")]
        [SerializeField, Tooltip("Anti-clip buffer distance (BeamNG default: 0.025m = 2.5cm)")]
        private float m_antiClipDistance = 0.025f;

        [SerializeField, Tooltip("Triangles activate when nodes within this radius (BeamNG: 0.10m = 10cm)")]
        private float m_triangleActivationRadius = 0.10f;

        [SerializeField, Tooltip("Shifts anti-clip start distance (positive = earlier contact)")]
        private float m_externalCollisionBias = 0f;

        [SerializeField, Tooltip("Iterations for V2V constraint solver")]
        private int m_v2vSolverIterations = 10;

        [SerializeField, Range(0f, 1f), Tooltip("Baumgarte stabilization factor")]
        private float m_v2vBaumgarteFactor = 0.2f;

        [SerializeField, Range(0f, 1f), Tooltip("Restitution (bounciness) for V2V collisions")]
        private float m_v2vRestitution = 0.1f;

        [SerializeField, Range(0f, 1f), Tooltip("Friction coefficient for V2V collisions")]
        private float m_v2vFriction = 0.5f;

        #endregion
    }
}