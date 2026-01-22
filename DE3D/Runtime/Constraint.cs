/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */
using UnityEngine;
using System.Collections.Generic;

namespace DynamicEngine
{
    public enum AttachmentType
    {
        Node = 1,
        Edge = 2
    }

    [System.Serializable]
    public class PointLink
    {
        public int sourceNodeIndex; // Index in Source Body
        
        // Target definition
        public AttachmentType attachmentType = AttachmentType.Node;
        public int targetNodeIndex = -1; // Index in Base Body
        
        // Physics Params
        public float restLength = 0f;
        public float minLength = 0f;
        public float maxLength = 0f; // 0 means default/auto? Or explicitly 0? In editor typically 0 means "auto" or we set it.
        // Let's rely on Editor to set 'restLength' to actual distance on init.
        
        public float strength = float.PositiveInfinity;
        public bool show = true;
        
        // Edge support (optional)
        public int targetNodeIndexB = -1; // If != -1, attached to Edge(targetNodeIndex, targetNodeIndexB)
    }

    [RequireComponent(typeof(SoftBody))]
    public class Constraint : MonoBehaviour
    {
        [SerializeField] private SoftBody m_baseBody;
        [SerializeField] private bool m_disableCollision = true;
        [SerializeField] private bool m_showLinks = false;
        
        [Header("Configuration")]
        [SerializeField] private string m_sourceNodeSet = "";
        [SerializeField] private List<PointLink> m_pointLinks = new List<PointLink>();

        // Removed legacy m_links
        // [SerializeField] private List<LinkConfig> m_links = new List<LinkConfig>();
        
        [HideInInspector] public List<bool> linkVisibility = new List<bool>();
        private bool m_beamsInitialized = false;
        private List<(Collider, Collider)> m_disabledCollisionPairs = new List<(Collider, Collider)>();
        private Dictionary<int, List<Beam>> m_linkToBeamsMap = new Dictionary<int, List<Beam>>();

        [Header("Motor")]
        [SerializeField] private bool m_enableMotor = false;
        [SerializeField] private string m_axisNodeSet = "";
        [SerializeField] private float m_targetRate = 90f;
        [SerializeField] private float m_maxTorque = 100f;
        [SerializeField] private float m_motorProportionalGain = 100f;
        private Dictionary<int, float> m_lastNodeAngles = new Dictionary<int, float>();
        private float GetCurrentDt() => m_attachedBody?.solver != null ?
            (Time.fixedDeltaTime / Mathf.Max(1, m_attachedBody.solver.GetSimulationSubSteps())) : Time.fixedDeltaTime;
        private float m_currentAngularVelocity = 0f;

        private SoftBody m_attachedBody;
        private List<Beam> m_addedBeams = new List<Beam>();

        public SoftBody baseBody => m_baseBody;
        public bool disableCollision => m_disableCollision;
        public bool showLinks => m_showLinks;
        
        public string sourceNodeSet 
        { 
            get => m_sourceNodeSet; 
            set => m_sourceNodeSet = value; 
        }
        
        public List<PointLink> pointLinks => m_pointLinks;
        
        public bool enableMotor => m_enableMotor;
        public string axisNodeSet => m_axisNodeSet;
        public float targetRate => m_targetRate;
        public float maxTorque => m_maxTorque;
        public SoftBody attachedBody => m_attachedBody;
        public int[] GetAxisNodeIndices()
        {
            if (string.IsNullOrEmpty(m_axisNodeSet) || m_attachedBody?.truss == null) return null;
            return m_attachedBody.truss.GetNodeSetIndices(m_axisNodeSet);
        }

        private Vector3 GetAxisCenter()
        {
            if (m_baseBody == null || m_baseBody.solver == null) return Vector3.zero;

            int[] axisIndices = GetAxisNodeIndices();
            if (axisIndices == null || axisIndices.Length == 0) return Vector3.zero;

            Vector3 center = Vector3.zero;
            foreach (int idx in axisIndices)
            {
                center += m_baseBody.solver.nodeManager.PredictedPositions[idx];
            }
            return center / axisIndices.Length;
        }

        private Vector3 GetAxisDirection()
        {
            if (m_baseBody == null || m_baseBody.solver == null) return Vector3.up;

            int[] axisIndices = GetAxisNodeIndices();
            if (axisIndices == null || axisIndices.Length < 2) return Vector3.up;

            Vector3 posA = m_baseBody.solver.nodeManager.PredictedPositions[axisIndices[0]];
            Vector3 posB = m_baseBody.solver.nodeManager.PredictedPositions[axisIndices[1]];

            Vector3 axis = (posB - posA).normalized;
            return axis.magnitude > 0.001f ? axis : Vector3.up;
        }

        private void ApplyMotorTorque(float dt)
        {
            if (!m_enableMotor || m_attachedBody?.solver == null || m_baseBody?.solver == null)
                return;

            int[] axisIndices = GetAxisNodeIndices();
            if (axisIndices == null || axisIndices.Length < 2) return;

            Vector3 axisCenter = GetAxisCenter();
            Vector3 axisDirection = GetAxisDirection();
            float totalAngularVelocity = 0f;
            int validNodes = 0;
            int attachedNodeCount = m_attachedBody.solver.nodeManager.Nodes.Count;

            // Velocity Tracking
            for (int i = 0; i < attachedNodeCount; i++)
            {
                if (m_attachedBody.solver.nodeManager.IsPinned[i]) continue;

                Vector3 nodePos = m_attachedBody.solver.nodeManager.PredictedPositions[i];
                Vector3 radius = nodePos - axisCenter;
                Vector3 radialComponent = radius - Vector3.Project(radius, axisDirection);
                float distance = radialComponent.magnitude;

                if (distance < 0.001f) continue;

                Vector3 referenceDir = Vector3.Cross(axisDirection, Vector3.up).normalized;
                if (referenceDir.magnitude < 0.001f) referenceDir = Vector3.Cross(axisDirection, Vector3.right).normalized;
                Vector3 perpDir = Vector3.Cross(axisDirection, referenceDir).normalized;

                float angle = Mathf.Atan2(Vector3.Dot(radialComponent, perpDir), Vector3.Dot(radialComponent, referenceDir));

                if (m_lastNodeAngles.ContainsKey(i))
                {
                    float deltaAngle = Mathf.DeltaAngle(m_lastNodeAngles[i] * Mathf.Rad2Deg, angle * Mathf.Rad2Deg) * Mathf.Deg2Rad;
                    totalAngularVelocity += deltaAngle / dt;
                    validNodes++;
                }
                m_lastNodeAngles[i] = angle;
            }

            m_currentAngularVelocity = validNodes > 0 ? totalAngularVelocity / validNodes : 0f;
            float targetAngularVelocity = m_targetRate * Mathf.Deg2Rad;
            float angularError = targetAngularVelocity - m_currentAngularVelocity;
            float appliedTorque = Mathf.Clamp(angularError * m_motorProportionalGain, -m_maxTorque, m_maxTorque);

            // Apply Forces
            for (int i = 0; i < attachedNodeCount; i++)
            {
                if (m_attachedBody.solver.nodeManager.IsPinned[i]) continue;

                Vector3 nodePos = m_attachedBody.solver.nodeManager.PredictedPositions[i];
                Vector3 radialComponent = (nodePos - axisCenter) - Vector3.Project(nodePos - axisCenter, axisDirection);
                float distance = radialComponent.magnitude;

                if (distance < 0.001f) continue;

                Vector3 tangent = Vector3.Cross(axisDirection, radialComponent).normalized;
                float forceMag = appliedTorque / (distance * attachedNodeCount);
                Vector3 force = tangent * forceMag;

                m_attachedBody.solver.ApplyWorldForceToNode(i, force, customDt: dt);

                foreach (int axisIdx in axisIndices)
                {
                    m_baseBody.solver.ApplyWorldForceToNode(axisIdx, -force / axisIndices.Length, customDt: dt);
                }
            }
        }

        private void Awake()
        {
            m_attachedBody = GetComponent<SoftBody>();
            if (m_attachedBody == null)
            {
                Debug.LogError("Constraint must be attached to a GameObject with SoftBody component.", this);
                enabled = false;
                return;
            }
        }

        void LateUpdate()
        {
            if (!m_beamsInitialized && Application.isPlaying)
            {
                InitializeBeams();
                m_beamsInitialized = true;
            }
#if UNITY_EDITOR
            if (Application.isPlaying)
                UnityEditor.SceneView.RepaintAll();
#endif
        }

        private void FixedUpdate()
        {
            if (!Application.isPlaying) return;

            // Update beam properties from inspector changes
            UpdateBeamPropertiesFromLinks();

            if (m_enableMotor)
            {
                ApplyMotorTorque(GetCurrentDt());
            }

            if (m_disableCollision && m_disabledCollisionPairs.Count == 0 && m_baseBody != null)
            {
                SetupCollisionDisabling();
            }
        }

        private void SetupCollisionDisabling()
        {
            if (!m_disableCollision || m_baseBody == null) return;

            Collider[] collidersA = m_attachedBody.GetComponentsInChildren<Collider>();
            Collider[] collidersB = m_baseBody.GetComponentsInChildren<Collider>();

            if (collidersA.Length == 0 || collidersB.Length == 0) return;

            RestoreCollisionPairs();

            foreach (Collider colA in collidersA)
            {
                foreach (Collider colB in collidersB)
                {
                    if (colA != null && colB != null)
                    {
                        Physics.IgnoreCollision(colA, colB, true);
                        m_disabledCollisionPairs.Add((colA, colB));
                    }
                }
            }

            if (m_attachedBody.solver?.collisionHandler != null)
            {
                m_attachedBody.solver.collisionHandler.SetIgnoreCollision(
                    m_attachedBody.transform, m_baseBody.transform, true);
            }

            if (m_baseBody.solver?.collisionHandler != null)
            {
                m_baseBody.solver.collisionHandler.SetIgnoreCollision(
                    m_attachedBody.transform, m_baseBody.transform, true);
            }
        }

        private void RestoreCollisionPairs()
        {
            foreach (var (colA, colB) in m_disabledCollisionPairs)
            {
                if (colA != null && colB != null)
                {
                    Physics.IgnoreCollision(colA, colB, false);
                }
            }
            m_disabledCollisionPairs.Clear();

            if (m_baseBody != null)
            {
                if (m_attachedBody?.solver?.collisionHandler != null)
                    m_attachedBody.solver.collisionHandler.SetIgnoreCollision(
                        m_attachedBody.transform, m_baseBody.transform, false);

                if (m_baseBody.solver?.collisionHandler != null)
                    m_baseBody.solver.collisionHandler.SetIgnoreCollision(
                        m_attachedBody.transform, m_baseBody.transform, false);
            }
        }

        private void InitializeBeams()
        {
            if (m_baseBody == null)
            {
                Debug.LogWarning("Base Body is mandatory for Constraints. Please assign a SoftBody.", this);
                return;
            }

            if (m_attachedBody.truss == null || m_baseBody.truss == null)
            {
                Debug.LogError("Truss asset must be assigned to both soft bodies.", this);
                return;
            }

            SetupCollisionDisabling();

            if (m_attachedBody.solver?.nodeManager?.Nodes == null || m_baseBody.solver?.nodeManager?.Nodes == null)
            {
                Debug.LogError("Solver or node manager not properly initialized.", this);
                return;
            }

            m_linkToBeamsMap.Clear();

            int successfulLinks = 0;
            
            // Iterate Point Links
            for (int i = 0; i < m_pointLinks.Count; i++)
            {
                var pointLink = m_pointLinks[i];
                
                // Validate Source Node
                if (pointLink.sourceNodeIndex < 0 || pointLink.sourceNodeIndex >= m_attachedBody.truss.NodePositions.Length) continue;
                
                // Validate Target Node(s)
                if (pointLink.targetNodeIndex < 0 || pointLink.targetNodeIndex >= m_baseBody.truss.NodePositions.Length) continue;
                
                m_linkToBeamsMap[i] = new List<Beam>();

                if (pointLink.attachmentType == AttachmentType.Node)
                {
                    // Node to Node
                    CreatePointToPointBeam(pointLink.sourceNodeIndex, pointLink.targetNodeIndex, pointLink, ref successfulLinks, i);
                }
                else if (pointLink.attachmentType == AttachmentType.Edge)
                {
                    // Node to Edge
                    if (pointLink.targetNodeIndexB >= 0 && pointLink.targetNodeIndexB < m_baseBody.truss.NodePositions.Length)
                    {
                        CreatePointToEdgeBeam(pointLink.sourceNodeIndex, pointLink.targetNodeIndex, pointLink.targetNodeIndexB, pointLink, ref successfulLinks, i);
                    }
                }
            }
        }

        private float DetermineTargetRestLength(PointLink config, Vector3 posA, Vector3 posB)
        {
            if (config.restLength > 0f) return config.restLength;

            float currentDist = Vector3.Distance(posA, posB);
            if (config.minLength > 0f || config.maxLength > 0f)
            {
                float min = config.minLength;
                float max = config.maxLength > 0f ? config.maxLength : float.PositiveInfinity;
                return Mathf.Clamp(currentDist, min, max);
            }

            return currentDist;
        }

        private void UpdateBeamPropertiesFromLinks()
        {
            if (!m_beamsInitialized) return;
            if (m_linkToBeamsMap == null || m_linkToBeamsMap.Count == 0) return;

            for (int i = 0; i < m_pointLinks.Count; i++)
            {
                if (!m_linkToBeamsMap.ContainsKey(i)) continue;

                var config = m_pointLinks[i];
                var beams = m_linkToBeamsMap[i];
                if (beams == null) continue;

                foreach (var beam in beams)
                {
                    if (beam == null) continue;

                    beam.minLength = config.minLength;
                    beam.maxLength = config.maxLength;
                    beam.strength = config.strength;

                    if (config.restLength > 0f)
                    {
                        beam.restLength = config.restLength;
                        if (beam.isEdgeSliding) beam.targetPerpDistance = config.restLength;
                    }
                    else if (config.minLength > 0f || config.maxLength > 0f)
                    {
                        float min = config.minLength;
                        float max = config.maxLength > 0f ? config.maxLength : float.PositiveInfinity;

                        if (beam.isEdgeSliding)
                        {
                            Vector3 nodePos = m_attachedBody.solver.nodeManager.PredictedPositions[beam.slidingNode];
                            Vector3 proj = GetProjectedPointOnEdge(m_baseBody, new int[] { beam.edgeNodeA, beam.edgeNodeB }, nodePos);
                            beam.targetPerpDistance = Mathf.Clamp(Vector3.Distance(nodePos, proj), min, max);
                        }
                        else
                        {
                            beam.restLength = Mathf.Clamp(beam.restLength, min, max);
                        }
                    }
                }
            }
        }

        private void CreatePointToPointBeam(int nodeA, int nodeB, PointLink config, ref int successfulLinks, int linkIndex)
        {
            float compliance = 0f;
            float damping = 0;

            Vector3 posA = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
            Vector3 posB = GetAttachmentPoint(m_baseBody, new int[] { nodeB }, AttachmentType.Node);
            float targetRestLength = DetermineTargetRestLength(config, posA, posB);

            Beam newBeam = new Beam(
                nodeA: nodeA,
                nodeB: nodeB,
                compliance: compliance,
                damping: damping,
                restLength: targetRestLength,
                bodyA: m_attachedBody,
                bodyB: m_baseBody
            );

            newBeam.minLength = config.minLength;
            newBeam.maxLength = config.maxLength;
            newBeam.strength = config.strength;

            m_attachedBody.solver.beams.Add(newBeam);
            m_baseBody.solver.beams.Add(newBeam);
            m_addedBeams.Add(newBeam);
            m_linkToBeamsMap[linkIndex].Add(newBeam);
            successfulLinks++;
        }

        private void CreatePointToEdgeBeam(int nodeA, int edgeNodeA, int edgeNodeB, PointLink config, ref int successfulLinks, int linkIndex)
        {
             // Similar logic to old CreateNodeToEdgeBeam but simplified for explicit indices
            float compliance = 0f;
            float damping = 0;

            float targetPerpDistance;
            if (config.minLength > 0f || config.maxLength > 0f)
            {
                if (config.minLength <= 0f) targetPerpDistance = config.maxLength;
                else if (config.maxLength <= 0f) targetPerpDistance = config.minLength;
                else targetPerpDistance = (config.minLength + config.maxLength) * 0.5f;
            }
            else if (config.restLength > 0f)
            {
                targetPerpDistance = config.restLength;
            }
            else
            {
                 // Auto calculate from current positions
                Vector3 nodePos = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
                Vector3 edgePosA = GetAttachmentPoint(m_baseBody, new int[] { edgeNodeA }, AttachmentType.Node);
                Vector3 edgePosB = GetAttachmentPoint(m_baseBody, new int[] { edgeNodeB }, AttachmentType.Node);

                Vector3 edgeVector = edgePosB - edgePosA;
                float edgeLength = edgeVector.magnitude;

                if (edgeLength > 0.001f)
                {
                    Vector3 edgeDir = edgeVector / edgeLength;
                    Vector3 nodeToEdgeStart = nodePos - edgePosA;
                    float t = Vector3.Dot(nodeToEdgeStart, edgeDir);
                    t = Mathf.Clamp(t, 0f, edgeLength);
                    Vector3 closestPoint = edgePosA + edgeDir * t;
                    targetPerpDistance = Vector3.Distance(nodePos, closestPoint);
                }
                else
                {
                    targetPerpDistance = 0.1f;
                }
            }

            for (int i = 0; i < 2; i++)
            {
                int currentEdgeNode = (i == 0) ? edgeNodeA : edgeNodeB;
                Vector3 nodePos = GetAttachmentPoint(m_attachedBody, new int[] { nodeA }, AttachmentType.Node);
                Vector3 pointB = GetAttachmentPoint(m_baseBody, new int[] { currentEdgeNode }, AttachmentType.Node);
                float actualRestLength = Vector3.Distance(nodePos, pointB);

                Beam newBeam = new Beam(
                    nodeA: nodeA,
                    nodeB: currentEdgeNode,
                    compliance: compliance,
                    damping: damping,
                    restLength: actualRestLength,
                    bodyA: m_attachedBody,
                    bodyB: m_baseBody
                );

                newBeam.isEdgeSliding = true;
                newBeam.edgeNodeA = edgeNodeA;
                newBeam.edgeNodeB = edgeNodeB;
                newBeam.slidingNode = nodeA;
                newBeam.targetPerpDistance = targetPerpDistance;
                newBeam.minLength = config.minLength;
                newBeam.maxLength = config.maxLength;
                newBeam.strength = config.strength;

                m_attachedBody.solver.beams.Add(newBeam);
                m_baseBody.solver.beams.Add(newBeam);
                m_addedBeams.Add(newBeam);
                m_linkToBeamsMap[linkIndex].Add(newBeam);
                successfulLinks++;
            }
        }


        private bool ValidateAttachment(int[] indices, AttachmentType type, string bodyLabel, int linkIndex, SoftBody body)
        {
            if (indices == null)
            {
                Debug.LogWarning($"Invalid attachment set for link {linkIndex} body {bodyLabel}. Set not found.", this);
                return false;
            }
            int requiredNodes = type == AttachmentType.Node ? 1 : 2;
            if (indices.Length != requiredNodes)
            {
                Debug.LogWarning($"Invalid attachment set for link {linkIndex} body {bodyLabel}. Expected {requiredNodes} nodes, got {indices.Length}.", this);
                return false;
            }
            foreach (int index in indices)
            {
                if (index < 0 || index >= body.solver.nodeManager.Nodes.Count)
                {
                    Debug.LogWarning($"Invalid node index {index} for link {linkIndex} body {bodyLabel}. Nodes={body.solver.nodeManager.Nodes.Count}. Skipping.", this);
                    return false;
                }
            }
            return true;
        }

        private Vector3 GetAttachmentPoint(SoftBody body, int[] indices, AttachmentType type)
        {
            if (body == null || body.solver == null || body.solver.nodeManager == null)
                return Vector3.zero;

            var predicted = body.solver.nodeManager.PredictedPositions;

            if (type == AttachmentType.Node)
            {
                int index = indices[0];
                if (index < 0 || index >= predicted.Count) return Vector3.zero;
                return predicted[index];
            }
            else
            {
                int indexA = indices[0];
                int indexB = indices[1];

                if (indexA < 0 || indexA >= predicted.Count) return Vector3.zero;
                if (indexB < 0 || indexB >= predicted.Count) return Vector3.zero;
                return (predicted[indexA] + predicted[indexB]) * 0.5f;
            }
        }

        private void OnDestroy()
        {
            RestoreCollisionPairs();

            if (m_baseBody != null && m_baseBody.solver != null)
            {
                foreach (var beam in m_addedBeams)
                {
                    m_baseBody.solver.beams.Remove(beam);
                }
            }
        }

        private void OnValidate()
        {
            if (m_pointLinks != null)
            {
                for (int i = 0; i < m_pointLinks.Count; i++)
                {
                    var link = m_pointLinks[i];
                    link.restLength = Mathf.Max(0f, link.restLength);
                    if (!float.IsInfinity(link.strength))
                    {
                        link.strength = Mathf.Max(1f, link.strength);
                    }
                }
            }

            linkVisibility.Clear();
            for (int i = 0; i < m_pointLinks.Count; i++)
            {
                linkVisibility.Add(m_pointLinks[i].show);
            }

            // Update beams if already initialized (during play mode)
            if (Application.isPlaying)
            {
                UpdateBeamPropertiesFromLinks();
            }
        }

        private bool IsLinkBroken(int linkIndex)
        {
            if (!m_linkToBeamsMap.ContainsKey(linkIndex))
                return false;

            var beams = m_linkToBeamsMap[linkIndex];
            if (beams == null || beams.Count == 0)
                return false;

            foreach (var beam in beams)
            {
                if (!beam.isBroken)
                    return false;
            }

            return true;
        }

        private void OnDrawGizmos()
        {
            if (!m_showLinks) return;
            if (m_baseBody == null && !Application.isPlaying) return;

            // In Editor, draw lines based on Config
            if (!Application.isPlaying)
            {
                if (m_attachedBody == null) m_attachedBody = GetComponent<SoftBody>();
                if (m_attachedBody == null || m_attachedBody.truss == null) return;
                
                // Draw all Point Links
                for (int i = 0; i < m_pointLinks.Count; i++)
                {
                    var link = m_pointLinks[i];
                    if (!link.show) continue;
                    
                    // Source Pos
                    int nodeA = link.sourceNodeIndex;
                    if (nodeA < 0 || nodeA >= m_attachedBody.truss.NodePositions.Length) continue;
                    Vector3 posA = m_attachedBody.transform.TransformPoint(m_attachedBody.truss.NodePositions[nodeA]);
                    
                    // Target Pos
                    Vector3 posB = Vector3.zero;
                    bool hasTarget = false;

                    if (m_baseBody != null)
                    {
                         if (link.attachmentType == AttachmentType.Node && link.targetNodeIndex >= 0 && link.targetNodeIndex < m_baseBody.truss.NodePositions.Length)
                         {
                             posB = m_baseBody.transform.TransformPoint(m_baseBody.truss.NodePositions[link.targetNodeIndex]);
                             hasTarget = true;
                         }
                         else if (link.attachmentType == AttachmentType.Edge && link.targetNodeIndex >= 0 && link.targetNodeIndexB >= 0 && link.targetNodeIndex < m_baseBody.truss.NodePositions.Length && link.targetNodeIndexB < m_baseBody.truss.NodePositions.Length)
                         {
                             // Projected point
                             Vector3 pA = m_baseBody.transform.TransformPoint(m_baseBody.truss.NodePositions[link.targetNodeIndex]);
                             Vector3 pB = m_baseBody.transform.TransformPoint(m_baseBody.truss.NodePositions[link.targetNodeIndexB]);
                             Vector3 edge = (pB - pA).normalized;
                             Vector3 toPos = posA - pA;
                             float dot = Vector3.Dot(toPos, edge);
                             dot = Mathf.Clamp(dot, 0, Vector3.Distance(pA, pB));
                             posB = pA + edge * dot;
                             hasTarget = true;
                         }
                    }

                    if (hasTarget)
                    {
                        Gizmos.color = Color.green;
                        Gizmos.DrawLine(posA, posB);
                        Gizmos.DrawWireSphere(posA, 0.05f);
                        Gizmos.DrawWireSphere(posB, 0.05f);
                    }
                }
            }
            else
            {
                // In Play mode, simple Beam drawing from added beams
                if (m_addedBeams == null) return;
                Gizmos.color = Color.green;
                foreach (var beam in m_addedBeams)
                {
                    if (beam == null) continue;
                    
                    Vector3 posA = Vector3.zero;
                    Vector3 posB = Vector3.zero;

                    if (beam.bodyA != null) posA = beam.bodyA.solver.nodeManager.PredictedPositions[beam.nodeA];
                    
                    if (beam.bodyB != null)
                    {
                        if (beam.isEdgeSliding)
                        {
                            posB = GetProjectedPointOnEdge(beam.bodyB, new int[] { beam.edgeNodeA, beam.edgeNodeB }, posA);
                        }
                        else
                        {
                             posB = beam.bodyB.solver.nodeManager.PredictedPositions[beam.nodeB];
                        }
                    }

                    if (posA != Vector3.zero && posB != Vector3.zero)
                    {
                         Gizmos.DrawLine(posA, posB);
                    }
                }
            }
        }
        
        private Vector3 GetProjectedPointOnEdge(SoftBody body, int[] edgeIndices, Vector3 point)
        {
             // Helper for runtime gizmos
            if (body == null || body.solver == null || body.solver.nodeManager == null || edgeIndices == null || edgeIndices.Length != 2) return Vector3.zero;
            
            Vector3 edgeStart = body.solver.nodeManager.PredictedPositions[edgeIndices[0]];
            Vector3 edgeEnd = body.solver.nodeManager.PredictedPositions[edgeIndices[1]];            
             
            Vector3 edgeVector = edgeEnd - edgeStart;
            float edgeLength = edgeVector.magnitude;
            if (edgeLength < 0.001f) return edgeStart;
            
            Vector3 edgeDir = edgeVector / edgeLength;
            Vector3 pointToEdgeStart = point - edgeStart;
            float t = Mathf.Clamp(Vector3.Dot(pointToEdgeStart, edgeDir), 0, edgeLength);
            
            return edgeStart + edgeDir * t;
        }
    }
}
