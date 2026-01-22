using UnityEngine;
using System.Collections.Generic;
using System;

/// <summary>
/// Constraint component that connects AdvancedVolumetricSoftBody nodes together
/// or anchors them to world space positions.
/// </summary>
[RequireComponent(typeof(AdvancedVolumetricSoftBody))]
public class SoftBodyConstraint : MonoBehaviour
{
    #region Enums and Structs
    
    public enum AttachmentType
    {
        World,  // Pin to world position
        Node    // Connect to another soft body's node
    }
    
    [Serializable]
    public class ConstraintLink
    {
        [Header("Attached Body (this)")]
        public AttachmentType typeA = AttachmentType.Node;
        public string nodeSetA = "";
        
        [Header("Base Body (target)")]
        public AttachmentType typeB = AttachmentType.Node;
        public string nodeSetB = "";
        
        [Header("Properties")]
        [Tooltip("0 = auto-calculate from initial positions")]
        public float restLength = 0f;
        [Tooltip("Spring stiffness. Higher = stiffer. Use Infinity for rigid")]
        public float stiffness = 1000f;
        [Range(0f, 1f)]
        public float damping = 0.1f;
        
        [Header("Display")]
        public bool show = true;
        
        // Runtime calculated
        [NonSerialized] public float actualRestLength;
    }
    
    #endregion
    
    #region Serialized Fields
    
    [Header("Base Body")]
    [Tooltip("The other soft body to connect to. Leave null for world constraints.")]
    [SerializeField] private AdvancedVolumetricSoftBody m_baseBody;
    
    [Header("Settings")]
    [Tooltip("If true, colliders between this body and base body will collide. If false, collision is disabled.")]
    [SerializeField] private bool m_collideWithParent = false;
    [SerializeField] private bool m_showLinks = true;
    
    [Header("Links")]
    [SerializeField] private List<ConstraintLink> m_links = new List<ConstraintLink>();
    
    #endregion
    
    #region Properties
    
    public AdvancedVolumetricSoftBody baseBody => m_baseBody;
    public bool collideWithParent => m_collideWithParent;
    public bool showLinks => m_showLinks;
    public List<ConstraintLink> links => m_links;
    
    #endregion
    
    #region Private Fields
    
    private AdvancedVolumetricSoftBody m_attachedBody;
    private bool m_isInitialized = false;
    private List<(Collider, Collider)> m_disabledCollisionPairs = new List<(Collider, Collider)>();
    
    // Store world anchor positions
    private Dictionary<int, Vector3> m_worldAnchors = new Dictionary<int, Vector3>();
    
    #endregion
    
    #region Unity Lifecycle
    
    private void Awake()
    {
        m_attachedBody = GetComponent<AdvancedVolumetricSoftBody>();
        if (m_attachedBody == null)
        {
            Debug.LogError("SoftBodyConstraint requires AdvancedVolumetricSoftBody component!", this);
            enabled = false;
        }
    }
    
    private void Start()
    {
        InitializeConstraints();
    }
    
    private void FixedUpdate()
    {
        if (!m_isInitialized) return;
        
        ApplyConstraints();
    }
    
    private void OnDestroy()
    {
        // Restore collision between bodies
        foreach (var (colA, colB) in m_disabledCollisionPairs)
        {
            if (colA != null && colB != null)
            {
                Physics.IgnoreCollision(colA, colB, false);
            }
        }
        m_disabledCollisionPairs.Clear();
    }
    
    #endregion
    
    #region Initialization
    
    private void InitializeConstraints()
    {
        if (m_attachedBody == null || m_attachedBody.trussData == null)
        {
            Debug.LogError("Attached body or truss data is null!", this);
            return;
        }
        
        // Setup collision disabling (when collideWithParent is false)
        if (!m_collideWithParent && m_baseBody != null)
        {
            SetupCollisionDisabling();
        }
        
        // Calculate actual rest lengths
        foreach (var link in m_links)
        {
            if (link.restLength <= 0)
            {
                link.actualRestLength = CalculateRestLength(link);
            }
            else
            {
                link.actualRestLength = link.restLength;
            }
            
            // For world constraints, store the anchor position
            if (link.typeB == AttachmentType.World)
            {
                var indicesAList = m_attachedBody.trussData.GetNodeSetIndices(link.nodeSetA);
                if (indicesAList != null && indicesAList.Count > 0)
                {
                    foreach (int idx in indicesAList)
                    {
                        if (!m_worldAnchors.ContainsKey(idx))
                        {
                            Vector3 worldPos = transform.TransformPoint(
                                m_attachedBody.trussData.nodes[idx].localPosition);
                            m_worldAnchors[idx] = worldPos;
                        }
                    }
                }
            }
        }
        
        m_isInitialized = true;
        Debug.Log($"[SoftBodyConstraint] Initialized {m_links.Count} links on {gameObject.name}");
    }
    
    private void SetupCollisionDisabling()
    {
        if (m_baseBody == null) return;
        
        Collider[] collidersA = m_attachedBody.GetComponentsInChildren<Collider>();
        Collider[] collidersB = m_baseBody.GetComponentsInChildren<Collider>();
        
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
    }
    
    private float CalculateRestLength(ConstraintLink link)
    {
        Vector3 pointA = GetAttachmentPoint(m_attachedBody, link.nodeSetA, link.typeA);
        
        if (link.typeB == AttachmentType.World)
        {
            // For world constraints, rest length is 0 (pin to position)
            return 0f;
        }
        else if (m_baseBody != null)
        {
            Vector3 pointB = GetAttachmentPoint(m_baseBody, link.nodeSetB, link.typeB);
            return Vector3.Distance(pointA, pointB);
        }
        
        return 1f;
    }
    
    private Vector3 GetAttachmentPoint(AdvancedVolumetricSoftBody body, string nodeSetName, AttachmentType type)
    {
        if (body == null || body.trussData == null) return Vector3.zero;
        
        var indices = body.trussData.GetNodeSetIndices(nodeSetName);
        if (indices == null || indices.Count == 0) return Vector3.zero;
        
        // Get center of all nodes in the set
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
    
    #endregion
    
    #region Constraint Application
    
    private void ApplyConstraints()
    {
        if (m_attachedBody == null || m_attachedBody.trussData == null) return;
        
        foreach (var link in m_links)
        {
            if (link.typeB == AttachmentType.World)
            {
                ApplyWorldConstraint(link);
            }
            else if (m_baseBody != null && m_baseBody.trussData != null)
            {
                ApplyBodyToBodyConstraint(link);
            }
        }
    }
    
    private void ApplyWorldConstraint(ConstraintLink link)
    {
        IReadOnlyList<int> indicesA = m_attachedBody.trussData.GetNodeSetIndices(link.nodeSetA);
        if (indicesA == null || indicesA.Count == 0) return;
        
        // Normalize stiffness to 0..1 range for constraint application
        // stiffness value is user-facing (higher = stiffer), map to correction ratio
        float stiffnessRatio = Mathf.Clamp01(link.stiffness / 10000f);  // 10000 = very stiff
        if (link.stiffness >= float.MaxValue / 2f) stiffnessRatio = 1f; // Infinity = rigid
        
        float maxDistance = link.actualRestLength;
        
        foreach (int idx in indicesA)
        {
            if (idx < 0 || idx >= m_attachedBody.trussData.nodes.Count) continue;
            
            var node = m_attachedBody.trussData.nodes[idx];
            if (node.isPinned || node.inverseMass <= 0) continue;
            
            // Get anchor position
            if (!m_worldAnchors.TryGetValue(idx, out Vector3 anchor)) continue;
            
            Vector3 toAnchor = anchor - node.worldPosition;
            float distance = toAnchor.magnitude;
            
            if (distance > maxDistance + 0.0001f)
            {
                // World constraint: move node fully toward anchor (no inverse mass scaling)
                float error = distance - maxDistance;
                Vector3 correction = (toAnchor / distance) * error * stiffnessRatio;
                
                node.worldPosition += correction;
                
                // Apply damping
                if (link.damping > 0)
                {
                    Vector3 velocity = node.Velocity;
                    node.oldPosition = node.worldPosition - velocity * (1f - link.damping);
                }
            }
        }
    }
    
    private void ApplyBodyToBodyConstraint(ConstraintLink link)
    {
        IReadOnlyList<int> indicesA = m_attachedBody.trussData.GetNodeSetIndices(link.nodeSetA);
        IReadOnlyList<int> indicesB = m_baseBody.trussData.GetNodeSetIndices(link.nodeSetB);
        
        if (indicesA == null || indicesB == null || 
            indicesA.Count == 0 || indicesB.Count == 0) return;
        
        // Warn about O(N*M) complexity
        int linkCount = indicesA.Count * indicesB.Count;
        if (linkCount > 100)
        {
            Debug.LogWarning($"[SoftBodyConstraint] High complexity: {linkCount} constraint pairs between {link.nodeSetA} and {link.nodeSetB}. Consider using smaller node sets.");
        }
        
        // Normalize stiffness to 0..1 range
        float stiffnessRatio = Mathf.Clamp01(link.stiffness / 10000f);
        if (link.stiffness >= float.MaxValue / 2f) stiffnessRatio = 1f;
        
        // Connect each node in setA to each node in setB
        foreach (int idxA in indicesA)
        {
            if (idxA < 0 || idxA >= m_attachedBody.trussData.nodes.Count) continue;
            var nodeA = m_attachedBody.trussData.nodes[idxA];
            if (nodeA.isPinned) continue;
            
            foreach (int idxB in indicesB)
            {
                if (idxB < 0 || idxB >= m_baseBody.trussData.nodes.Count) continue;
                var nodeB = m_baseBody.trussData.nodes[idxB];
                
                Vector3 delta = nodeB.worldPosition - nodeA.worldPosition;
                float currentLength = delta.magnitude;
                
                if (currentLength < 0.0001f) continue;
                
                float error = currentLength - link.actualRestLength;
                Vector3 correction = (delta / currentLength) * error * stiffnessRatio;
                
                float totalInvMass = nodeA.inverseMass + (nodeB.isPinned ? 0 : nodeB.inverseMass);
                if (totalInvMass <= 0) continue;
                
                // Apply correction
                if (nodeA.inverseMass > 0)
                {
                    nodeA.worldPosition += correction * (nodeA.inverseMass / totalInvMass);
                }
                
                if (!nodeB.isPinned && nodeB.inverseMass > 0)
                {
                    nodeB.worldPosition -= correction * (nodeB.inverseMass / totalInvMass);
                }
            }
        }
    }
    
    #endregion
    
    #region Gizmos
    
    private void OnDrawGizmosSelected()
    {
        if (!m_showLinks || m_attachedBody == null || m_attachedBody.trussData == null) return;
        
        foreach (var link in m_links)
        {
            if (!link.show) continue;
            
            Vector3 pointA = GetAttachmentPoint(m_attachedBody, link.nodeSetA, link.typeA);
            
            if (link.typeB == AttachmentType.World)
            {
                // Draw sphere at anchor point
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(pointA, 0.05f);
            }
            else if (m_baseBody != null && m_baseBody.trussData != null)
            {
                // Draw line to other body
                Vector3 pointB = GetAttachmentPoint(m_baseBody, link.nodeSetB, link.typeB);
                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(pointA, pointB);
            }
        }
    }
    
    #endregion
}
