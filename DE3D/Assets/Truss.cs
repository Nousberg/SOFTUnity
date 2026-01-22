/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    AI-Assisted Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */
using UnityEngine;
using System.Collections.Generic;
using System.Linq;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace DynamicEngine
{
    #region Truss
    [CreateAssetMenu(fileName = "Truss", menuName = "DynamicEngine/Truss")]
    public class Truss : ScriptableObject
    {
        [SerializeField] private Vector3[] nodePositions;
        [SerializeField] private List<Beam> beams = new List<Beam>();
        [SerializeField] private List<int> pinnedNodes = new List<int>();
        [SerializeField] private List<float> nodeMasses = new List<float>();
        [SerializeField] private List<Face> faces = new List<Face>();
        [SerializeField] private List<NodeSet> nodeSets = new List<NodeSet>();
        [SerializeField] private List<LinkSet> linkSets = new List<LinkSet>();
        [SerializeField] public float compliance = 0.01f;
        [SerializeField] public float DefaultBeamDamping = 0.3f;
        [SerializeField] public float uniformMassEditorValue = 0.5f;


        public Vector3[] NodePositions => nodePositions;
        public List<int> PinnedNodes => pinnedNodes;
        public List<float> NodeMasses => nodeMasses;
        public List<Face> Faces => faces;
        public List<Face> GetTrussFaces() => faces;
        public List<NodeSet> GetNodeSets() => nodeSets;
        public List<LinkSet> GetLinkSets() => linkSets;
        public List<Beam> GetTrussBeams() => beams;

        public int[] GetNodeSetIndices(string nodeSetName)
        {
            NodeSet nodeSet = nodeSets.Find(ns => ns.name == nodeSetName);
            return nodeSet != null ? nodeSet.nodeIndices.ToArray() : null;
        }

        public int[] GetLinkSetIndices(string linkSetName)
        {
            LinkSet linkSet = linkSets.Find(ls => ls.name == linkSetName);
            return linkSet != null ? linkSet.linkIndices.ToArray() : null;
        }

        public void SetNodePositions(Vector3[] positions)
        {
            nodePositions = positions != null ? new Vector3[positions.Length] : new Vector3[0];
            if (positions != null)
                System.Array.Copy(positions, nodePositions, positions.Length);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetBeams(List<Beam> newBeams)
        {
            beams = newBeams != null ? new List<Beam>(newBeams) : new List<Beam>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetPinnedNodes(List<int> newPinnedNodes)
        {
            pinnedNodes = newPinnedNodes != null ? new List<int>(newPinnedNodes) : new List<int>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetNodeMasses(List<float> masses)
        {
            nodeMasses = masses != null ? new List<float>(masses) : new List<float>();

            while (nodeMasses.Count < nodePositions.Length)
                nodeMasses.Add(0.5f); // default mass
            while (nodeMasses.Count > nodePositions.Length)
                nodeMasses.RemoveAt(nodeMasses.Count - 1);

            for (int i = 0; i < nodeMasses.Count; i++)
            {
                nodeMasses[i] = Mathf.Max(0.1f, nodeMasses[i]);
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetUniformNodeMass(float mass)
        {
            float clampedMass = Mathf.Max(0.1f, mass);
            for (int i = 0; i < nodePositions.Length; i++)
            {
                if (i < nodeMasses.Count)
                    nodeMasses[i] = clampedMass;
                else
                    nodeMasses.Add(clampedMass);
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetNodeSets(List<NodeSet> newNodeSets)
        {
            nodeSets = newNodeSets != null ? new List<NodeSet>(newNodeSets) : new List<NodeSet>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddNodeSet(NodeSet nodeSet)
        {
            if (nodeSet != null && !nodeSets.Exists(ns => ns.name == nodeSet.name))
            {
                nodeSets.Add(new NodeSet(nodeSet.name, nodeSet.nodeIndices)
                {
                    color = nodeSet.color,
                    isVisible = nodeSet.isVisible
                });
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void RemoveNodeSet(string name)
        {
            nodeSets.RemoveAll(ns => ns.name == name);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void ValidateNodeSets()
        {
            for (int i = nodeSets.Count - 1; i >= 0; i--)
            {
                var nodeSet = nodeSets[i];
                nodeSet.nodeIndices.RemoveAll(idx => idx < 0 || idx >= nodePositions.Length);
                if (nodeSet.nodeIndices.Count == 0)
                {
                    nodeSets.RemoveAt(i);
                }
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void SetLinkSets(List<LinkSet> newLinkSets)
        {
            linkSets = newLinkSets != null ? new List<LinkSet>(newLinkSets) : new List<LinkSet>();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddLinkSet(LinkSet linkSet)
        {
            if (linkSet != null && !linkSets.Exists(ls => ls.name == linkSet.name))
            {
                linkSets.Add(new LinkSet(linkSet.name, linkSet.linkIndices)
                {
                    color = linkSet.color,
                    isVisible = linkSet.isVisible
                });
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void RemoveLinkSet(string name)
        {
            linkSets.RemoveAll(ls => ls.name == name);
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void ValidateLinkSets()
        {
            for (int i = linkSets.Count - 1; i >= 0; i--)
            {
                var linkSet = linkSets[i];
                linkSet.linkIndices.RemoveAll(idx => idx < 0 || idx >= beams.Count);
                if (linkSet.linkIndices.Count == 0)
                {
                    linkSets.RemoveAt(i);
                }
            }
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void AddFace(Face face)
        {
            if (!faces.Exists(f => f.nodeA == face.nodeA && f.nodeB == face.nodeB && f.nodeC == face.nodeC))
            {
                faces.Add(face);
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void RemoveFace(int index)
        {
            if (index >= 0 && index < faces.Count)
            {
                faces.RemoveAt(index);
#if UNITY_EDITOR
                EditorUtility.SetDirty(this);
#endif
            }
        }

        public void ClearBeams()
        {
            beams.Clear();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void ClearGrid()
        {
            nodePositions = new Vector3[0];
            beams.Clear();
            pinnedNodes.Clear();
            nodeMasses.Clear();
            faces.Clear();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        public void Clear()
        {
            nodePositions = new Vector3[0];
            beams.Clear();
            pinnedNodes.Clear();
            nodeMasses.Clear();
            faces.Clear();
            nodeSets.Clear();
            linkSets.Clear();
#if UNITY_EDITOR
            EditorUtility.SetDirty(this);
#endif
        }

        // Properties for editor compatibility
        public Beam[] Beams => beams.ToArray();
        public int[] Tetrahedra => new int[0]; // Not implemented yet
        public int[] Triangles => faces.SelectMany(f => new[] { f.nodeA, f.nodeB, f.nodeC }).ToArray();
    }
    #endregion
    #region NodeSet
    [System.Serializable]
    public class NodeSet
    {
        public string name;
        public List<int> nodeIndices = new List<int>();
        public Color color = Color.cyan;
        public bool isVisible = true;
        public NodeSet()
        {
            nodeIndices = new List<int>();
        }

        public NodeSet(string name, List<int> indices)
        {
            this.name = name;
            this.nodeIndices = new List<int>(indices);
        }

        public bool IsValid()
        {
            return !string.IsNullOrEmpty(name) && nodeIndices != null && nodeIndices.Count > 0;
        }

        public bool ContainsNode(int nodeIndex)
        {
            return nodeIndices.Contains(nodeIndex);
        }
    }
    #endregion
    #region LinkSet
    [System.Serializable]
    public class LinkSet
    {
        public string name;
        public List<int> linkIndices = new List<int>();
        public Color color = Color.magenta;
        public bool isVisible = true;

        public LinkSet()
        {
            linkIndices = new List<int>();
        }

        public LinkSet(string name, List<int> indices)
        {
            this.name = name;
            this.linkIndices = new List<int>(indices);
        }

        public bool IsValid()
        {
            return !string.IsNullOrEmpty(name) && linkIndices != null && linkIndices.Count > 0;
        }

        public bool ContainsLink(int linkIndex)
        {
            return linkIndices.Contains(linkIndex);
        }
    }
    #endregion
    #region Beam

    [System.Serializable]
    public class Beam
    {
        public int nodeA;
        public int nodeB;
        public float compliance = 1e-3f;
        public float damping = 0.3f;
        public float restLength;
        public float minLength = 0f;
        public float maxLength = float.PositiveInfinity;
        public float lagrangeMultiplier;
        public bool isBroken = false;
        public float strength = float.PositiveInfinity;
        [System.NonSerialized]
        public SoftBody bodyA;
        [System.NonSerialized]
        public SoftBody bodyB;
        public bool isEdgeSliding = false;
        public int edgeNodeA = -1; // First node of the edge
        public int edgeNodeB = -1; // Second node of the edge
        public int slidingNode = -1; // The node that slides along the edge
        public float targetPerpDistance = 0f;
        public bool isActive = true;
        public bool IsCrossBody => bodyA != null && bodyB != null && bodyA != bodyB;
        public int lastSolvedFrame = -1;

        public Beam(int nodeA, int nodeB, float compliance, float damping, float restLength,
                        SoftBody bodyA = null, SoftBody bodyB = null,
                        float minLength = 0f, float maxLength = float.PositiveInfinity)  // NEW parameters
        {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            this.compliance = compliance;
            this.damping = damping;
            this.restLength = restLength;
            this.minLength = minLength;
            this.maxLength = maxLength;
            this.lagrangeMultiplier = 0f;
            this.bodyA = bodyA;
            this.bodyB = bodyB ?? bodyA;
            this.isActive = true;
        }

        public void SetSoftBodyReferences(SoftBody bodyA, SoftBody bodyB = null)
        {
            this.bodyA = bodyA;
            this.bodyB = bodyB ?? bodyA;
        }

        public Vector3 GetNodeAWorldPosition()
        {
            return bodyA?.solver?.nodeManager?.Nodes?[nodeA]?.position ?? Vector3.zero;
        }

        public Vector3 GetNodeBWorldPosition()
        {
            return bodyB?.solver?.nodeManager?.Nodes?[nodeB]?.position ?? Vector3.zero;
        }

        public Vector3 GetNodeALocalPosition()
        {
            return bodyA?.solver?.nodeManager?.PredictedPositions?[nodeA] ?? Vector3.zero;
        }

        public Vector3 GetNodeBLocalPosition()
        {
            return bodyB?.solver?.nodeManager?.PredictedPositions?[nodeB] ?? Vector3.zero;
        }

        public bool AreNodesValid()
        {
            bool validA = bodyA?.solver?.nodeManager?.Nodes != null && nodeA >= 0 && nodeA < bodyA.solver.nodeManager.Nodes.Count && bodyA.solver.nodeManager.Nodes[nodeA] != null;
            bool validB = bodyB?.solver?.nodeManager?.Nodes != null && nodeB >= 0 && nodeB < bodyB.solver.nodeManager.Nodes.Count && bodyB.solver.nodeManager.Nodes[nodeB] != null;
            return validA && validB;
        }

        public bool IsNodeAPinned()
        {
            return bodyA?.solver?.nodeManager?.IsPinned?[nodeA] ?? false;
        }

        public bool IsNodeBPinned()
        {
            return bodyB?.solver?.nodeManager?.IsPinned?[nodeB] ?? false;
        }

        private Vector3 GetNodeAVelocity(float deltaTime)
        {
            Vector3 curr = GetNodeALocalPosition();
            Vector3 prev = bodyA?.solver?.nodeManager?.PreviousPositions?[nodeA] ?? curr;
            return (curr - prev) / deltaTime;
        }

        private Vector3 GetNodeBVelocity(float deltaTime)
        {
            Vector3 curr = GetNodeBLocalPosition();
            Vector3 prev = bodyB?.solver?.nodeManager?.PreviousPositions?[nodeB] ?? curr;
            return (curr - prev) / deltaTime;
        }

        private void UpdateNodeAPosition(Vector3 correction)
        {
            if (bodyA?.solver?.nodeManager?.PredictedPositions != null && nodeA < bodyA.solver.nodeManager.PredictedPositions.Count)
                bodyA.solver.nodeManager.PredictedPositions[nodeA] += correction;
        }

        private void UpdateNodeBPosition(Vector3 correction)
        {
            if (bodyB?.solver?.nodeManager?.PredictedPositions != null && nodeB < bodyB.solver.nodeManager.PredictedPositions.Count)
                bodyB.solver.nodeManager.PredictedPositions[nodeB] += correction;
        }
        public bool ShouldBreak(float currentForce, float strength)
        {
            if (isBroken)
                return false;

            // ONLY break when force exceeds strength
            if (!float.IsInfinity(strength) && strength > 0f)
            {
                if (Mathf.Abs(currentForce) > strength)
                    return true;
            }

            return false;
        }

    }
    #endregion
    #region Face

    [System.Serializable]
    public struct Face
    {
        public int nodeA;
        public int nodeB;
        public int nodeC;
        public bool isCollidable;  // true = NORMALTYPE (collision forces), false = NONCOLLIDABLE (anti-clip only)

        // Area plasticity fields
        public float originalRestArea;       // Saved at initialization, never modified
        public float restArea;               // Mutable, updated by plasticity
        public float areaPlasticityThreshold; // Area strain threshold to start plastic deformation (default 0.1 = 10%)
        public float areaPlasticityRate;      // Rate of permanent deformation (default 0.5)
        public float maxAreaDeformation;      // Max deviation from original (default 0.3 = 30%)

        public Face(int nodeA, int nodeB, int nodeC)
        {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            this.nodeC = nodeC;
            this.isCollidable = true;
            this.originalRestArea = 0f;
            this.restArea = 0f;
            this.areaPlasticityThreshold = 0.1f;
            this.areaPlasticityRate = 0.5f;
            this.maxAreaDeformation = 0.3f;
        }

        public Face(int nodeA, int nodeB, int nodeC, bool isCollidable)
        {
            this.nodeA = nodeA;
            this.nodeB = nodeB;
            this.nodeC = nodeC;
            this.isCollidable = isCollidable;
            this.originalRestArea = 0f;
            this.restArea = 0f;
            this.areaPlasticityThreshold = 0.1f;
            this.areaPlasticityRate = 0.5f;
            this.maxAreaDeformation = 0.3f;
        }

        /// <summary>
        /// Initialize rest area from node positions. Call once at start.
        /// </summary>
        public void InitializeRestArea(Vector3 p0, Vector3 p1, Vector3 p2)
        {
            float area = Vector3.Cross(p1 - p0, p2 - p0).magnitude * 0.5f;
            this.originalRestArea = area;
            this.restArea = area;
        }
    }
    #endregion
}
