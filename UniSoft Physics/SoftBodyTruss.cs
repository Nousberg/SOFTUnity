using UnityEngine;
using System.Collections.Generic;
using System;

/// <summary>
/// ScriptableObject that stores the physics mesh data for a soft body.
/// This allows editing outside of runtime and preserving data between sessions.
/// </summary>
[CreateAssetMenu(fileName = "NewSoftBodyTruss", menuName = "Physics/Soft Body Truss", order = 1)]
public class SoftBodyTruss : ScriptableObject
{
    #region Data Classes
    
    [Serializable]
    public class TrussNode
    {
        public Vector3 localPosition;
        [Range(0f, 10f)]
        public float inverseMass = 1f;
        public bool isPinned;
        public int originalVertexIndex = -1;
        
        // Runtime-only data
        [NonSerialized] public Vector3 worldPosition;
        [NonSerialized] public Vector3 oldPosition;
        [NonSerialized] public bool isSelected;
        
        public Vector3 Velocity => worldPosition - oldPosition;
        
        public TrussNode() { }
        
        public TrussNode(Vector3 localPos, float invMass = 1f, int vertIndex = -1)
        {
            localPosition = localPos;
            inverseMass = invMass;
            isPinned = false;
            originalVertexIndex = vertIndex;
            isSelected = false;
        }
        
        public TrussNode Clone()
        {
            return new TrussNode
            {
                localPosition = localPosition,
                inverseMass = inverseMass,
                isPinned = isPinned,
                originalVertexIndex = originalVertexIndex
            };
        }
    }
    
    [Serializable]
    public class TrussBeam
    {
        public int nodeA;
        public int nodeB;
        public float restLength;
        [Range(-1f, 1f)]
        public float stiffnessOverride = -1f;
        
        [NonSerialized] public bool isSelected;
        
        public TrussBeam() { }
        
        public TrussBeam(int a, int b, float length)
        {
            nodeA = a;
            nodeB = b;
            restLength = length;
            stiffnessOverride = -1f;
            isSelected = false;
        }
        
        public TrussBeam Clone()
        {
            return new TrussBeam
            {
                nodeA = nodeA,
                nodeB = nodeB,
                restLength = restLength,
                stiffnessOverride = stiffnessOverride
            };
        }
    }
    
    [Serializable]
    public class TrussTetrahedron
    {
        public int[] nodes = new int[4];
        public float restVolume;
        
        [NonSerialized] public float currentStress;
        
        public TrussTetrahedron() { nodes = new int[4]; }
        
        public TrussTetrahedron(int a, int b, int c, int d, float volume)
        {
            nodes = new int[] { a, b, c, d };
            restVolume = volume;
        }
    }
    
    [Serializable]
    public class TrussTriangle
    {
        public int[] nodes = new int[3];
        public float restArea;
        
        [NonSerialized] public float currentStress;
        
        public TrussTriangle() { nodes = new int[3]; }
        
        public TrussTriangle(int a, int b, int c, float area)
        {
            nodes = new int[] { a, b, c };
            restArea = area;
        }
    }
    
    [Serializable]
    public class NodeSet
    {
        public string name;
        public List<int> nodeIndices = new List<int>();
        
        public NodeSet() { }
        
        public NodeSet(string name, params int[] indices)
        {
            this.name = name;
            nodeIndices = new List<int>(indices);
        }
    }
    
    #endregion
    
    #region Serialized Data
    
    [Header("Mesh Data")]
    public List<TrussNode> nodes = new List<TrussNode>();
    public List<TrussBeam> beams = new List<TrussBeam>();
    public List<TrussTetrahedron> tetrahedra = new List<TrussTetrahedron>();
    public List<TrussTriangle> triangles = new List<TrussTriangle>();
    
    [Header("Node Sets")]
    public List<NodeSet> nodeSets = new List<NodeSet>();
    
    [Header("Generation Settings")]
    public bool usedTetrahedra = true;
    
    #endregion
    
    #region Methods
    
    public void Clear()
    {
        nodes.Clear();
        beams.Clear();
        tetrahedra.Clear();
        triangles.Clear();
        nodeSets.Clear();
    }
    
    public void ClearBeams()
    {
        beams.Clear();
    }
    
    public void ClearGrid()
    {
        nodes.Clear();
        beams.Clear();
        tetrahedra.Clear();
        triangles.Clear();
        nodeSets.Clear();
    }
    
    public int AddNode(Vector3 localPosition, float inverseMass = 1f, int originalVertexIndex = -1)
    {
        var node = new TrussNode(localPosition, inverseMass, originalVertexIndex);
        nodes.Add(node);
        return nodes.Count - 1;
    }
    
    public void AddBeam(int nodeA, int nodeB, float restLength)
    {
        // Check if beam already exists
        foreach (var beam in beams)
        {
            if ((beam.nodeA == nodeA && beam.nodeB == nodeB) ||
                (beam.nodeA == nodeB && beam.nodeB == nodeA))
                return;
        }
        
        beams.Add(new TrussBeam(nodeA, nodeB, restLength));
    }
    
    public void RemoveNode(int index)
    {
        if (index < 0 || index >= nodes.Count) return;
        
        // Remove beams connected to this node
        beams.RemoveAll(b => b.nodeA == index || b.nodeB == index);
        
        // Update beam indices
        foreach (var beam in beams)
        {
            if (beam.nodeA > index) beam.nodeA--;
            if (beam.nodeB > index) beam.nodeB--;
        }
        
        // Update tetrahedra
        tetrahedra.RemoveAll(t => 
            t.nodes[0] == index || t.nodes[1] == index || 
            t.nodes[2] == index || t.nodes[3] == index);
        
        foreach (var tet in tetrahedra)
        {
            for (int i = 0; i < 4; i++)
                if (tet.nodes[i] > index) tet.nodes[i]--;
        }
        
        // Update triangles
        triangles.RemoveAll(t => 
            t.nodes[0] == index || t.nodes[1] == index || t.nodes[2] == index);
        
        foreach (var tri in triangles)
        {
            for (int i = 0; i < 3; i++)
                if (tri.nodes[i] > index) tri.nodes[i]--;
        }
        
        // Update nodeSets: remove the deleted index and decrement higher indices
        foreach (var nodeSet in nodeSets)
        {
            nodeSet.nodeIndices.Remove(index);
            for (int i = 0; i < nodeSet.nodeIndices.Count; i++)
            {
                if (nodeSet.nodeIndices[i] > index)
                    nodeSet.nodeIndices[i]--;
            }
        }
        
        nodes.RemoveAt(index);
    }
    
    public void RemoveSelectedNodes()
    {
        List<int> toRemove = new List<int>();
        for (int i = 0; i < nodes.Count; i++)
        {
            if (nodes[i].isSelected)
                toRemove.Add(i);
        }
        
        // Remove in reverse order
        toRemove.Sort();
        toRemove.Reverse();
        
        foreach (int index in toRemove)
            RemoveNode(index);
    }
    
    public void RemoveSelectedBeams()
    {
        beams.RemoveAll(b => b.isSelected);
    }
    
    public void DeselectAllNodes()
    {
        foreach (var node in nodes)
            node.isSelected = false;
    }
    
    public void DeselectAllBeams()
    {
        foreach (var beam in beams)
            beam.isSelected = false;
    }
    
    public List<int> GetSelectedNodeIndices()
    {
        List<int> indices = new List<int>();
        for (int i = 0; i < nodes.Count; i++)
        {
            if (nodes[i].isSelected)
                indices.Add(i);
        }
        return indices;
    }
    
    public bool HasSelectedNodes()
    {
        foreach (var node in nodes)
            if (node.isSelected) return true;
        return false;
    }
    
    public bool HasSelectedBeams()
    {
        foreach (var beam in beams)
            if (beam.isSelected) return true;
        return false;
    }
    
    public int SelectedNodeCount()
    {
        int count = 0;
        foreach (var node in nodes)
            if (node.isSelected) count++;
        return count;
    }
    
    #endregion
    
    #region NodeSet Methods
    
    /// <summary>
    /// Get node indices for a named set. Returns the list directly (no copy).
    /// FIX: Returns IReadOnlyList instead of ToArray() to avoid GC (bug 3.2)
    /// </summary>
    public IReadOnlyList<int> GetNodeSetIndices(string setName)
    {
        if (string.IsNullOrEmpty(setName)) return null;
        
        foreach (var set in nodeSets)
        {
            if (set.name == setName)
                return set.nodeIndices;
        }
        return null;
    }
    
    public List<NodeSet> GetNodeSets()
    {
        return nodeSets;
    }
    
    public NodeSet GetNodeSet(string setName)
    {
        foreach (var set in nodeSets)
        {
            if (set.name == setName)
                return set;
        }
        return null;
    }
    
    public void AddNodeSet(string setName, List<int> indices)
    {
        // Check if set already exists
        foreach (var set in nodeSets)
        {
            if (set.name == setName)
            {
                set.nodeIndices = new List<int>(indices);
                return;
            }
        }
        
        nodeSets.Add(new NodeSet(setName, indices.ToArray()));
    }
    
    public void RemoveNodeSet(string setName)
    {
        nodeSets.RemoveAll(s => s.name == setName);
    }
    
    public string[] GetNodeSetNames()
    {
        string[] names = new string[nodeSets.Count];
        for (int i = 0; i < nodeSets.Count; i++)
            names[i] = nodeSets[i].name;
        return names;
    }
    
    #endregion
}
