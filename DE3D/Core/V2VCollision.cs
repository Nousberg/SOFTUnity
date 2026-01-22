/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    BeamNG-Style V2V Collision System         /____/                            
                                                                    By: Elitmers & Nousberg */
using UnityEngine;
using System.Collections.Generic;

namespace DynamicEngine
{
    /// <summary>
    /// BeamNG-style Vehicle-to-Vehicle collision system.
    /// Nodes collide with triangles (coltris), not with other nodes.
    /// Based on BeamNG documentation: https://documentation.beamng.com/modding/vehicle/sections/triangles/
    /// </summary>
    public class V2VCollision
    {
        #region Contact Data Structures

        /// <summary>
        /// Represents a contact between a node and a triangle.
        /// </summary>
        public struct NodeTriangleContact
        {
            public int nodeIndex;
            public SoftBody nodeOwner;
            public int faceIndex;
            public SoftBody faceOwner;
            public Vector3 closestPoint;
            public Vector3 normal;
            public float gap;
            public Vector3 baryWeights;
            public int approachSide;  // -1 = back, +1 = front (anti-clip side tracking)
            public float accumulatedNormalImpulse;
        }

        #endregion

        #region Fields

        private List<NodeTriangleContact> _contacts = new List<NodeTriangleContact>();
        private Dictionary<(int nodeIdx, int faceIdx, SoftBody nodeOwner, SoftBody faceOwner), int> _approachSideCache 
            = new Dictionary<(int, int, SoftBody, SoftBody), int>();
        private bool _enableDebugVisualization = false;

        #endregion

        #region Public Methods

        /// <summary>
        /// Main entry point - resolves all V2V collisions between soft bodies.
        /// </summary>
        public void ResolveCollisions(float dt, bool visualize = false)
        {
            _enableDebugVisualization = visualize;
            _contacts.Clear();

            var allBodies = SoftBody.AllSoftBodies;
            if (allBodies.Count == 0) return;

            var activeBodies = new List<SoftBody>();
            for (int i = 0; i < allBodies.Count; i++)
            {
                SoftBody body = allBodies[i];
                if (body == null || !body.enableCollision || body.solver?.nodeManager == null)
                    continue;
                activeBodies.Add(body);
            }

            if (activeBodies.Count == 0) return;

            var settings = SceneSettings.Instance;
            float activationRadius = settings.TriangleActivationRadius;
            float antiClipDist = settings.AntiClipDistance + settings.ExternalCollisionBias;
            int iterations = settings.V2VSolverIterations;

            // Detect contacts: nodes of bodyA vs triangles of bodyB (and vice versa)
            for (int a = 0; a < activeBodies.Count; a++)
            {
                SoftBody bodyA = activeBodies[a];

                for (int b = 0; b < activeBodies.Count; b++)
                {
                    SoftBody bodyB = activeBodies[b];
                    if (bodyB.truss == null) continue;

                    // Self-collision check
                    if (a == b && !bodyA.enableSelfCollision) continue;

                    DetectContacts(bodyA, bodyB, activationRadius, antiClipDist);
                }
            }

            if (_contacts.Count == 0) return;

            // Solve constraints iteratively
            SolveConstraints(iterations, dt);
        }

        #endregion

        #region Detection

        /// <summary>
        /// Detect contacts between nodes of nodeBody and triangles of faceBody.
        /// </summary>
        private void DetectContacts(SoftBody nodeBody, SoftBody faceBody, float activationRadius, float antiClipDist)
        {
            var nodeManager = nodeBody.solver.nodeManager;
            var faces = faceBody.truss.GetTrussFaces();
            var faceNodePositions = faceBody.solver.nodeManager.PredictedPositions;

            if (faces == null || faces.Count == 0) return;
            if (faceNodePositions == null || faceNodePositions.Count == 0) return;

            for (int n = 0; n < nodeManager.Nodes.Count; n++)
            {
                if (nodeManager.IsPinned[n]) continue;

                Vector3 nodePos = nodeManager.PredictedPositions[n];

                for (int f = 0; f < faces.Count; f++)
                {
                    Face face = faces[f];

                    // Validate face node indices
                    if (face.nodeA >= faceNodePositions.Count ||
                        face.nodeB >= faceNodePositions.Count ||
                        face.nodeC >= faceNodePositions.Count)
                        continue;

                    // Self-collision: skip if node is part of this triangle
                    if (nodeBody == faceBody)
                    {
                        if (n == face.nodeA || n == face.nodeB || n == face.nodeC)
                            continue;
                    }

                    Vector3 vA = faceNodePositions[face.nodeA];
                    Vector3 vB = faceNodePositions[face.nodeB];
                    Vector3 vC = faceNodePositions[face.nodeC];

                    // Quick distance check for activation (10cm rule)
                    Vector3 faceCenter = (vA + vB + vC) / 3f;
                    float distToCenter = Vector3.Distance(nodePos, faceCenter);
                    
                    // Crude activation check (expand by triangle size estimate)
                    float maxEdge = Mathf.Max(
                        Vector3.Distance(vA, vB),
                        Vector3.Distance(vB, vC),
                        Vector3.Distance(vC, vA)
                    );
                    
                    if (distToCenter > activationRadius + maxEdge * 0.6f)
                        continue;

                    // Compute closest point on triangle
                    Vector3 closestPoint = ClosestPointOnTriangle(nodePos, vA, vB, vC, out Vector3 baryWeights);
                    
                    // Compute normal (CCW winding)
                    Vector3 edge1 = vB - vA;
                    Vector3 edge2 = vC - vA;
                    Vector3 normal = Vector3.Cross(edge1, edge2);
                    float normalLen = normal.magnitude;
                    
                    if (normalLen < 0.0001f) continue; // Degenerate triangle
                    normal /= normalLen;

                    // Signed distance (positive = front side, negative = back side)
                    float signedDist = Vector3.Dot(nodePos - closestPoint, normal);
                    
                    // Improved Collision Detection:
                    // If signedDist is negative, we are inside. 
                    // To fix tunneling: If node is inside (signedDist < 0) within activation radius, 
                    // treat it as a collision that needs to be pushed out (Positive Normal).
                    
                    // Allow a small "shell" thickness for thin objects if needed, but for falling-through fix:
                    // Priority is to keep nodes OUTSIDE.
                    
                    bool isInside = signedDist < 0;
                    bool isClose = Mathf.Abs(signedDist) < antiClipDist;
                    
                    // Check if we should generate a contact
                    if (isInside || isClose)
                    {
                        // FORCE approach side to be FRONT (1) if it's not established.
                        // This prevents the "stuck inside" or "pushed deeper" behavior for tunneled nodes.
                        
                        var key = (n, f, nodeBody, faceBody);
                        int approachSide = 1; // Default to Front/Outside
                        
                        if (_approachSideCache.TryGetValue(key, out int cachedSide))
                        {
                            approachSide = cachedSide;
                        }
                        else
                        {
                            // If first contact and we are inside, ASSUME we prefer to be OUTSIDE.
                            // Only exception: if user specifically wants double-sided collision, but standard softbodies are volumes.
                            // We force side 1.
                            approachSide = 1;
                            _approachSideCache[key] = approachSide;
                        }

                        // Determine contact normal based on approach side
                        // If side is 1 (Front), we push along Normal (Out).
                        // If side is -1 (Back), we push along -Normal (In).
                        Vector3 contactNormal = approachSide >= 0 ? normal : -normal;
                        
                        // Recalculate gap:
                        // Gap should be relative to the surface + antiClipDist
                        // If approachSide=1 (Front): valid region is d >= -antiClipDist? No, usually valid is d > 0.
                        // We want to enforce d >= antiClipDist (maintain buffer)? 
                        // Or just d >= 0?
                        // Standard BeamNG: gap = dist - thickness.
                        // Here: gap = Abs(dist) - antiClipDist is used in original code.
                        // If we are strictly finding penetration: 
                        // Penetration = antiClipDist - signedDist (if Front).
                        
                        // Modified logic:
                        // If approachSide is Front (1):
                        //    Target is dist >= antiClipDist (or 0). 
                        //    Penetration is (antiClipDist - signedDist).
                        //    Gap = -Penetration.
                        
                        float gap;
                        if (approachSide >= 0)
                            gap = signedDist - antiClipDist; 
                        else
                            gap = -signedDist - antiClipDist;
                            
                        // If gap < 0, we have contact
                        if (gap < 0f)
                        {
                             _contacts.Add(new NodeTriangleContact
                            {
                                nodeIndex = n,
                                nodeOwner = nodeBody,
                                faceIndex = f,
                                faceOwner = faceBody,
                                closestPoint = closestPoint,
                                normal = contactNormal,
                                gap = gap,
                                baryWeights = baryWeights,
                                approachSide = approachSide,
                                accumulatedNormalImpulse = 0f
                            });
                        }
                    }
                    else
                    {
                        // Clear cache if safely outside
                        var key = (n, f, nodeBody, faceBody);
                         if (_approachSideCache.ContainsKey(key))
                        {
                            _approachSideCache.Remove(key);
                        }
                    }
                }
            }
        }

        #endregion

        #region Solver

        /// <summary>
        /// Iteratively solve contact constraints with Baumgarte stabilization.
        /// </summary>
        private void SolveConstraints(int iterations, float dt)
        {
            var settings = SceneSettings.Instance;
            float beta = settings.V2VBaumgarteFactor;
            float restitution = settings.V2VRestitution;
            float friction = settings.V2VFriction;

            for (int iter = 0; iter < iterations; iter++)
            {
                for (int c = 0; c < _contacts.Count; c++)
                {
                    NodeTriangleContact contact = _contacts[c];
                    
                    // Skip NONCOLLIDABLE triangles for collision forces (but anti-clip still handled via position)
                    Face face = contact.faceOwner.truss.GetTrussFaces()[contact.faceIndex];
                    if (!face.isCollidable)
                    {
                        // For NONCOLLIDABLE: only do position projection, no impulse
                        ApplyAntiClipProjection(ref contact, dt);
                        continue;
                    }

                    SolveNormalConstraint(ref contact, beta, restitution, dt);
                    SolveFrictionConstraint(ref contact, friction, dt);

                    // Update accumulated impulse
                    _contacts[c] = contact;
                }
            }
        }

        private void SolveNormalConstraint(ref NodeTriangleContact contact, float beta, float restitution, float dt)
        {
            var nodeManager = contact.nodeOwner.solver.nodeManager;
            var faceNodeManager = contact.faceOwner.solver.nodeManager;
            var nodeMasses = contact.nodeOwner.solver.nodeMasses;
            var faceMasses = contact.faceOwner.solver.nodeMasses;

            Face face = contact.faceOwner.truss.GetTrussFaces()[contact.faceIndex];

            // Get masses
            float mNode = nodeMasses[contact.nodeIndex];
            float mA = faceMasses[face.nodeA];
            float mB = faceMasses[face.nodeB];
            float mC = faceMasses[face.nodeC];

            float wA = contact.baryWeights.x;
            float wB = contact.baryWeights.y;
            float wC = contact.baryWeights.z;

            // Check for pinned nodes
            float invMNode = nodeManager.IsPinned[contact.nodeIndex] ? 0f : 1f / mNode;
            float invMA = faceNodeManager.IsPinned[face.nodeA] ? 0f : wA * wA / mA;
            float invMB = faceNodeManager.IsPinned[face.nodeB] ? 0f : wB * wB / mB;
            float invMC = faceNodeManager.IsPinned[face.nodeC] ? 0f : wC * wC / mC;

            // Effective mass
            float K = invMNode + invMA + invMB + invMC;
            if (K < 0.0001f) return;

            // Get velocities
            Vector3 vNode = (nodeManager.PredictedPositions[contact.nodeIndex] - 
                            nodeManager.PreviousPositions[contact.nodeIndex]) / dt;
            
            Vector3 vTriangle = 
                wA * (faceNodeManager.PredictedPositions[face.nodeA] - faceNodeManager.PreviousPositions[face.nodeA]) / dt +
                wB * (faceNodeManager.PredictedPositions[face.nodeB] - faceNodeManager.PreviousPositions[face.nodeB]) / dt +
                wC * (faceNodeManager.PredictedPositions[face.nodeC] - faceNodeManager.PreviousPositions[face.nodeC]) / dt;

            // Relative velocity
            Vector3 vRel = vNode - vTriangle;
            float vn = Vector3.Dot(vRel, contact.normal);

            // Baumgarte positional correction
            float b = (beta / dt) * Mathf.Min(contact.gap, 0f);

            // Restitution (only for separating velocity)
            float vnTarget = vn < 0 ? -restitution * vn : 0f;

            // Compute impulse
            float lambda = -(vn - vnTarget + b) / K;

            // Clamp to unilateral (can only push, not pull)
            float oldImpulse = contact.accumulatedNormalImpulse;
            contact.accumulatedNormalImpulse = Mathf.Max(0f, oldImpulse + lambda);
            lambda = contact.accumulatedNormalImpulse - oldImpulse;

            // Apply impulse to node
            if (!nodeManager.IsPinned[contact.nodeIndex])
            {
                Vector3 deltaV = (lambda / mNode) * contact.normal;
                nodeManager.PredictedPositions[contact.nodeIndex] += deltaV * dt;
            }

            // Apply impulse to triangle vertices
            if (!faceNodeManager.IsPinned[face.nodeA])
            {
                Vector3 deltaV = -(wA * lambda / mA) * contact.normal;
                faceNodeManager.PredictedPositions[face.nodeA] += deltaV * dt;
            }
            if (!faceNodeManager.IsPinned[face.nodeB])
            {
                Vector3 deltaV = -(wB * lambda / mB) * contact.normal;
                faceNodeManager.PredictedPositions[face.nodeB] += deltaV * dt;
            }
            if (!faceNodeManager.IsPinned[face.nodeC])
            {
                Vector3 deltaV = -(wC * lambda / mC) * contact.normal;
                faceNodeManager.PredictedPositions[face.nodeC] += deltaV * dt;
            }

            // Debug visualization
            if (_enableDebugVisualization)
            {
                Debug.DrawRay(contact.closestPoint, contact.normal * 0.1f, Color.red, 0.02f);
            }
        }

        private void SolveFrictionConstraint(ref NodeTriangleContact contact, float friction, float dt)
        {
            if (contact.accumulatedNormalImpulse <= 0f) return;

            var nodeManager = contact.nodeOwner.solver.nodeManager;
            var faceNodeManager = contact.faceOwner.solver.nodeManager;
            var nodeMasses = contact.nodeOwner.solver.nodeMasses;
            var faceMasses = contact.faceOwner.solver.nodeMasses;

            Face face = contact.faceOwner.truss.GetTrussFaces()[contact.faceIndex];

            float mNode = nodeMasses[contact.nodeIndex];
            float mA = faceMasses[face.nodeA];
            float mB = faceMasses[face.nodeB];
            float mC = faceMasses[face.nodeC];

            float wA = contact.baryWeights.x;
            float wB = contact.baryWeights.y;
            float wC = contact.baryWeights.z;

            // Get velocities
            Vector3 vNode = (nodeManager.PredictedPositions[contact.nodeIndex] - 
                            nodeManager.PreviousPositions[contact.nodeIndex]) / dt;
            
            Vector3 vTriangle = 
                wA * (faceNodeManager.PredictedPositions[face.nodeA] - faceNodeManager.PreviousPositions[face.nodeA]) / dt +
                wB * (faceNodeManager.PredictedPositions[face.nodeB] - faceNodeManager.PreviousPositions[face.nodeB]) / dt +
                wC * (faceNodeManager.PredictedPositions[face.nodeC] - faceNodeManager.PreviousPositions[face.nodeC]) / dt;

            Vector3 vRel = vNode - vTriangle;

            // Tangent velocity
            Vector3 vn = Vector3.Dot(vRel, contact.normal) * contact.normal;
            Vector3 vt = vRel - vn;
            float vtMag = vt.magnitude;

            if (vtMag < 0.0001f) return;

            Vector3 tangent = vt / vtMag;

            // Effective mass for tangent
            float invMNode = nodeManager.IsPinned[contact.nodeIndex] ? 0f : 1f / mNode;
            float invMA = faceNodeManager.IsPinned[face.nodeA] ? 0f : wA * wA / mA;
            float invMB = faceNodeManager.IsPinned[face.nodeB] ? 0f : wB * wB / mB;
            float invMC = faceNodeManager.IsPinned[face.nodeC] ? 0f : wC * wC / mC;
            float K = invMNode + invMA + invMB + invMC;
            if (K < 0.0001f) return;

            // Friction impulse (Coulomb clamp)
            float maxFriction = friction * contact.accumulatedNormalImpulse;
            float lambdaT = -vtMag / K;
            lambdaT = Mathf.Clamp(lambdaT, -maxFriction, maxFriction);

            // Apply friction impulse
            if (!nodeManager.IsPinned[contact.nodeIndex])
            {
                Vector3 deltaV = (lambdaT / mNode) * tangent;
                nodeManager.PredictedPositions[contact.nodeIndex] += deltaV * dt;
            }
            if (!faceNodeManager.IsPinned[face.nodeA])
            {
                Vector3 deltaV = -(wA * lambdaT / mA) * tangent;
                faceNodeManager.PredictedPositions[face.nodeA] += deltaV * dt;
            }
            if (!faceNodeManager.IsPinned[face.nodeB])
            {
                Vector3 deltaV = -(wB * lambdaT / mB) * tangent;
                faceNodeManager.PredictedPositions[face.nodeB] += deltaV * dt;
            }
            if (!faceNodeManager.IsPinned[face.nodeC])
            {
                Vector3 deltaV = -(wC * lambdaT / mC) * tangent;
                faceNodeManager.PredictedPositions[face.nodeC] += deltaV * dt;
            }
        }

        /// <summary>
        /// For NONCOLLIDABLE triangles: project node position to correct side without impulse.
        /// </summary>
        private void ApplyAntiClipProjection(ref NodeTriangleContact contact, float dt)
        {
            var nodeManager = contact.nodeOwner.solver.nodeManager;
            if (nodeManager.IsPinned[contact.nodeIndex]) return;

            // Simply push node to the correct side of the triangle
            float correction = -contact.gap;
            if (correction > 0f)
            {
                nodeManager.PredictedPositions[contact.nodeIndex] += contact.normal * correction * 0.5f;
            }
        }

        #endregion

        #region Geometry Helpers

        /// <summary>
        /// Compute closest point on triangle to point p, and return barycentric weights.
        /// </summary>
        private Vector3 ClosestPointOnTriangle(Vector3 p, Vector3 a, Vector3 b, Vector3 c, out Vector3 baryWeights)
        {
            // Compute vectors
            Vector3 ab = b - a;
            Vector3 ac = c - a;
            Vector3 ap = p - a;

            float d1 = Vector3.Dot(ab, ap);
            float d2 = Vector3.Dot(ac, ap);

            // Vertex region outside A
            if (d1 <= 0f && d2 <= 0f)
            {
                baryWeights = new Vector3(1f, 0f, 0f);
                return a;
            }

            Vector3 bp = p - b;
            float d3 = Vector3.Dot(ab, bp);
            float d4 = Vector3.Dot(ac, bp);

            // Vertex region outside B
            if (d3 >= 0f && d4 <= d3)
            {
                baryWeights = new Vector3(0f, 1f, 0f);
                return b;
            }

            // Edge region AB
            float vc = d1 * d4 - d3 * d2;
            if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            {
                float v = d1 / (d1 - d3);
                baryWeights = new Vector3(1f - v, v, 0f);
                return a + v * ab;
            }

            Vector3 cp = p - c;
            float d5 = Vector3.Dot(ab, cp);
            float d6 = Vector3.Dot(ac, cp);

            // Vertex region outside C
            if (d6 >= 0f && d5 <= d6)
            {
                baryWeights = new Vector3(0f, 0f, 1f);
                return c;
            }

            // Edge region AC
            float vb = d5 * d2 - d1 * d6;
            if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            {
                float w = d2 / (d2 - d6);
                baryWeights = new Vector3(1f - w, 0f, w);
                return a + w * ac;
            }

            // Edge region BC
            float va = d3 * d6 - d5 * d4;
            if (va <= 0f && (d4 - d3) >= 0f && (d5 - d6) >= 0f)
            {
                float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
                baryWeights = new Vector3(0f, 1f - w, w);
                return b + w * (c - b);
            }

            // Inside triangle
            float denom = 1f / (va + vb + vc);
            float v2 = vb * denom;
            float w2 = vc * denom;
            baryWeights = new Vector3(1f - v2 - w2, v2, w2);
            return a + ab * v2 + ac * w2;
        }

        #endregion

        #region Cache Management

        /// <summary>
        /// Clear approach side cache (call when resetting simulation).
        /// </summary>
        public void ClearCache()
        {
            _approachSideCache.Clear();
            _contacts.Clear();
        }

        #endregion
    }
}
