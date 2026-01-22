/* ____                               ______            _            _____ ____ 
  / __ \__  ______  ____ _____ ___  (_)____/ ____/___  ____ _(_)___  _____|__  // __ \
 / / / / / / / __ \/ __ `/ __ `__ \/ / ___/ __/ / __ \/ __ `/ / __ \/ _ /___/ // / / /
/ /_/ / /_/ / / / / /_/ / / / / / / / /__/ /___/ / / / /_/ / / / / /  __/  / // /_/ / 
\____/\__, /_/ /_/\__,_/_/ /_/ /_/_/\___/_____/_/ /_/\__, /_/_/ /_/\___/  /_/ \____/  
     /____/    Soft-Body Physics for Unity3D        /____/                            
                                                                    By: Elitmers & Nousberg */
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using Unity.Collections.LowLevel.Unsafe;

#if UNITY_EDITOR
using UnityEditor;
#endif


namespace DynamicEngine
{
    public class Solver
    {
        #region Static Fields

        private static bool s_isApplicationQuitting = false;

        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.SubsystemRegistration)]
        private static void InitializeOnLoad()
        {
            s_isApplicationQuitting = false;
            Application.quitting += () => s_isApplicationQuitting = true;
        }

        #endregion

        #region Fields

        private readonly Transform _owner;
        private readonly SoftBody _ownerBody;
        public readonly NodeManager nodeManager;
        public List<Beam> beams;
        public readonly MeshDeformer meshDeformer;
        public CollisionHandler collisionHandler;

        public List<float> nodeMasses = new List<float>();

        private Truss _trussAsset;
        private Matter _matterAsset;

        // Reusable arrays for memory optimization
        private NativeArray<float3> _reusableWorldPositions;
        private readonly List<int> _reusableNeighborsList = new List<int>();
        private readonly List<GameObject> _reusableDestroyList = new List<GameObject>();

        #endregion

        #region Properties
        public bool visualizeForces { get; set; } = false;
        public float compliance { get; set; } = 1e-3f;
        public float defaultDamping { get; set; } = 0.3f;
        public float maxDeformation { get; set; } = 1f;
        public float internalPressure { get; set; } = 0f;
        public float areaCompliance { get; set; } = 0f; // New parameter

        private Vector3 _cachedVolumeCenter = Vector3.zero;

        #endregion

        #region Visualization Properties

        public bool showNodesAndLinks = false;
        public bool showNodes = true;
        public bool showLinks = true;
        public bool showNodeIndices = false;
        public bool showLinkForces = true;
        public float nodeDisplaySize = 0.1f;
        public bool debugEdgeSliding = true;
        public Color nodeColor = Color.green;
        public Color pinnedNodeColor = Color.red;
        public Color linkColor = Color.blue;
        public Color influenceRadiusColor = new Color(1f, 1f, 0f, 0.3f);
        public float forceVisualizationScale = 0.01f;
        private int _currentFrameIndex = 0;

        #endregion

        #region Job Structs

        public struct BeamData
        {
            public int nodeA;
            public int nodeB;
            public float compliance;
            public float damping;
            public float minLength;
            public float maxLength;
            public bool isActive;
        }

        public struct TriangleData
        {
            public int nodeA;
            public int nodeB;
            public int nodeC;
            public float restArea;
            public float compliance;
            public bool isCollidable;
            // Plasticity fields
            public float originalRestArea;
            public float areaPlasticityThreshold;
            public float areaPlasticityRate;
            public float maxAreaDeformation;
        }

        [BurstCompile]
        private struct VerletIntegrationJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<float3> currentWorldPositions;
            [ReadOnly] public NativeArray<float3> previousPositions;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public float dt;
            [ReadOnly] public float dampingFactor;
            [ReadOnly] public float3 worldGravity;
            [ReadOnly] public float dtSquared;

            public NativeArray<float3> predictedPositions;

            public void Execute(int i)
            {
                if (isPinned[i]) return;

                float3 currentWorldPos = currentWorldPositions[i];
                float3 prevWorldPos = previousPositions[i];
                float3 worldVelocity = (currentWorldPos - prevWorldPos) / dt;

                worldVelocity *= dampingFactor;

                float3 newWorldPosition = currentWorldPos + worldVelocity * dt + worldGravity * dtSquared;
                predictedPositions[i] = newWorldPosition;
            }
        }

        [BurstCompile]
        private struct SolveDistanceConstraintsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<BeamData> beams;
            [ReadOnly] public NativeArray<float3> previousPositions;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public float dt;
            [ReadOnly] public float worldRestLengthScale;

            [NativeDisableParallelForRestriction]
            public NativeArray<float3> predictedPositions;

            [NativeDisableParallelForRestriction]
            public NativeArray<float> beamRestLengths;

            [NativeDisableParallelForRestriction]
            public NativeArray<float> beamLagrangeMultipliers;

            public void Execute(int beamIndex)
            {
                BeamData beam = beams[beamIndex];
                if (!beam.isActive) return;

                SolveBeamConstraint(beamIndex, beam);
            }

            private void SolveBeamConstraint(int beamIndex, BeamData beam)
            {
                int nodeA = beam.nodeA;
                int nodeB = beam.nodeB;

                float3 posA = predictedPositions[nodeA];
                float3 posB = predictedPositions[nodeB];
                float3 delta = posB - posA;
                float currentLengthSq = math.lengthsq(delta);

                // Guard: Zero-length constraint
                if (currentLengthSq < PhysicsConstants.MIN_BEAM_LENGTH * PhysicsConstants.MIN_BEAM_LENGTH)
                    return;

                float currentLength = math.sqrt(currentLengthSq);
                float3 gradient = delta * math.rcp(currentLength); // Normalized direction

                // Get rest length (plasticity is applied AFTER all iterations, not here)
                float restLength = beamRestLengths[beamIndex];
                float worldRestLength = restLength * worldRestLengthScale;

                // Determine target length based on limits
                bool hasMinLimit = beam.minLength > 0f;
                bool hasMaxLimit = !math.isinf(beam.maxLength) && beam.maxLength > 0f;

                float avgScale = worldRestLengthScale;
                float worldMinLength = hasMinLimit ? beam.minLength * avgScale : 0f;
                float worldMaxLength = hasMaxLimit ? beam.maxLength * avgScale : float.PositiveInfinity;

                // Free movement between limits (slack behavior)
                float targetLength = currentLength;

                if (hasMinLimit && currentLength < worldMinLength)
                {
                    targetLength = worldMinLength; // Enforce floor
                }
                else if (hasMaxLimit && currentLength > worldMaxLength)
                {
                    targetLength = worldMaxLength; // Enforce ceiling
                }
                else if (!hasMinLimit && !hasMaxLimit)
                {
                    targetLength = worldRestLength; // Standard spring behavior
                }

                float constraint = currentLength - targetLength;

                // Early exit if constraint is satisfied
                if (math.abs(constraint) < 1e-6f) return;

                // Rayleigh damping: α_effective = α + d·dt
                // Where d is the damping coefficient
                float dampedCompliance = beam.compliance + beam.damping * dt;

                // Calculate mass-weighted inverse masses
                bool pinnedA = isPinned[nodeA];
                bool pinnedB = isPinned[nodeB];

                float wA = math.select(math.rcp(nodeMasses[nodeA]), 0f, pinnedA);
                float wB = math.select(math.rcp(nodeMasses[nodeB]), 0f, pinnedB);
                float wSum = wA + wB;

                // Guard: All nodes pinned or zero mass
                if (wSum <= 0f) return;

                // XPBD compliance: α̂ = α / dt²
                float dtSq = dt * dt;
                float effectiveCompliance = dampedCompliance * math.rcp(dtSq);
                float deltaLambda = -constraint * math.rcp(wSum + effectiveCompliance);

                if (!math.isfinite(deltaLambda))
                    return;

                // Handle inequality constraints
                bool isCable = hasMinLimit && !hasMaxLimit;  // Can only pull
                bool isStrut = hasMaxLimit && !hasMinLimit;  // Can only push

                if (isCable)
                {
                    // Cable: lambda must be >= 0 (tension only)
                    float newLambda = beamLagrangeMultipliers[beamIndex] + deltaLambda;
                    if (newLambda < 0f)
                    {
                        deltaLambda = -beamLagrangeMultipliers[beamIndex];
                    }
                }
                else if (isStrut)
                {
                    // Strut: lambda must be <= 0 (compression only)
                    float newLambda = beamLagrangeMultipliers[beamIndex] + deltaLambda;
                    if (newLambda > 0f)
                    {
                        deltaLambda = -beamLagrangeMultipliers[beamIndex];
                    }
                }

                // Accumulate Lagrange multiplier
                beamLagrangeMultipliers[beamIndex] += deltaLambda;

                // Compute position correction
                float3 correction = gradient * deltaLambda;

                // Sanity check: prevent excessive corrections
                float correctionMag = math.length(correction);
                if (correctionMag > PhysicsConstants.MAX_ALLOWED_DISPLACEMENT)
                {
                    correction *= PhysicsConstants.MAX_ALLOWED_DISPLACEMENT * math.rcp(correctionMag);
                }

                // Apply symmetric mass-weighted corrections
                if (!pinnedA)
                    predictedPositions[nodeA] -= wA * correction;
                if (!pinnedB)
                    predictedPositions[nodeB] += wB * correction;
            }
        }

        [BurstCompile]
        private struct SolveAreaConstraintsJob : IJobParallelFor
        {
            [ReadOnly] public NativeArray<TriangleData> triangles;
            [ReadOnly] public NativeArray<float> nodeMasses;
            [ReadOnly] public NativeArray<bool> isPinned;
            [ReadOnly] public float dt;
            [ReadOnly] public float worldRestLengthScale;

            [NativeDisableParallelForRestriction]
            public NativeArray<float3> predictedPositions;

            public void Execute(int index)
            {
                TriangleData tri = triangles[index];
                int nA = tri.nodeA;
                int nB = tri.nodeB;
                int nC = tri.nodeC;

                float3 p0 = predictedPositions[nA];
                float3 p1 = predictedPositions[nB];
                float3 p2 = predictedPositions[nC];

                // Area C(x) = |(p1-p0) x (p2-p0)| / 2 - A0
                // XPBD: deltaLambda = -C / (sum(w * |grad|^2) + alpha/dt^2)

                float3 u = p1 - p0;
                float3 v = p2 - p0;
                float3 cross = math.cross(u, v);
                float currentDoubleArea = math.length(cross);
                
                // Avoid division by zero for degenerate triangles
                if (currentDoubleArea < 1e-6f) return;

                float currentArea = currentDoubleArea * 0.5f;
                // restArea is raw, scale it by world scale squared? 
                // Wait, initialization usually captures world positions. 
                // Let's assume restArea is updated or we use worldRestLengthScale to scale A0.
                // Better: Calculate restArea from current WORLD positions at init, so no scaling needed inside job IF mesh is static scale.
                // But if we support scaling, we need to scale area by scale^2.
                float targetArea = tri.restArea * (worldRestLengthScale * worldRestLengthScale);

                float C = currentArea - targetArea;
                
                // Gradients
                // grad_p0 = 0.5 * (p1 - p2) x n / |n| ? No.
                // simpler: grad_area = (gradient of |cross|) / 2
                // grad |n| = n / |n| * grad n.
                // Let n_hat = cross / |cross|.
                // grad_p1 A = 0.5 * cross(v, n_hat) ?
                // Gradient of Area w.r.t p1: 0.5 * ((p2-p0) x n) / |n| = 0.5 * cross(v, n_hat) = 0.5 * cross(v, cross)/|cross|
                // Actually:
                // Area A = 0.5 * |u x v|
                // partial A / partial p1 (index B) = 0.5 * (u x v)/|u x v| x (-p2 + p0)? No.
                // Correct gradients for A = 0.5 * |(p1-p0) x (p2-p0)|:
                // n = (p1-p0) x (p2-p0)
                // mag = |n|
                // dA/dp1 = 0.5 * (1/mag) * n dot (d(n)/dp1)
                // d(n)/dp1 = d/dp1 (p1 x p2 - p1 x p0 - p0 x p2) 
                // This is getting complex.
                // Standard formula:
                // grad_p1 (B) = 0.5 * (p2 - p0) x (n / |n|) ? 
                // Let's check magnitude. |grad_p1| = 0.5 * |p2-p0|? Yes, base * height / 2.
                
                float3 n = cross;
                float3 n_hat = n / currentDoubleArea; // Normalized normal

                // Gradients of AREA
                // grad_p1 (B) = 0.5 * cross(p2 - p0, n_hat)
                float3 grad1 = 0.5f * math.cross(p2 - p0, n_hat);
                // grad_p2 (C) = 0.5 * cross(p0 - p1, n_hat)
                float3 grad2 = 0.5f * math.cross(p0 - p1, n_hat);
                // grad_p0 (A) = 0.5 * cross(p1 - p2, n_hat)
                float3 grad0 = 0.5f * math.cross(p1 - p2, n_hat);

                float w0 = isPinned[nA] ? 0f : math.rcp(nodeMasses[nA]);
                float w1 = isPinned[nB] ? 0f : math.rcp(nodeMasses[nB]);
                float w2 = isPinned[nC] ? 0f : math.rcp(nodeMasses[nC]);

                float wSum = w0 * math.lengthsq(grad0) + 
                             w1 * math.lengthsq(grad1) + 
                             w2 * math.lengthsq(grad2);

                if (wSum < 1e-6f) return;

                float alpha = tri.compliance / (dt * dt);
                float dLambda = -C / (wSum + alpha);

                // Apply
                if (w0 > 0f) predictedPositions[nA] += w0 * grad0 * dLambda;
                if (w1 > 0f) predictedPositions[nB] += w1 * grad1 * dLambda;
                if (w2 > 0f) predictedPositions[nC] += w2 * grad2 * dLambda;
            }
        }

        #endregion

        #region Constructor

        public Solver(float influenceRadius, Mesh mesh, Vector3[] originalVertices, SoftBody owner)
        {
            this.beams = new List<Beam>();
            this.meshDeformer = new MeshDeformer(mesh, originalVertices, influenceRadius);
            this.collisionHandler = new CollisionHandler();
            this._ownerBody = owner;
            this._owner = owner.transform;
            this.nodeManager = new NodeManager();
            nodeMasses = new List<float>();
        }

        #endregion

        #region Initialization Methods


        public void InitializeFromTruss(Truss truss)
        {
            _trussAsset = truss;

            if (truss != null)
            {
                nodeMasses = new List<float>(truss.NodeMasses);

                // Ensure node masses match node count
                while (nodeMasses.Count < nodeManager.Nodes.Count)
                    nodeMasses.Add(PhysicsConstants.DEFAULT_NODE_MASS);

                if (nodeMasses.Count > nodeManager.Nodes.Count)
                    nodeMasses.RemoveRange(nodeManager.Nodes.Count,
                                          nodeMasses.Count - nodeManager.Nodes.Count);

                compliance = truss.compliance;
                defaultDamping = truss.DefaultBeamDamping;
            }
            else
            {
                for (int i = 0; i < nodeManager.Nodes.Count; i++)
                    nodeMasses.Add(PhysicsConstants.DEFAULT_NODE_MASS);
            }
        }


        public void SetTrussAsset(Truss truss)
        {
            _trussAsset = truss;
        }


        public void SetMatterAsset(Matter matter)
        {
            _matterAsset = matter;
        }

        #endregion

        #region Node Query Methods


        public Vector3 GetNodeWorldPosition(int index)
        {
            if (index < 0 || index >= nodeManager.Nodes.Count)
                return Vector3.zero;

            return _owner.TransformPoint(nodeManager.Nodes[index].localPosition);
        }

        #endregion
        #region Synchronized Simulation Methods

        /// <summary>
        /// Phase 1: Prepare buffers and apply external forces (Gravity, Pressure, Motors).
        /// </summary>
        public void SubStep_PrepareAndIntegrate(float dt)
        {
            if (!ValidateSimulationState("Integrate")) return;

            int nodeCount = nodeManager.Nodes.Count;
            float damping = _matterAsset != null ? _matterAsset.CollisionDamping : PhysicsConstants.DEFAULT_COLLISION_DAMPING;
            Vector3 worldGravity = _ownerBody != null ? _ownerBody.gravity : Vector3.down * PhysicsConstants.STANDARD_GRAVITY;

            EnsureBufferSizes(nodeCount);

            // 1. Apply Forces
            ApplyPressureForces(dt);

            // 2. Predict next positions via Verlet Integration
            PerformVerletIntegration(dt, dt * dt, nodeCount, damping, worldGravity);
            _currentFrameIndex++;
        }

        public void SubStep_ResolveConstraints(float dt)
        {
            if (nodeManager.Nodes.Count == 0) return;

            int constraintIters = SceneSettings.Instance != null
                ? SceneSettings.Instance.ConstraintIterations : 20;

            SolveConstraints(dt, constraintIters, nodeManager.Nodes.Count);

            CheckAndBreakConstraints();
            ResolveCollisions(dt, nodeManager.Nodes.Count);

            // Area-based plasticity: apply AFTER collisions since we need collision data
            HashSet<int> collidedNodes = collisionHandler.GetCollidedNodes();
            ApplyAreaPlasticityPostCollision(dt, collidedNodes);
        }

        public void SubStep_Finalize()
        {
            UpdateNodePositions(nodeManager.Nodes.Count);
        }

        #endregion

        #region Main Simulation Methods


        public void Solve()
        {
            if (!ValidateSimulationState("Solve")) return;

            int numSubSteps = GetSimulationSubSteps();
            float fullDt = Time.fixedDeltaTime * GetSimulationTimeScale();
            float subDt = fullDt / numSubSteps;

            for (int step = 0; step < numSubSteps; step++)
            {
                SubStep_PrepareAndIntegrate(subDt);
                SubStep_ResolveConstraints(subDt);
                SubStep_Finalize();
            }
        }


        private bool ValidateSimulationState(string context)
        {
            if (nodeManager == null)
            {
                Debug.LogError($"[{context}] NodeManager is null");
                return false;
            }

            if (beams == null)
            {
                Debug.LogError($"[{context}] Beams list is null");
                return false;
            }

            if (nodeManager.Nodes == null || nodeManager.Nodes.Count == 0)
            {
                Debug.LogWarning($"[{context}] No nodes to simulate");
                return false;
            }

            if (_owner == null)
            {
                Debug.LogError($"[{context}] Owner transform is null");
                return false;
            }

            return true;
        }


        public int GetSimulationSubSteps()
        {
            if (SceneSettings.Instance == null)
                return 1;

            return Mathf.Clamp(SceneSettings.Instance.BaseSubSteps,
                              SceneSettings.Instance.MinSubSteps, 50);
        }


        private float GetSimulationTimeScale()
        {
            return SceneSettings.Instance != null
                ? SceneSettings.Instance.SimulationTimeScale
                : 1.0f;
        }

        #endregion

        #region Sub-Step Simulation

        private void SolveSubStep(float dt)
        {
            PrepareSimulationStep(dt, out float dtSquared, out int nodeCount,
                                 out float damping, out Vector3 worldGravity,
                                 out int constraintIters);

            ApplyPressureForces(dt);
            PerformVerletIntegration(dt, dtSquared, nodeCount, damping, worldGravity);
            SolveConstraints(dt, constraintIters, nodeCount);
            ResolveCollisions(dt, nodeCount);
            UpdateNodePositions(nodeCount);
        }

        private void PrepareSimulationStep(float dt, out float dtSquared,
                                          out int nodeCount, out float damping,
                                          out Vector3 worldGravity,
                                          out int constraintIters)
        {
            dtSquared = dt * dt;
            nodeCount = nodeManager.Nodes.Count;
            damping = _matterAsset != null
                ? _matterAsset.CollisionDamping
                : PhysicsConstants.DEFAULT_COLLISION_DAMPING;

            worldGravity = _ownerBody != null ? _ownerBody.gravity : Vector3.down * PhysicsConstants.STANDARD_GRAVITY;

            constraintIters = SceneSettings.Instance != null
                ? SceneSettings.Instance.ConstraintIterations
                : 20;

            EnsureBufferSizes(nodeCount);
        }


        private void EnsureBufferSizes(int nodeCount)
        {
            EnsureBufferSize(nodeManager.PreviousPositions, nodeCount, true);
            EnsureBufferSize(nodeManager.PredictedPositions, nodeCount, false);
        }

        private void EnsureBufferSize(List<Vector3> buffer, int requiredSize, bool isPrevious)
        {
            if (buffer.Count == requiredSize)
                return;

            buffer.Clear();

            for (int i = 0; i < requiredSize; i++)
            {
                Vector3 worldPos = nodeManager.Nodes[i] != null
                    ? _owner.TransformPoint(nodeManager.Nodes[i].localPosition)
                    : Vector3.zero;
                buffer.Add(worldPos);
            }
        }

        #endregion

        #region Verlet Integration (UPDATED)

        private NativeArray<float3> CreateCurrentWorldPositions(int nodeCount)
        {
            var currentWorldPositions = new NativeArray<float3>(nodeCount, Allocator.TempJob);

            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    Vector3 worldPos = _owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                    currentWorldPositions[i] = worldPos;
                }
                else
                {
                    currentWorldPositions[i] = float3.zero;
                }
            }

            return currentWorldPositions;
        }

        private void PerformVerletIntegration(float dt, float dtSquared, int nodeCount,
                                             float damping, Vector3 worldGravity)
        {
            nodeManager.AllocateNativeArrays();
            nodeManager.SyncToNativeArrays();

            NativeArray<float3> currentWorldPositions = CreateCurrentWorldPositions(nodeCount);
            // Convert damping parameter to exponential decay
            // dampingFactor = exp(-damping * dt)
            // For small dt: exp(-x) ≈ 1 - x, so linear damping is: 1 - damping * dt

            // Use exponential for accuracy:
            float dampingFactor = Mathf.Exp(-damping * dt);

            // Or use linear approximation (cheaper, good for small dt):
            // float dampingFactor = math.max(0f, 1.0f - damping * dt);

            var verletJob = new VerletIntegrationJob
            {
                currentWorldPositions = currentWorldPositions,
                previousPositions = nodeManager.nativePreviousPositions,
                isPinned = nodeManager.nativeIsPinned,
                dt = dt,
                dampingFactor = dampingFactor,
                worldGravity = worldGravity,
                dtSquared = dtSquared,
                predictedPositions = nodeManager.nativePredictedPositions
            };

            JobHandle verletHandle = verletJob.Schedule(nodeCount,
                                                        PhysicsConstants.VERLET_JOB_BATCH_SIZE);
            verletHandle.Complete();

            currentWorldPositions.Dispose();

            nodeManager.SyncFromNativeArrays();
        }

        #endregion

        #region Collision Resolution
        private void ResolveCollisions(float dt, int nodeCount)
        {
            if (_ownerBody != null)
            {
                if (!_ownerBody.enableCollision)
                {
                    collisionHandler.ClearCollidedNodes();
                    return;
                }

                ApplyCollisionSettings(_ownerBody.collisionRadius, _ownerBody.collisionLayers);
            }

            collisionHandler.ResolveCollisions(nodeManager, _owner, _matterAsset,
                                              nodeMasses, dt,
                                              PhysicsConstants.DEFAULT_COLLISION_EPSILON,
                                              visualizeForces);
        }

        #endregion

        public void ApplyCollisionSettings(float radius, LayerMask layers)
        {
            nodeManager.SetBaseRadius(radius);
            collisionHandler.CollisionLayerMask = layers;
        }

        #region Constraint Solving

        private void SolveConstraints(float dt, int constraintIters, int nodeCount)
        {
            var nativeBeamData = new NativeArray<BeamData>(beams.Count, Allocator.TempJob);
            var nativeBeamRestLengths = new NativeArray<float>(beams.Count, Allocator.TempJob);
            var nativeBeamLagrange = new NativeArray<float>(beams.Count, Allocator.TempJob);
            
            // Area Constraints Data
            List<Face> faces = _trussAsset != null ? _trussAsset.GetTrussFaces() : new List<Face>();
            var nativeTriangles = new NativeArray<TriangleData>(faces.Count, Allocator.TempJob);
            PrepareNativeTriangleData(nativeTriangles, faces);

            var nativeNodeMasses = new NativeArray<float>(nodeMasses.Count, Allocator.TempJob);

            PrepareNativeBeamData(nativeBeamData, nativeBeamRestLengths, nativeBeamLagrange);
            CopyNodeMassesToNative(nativeNodeMasses);

            nodeManager.SyncToNativeArrays();

            float worldRestLengthScale = _owner.TransformDirection(Vector3.right).magnitude;

            ExecuteConstraintIterations(constraintIters, nativeBeamData, nativeBeamRestLengths,
                                       nativeBeamLagrange, nativeNodeMasses, dt,
                                       worldRestLengthScale);
            
            ExecuteAreaConstraintIterations(constraintIters, nativeTriangles, nativeNodeMasses, dt, worldRestLengthScale);

            UpdateBeamsFromNativeData(nativeBeamRestLengths, nativeBeamLagrange);

            nodeManager.SyncFromNativeArrays();

            // Dispose native arrays
            nativeBeamData.Dispose();
            nativeBeamRestLengths.Dispose();
            nativeBeamLagrange.Dispose();
            nativeTriangles.Dispose();
            nativeNodeMasses.Dispose();

            SolveDistanceConstraintsCrossBody(dt);
        }


        private void PrepareNativeBeamData(NativeArray<BeamData> nativeBeamData,
                                  NativeArray<float> nativeBeamRestLengths,
                                  NativeArray<float> nativeBeamLagrange)
        {
            // Get runtime parameters from SoftBody
            bool deformationEnabled = _ownerBody != null && _ownerBody.enableDeformation;

            for (int i = 0; i < beams.Count; i++)
            {
                if (!beams[i].IsCrossBody)
                {
                    nativeBeamData[i] = new BeamData
                    {
                        nodeA = beams[i].nodeA,
                        nodeB = beams[i].nodeB,
                        // Fix 1: Use beam's compliance directly. If deformation disabled, rigid (0).
                        compliance = deformationEnabled ? beams[i].compliance : 0f,
                        // Fix 2: Use beam's damping directly.
                        damping = beams[i].damping,
                        minLength = beams[i].minLength,
                        maxLength = beams[i].maxLength,
                        isActive = beams[i].isActive
                    };
                    nativeBeamRestLengths[i] = beams[i].restLength;
                    nativeBeamLagrange[i] = beams[i].lagrangeMultiplier;
                }
                else
                {
                    nativeBeamData[i] = new BeamData { isActive = false };
                }
            }
        }

        private void CopyNodeMassesToNative(NativeArray<float> nativeNodeMasses)
        {
            for (int i = 0; i < nodeMasses.Count; i++)
            {
                nativeNodeMasses[i] = nodeMasses[i];
            }
        }


        private void ExecuteConstraintIterations(int constraintIters,
                                        NativeArray<BeamData> nativeBeamData,
                                        NativeArray<float> nativeBeamRestLengths,
                                        NativeArray<float> nativeBeamLagrange,
                                        NativeArray<float> nativeNodeMasses,
                                        float dt, float worldRestLengthScale)
        {
            // Calculate optimal batch size based on beam count
            int beamCount = nativeBeamData.Length;
            int batchSize = math.max(1, beamCount / (Unity.Jobs.LowLevel.Unsafe.JobsUtility.JobWorkerCount * 4));
            batchSize = math.clamp(batchSize, 8, 128);

            for (int iter = 0; iter < constraintIters; iter++)
            {
                var constraintJob = new SolveDistanceConstraintsJob
                {
                    beams = nativeBeamData,
                    previousPositions = nodeManager.nativePreviousPositions,
                    isPinned = nodeManager.nativeIsPinned,
                    nodeMasses = nativeNodeMasses,
                    dt = dt,
                    worldRestLengthScale = worldRestLengthScale,
                    predictedPositions = nodeManager.nativePredictedPositions,
                    beamRestLengths = nativeBeamRestLengths,
                    beamLagrangeMultipliers = nativeBeamLagrange
                };

                // Schedule parallel job
                JobHandle constraintHandle = constraintJob.Schedule(beamCount, batchSize);
                constraintHandle.Complete();
            }
        }
        private void UpdateBeamsFromNativeData(NativeArray<float> nativeBeamRestLengths,
                                              NativeArray<float> nativeBeamLagrange)
        {
            for (int i = 0; i < beams.Count; i++)
            {
                if (!beams[i].IsCrossBody)
                {
                    beams[i].restLength = nativeBeamRestLengths[i];
                    beams[i].lagrangeMultiplier = nativeBeamLagrange[i];
                }
            }
        }

        #endregion


        #region Area Constraint Setup

        private void PrepareNativeTriangleData(NativeArray<TriangleData> nativeTriangles, List<Face> faces)
        {
            Vector3[] restPositions = _trussAsset != null ? _trussAsset.NodePositions : null;
            
            for (int i = 0; i < faces.Count; i++)
            {
                Face f = faces[i];
                
                // Use Face.restArea if already initialized, otherwise calculate from rest positions
                float restArea = f.restArea;
                float originalRestArea = f.originalRestArea;
                
                if (restArea <= 0f && restPositions != null && 
                    f.nodeA < restPositions.Length && f.nodeB < restPositions.Length && f.nodeC < restPositions.Length)
                {
                    Vector3 p0 = restPositions[f.nodeA];
                    Vector3 p1 = restPositions[f.nodeB];
                    Vector3 p2 = restPositions[f.nodeC];
                    restArea = 0.5f * Vector3.Cross(p1 - p0, p2 - p0).magnitude;
                    originalRestArea = restArea;
                }

                nativeTriangles[i] = new TriangleData
                {
                    nodeA = f.nodeA,
                    nodeB = f.nodeB,
                    nodeC = f.nodeC,
                    restArea = restArea,
                    originalRestArea = originalRestArea,
                    compliance = areaCompliance,
                    isCollidable = f.isCollidable,
                    areaPlasticityThreshold = f.areaPlasticityThreshold,
                    areaPlasticityRate = f.areaPlasticityRate,
                    maxAreaDeformation = f.maxAreaDeformation
                };
            }
        }

        private void ExecuteAreaConstraintIterations(int constraintIters, 
                                                    NativeArray<TriangleData> nativeTriangles,
                                                    NativeArray<float> nativeNodeMasses,
                                                    float dt, float worldRestLengthScale)
        {
            if (nativeTriangles.Length == 0) return;
            // Run alongside distance constraints or interleaved? 
            // Running sequentially for now.
            
            int triCount = nativeTriangles.Length;
            int batchSize = math.max(1, triCount / (Unity.Jobs.LowLevel.Unsafe.JobsUtility.JobWorkerCount * 4));

            for (int iter = 0; iter < constraintIters; iter++)
            {
                var areaJob = new SolveAreaConstraintsJob
                {
                    triangles = nativeTriangles,
                    nodeMasses = nativeNodeMasses,
                    isPinned = nodeManager.nativeIsPinned,
                    dt = dt,
                    worldRestLengthScale = worldRestLengthScale,
                    predictedPositions = nodeManager.nativePredictedPositions
                };
                
                areaJob.Schedule(triCount, batchSize).Complete();
            }
        }

        #endregion
        #region Area Plasticity

        // Node-to-triangle mapping for efficient lookup
        private Dictionary<int, List<int>> _nodeToTriangles;

        /// <summary>
        /// Build mapping from node indices to triangle indices that contain them.
        /// Call once after faces are initialized.
        /// </summary>
        private void BuildNodeToTriangleMap()
        {
            _nodeToTriangles = new Dictionary<int, List<int>>();
            
            if (_trussAsset?.Faces == null) return;
            
            for (int i = 0; i < _trussAsset.Faces.Count; i++)
            {
                Face f = _trussAsset.Faces[i];
                AddNodeTriangleMapping(f.nodeA, i);
                AddNodeTriangleMapping(f.nodeB, i);
                AddNodeTriangleMapping(f.nodeC, i);
            }
        }

        private void AddNodeTriangleMapping(int nodeIndex, int triangleIndex)
        {
            if (!_nodeToTriangles.ContainsKey(nodeIndex))
                _nodeToTriangles[nodeIndex] = new List<int>();
            _nodeToTriangles[nodeIndex].Add(triangleIndex);
        }

        /// <summary>
        /// Apply area-based plasticity to triangles whose nodes experienced collisions.
        /// This causes permanent deformation by shifting rest areas toward current areas.
        /// </summary>
        public void ApplyAreaPlasticityPostCollision(float dt, HashSet<int> collidedNodes)
        {
            if (_ownerBody == null || !_ownerBody.enablePlasticity) return;
            if (_trussAsset?.Faces == null) return;
            if (collidedNodes == null || collidedNodes.Count == 0) return;
            
            // Build mapping if not done
            if (_nodeToTriangles == null)
                BuildNodeToTriangleMap();

            float worldAreaScale = _owner.TransformDirection(Vector3.right).magnitude;
            worldAreaScale *= worldAreaScale; // Scale squared for area

            // Collect unique triangle indices affected by collisions
            HashSet<int> affectedTriangles = new HashSet<int>();
            foreach (int nodeIndex in collidedNodes)
            {
                if (_nodeToTriangles.TryGetValue(nodeIndex, out var triangles))
                {
                    foreach (int triIndex in triangles)
                        affectedTriangles.Add(triIndex);
                }
            }

            // Apply plasticity only to affected triangles
            List<Face> faces = _trussAsset.Faces;
            foreach (int triIndex in affectedTriangles)
            {
                if (triIndex < 0 || triIndex >= faces.Count) continue;
                
                Face f = faces[triIndex];
                
                // Skip if no plasticity configured
                if (f.areaPlasticityRate <= 0f) continue;
                if (f.originalRestArea <= 0f) continue;

                // Calculate current world area
                Vector3 p0 = nodeManager.PredictedPositions[f.nodeA];
                Vector3 p1 = nodeManager.PredictedPositions[f.nodeB];
                Vector3 p2 = nodeManager.PredictedPositions[f.nodeC];
                float currentArea = Vector3.Cross(p1 - p0, p2 - p0).magnitude * 0.5f;

                // Calculate area strain
                float targetArea = f.restArea * worldAreaScale;
                if (targetArea <= 0f) continue;
                
                float areaStrain = Mathf.Abs(currentArea - targetArea) / targetArea;

                // Check threshold
                if (areaStrain <= f.areaPlasticityThreshold)
                    continue;

                // Check max deformation limit
                float currentDeformation = Mathf.Abs(f.restArea - f.originalRestArea) / f.originalRestArea;
                if (currentDeformation >= f.maxAreaDeformation)
                    continue;

                // Calculate plastic change
                float excessStrain = areaStrain - f.areaPlasticityThreshold;
                float plasticChange = excessStrain * f.areaPlasticityRate * dt;

                // Move restArea toward currentArea (in local scale)
                float localCurrentArea = currentArea / worldAreaScale;
                float direction = localCurrentArea > f.restArea ? 1f : -1f;
                f.restArea += direction * plasticChange * f.restArea;

                // Clamp to limits
                float minArea = f.originalRestArea * (1f - f.maxAreaDeformation);
                float maxArea = f.originalRestArea * (1f + f.maxAreaDeformation);
                f.restArea = Mathf.Clamp(f.restArea, minArea, maxArea);

                // Write back (Face is a struct, so we need to reassign)
                faces[triIndex] = f;
            }
        }

        #endregion

        #region Legacy Beam Plasticity (DEPRECATED - Use Area Plasticity Instead)

        /// <summary>
        /// DEPRECATED: Use ApplyAreaPlasticityPostCollision instead.
        /// This method modifies beam rest lengths which can conflict with area constraints.
        /// </summary>
        [System.Obsolete("Use ApplyAreaPlasticityPostCollision for area-based plasticity instead.")]
        private void ApplyPlasticityPostIteration(float dt)
        {
            // DEPRECATED: Method body removed as fields no longer exist on Beam class.
            // Functionality replaced by area-based plasticity.
        }

        #endregion

        #region Cross-Body Constraint Solving


        private void SolveDistanceConstraintsCrossBody(float dt)
        {
            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody)
                    continue;

                if (!beam.AreNodesValid())
                    continue;

                // Skip if already solved this frame
                if (beam.lastSolvedFrame == _currentFrameIndex)
                    continue;

                beam.lastSolvedFrame = _currentFrameIndex; // Mark as solved

                SolveCrossBodyBeamConstraint(beam, dt);
            }
        }


        private void SolveCrossBodyBeamConstraint(Beam beam, float dt)
        {
            if (beam.isEdgeSliding)
            {
                SolveEdgeSlidingConstraint(beam, dt);
                return;
            }

            Vector3 posA = beam.GetNodeALocalPosition();
            Vector3 posB = beam.GetNodeBLocalPosition();
            Vector3 delta = posB - posA;
            float currentLength = delta.magnitude;

            if (currentLength < PhysicsConstants.MIN_BEAM_LENGTH)
                return;

            Vector3 gradient = delta / currentLength;

            // Determine target limits in world space
            bool hasMinLimit = beam.minLength > 0f;
            bool hasMaxLimit = !float.IsPositiveInfinity(beam.maxLength) && beam.maxLength > 0f;

            float worldMinLength = hasMinLimit ? beam.bodyA.transform.TransformDirection(
                Vector3.right * beam.minLength).magnitude : 0f;
            float worldMaxLength = hasMaxLimit ? beam.bodyA.transform.TransformDirection(
                Vector3.right * beam.maxLength).magnitude : float.PositiveInfinity;

            float targetLength = currentLength;

            if (hasMinLimit && currentLength < worldMinLength)
            {
                targetLength = worldMinLength;
            }
            else if (hasMaxLimit && currentLength > worldMaxLength)
            {
                targetLength = worldMaxLength;
            }
            else if (!hasMinLimit && !hasMaxLimit)
            {
                targetLength = beam.bodyA.transform.TransformDirection(
                    Vector3.right * beam.restLength).magnitude;
            }

            float constraint = currentLength - targetLength;

            if (Mathf.Abs(constraint) < 1e-6f) return;

            // ============================================================
            // XPBD FIX #1 & #8: DAMPING VIA COMPLIANCE (CROSS-BODY)
            // ============================================================
            float dampedCompliance = beam.compliance + beam.damping * dt;

            bool pinnedA = beam.IsNodeAPinned();
            bool pinnedB = beam.IsNodeBPinned();
            float wA = pinnedA ? 0f : 1f / beam.bodyA.solver.nodeMasses[beam.nodeA];
            float wB = pinnedB ? 0f : 1f / beam.bodyB.solver.nodeMasses[beam.nodeB];
            float wSum = wA + wB;

            if (wSum <= 0f) return;

            // XPBD compliance
            float effectiveCompliance = dampedCompliance / (dt * dt);
            float deltaLambda = -constraint / (wSum + effectiveCompliance);

            // Guard: Non-finite check
            if (!float.IsFinite(deltaLambda))
                return;

            // Inequality constraint clamping (cables only)
            if (hasMinLimit && !hasMaxLimit)
            {
                float newLambda = beam.lagrangeMultiplier + deltaLambda;
                if (newLambda < 0f)
                    deltaLambda = -beam.lagrangeMultiplier;
            }

            beam.lagrangeMultiplier += deltaLambda;

            Vector3 correction = gradient * deltaLambda;

            // Sanity check
            if (correction.magnitude > PhysicsConstants.MAX_ALLOWED_DISPLACEMENT)
            {
                correction = correction.normalized * PhysicsConstants.MAX_ALLOWED_DISPLACEMENT;
            }

            if (!pinnedA)
                beam.bodyA.solver.nodeManager.PredictedPositions[beam.nodeA] -= wA * correction;
            if (!pinnedB)
                beam.bodyB.solver.nodeManager.PredictedPositions[beam.nodeB] += wB * correction;
        }
        private void SolveEdgeSlidingConstraint(Beam beam, float dt)
        {
            int slidingNode = beam.slidingNode;
            int edgeNodeA = beam.edgeNodeA;
            int edgeNodeB = beam.edgeNodeB;

            SoftBody slidingBody = beam.bodyA;
            SoftBody edgeBody = beam.bodyB;

            Vector3 nodePos = slidingBody.solver.nodeManager.PredictedPositions[slidingNode];
            Vector3 edgePosA = edgeBody.solver.nodeManager.PredictedPositions[edgeNodeA];
            Vector3 edgePosB = edgeBody.solver.nodeManager.PredictedPositions[edgeNodeB];

            Vector3 edgeVector = edgePosB - edgePosA;
            float edgeLength = edgeVector.magnitude;

            if (edgeLength < PhysicsConstants.MIN_BEAM_LENGTH)
            {
                if (debugEdgeSliding)
                    Debug.LogWarning($"[EdgeSliding] Edge too short: {edgeLength}");
                return;
            }

            Vector3 edgeDir = edgeVector / edgeLength;

            // Project node onto infinite edge (no clamping)
            Vector3 nodeToEdgeStart = nodePos - edgePosA;
            float t = Vector3.Dot(nodeToEdgeStart, edgeDir);
            t = Mathf.Clamp(t, 0f, edgeLength);

            Vector3 closestPoint = edgePosA + edgeDir * t;
            Vector3 perpVector = nodePos - closestPoint;
            float perpDistance = perpVector.magnitude;

            if (perpDistance < PhysicsConstants.MIN_BEAM_LENGTH)
            {
                if (debugEdgeSliding)
                    return;
            }

            Vector3 perpDir = perpVector / perpDistance;

            float targetPerpDistance = beam.targetPerpDistance;
            float constraint = perpDistance - targetPerpDistance;

            // ============================================================
            // XPBD FIX #1: DAMPING VIA COMPLIANCE
            // ============================================================
            float dampedCompliance = beam.compliance + beam.damping * dt;

            // ============================================================
            // XPBD FIX #4: CORRECT MASS WEIGHTING
            // ============================================================
            // For constraint C where:
            //   ∇C_node = perpDir (normalized)
            //   ∇C_edgeA = -perpDir * weightA
            //   ∇C_edgeB = -perpDir * weightB
            //
            // Therefore:
            //   ∑ wᵢ|∇Cᵢ|² = (1/m_node)·1 + (1/m_edgeA)·weightA² + (1/m_edgeB)·weightB²

            bool pinnedNode = slidingBody.solver.nodeManager.IsPinned[slidingNode];
            bool pinnedEdgeA = edgeBody.solver.nodeManager.IsPinned[edgeNodeA];
            bool pinnedEdgeB = edgeBody.solver.nodeManager.IsPinned[edgeNodeB];

            if (pinnedNode && pinnedEdgeA && pinnedEdgeB)
            {
                if (debugEdgeSliding)
                    Debug.LogWarning("[EdgeSliding] All nodes pinned!");
                return;
            }

            float massNode = slidingBody.solver.nodeMasses[slidingNode];
            float massEdgeA = edgeBody.solver.nodeMasses[edgeNodeA];
            float massEdgeB = edgeBody.solver.nodeMasses[edgeNodeB];

            // Weight distribution (can extend beyond segment)
            float weightA = 1.0f - (t / edgeLength);
            float weightB = t / edgeLength;

            // CORRECT FORMULA:
            float wNode = pinnedNode ? 0f : 1f / massNode;
            float wEdgeA = pinnedEdgeA ? 0f : (1f / massEdgeA) * weightA * weightA;
            float wEdgeB = pinnedEdgeB ? 0f : (1f / massEdgeB) * weightB * weightB;
            float wSum = wNode + wEdgeA + wEdgeB;

            if (wSum <= 0f)
            {
                if (debugEdgeSliding)
                    Debug.LogWarning("[EdgeSliding] Zero mass sum!");
                return;
            }

            // XPBD compliance
            float effectiveCompliance = dampedCompliance / (dt * dt);

            float deltaLambda = -constraint / (wSum + effectiveCompliance);

            // Guard: Non-finite check
            if (!float.IsFinite(deltaLambda))
                return;

            beam.lagrangeMultiplier += deltaLambda;

            // Limit correction for stability
            float maxCorrection = Mathf.Max(0.1f, perpDistance * 0.5f);
            float correctionMagnitude = Mathf.Abs(deltaLambda);
            if (correctionMagnitude > maxCorrection)
            {
                deltaLambda = Mathf.Sign(deltaLambda) * maxCorrection;
            }

            // Apply corrections using proper gradients
            if (!pinnedNode)
            {
                Vector3 nodeCorrection = (1f / massNode) * deltaLambda * perpDir;
                slidingBody.solver.nodeManager.PredictedPositions[slidingNode] += nodeCorrection;
            }

            if (!pinnedEdgeA)
            {
                Vector3 edgeACorrection = (1f / massEdgeA) * deltaLambda * (-perpDir * weightA);
                edgeBody.solver.nodeManager.PredictedPositions[edgeNodeA] += edgeACorrection;
            }

            if (!pinnedEdgeB)
            {
                Vector3 edgeBCorrection = (1f / massEdgeB) * deltaLambda * (-perpDir * weightB);
                edgeBody.solver.nodeManager.PredictedPositions[edgeNodeB] += edgeBCorrection;
            }
        }
        private void CheckAndBreakConstraints()
        {
            float dt = Time.fixedDeltaTime / GetSimulationSubSteps();
            float dtSquared = dt * dt;

            foreach (var beam in beams)
            {
                if (!beam.isActive || !beam.IsCrossBody || beam.isBroken)
                    continue;

                float forceMagnitude = Mathf.Abs(beam.lagrangeMultiplier) / dtSquared;

                if (beam.ShouldBreak(forceMagnitude, beam.strength))
                {
                    beam.isBroken = true;
                    beam.isActive = false;

                    Debug.Log($"[Breakage] Constraint broken! Force: {forceMagnitude:F2}N exceeded strength: {beam.strength:F2}N");
                }
            }
        }

        private void VisualizeCrossBodyForce(Vector3 posA, Vector3 posB,
                                            Vector3 correction, float constraint, float dt)
        {
            Vector3 mid = (posA + posB) * 0.5f;
            Color forceCol = constraint > 0f ? Color.red : Color.cyan;
            Debug.DrawLine(mid, mid + correction * forceVisualizationScale * 100f,
                          forceCol, dt);
        }

        #endregion

        #region Position Update

        private void UpdateNodePositions(int nodeCount)
        {
            for (int i = 0; i < nodeCount; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i])
                    continue;

                ClampAndApplyNodeMotion(i);
            }
        }


        private void ClampAndApplyNodeMotion(int nodeIndex)
        {
            Vector3 currentWorldPos = _owner.TransformPoint(
                nodeManager.Nodes[nodeIndex].localPosition);
            Vector3 predictedWorldPos = nodeManager.PredictedPositions[nodeIndex];

            Vector3 worldMotion = predictedWorldPos - currentWorldPos;

            if (worldMotion.magnitude > PhysicsConstants.MAX_ALLOWED_DISPLACEMENT)
            {
                predictedWorldPos = currentWorldPos +
                    worldMotion.normalized * PhysicsConstants.MAX_ALLOWED_DISPLACEMENT;
                nodeManager.PredictedPositions[nodeIndex] = predictedWorldPos;
            }

            Vector3 finalLocalPos = _owner.InverseTransformPoint(predictedWorldPos);
            nodeManager.Nodes[nodeIndex].localPosition = finalLocalPos;
            nodeManager.PreviousPositions[nodeIndex] = currentWorldPos;
        }

        #endregion

        #region Pressure Forces


        public Vector3 CalculateVolumeCenter()
        {
            if (nodeManager?.Nodes == null || nodeManager.Nodes.Count == 0)
                return Vector3.zero;

            Vector3 center = Vector3.zero;
            int validNodes = 0;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] != null)
                {
                    center += _owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                    validNodes++;
                }
            }

            if (validNodes > 0)
                center /= validNodes;

            _cachedVolumeCenter = center;
            return center;
        }


        private void ApplyPressureForces(float dt)
        {
            if (internalPressure <= 0f || nodeManager.Nodes.Count == 0)
                return;

            Vector3 volumeCenter = CalculateVolumeCenter();

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null || nodeManager.IsPinned[i])
                    continue;

                ApplyPressureForceToNode(i, volumeCenter, dt);
            }
        }


        private void ApplyPressureForceToNode(int nodeIndex, Vector3 volumeCenter, float dt)
        {
            Vector3 nodeWorldPos = _owner.TransformPoint(
                nodeManager.Nodes[nodeIndex].localPosition);

            Vector3 direction = nodeWorldPos - volumeCenter;
            float distance = direction.magnitude;

            if (distance < PhysicsConstants.MIN_CENTER_DISTANCE)
                return;

            direction /= distance;

            float pressureForce = internalPressure * PhysicsConstants.PRESSURE_SCALE_FACTOR;
            float distanceFactor = 1f / (distance + PhysicsConstants.PRESSURE_DISTANCE_OFFSET);
            Vector3 force = direction * pressureForce * distanceFactor;

            ApplyWorldForceToNode(nodeIndex, force, null, dt);
        }

        #endregion

        #region Force Application

        public void ApplyForceToNode(int nodeIndex, Vector3 localForce)
        {
            if (!IsValidNodeForForce(nodeIndex))
                return;

            float mass = nodeMasses[nodeIndex];
            if (mass <= 0f)
                return;

            float simScale = GetSimulationTimeScale();
            float fullDt = Time.fixedDeltaTime * simScale;
            int subSteps = GetSimulationSubSteps();
            float effectiveDt = fullDt / Mathf.Max(1, subSteps);

            Vector3 worldForce = _owner.TransformDirection(localForce);
            ApplyWorldForceToNode(nodeIndex, worldForce, mass, effectiveDt);
        }


        public void ApplyWorldForceToNode(int nodeIndex, Vector3 worldForce,
                                         float? customMass = null, float? customDt = null)
        {
            if (!IsValidNodeForForce(nodeIndex))
                return;

            float mass = customMass ?? nodeMasses[nodeIndex];
            if (mass <= 0f)
                return;

            float dt = customDt ?? Time.fixedDeltaTime;
            Vector3 accel = worldForce / mass;

            Vector3 currentWorldPos = _owner.TransformPoint(
                nodeManager.Nodes[nodeIndex].localPosition);
            Vector3 currentVelocity = (currentWorldPos -
                nodeManager.PreviousPositions[nodeIndex]) / dt;
            Vector3 newVelocity = currentVelocity + accel * dt;

            nodeManager.PreviousPositions[nodeIndex] = currentWorldPos - newVelocity * dt;
        }


        private bool IsValidNodeForForce(int nodeIndex)
        {
            return nodeIndex >= 0 &&
                   nodeIndex < nodeMasses.Count &&
                   !nodeManager.IsPinned[nodeIndex];
        }


        public void ApplyImpulse(int nodeIndex, Vector3 targetPosition, float strength)
        {
            if (nodeIndex < 0 || nodeIndex >= nodeManager.Nodes.Count ||
                nodeMasses.Count <= nodeIndex ||
                nodeManager.Nodes[nodeIndex] == null ||
                nodeManager.IsPinned[nodeIndex])
                return;

            float simScale = GetSimulationTimeScale();
            float effectiveDt = Time.fixedDeltaTime * simScale;

            Vector3 currentPos = nodeManager.Nodes[nodeIndex].localPosition;
            Vector3 targetLocalPos = _owner.InverseTransformPoint(targetPosition);
            Vector3 delta = targetLocalPos - currentPos;
            Vector3 impulse = delta * strength * nodeMasses[nodeIndex] / effectiveDt;

            ApplyForceToNode(nodeIndex, impulse);
        }

        #endregion

        #region Node Search


        public int FindClosestNodeToRay(Ray ray, float maxDistance)
        {
            int closestNode = -1;
            float minDistance = float.MaxValue;

            for (int i = 0; i < nodeManager.Nodes.Count; i++)
            {
                if (nodeManager.Nodes[i] == null)
                    continue;

                Vector3 nodePos = _owner.TransformPoint(nodeManager.Nodes[i].localPosition);
                float distance = Vector3.Cross(ray.direction, nodePos - ray.origin).magnitude;

                if (distance < nodeManager.GetNodeRadius(i) && distance < minDistance)
                {
                    float t = Vector3.Dot(nodePos - ray.origin, ray.direction);
                    if (t > 0 && t < maxDistance)
                    {
                        minDistance = distance;
                        closestNode = i;
                    }
                }
            }

            return closestNode;
        }

        #endregion

        #region Mesh Deformation


        public void DeformMesh(Transform transform)
        {
            if (transform == null)
                return;

            UpdateNodeRotations();
            meshDeformer.Deform(transform, nodeManager.Nodes,
                              nodeManager.InitialPositions,
                              nodeManager.InitialRotations);
        }

        #endregion

        #region Node Rotation


        private void UpdateNodeRotations()
        {
            int nodeCount = nodeManager.Nodes.Count;
            List<List<int>> neighbors = BuildNeighborLists(nodeCount);

            for (int i = 0; i < nodeCount; i++)
            {
                UpdateSingleNodeRotation(i, neighbors[i]);
            }
        }


        private List<List<int>> BuildNeighborLists(int nodeCount)
        {
            List<List<int>> neighbors = new List<List<int>>(nodeCount);

            for (int i = 0; i < nodeCount; i++)
            {
                neighbors.Add(new List<int>());
            }

            foreach (var beam in beams)
            {
                if (beam.IsCrossBody) continue;
                if (beam.nodeA < 0 || beam.nodeA >= nodeCount ||
                    beam.nodeB < 0 || beam.nodeB >= nodeCount)
                    continue;

                neighbors[beam.nodeA].Add(beam.nodeB);
                neighbors[beam.nodeB].Add(beam.nodeA);
            }

            return neighbors;
        }

        private void UpdateSingleNodeRotation(int nodeIndex, List<int> neighborIndices)
        {
            List<int> cluster = new List<int>(neighborIndices) { nodeIndex };

            if (cluster.Count < 2)
                return;

            CalculateCentersOfMass(cluster, out Vector3 comRest,
                                  out Vector3 comCurrent, out float totalMass);

            if (totalMass < PhysicsConstants.MIN_TOTAL_MASS)
                return;

            comRest /= totalMass;
            comCurrent /= totalMass;

            Matrix4x4 covarianceMatrix = CalculateCovarianceMatrix(cluster, comRest,
                                                                   comCurrent);

            Quaternion rotation = ExtractRotationFromCovariance(nodeIndex, covarianceMatrix);

            nodeManager.Nodes[nodeIndex].localRotation = rotation;
        }
        private void CalculateCentersOfMass(List<int> cluster, out Vector3 comRest,
                                           out Vector3 comCurrent, out float totalMass)
        {
            comRest = Vector3.zero;
            comCurrent = Vector3.zero;
            totalMass = 0f;

            foreach (int idx in cluster)
            {
                float m = nodeMasses[idx];
                comRest += m * nodeManager.InitialPositions[idx];
                comCurrent += m * nodeManager.Nodes[idx].localPosition;
                totalMass += m;
            }
        }
        private Matrix4x4 CalculateCovarianceMatrix(List<int> cluster, Vector3 comRest,
                                                   Vector3 comCurrent)
        {
            Matrix4x4 A = Matrix4x4.zero;

            foreach (int idx in cluster)
            {
                Vector3 pRest = nodeManager.InitialPositions[idx] - comRest;
                Vector3 pCurrent = nodeManager.Nodes[idx].localPosition - comCurrent;
                float m = nodeMasses[idx];

                A[0, 0] += m * pCurrent.x * pRest.x;
                A[0, 1] += m * pCurrent.x * pRest.y;
                A[0, 2] += m * pCurrent.x * pRest.z;
                A[1, 0] += m * pCurrent.y * pRest.x;
                A[1, 1] += m * pCurrent.y * pRest.y;
                A[1, 2] += m * pCurrent.y * pRest.z;
                A[2, 0] += m * pCurrent.z * pRest.x;
                A[2, 1] += m * pCurrent.z * pRest.y;
                A[2, 2] += m * pCurrent.z * pRest.z;
            }

            return A;
        }

        private Quaternion ExtractRotationFromCovariance(int nodeIndex, Matrix4x4 A)
        {
            Quaternion q = nodeManager.Nodes[nodeIndex].localRotation;
            Matrix4x4 R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);

            for (int iter = 0; iter < PhysicsConstants.MAX_ROTATION_ITERATIONS; iter++)
            {
                Vector3 tau = CalculateRotationAxisAngle(R, A);
                float denom = Mathf.Abs(CalculateRotationDenominator(R, A)) +
                             PhysicsConstants.ROTATION_CONVERGENCE_EPSILON;
                Vector3 omega = tau / denom;
                float w = omega.magnitude;

                if (w < PhysicsConstants.MIN_OMEGA_MAGNITUDE)
                    break;

                Vector3 axis = omega / w;
                Quaternion deltaQ = Quaternion.AngleAxis(w * Mathf.Rad2Deg, axis);
                q = deltaQ * q;
                q = q.normalized;
                R = Matrix4x4.TRS(Vector3.zero, q, Vector3.one);
            }

            return q;
        }
        private Vector3 CalculateRotationAxisAngle(Matrix4x4 R, Matrix4x4 A)
        {
            Vector3 r0 = R.GetColumn(0);
            Vector3 r1 = R.GetColumn(1);
            Vector3 r2 = R.GetColumn(2);
            Vector3 a0 = A.GetColumn(0);
            Vector3 a1 = A.GetColumn(1);
            Vector3 a2 = A.GetColumn(2);

            return Vector3.Cross(r0, a0) + Vector3.Cross(r1, a1) + Vector3.Cross(r2, a2);
        }
        private float CalculateRotationDenominator(Matrix4x4 R, Matrix4x4 A)
        {
            Vector3 r0 = R.GetColumn(0);
            Vector3 r1 = R.GetColumn(1);
            Vector3 r2 = R.GetColumn(2);
            Vector3 a0 = A.GetColumn(0);
            Vector3 a1 = A.GetColumn(1);
            Vector3 a2 = A.GetColumn(2);

            return Vector3.Dot(r0, a0) + Vector3.Dot(r1, a1) + Vector3.Dot(r2, a2);
        }

        #endregion

        #region Node Generation
        public void GenerateNodesAndBeams(Vector3[] positions, float connectionDistance = -1f,
                                         Beam[] beamsArray = null, Transform parent = null)
        {
            if (!ValidateGenerationParameters(positions, connectionDistance, beamsArray, parent))
                return;

            if (s_isApplicationQuitting || parent.gameObject == null)
                return;

            ClearExistingNodes(parent);
            GenerateNodes(positions, parent);

            if (beamsArray != null)
                GenerateBeamsFromArray(beamsArray, positions);
            else
                GenerateBeamsFromDistance(positions, connectionDistance);

            meshDeformer.MapVerticesToNodes(parent, nodeManager.Nodes,
                                          nodeManager.InitialPositions);
        }
        private bool ValidateGenerationParameters(Vector3[] positions, float connectionDistance,
                                                 Beam[] beamsArray, Transform parent)
        {
            if (positions == null || positions.Length == 0 ||
                (connectionDistance <= 0f && beamsArray == null))
            {
                Debug.LogWarning("[Solver] Cannot generate nodes and beams: Invalid input.");
                return false;
            }

            if (parent == null)
            {
                Debug.LogWarning("[Solver] GenerateNodesAndBeams: parent Transform is null.");
                return false;
            }

            return true;
        }
        private void ClearExistingNodes(Transform parent)
        {
            nodeManager.Clear();
            beams.Clear();
            nodeMasses.Clear();
            DestroyOldNodes(parent);
        }
        private void GenerateNodes(Vector3[] positions, Transform parent)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = parent;
                nodeObj.transform.localPosition = positions[i];
                nodeObj.transform.localRotation = Quaternion.identity;
                nodeObj.AddComponent<SphereCollider>().radius = nodeManager.GetNodeRadius(i);
                nodeObj.hideFlags = HideFlags.HideAndDontSave;


                nodeManager.AddNode(nodeObj.transform, positions[i]);

                float mass = (_trussAsset != null && i < _trussAsset.NodeMasses.Count)
                    ? _trussAsset.NodeMasses[i]
                    : PhysicsConstants.DEFAULT_NODE_MASS;
                nodeMasses.Add(mass);
            }
        }
        private void GenerateBeamsFromArray(Beam[] beamsArray, Vector3[] positions)
        {
            foreach (var b in beamsArray)
            {
                if (IsValidBeam(b, positions))
                {
                    beams.Add(new Beam(
                        nodeA: b.nodeA,
                        nodeB: b.nodeB,
                        compliance: b.compliance,
                        damping: b.damping,
                        restLength: b.restLength
                    ));
                }
            }
        }
        private bool IsValidBeam(Beam beam, Vector3[] positions)
        {
            return beam.nodeA >= 0 && beam.nodeA < positions.Length &&
                   beam.nodeB >= 0 && beam.nodeB < positions.Length &&
                   beam.restLength > PhysicsConstants.MIN_BEAM_LENGTH;
        }
        private void GenerateBeamsFromDistance(Vector3[] positions, float connectionDistance)
        {
            for (int i = 0; i < positions.Length; i++)
            {
                for (int j = i + 1; j < positions.Length; j++)
                {
                    float distance = Vector3.Distance(positions[i], positions[j]);

                    if (distance <= connectionDistance &&
                        distance > PhysicsConstants.MIN_BEAM_LENGTH)
                    {
                        beams.Add(new Beam(i, j, compliance, defaultDamping, distance));
                    }
                }
            }
        }
        public void GenerateCubeTest(Transform transform)
        {
            if (transform == null || meshDeformer.Mesh == null)
                return;

            Bounds bounds = meshDeformer.Mesh.bounds;
            Vector3 min = bounds.min;
            Vector3 max = bounds.max;

            Vector3[] cubeNodes = new Vector3[]
            {
                new Vector3(min.x, min.y, min.z),
                new Vector3(max.x, min.y, min.z),
                new Vector3(min.x, max.y, min.z),
                new Vector3(max.x, max.y, min.z),
                new Vector3(min.x, min.y, max.z),
                new Vector3(max.x, min.y, max.z),
                new Vector3(min.x, max.y, max.z),
                new Vector3(max.x, max.y, max.z)
            };

            float connectionDistance = Vector3.Distance(min, max) *
                PhysicsConstants.CUBE_CONNECTION_DISTANCE_MULTIPLIER;

            GenerateNodesAndBeams(cubeNodes, connectionDistance, parent: transform);
        }

        #endregion

        #region Node Restoration

        public void RestoreNodesAndBeams(Transform transform)
        {
            if (transform == null)
                return;

            var currentBeams = new List<Beam>(beams);
            var currentInitialPositions = new List<Vector3>(nodeManager.InitialPositions);

            nodeManager.Clear();
            beams.Clear();

            RestoreNodes(currentInitialPositions, transform);
            RestoreBeams(currentBeams);

            Debug.Log($"[Solver] Restored {nodeManager.Nodes.Count} nodes, {beams.Count} beams");
        }
        private void RestoreNodes(List<Vector3> initialPositions, Transform transform)
        {
            for (int i = 0; i < initialPositions.Count; i++)
            {
                GameObject nodeObj = new GameObject($"Node_{i}");
                nodeObj.transform.parent = transform;
                nodeObj.transform.localPosition = initialPositions[i];
                nodeObj.hideFlags = HideFlags.HideAndDontSave;

                nodeManager.AddNode(nodeObj.transform, initialPositions[i]);
            }
        }
        private void RestoreBeams(List<Beam> savedBeams)
        {
            foreach (var beam in savedBeams)
            {
                if (IsRestoredBeamValid(beam))
                {
                    Vector3 localPosA = nodeManager.Nodes[beam.nodeA].localPosition;
                    Vector3 localPosB = nodeManager.Nodes[beam.nodeB].localPosition;
                    float distance = Vector3.Distance(localPosA, localPosB);

                    beams.Add(new Beam(beam.nodeA, beam.nodeB, beam.compliance,
                                     beam.damping, distance));
                }
            }
        }
        private bool IsRestoredBeamValid(Beam beam)
        {
            return beam.nodeA < nodeManager.Nodes.Count &&
                   beam.nodeB < nodeManager.Nodes.Count &&
                   nodeManager.Nodes[beam.nodeA] != null &&
                   nodeManager.Nodes[beam.nodeB] != null &&
                   beam.restLength > PhysicsConstants.MIN_BEAM_LENGTH;
        }

        #endregion

        #region Cleanup
        private void DestroyOldNodes(Transform parent)
        {
            _reusableDestroyList.Clear();

            foreach (Transform child in parent)
            {
                if (child.name.StartsWith("Node_"))
                    _reusableDestroyList.Add(child.gameObject);
            }

            foreach (var go in _reusableDestroyList)
            {
                if (Application.isEditor && !Application.isPlaying)
                    Object.DestroyImmediate(go);
                else
                    Object.Destroy(go);
            }
        }
        ~Solver()
        {
            Cleanup();
        }
        public void Cleanup()
        {
            nodeManager?.DisposeNativeArrays();

            if (_reusableWorldPositions.IsCreated)
                _reusableWorldPositions.Dispose();
        }

        #endregion
    }
}
