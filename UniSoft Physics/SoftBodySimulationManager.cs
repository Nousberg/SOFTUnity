using UnityEngine;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

/// <summary>
/// Global manager for soft body simulation.
/// Handles: batched job scheduling, generation semaphore for Task.Run limiting.
/// </summary>
public class SoftBodySimulationManager : MonoBehaviour
{
    public static SoftBodySimulationManager Instance { get; private set; }
    
    /// <summary>
    /// Limits concurrent background mesh generation (Task.Run).
    /// Default = 1 means only one body generates at a time.
    /// </summary>
    public static SemaphoreSlim GenerationSemaphore { get; } = new SemaphoreSlim(1, 1);
    
    /// <summary>
    /// All registered soft bodies.
    /// </summary>
    private List<AdvancedVolumetricSoftBody> bodies = new List<AdvancedVolumetricSoftBody>();
    
    /// <summary>
    /// Global solver iterations for all soft bodies.
    /// Higher = stiffer and more stable, but more expensive.
    /// </summary>
    [Range(1, 50)]
    public int globalSolverIterations = 10;
    
    private void Awake()
    {
        if (Instance != null && Instance != this)
        {
            Destroy(gameObject);
            return;
        }
        Instance = this;
        // removed DontDestroyOnLoad to restrict to current scene
    }
    
    private void OnDestroy()
    {
        if (Instance == this)
            Instance = null;
    }

    /// <summary>
    /// Register a soft body for managed simulation.
    /// </summary>
    public void Register(AdvancedVolumetricSoftBody body)
    {
        if (!bodies.Contains(body))
            bodies.Add(body);
    }
    
    /// <summary>
    /// Unregister a soft body.
    /// </summary>
    public void Unregister(AdvancedVolumetricSoftBody body)
    {
        bodies.Remove(body);
    }
    
    /// <summary>
    /// Acquire generation semaphore before Task.Run.
    /// Use: await SoftBodySimulationManager.AcquireGenerationSlot();
    /// </summary>
    public static async Task AcquireGenerationSlot()
    {
        await GenerationSemaphore.WaitAsync();
    }
    
    /// <summary>
    /// Release generation semaphore after Task.Run completes.
    /// </summary>
    public static void ReleaseGenerationSlot()
    {
        GenerationSemaphore.Release();
    }
    
    /// <summary>
    /// Auto-create manager if needed. Called by SoftBodies.
    /// </summary>
    public static void EnsureInstance()
    {
        if (Instance == null)
        {
            // Check if one already exists in scene but not yet assigned
            Instance = FindObjectOfType<SoftBodySimulationManager>();
            
            if (Instance == null)
            {
                var go = new GameObject("SoftBodySimulationManager");
                Instance = go.AddComponent<SoftBodySimulationManager>();
            }
        }
    }
}
