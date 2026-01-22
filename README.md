# SOFTUnity

**An AI-generated open source soft-body physics simulation for Unity**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Unity](https://img.shields.io/badge/Unity-2021.3+-black.svg)](https://unity.com/)

Fast and painless 3D soft body integration in Unity Engine.

---

## Features

- **Position Based Dynamics (PBD)** — Stable real-time soft body simulation
- **Verlet Integration** — Velocity-free physics with natural damping
- **Bowyer-Watson Tetrahedralization** — Automatic internal mesh generation
- **Visual Mesh Deformation** — Real-time mesh updates from physics
- **Constraint System** — Connect soft bodies together or anchor to world
- **ScriptableObject Storage** — Edit physics mesh in editor, preserve at runtime
- **Scene Editor Tools** — Visual node/beam manipulation with handles

## Installation

1. Clone or download this repository
2. Copy the `SoftBody` folder into your Unity project's `Assets/Scripts/`
3. Done! No external dependencies required

## Quick Start

```csharp
// 1. Add component to any GameObject with MeshFilter
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class MySoftBody : MonoBehaviour
{
    public AdvancedVolumetricSoftBody softBody;
}

// 2. Create SoftBodyTruss asset: Assets > Create > Physics > Soft Body Truss
// 3. Assign truss to component
// 4. Click "Generate from Mesh" in inspector
// 5. Hit Play!
```

## Inspector Tabs

| Tab | Description |
|-----|-------------|
| **Truss** | Statistics, generation settings, danger zone (clear buttons) |
| **Nodes** | Edit selected nodes: mass, pin state |
| **Beams** | Edit selected beams: rest length, stiffness |
| **Sets** | Create named node groups for constraints |

## Constraints

Connect multiple soft bodies or anchor nodes to world space:

```csharp
// Add SoftBodyConstraint component
// Set Base Body (target soft body, or null for world anchor)
// Create Links between Node Sets
```

**Constraint Types:**
- `Node -> Node` — Distance constraint between bodies
- `Node -> World` — Pin nodes to initial world position

## Physics Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `distanceStiffness` | Beam constraint strength | 0.8 |
| `volumeStiffness` | Volume preservation | 0.5 |
| `damping` | Velocity damping | 0.99 |
| `solverIterations` | Constraint solver passes | 10 |
| `gravity` | World gravity | (0, -9.81, 0) |

## Advanced Features

### Plasticity

Permanent deformation when stress exceeds threshold:

```csharp
plasticityThreshold = 0.1f;  // 10% stretch before plastic deformation
plasticityRate = 0.5f;       // How fast rest length adapts
maxDeformation = 2.0f;       // Maximum stretch multiplier
```

### Collision

Built-in sphere-based collision with Unity colliders:

```csharp
enableCollision = true;
collisionRadius = 0.05f;
friction = 0.3f;
```

## Project Structure

```
SoftBody/
├── AdvancedVolumetricSoftBody.cs   # Main physics component
├── SoftBodyTruss.cs                # ScriptableObject data storage
├── SoftBodyConstraint.cs           # Multi-body connections
└── Editor/
    ├── AdvancedVolumetricSoftBodyEditor.cs  # Scene tools
    └── SoftBodyConstraintEditor.cs          # Constraint UI
```

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

**Made with AI assistance**
