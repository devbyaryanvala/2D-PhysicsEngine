# 2D Physics Engine (ep)

A modular, C-based 2D physics engine designed for high precision, scientific accuracy, and clean software architecture. This engine has been fully upgraded to support arbitrary convex polygons, advanced joint constraints, continuous collision detection, and performance-oriented broad-phase structures. It uses **SI units (MKS)**.

---

## 🔬 Core Architecture & Features

### 1. Integration & Dynamics
The engine utilizes **Semi-Implicit Euler** integration.
- Supports complete linear and angular dynamics (Torque, Inertia).
- Mass and Inertia tensors are correctly computed for arbitrary convex polygons and circles based on material density.
- Configurable Material properties (Restitution, Static/Dynamic Friction).

### 2. Advanced Collision Pipeline
- **Broad-Phase (BVH)**: Dynamic Bounding Volume Hierarchy tree for O(log n) pair detection, drastically improving performance with large numbers of bodies.
- **Narrow-Phase (GJK & EPA)**: Full implementation of the Gilbert-Johnson-Keerthi (GJK) algorithm for Boolean intersection, backed by the Expanding Polytope Algorithm (EPA) to extract exact penetration vectors and contact manifolds.
- **Supported Shapes**: True Circles and Arbitrary Convex Polygons.

### 3. Constraints & Solver
- **Island Solver**: A Depth-First Search (DFS) graph builder automatically groups resting bodies into isolated "islands".
- **Sleeping**: Islands that come to rest are put to sleep, bypassing integration and collision to save CPU cycles. Dynamic waking occurs upon interaction.
- **Sequential Impulse Solver**: Solves friction and restitution across contact manifolds using accumulated impulses and warm-starting.
- **Joint Constraints (Revolute/Pin)**: Fully featured hinge joints with configurable:
  - **Motors**: Drive joints at specific speeds with max torque.
  - **Limits**: Baumgarte-stabilized hard angular limits to restrict range of motion.

### 4. Continuous Collision Detection (CCD)
- Fast-moving bodies (like projectiles) can be tagged with an `isBullet` flag.
- The engine runs a specialized sub-stepping CCD pass to prevent these objects from "tunneling" (ghosting) through thin static geometry.

---

## 📂 Project Structure

```text
2D Physics Engine/
├── include/ep/       # Public Headers (phys namespace)
│   ├── config.h        # Constants
│   ├── vec2.h          # Vector math
│   ├── body.h          # Physical state, Geometry
│   ├── collision.h     # GJK/EPA Narrowphase
│   ├── broadphase.h    # BVH Tree
│   ├── solver.h        # Sequential Impulse Solver
│   ├── joint.h         # Constraint logic
│   ├── island.h        # Island graph & sleeping
│   ├── render.h        # Dependency-Free Bitmap Font Engine
│   └── world.h         # High-level World API
├── src/                # Implementation Files (Core Engine)
│   └── ...             # Core C source files
├── demos/              # Example Applications
│   ├── demo0_blocks.c
│   ├── demo1_pendulum.c
│   ├── demo2_chain.c
│   ├── demo3_stack_sleeping.c
│   ├── demo4_polygons.c
│   ├── demo5_motors_limits.c
│   └── demo6_ccd.c
├── build/              # Build Artifacts
│   ├── obj/            # Compiled Engine Objects (.o)
│   └── *.exe           # Compiled Demo Executables
└── Makefile            # Build script
```

---

## 🛠️ Build and Installation

### Prerequisites
* **C99 Compiler**: (GCC/MinGW)
* **SDL2 Library**: Default path is `C:/SDL2/i686-w64-mingw32`.

### Build Instructions
The engine is structured to compile into independent object files, which are then linked to the demos.
1. Open your terminal in the project root.
2. Run the master build command to compile the engine and all 7 executables:
   ```bash
   make
   ```
3. Run a specific demo by using the `make run` target and specifying the demo name:
   ```bash
   make run DEMO=demo6_ccd
   ```

---

## 🎮 Included Demos

The project includes 7 isolated demo applications in the `demos/` directory. Each demo features a **Real-Time OSD (On-Screen Display)** tracking physics stats (Velocity, Force, etc.) rendered natively without external font dependencies.

* **`demo0_blocks`**: Basic AABB vs. Ground interactions.
* **`demo1_pendulum`**: A simple revolute joint swinging under gravity.
* **`demo2_chain`**: A multi-link chain of rigid bodies.
* **`demo3_stack_sleeping`**: A tall stack of boxes that optimally fall asleep. Press SPACE to shoot a projectile and wake them up!
* **`demo4_polygons`**: Complex shapes (triangles, hexagons, custom polygons) bouncing on a sloped surface.
* **`demo5_motors_limits`**: A motorized, drivable car (WASD/Arrows) with circular wheels, and a limited-angle pendulum.
* **`demo6_ccd`**: Compares a standard fast projectile against a CCD-enabled bullet hitting a thin wall.
