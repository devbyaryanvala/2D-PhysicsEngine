# Modular 2D Physics Engine

A modular, C-based 2D physics engine designed for high precision, scientific accuracy, and clean software architecture. This engine moves away from arbitrary pixel-based movement to a robust system grounded in real-world physical principles and **SI units (MKS)**.

---

## 🔬 Core Physics Concepts

### 1. Semi-Implicit Euler Integration
The engine utilizes **Semi-Implicit Euler** (also known as symplectic Euler) to calculate the motion of bodies. Unlike standard Euler, this method updates velocity *before* position, providing significantly better energy conservation for stable simulations.

**The Equations of Motion:**
1.  **Acceleration**: Derived from Newton's Second Law ($F=ma$):

    `aₙ = Fₙ × invMass`
3.  **Velocity Update**: 

    `vₙ₊₁ = vₙ + aₙ × Δt`
5.  **Position Update**:

    `pₙ₊₁ = pₙ + vₙ₊₁ × Δt`



### 2. Linear and Rotational Dynamics
Every physical object (`Body`) tracks both linear and angular states:
* **Mass & Inertia**: Mass is calculated based on area and density ($\rho$). The **Moment of Inertia** ($I$) for rectangular bodies is calculated as:
    $$I = \frac{1}{12} \cdot m \cdot (w^2 + h^2)$$
* **Torque**: Rotational equivalent of force, used to change angular velocity.

### 3. Collision System (AABB)
The engine currently implements **Axis-Aligned Bounding Box** detection. 

**Collision Condition:**
Two bodies, $A$ and $B$, are colliding if:
* $A_{minX} < B_{maxX}$ AND $A_{maxX} > B_{minX}$
* $A_{minY} < B_{maxY}$ AND $A_{maxY} > B_{minY}$

### 4. Collision Resolution
When a collision is detected, the engine performs two steps:
1.  **Positional Correction**: Moving bodies out of overlap to prevent "sinking."
2.  **Impulse Reflection**: Reversing velocity based on the **Coefficient of Restitution** ($e$), which determines bounciness.
    $$\vec{v}_{after} = \vec{v}_{before} \cdot -e$$

---

## 📂 Project Structure

The project follows a modular design to separate concerns and ensure maintainability:

```text
2D Physics Engine/
├── include/ep/       # Public Headers (phys namespace)
│   ├── config.h        # SI Units, Gravity, and World Constants
│   ├── vec2.h          # Vector math (Dot products, Normalization)
│   ├── body.h          # Physical state and Mass/Inertia logic
│   └── collision.h     # AABB and World Boundary logic
├── src/                # Implementation Files
│   ├── vec2.c          # Linear algebra implementation
│   ├── body.c          # Physics solver (Integration)
│   ├── collision.c     # Narrowphase collision logic
│   └── engine.c        # Main App, SDL2 Rendering, and Input
└── Makefile            # Automated build script
```

---

## 🛠️ Build and Installation

### Prerequisites
* **C99 Compiler**: (GCC/MinGW)
* **SDL2 Library**: Ensure SDL2 is installed. The default path is `C:/SDL2/i686-w64-mingw32`.

### Build Instructions
1.  Open your terminal in the project root.
2.  Run the build command:
    ```bash
    make
    ```
3.  Execute the engine:
    ```bash
    make run
    ```


---

## 🎮 Controls and Calibration

### Controls
* **Left/Right Arrows**: Apply horizontal forces to the player box.
* **Up Arrow**: Jump (Triggered by an instantaneous velocity change).
* **Window [X]**: Cleanly shuts down SDL2 and exits.

### Calibration (SI Units)
To maintain scientific accuracy, the engine uses a specific scaling factor:
* **Scale**: **50 Pixels = 1 Meter**
* **Gravity**: $9.80665\ m/s^2$ (Applied downward)
* **Time Step**: Fixed at $1/60$ seconds ($60\text{Hz}$) for deterministic behavior.

---

## 🚀 Future Roadmap
* **SAT (Separating Axis Theorem)**: To support rotated polygons.
* **Spatial Hashing**: Broadphase optimization for thousands of objects.
* **Constraints & Joints**: Implementation of springs and hinges.

How do you want to handle the next step? We can start implementing **Rotational Impulse** so objects tumble realistically, or we could add **Circle-to-Box** collision detection!
