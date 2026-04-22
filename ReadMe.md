# 2D Physics Engine

A lightweight, C-based 2D physics engine demonstrating fundamental physics concepts, including semi-implicit Euler integration, Axis-Aligned Bounding Box (AABB) collision detection, and basic collision resolution. The project utilizes SDL2 for hardware-accelerated rendering.

## Features

* **Semi-Implicit Euler Integration**: Accurately calculates body acceleration, velocity, and position based on applied forces and mass.
* **AABB Collision System**: Efficiently detects overlaps between rectangular bodies.
* **Collision Resolution**: Handles positional correction and impulse-based velocity changes with configurable restitution (bounciness).
* **Force-Based Movement**: Supports external forces such as constant gravity and user-defined movement forces.
* **SDL2 Rendering**: Real-time visualization of physics bodies in an 800x600 window.

## Project Structure

* `include/`: Header files defining core data structures for vectors and physics bodies.
* `src/`: Implementation files for math utilities, integration logic, and the main simulation loop.
* `build/`: Output directory for the compiled executable (`engine.exe`).
* `Makefile`: Automated build script for the GCC compiler.

## Prerequisites

To build and run this engine, you will need:

1.  **GCC Compiler**: A C99-compliant compiler (such as MinGW for Windows).
2.  **SDL2 Library**: The engine is configured to look for SDL2 in `C:/SDL2/i686-w64-mingw32/`.
    * *Note: If your SDL2 installation is in a different directory, update the `CFLAGS` and library paths in the `Makefile`*.

## Building and Running

### 1. Compile the Project
Open your terminal in the project root directory and run:
```bash
make

This command compiles the source files and links them against SDL2 to create `build/engine.exe`.

### 2. Run the Application
Launch the engine immediately after building with:

```bash
make run
```


### Controls
* **Left Arrow Key**: Apply a leftward force of -500 units to the player box.
* **Right Arrow Key**: Apply a rightward force of 500 units to the player box.
* **Close Window**: Gracefully exit the application and clean up SDL resources.

### Technical Implementation Details

#### Physics Integration
The engine uses **Semi-Implicit Euler Integration** to update the state of each physics body. Unlike standard Euler, this method updates velocity before position, which provides better energy conservation.



**Formulas used:**
1.  **Acceleration**: $\vec{a} = \vec{F} \cdot \text{invMass}$
2.  **Velocity Update**: $\vec{v}_{t+1} = \vec{v}_t + \vec{a} \cdot \Delta t$
3.  **Position Update**: $\vec{p}_{t+1} = \vec{p}_t + \vec{v}_{t+1} \cdot \Delta t$

#### Collision Detection (AABB)
The engine implements **Axis-Aligned Bounding Box** detection. It checks if two rectangles overlap by comparing their edges on both the X and Y axes.



**Collision Condition:**
A collision occurs if all the following are true:
* $A.x < B.x + B.width$
* $A.x + A.width > B.x$
* $A.y < B.y + B.height$
* $A.y + A.height > B.y$

#### Collision Resolution
When a collision is detected, the engine calculates the **penetration depth** on the axis of least overlap and moves the objects apart to prevent "sinking".



**Formulas used:**
1.  **Overlap X**: $\text{halfWidths} - |(A_{center}.x - B_{center}.x)|$
2.  **Velocity Reflection**: $\vec{v} = \vec{v} \cdot -\text{restitution}$

#### Static Bodies
Bodies with an `invMass` of 0 are treated as static. The integration function checks this value and skips the update for these objects, effectively giving them infinite mass.
