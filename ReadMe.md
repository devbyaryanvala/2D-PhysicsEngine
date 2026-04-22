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
