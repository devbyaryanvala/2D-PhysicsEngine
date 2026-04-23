#ifndef EP_SOLVER_H
#define EP_SOLVER_H

#include "collision.h"

// Maximum number of contact constraints the solver can hold per frame
#define EP_MAX_CONSTRAINTS 512
#define EP_SOLVER_ITERATIONS 10

// Per-contact-point velocity constraint (one manifold can have up to 2)
typedef struct {
    // Bodies
    Body* bodyA;
    Body* bodyB;

    // Contact geometry
    Vec2 contactPoint;  // World-space contact point
    Vec2 rA;            // Vector from center of A to contact point
    Vec2 rB;            // Vector from center of B to contact point
    Vec2 normal;        // Collision normal (A -> B)
    Vec2 tangent;       // Friction tangent (perp to normal)

    // Pre-computed effective masses
    float normalMass;   // 1 / (J M^-1 J^T) for normal
    float tangentMass;  // 1 / (J M^-1 J^T) for tangent

    // Bias velocity (Baumgarte stabilization)
    float biasVelocity;

    // Accumulated impulses for warm-starting
    float normalImpulse;
    float tangentImpulse;

    // Blended friction and restitution for this contact
    float friction;
    float rollingFriction;
    float restitution;
} ContactConstraint;

typedef struct {
    ContactConstraint constraints[EP_MAX_CONSTRAINTS];
    int count;
} Solver;

// Populate the solver with constraints from a manifold
void solver_add_manifold(Solver* s, ContactManifold* m, float dt);

// Run all velocity constraint iterations (normal + friction + rolling friction)
void solver_solve_velocity(Solver* s);

// Apply final positional correction (Baumgarte is baked into biasVelocity, this is a fallback)
void solver_solve_position(Solver* s);

// Clear all constraints (call at start of each frame)
void solver_clear(Solver* s);

#endif
