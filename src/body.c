#include "ep/body.h"
// Simple physics body implementation for the 2D physics engine
void body_init(Body* b, float x, float y, float w, float h, float density) {

    // Initialize body properties based on position, size, and density
    b->position = (Vec2){x, y};

    // Start with zero velocity and force
    b->velocity = (Vec2){0, 0};

    // Force will be accumulated each frame before integration
    b->force = (Vec2){0, 0};

    // Set size and calculate mass properties
    b->width = w; b->height = h;

    // Calculate mass and inertia based on area and density
    float area = w * h;

    // Mass is area times density, with a check to prevent division by zero for static bodies
    b->mass = area * density;

    // Inverse mass is zero for static bodies (infinite mass) and 1/mass for dynamic bodies
    b->invMass = (b->mass > 0) ? 1.0f / b->mass : 0.0f;

    // Inertia for a rectangle is (1/12) * mass * (width^2 + height^2)
    b->inertia = (1.0f / 12.0f) * b->mass * (w * w + h * h);

    // Inverse inertia is zero for static bodies and 1/inertia for dynamic bodies
    b->invInertia = (b->inertia > 0) ? 1.0f / b->inertia : 0.0f;

    // Set default restitution and friction values for the body
    b->restitution = 0.5f; 
    b->friction = 0.8f;

    // Initialize orientation and angular properties to zero
    b->orientation = 0; 
    b->angularVelocity = 0; 
    b->torque = 0;
}

// Applies a force to the body, which will affect its acceleration during integration
void body_apply_force(Body* b, Vec2 f) {
    // Accumulate the applied force for the current frame
    b->force = vec2_add(b->force, f); 
}

// Integrates the body's position and velocity over time using simple Euler integration
void body_integrate(Body* b, float dt) {

    // Static bodies (with infinite mass) do not move, so we skip integration for them
    if (b->invMass == 0){
        return;
    };

    // Calculate acceleration from the accumulated force and inverse mass
    Vec2 acceleration = vec2_scale(b->force, b->invMass);

    // Update velocity and position based on acceleration and time step
    b->velocity = vec2_add(b->velocity, vec2_scale(acceleration, dt));
    b->position = vec2_add(b->position, vec2_scale(b->velocity, dt));
    b->angularVelocity += (b->torque * b->invInertia) * dt;
    b->orientation += b->angularVelocity * dt;
    b->force = (Vec2){0, 0}; b->torque = 0;
}