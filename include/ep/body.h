#ifndef EP_BODY_H
#define EP_BODY_H

#include "vec2.h"

// Represents a physical body in the physics engine with properties for position, velocity, mass, and collision response
typedef struct {
    Vec2 position;
    Vec2 velocity;
    Vec2 force;
    float orientation, angularVelocity, torque;
    float mass, invMass, inertia, invInertia;
    float restitution, friction, width, height;
} Body;

// Initializes a body with given parameters and calculates inverse mass and inertia for physics calculations
void body_init(Body* b, float x, float y, float w, float h, float density);
void body_apply_force(Body* b, Vec2 f);
void body_integrate(Body* b, float dt);

#endif