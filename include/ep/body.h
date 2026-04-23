#ifndef EP_BODY_H
#define EP_BODY_H

#include "vec2.h"
#include "material.h"
#include <stdbool.h>

#define EP_MAX_POLYGON_VERTICES 8

typedef enum {
    BODY_TYPE_STATIC,
    BODY_TYPE_KINEMATIC,
    BODY_TYPE_DYNAMIC
} BodyType;

typedef enum {
    SHAPE_TYPE_CIRCLE,
    SHAPE_TYPE_RECT,
    SHAPE_TYPE_POLYGON
} ShapeType;

typedef struct {
    ShapeType type;
    union {
        struct { float radius; } circle;
        struct { float width, height; } rect;
        struct {
            int vertexCount;
            Vec2 vertices[EP_MAX_POLYGON_VERTICES];
            Vec2 normals[EP_MAX_POLYGON_VERTICES];
        } polygon;
    };
} ShapeDef;

// Represents a physical body in the physics engine with properties for position, velocity, mass, and collision response
typedef struct {
    BodyType type;
    ShapeDef shape;

    Vec2 position;
    Vec2 velocity;
    Vec2 acceleration;
    Vec2 force;

    float orientation;     // radians
    float angularVelocity;
    float angularAcceleration;
    float torque;

    float mass, invMass;
    float inertia, invInertia;

    float restitution;
    float staticFriction;
    float dynamicFriction;
    
    float linearDamping;
    float angularDamping;

    Vec2 gravityScale; // Custom gravity vector scale, default (0, 1) or similar depending on world gravity
    bool useRK4;
    int broadphaseId;    // Used by BroadPhase BVH
    MaterialType material; // Physical material (drives friction, restitution, density)
} Body;

// Initializes a body with given parameters and calculates inverse mass and inertia for physics calculations
void body_init(Body* b, BodyType type, ShapeDef shape, float x, float y, float density);
void body_set_material(Body* b, MaterialType mat);
void body_apply_force(Body* b, Vec2 f);
void body_apply_torque(Body* b, float t);
void body_integrate(Body* b, float dt, Vec2 worldGravity);

#endif