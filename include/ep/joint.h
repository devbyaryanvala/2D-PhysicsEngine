#ifndef EP_JOINT_H
#define EP_JOINT_H

#include "body.h"

typedef enum {
    JOINT_DISTANCE,   // Fixed-length rod/rope between two anchor points
    JOINT_REVOLUTE,   // Pin joint — bodies share an anchor, can rotate freely
    JOINT_MOUSE       // Spring-damper that pulls a body toward a world-space target
} JointType;

// ── Distance Joint ─────────────────────────────────────────────────────────────
// Maintains |worldAnchorB - worldAnchorA| == length.
// stiffness/damping < 0 means rigid (hard constraint).
typedef struct {
    float length;       // Rest length (m)
    float stiffness;    // Spring constant (N/m), 0 = rigid
    float damping;      // Damping coefficient (N·s/m)
    // Solver cache
    Vec2  u;            // Unit vector A→B at pre-solve time
    float mass;         // Effective mass along u
    float impulse;      // Accumulated normal impulse
    float bias;         // Baumgarte/spring bias velocity
} DistanceJointData;

// ── Revolute (Pin) Joint ───────────────────────────────────────────────────────
// Constrains two anchor points to coincide.  Bodies can rotate freely.
typedef struct {
    float referenceAngle;  // angle_B - angle_A at rest
    // Motor
    bool  enableMotor;
    float motorSpeed;         // rad/s target
    float maxMotorTorque;     // N·m cap
    // Limits
    bool  enableLimits;
    float lowerAngle;         // rad
    float upperAngle;         // rad
    // Solver cache (2-DOF linear constraint)
    float K[2][2];            // Effective mass matrix (inverted)
    Vec2  impulse;            // Accumulated linear impulse (warm-start)
    Vec2  biasVelocity;       // Baumgarte correction velocity (recomputed each frame)
    float motorImpulse;
    float limitImpulse;
} RevoluteJointData;

// ── Mouse Joint ────────────────────────────────────────────────────────────────
typedef struct {
    Vec2  target;       // World-space pull target
    float maxForce;     // N cap
    float stiffness;    // Spring constant
    float damping;      // Damping coefficient
    // Solver cache
    float K[2][2];
    Vec2  impulse;
    Vec2  C;            // Position error
} MouseJointData;

// ── Unified Joint ──────────────────────────────────────────────────────────────
typedef struct Joint {
    JointType type;
    Body*     bodyA;
    Body*     bodyB;
    Vec2      localAnchorA;   // Anchor in body A local space
    Vec2      localAnchorB;   // Anchor in body B local space
    bool      collideConnected; // Allow collision between connected bodies?

    union {
        DistanceJointData distance;
        RevoluteJointData revolute;
        MouseJointData    mouse;
    };
} Joint;

// ── Factory helpers ────────────────────────────────────────────────────────────
void joint_init_distance(Joint* j, Body* a, Body* b,
                          Vec2 worldAnchorA, Vec2 worldAnchorB,
                          float length, float stiffness, float damping);

void joint_init_revolute(Joint* j, Body* a, Body* b, Vec2 worldAnchor);

void joint_init_mouse(Joint* j, Body* b,
                       Vec2 localAnchor, Vec2 worldTarget,
                       float maxForce, float stiffness, float damping);

// ── Solver interface ──────────────────────────────────────────────────────────
void joint_pre_solve(Joint* j, float dt);
void joint_solve_velocity(Joint* j);
void joint_solve_position(Joint* j);

#endif
