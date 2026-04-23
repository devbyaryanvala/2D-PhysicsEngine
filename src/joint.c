#include "ep/joint.h"
#include <math.h>
#include <string.h>

#define BAUMGARTE_JOINT  0.2f
#define JOINT_SLOP       0.005f

// ── Helpers ────────────────────────────────────────────────────────────────────

// Transform a local-space anchor to world space
static Vec2 local_to_world(const Body* b, Vec2 local) {
    float c = cosf(b->orientation);
    float s = sinf(b->orientation);
    return (Vec2){
        local.x * c - local.y * s + b->position.x,
        local.x * s + local.y * c + b->position.y
    };
}

// Velocity at a world-space point on body b (linear + angular contribution)
static Vec2 point_velocity(const Body* b, Vec2 r) {
    return (Vec2){
        b->velocity.x - b->angularVelocity * r.y,
        b->velocity.y + b->angularVelocity * r.x
    };
}

// Apply a linear impulse at arm r on body b (updates vel + angVel)
static void apply_linear_impulse(Body* b, Vec2 impulse, Vec2 r, float sign) {
    if (b->type != BODY_TYPE_DYNAMIC) return;
    b->velocity.x += sign * b->invMass * impulse.x;
    b->velocity.y += sign * b->invMass * impulse.y;
    b->angularVelocity += sign * b->invInertia * vec2_cross(r, impulse);
}

// 2×2 matrix inversion (returns 1 on success, 0 if singular)
static int mat2_invert(const float m[2][2], float out[2][2]) {
    float det = m[0][0]*m[1][1] - m[0][1]*m[1][0];
    if (fabsf(det) < 1e-9f) return 0;
    float inv = 1.0f / det;
    out[0][0] =  m[1][1] * inv;
    out[0][1] = -m[0][1] * inv;
    out[1][0] = -m[1][0] * inv;
    out[1][1] =  m[0][0] * inv;
    return 1;
}

// ── Factory ────────────────────────────────────────────────────────────────────

void joint_init_distance(Joint* j, Body* a, Body* b,
                          Vec2 worldAnchorA, Vec2 worldAnchorB,
                          float length, float stiffness, float damping) {
    memset(j, 0, sizeof(Joint));
    j->type  = JOINT_DISTANCE;
    j->bodyA = a;
    j->bodyB = b;
    // Store anchors in local body space so they survive rotation
    float cA = cosf(a->orientation), sA = sinf(a->orientation);
    float cB = cosf(b->orientation), sB = sinf(b->orientation);
    Vec2 dA = vec2_sub(worldAnchorA, a->position);
    Vec2 dB = vec2_sub(worldAnchorB, b->position);
    j->localAnchorA = (Vec2){ dA.x*cA + dA.y*sA, -dA.x*sA + dA.y*cA };
    j->localAnchorB = (Vec2){ dB.x*cB + dB.y*sB, -dB.x*sB + dB.y*cB };
    j->distance.length    = (length < 0.0f) ? vec2_length(vec2_sub(worldAnchorB, worldAnchorA)) : length;
    j->distance.stiffness = stiffness;
    j->distance.damping   = damping;
}

void joint_init_revolute(Joint* j, Body* a, Body* b, Vec2 worldAnchor) {
    memset(j, 0, sizeof(Joint));
    j->type  = JOINT_REVOLUTE;
    j->bodyA = a;
    j->bodyB = b;
    float cA = cosf(a->orientation), sA = sinf(a->orientation);
    float cB = cosf(b->orientation), sB = sinf(b->orientation);
    Vec2 dA = vec2_sub(worldAnchor, a->position);
    Vec2 dB = vec2_sub(worldAnchor, b->position);
    j->localAnchorA = (Vec2){ dA.x*cA + dA.y*sA, -dA.x*sA + dA.y*cA };
    j->localAnchorB = (Vec2){ dB.x*cB + dB.y*sB, -dB.x*sB + dB.y*cB };
    j->revolute.referenceAngle = b->orientation - a->orientation;
}

void joint_init_mouse(Joint* j, Body* b, Vec2 localAnchor, Vec2 worldTarget,
                       float maxForce, float stiffness, float damping) {
    memset(j, 0, sizeof(Joint));
    j->type         = JOINT_MOUSE;
    j->bodyA        = NULL;
    j->bodyB        = b;
    j->localAnchorB = localAnchor;
    j->mouse.target    = worldTarget;
    j->mouse.maxForce  = maxForce;
    j->mouse.stiffness = stiffness;
    j->mouse.damping   = damping;
}

// ── Distance Joint Solver ──────────────────────────────────────────────────────

static void distance_pre_solve(Joint* j, float dt) {
    Body* a = j->bodyA;
    Body* b = j->bodyB;
    DistanceJointData* d = &j->distance;

    Vec2 rA = vec2_sub(local_to_world(a, j->localAnchorA), a->position);
    Vec2 rB = vec2_sub(local_to_world(b, j->localAnchorB), b->position);
    Vec2 diff = vec2_sub(vec2_add(b->position, rB), vec2_add(a->position, rA));
    float length = vec2_length(diff);

    if (length < 1e-6f) { d->u = (Vec2){1,0}; return; }
    d->u = vec2_scale(diff, 1.0f / length);

    // Effective mass along the constraint axis
    float rAxU = vec2_cross(rA, d->u);
    float rBxU = vec2_cross(rB, d->u);
    float invMass = a->invMass + b->invMass
                  + rAxU * rAxU * a->invInertia
                  + rBxU * rBxU * b->invInertia;

    if (d->stiffness > 0.0f) {
        // Soft constraint: spring-damper frequency response
        float omega   = 2.0f * 3.14159f * d->stiffness;
        float zeta    = d->damping;
        float gamma   = 1.0f / (dt * (2.0f*zeta*omega + dt*omega*omega));
        float beta    = dt * omega*omega * gamma;
        d->mass       = 1.0f / (invMass + gamma);
        d->bias       = beta * (length - d->length);
    } else {
        // Hard constraint
        d->mass = (invMass > 0) ? 1.0f / invMass : 0.0f;
        float C = length - d->length;
        d->bias = (BAUMGARTE_JOINT / dt) * fmaxf(fabsf(C) - JOINT_SLOP, 0.0f) * (C > 0 ? 1.f : -1.f);
    }

    // Warm-start
    Vec2 P = vec2_scale(d->u, d->impulse);
    apply_linear_impulse(a, P, rA, -1.0f);
    apply_linear_impulse(b, P, rB,  1.0f);
}

static void distance_solve_velocity(Joint* j) {
    Body* a = j->bodyA;
    Body* b = j->bodyB;
    DistanceJointData* d = &j->distance;

    Vec2 rA = vec2_sub(local_to_world(a, j->localAnchorA), a->position);
    Vec2 rB = vec2_sub(local_to_world(b, j->localAnchorB), b->position);

    Vec2 vA = point_velocity(a, rA);
    Vec2 vB = point_velocity(b, rB);
    float Cdot = vec2_dot(d->u, vec2_sub(vB, vA));

    float lambda = -d->mass * (Cdot + d->bias);
    d->impulse += lambda;

    Vec2 P = vec2_scale(d->u, lambda);
    apply_linear_impulse(a, P, rA, -1.0f);
    apply_linear_impulse(b, P, rB,  1.0f);
}

// ── Revolute Joint Solver ──────────────────────────────────────────────────────

static void revolute_pre_solve(Joint* j, float dt) {
    Body* a = j->bodyA;
    Body* b = j->bodyB;
    RevoluteJointData* r = &j->revolute;

    Vec2 rA = vec2_sub(local_to_world(a, j->localAnchorA), a->position);
    Vec2 rB = vec2_sub(local_to_world(b, j->localAnchorB), b->position);

    // Build 2x2 effective mass matrix and invert it
    float K[2][2];
    K[0][0] = a->invMass + b->invMass + a->invInertia*rA.y*rA.y + b->invInertia*rB.y*rB.y;
    K[0][1] =                          -a->invInertia*rA.x*rA.y  - b->invInertia*rB.x*rB.y;
    K[1][0] = K[0][1];
    K[1][1] = a->invMass + b->invMass + a->invInertia*rA.x*rA.x + b->invInertia*rB.x*rB.x;
    mat2_invert(K, r->K);

    // Compute position error and store as bias velocity (Baumgarte)
    // IMPORTANT: bias is stored separately, never added to accumulated impulse
    Vec2 posA = vec2_add(a->position, rA);
    Vec2 posB = vec2_add(b->position, rB);
    Vec2 C    = vec2_sub(posB, posA);
    r->biasVelocity = (Vec2){
        (BAUMGARTE_JOINT / dt) * C.x,
        (BAUMGARTE_JOINT / dt) * C.y
    };

    // Motor effective mass
    if (r->enableMotor) {
        float invI = a->invInertia + b->invInertia;
        r->motorImpulse = (invI > 0) ? 1.0f/invI : 0.0f;
    }

    // Warm-start: re-apply accumulated impulse from previous frame
    Vec2 P = r->impulse;
    apply_linear_impulse(a, P, rA, -1.0f);
    apply_linear_impulse(b, P, rB,  1.0f);
}

static void revolute_solve_velocity(Joint* j) {
    Body* a = j->bodyA;
    Body* b = j->bodyB;
    RevoluteJointData* r = &j->revolute;

    Vec2 rA = vec2_sub(local_to_world(a, j->localAnchorA), a->position);
    Vec2 rB = vec2_sub(local_to_world(b, j->localAnchorB), b->position);

    // Relative velocity at anchor points
    Vec2 vA = point_velocity(a, rA);
    Vec2 vB = point_velocity(b, rB);
    Vec2 Cdot = vec2_sub(vB, vA);

    // Add Baumgarte bias to drive positional error to zero
    Vec2 Cdot_bias = (Vec2){ Cdot.x + r->biasVelocity.x, Cdot.y + r->biasVelocity.y };

    // Apply 2x2 effective mass matrix to compute impulse
    Vec2 lambda = (Vec2){
        -(r->K[0][0]*Cdot_bias.x + r->K[0][1]*Cdot_bias.y),
        -(r->K[1][0]*Cdot_bias.x + r->K[1][1]*Cdot_bias.y)
    };
    r->impulse = vec2_add(r->impulse, lambda);

    apply_linear_impulse(a, lambda, rA, -1.0f);
    apply_linear_impulse(b, lambda, rB,  1.0f);

    // Motor constraint (independent of position constraint)
    if (r->enableMotor) {
        float relW = b->angularVelocity - a->angularVelocity - r->motorSpeed;
        float motorLambda = -r->motorImpulse * relW;
        float oldMotor = r->motorImpulse;
        float maxImpulse = r->maxMotorTorque;
        r->motorImpulse = fmaxf(-maxImpulse, fminf(oldMotor + motorLambda, maxImpulse));
        motorLambda = r->motorImpulse - oldMotor;
        if (a->type == BODY_TYPE_DYNAMIC) a->angularVelocity -= a->invInertia * motorLambda;
        if (b->type == BODY_TYPE_DYNAMIC) b->angularVelocity += b->invInertia * motorLambda;
    }
}

// ── Mouse Joint Solver ─────────────────────────────────────────────────────────

static void mouse_pre_solve(Joint* j, float dt) {
    Body* b = j->bodyB;
    MouseJointData* m = &j->mouse;

    Vec2 rB  = vec2_sub(local_to_world(b, j->localAnchorB), b->position);
    Vec2 pos = vec2_add(b->position, rB);
    m->C = vec2_sub(pos, m->target);

    float omega = 2.0f * 3.14159f * m->stiffness;
    float zeta  = m->damping;
    float gamma = 1.0f / (dt * (2.0f*zeta*omega + dt*omega*omega));
    float beta  = dt * omega*omega * gamma;

    float K00 = b->invMass + b->invInertia*rB.y*rB.y + gamma;
    float K11 = b->invMass + b->invInertia*rB.x*rB.x + gamma;
    float K01 = -b->invInertia*rB.x*rB.y;
    float Km[2][2] = {{ K00, K01 }, { K01, K11 }};
    mat2_invert(Km, m->K);

    m->C.x *= beta;
    m->C.y *= beta;

    // Warm-start
    Vec2 P = m->impulse;
    if (b->type == BODY_TYPE_DYNAMIC) {
        b->velocity.x += b->invMass * P.x;
        b->velocity.y += b->invMass * P.y;
        b->angularVelocity += b->invInertia * vec2_cross(rB, P);
    }
    (void)dt;
}

static void mouse_solve_velocity(Joint* j) {
    Body* b = j->bodyB;
    MouseJointData* m = &j->mouse;

    Vec2 rB  = vec2_sub(local_to_world(b, j->localAnchorB), b->position);
    Vec2 vB  = point_velocity(b, rB);

    Vec2 Cdot = vec2_add(vB, m->C);

    Vec2 lambda = (Vec2){
        -(m->K[0][0]*Cdot.x + m->K[0][1]*Cdot.y),
        -(m->K[1][0]*Cdot.x + m->K[1][1]*Cdot.y)
    };

    Vec2 oldImpulse = m->impulse;
    m->impulse = vec2_add(m->impulse, lambda);

    // Clamp to maxForce * dt (done per frame so we approximate with maxForce)
    float maxImpulse = m->maxForce;
    float len = vec2_length(m->impulse);
    if (len > maxImpulse) m->impulse = vec2_scale(m->impulse, maxImpulse / len);

    lambda = vec2_sub(m->impulse, oldImpulse);

    if (b->type == BODY_TYPE_DYNAMIC) {
        b->velocity.x += b->invMass * lambda.x;
        b->velocity.y += b->invMass * lambda.y;
        b->angularVelocity += b->invInertia * vec2_cross(rB, lambda);
    }
}

// ── Public interface ───────────────────────────────────────────────────────────

void joint_pre_solve(Joint* j, float dt) {
    switch (j->type) {
        case JOINT_DISTANCE: distance_pre_solve(j, dt); break;
        case JOINT_REVOLUTE: revolute_pre_solve(j, dt); break;
        case JOINT_MOUSE:    mouse_pre_solve(j, dt);    break;
    }
}

void joint_solve_velocity(Joint* j) {
    switch (j->type) {
        case JOINT_DISTANCE: distance_solve_velocity(j); break;
        case JOINT_REVOLUTE: revolute_solve_velocity(j); break;
        case JOINT_MOUSE:    mouse_solve_velocity(j);    break;
    }
}

void joint_solve_position(Joint* j) {
    // Position error is corrected via Baumgarte bias baked into velocity solve.
    // For very stiff hard constraints, an additional direct position projection
    // can be added here in a future iteration.
    (void)j;
}
