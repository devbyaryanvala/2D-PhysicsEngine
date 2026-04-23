#include "ep/solver.h"
#include "ep/material.h"
#include <math.h>
#include <string.h>

// Baumgarte stabilization constants
#define BAUMGARTE        0.2f   // How strongly to push overlapping bodies apart per frame
#define PENETRATION_SLOP 0.02f  // Minimum penetration before Baumgarte kicks in (prevents jitter)
#define RESTITUTION_SLOP 1.0f   // Relative velocity below this threshold is treated as 0 (prevents jitter)

void solver_clear(Solver* s) {
    s->count = 0;
}

// Effective mass for a single constraint direction:
// K = invMassA + invMassB + (rA x n)^2 * invInertiaA + (rB x n)^2 * invInertiaB
static float compute_effective_mass(Body* a, Body* b, Vec2 rA, Vec2 rB, Vec2 dir) {
    float rACrossDir = vec2_cross(rA, dir);
    float rBCrossDir = vec2_cross(rB, dir);
    float k = a->invMass + b->invMass
            + (rACrossDir * rACrossDir) * a->invInertia
            + (rBCrossDir * rBCrossDir) * b->invInertia;
    return (k > 1e-9f) ? (1.0f / k) : 0.0f;
}

void solver_add_manifold(Solver* s, ContactManifold* m, float dt) {
    Body* a = m->bodyA;
    Body* b = m->bodyB;

    // Use material system for physically correct property blending
    const MaterialProps* mA = material_get(a->material);
    const MaterialProps* mB = material_get(b->material);
    MaterialProps blended = material_blend(mA, mB);

    float blendedFriction    = blended.staticFriction;   // Static friction for cone clamping
    float blendedRestitution = blended.restitution;
    float blendedRolling     = blended.rollingFriction;

    for (int i = 0; i < m->contactCount; i++) {
        if (s->count >= EP_MAX_CONSTRAINTS) return;

        ContactConstraint* c = &s->constraints[s->count++];
        memset(c, 0, sizeof(ContactConstraint));

        c->bodyA = a;
        c->bodyB = b;
        c->contactPoint = m->contacts[i];
        c->normal = m->normal;
        c->friction = blendedFriction;
        c->rollingFriction = blendedRolling;
        c->restitution = blendedRestitution;

        // r = contact - center
        c->rA = vec2_sub(c->contactPoint, a->position);
        c->rB = vec2_sub(c->contactPoint, b->position);

        // Tangent = perp to normal, in the plane of relative sliding
        Vec2 relVel = vec2_sub(
            vec2_add(b->velocity, vec2_cross_sv(b->angularVelocity, c->rB)),
            vec2_add(a->velocity, vec2_cross_sv(a->angularVelocity, c->rA))
        );
        float relVelTangent = vec2_dot(relVel, (Vec2){ -c->normal.y, c->normal.x });
        c->tangent = (relVelTangent >= 0.0f)
            ? (Vec2){ -c->normal.y,  c->normal.x }
            : (Vec2){  c->normal.y, -c->normal.x };

        // Pre-compute effective masses
        c->normalMass  = compute_effective_mass(a, b, c->rA, c->rB, c->normal);
        c->tangentMass = compute_effective_mass(a, b, c->rA, c->rB, c->tangent);

        // Baumgarte bias velocity: target separation speed we want the constraint to achieve.
        // Must be POSITIVE so that lambda = normalMass * (-vn + bias) stays positive when penetrating.
        float penetration = m->penetration;
        c->biasVelocity = (BAUMGARTE / dt) * fmaxf(penetration - PENETRATION_SLOP, 0.0f);

        // Restitution bias: closingVel is negative (approaching), so negate it to get positive bounce speed.
        float closingVel = vec2_dot(relVel, c->normal);
        if (closingVel < -RESTITUTION_SLOP) {
            c->biasVelocity += -c->restitution * closingVel;  // closingVel < 0, so -e*vn > 0
        }
    }
}

// Apply a velocity impulse along a direction at the contact rA/rB points
static void apply_impulse(Body* a, Body* b, Vec2 rA, Vec2 rB, Vec2 dir, float magnitude) {
    Vec2 impulse = vec2_scale(dir, magnitude);
    if (a->type == BODY_TYPE_DYNAMIC) {
        a->velocity = vec2_sub(a->velocity, vec2_scale(impulse, a->invMass));
        a->angularVelocity -= a->invInertia * vec2_cross(rA, impulse);
    }
    if (b->type == BODY_TYPE_DYNAMIC) {
        b->velocity = vec2_add(b->velocity, vec2_scale(impulse, b->invMass));
        b->angularVelocity += b->invInertia * vec2_cross(rB, impulse);
    }
}

void solver_solve_velocity(Solver* s) {
    for (int iter = 0; iter < EP_SOLVER_ITERATIONS; iter++) {
        for (int i = 0; i < s->count; i++) {
            ContactConstraint* c = &s->constraints[i];
            Body* a = c->bodyA;
            Body* b = c->bodyB;

            // Relative velocity at contact point (includes angular contributions)
            Vec2 vA = vec2_add(a->velocity, vec2_cross_sv(a->angularVelocity, c->rA));
            Vec2 vB = vec2_add(b->velocity, vec2_cross_sv(b->angularVelocity, c->rB));
            Vec2 relVel = vec2_sub(vB, vA);

            // ---- Normal impulse ----
            float vn = vec2_dot(relVel, c->normal);
            float lambda = c->normalMass * (-vn + c->biasVelocity);

            // Clamp: normal impulse must be >= 0 (can't pull bodies together)
            float oldNormal = c->normalImpulse;
            c->normalImpulse = fmaxf(oldNormal + lambda, 0.0f);
            float deltaLambda = c->normalImpulse - oldNormal;

            apply_impulse(a, b, c->rA, c->rB, c->normal, deltaLambda);

            // ---- Tangent (friction) impulse ----
            // Re-compute relative velocity after normal impulse
            vA = vec2_add(a->velocity, vec2_cross_sv(a->angularVelocity, c->rA));
            vB = vec2_add(b->velocity, vec2_cross_sv(b->angularVelocity, c->rB));
            relVel = vec2_sub(vB, vA);

            float vt = vec2_dot(relVel, c->tangent);
            float lambdaT = c->tangentMass * (-vt);

            // Coulomb friction cone: |friction impulse| <= mu * |normal impulse|
            float frictionLimit = c->friction * c->normalImpulse;
            float oldTangent = c->tangentImpulse;
            c->tangentImpulse = fmaxf(-frictionLimit, fminf(oldTangent + lambdaT, frictionLimit));
            float deltaTangent = c->tangentImpulse - oldTangent;

            apply_impulse(a, b, c->rA, c->rB, c->tangent, deltaTangent);

            // ---- Rolling friction ----
            // Damp angular velocity by an amount proportional to the normal impulse.
            // This models energy dissipation at the contact patch (tyre/ball on surface).
            float rollLimit = c->rollingFriction * c->normalImpulse;
            if (a->type == BODY_TYPE_DYNAMIC) {
                float wA = a->angularVelocity;
                float dw = -wA;  // target: zero angular velocity
                float maxDW = rollLimit * a->invInertia;
                if (dw > maxDW) dw = maxDW;
                if (dw < -maxDW) dw = -maxDW;
                a->angularVelocity += dw;
            }
            if (b->type == BODY_TYPE_DYNAMIC) {
                float wB = b->angularVelocity;
                float dw = -wB;
                float maxDW = rollLimit * b->invInertia;
                if (dw > maxDW) dw = maxDW;
                if (dw < -maxDW) dw = -maxDW;
                b->angularVelocity += dw;
            }
        }
    }
}

void solver_solve_position(Solver* s) {
    // Position correction is handled via Baumgarte bias in the velocity solve.
    // This pass handles any residual drift with a hard projection step.
    for (int i = 0; i < s->count; i++) {
        ContactConstraint* c = &s->constraints[i];
        Body* a = c->bodyA;
        Body* b = c->bodyB;

        float invMassSum = a->invMass + b->invMass;
        if (invMassSum == 0.0f) continue;

        // Recompute penetration from current positions
        Vec2 rA = vec2_sub(c->contactPoint, a->position);
        Vec2 rB = vec2_sub(c->contactPoint, b->position);
        Vec2 separation = vec2_sub(
            vec2_add(b->position, rB),
            vec2_add(a->position, rA)
        );
        float penetration = -vec2_dot(separation, c->normal);
        if (penetration <= PENETRATION_SLOP) continue;

        float corrMag = fmaxf(penetration - PENETRATION_SLOP, 0.0f) * 0.5f / invMassSum;
        Vec2 correction = vec2_scale(c->normal, corrMag);

        if (a->type == BODY_TYPE_DYNAMIC) a->position = vec2_sub(a->position, vec2_scale(correction, a->invMass));
        if (b->type == BODY_TYPE_DYNAMIC) b->position = vec2_add(b->position, vec2_scale(correction, b->invMass));
    }
}
