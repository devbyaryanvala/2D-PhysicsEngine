#include "ep/island.h"
#include <stdlib.h>
#include <math.h>

void island_init(Island* island, int bodyCapacity, int manifoldCapacity, int jointCapacity) {
    island->bodyCapacity = bodyCapacity;
    island->manifoldCapacity = manifoldCapacity;
    island->jointCapacity = jointCapacity;
    island->bodyCount = 0;
    island->manifoldCount = 0;
    island->jointCount = 0;
    
    island->bodies = (Body**)malloc(sizeof(Body*) * bodyCapacity);
    island->manifolds = (ContactManifold**)malloc(sizeof(ContactManifold*) * manifoldCapacity);
    island->joints = (Joint**)malloc(sizeof(Joint*) * jointCapacity);
}

void island_destroy(Island* island) {
    free(island->bodies);
    free(island->manifolds);
    free(island->joints);
    island->bodyCapacity = 0;
    island->manifoldCapacity = 0;
    island->jointCapacity = 0;
}

void island_clear(Island* island) {
    island->bodyCount = 0;
    island->manifoldCount = 0;
    island->jointCount = 0;
}

void island_add_body(Island* island, Body* b) {
    if (island->bodyCount < island->bodyCapacity) {
        island->bodies[island->bodyCount++] = b;
    }
}

void island_add_manifold(Island* island, ContactManifold* m) {
    if (island->manifoldCount < island->manifoldCapacity) {
        island->manifolds[island->manifoldCount++] = m;
    }
}

void island_add_joint(Island* island, Joint* j) {
    if (island->jointCount < island->jointCapacity) {
        island->joints[island->jointCount++] = j;
    }
}

void island_solve(Island* island, float dt) {
    // 1. Solve constraints (contacts + joints)
    Solver solver;
    solver_clear(&solver);
    for (int i = 0; i < island->manifoldCount; i++) {
        solver_add_manifold(&solver, island->manifolds[i], dt);
    }
    
    // Warm start
    for (int i = 0; i < island->jointCount; i++) {
        joint_pre_solve(island->joints[i], dt);
    }
    
    // Velocity solve iterations
    for (int iter = 0; iter < EP_SOLVER_ITERATIONS; iter++) {
        solver_solve_velocity(&solver);
        for (int i = 0; i < island->jointCount; i++) {
            joint_solve_velocity(island->joints[i]);
        }
    }

    // Position solve
    solver_solve_position(&solver);
    for (int i = 0; i < island->jointCount; i++) {
        joint_solve_position(island->joints[i]);
    }
}

void island_update_sleep(Island* island, float dt) {
    float minSleepTime = 9999.0f;
    
    for (int i = 0; i < island->bodyCount; i++) {
        Body* b = island->bodies[i];
        if (b->type == BODY_TYPE_STATIC) continue;
        
        if (!b->allowSleep) {
            b->sleepTime = 0.0f;
            minSleepTime = 0.0f;
            continue;
        }

        // Check if velocity is below tolerance
        float linVelSq = vec2_dot(b->velocity, b->velocity);
        float angVel = fabsf(b->angularVelocity);
        
        if (linVelSq > EP_SLEEP_LINEAR_TOL * EP_SLEEP_LINEAR_TOL || 
            angVel > EP_SLEEP_ANGULAR_TOL) {
            b->sleepTime = 0.0f;
            minSleepTime = 0.0f;
        } else {
            b->sleepTime += dt;
            if (b->sleepTime < minSleepTime) {
                minSleepTime = b->sleepTime;
            }
        }
    }

    // If the entire island has been resting for long enough, put it to sleep
    if (minSleepTime >= EP_TIME_TO_SLEEP) {
        for (int i = 0; i < island->bodyCount; i++) {
            Body* b = island->bodies[i];
            b->isSleeping = true;
            b->velocity = (Vec2){0, 0};
            b->angularVelocity = 0.0f;
            b->force = (Vec2){0, 0};
            b->torque = 0.0f;
        }
    }
}
