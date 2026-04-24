#include "ep/world.h"
#include "ep/island.h"
#include "ep/collision.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

World* world_create(Vec2 gravity) {
    World* world = (World*)malloc(sizeof(World));
    world->bodyCount = 0;
    world->jointCount = 0;
    world->gravity = gravity;
    world->broadphase = broadphase_create(BROADPHASE_BVH);
    return world;
}

void world_destroy(World* world) {
    broadphase_destroy(world->broadphase);
    free(world);
}

void world_add_body(World* world, Body* body) {
    if (world->bodyCount < EP_MAX_BODIES) {
        world->bodies[world->bodyCount++] = body;
        broadphase_insert(world->broadphase, body);
    }
}

void world_add_joint(World* world, Joint* joint) {
    if (world->jointCount < EP_MAX_JOINTS) {
        world->joints[world->jointCount++] = joint;
    }
}

static bool are_bodies_connected(Body* a, Body* b, Joint** joints, int jointCount) {
    if (!joints) return false;
    for (int i = 0; i < jointCount; i++) {
        if (!joints[i]->collideConnected) {
            if ((joints[i]->bodyA == a && joints[i]->bodyB == b) ||
                (joints[i]->bodyA == b && joints[i]->bodyB == a)) {
                return true;
            }
        }
    }
    return false;
}

void world_step(World* world, float dt) {
    // 0. CCD Pass for Bullets
    bool ccdStepped[EP_MAX_BODIES] = {false};
    
    for (int i = 0; i < world->bodyCount; i++) {
        Body* b = world->bodies[i];
        if (!b->isBullet || b->type != BODY_TYPE_DYNAMIC) continue;
        
        float dist = vec2_length(b->velocity) * dt;
        if (dist > 0.5f) { // Tunneling threshold (0.5 meters)
            int steps = (int)ceilf(dist / 0.5f);
            float subDt = dt / steps;
            
            for (int s = 0; s < steps; s++) {
                // Integrate just the bullet
                body_integrate(b, subDt, world->gravity);
                collision_resolve_world_bounds(b);
                
                // Narrow-phase against all static bodies
                for (int j = 0; j < world->bodyCount; j++) {
                    Body* other = world->bodies[j];
                    if (other->type != BODY_TYPE_STATIC) continue;
                    
                    ContactManifold m;
                    if (collision_narrow_phase(b, other, &m)) {
                        // Resolve immediately!
                        Vec2 normal = (m.bodyA == b) ? m.normal : vec2_scale(m.normal, -1.0f);
                        
                        float velAlongNormal = vec2_dot(b->velocity, normal);
                        if (velAlongNormal > 0) continue;
                        
                        float e = fminf(b->restitution, other->restitution);
                        float j_imp = -(1.0f + e) * velAlongNormal;
                        j_imp /= b->invMass; // other is static (invMass=0)
                        
                        Vec2 impulse = vec2_scale(normal, j_imp);
                        b->velocity = vec2_add(b->velocity, vec2_scale(impulse, b->invMass));
                        
                        // Positional correction
                        float correctionMag = fmaxf(m.penetration - 0.01f, 0.0f) * 0.8f;
                        b->position = vec2_add(b->position, vec2_scale(normal, correctionMag));
                    }
                }
            }
            
            broadphase_update(world->broadphase, b);
            ccdStepped[i] = true;
        }
    }

    // 1. Integrate velocities and update broadphase
    for (int i = 0; i < world->bodyCount; i++) {
        if (ccdStepped[i]) continue; // Skip standard integration if CCD handled it
        
        Body* b = world->bodies[i];
        body_integrate(b, dt, world->gravity);
        broadphase_update(world->broadphase, b);
        collision_resolve_world_bounds(b);
    }

    // 2. Broadphase & Narrowphase collision detection
    int pairCount = broadphase_get_pairs(world->broadphase);
    CollisionPair* pairs = broadphase_get_pair_array(world->broadphase);
    
    world->manifoldCount = 0;

    for (int i = 0; i < pairCount; i++) {
        if (are_bodies_connected(pairs[i].bodyA, pairs[i].bodyB, world->joints, world->jointCount)) continue;
        
        ContactManifold m;
        if (collision_narrow_phase(pairs[i].bodyA, pairs[i].bodyB, &m)) {
            if (world->manifoldCount < EP_MAX_MANIFOLDS) {
                world->manifolds[world->manifoldCount++] = m;
            }
        }
    }

    // 3. Build Islands and Solve Constraints
    for (int i = 0; i < world->bodyCount; i++) {
        world->bodies[i]->islandId = -1;
    }
    
    bool* contactAdded = (bool*)calloc(world->manifoldCount, sizeof(bool));
    bool* jointAdded   = (bool*)calloc(world->jointCount, sizeof(bool));
    
    Body** stack = (Body**)malloc(sizeof(Body*) * world->bodyCount);
    
    int currentIsland = 0;
    Island island;
    island_init(&island, world->bodyCount, world->manifoldCount, world->jointCount);
    
    for (int i = 0; i < world->bodyCount; i++) {
        Body* seed = world->bodies[i];
        if (seed->islandId != -1) continue;
        if (seed->type == BODY_TYPE_STATIC) continue;
        if (seed->isSleeping) continue;

        island_clear(&island);
        
        int stackCount = 0;
        stack[stackCount++] = seed;
        seed->islandId = currentIsland;
        
        while (stackCount > 0) {
            Body* b = stack[--stackCount];
            island_add_body(&island, b);
            
            b->isSleeping = false;
            
            for (int k = 0; k < world->manifoldCount; k++) {
                if (contactAdded[k]) continue;
                ContactManifold* m = &world->manifolds[k];
                if (m->bodyA == b || m->bodyB == b) {
                    contactAdded[k] = true;
                    island_add_manifold(&island, m);
                    
                    Body* other = (m->bodyA == b) ? m->bodyB : m->bodyA;
                    if (other->islandId == -1 && other->type != BODY_TYPE_STATIC) {
                        other->islandId = currentIsland;
                        stack[stackCount++] = other;
                    }
                }
            }
            
            for (int k = 0; k < world->jointCount; k++) {
                if (jointAdded[k]) continue;
                Joint* j = world->joints[k];
                if (j->bodyA == b || j->bodyB == b) {
                    jointAdded[k] = true;
                    island_add_joint(&island, j);
                    
                    Body* other = (j->bodyA == b) ? j->bodyB : j->bodyA;
                    if (other->islandId == -1 && other->type != BODY_TYPE_STATIC) {
                        other->islandId = currentIsland;
                        stack[stackCount++] = other;
                    }
                }
            }
        }
        
        island_solve(&island, dt);
        island_update_sleep(&island, dt);
        currentIsland++;
    }
    
    island_destroy(&island);
    free(stack);
    free(contactAdded);
    free(jointAdded);
}
