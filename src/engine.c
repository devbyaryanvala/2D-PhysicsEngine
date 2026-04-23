#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdio.h>
#include "ep/body.h"
#include "ep/config.h"
#include "ep/collision.h"
#include "ep/broadphase.h"
#include "ep/solver.h"
#include "ep/material.h"
#include "ep/joint.h"

// ══════════════════════════════════════════════════════════════════════════════
//  DEMO SELECTOR — Change this value to switch between demos
//
//    0 = Block Collision  (WASD/arrows to move a rubber box onto a concrete slab)
//    1 = Pendulum         (Revolute joint swinging box, arrows apply torque)
//    2 = Chain            (5-link chain of revolute joints hanging from ceiling)
//
// ══════════════════════════════════════════════════════════════════════════════
#define ACTIVE_DEMO 2

#define EP_MAX_JOINTS        64
#define EP_SOLVER_JOINT_ITERS 10

// ── Shared helpers ─────────────────────────────────────────────────────────────

static void run_solver(Solver* solver,
                        ContactManifold* manifolds, int manifoldCount,
                        Joint* joints, int jointCount)
{
    solver_clear(solver);
    for (int i = 0; i < manifoldCount; i++)
        solver_add_manifold(solver, &manifolds[i], FIXED_DT);

    for (int i = 0; i < jointCount; i++)
        joint_pre_solve(&joints[i], FIXED_DT);

    for (int iter = 0; iter < EP_SOLVER_JOINT_ITERS; iter++) {
        solver_solve_velocity(solver);
        for (int i = 0; i < jointCount; i++)
            joint_solve_velocity(&joints[i]);
    }

    solver_solve_position(solver);
    for (int i = 0; i < jointCount; i++)
        joint_solve_position(&joints[i]);
}

static void draw_body(SDL_Renderer* r, const Body* b,
                       Uint8 red, Uint8 green, Uint8 blue)
{
    SDL_SetRenderDrawColor(r, red, green, blue, 255);
    if (b->shape.type == SHAPE_TYPE_CIRCLE) {
        // Approximate circle as a small square for now
        int s = (int)METERS_TO_PIXELS(b->shape.circle.radius * 2.0f);
        SDL_Rect rc = {
            (int)METERS_TO_PIXELS(b->position.x) - s/2,
            (int)METERS_TO_PIXELS(b->position.y) - s/2,
            s, s
        };
        SDL_RenderFillRect(r, &rc);
    } else {
        SDL_Rect rc = {
            (int)METERS_TO_PIXELS(b->position.x - b->shape.rect.width  * 0.5f),
            (int)METERS_TO_PIXELS(b->position.y - b->shape.rect.height * 0.5f),
            (int)METERS_TO_PIXELS(b->shape.rect.width),
            (int)METERS_TO_PIXELS(b->shape.rect.height)
        };
        SDL_RenderFillRect(r, &rc);
    }
}

static void draw_contacts(SDL_Renderer* r,
                            const ContactManifold* manifolds, int count)
{
    SDL_SetRenderDrawColor(r, 255, 220, 0, 255);
    for (int i = 0; i < count; i++) {
        for (int k = 0; k < manifolds[i].contactCount; k++) {
            int cx = (int)METERS_TO_PIXELS(manifolds[i].contacts[k].x);
            int cy = (int)METERS_TO_PIXELS(manifolds[i].contacts[k].y);
            SDL_Rect cr = {cx-3, cy-3, 6, 6};
            SDL_RenderFillRect(r, &cr);
            SDL_RenderDrawLine(r, cx, cy,
                cx + (int)(manifolds[i].normal.x * 20),
                cy + (int)(manifolds[i].normal.y * 20));
        }
    }
}

static void draw_force_indicator(SDL_Renderer* r, const Body* b) {
    int px = (int)METERS_TO_PIXELS(b->position.x);
    int py = (int)METERS_TO_PIXELS(b->position.y);

    // Linear force vector (Red)
    if (b->force.x != 0 || b->force.y != 0) {
        float scale = 0.05f; // Scale down large forces for display
        int fx = (int)METERS_TO_PIXELS(b->position.x + b->force.x * scale);
        int fy = (int)METERS_TO_PIXELS(b->position.y + b->force.y * scale);
        SDL_SetRenderDrawColor(r, 255, 50, 50, 255);
        SDL_RenderDrawLine(r, px, py, fx, fy);
        SDL_Rect tip = {fx-3, fy-3, 6, 6};
        SDL_RenderFillRect(r, &tip);
    }

    // Torque indicator (Magenta arc approximation using a line offset)
    if (b->torque != 0) {
        float sign = (b->torque > 0) ? 1.0f : -1.0f;
        int tx = (int)METERS_TO_PIXELS(b->position.x + sign * 0.5f);
        int ty = (int)METERS_TO_PIXELS(b->position.y - 0.5f);
        SDL_SetRenderDrawColor(r, 255, 50, 255, 255);
        SDL_RenderDrawLine(r, px, py - (int)METERS_TO_PIXELS(0.5f), tx, ty);
        SDL_Rect tip = {tx-3, ty-3, 6, 6};
        SDL_RenderFillRect(r, &tip);
    }
}

static bool are_bodies_connected(Body* a, Body* b, Joint* joints, int jointCount) {
    if (!joints) return false;
    for (int i = 0; i < jointCount; i++) {
        if (!joints[i].collideConnected) {
            if ((joints[i].bodyA == a && joints[i].bodyB == b) ||
                (joints[i].bodyA == b && joints[i].bodyB == a)) {
                return true;
            }
        }
    }
    return false;
}

// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 0 — Block Collision
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 0
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 0: Block Collision",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    Body player, obstacle;

    ShapeDef pShape = { .type = SHAPE_TYPE_RECT, .rect = {1.0f, 1.0f} };
    body_init(&player, BODY_TYPE_DYNAMIC, pShape, 2.5f, 1.5f, 2.0f);
    body_set_material(&player, MATERIAL_RUBBER);

    ShapeDef oShape = { .type = SHAPE_TYPE_RECT, .rect = {4.0f, 1.0f} };
    body_init(&obstacle, BODY_TYPE_STATIC, oShape, 8.0f, 8.5f, 0.0f);
    body_set_material(&obstacle, MATERIAL_CONCRETE);

    BroadPhase* bp = broadphase_create(BROADPHASE_BVH);
    broadphase_insert(bp, &player);
    broadphase_insert(bp, &obstacle);

    int running = 1;
    SDL_Event event;
    bool grounded = false; // Tracked from manifolds each frame

    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT) running = 0;

        const Uint8* ks = SDL_GetKeyboardState(NULL);
        if (ks[SDL_SCANCODE_LEFT])  body_apply_force(&player, (Vec2){-30.0f, 0});
        if (ks[SDL_SCANCODE_RIGHT]) body_apply_force(&player, (Vec2){ 30.0f, 0});
        // Jump: allowed when standing on anything (obstacle or world floor)
        if (ks[SDL_SCANCODE_UP] && grounded)
            player.velocity.y = -8.0f;

        // Print real-time stats before integrating (so force hasn't been cleared yet)
        printf("\r[Demo 0 Stats] Pos:(%5.2f, %5.2f) | Vel:(%5.2f, %5.2f) | Force:(%6.1f, %6.1f)    ", 
            player.position.x, player.position.y, 
            player.velocity.x, player.velocity.y, 
            player.force.x, player.force.y);
        fflush(stdout);

        body_integrate(&player, FIXED_DT, (Vec2){0, GRAVITY_MS2});
        broadphase_update(bp, &player);
        collision_resolve_world_bounds(&player);

        int pairCount = broadphase_get_pairs(bp);
        CollisionPair* pairs = broadphase_get_pair_array(bp);
        ContactManifold manifolds[64]; int manifoldCount = 0;
        for (int i = 0; i < pairCount; i++) {
            if (are_bodies_connected(pairs[i].bodyA, pairs[i].bodyB, NULL, 0)) continue;
            ContactManifold m;
            if (collision_narrow_phase(pairs[i].bodyA, pairs[i].bodyB, &m))
                if (manifoldCount < 64) manifolds[manifoldCount++] = m;
        }

        Solver solver;
        run_solver(&solver, manifolds, manifoldCount, NULL, 0);

        // Grounded if player has a contact with a mostly-vertical normal
        // (normal.y < -0.5 means normal points upward → player is on top of something)
        grounded = (player.position.y + player.shape.rect.height*0.5f >= WORLD_HEIGHT_M - 0.05f);
        for (int i = 0; i < manifoldCount && !grounded; i++) {
            bool involvesPlayer = (manifolds[i].bodyA == &player ||
                                   manifolds[i].bodyB == &player);
            // Normal pointing up (negative Y) means player is above the other body
            float ny = (manifolds[i].bodyA == &player)
                       ?  manifolds[i].normal.y   // normal A->B, positive = player pushes down
                       : -manifolds[i].normal.y;
            if (involvesPlayer && ny > 0.5f) grounded = true;
        }

        // Render
        SDL_SetRenderDrawColor(renderer, 28, 28, 35, 255); SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 70, 70, 80, 255);
        SDL_RenderDrawLine(renderer, 0, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M),
                                    800, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M));

        draw_body(renderer, &player,   255, 255, 255);
        draw_body(renderer, &obstacle, 200,  50,  50);
        draw_contacts(renderer, manifolds, manifoldCount);
        draw_force_indicator(renderer, &player);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    broadphase_destroy(bp);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 0

// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 1 — Pendulum (Revolute Joint)
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 1
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 1: Pendulum (Revolute Joint)",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    Body pivot, bob, platform;

    ShapeDef pivotShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.15f} };
    body_init(&pivot, BODY_TYPE_STATIC, pivotShape, 8.0f, 2.0f, 0.0f);

    ShapeDef bobShape = { .type = SHAPE_TYPE_RECT, .rect = {0.8f, 0.8f} };
    body_init(&bob, BODY_TYPE_DYNAMIC, bobShape, 8.0f, 5.5f, 2.0f);
    body_set_material(&bob, MATERIAL_WOOD);
    bob.velocity.x = 3.0f;

    ShapeDef platShape = { .type = SHAPE_TYPE_RECT, .rect = {6.0f, 0.5f} };
    body_init(&platform, BODY_TYPE_STATIC, platShape, 8.0f, 9.5f, 0.0f);
    body_set_material(&platform, MATERIAL_CONCRETE);

    BroadPhase* bp = broadphase_create(BROADPHASE_BVH);
    broadphase_insert(bp, &pivot);
    broadphase_insert(bp, &bob);
    broadphase_insert(bp, &platform);

    Joint joints[EP_MAX_JOINTS]; int jointCount = 0;
    joint_init_revolute(&joints[jointCount++], &pivot, &bob, pivot.position);

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT) running = 0;

        const Uint8* ks = SDL_GetKeyboardState(NULL);
        if (ks[SDL_SCANCODE_LEFT])  body_apply_torque(&bob, -15.0f);
        if (ks[SDL_SCANCODE_RIGHT]) body_apply_torque(&bob,  15.0f);

        // Print real-time stats before integrating
        printf("\r[Demo 1 Stats] Angle:%6.2f rad | AngVel:%6.2f rad/s | Torque:%6.1f Nm    ", 
            bob.orientation, bob.angularVelocity, bob.torque);
        fflush(stdout);

        body_integrate(&bob, FIXED_DT, (Vec2){0, GRAVITY_MS2});
        broadphase_update(bp, &bob);
        collision_resolve_world_bounds(&bob);

        int pairCount = broadphase_get_pairs(bp);
        CollisionPair* pairs = broadphase_get_pair_array(bp);
        ContactManifold manifolds[64]; int manifoldCount = 0;
        for (int i = 0; i < pairCount; i++) {
            if (are_bodies_connected(pairs[i].bodyA, pairs[i].bodyB, joints, jointCount)) continue;
            ContactManifold m;
            if (collision_narrow_phase(pairs[i].bodyA, pairs[i].bodyB, &m))
                if (manifoldCount < 64) manifolds[manifoldCount++] = m;
        }

        Solver solver;
        run_solver(&solver, manifolds, manifoldCount, joints, jointCount);

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 20, 28, 255); SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 70, 70, 80, 255);
        SDL_RenderDrawLine(renderer, 0, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M),
                                    800, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M));

        draw_body(renderer, &platform,  90,  90, 110);
        draw_body(renderer, &bob,       80, 160, 255);

        // Pendulum rod
        int px = (int)METERS_TO_PIXELS(pivot.position.x);
        int py = (int)METERS_TO_PIXELS(pivot.position.y);
        int bx = (int)METERS_TO_PIXELS(bob.position.x);
        int by = (int)METERS_TO_PIXELS(bob.position.y);
        SDL_SetRenderDrawColor(renderer, 180, 150, 80, 255);
        SDL_RenderDrawLine(renderer, px, py, bx, by);

        // Pivot dot
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect pivR = {px-5, py-5, 10, 10}; SDL_RenderFillRect(renderer, &pivR);

        draw_contacts(renderer, manifolds, manifoldCount);
        draw_force_indicator(renderer, &bob);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    broadphase_destroy(bp);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 1

// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 2 — Chain (5-link revolute joints)
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 2
#define CHAIN_LINKS 5
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 2: Chain",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    // One static ceiling anchor + CHAIN_LINKS dynamic links
    Body anchor;
    ShapeDef anchorShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.1f} };
    body_init(&anchor, BODY_TYPE_STATIC, anchorShape, 8.0f, 1.0f, 0.0f);

    Body links[CHAIN_LINKS];
    ShapeDef linkShape = { .type = SHAPE_TYPE_RECT, .rect = {0.3f, 0.8f} };
    for (int i = 0; i < CHAIN_LINKS; i++) {
        body_init(&links[i], BODY_TYPE_DYNAMIC, linkShape,
                  8.0f, 1.0f + (i + 1) * 0.9f, 1.5f);
        body_set_material(&links[i], MATERIAL_STEEL);
    }
    // Give the bottom link a push
    links[CHAIN_LINKS - 1].velocity.x = 4.0f;

    BroadPhase* bp = broadphase_create(BROADPHASE_BVH);
    broadphase_insert(bp, &anchor);
    for (int i = 0; i < CHAIN_LINKS; i++)
        broadphase_insert(bp, &links[i]);

    Joint joints[EP_MAX_JOINTS]; int jointCount = 0;
    // anchor → link[0]
    joint_init_revolute(&joints[jointCount++], &anchor, &links[0],
                         (Vec2){anchor.position.x, anchor.position.y});
    // link[i] → link[i+1]
    for (int i = 0; i < CHAIN_LINKS - 1; i++) {
        Vec2 connPt = {links[i].position.x, links[i].position.y + 0.4f};
        joint_init_revolute(&joints[jointCount++], &links[i], &links[i+1], connPt);
    }

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT) running = 0;

        for (int i = 0; i < CHAIN_LINKS; i++) {
            body_integrate(&links[i], FIXED_DT, (Vec2){0, GRAVITY_MS2});
            broadphase_update(bp, &links[i]);
            collision_resolve_world_bounds(&links[i]);
        }

        int pairCount = broadphase_get_pairs(bp);
        CollisionPair* pairs = broadphase_get_pair_array(bp);
        ContactManifold manifolds[64]; int manifoldCount = 0;
        for (int i = 0; i < pairCount; i++) {
            if (are_bodies_connected(pairs[i].bodyA, pairs[i].bodyB, joints, jointCount)) continue;
            ContactManifold m;
            if (collision_narrow_phase(pairs[i].bodyA, pairs[i].bodyB, &m))
                if (manifoldCount < 64) manifolds[manifoldCount++] = m;
        }

        Solver solver;
        run_solver(&solver, manifolds, manifoldCount, joints, jointCount);

        // Render
        SDL_SetRenderDrawColor(renderer, 18, 18, 24, 255); SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 70, 70, 80, 255);
        SDL_RenderDrawLine(renderer, 0, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M),
                                    800, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M));

        for (int i = 0; i < CHAIN_LINKS; i++) {
            // Rod to next body
            int ax = (int)METERS_TO_PIXELS(i == 0 ? anchor.position.x : links[i-1].position.x);
            int ay = (int)METERS_TO_PIXELS(i == 0 ? anchor.position.y : links[i-1].position.y);
            int bx = (int)METERS_TO_PIXELS(links[i].position.x);
            int by = (int)METERS_TO_PIXELS(links[i].position.y);
            SDL_SetRenderDrawColor(renderer, 160, 130, 60, 255);
            SDL_RenderDrawLine(renderer, ax, ay, bx, by);
            draw_body(renderer, &links[i], 120, 190, 255);
        }

        // Anchor dot
        int apx = (int)METERS_TO_PIXELS(anchor.position.x);
        int apy = (int)METERS_TO_PIXELS(anchor.position.y);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect ar = {apx-5, apy-5, 10, 10}; SDL_RenderFillRect(renderer, &ar);

        draw_contacts(renderer, manifolds, manifoldCount);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    broadphase_destroy(bp);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 2