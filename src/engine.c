#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "ep/body.h"
#include "ep/config.h"
#include "ep/collision.h"
#include "ep/broadphase.h"
#include "ep/solver.h"
#include "ep/material.h"
#include "ep/joint.h"
#include "ep/world.h"

// ══════════════════════════════════════════════════════════════════════════════
//  DEMO SELECTOR — Change this value to switch between demos
//
//    0 = Block Collision  (WASD/arrows to move a rubber box onto a concrete slab)
//    1 = Pendulum         (Revolute joint swinging box, arrows apply torque)
//    2 = Chain            (5-link chain of revolute joints hanging from ceiling)
//    3 = Stack Sleeping   (Space to shoot projectile at stack, see bodies dim when asleep)
//    4 = Polygons         (Showcases arbitrary convex polygons colliding)
//    5 = Advanced Joints  (Motorized car and a pendulum with angular limits)
//
// ══════════════════════════════════════════════════════════════════════════════
#define ACTIVE_DEMO 5



// ── Shared helpers ─────────────────────────────────────────────────────────────


static void draw_body(SDL_Renderer* r, const Body* b,
                       Uint8 red, Uint8 green, Uint8 blue)
{
    if (b->isSleeping) {
        red /= 2;
        green /= 2;
        blue /= 2;
    }
    
    SDL_SetRenderDrawColor(r, red, green, blue, 255);
    if (b->shape.type == SHAPE_TYPE_CIRCLE) {
        int segments = 24; // Draw circle using 24 segments
        float angleStep = 2.0f * 3.14159265f / segments;
        float rad = b->shape.circle.radius;
        Vec2 center = b->position;
        
        for (int i = 0; i < segments; i++) {
            float a1 = b->orientation + i * angleStep;
            float a2 = b->orientation + (i + 1) * angleStep;
            Vec2 p1 = {center.x + cosf(a1) * rad, center.y + sinf(a1) * rad};
            Vec2 p2 = {center.x + cosf(a2) * rad, center.y + sinf(a2) * rad};
            SDL_RenderDrawLine(r, 
                (int)METERS_TO_PIXELS(p1.x), (int)METERS_TO_PIXELS(p1.y),
                (int)METERS_TO_PIXELS(p2.x), (int)METERS_TO_PIXELS(p2.y)
            );
        }
        
        // Draw a line from center to edge to show rotation
        Vec2 edge = {center.x + cosf(b->orientation) * rad, center.y + sinf(b->orientation) * rad};
        SDL_RenderDrawLine(r, 
            (int)METERS_TO_PIXELS(center.x), (int)METERS_TO_PIXELS(center.y),
            (int)METERS_TO_PIXELS(edge.x), (int)METERS_TO_PIXELS(edge.y)
        );
    } else {
        Vec2 verts[EP_MAX_POLYGON_VERTICES];
        int count;
        body_get_world_vertices(b, verts, &count);
        
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                Vec2 p1 = verts[i];
                Vec2 p2 = verts[(i + 1) % count];
                
                SDL_RenderDrawLine(r, 
                    (int)METERS_TO_PIXELS(p1.x), (int)METERS_TO_PIXELS(p1.y),
                    (int)METERS_TO_PIXELS(p2.x), (int)METERS_TO_PIXELS(p2.y)
                );
            }
        }
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

static void __attribute__((unused)) draw_force_indicator(SDL_Renderer* r, const Body* b) {
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


// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 0 — Block Collision
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 0
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 0: Block Collision",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    World* world = world_create((Vec2){0, GRAVITY_MS2});

    Body player, obstacle;

    ShapeDef pShape = { .type = SHAPE_TYPE_RECT, .rect = {1.0f, 1.0f} };
    body_init(&player, BODY_TYPE_DYNAMIC, pShape, 2.5f, 1.5f, 2.0f);
    body_set_material(&player, MATERIAL_RUBBER);
    world_add_body(world, &player);

    ShapeDef oShape = { .type = SHAPE_TYPE_RECT, .rect = {4.0f, 1.0f} };
    body_init(&obstacle, BODY_TYPE_STATIC, oShape, 8.0f, 8.5f, 0.0f);
    body_set_material(&obstacle, MATERIAL_CONCRETE);
    world_add_body(world, &obstacle);

    int running = 1;
    SDL_Event event;
    bool grounded = false; 

    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT) running = 0;

        const Uint8* ks = SDL_GetKeyboardState(NULL);
        if (ks[SDL_SCANCODE_LEFT])  body_apply_force(&player, (Vec2){-30.0f, 0});
        if (ks[SDL_SCANCODE_RIGHT]) body_apply_force(&player, (Vec2){ 30.0f, 0});
        // Jump: allowed when standing on anything (obstacle or world floor)
        if (ks[SDL_SCANCODE_UP] && grounded)
            player.velocity.y = -8.0f;

        printf("\r[Demo 0 Stats] Pos:(%5.2f, %5.2f) | Vel:(%5.2f, %5.2f) | Force:(%6.1f, %6.1f)    ", 
            player.position.x, player.position.y, 
            player.velocity.x, player.velocity.y, 
            player.force.x, player.force.y);
        fflush(stdout);

        world_step(world, FIXED_DT);

        grounded = (player.position.y + player.shape.rect.height*0.5f >= WORLD_HEIGHT_M - 0.05f);
        for (int i = 0; i < world->manifoldCount && !grounded; i++) {
            bool involvesPlayer = (world->manifolds[i].bodyA == &player ||
                                   world->manifolds[i].bodyB == &player);
            float ny = (world->manifolds[i].bodyA == &player)
                       ?  world->manifolds[i].normal.y
                       : -world->manifolds[i].normal.y;
            if (involvesPlayer && ny > 0.5f) grounded = true;
        }

        SDL_SetRenderDrawColor(renderer, 28, 28, 35, 255); SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 70, 70, 80, 255);
        SDL_RenderDrawLine(renderer, 0, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M),
                                    800, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M));

        draw_body(renderer, &player,   255, 255, 255);
        draw_body(renderer, &obstacle, 200,  50,  50);
        draw_contacts(renderer, world->manifolds, world->manifoldCount);
        draw_force_indicator(renderer, &player);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
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

    World* world = world_create((Vec2){0, GRAVITY_MS2});

    Body pivot, bob, platform;

    ShapeDef pivotShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.15f} };
    body_init(&pivot, BODY_TYPE_STATIC, pivotShape, 8.0f, 2.0f, 0.0f);
    world_add_body(world, &pivot);

    ShapeDef bobShape = { .type = SHAPE_TYPE_RECT, .rect = {0.8f, 0.8f} };
    body_init(&bob, BODY_TYPE_DYNAMIC, bobShape, 8.0f, 5.5f, 2.0f);
    body_set_material(&bob, MATERIAL_WOOD);
    bob.velocity.x = 3.0f;
    world_add_body(world, &bob);

    ShapeDef platShape = { .type = SHAPE_TYPE_RECT, .rect = {6.0f, 0.5f} };
    body_init(&platform, BODY_TYPE_STATIC, platShape, 8.0f, 9.5f, 0.0f);
    body_set_material(&platform, MATERIAL_CONCRETE);
    world_add_body(world, &platform);

    Joint j1;
    joint_init_revolute(&j1, &pivot, &bob, pivot.position);
    world_add_joint(world, &j1);

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT) running = 0;

        const Uint8* ks = SDL_GetKeyboardState(NULL);
        if (ks[SDL_SCANCODE_LEFT])  body_apply_torque(&bob, -15.0f);
        if (ks[SDL_SCANCODE_RIGHT]) body_apply_torque(&bob,  15.0f);

        printf("\r[Demo 1 Stats] Angle:%6.2f rad | AngVel:%6.2f rad/s | Torque:%6.1f Nm    ", 
            bob.orientation, bob.angularVelocity, bob.torque);
        fflush(stdout);

        world_step(world, FIXED_DT);

        SDL_SetRenderDrawColor(renderer, 20, 20, 28, 255); SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 70, 70, 80, 255);
        SDL_RenderDrawLine(renderer, 0, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M),
                                    800, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M));

        draw_body(renderer, &platform,  90,  90, 110);
        draw_body(renderer, &bob,       80, 160, 255);

        int px = (int)METERS_TO_PIXELS(pivot.position.x);
        int py = (int)METERS_TO_PIXELS(pivot.position.y);
        int bx = (int)METERS_TO_PIXELS(bob.position.x);
        int by = (int)METERS_TO_PIXELS(bob.position.y);
        SDL_SetRenderDrawColor(renderer, 180, 150, 80, 255);
        SDL_RenderDrawLine(renderer, px, py, bx, by);

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect pivR = {px-5, py-5, 10, 10}; SDL_RenderFillRect(renderer, &pivR);

        draw_contacts(renderer, world->manifolds, world->manifoldCount);
        draw_force_indicator(renderer, &bob);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
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

    World* world = world_create((Vec2){0, GRAVITY_MS2});

    // One static ceiling anchor + CHAIN_LINKS dynamic links
    Body anchor;
    ShapeDef anchorShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.1f} };
    body_init(&anchor, BODY_TYPE_STATIC, anchorShape, 8.0f, 1.0f, 0.0f);
    world_add_body(world, &anchor);

    Body links[CHAIN_LINKS];
    ShapeDef linkShape = { .type = SHAPE_TYPE_RECT, .rect = {0.3f, 0.8f} };
    for (int i = 0; i < CHAIN_LINKS; i++) {
        body_init(&links[i], BODY_TYPE_DYNAMIC, linkShape,
                  8.0f, 1.0f + (i + 1) * 0.9f, 1.5f);
        body_set_material(&links[i], MATERIAL_STEEL);
        world_add_body(world, &links[i]);
    }
    // Give the bottom link a push
    links[CHAIN_LINKS - 1].velocity.x = 4.0f;

    Joint joints[EP_MAX_JOINTS]; int jointCount = 0;
    // anchor → link[0]
    joint_init_revolute(&joints[jointCount], &anchor, &links[0],
                         (Vec2){anchor.position.x, anchor.position.y});
    world_add_joint(world, &joints[jointCount++]);

    // link[i] → link[i+1]
    for (int i = 0; i < CHAIN_LINKS - 1; i++) {
        Vec2 connPt = {links[i].position.x, links[i].position.y + 0.4f};
        joint_init_revolute(&joints[jointCount], &links[i], &links[i+1], connPt);
        world_add_joint(world, &joints[jointCount++]);
    }

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event))
            if (event.type == SDL_QUIT) running = 0;

        world_step(world, FIXED_DT);

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
            draw_force_indicator(renderer, &links[i]);
        }

        // Anchor dot
        int apx = (int)METERS_TO_PIXELS(anchor.position.x);
        int apy = (int)METERS_TO_PIXELS(anchor.position.y);
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_Rect ar = {apx-5, apy-5, 10, 10}; SDL_RenderFillRect(renderer, &ar);

        draw_contacts(renderer, world->manifolds, world->manifoldCount);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 2

// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 3 — Stack Sleeping
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 3
#define STACK_HEIGHT 10
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 3: Stack Sleeping",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    World* world = world_create((Vec2){0, GRAVITY_MS2});

    Body floor;
    ShapeDef fShape = { .type = SHAPE_TYPE_RECT, .rect = {10.0f, 1.0f} };
    body_init(&floor, BODY_TYPE_STATIC, fShape, 8.0f, 10.0f, 0.0f);
    body_set_material(&floor, MATERIAL_CONCRETE);
    world_add_body(world, &floor);

    Body boxes[STACK_HEIGHT];
    ShapeDef bShape = { .type = SHAPE_TYPE_RECT, .rect = {0.8f, 0.8f} };
    for (int i = 0; i < STACK_HEIGHT; i++) {
        body_init(&boxes[i], BODY_TYPE_DYNAMIC, bShape, 8.0f, 9.0f - i * 1.0f, 1.0f);
        body_set_material(&boxes[i], MATERIAL_WOOD);
        world_add_body(world, &boxes[i]);
    }

    Body projectile;
    ShapeDef pShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.4f} };
    body_init(&projectile, BODY_TYPE_DYNAMIC, pShape, -2.0f, 5.0f, 5.0f); // Offscreen initially
    body_set_material(&projectile, MATERIAL_STEEL);
    projectile.isSleeping = true;
    world_add_body(world, &projectile);

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = 0;
            // Press SPACE to shoot a projectile
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                projectile.position = (Vec2){ 0.0f, 5.0f };
                projectile.velocity = (Vec2){ 25.0f, 0.0f };
                projectile.angularVelocity = 0.0f;
                projectile.isSleeping = false;
            }
        }

        world_step(world, FIXED_DT);

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 25, 30, 255); SDL_RenderClear(renderer);
        
        draw_body(renderer, &floor, 80, 80, 80);
        for (int i = 0; i < STACK_HEIGHT; i++) {
            draw_body(renderer, &boxes[i], 180, 130, 60);
        }
        draw_body(renderer, &projectile, 220, 50, 50);

        draw_contacts(renderer, world->manifolds, world->manifoldCount);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 3

// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 4 — Arbitrary Convex Polygons
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 4
#define POLY_COUNT 6
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 4: Polygons",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    World* world = world_create((Vec2){0, GRAVITY_MS2});

    // Floor (standard rect)
    Body floor;
    ShapeDef fShape = { .type = SHAPE_TYPE_RECT, .rect = {12.0f, 1.0f} };
    body_init(&floor, BODY_TYPE_STATIC, fShape, 8.0f, 11.0f, 0.0f);
    body_set_material(&floor, MATERIAL_CONCRETE);
    world_add_body(world, &floor);
    
    // Sloped ramp (custom polygon)
    Body ramp;
    Vec2 rampVerts[] = {
        { -3.0f, -1.0f },
        {  3.0f, -1.0f },
        {  3.0f, 3.0f },
        { -3.0f, 1.0f }
    };
    body_init_polygon(&ramp, BODY_TYPE_STATIC, rampVerts, 4, 12.0f, 8.0f, 0.0f);
    body_set_material(&ramp, MATERIAL_WOOD);
    world_add_body(world, &ramp);

    // Dynamic polygons
    Body polys[POLY_COUNT];
    
    // 1. A Hexagon
    body_init_regular_polygon(&polys[0], BODY_TYPE_DYNAMIC, 6, 0.8f, 5.0f, 2.0f, 1.0f);
    
    // 2. A Triangle
    body_init_regular_polygon(&polys[1], BODY_TYPE_DYNAMIC, 3, 0.8f, 6.0f, 0.0f, 1.0f);
    
    // 3. A Pentagon
    body_init_regular_polygon(&polys[2], BODY_TYPE_DYNAMIC, 5, 0.7f, 5.5f, -2.0f, 1.0f);
    
    // 4. A Custom Diamond
    Vec2 diamond[] = { {-0.5f, 0.0f}, {0.0f, -1.0f}, {0.5f, 0.0f}, {0.0f, 1.0f} };
    body_init_polygon(&polys[3], BODY_TYPE_DYNAMIC, diamond, 4, 6.5f, -4.0f, 1.0f);
    
    // 5. A Rectangle (using polygon)
    body_init_regular_polygon(&polys[4], BODY_TYPE_DYNAMIC, 4, 0.8f, 4.5f, -6.0f, 1.0f);
    
    // 6. Another Hexagon
    body_init_regular_polygon(&polys[5], BODY_TYPE_DYNAMIC, 6, 0.6f, 5.5f, -8.0f, 1.0f);

    Body player;
    body_init_regular_polygon(&player, BODY_TYPE_DYNAMIC, 8, 0.8f, 2.0f, 0.0f, 2.0f);
    body_set_material(&player, MATERIAL_STEEL);
    world_add_body(world, &player);

    for(int i = 0; i < POLY_COUNT; i++) {
        body_set_material(&polys[i], MATERIAL_WOOD);
        world_add_body(world, &polys[i]);
    }

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = 0;
            // Press SPACE to jump
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                body_apply_force(&player, (Vec2){0.0f, -800.0f});
            }
        }

        const Uint8* state = SDL_GetKeyboardState(NULL);
        if (state[SDL_SCANCODE_LEFT] || state[SDL_SCANCODE_A])  body_apply_force(&player, (Vec2){-50.0f, 0.0f});
        if (state[SDL_SCANCODE_RIGHT] || state[SDL_SCANCODE_D]) body_apply_force(&player, (Vec2){50.0f, 0.0f});

        world_step(world, FIXED_DT);

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 25, 30, 255); SDL_RenderClear(renderer);
        
        draw_body(renderer, &floor, 80, 80, 80);
        draw_body(renderer, &ramp, 120, 80, 80);
        draw_body(renderer, &player, 255, 100, 100); // Draw player in red
        
        for (int i = 0; i < POLY_COUNT; i++) {
            draw_body(renderer, &polys[i], 80, 180, 80 + i * 20);
        }

        draw_contacts(renderer, world->manifolds, world->manifoldCount);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 4

// ════════════════════════════════════════════════════════════════════════════════
//  DEMO 5 — Advanced Joint Constraints (Motors & Limits)
// ════════════════════════════════════════════════════════════════════════════════
#if ACTIVE_DEMO == 5
int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window*   window   = SDL_CreateWindow("EP | Demo 5: Motors & Limits",
                                               100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    World* world = world_create((Vec2){0, GRAVITY_MS2});

    // Floor
    Body floor;
    ShapeDef fShape = { .type = SHAPE_TYPE_RECT, .rect = {16.0f, 1.0f} };
    body_init(&floor, BODY_TYPE_STATIC, fShape, 8.0f, 11.0f, 0.0f);
    body_set_material(&floor, MATERIAL_CONCRETE);
    world_add_body(world, &floor);

    // Obstacle (to drive over)
    Body box;
    ShapeDef boxShape = { .type = SHAPE_TYPE_RECT, .rect = {1.5f, 0.5f} };
    body_init(&box, BODY_TYPE_DYNAMIC, boxShape, 12.0f, 10.0f, 1.0f);
    body_set_material(&box, MATERIAL_WOOD);
    world_add_body(world, &box);

    // --- CAR WITH MOTORS ---
    Body chassis, wheel1, wheel2;
    Joint motor1, motor2;
    
    // Chassis (make it heavy so it doesn't wheelie as violently)
    ShapeDef chassisShape = { .type = SHAPE_TYPE_RECT, .rect = {3.5f, 0.5f} };
    body_init(&chassis, BODY_TYPE_DYNAMIC, chassisShape, 4.0f, 9.0f, 10.0f);
    world_add_body(world, &chassis);
    
    // Wheels (Circles)
    ShapeDef wheelShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.6f} };
    body_init(&wheel1, BODY_TYPE_DYNAMIC, wheelShape, 2.75f, 9.5f, 1.0f);
    body_init(&wheel2, BODY_TYPE_DYNAMIC, wheelShape, 5.25f, 9.5f, 1.0f);
    body_set_material(&wheel1, MATERIAL_RUBBER);
    body_set_material(&wheel2, MATERIAL_RUBBER);
    world_add_body(world, &wheel1);
    world_add_body(world, &wheel2);
    
    // Revolute Joints with Motors
    joint_init_revolute(&motor1, &chassis, &wheel1, (Vec2){2.75f, 9.5f});
    joint_revolute_enable_motor(&motor1, true);
    joint_revolute_set_motor_speed(&motor1, 0.0f);
    joint_revolute_set_max_motor_torque(&motor1, 800.0f);
    world_add_joint(world, &motor1);
    
    joint_init_revolute(&motor2, &chassis, &wheel2, (Vec2){5.25f, 9.5f});
    joint_revolute_enable_motor(&motor2, true);
    joint_revolute_set_motor_speed(&motor2, 0.0f);
    joint_revolute_set_max_motor_torque(&motor2, 800.0f);
    world_add_joint(world, &motor2);

    // --- LIMITED PENDULUM ---
    Body pendulumAnchor, pendulumBob;
    Joint limitJoint;
    
    // Anchor
    ShapeDef anchorShape = { .type = SHAPE_TYPE_RECT, .rect = {0.5f, 0.5f} };
    body_init(&pendulumAnchor, BODY_TYPE_STATIC, anchorShape, 10.0f, 2.0f, 0.0f);
    world_add_body(world, &pendulumAnchor);
    
    // Bob
    ShapeDef bobShape = { .type = SHAPE_TYPE_RECT, .rect = {0.5f, 3.0f} };
    body_init(&pendulumBob, BODY_TYPE_DYNAMIC, bobShape, 10.0f, 3.5f, 2.0f);
    pendulumBob.angularVelocity = 5.0f; // Give it an initial push so it swings!
    world_add_body(world, &pendulumBob);
    
    // Revolute Joint with Limits (can only swing -45 to +45 degrees)
    joint_init_revolute(&limitJoint, &pendulumAnchor, &pendulumBob, (Vec2){10.0f, 2.0f});
    joint_revolute_enable_limit(&limitJoint, true);
    // Lower angle: -45 deg, Upper angle: 45 deg (in radians)
    joint_revolute_set_limits(&limitJoint, -0.785f, 0.785f);
    world_add_joint(world, &limitJoint);

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = 0;
        }

        const Uint8* state = SDL_GetKeyboardState(NULL);
        // Left/Right arrows drive the car
        float targetSpeed = 0.0f;
        if (state[SDL_SCANCODE_LEFT] || state[SDL_SCANCODE_A])  targetSpeed = -8.0f;
        if (state[SDL_SCANCODE_RIGHT] || state[SDL_SCANCODE_D]) targetSpeed =  8.0f;
        
        joint_revolute_set_motor_speed(&motor1, targetSpeed);
        joint_revolute_set_motor_speed(&motor2, targetSpeed);

        world_step(world, FIXED_DT);

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 25, 30, 255); SDL_RenderClear(renderer);
        
        draw_body(renderer, &floor, 80, 80, 80);
        draw_body(renderer, &box, 130, 80, 80);
        draw_body(renderer, &chassis, 100, 100, 150);
        draw_body(renderer, &wheel1, 50, 50, 50);
        draw_body(renderer, &wheel2, 50, 50, 50);
        
        draw_body(renderer, &pendulumAnchor, 100, 150, 100);
        draw_body(renderer, &pendulumBob, 80, 200, 80);

        draw_contacts(renderer, world->manifolds, world->manifoldCount);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
#endif // ACTIVE_DEMO == 5