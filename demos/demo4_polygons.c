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
#include "ep/render.h"

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

        Body* tracking[2] = { &polys[0], &player };
        draw_stats(renderer, world, tracking, 2);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
