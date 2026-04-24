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

        Body* tracking[2] = { &projectile, &boxes[0] };
        draw_stats(renderer, world, tracking, 2);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
