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

int main(int argc, char* argv[]) {
    SDL_Init(SDL_INIT_VIDEO);
    SDL_Window* window = SDL_CreateWindow("2D Physics Engine - Demo 6: CCD",
        100, 100, 800, 600, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    World* world = world_create((Vec2){0, 0.0f}); // Zero gravity for a pure horizontal test

    // The Thin Wall
    Body thinWall;
    ShapeDef wallShape = { .type = SHAPE_TYPE_RECT, .rect = {0.2f, 6.0f} };
    body_init(&thinWall, BODY_TYPE_STATIC, wallShape, 8.0f, 6.0f, 0.0f);
    world_add_body(world, &thinWall);

    // Bullet 1: Without CCD (will tunnel through)
    Body badBullet;
    ShapeDef bulletShape = { .type = SHAPE_TYPE_CIRCLE, .circle = {0.2f} };
    body_init(&badBullet, BODY_TYPE_DYNAMIC, bulletShape, 2.0f, 4.0f, 10.0f);
    badBullet.velocity = (Vec2){150.0f, 0.0f}; // Very fast!
    world_add_body(world, &badBullet);

    // Bullet 2: With CCD (will perfectly hit)
    Body goodBullet;
    body_init(&goodBullet, BODY_TYPE_DYNAMIC, bulletShape, 2.0f, 8.0f, 10.0f);
    goodBullet.velocity = (Vec2){150.0f, 0.0f}; // Very fast!
    body_set_bullet(&goodBullet, true); // <--- CCD ON
    world_add_body(world, &goodBullet);

    int running = 1;
    SDL_Event event;
    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = 0;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_SPACE) {
                // Reset positions and fire again!
                badBullet.position = (Vec2){2.0f, 4.0f};
                badBullet.velocity = (Vec2){150.0f, 0.0f};
                
                goodBullet.position = (Vec2){2.0f, 8.0f};
                goodBullet.velocity = (Vec2){150.0f, 0.0f};
            }
        }

        world_step(world, FIXED_DT);

        // Render
        SDL_SetRenderDrawColor(renderer, 20, 25, 30, 255); SDL_RenderClear(renderer);
        
        // Draw Wall
        draw_body(renderer, &thinWall, 200, 100, 100);
        
        // Draw bad bullet (Red)
        draw_body(renderer, &badBullet, 255, 50, 50);
        
        // Draw good bullet (Green)
        draw_body(renderer, &goodBullet, 50, 255, 50);

        draw_contacts(renderer, world->manifolds, world->manifoldCount);

        Body* tracking[2] = { &badBullet, &goodBullet };
        draw_stats(renderer, world, tracking, 2);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
