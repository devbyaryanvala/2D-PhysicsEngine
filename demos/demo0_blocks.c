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
        
        Body* tracking[1] = { &player };
        draw_stats(renderer, world, tracking, 1);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
