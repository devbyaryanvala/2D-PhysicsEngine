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

        Body* tracking[1] = { &bob };
        draw_stats(renderer, world, tracking, 1);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
