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

        Body* tracking[1] = { &links[4] };
        draw_stats(renderer, world, tracking, 1);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
