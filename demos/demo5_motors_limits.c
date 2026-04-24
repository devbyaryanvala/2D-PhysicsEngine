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

        Body* tracking[2] = { &chassis, &pendulumBob };
        draw_stats(renderer, world, tracking, 2);

        SDL_RenderPresent(renderer);
        SDL_Delay(16);
    }

    world_destroy(world);
    SDL_Quit();
    return 0;
}
