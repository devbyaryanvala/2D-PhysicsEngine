#include <SDL2/SDL.h>
#include <math.h>
#include "../include/body.h"

// AABB collision check
int checkCollision(Body* a, Body* b) {
    return (
        a->position.x < b->position.x + b->width &&
        a->position.x + a->width > b->position.x &&
        a->position.y < b->position.y + b->height &&
        a->position.y + a->height > b->position.y
    );
}

// Simple collision resolution
void resolveCollision(Body* a, Body* b) {
    float overlapX = (a->position.x + a->width / 2) - (b->position.x + b->width / 2);
    float overlapY = (a->position.y + a->height / 2) - (b->position.y + b->height / 2);

    float halfWidth = (a->width + b->width) / 2;
    float halfHeight = (a->height + b->height) / 2;

    float dx = halfWidth - fabs(overlapX);
    float dy = halfHeight - fabs(overlapY);

    if (dx < dy) {
        if (overlapX > 0) {
            a->position.x += dx;
        } else {
            a->position.x -= dx;
        }
        a->velocity.x *= -a->restitution;
    } else {
        if (overlapY > 0) {
            a->position.y += dy;
        } else {
            a->position.y -= dy;
        }
        a->velocity.y *= -a->restitution;
    }
}

int main(int argc, char *argv[]) {
    SDL_Init(SDL_INIT_VIDEO);

    SDL_Window* window = SDL_CreateWindow(
        "2D Physics Engine",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        800, 600,
        0
    );

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    int running = 1;
    SDL_Event event;

    // Body 1 (player)
    Body b1 = {
        .position = {300, 100},
        .velocity = {0, 0},
        .force = {0, 0},
        .mass = 1.0f,
        .invMass = 1.0f,
        .restitution = 0.5f,
        .friction = 0.8f,
        .width = 50,
        .height = 50
    };

    // Body 2 (static-ish box)
    Body b2 = {
        .position = {400, 400},
        .velocity = {0, 0},
        .force = {0, 0},
        .mass = 0.0f,
        .invMass = 0.0f,
        .restitution = 0.3f,
        .friction = 0.8f,
        .width = 80,
        .height = 80
    };

    Vec2 gravity = {0, 98.0f};
    float dt = 1.0f / 60.0f;

    while (running) {

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT)
                running = 0;
        }

        // Input (force-based)
        const Uint8* keystate = SDL_GetKeyboardState(NULL);
        Vec2 moveForce = {0, 0};

        if (keystate[SDL_SCANCODE_LEFT]) moveForce.x = -500;
        if (keystate[SDL_SCANCODE_RIGHT]) moveForce.x = 500;

        body_apply_force(&b1, moveForce);

        // Physics
        body_apply_force(&b1, gravity);
        body_integrate(&b1, dt);

        // Ground
        float ground = 550;
        if (b1.position.y + b1.height >= ground) {
            b1.position.y = ground - b1.height;
            b1.velocity.y *= -b1.restitution;

            if (fabs(b1.velocity.y) < 1.0f)
                b1.velocity.y = 0;

            // friction
            b1.velocity.x *= 0.9f;
        }

        // Box collision
        if (checkCollision(&b1, &b2)) {
            resolveCollision(&b1, &b2);
        }

        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_Rect rect1 = {
            (int)b1.position.x,
            (int)b1.position.y,
            (int)b1.width,
            (int)b1.height
        };

        SDL_Rect rect2 = {
            (int)b2.position.x,
            (int)b2.position.y,
            (int)b2.width,
            (int)b2.height
        };

        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
        SDL_RenderFillRect(renderer, &rect1);

        SDL_SetRenderDrawColor(renderer, 200, 0, 0, 255);
        SDL_RenderFillRect(renderer, &rect2);

        SDL_RenderPresent(renderer);

        SDL_Delay(16);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}