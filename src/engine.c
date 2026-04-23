#include <SDL2/SDL.h>
#include "ep/body.h"
#include "ep/config.h"
#include "ep/collision.h"

// Main entry point for the physics engine demo
int main(int argc, char* argv[]) {

    // Initialize SDL and create a window and renderer
    SDL_Init(SDL_INIT_VIDEO);

    // Create a window and renderer for visualization
    SDL_Window* window = SDL_CreateWindow("Enterprise Physics", 100, 100, 800, 600, 0);

    // Create a renderer for drawing
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

    // Initialize player and obstacle bodies
    Body player, obstacle;
    ShapeDef pShape = { .type = SHAPE_TYPE_RECT, .rect = {1.0f, 1.0f} };
    body_init(&player, BODY_TYPE_DYNAMIC, pShape, 2.0f, 1.0f, 2.0f);
    ShapeDef oShape = { .type = SHAPE_TYPE_RECT, .rect = {4.0f, 1.0f} };
    body_init(&obstacle, BODY_TYPE_STATIC, oShape, 6.0f, 8.0f, 0.0f);

    // Main game loop
    int running = 1; SDL_Event event;

    // Game loop with fixed time step for consistent physics updates
    while (running) {
        // Handle events (e.g., window close)
        while (SDL_PollEvent(&event)) if (event.type == SDL_QUIT) running = 0;

        // Handle player input for movement and jumping
        const Uint8* ks = SDL_GetKeyboardState(NULL);
        if (ks[SDL_SCANCODE_LEFT]) body_apply_force(&player, (Vec2){-30.0f, 0});
        if (ks[SDL_SCANCODE_RIGHT]) body_apply_force(&player, (Vec2){30.0f, 0});
        if (ks[SDL_SCANCODE_UP] && (player.position.y + player.shape.rect.height >= WORLD_HEIGHT_M - 0.1f)) player.velocity.y = -8.0f;

        // Apply gravity, integrate physics, and resolve collisions
        body_integrate(&player, FIXED_DT, (Vec2){0, GRAVITY_MS2});
        collision_resolve_world_bounds(&player);
        if (collision_check_aabb(&player, &obstacle)) collision_resolve_aabb(&player, &obstacle);

        // Render the scene (player, obstacle, and ground)
        SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
        SDL_RenderClear(renderer);
        SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
        SDL_RenderDrawLine(renderer, 0, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M), 800, (int)METERS_TO_PIXELS(WORLD_HEIGHT_M));
        

        // Convert world coordinates to screen coordinates for rendering
        SDL_Rect pR = { (int)METERS_TO_PIXELS(player.position.x), (int)METERS_TO_PIXELS(player.position.y), (int)METERS_TO_PIXELS(player.shape.rect.width), (int)METERS_TO_PIXELS(player.shape.rect.height) };
        SDL_Rect oR = { (int)METERS_TO_PIXELS(obstacle.position.x), (int)METERS_TO_PIXELS(obstacle.position.y), (int)METERS_TO_PIXELS(obstacle.shape.rect.width), (int)METERS_TO_PIXELS(obstacle.shape.rect.height) };
        

        // Draw the player and obstacle rectangles
        SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255); SDL_RenderFillRect(renderer, &pR);
        SDL_SetRenderDrawColor(renderer, 200, 50, 50, 255); SDL_RenderFillRect(renderer, &oR);
        SDL_RenderPresent(renderer); SDL_Delay(16);
    }
    SDL_Quit(); return 0;
}