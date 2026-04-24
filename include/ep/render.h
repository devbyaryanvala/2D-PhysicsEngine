#ifndef EP_RENDER_H
#define EP_RENDER_H

#include <SDL2/SDL.h>
#include "ep/world.h"

// Draws text using a built-in 5x7 bitmap font
void draw_text(SDL_Renderer* renderer, int x, int y, const char* text, SDL_Color color, int scale);

// Draws detailed physics stats to the screen
void draw_stats(SDL_Renderer* renderer, World* world, Body** trackingBodies, int trackCount);

// Draws a physics body (moved from engine.c)
void draw_body(SDL_Renderer* renderer, const Body* b, Uint8 red, Uint8 green, Uint8 blue);

// Draws collision manifolds (moved from engine.c)
void draw_contacts(SDL_Renderer* renderer, const ContactManifold* manifolds, int count);

// Draws a visual representation of force and torque on a body
void draw_force_indicator(SDL_Renderer* renderer, const Body* b);

#endif
