#include "ep/render.h"
#include "ep/config.h"
#include <stdio.h>
#include <math.h>

// 5x7 ASCII bitmap font (Characters 32 to 127)
static const unsigned char font5x7[96][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // 32  
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // 33 !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // 34 "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // 35 #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // 36 $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // 37 %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // 38 &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // 39 '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // 40 (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // 41 )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // 42 *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // 43 +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // 44 ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // 45 -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // 46 .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // 47 /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 48 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 49 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 50 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 51 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 52 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 53 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 54 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 55 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 56 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 57 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // 58 :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // 59 ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // 60 <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // 61 =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // 62 >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // 63 ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // 64 @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 65 A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // 66 B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // 67 C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // 68 D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 69 E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 70 F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // 71 G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 72 H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 73 I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // 74 J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // 75 K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // 76 L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // 77 M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 78 N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // 79 O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // 80 P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 81 Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 82 R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // 83 S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 84 T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // 85 U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // 86 V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 87 W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // 88 X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // 89 Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 90 Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // 91 [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // 92 Backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // 93 ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // 94 ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // 95 _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // 96 `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // 97 a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // 98 b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // 99 c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // 100 d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // 101 e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // 102 f
    {0x18, 0xA4, 0xA4, 0xA4, 0x7C}, // 103 g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // 104 h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // 105 i
    {0x40, 0x80, 0x84, 0x7D, 0x00}, // 106 j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // 107 k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // 108 l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // 109 m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // 110 n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // 111 o
    {0xFC, 0x24, 0x24, 0x24, 0x18}, // 112 p
    {0x18, 0x24, 0x24, 0x18, 0xFC}, // 113 q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // 114 r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // 115 s
    {0x04, 0x3F, 0x44, 0x40, 0x20}, // 116 t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // 117 u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // 118 v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // 119 w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // 120 x
    {0x1C, 0xA0, 0xA0, 0xA0, 0x7C}, // 121 y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // 122 z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // 123 {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // 124 |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // 125 }
    {0x10, 0x08, 0x08, 0x10, 0x08}, // 126 ~
    {0x00, 0x00, 0x00, 0x00, 0x00}  // 127
};

void draw_text(SDL_Renderer* renderer, int x, int y, const char* text, SDL_Color color, int scale) {
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    int cx = x;
    while (*text) {
        if (*text == '\n') {
            y += 8 * scale;
            cx = x;
            text++;
            continue;
        }
        
        int charIdx = *text - 32;
        if (charIdx >= 0 && charIdx < 96) {
            for (int col = 0; col < 5; col++) {
                unsigned char line = font5x7[charIdx][col];
                for (int row = 0; row < 7; row++) {
                    if (line & (1 << row)) {
                        SDL_Rect r = { cx + col * scale, y + row * scale, scale, scale };
                        SDL_RenderFillRect(renderer, &r);
                    }
                }
            }
        }
        cx += 6 * scale; // 5 cols + 1 spacing
        text++;
    }
}

void draw_stats(SDL_Renderer* renderer, World* world, Body** trackingBodies, int trackCount) {
    char buffer[512];
    SDL_Color white = {255, 255, 255, 255};
    SDL_Color green = {100, 255, 100, 255};
    SDL_Color yellow = {255, 255, 100, 255};

    int sleepCount = 0;
    for (int i = 0; i < world->bodyCount; i++) {
        if (world->bodies[i]->isSleeping) sleepCount++;
    }

    snprintf(buffer, sizeof(buffer), "WORLD STATS:\nBodies: %d (%d sleeping)\nJoints: %d\nContacts: %d\nGravity: %.1f, %.1f",
             world->bodyCount, sleepCount, world->jointCount, world->manifoldCount,
             world->gravity.x, world->gravity.y);
    
    draw_text(renderer, 10, 10, buffer, white, 2);

    int yOffset = 110;
    for (int i = 0; i < trackCount; i++) {
        if (!trackingBodies[i]) continue;
        Body* b = trackingBodies[i];
        snprintf(buffer, sizeof(buffer), "BODY %d:\nVel: %.2f, %.2f\nPos: %.2f, %.2f\nForce: %.2f, %.2f\nTorque: %.2f\nAngVel: %.2f",
                 i, b->velocity.x, b->velocity.y, b->position.x, b->position.y,
                 b->force.x, b->force.y, b->torque, b->angularVelocity);
        
        draw_text(renderer, 10, yOffset, buffer, b->isSleeping ? yellow : green, 1);
        yOffset += 70;
    }
}

void draw_body(SDL_Renderer* r, const Body* b, Uint8 red, Uint8 green, Uint8 blue) {
    if (b->isSleeping) {
        red /= 2;
        green /= 2;
        blue /= 2;
    }
    
    SDL_SetRenderDrawColor(r, red, green, blue, 255);
    if (b->shape.type == SHAPE_TYPE_CIRCLE) {
        int segments = 24; 
        float angleStep = 2.0f * 3.14159265f / segments;
        float rad = b->shape.circle.radius;
        Vec2 center = b->position;
        
        for (int i = 0; i < segments; i++) {
            float a1 = b->orientation + i * angleStep;
            float a2 = b->orientation + (i + 1) * angleStep;
            Vec2 p1 = {center.x + cosf(a1) * rad, center.y + sinf(a1) * rad};
            Vec2 p2 = {center.x + cosf(a2) * rad, center.y + sinf(a2) * rad};
            SDL_RenderDrawLine(r, 
                (int)METERS_TO_PIXELS(p1.x), (int)METERS_TO_PIXELS(p1.y),
                (int)METERS_TO_PIXELS(p2.x), (int)METERS_TO_PIXELS(p2.y)
            );
        }
        
        Vec2 edge = {center.x + cosf(b->orientation) * rad, center.y + sinf(b->orientation) * rad};
        SDL_RenderDrawLine(r, 
            (int)METERS_TO_PIXELS(center.x), (int)METERS_TO_PIXELS(center.y),
            (int)METERS_TO_PIXELS(edge.x), (int)METERS_TO_PIXELS(edge.y)
        );
    } else {
        Vec2 verts[EP_MAX_POLYGON_VERTICES];
        int count;
        body_get_world_vertices(b, verts, &count);
        
        if (count > 0) {
            for (int i = 0; i < count; i++) {
                Vec2 p1 = verts[i];
                Vec2 p2 = verts[(i + 1) % count];
                
                SDL_RenderDrawLine(r, 
                    (int)METERS_TO_PIXELS(p1.x), (int)METERS_TO_PIXELS(p1.y),
                    (int)METERS_TO_PIXELS(p2.x), (int)METERS_TO_PIXELS(p2.y)
                );
            }
        }
    }
}

void draw_contacts(SDL_Renderer* r, const ContactManifold* manifolds, int count) {
    SDL_SetRenderDrawColor(r, 255, 220, 0, 255);
    for (int i = 0; i < count; i++) {
        for (int k = 0; k < manifolds[i].contactCount; k++) {
            int cx = (int)METERS_TO_PIXELS(manifolds[i].contacts[k].x);
            int cy = (int)METERS_TO_PIXELS(manifolds[i].contacts[k].y);
            SDL_Rect cr = {cx-3, cy-3, 6, 6};
            SDL_RenderFillRect(r, &cr);
            SDL_RenderDrawLine(r, cx, cy,
                cx + (int)(manifolds[i].normal.x * 20),
                cy + (int)(manifolds[i].normal.y * 20));
        }
    }
}

void draw_force_indicator(SDL_Renderer* r, const Body* b) {
    int px = (int)METERS_TO_PIXELS(b->position.x);
    int py = (int)METERS_TO_PIXELS(b->position.y);

    if (b->force.x != 0 || b->force.y != 0) {
        float scale = 0.05f; 
        int fx = (int)METERS_TO_PIXELS(b->position.x + b->force.x * scale);
        int fy = (int)METERS_TO_PIXELS(b->position.y + b->force.y * scale);
        SDL_SetRenderDrawColor(r, 255, 50, 50, 255);
        SDL_RenderDrawLine(r, px, py, fx, fy);
        SDL_Rect tip = {fx-3, fy-3, 6, 6};
        SDL_RenderFillRect(r, &tip);
    }

    if (b->torque != 0) {
        float sign = (b->torque > 0) ? 1.0f : -1.0f;
        int tx = (int)METERS_TO_PIXELS(b->position.x + sign * 0.5f);
        int ty = (int)METERS_TO_PIXELS(b->position.y - 0.5f);
        SDL_SetRenderDrawColor(r, 255, 50, 255, 255);
        SDL_RenderDrawLine(r, px, py - (int)METERS_TO_PIXELS(0.5f), tx, ty);
        SDL_Rect tip = {tx-3, ty-3, 6, 6};
        SDL_RenderFillRect(r, &tip);
    }
}
