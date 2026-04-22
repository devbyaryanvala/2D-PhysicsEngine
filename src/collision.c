#include "ep/collision.h"
#include "ep/config.h"
#include <math.h>

// AABB collision detection and resolution functions for the physics engine
int collision_check_aabb(Body* a, Body* b) {
    return (a->position.x < b->position.x + b->width && a->position.x + a->width > b->position.x &&
            a->position.y < b->position.y + b->height && a->position.y + a->height > b->position.y);
}

// Resolves collision by calculating the minimum translation vector and applying restitution
void collision_resolve_aabb(Body* a, Body* b) {
    // Calculate the overlap on both axes
    float dx = (a->position.x + a->width/2) - (b->position.x + b->width/2);
    float dy = (a->position.y + a->height/2) - (b->position.y + b->height/2);
    float overlapX = (a->width + b->width)/2 - fabsf(dx);
    float overlapY = (a->height + b->height)/2 - fabsf(dy);
    
    // Resolve along the axis of least penetration
    if (overlapX < overlapY) {
        a->position.x += (dx > 0) ? overlapX : -overlapX;
        a->velocity.x *= -a->restitution;
    } else {
        a->position.y += (dy > 0) ? overlapY : -overlapY;
        a->velocity.y *= -a->restitution;
    }
}


// Resolves collisions with world bounds (e.g., ground and walls) by clamping position and applying restitution
void collision_resolve_world_bounds(Body* b) {
    // Static bodies don't need to resolve world bounds
    if (b->invMass == 0) return;
    // Prevent going below the ground
    if (b->position.y + b->height > WORLD_HEIGHT_M) {
        b->position.y = WORLD_HEIGHT_M - b->height;
        b->velocity.y *= -b->restitution;
        if (fabsf(b->velocity.y) < 0.5f) b->velocity.y = 0;
        b->velocity.x *= 0.9f;
    }
    // Prevent going out of horizontal bounds
    if (b->position.x < 0) { b->position.x = 0; b->velocity.x *= -b->restitution; }
    else if (b->position.x + b->width > WORLD_WIDTH_M) { 
        b->position.x = WORLD_WIDTH_M - b->width; 
        b->velocity.x *= -b->restitution; 
    }
}