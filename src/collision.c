#include "ep/collision.h"
#include "ep/config.h"
#include <math.h>

int aabb_overlap(AABB a, AABB b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x &&
            a.min.y <= b.max.y && a.max.y >= b.min.y);
}

AABB aabb_union(AABB a, AABB b) {
    AABB c;
    c.min.x = fminf(a.min.x, b.min.x);
    c.min.y = fminf(a.min.y, b.min.y);
    c.max.x = fmaxf(a.max.x, b.max.x);
    c.max.y = fmaxf(a.max.y, b.max.y);
    return c;
}

float aabb_area(AABB a) {
    float wx = a.max.x - a.min.x;
    float wy = a.max.y - a.min.y;
    return 2.0f * (wx + wy);
}

void collision_compute_aabb(const Body* b, AABB* outAABB) {
    if (b->shape.type == SHAPE_TYPE_CIRCLE) {
        float r = b->shape.circle.radius;
        outAABB->min = (Vec2){b->position.x - r, b->position.y - r};
        outAABB->max = (Vec2){b->position.x + r, b->position.y + r};
    } else if (b->shape.type == SHAPE_TYPE_RECT) {
        float hw = b->shape.rect.width * 0.5f;
        float hh = b->shape.rect.height * 0.5f;
        float c = cosf(b->orientation);
        float s = sinf(b->orientation);
        
        Vec2 pts[4] = {{-hw, -hh}, {hw, -hh}, {hw, hh}, {-hw, hh}};
        
        outAABB->min = (Vec2){1e20f, 1e20f};
        outAABB->max = (Vec2){-1e20f, -1e20f};
        
        for (int i=0; i<4; ++i) {
            float rx = pts[i].x * c - pts[i].y * s + b->position.x;
            float ry = pts[i].x * s + pts[i].y * c + b->position.y;
            outAABB->min.x = fminf(outAABB->min.x, rx);
            outAABB->min.y = fminf(outAABB->min.y, ry);
            outAABB->max.x = fmaxf(outAABB->max.x, rx);
            outAABB->max.y = fmaxf(outAABB->max.y, ry);
        }
    } else if (b->shape.type == SHAPE_TYPE_POLYGON) {
        float c = cosf(b->orientation);
        float s = sinf(b->orientation);
        outAABB->min = (Vec2){1e20f, 1e20f};
        outAABB->max = (Vec2){-1e20f, -1e20f};
        for (int i=0; i<b->shape.polygon.vertexCount; ++i) {
            Vec2 v = b->shape.polygon.vertices[i];
            float rx = v.x * c - v.y * s + b->position.x;
            float ry = v.x * s + v.y * c + b->position.y;
            outAABB->min.x = fminf(outAABB->min.x, rx);
            outAABB->min.y = fminf(outAABB->min.y, ry);
            outAABB->max.x = fmaxf(outAABB->max.x, rx);
            outAABB->max.y = fmaxf(outAABB->max.y, ry);
        }
    }
}

// Old functions updated for demo compatibility
int collision_check_aabb(Body* a, Body* b) {
    AABB boxA, boxB;
    collision_compute_aabb(a, &boxA);
    collision_compute_aabb(b, &boxB);
    return aabb_overlap(boxA, boxB);
}

void collision_resolve_aabb(Body* a, Body* b) {
    float dx = a->position.x - b->position.x;
    float dy = a->position.y - b->position.y;
    float overlapX = (a->shape.rect.width + b->shape.rect.width)/2.0f - fabsf(dx);
    float overlapY = (a->shape.rect.height + b->shape.rect.height)/2.0f - fabsf(dy);
    
    if (overlapX < overlapY) {
        a->position.x += (dx > 0) ? overlapX : -overlapX;
        a->velocity.x *= -a->restitution;
    } else {
        a->position.y += (dy > 0) ? overlapY : -overlapY;
        a->velocity.y *= -a->restitution;
    }
}

void collision_resolve_world_bounds(Body* b) {
    if (b->invMass == 0) return;
    AABB box;
    collision_compute_aabb(b, &box);
    
    if (box.max.y > WORLD_HEIGHT_M) {
        b->position.y -= (box.max.y - WORLD_HEIGHT_M);
        b->velocity.y *= -b->restitution;
        if (fabsf(b->velocity.y) < 0.5f) b->velocity.y = 0;
        b->velocity.x *= 0.9f;
    }
    if (box.min.x < 0) { 
        b->position.x -= box.min.x; 
        b->velocity.x *= -b->restitution; 
    }
    else if (box.max.x > WORLD_WIDTH_M) { 
        b->position.x -= (box.max.x - WORLD_WIDTH_M); 
        b->velocity.x *= -b->restitution; 
    }
}