#ifndef EP_COLLISION_H
#define EP_COLLISION_H

#include "body.h"

// Axis-Aligned Bounding Box
typedef struct {
    Vec2 min;
    Vec2 max;
} AABB;

// Check if two AABBs overlap
int aabb_overlap(AABB a, AABB b);

// Combine two AABBs into one
AABB aabb_union(AABB a, AABB b);

// Compute the surface area of an AABB
float aabb_area(AABB a);

// Compute AABB for a specific body based on its shape and transform
void collision_compute_aabb(const Body* b, AABB* outAABB);

int collision_check_aabb(Body* a, Body* b);
void collision_resolve_aabb(Body* a, Body* b);
void collision_resolve_world_bounds(Body* b);

#endif