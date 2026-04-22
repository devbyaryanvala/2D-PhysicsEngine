#ifndef EP_COLLISION_H
#define EP_COLLISION_H

#include "body.h"

// Collision detection and resolution functions for the physics engine
int  collision_check_aabb(Body* a, Body* b);
void collision_resolve_aabb(Body* a, Body* b);
void collision_resolve_world_bounds(Body* b);

#endif