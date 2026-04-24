#ifndef EP_WORLD_H
#define EP_WORLD_H

#include "ep/body.h"
#include "ep/joint.h"
#include "ep/broadphase.h"

#define EP_MAX_BODIES    1024
#define EP_MAX_JOINTS    256
#define EP_MAX_MANIFOLDS 1024

typedef struct {
    Body*  bodies[EP_MAX_BODIES];
    int    bodyCount;

    Joint* joints[EP_MAX_JOINTS];
    int    jointCount;

    ContactManifold manifolds[EP_MAX_MANIFOLDS];
    int             manifoldCount;

    BroadPhase* broadphase;
    Vec2 gravity;
} World;

World* world_create(Vec2 gravity);
void world_destroy(World* world);

void world_add_body(World* world, Body* body);
void world_add_joint(World* world, Joint* joint);

void world_step(World* world, float dt);

#endif
