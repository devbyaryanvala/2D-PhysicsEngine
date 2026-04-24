#ifndef EP_ISLAND_H
#define EP_ISLAND_H

#include "ep/body.h"
#include "ep/collision.h"
#include "ep/joint.h"
#include "ep/solver.h"

// Sleep thresholds
#define EP_SLEEP_LINEAR_TOL   0.1f   // m/s
#define EP_SLEEP_ANGULAR_TOL  0.035f // ~2 degrees/s
#define EP_TIME_TO_SLEEP      0.5f   // seconds

typedef struct {
    Body** bodies;
    int bodyCount;
    int bodyCapacity;

    ContactManifold** manifolds;
    int manifoldCount;
    int manifoldCapacity;

    Joint** joints;
    int jointCount;
    int jointCapacity;
} Island;

void island_init(Island* island, int bodyCapacity, int manifoldCapacity, int jointCapacity);
void island_destroy(Island* island);

void island_clear(Island* island);
void island_add_body(Island* island, Body* b);
void island_add_manifold(Island* island, ContactManifold* m);
void island_add_joint(Island* island, Joint* j);

void island_solve(Island* island, float dt);
void island_update_sleep(Island* island, float dt);

#endif
