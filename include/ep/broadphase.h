#ifndef EP_BROADPHASE_H
#define EP_BROADPHASE_H

#include "body.h"
#include "collision.h"

typedef enum {
    BROADPHASE_BVH,
    BROADPHASE_GRID
} BroadPhaseType;

typedef struct {
    Body* bodyA;
    Body* bodyB;
} CollisionPair;

// Forward declaration of BroadPhase structure
typedef struct BroadPhase BroadPhase;

BroadPhase* broadphase_create(BroadPhaseType type);
void broadphase_destroy(BroadPhase* bp);

// Add a body to the broadphase
void broadphase_insert(BroadPhase* bp, Body* b);

// Remove a body from the broadphase
void broadphase_remove(BroadPhase* bp, Body* b);

// Update body's position in the broadphase (handles fat AABBs)
void broadphase_update(BroadPhase* bp, Body* b);

// Query all potential collision pairs
int broadphase_get_pairs(BroadPhase* bp);
CollisionPair* broadphase_get_pair_array(BroadPhase* bp);

#endif
