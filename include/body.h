#ifndef BODY_H
#define BODY_H

#include "vec2.h"

typedef struct {
    Vec2 position;
    Vec2 velocity;
    Vec2 force;

    float mass;
    float invMass;

    float restitution;
    float friction;

    float width;
    float height;

} Body;

void body_apply_force(Body* b, Vec2 force);
void body_integrate(Body* b, float dt);

#endif