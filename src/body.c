#include "../include/body.h"

void body_apply_force(Body* b, Vec2 force) {
    b->force = vec2_add(b->force, force);
}

void body_integrate(Body* b, float dt) {
    if (b->invMass == 0) return;

    Vec2 acceleration = vec2_scale(b->force, b->invMass);

    b->velocity = vec2_add(b->velocity, vec2_scale(acceleration, dt));
    b->position = vec2_add(b->position, vec2_scale(b->velocity, dt));

    b->force = (Vec2){0, 0};
}