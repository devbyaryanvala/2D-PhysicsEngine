#ifndef EP_VEC2_H
#define EP_VEC2_H

// 2D vector math utilities for the physics engine, providing basic operations like addition, subtraction, scaling, and normalization
typedef struct { float x, y; } Vec2;

// Function prototypes for vector operations
Vec2  vec2_add(Vec2 a, Vec2 b);
Vec2  vec2_sub(Vec2 a, Vec2 b);
Vec2  vec2_scale(Vec2 v, float s);
float vec2_dot(Vec2 a, Vec2 b);
float vec2_length_sq(Vec2 v);
float vec2_length(Vec2 v);
Vec2  vec2_normalize(Vec2 v);

#endif