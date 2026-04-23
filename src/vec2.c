#include "ep/vec2.h"
#include <math.h>

// Simple 2D vector operations for physics calculations
Vec2  vec2_add(Vec2 a, Vec2 b) { 
    return (Vec2){a.x + b.x, a.y + b.y}; 
}
// Subtracts two vectors
Vec2  vec2_sub(Vec2 a, Vec2 b) { 
    return (Vec2){a.x - b.x, a.y - b.y}; 
}

// Scales a vector by a scalar
Vec2  vec2_scale(Vec2 v, float s) { 
    return (Vec2){v.x * s, v.y * s}; 
}

// Dot product of two vectors
float vec2_dot(Vec2 a, Vec2 b) { 
    return a.x * b.x + a.y * b.y; 
}

// Length squared of a vector (avoids sqrt for efficiency)
float vec2_length_sq(Vec2 v) { 
    return v.x * v.x + v.y * v.y; 
}

// Length of a vector
float vec2_length(Vec2 v) { 
    return sqrtf(vec2_length_sq(v)); 
}

// Normalizes a vector to have a length of 1 (returns zero vector if length is zero)
Vec2  vec2_normalize(Vec2 v) {
    float len = vec2_length(v);
    if (len > 0){
        return vec2_scale(v, 1.0f / len);
    } else {
        return (Vec2){0,0};
    }
}

// 2D Cross product of two vectors (returns scalar z-value in 3D)
float vec2_cross(Vec2 a, Vec2 b) {
    return a.x * b.y - a.y * b.x;
}

// Cross product of vector and scalar (returns vector)
Vec2 vec2_cross_vs(Vec2 v, float s) {
    return (Vec2){s * v.y, -s * v.x};
}

// Cross product of scalar and vector (returns vector)
Vec2 vec2_cross_sv(float s, Vec2 v) {
    return (Vec2){-s * v.y, s * v.x};
}