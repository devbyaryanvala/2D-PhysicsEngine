#include "ep/material.h"
#include <math.h>

// Real-world approximate material properties
// { density (kg/m^3), restitution, staticFriction, dynamicFriction, rollingFriction }
static const MaterialProps s_materials[] = {
    /* DEFAULT   */ { 1.0f,    0.4f,  0.5f,  0.3f,  0.001f },
    /* STEEL     */ { 7850.0f, 0.6f,  0.74f, 0.57f, 0.002f },
    /* WOOD      */ { 700.0f,  0.3f,  0.6f,  0.4f,  0.005f },
    /* RUBBER    */ { 1100.0f, 0.85f, 1.0f,  0.8f,  0.02f  },
    /* ICE       */ { 917.0f,  0.1f,  0.03f, 0.01f, 0.001f },
    /* CONCRETE  */ { 2300.0f, 0.2f,  0.9f,  0.7f,  0.01f  },
    /* GLASS     */ { 2500.0f, 0.65f, 0.9f,  0.4f,  0.001f },
    /* ROCK      */ { 2600.0f, 0.2f,  0.8f,  0.6f,  0.015f },
    /* CUSTOM    */ { 1.0f,    0.5f,  0.5f,  0.3f,  0.001f },
};

const MaterialProps* material_get(MaterialType type) {
    if (type < 0 || type >= MATERIAL_CUSTOM) type = MATERIAL_DEFAULT;
    return &s_materials[type];
}

MaterialProps material_blend(const MaterialProps* a, const MaterialProps* b) {
    MaterialProps result;
    // Geometric mean: widely used for friction blending (matches Box2D/Bullet convention)
    result.density         = (a->density + b->density) * 0.5f;
    result.restitution     = (a->restitution < b->restitution) ? a->restitution : b->restitution;
    result.staticFriction  = sqrtf(a->staticFriction  * b->staticFriction);
    result.dynamicFriction = sqrtf(a->dynamicFriction * b->dynamicFriction);
    result.rollingFriction = sqrtf(a->rollingFriction * b->rollingFriction);
    return result;
}
