#ifndef EP_MATERIAL_H
#define EP_MATERIAL_H

// Predefined physics materials with real-world approximate values
typedef enum {
    MATERIAL_DEFAULT = 0,
    MATERIAL_STEEL,
    MATERIAL_WOOD,
    MATERIAL_RUBBER,
    MATERIAL_ICE,
    MATERIAL_CONCRETE,
    MATERIAL_GLASS,
    MATERIAL_ROCK,
    MATERIAL_CUSTOM  // User-defined: set properties manually
} MaterialType;

typedef struct {
    float density;          // kg/m^3
    float restitution;      // Coefficient of restitution (0 = inelastic, 1 = elastic)
    float staticFriction;   // Coefficient of static friction
    float dynamicFriction;  // Coefficient of dynamic friction (kinetic)
    float rollingFriction;  // Rolling resistance coefficient
} MaterialProps;

// Returns a pointer to the property table entry for a given material
const MaterialProps* material_get(MaterialType type);

// Blends two material property sets for a contact pair using geometric-mean mixing
MaterialProps material_blend(const MaterialProps* a, const MaterialProps* b);

#endif
