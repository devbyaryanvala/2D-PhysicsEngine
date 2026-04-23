#include "ep/body.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Helper to compute polygon area and inertia
static void compute_polygon_mass_properties(const ShapeDef* shape, float density, float* outMass, float* outInertia) {
    float area = 0.0f;
    float inertia = 0.0f;
    const int count = shape->polygon.vertexCount;
    
    for (int i = 0; i < count; ++i) {
        Vec2 p1 = shape->polygon.vertices[i];
        Vec2 p2 = shape->polygon.vertices[(i + 1) % count];
        
        float D = vec2_cross(p1, p2);
        float triangleArea = 0.5f * D;
        
        area += triangleArea;
        
        float intx2 = p1.x * p1.x + p1.x * p2.x + p2.x * p2.x;
        float inty2 = p1.y * p1.y + p1.y * p2.y + p2.y * p2.y;
        inertia += (0.25f * 1.0f / 3.0f) * D * (intx2 + inty2);
    }
    
    *outMass = density * area;
    *outInertia = density * inertia;
}

// Simple physics body implementation for the 2D physics engine
void body_init(Body* b, BodyType type, ShapeDef shape, float x, float y, float density) {
    b->type = type;
    b->shape = shape;

    b->position = (Vec2){x, y};
    b->velocity = (Vec2){0, 0};
    b->acceleration = (Vec2){0, 0};
    b->force = (Vec2){0, 0};

    b->orientation = 0; 
    b->angularVelocity = 0; 
    b->angularAcceleration = 0;
    b->torque = 0;

    b->gravityScale = (Vec2){0, 1.0f};
    b->useRK4 = false;

    b->restitution = 0.5f; 
    b->staticFriction = 0.6f;
    b->dynamicFriction = 0.4f;
    b->linearDamping = 0.0f;
    b->angularDamping = 0.01f;

    if (type == BODY_TYPE_STATIC || type == BODY_TYPE_KINEMATIC) {
        b->mass = 0.0f;
        b->invMass = 0.0f;
        b->inertia = 0.0f;
        b->invInertia = 0.0f;
    } else {
        if (shape.type == SHAPE_TYPE_CIRCLE) {
            float r = shape.circle.radius;
            b->mass = M_PI * r * r * density;
            b->inertia = 0.5f * b->mass * r * r;
        } else if (shape.type == SHAPE_TYPE_RECT) {
            float w = shape.rect.width;
            float h = shape.rect.height;
            b->mass = w * h * density;
            b->inertia = (1.0f / 12.0f) * b->mass * (w * w + h * h);
        } else if (shape.type == SHAPE_TYPE_POLYGON) {
            compute_polygon_mass_properties(&shape, density, &b->mass, &b->inertia);
        }

        b->invMass = (b->mass > 0) ? 1.0f / b->mass : 0.0f;
        b->invInertia = (b->inertia > 0) ? 1.0f / b->inertia : 0.0f;
    }
}

// Applies a force to the body
void body_apply_force(Body* b, Vec2 f) {
    if (b->type != BODY_TYPE_DYNAMIC) return;
    b->force = vec2_add(b->force, f); 
}

// Applies a torque to the body
void body_apply_torque(Body* b, float t) {
    if (b->type != BODY_TYPE_DYNAMIC) return;
    b->torque += t;
}

// State struct for RK4
typedef struct {
    Vec2 pos;
    Vec2 vel;
    float angle;
    float angVel;
} State;

typedef struct {
    Vec2 dx;
    Vec2 dv;
    float da;
    float dw;
} Derivative;

static Derivative evaluate(const Body* b, const State* initial, float dt, const Derivative* d, Vec2 worldGravity) {
    State state;
    state.pos = vec2_add(initial->pos, vec2_scale(d->dx, dt));
    state.vel = vec2_add(initial->vel, vec2_scale(d->dv, dt));
    state.angle = initial->angle + d->da * dt;
    state.angVel = initial->angVel + d->dw * dt;

    Derivative output;
    output.dx = state.vel;
    
    Vec2 gravForce = (Vec2){worldGravity.x * b->gravityScale.x * b->mass, worldGravity.y * b->gravityScale.y * b->mass};
    Vec2 totalForce = vec2_add(b->force, gravForce);
    totalForce = vec2_sub(totalForce, vec2_scale(state.vel, b->linearDamping));
    
    output.dv = vec2_scale(totalForce, b->invMass);
    
    output.da = state.angVel;
    float totalTorque = b->torque - state.angVel * b->angularDamping;
    output.dw = totalTorque * b->invInertia;
    
    return output;
}

// Integrates the body's position and velocity over time
void body_integrate(Body* b, float dt, Vec2 worldGravity) {
    if (b->type == BODY_TYPE_STATIC) {
        b->velocity = (Vec2){0,0};
        b->angularVelocity = 0;
        return;
    }

    if (b->type == BODY_TYPE_KINEMATIC) {
        b->position = vec2_add(b->position, vec2_scale(b->velocity, dt));
        b->orientation += b->angularVelocity * dt;
        return;
    }

    // Dynamic Body Integration
    if (b->useRK4) {
        State state = {b->position, b->velocity, b->orientation, b->angularVelocity};
        Derivative zero = {(Vec2){0,0}, (Vec2){0,0}, 0.0f, 0.0f};

        Derivative a = evaluate(b, &state, 0.0f, &zero, worldGravity);
        Derivative b_deriv = evaluate(b, &state, dt*0.5f, &a, worldGravity);
        Derivative c = evaluate(b, &state, dt*0.5f, &b_deriv, worldGravity);
        Derivative d = evaluate(b, &state, dt, &c, worldGravity);

        Vec2 dxdt = vec2_scale(vec2_add(a.dx, vec2_add(vec2_scale(vec2_add(b_deriv.dx, c.dx), 2.0f), d.dx)), 1.0f/6.0f);
        Vec2 dvdt = vec2_scale(vec2_add(a.dv, vec2_add(vec2_scale(vec2_add(b_deriv.dv, c.dv), 2.0f), d.dv)), 1.0f/6.0f);
        float dadt = 1.0f/6.0f * (a.da + 2.0f*(b_deriv.da + c.da) + d.da);
        float dwdt = 1.0f/6.0f * (a.dw + 2.0f*(b_deriv.dw + c.dw) + d.dw);

        b->position = vec2_add(b->position, vec2_scale(dxdt, dt));
        b->velocity = vec2_add(b->velocity, vec2_scale(dvdt, dt));
        b->orientation += dadt * dt;
        b->angularVelocity += dwdt * dt;
        
        b->acceleration = dvdt;
        b->angularAcceleration = dwdt;

    } else {
        // Semi-implicit (symplectic) Euler
        Vec2 gravForce = (Vec2){worldGravity.x * b->gravityScale.x * b->mass, worldGravity.y * b->gravityScale.y * b->mass};
        Vec2 totalForce = vec2_add(b->force, gravForce);
        totalForce = vec2_sub(totalForce, vec2_scale(b->velocity, b->linearDamping));
        
        b->acceleration = vec2_scale(totalForce, b->invMass);
        b->velocity = vec2_add(b->velocity, vec2_scale(b->acceleration, dt));
        b->position = vec2_add(b->position, vec2_scale(b->velocity, dt));

        float totalTorque = b->torque - b->angularVelocity * b->angularDamping;
        b->angularAcceleration = totalTorque * b->invInertia;
        b->angularVelocity += b->angularAcceleration * dt;
        b->orientation += b->angularVelocity * dt;
    }

    // Reset force and torque accumulators
    b->force = (Vec2){0, 0}; 
    b->torque = 0;
}