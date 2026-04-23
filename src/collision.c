#include "ep/collision.h"
#include "ep/config.h"
#include <math.h>

int aabb_overlap(AABB a, AABB b) {
    return (a.min.x <= b.max.x && a.max.x >= b.min.x &&
            a.min.y <= b.max.y && a.max.y >= b.min.y);
}

AABB aabb_union(AABB a, AABB b) {
    AABB c;
    c.min.x = fminf(a.min.x, b.min.x);
    c.min.y = fminf(a.min.y, b.min.y);
    c.max.x = fmaxf(a.max.x, b.max.x);
    c.max.y = fmaxf(a.max.y, b.max.y);
    return c;
}

float aabb_area(AABB a) {
    float wx = a.max.x - a.min.x;
    float wy = a.max.y - a.min.y;
    return 2.0f * (wx + wy);
}

static Vec2 transform_point(const Body* b, Vec2 p) {
    float c = cosf(b->orientation);
    float s = sinf(b->orientation);
    return (Vec2){ p.x * c - p.y * s + b->position.x,
                   p.x * s + p.y * c + b->position.y };
}

static void get_world_vertices(const Body* b, Vec2* outVerts, int* outCount) {
    if (b->shape.type == SHAPE_TYPE_RECT) {
        *outCount = 4;
        float hw = b->shape.rect.width * 0.5f;
        float hh = b->shape.rect.height * 0.5f;
        outVerts[0] = transform_point(b, (Vec2){-hw, -hh});
        outVerts[1] = transform_point(b, (Vec2){hw, -hh});
        outVerts[2] = transform_point(b, (Vec2){hw, hh});
        outVerts[3] = transform_point(b, (Vec2){-hw, hh});
    } else if (b->shape.type == SHAPE_TYPE_POLYGON) {
        *outCount = b->shape.polygon.vertexCount;
        for (int i=0; i<*outCount; i++) {
            outVerts[i] = transform_point(b, b->shape.polygon.vertices[i]);
        }
    }
}

void collision_compute_aabb(const Body* b, AABB* outAABB) {
    if (b->shape.type == SHAPE_TYPE_CIRCLE) {
        float r = b->shape.circle.radius;
        outAABB->min = (Vec2){b->position.x - r, b->position.y - r};
        outAABB->max = (Vec2){b->position.x + r, b->position.y + r};
    } else {
        Vec2 verts[EP_MAX_POLYGON_VERTICES];
        int count;
        get_world_vertices(b, verts, &count);
        outAABB->min = (Vec2){1e20f, 1e20f};
        outAABB->max = (Vec2){-1e20f, -1e20f};
        for (int i=0; i<count; ++i) {
            outAABB->min.x = fminf(outAABB->min.x, verts[i].x);
            outAABB->min.y = fminf(outAABB->min.y, verts[i].y);
            outAABB->max.x = fmaxf(outAABB->max.x, verts[i].x);
            outAABB->max.y = fmaxf(outAABB->max.y, verts[i].y);
        }
    }
}

// Circle vs Circle Fast Path
static bool narrow_circle_circle(Body* a, Body* b, ContactManifold* m) {
    Vec2 d = vec2_sub(b->position, a->position);
    float distSq = vec2_length_sq(d);
    float radii = a->shape.circle.radius + b->shape.circle.radius;
    
    if (distSq >= radii * radii) return false;
    
    float dist = sqrtf(distSq);
    m->contactCount = 1;
    if (dist == 0.0f) {
        m->normal = (Vec2){0, 1};
        m->penetration = radii;
        m->contacts[0] = a->position;
    } else {
        m->normal = vec2_scale(d, 1.0f / dist);
        m->penetration = radii - dist;
        m->contacts[0] = vec2_add(a->position, vec2_scale(m->normal, a->shape.circle.radius));
    }
    return true;
}

// --- GJK + EPA ---

typedef struct {
    Vec2 pA, pB, p;
} SupportPoint;

static Vec2 support_poly(const Vec2* verts, int count, Vec2 dir) {
    float bestDot = -1e30f;
    Vec2 bestVert = verts[0];
    for(int i=0; i<count; i++){
        float d = vec2_dot(verts[i], dir);
        if(d > bestDot){ bestDot = d; bestVert = verts[i]; }
    }
    return bestVert;
}

static Vec2 get_shape_support(const Body* b, Vec2 dir) {
    if (b->shape.type == SHAPE_TYPE_CIRCLE) {
        Vec2 ndir = vec2_normalize(dir);
        return vec2_add(b->position, vec2_scale(ndir, b->shape.circle.radius));
    } else {
        Vec2 verts[8]; int count;
        get_world_vertices(b, verts, &count);
        return support_poly(verts, count, dir);
    }
}

static SupportPoint get_support(const Body* a, const Body* b, Vec2 dir) {
    SupportPoint sp;
    sp.pA = get_shape_support(a, dir);
    sp.pB = get_shape_support(b, vec2_scale(dir, -1.0f));
    sp.p = vec2_sub(sp.pA, sp.pB);
    return sp;
}

static Vec2 triple_product(Vec2 a, Vec2 b, Vec2 c) {
    float z = vec2_cross(a, b);
    return vec2_cross_sv(z, c);
}

static bool gjk(const Body* a, const Body* b, SupportPoint* simplex, int* sCount) {
    Vec2 dir = (Vec2){1, 0};
    simplex[0] = get_support(a, b, dir);
    *sCount = 1;
    dir = vec2_scale(simplex[0].p, -1.0f);
    
    for (int iter = 0; iter < 64; iter++) {
        SupportPoint sp = get_support(a, b, dir);
        if (vec2_dot(sp.p, dir) <= 0) return false;
        
        simplex[*sCount] = sp;
        (*sCount)++;
        
        if (*sCount == 2) {
            Vec2 ab = vec2_sub(simplex[0].p, simplex[1].p);
            Vec2 ao = vec2_scale(simplex[1].p, -1.0f);
            dir = triple_product(ab, ao, ab);
            if (vec2_length_sq(dir) == 0.0f) dir = (Vec2){-ab.y, ab.x};
        } else if (*sCount == 3) {
            Vec2 ab = vec2_sub(simplex[1].p, simplex[2].p);
            Vec2 ac = vec2_sub(simplex[0].p, simplex[2].p);
            Vec2 ao = vec2_scale(simplex[2].p, -1.0f);
            
            Vec2 abPerp = (Vec2){-ab.y, ab.x};
            if (vec2_dot(abPerp, ac) > 0) abPerp = vec2_scale(abPerp, -1.0f);
            
            if (vec2_dot(abPerp, ao) > 0) {
                simplex[0] = simplex[1]; simplex[1] = simplex[2]; *sCount = 2;
                dir = abPerp;
            } else {
                Vec2 acPerp = (Vec2){-ac.y, ac.x};
                if (vec2_dot(acPerp, ab) > 0) acPerp = vec2_scale(acPerp, -1.0f);
                
                if (vec2_dot(acPerp, ao) > 0) {
                    simplex[1] = simplex[2]; *sCount = 2;
                    dir = acPerp;
                } else {
                    return true;
                }
            }
        }
    }
    return false;
}

#define EPA_TOLERANCE 0.001f
#define EPA_MAX_ITER 32

static void epa(const Body* a, const Body* b, SupportPoint* simplex, ContactManifold* m) {
    SupportPoint poly[64];
    int count = 3;
    for (int i=0; i<3; i++) poly[i] = simplex[i];
    
    if (vec2_cross(vec2_sub(poly[1].p, poly[0].p), vec2_sub(poly[2].p, poly[0].p)) < 0) {
        SupportPoint temp = poly[0]; poly[0] = poly[1]; poly[1] = temp;
    }
    
    float minDistance = 1e30f;
    Vec2 minNormal = {0,0};
    int minIndex = 0;
    
    for (int iter = 0; iter < EPA_MAX_ITER; iter++) {
        minDistance = 1e30f;
        for (int i = 0; i < count; i++) {
            int j = (i + 1) % count;
            Vec2 edge = vec2_sub(poly[j].p, poly[i].p);
            Vec2 normal = vec2_normalize((Vec2){edge.y, -edge.x});
            float dist = vec2_dot(normal, poly[i].p);
            
            if (dist < 0) { dist = -dist; normal = vec2_scale(normal, -1.0f); }
            if (dist < minDistance) { minDistance = dist; minNormal = normal; minIndex = i; }
        }
        
        SupportPoint sp = get_support(a, b, minNormal);
        float dist = vec2_dot(minNormal, sp.p);
        
        if (dist - minDistance < EPA_TOLERANCE) break;
        
        for (int i = count; i > minIndex + 1; i--) poly[i] = poly[i - 1];
        poly[minIndex + 1] = sp;
        count++;
        if (count >= 64) break;
    }
    
    m->normal = minNormal;
    m->penetration = minDistance + 0.005f; // Small slop
}

// --- Sutherland-Hodgman Clipping ---

typedef struct { Vec2 v1, v2; Vec2 maxPoint; } Edge;

static Edge best_edge(const Vec2* verts, int count, Vec2 normal) {
    float maxDot = -1e30f;
    int index = 0;
    for (int i=0; i<count; i++) {
        float d = vec2_dot(verts[i], normal);
        if (d > maxDot) { maxDot = d; index = i; }
    }
    Vec2 v = verts[index];
    Vec2 v0 = verts[(index - 1 + count) % count];
    Vec2 v1 = verts[(index + 1) % count];
    
    Vec2 leftEdge = vec2_normalize(vec2_sub(v, v0));
    Vec2 rightEdge = vec2_normalize(vec2_sub(v1, v));
    
    if (vec2_dot(rightEdge, normal) <= vec2_dot(leftEdge, normal)) return (Edge){v, v1, v};
    else return (Edge){v0, v, v};
}

static void clip_segment_to_line(Vec2 vIn[2], Vec2 vOut[2], int* outCount, Vec2 normal, float offset) {
    *outCount = 0;
    float d0 = vec2_dot(normal, vIn[0]) - offset;
    float d1 = vec2_dot(normal, vIn[1]) - offset;
    if (d0 >= 0.0f) vOut[(*outCount)++] = vIn[0];
    if (d1 >= 0.0f) vOut[(*outCount)++] = vIn[1];
    if (d0 * d1 < 0.0f) {
        float t = d0 / (d0 - d1);
        vOut[(*outCount)++] = vec2_add(vIn[0], vec2_scale(vec2_sub(vIn[1], vIn[0]), t));
    }
}

static void extract_contacts(Body* a, Body* b, ContactManifold* m) {
    if (a->shape.type == SHAPE_TYPE_CIRCLE && b->shape.type != SHAPE_TYPE_CIRCLE) {
        m->contactCount = 1;
        m->contacts[0] = vec2_add(a->position, vec2_scale(m->normal, a->shape.circle.radius));
        return;
    } else if (b->shape.type == SHAPE_TYPE_CIRCLE && a->shape.type != SHAPE_TYPE_CIRCLE) {
        m->contactCount = 1;
        m->contacts[0] = vec2_sub(b->position, vec2_scale(m->normal, b->shape.circle.radius));
        return;
    }

    Vec2 vA[8], vB[8]; int cA, cB;
    get_world_vertices(a, vA, &cA);
    get_world_vertices(b, vB, &cB);
    
    Edge eA = best_edge(vA, cA, m->normal);
    Edge eB = best_edge(vB, cB, vec2_scale(m->normal, -1.0f));
    
    Edge ref, inc; bool flip = false;
    Vec2 eAVec = vec2_normalize(vec2_sub(eA.v2, eA.v1));
    Vec2 eBVec = vec2_normalize(vec2_sub(eB.v2, eB.v1));
    
    if (fabsf(vec2_dot(eAVec, m->normal)) <= fabsf(vec2_dot(eBVec, m->normal))) {
        ref = eA; inc = eB;
    } else {
        ref = eB; inc = eA; flip = true;
    }
    
    Vec2 refv = vec2_normalize(vec2_sub(ref.v2, ref.v1));
    float o1 = vec2_dot(refv, ref.v1);
    
    Vec2 clip1[2]; int c1 = 0;
    Vec2 incVerts[2] = {inc.v1, inc.v2};
    clip_segment_to_line(incVerts, clip1, &c1, refv, o1);
    if (c1 < 2) return;
    
    float o2 = vec2_dot(refv, ref.v2);
    Vec2 clip2[2]; int c2 = 0;
    clip_segment_to_line(clip1, clip2, &c2, vec2_scale(refv, -1.0f), -o2);
    if (c2 < 2) return;
    
    Vec2 refNormal = (Vec2){-refv.y, refv.x};
    if (flip) refNormal = vec2_scale(refNormal, -1.0f);
    float maxDepth = vec2_dot(refNormal, ref.maxPoint);
    
    m->contactCount = 0;
    for (int i=0; i<c2; i++) {
        // depth < 0 means the clipped point is behind (inside) the reference face — keep it
        float depth = vec2_dot(refNormal, clip2[i]) - maxDepth;
        if (depth <= 0.001f) {  // Small slop to allow near-coplanar points
            m->contacts[m->contactCount++] = clip2[i];
            if (m->contactCount == EP_MAX_CONTACTS) break;
        }
    }
    
    if (m->contactCount == 0) {
        m->contacts[0] = inc.maxPoint;
        m->contactCount = 1;
    }
}

bool collision_narrow_phase(Body* a, Body* b, ContactManifold* m) {
    m->bodyA = a;
    m->bodyB = b;
    m->contactCount = 0;
    
    if (a->shape.type == SHAPE_TYPE_CIRCLE && b->shape.type == SHAPE_TYPE_CIRCLE) {
        return narrow_circle_circle(a, b, m);
    }
    
    SupportPoint simplex[3];
    int sCount = 0;
    if (gjk(a, b, simplex, &sCount)) {
        epa(a, b, simplex, m);
        extract_contacts(a, b, m);
        return true;
    }
    return false;
}

// Temporary resolution until Step 04 SI solver
void collision_resolve_manifold(ContactManifold* m) {
    Body* a = m->bodyA;
    Body* b = m->bodyB;
    
    float invMassSum = a->invMass + b->invMass;
    if (invMassSum == 0.0f) return;
    
    // --- Velocity Impulse ---
    Vec2 relVel = vec2_sub(b->velocity, a->velocity);
    float velAlongNormal = vec2_dot(relVel, m->normal);
    // Only resolve if bodies are approaching each other
    if (velAlongNormal > 0) goto positional_correction;
    
    {
        float e = fminf(a->restitution, b->restitution);
        float j = -(1.0f + e) * velAlongNormal / invMassSum;
        Vec2 impulse = vec2_scale(m->normal, j);
        if (a->type == BODY_TYPE_DYNAMIC) a->velocity = vec2_sub(a->velocity, vec2_scale(impulse, a->invMass));
        if (b->type == BODY_TYPE_DYNAMIC) b->velocity = vec2_add(b->velocity, vec2_scale(impulse, b->invMass));
    }
    
    positional_correction:;
    // --- Positional Correction (Baumgarte) ---
    // Move each body proportionally to its invMass fraction (lighter body moves more)
    const float percent = 0.6f;   // How aggressively to correct (0.2 - 0.8)
    const float slop = 0.02f;     // Allow a tiny penetration to avoid jitter
    float correctionMag = fmaxf(m->penetration - slop, 0.0f) * percent / invMassSum;
    Vec2 correction = vec2_scale(m->normal, correctionMag);
    if (a->type == BODY_TYPE_DYNAMIC) a->position = vec2_sub(a->position, vec2_scale(correction, a->invMass));
    if (b->type == BODY_TYPE_DYNAMIC) b->position = vec2_add(b->position, vec2_scale(correction, b->invMass));
}

void collision_resolve_world_bounds(Body* b) {
    if (b->invMass == 0) return;
    AABB box;
    collision_compute_aabb(b, &box);
    
    if (box.max.y > WORLD_HEIGHT_M) {
        b->position.y -= (box.max.y - WORLD_HEIGHT_M);
        b->velocity.y *= -b->restitution;
        if (fabsf(b->velocity.y) < 0.5f) b->velocity.y = 0;
        b->velocity.x *= 0.9f;
    }
    if (box.min.x < 0) { 
        b->position.x -= box.min.x; 
        b->velocity.x *= -b->restitution; 
    }
    else if (box.max.x > WORLD_WIDTH_M) { 
        b->position.x -= (box.max.x - WORLD_WIDTH_M); 
        b->velocity.x *= -b->restitution; 
    }
}