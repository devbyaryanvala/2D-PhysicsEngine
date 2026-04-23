#include "ep/broadphase.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define FAT_AABB_MARGIN 0.2f
#define MAX_BODIES 1024
#define MAX_PAIRS 4096
#define GRID_CELL_SIZE 4.0f
#define HASH_TABLE_SIZE 1024

// --- BVH Tree Implementation ---
typedef struct Node {
    AABB aabb;
    Body* body; 
    int parentIndex;
    int leftIndex;
    int rightIndex;
    int height;
} Node;

typedef struct GridNode {
    Body* body;
    struct GridNode* next;
} GridNode;

typedef struct {
    BroadPhaseType type;

    Body* bodies[MAX_BODIES];
    int bodyCount;

    // BVH
    Node* nodes;
    int nodeCapacity;
    int nodeCount;
    int rootIndex;
    int freeList;

    // Pairs
    CollisionPair pairs[MAX_PAIRS];
    int pairCount;

} BroadPhaseStruct;

static int bvh_allocate_node(BroadPhaseStruct* bp) {
    if (bp->freeList == -1) {
        if (bp->nodeCount == bp->nodeCapacity) {
            bp->nodeCapacity *= 2;
            bp->nodes = realloc(bp->nodes, bp->nodeCapacity * sizeof(Node));
        }
        int index = bp->nodeCount++;
        bp->nodes[index].height = 0;
        return index;
    }
    int index = bp->freeList;
    bp->freeList = bp->nodes[index].parentIndex;
    bp->nodes[index].height = 0;
    return index;
}

static void bvh_free_node(BroadPhaseStruct* bp, int index) {
    bp->nodes[index].parentIndex = bp->freeList;
    bp->nodes[index].height = -1;
    bp->freeList = index;
}

static AABB compute_fat_aabb(const Body* b) {
    AABB aabb;
    collision_compute_aabb(b, &aabb);
    aabb.min.x -= FAT_AABB_MARGIN;
    aabb.min.y -= FAT_AABB_MARGIN;
    aabb.max.x += FAT_AABB_MARGIN;
    aabb.max.y += FAT_AABB_MARGIN;
    return aabb;
}

static void bvh_insert_leaf(BroadPhaseStruct* bp, int leaf) {
    if (bp->rootIndex == -1) {
        bp->rootIndex = leaf;
        bp->nodes[leaf].parentIndex = -1;
        return;
    }

    AABB leafAABB = bp->nodes[leaf].aabb;
    int index = bp->rootIndex;

    while (bp->nodes[index].leftIndex != -1) {
        int left = bp->nodes[index].leftIndex;
        int right = bp->nodes[index].rightIndex;
        float area = aabb_area(bp->nodes[index].aabb);

        AABB combinedAABB = aabb_union(bp->nodes[index].aabb, leafAABB);
        float combinedArea = aabb_area(combinedAABB);

        float cost = 2.0f * combinedArea;
        float inheritanceCost = 2.0f * (combinedArea - area);

        float costLeft;
        if (bp->nodes[left].leftIndex == -1) {
            costLeft = aabb_area(aabb_union(leafAABB, bp->nodes[left].aabb)) + inheritanceCost;
        } else {
            float oldArea = aabb_area(bp->nodes[left].aabb);
            float newArea = aabb_area(aabb_union(leafAABB, bp->nodes[left].aabb));
            costLeft = (newArea - oldArea) + inheritanceCost;
        }

        float costRight;
        if (bp->nodes[right].leftIndex == -1) {
            costRight = aabb_area(aabb_union(leafAABB, bp->nodes[right].aabb)) + inheritanceCost;
        } else {
            float oldArea = aabb_area(bp->nodes[right].aabb);
            float newArea = aabb_area(aabb_union(leafAABB, bp->nodes[right].aabb));
            costRight = (newArea - oldArea) + inheritanceCost;
        }

        if (cost < costLeft && cost < costRight) break;

        if (costLeft < costRight) index = left;
        else index = right;
    }

    int sibling = index;
    int oldParent = bp->nodes[sibling].parentIndex;
    int newParent = bvh_allocate_node(bp);
    bp->nodes[newParent].parentIndex = oldParent;
    bp->nodes[newParent].aabb = aabb_union(leafAABB, bp->nodes[sibling].aabb);
    bp->nodes[newParent].height = bp->nodes[sibling].height + 1;

    if (oldParent != -1) {
        if (bp->nodes[oldParent].leftIndex == sibling) bp->nodes[oldParent].leftIndex = newParent;
        else bp->nodes[oldParent].rightIndex = newParent;
    } else {
        bp->rootIndex = newParent;
    }

    bp->nodes[newParent].leftIndex = sibling;
    bp->nodes[newParent].rightIndex = leaf;
    bp->nodes[sibling].parentIndex = newParent;
    bp->nodes[leaf].parentIndex = newParent;

    index = bp->nodes[leaf].parentIndex;
    while (index != -1) {
        int left = bp->nodes[index].leftIndex;
        int right = bp->nodes[index].rightIndex;
        bp->nodes[index].aabb = aabb_union(bp->nodes[left].aabb, bp->nodes[right].aabb);
        bp->nodes[index].height = 1 + (bp->nodes[left].height > bp->nodes[right].height ? bp->nodes[left].height : bp->nodes[right].height);
        index = bp->nodes[index].parentIndex;
    }
}

static void bvh_remove_leaf(BroadPhaseStruct* bp, int leaf) {
    if (leaf == bp->rootIndex) {
        bp->rootIndex = -1;
        return;
    }
    int parent = bp->nodes[leaf].parentIndex;
    int grandParent = bp->nodes[parent].parentIndex;
    int sibling;
    if (bp->nodes[parent].leftIndex == leaf) sibling = bp->nodes[parent].rightIndex;
    else sibling = bp->nodes[parent].leftIndex;

    if (grandParent != -1) {
        if (bp->nodes[grandParent].leftIndex == parent) bp->nodes[grandParent].leftIndex = sibling;
        else bp->nodes[grandParent].rightIndex = sibling;
        bp->nodes[sibling].parentIndex = grandParent;
        bvh_free_node(bp, parent);

        int index = grandParent;
        while (index != -1) {
            int left = bp->nodes[index].leftIndex;
            int right = bp->nodes[index].rightIndex;
            bp->nodes[index].aabb = aabb_union(bp->nodes[left].aabb, bp->nodes[right].aabb);
            bp->nodes[index].height = 1 + (bp->nodes[left].height > bp->nodes[right].height ? bp->nodes[left].height : bp->nodes[right].height);
            index = bp->nodes[index].parentIndex;
        }
    } else {
        bp->rootIndex = sibling;
        bp->nodes[sibling].parentIndex = -1;
        bvh_free_node(bp, parent);
    }
}

BroadPhase* broadphase_create(BroadPhaseType type) {
    BroadPhaseStruct* bp = malloc(sizeof(BroadPhaseStruct));
    bp->type = type;
    bp->bodyCount = 0;
    bp->nodeCapacity = 256;
    bp->nodes = malloc(bp->nodeCapacity * sizeof(Node));
    bp->nodeCount = 0;
    bp->rootIndex = -1;
    bp->freeList = -1;
    for(int i=0; i<bp->nodeCapacity-1; i++) bp->nodes[i].parentIndex = i + 1;
    bp->nodes[bp->nodeCapacity-1].parentIndex = -1;
    bp->freeList = 0;
    bp->pairCount = 0;
    return (BroadPhase*)bp;
}

void broadphase_destroy(BroadPhase* bp_in) {
    BroadPhaseStruct* bp = (BroadPhaseStruct*)bp_in;
    free(bp->nodes);
    free(bp);
}

void broadphase_insert(BroadPhase* bp_in, Body* b) {
    BroadPhaseStruct* bp = (BroadPhaseStruct*)bp_in;
    bp->bodies[bp->bodyCount++] = b;

    int leaf = bvh_allocate_node(bp);
    bp->nodes[leaf].aabb = compute_fat_aabb(b);
    bp->nodes[leaf].body = b;
    bp->nodes[leaf].leftIndex = -1;
    bp->nodes[leaf].rightIndex = -1;
    b->broadphaseId = leaf;

    bvh_insert_leaf(bp, leaf);
}

void broadphase_remove(BroadPhase* bp_in, Body* b) {
    BroadPhaseStruct* bp = (BroadPhaseStruct*)bp_in;
    bvh_remove_leaf(bp, b->broadphaseId);
    
    for (int i=0; i<bp->bodyCount; i++) {
        if (bp->bodies[i] == b) {
            bp->bodies[i] = bp->bodies[--bp->bodyCount];
            break;
        }
    }
}

void broadphase_update(BroadPhase* bp_in, Body* b) {
    BroadPhaseStruct* bp = (BroadPhaseStruct*)bp_in;
    int leaf = b->broadphaseId;
    AABB strictAABB;
    collision_compute_aabb(b, &strictAABB);
    AABB fatAABB = bp->nodes[leaf].aabb;

    if (strictAABB.min.x < fatAABB.min.x || strictAABB.min.y < fatAABB.min.y ||
        strictAABB.max.x > fatAABB.max.x || strictAABB.max.y > fatAABB.max.y) {
        
        bvh_remove_leaf(bp, leaf);
        bp->nodes[leaf].aabb = compute_fat_aabb(b);
        bvh_insert_leaf(bp, leaf);
    }
}

static void query_bvh(BroadPhaseStruct* bp, int nodeA, int nodeB) {
    if (nodeA == -1 || nodeB == -1) return;
    if (!aabb_overlap(bp->nodes[nodeA].aabb, bp->nodes[nodeB].aabb)) return;

    int isLeafA = (bp->nodes[nodeA].leftIndex == -1);
    int isLeafB = (bp->nodes[nodeB].leftIndex == -1);

    if (isLeafA && isLeafB) {
        Body* bA = bp->nodes[nodeA].body;
        Body* bB = bp->nodes[nodeB].body;
        
        if ((bA->type == BODY_TYPE_STATIC && bB->type == BODY_TYPE_STATIC) ||
            (bA->type == BODY_TYPE_KINEMATIC && bB->type == BODY_TYPE_STATIC) ||
            (bA->type == BODY_TYPE_STATIC && bB->type == BODY_TYPE_KINEMATIC) ||
            (bA->type == BODY_TYPE_KINEMATIC && bB->type == BODY_TYPE_KINEMATIC)) {
            return;
        }
        
        if (bp->pairCount < MAX_PAIRS) {
            bp->pairs[bp->pairCount].bodyA = bA;
            bp->pairs[bp->pairCount].bodyB = bB;
            bp->pairCount++;
        }
        return;
    }

    if (isLeafA) {
        query_bvh(bp, nodeA, bp->nodes[nodeB].leftIndex);
        query_bvh(bp, nodeA, bp->nodes[nodeB].rightIndex);
    } else if (isLeafB) {
        query_bvh(bp, bp->nodes[nodeA].leftIndex, nodeB);
        query_bvh(bp, bp->nodes[nodeA].rightIndex, nodeB);
    } else {
        query_bvh(bp, bp->nodes[nodeA].leftIndex, bp->nodes[nodeB].leftIndex);
        query_bvh(bp, bp->nodes[nodeA].leftIndex, bp->nodes[nodeB].rightIndex);
        query_bvh(bp, bp->nodes[nodeA].rightIndex, bp->nodes[nodeB].leftIndex);
        query_bvh(bp, bp->nodes[nodeA].rightIndex, bp->nodes[nodeB].rightIndex);
    }
}

static void query_bvh_self(BroadPhaseStruct* bp, int node) {
    if (node == -1) return;
    int left = bp->nodes[node].leftIndex;
    int right = bp->nodes[node].rightIndex;
    if (left == -1) return; // Leaf

    query_bvh(bp, left, right);
    query_bvh_self(bp, left);
    query_bvh_self(bp, right);
}

// Simple Spatial Hash Grid implementation
static int hash_pos(int x, int y) {
    return abs((x * 73856093) ^ (y * 19349663)) % HASH_TABLE_SIZE;
}

int broadphase_get_pairs(BroadPhase* bp_in) {
    BroadPhaseStruct* bp = (BroadPhaseStruct*)bp_in;
    bp->pairCount = 0;
    
    if (bp->type == BROADPHASE_BVH) {
        query_bvh_self(bp, bp->rootIndex);
    } else {
        // Grid fallback
        GridNode* hashTable[HASH_TABLE_SIZE] = {0};
        
        // Insert into grid
        for (int i=0; i<bp->bodyCount; i++) {
            Body* b = bp->bodies[i];
            AABB aabb;
            collision_compute_aabb(b, &aabb);
            
            int minX = (int)floorf(aabb.min.x / GRID_CELL_SIZE);
            int maxX = (int)floorf(aabb.max.x / GRID_CELL_SIZE);
            int minY = (int)floorf(aabb.min.y / GRID_CELL_SIZE);
            int maxY = (int)floorf(aabb.max.y / GRID_CELL_SIZE);
            
            for (int y = minY; y <= maxY; y++) {
                for (int x = minX; x <= maxX; x++) {
                    int h = hash_pos(x, y);
                    GridNode* node = malloc(sizeof(GridNode));
                    node->body = b;
                    node->next = hashTable[h];
                    hashTable[h] = node;
                }
            }
        }
        
        // Find pairs
        for (int h=0; h<HASH_TABLE_SIZE; h++) {
            GridNode* n1 = hashTable[h];
            while (n1) {
                GridNode* n2 = n1->next;
                while (n2) {
                    Body* bA = n1->body;
                    Body* bB = n2->body;
                    
                    if (bA < bB) { // Avoid duplicates and self
                        if (!((bA->type == BODY_TYPE_STATIC && bB->type == BODY_TYPE_STATIC) ||
                              (bA->type == BODY_TYPE_KINEMATIC && bB->type == BODY_TYPE_STATIC) ||
                              (bA->type == BODY_TYPE_STATIC && bB->type == BODY_TYPE_KINEMATIC) ||
                              (bA->type == BODY_TYPE_KINEMATIC && bB->type == BODY_TYPE_KINEMATIC))) {
                            
                            AABB a, b;
                            collision_compute_aabb(bA, &a);
                            collision_compute_aabb(bB, &b);
                            if (aabb_overlap(a, b)) {
                                // Check if already added (grid can produce duplicates)
                                int dup = 0;
                                for (int p=0; p<bp->pairCount; p++) {
                                    if ((bp->pairs[p].bodyA == bA && bp->pairs[p].bodyB == bB) ||
                                        (bp->pairs[p].bodyA == bB && bp->pairs[p].bodyB == bA)) {
                                        dup = 1; break;
                                    }
                                }
                                if (!dup && bp->pairCount < MAX_PAIRS) {
                                    bp->pairs[bp->pairCount].bodyA = bA;
                                    bp->pairs[bp->pairCount].bodyB = bB;
                                    bp->pairCount++;
                                }
                            }
                        }
                    }
                    n2 = n2->next;
                }
                n1 = n1->next;
            }
        }
        
        // Cleanup grid
        for (int h=0; h<HASH_TABLE_SIZE; h++) {
            GridNode* node = hashTable[h];
            while (node) {
                GridNode* next = node->next;
                free(node);
                node = next;
            }
        }
    }
    
    return bp->pairCount;
}

CollisionPair* broadphase_get_pair_array(BroadPhase* bp_in) {
    BroadPhaseStruct* bp = (BroadPhaseStruct*)bp_in;
    return bp->pairs;
}
