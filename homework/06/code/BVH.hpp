//
// Created by LEI XU on 5/16/19.
//

#ifndef RAYTRACING_BVH_H
#define RAYTRACING_BVH_H

#include <atomic>
#include <vector>
#include <memory>
#include <ctime>
#include <unordered_map>
#include "Object.hpp"
#include "Ray.hpp"
#include "Bounds3.hpp"
#include "Intersection.hpp"
#include "Vector.hpp"

struct BVHBuildNode;
// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;

// BVHAccel Declarations
inline int leafNodes, totalLeafNodes, totalPrimitives, interiorNodes;
class BVHAccel
{

public:
    // BVHAccel Public Types
    enum class SplitMethod
    {
        NAIVE,
        SAH
    };

    // BVHAccel Public Methods
    BVHAccel(std::vector<Object *> p, int maxPrimsInNode = 1, SplitMethod splitMethod = SplitMethod::NAIVE,
             int costTraversal = 1, int costIntersection = 1);
    Bounds3 WorldBound() const;
    ~BVHAccel();

    Intersection Intersect(const Ray &ray) const;
    Intersection getIntersection(BVHBuildNode *node, const Ray &ray) const;
    bool IntersectP(const Ray &ray) const;
    BVHBuildNode *root;

    // BVHAccel Private Methods
    BVHBuildNode *recursiveBuild(std::vector<Object *> objects);
    Bounds3 &getObjectBounds(Object *obj);

    // BVHAccel Private Data
    const int maxPrimsInNode;
    const SplitMethod splitMethod;
    std::vector<Object *> primitives;
    std::unordered_map<Object *, Bounds3> boundsCache;
    const int costTraversal, costIntersection;
};

struct BVHBuildNode
{
    Bounds3 bounds;
    BVHBuildNode *left;
    BVHBuildNode *right;
    std::vector<Object *> objects;

public:
    int splitAxis = 0, firstPrimOffset = 0, nPrimitives = 0;
    // BVHBuildNode Public Methods
    BVHBuildNode()
    {
        bounds = Bounds3();
        left = nullptr;
        right = nullptr;
        // objects = nullptr;
    }
};

#endif // RAYTRACING_BVH_H
