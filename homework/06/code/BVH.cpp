#include <algorithm>
#include <cassert>
#include "BVH.hpp"

BVHAccel::BVHAccel(std::vector<Object *> p, int maxPrimsInNode,
                   SplitMethod splitMethod, int costTraversal, int costIntersection)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p)), costTraversal(costTraversal), costIntersection(costIntersection)
{
    time_t start, stop;
    time(&start);
    if (primitives.empty())
        return;

    root = recursiveBuild(primitives);

    time(&stop);
    double diff = difftime(stop, start);
    int hrs = (int)diff / 3600;
    int mins = ((int)diff / 60) - (hrs * 60);
    int secs = (int)diff - (hrs * 3600) - (mins * 60);

    printf(
        "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
        hrs, mins, secs);
}

Bounds3 &BVHAccel::getObjectBounds(Object *obj)
{
    auto it = boundsCache.find(obj);
    if (it == boundsCache.end())
    {
        Bounds3 bounds_to_cache = obj->getBounds();
        boundsCache[obj] = bounds_to_cache;
        return boundsCache[obj];
    }
    return it->second;
}

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, getObjectBounds(objects[i]));
    if (objects.size() <= this->maxPrimsInNode)
    {
        // Create leaf _BVHBuildNode_
        node->bounds = bounds;
        node->objects = objects;
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});
        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else if (splitMethod == SplitMethod::NAIVE)
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, getObjectBounds(objects[i]).Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [this](auto f1, auto f2)
                      { return this->getObjectBounds(f1).Centroid().x <
                               this->getObjectBounds(f2).Centroid().x; });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [this](auto f1, auto f2)
                      { return this->getObjectBounds(f1).Centroid().y <
                               this->getObjectBounds(f2).Centroid().y; });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [this](auto f1, auto f2)
                      { return this->getObjectBounds(f1).Centroid().z <
                               this->getObjectBounds(f2).Centroid().z; });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }
    else if (splitMethod == SplitMethod::SAH)
    {
        double area_parent = bounds.SurfaceArea();
        double cost_min = std::numeric_limits<float>::infinity();
        int axis_best, index_best;
        for (int i = 0; i < 3; i++)
        {
            std::sort(objects.begin(), objects.end(), [this, i](auto f1, auto f2)
                      { return this->getObjectBounds(f1).Centroid()[i] <
                               this->getObjectBounds(f2).Centroid()[i]; });
            Bounds3 bounds_left;
            for (int j = 0; j < objects.size() - 1; j++)
            {
                bounds_left = Union(bounds_left, getObjectBounds(objects[j]));
                Bounds3 bounds_right;
                for (int k = j + 1; k < objects.size(); k++)
                {
                    bounds_right = Union(bounds_right, getObjectBounds(objects[k]));
                }
                double pl = bounds_left.SurfaceArea() / area_parent, pr = bounds_right.SurfaceArea() / area_parent;
                int nl = j + 1.f, nr = objects.size() - j - 1.f;
                double cost_tmp = costTraversal + pl * nl * costIntersection + pr * nr * costIntersection;
                if (cost_tmp < cost_min)
                {
                    cost_min = cost_tmp;
                    axis_best = i;
                    index_best = j;
                }
            }
        }
        double cost_leaf = objects.size() * costIntersection;
        if (cost_leaf <= cost_min)
        {
            node->bounds = bounds;
            node->objects = objects;
            node->left = nullptr;
            node->right = nullptr;
            return node;
        }
        else
        {
            std::sort(objects.begin(), objects.end(), [this, axis_best](auto f1, auto f2)
                      { return this->getObjectBounds(f1).Centroid()[axis_best] <
                               this->getObjectBounds(f2).Centroid()[axis_best]; });
            auto objs_left = std::vector<Object *>(objects.begin(), objects.begin() + index_best + 1);
            auto objs_right = std::vector<Object *>(objects.begin() + index_best + 1, objects.end());
            node->left = recursiveBuild(objs_left);
            node->right = recursiveBuild(objs_right);
            node->bounds = Union(node->left->bounds, node->right->bounds);
        }
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray &ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode *node, const Ray &ray) const
{
    // TODO Traverse the BVH to find intersection
    std::array<int, 3> dirIsNeg = {static_cast<int>(ray.direction.x < 0),
                                   static_cast<int>(ray.direction.y < 0),
                                   static_cast<int>(ray.direction.z < 0)};
    if (!node->bounds.IntersectP(ray, ray.direction_inv, dirIsNeg))
    {
        return Intersection();
    }
    if (node->left == nullptr && node->right == nullptr)
    {
        Intersection inter_result;
        for (auto &leaf_object : node->objects)
        {
            Intersection inter_temp = leaf_object->getIntersection(ray);
            if (inter_temp.distance < inter_result.distance)
            {
                inter_result = inter_temp;
            }
        }
        return inter_result;
    }
    const Intersection &inter_left = getIntersection(node->left, ray);
    const Intersection &inter_right = getIntersection(node->right, ray);
    return inter_left.distance < inter_right.distance ? inter_left : inter_right;
}