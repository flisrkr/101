//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"

void Scene::buildBVH()
{
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        if (objects[k]->hasEmit())
        {
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum)
            {
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
    const Ray &ray,
    const std::vector<Object *> &objects,
    float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k)
    {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear)
        {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }

    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // no intersection
    if (depth > maxDepth)
    {
        return Vector3f(0.f);
    }
    Intersection pos_shading = intersect(ray);
    if (!pos_shading.happened)
    {
        return Vector3f(0.f);
    }

    // declare variables
    // const float pdf_indir = 1.f / (2.f * M_PI);
    const float pdf_indir = 0.5f / M_PI;
    Vector3f L, L_dir, L_indir, L_emit = 0.f;
    float pdf_dir;

    // emission
    if (pos_shading.m->hasEmission())
    {
        L_emit = pos_shading.m->getEmission();
    }

    // direct illumination
    Intersection pos_light;
    sampleLight(pos_light, pdf_dir);
    Vector3f vector_light = pos_light.coords - (pos_shading.coords + pos_shading.normal * EPSILON);
    Ray ray_light(pos_shading.coords + pos_shading.normal * EPSILON, vector_light.normalized());

    if ((intersect(ray_light).coords - pos_light.coords).norm() < EPSILON)
    {
        L_dir = pos_light.emit * pos_shading.m->eval(ray_light.direction, -ray.direction, pos_shading.normal) *
                dotProduct(ray_light.direction, pos_shading.normal) * dotProduct(-ray_light.direction, pos_light.normal) /
                pow(vector_light.norm(), 2) / pdf_dir;
    }

    // indirect illumination
    if (get_random_float() < RussianRoulette)
    {
        Vector3f vector_sample = pos_shading.m->sample(-ray.direction, pos_shading.normal);
        Ray ray_sample(pos_shading.coords + pos_shading.normal * EPSILON, vector_sample.normalized());
        Intersection pos_sample = intersect(ray_sample);
        if (pos_sample.happened && !pos_sample.m->hasEmission())
        {
            L_indir = castRay(ray_sample, depth + 1) *
                      pos_shading.m->eval(ray_sample.direction, -ray.direction, pos_shading.normal) *
                      dotProduct(ray_sample.direction, pos_shading.normal) /
                      pos_shading.m->pdf(ray_sample.direction, -ray.direction, pos_shading.normal);
        }
    }

    L = L_dir + L_indir + L_emit;
    return L;
}