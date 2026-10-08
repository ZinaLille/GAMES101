//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
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
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
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
    // TO DO Implement Path Tracing Algorithm here
    Intersection inter = intersect(ray);
    if (!inter.happened) {
        return backgroundColor;
    }
    if (inter.m->hasEmission()) {
        return inter.m->getEmission();
    }

    Vector3f p = inter.coords;
    Vector3f N = inter.normal.normalized();
    Vector3f wo = -ray.direction;
    wo.normalized();
    if(dotProduct(wo, N) < 0.0f) {
        N = -N;
    }
    Vector3f L_dir(0.0f);
    Intersection lightInter;
    float pdf_light=0.0f;
    sampleLight(lightInter, pdf_light);
    if(pdf_light > EPSILON) {
        Vector3f x = lightInter.coords;
        Vector3f ws = (x - p).normalized();
        Vector3f NN = lightInter.normal.normalized();
        float d = (x - p).norm();
        Ray shadowRay(p + N * EPSILON, ws);
        Intersection shadowInter = intersect(shadowRay);
        if (!shadowInter.happened || shadowInter.distance + 2 * EPSILON > d) {
            float cos_theta = std::max(0.0f, dotProduct(ws, N));
            float cos_theta_light = std::max(0.0f, dotProduct(-ws, NN));
            if(cos_theta > 0.0f && cos_theta_light > 0.0f){
                Vector3f f_r = inter.m->eval(wo, ws, N);
                L_dir = lightInter.emit * f_r * cos_theta * cos_theta_light / (d * d * pdf_light);
            }
        }
    }
    Vector3f L_indir(0.0f);
    float P_RR = RussianRoulette;
    if(get_random_float() < P_RR) {
        Vector3f wi = inter.m->sample(wo, N);
        Ray newRay(p + N * EPSILON, wi);
        Intersection qInter = intersect(newRay);
        if (qInter.happened && !qInter.m->hasEmission()) {
            float pdf_wi = inter.m->pdf(wo, wi, N);
            if(pdf_wi > EPSILON) {
                Vector3f f_r = inter.m->eval(wo, wi, N);
                float cos_theta = std::max(0.0f, dotProduct(wi, N));
                L_indir = castRay(newRay, depth + 1) * f_r * cos_theta / (pdf_wi * P_RR);
            }
        }
    }
    return L_dir + L_indir;
}
