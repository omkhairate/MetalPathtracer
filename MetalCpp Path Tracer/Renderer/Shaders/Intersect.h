#ifndef INTERSECT_H
#define INTERSECT_H

#include <metal_stdlib>
using namespace metal;

#include "Structs.h"

// Sphere intersection (your original function)
inline float sphereIntersection(thread const ray &ray, float4 sphere)
{
    const float3 sphereCenter = sphere.xyz;
    const float r = sphere.w;

    float3 originToCenter = sphereCenter - ray.origin;
    float a = length_squared(ray.direction);
    float h = dot(ray.direction, originToCenter);
    float c = length_squared(originToCenter) - r*r;

    float discriminant = h*h - a*c;

    if (discriminant < 0) return INFINITY;

    float sqrtDiscriminant = sqrt(discriminant);

    float root = (h - sqrtDiscriminant) / a;

    if (ray.min_distance > root || ray.max_distance < root)
    {
        root = (h + sqrtDiscriminant) / a;
        if (ray.max_distance < root || ray.min_distance > root)
        {
            return INFINITY;
        }
    }

    return root;
}


// Triangle intersection (Möller–Trumbore)
inline bool triangleIntersection(
    thread const ray &ray,
    float3 v0,
    float3 v1,
    float3 v2,
    thread float &tHit,
    thread float3 &barycentric)
{
    const float EPSILON = 1e-6;

    float3 edge1 = v1 - v0;
    float3 edge2 = v2 - v0;

    float3 h = cross(ray.direction, edge2);
    float a = dot(edge1, h);

    if (abs(a) < EPSILON)
        return false; // Ray parallel to triangle

    float f = 1.0 / a;
    float3 s = ray.origin - v0;
    float u = f * dot(s, h);

    if (u < 0.0 || u > 1.0)
        return false;

    float3 q = cross(s, edge1);
    float v = f * dot(ray.direction, q);

    if (v < 0.0 || u + v > 1.0)
        return false;

    float t = f * dot(edge2, q);

    if (t < ray.min_distance || t > ray.max_distance)
        return false;

    // Valid intersection
    tHit = t;
    barycentric = float3(1.0 - u - v, u, v);
    return true;
}

#endif


