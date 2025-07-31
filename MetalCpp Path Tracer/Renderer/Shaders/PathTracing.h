#ifndef PATH_TRACING_H
#define PATH_TRACING_H

#include <metal_stdlib>
#include <metal_raytracing>

#define M_PI 3.14159265358979323846 

#include "Random.h"
#include "Structs.h"
#include "Intersect.h"
#include "Scatter.h"

using namespace metal;

template <typename T>
inline void swap(thread T& a, thread T& b)
{
    T tmp = a;
    a = b;
    b = tmp;
}


inline float3 randomUnitVector(thread uint32_t& seed)
{
    float z = 2.0 * randomFloat(seed) - 1.0;
    float t = 2.0 * M_PI * randomFloat(seed);
    float r = sqrt(1.0 - z * z);
    return float3(r * cos(t), r * sin(t), z);
}



// Helper: Random point in unit sphere
inline float3 randomInUnitSphere(thread uint32_t& seed)
{
    while (true)
    {
        float3 p = 2.0 * float3(
            randomFloat(seed),
            randomFloat(seed),
            randomFloat(seed)
        ) - float3(1.0);

        if (length_squared(p) < 1.0)
            return p;
    }
}

// BVH intersection helper
inline bool intersectAABB(
    thread const ray& r,
    float3 bmin,
    float3 bmax,
    float tMin,
    float tMax)
{
    for (int i = 0; i < 3; ++i)
    {
        float invD = 1.0 / r.direction[i];
        float t0 = (bmin[i] - r.origin[i]) * invD;
        float t1 = (bmax[i] - r.origin[i]) * invD;
        if (invD < 0.0)
            swap(t0, t1);
        tMin = max(tMin, t0);
        tMax = min(tMax, t1);
        if (tMax <= tMin)
            return false;
    }
    return true;
}

// BVH traversal version of firstHit
inline intersection firstHitBVH(
    thread const ray& ray,
    device const float4* bvhNodes,
    device const int* primitiveIndices,
    device const float4* primitives,
    uint32_t primitiveCount)
{
    intersection in;
    in.t = INFINITY;
    in.sphereId = -1;
    in.triangleId = -1;

    constexpr int stackSize = 64;
    int stack[stackSize];
    int stackPtr = 0;
    stack[stackPtr++] = 0; // Root node index = 0

    while (stackPtr > 0)
    {
        int nodeIdx = stack[--stackPtr];

        float3 bmin = bvhNodes[2 * nodeIdx + 0].xyz;
        float3 bmax = bvhNodes[2 * nodeIdx + 1].xyz;
        int leftFirst = as_type<int>(bvhNodes[2 * nodeIdx + 0].w);
        int count = as_type<int>(bvhNodes[2 * nodeIdx + 1].w);

        if (!intersectAABB(ray, bmin, bmax, 0.0001, in.t))
            continue;

        if (count > 0)
        {
            for (int i = 0; i < count; ++i)
            {
                int primIdx = primitiveIndices[leftFirst + i];
                if (primIdx < 0 || primIdx >= primitiveCount)
                    continue;

                int base = primIdx * 3;
                float4 p0 = primitives[base + 0];
                float4 p1 = primitives[base + 1];
                float4 p2 = primitives[base + 2];

                int primitiveType = int(p0.w);

                if (primitiveType == 0) // Sphere
                {
                    float4 sphere = float4(p0.xyz, p1.x);
                    float root = sphereIntersection(ray, sphere);
                    if (root < in.t && root != INFINITY)
                    {
                        in.t = root;
                        in.sphereId = primIdx;
                        in.triangleId = -1;
                        in.normal = normalize((ray.origin + root * ray.direction) - p0.xyz);
                    }
                }
                else if (primitiveType == 1) // Triangle
                {
                    float tTri;
                    float3 bary;
                    if (triangleIntersection(ray, p0.xyz, p1.xyz, p2.xyz, tTri, bary))
                    {
                        if (tTri < in.t)
                        {
                            in.t = tTri;
                            in.sphereId = -1;
                            in.triangleId = primIdx;
                            in.normal = normalize(cross(p1.xyz - p0.xyz, p2.xyz - p0.xyz));
                        }
                    }
                }
            }
        }
        else
        {
            stack[stackPtr++] = leftFirst;
            stack[stackPtr++] = leftFirst + 1;
        }
    }

    if (in.t == INFINITY)
        return in;

    in.point = ray.origin + (in.t * ray.direction);

    in.frontFace = dot(in.normal, ray.direction) < 0.0;
    if (!in.frontFace)
        in.normal = -in.normal;

    return in;
}


inline float4 rayColor(
    ray ray,
    device const float4* bvhNodes,
    device const float4* primitives,
    device const float4* materials,
    uint primitiveCount,
    device const int* primitiveIndices,
    thread uint32_t& seed)
{
    constexpr size_t maxRayDepth = 32;

    float4 absorption = float4(1.0);
    float4 light = float4(0.0);

    for (size_t depth = 0; depth < maxRayDepth; ++depth)
    {
        intersection hit = firstHitBVH(ray, bvhNodes, primitiveIndices, primitives, primitiveCount);

        if (hit.t == INFINITY)
        {
            float3 unitDir = normalize(ray.direction);
            float t = 0.5 * (unitDir.y + 1.0);
            float3 skyColor = mix(float3(1.0), float3(0.6, 0.7, 1.0), t);
            light += absorption * float4(skyColor, 1.0);
            break;
        }
        int hitIndex = hit.sphereId >= 0 ? hit.sphereId : hit.triangleId;
        float3 hitNormal = hit.normal;
        float3 hitPosition = hit.point;

        int matIndex = hitIndex * 2;
        float4 m0 = materials[matIndex + 0];
        float4 m1 = materials[matIndex + 1];

        float3 albedo = m0.xyz;
        float materialType = m0.w;
        float3 emissionColor = m1.xyz;
        float emissionPower = m1.w;

        if (emissionPower > 0.0 || materialType == 2)
        {
            light += absorption * float4(emissionColor, 1.0) * emissionPower;
            // Let it continue bouncing if you want indirect light to propagate
        }

        // Lambertian scatter
        float3 target = hitNormal + randomUnitVector(seed);
        ray.origin = hitPosition + 0.0001 * hitNormal;
        ray.direction = normalize(target);
        absorption *= float4(albedo, 1.0);
    }

    return clamp(light, 0.0, 1.0);
}





#endif
