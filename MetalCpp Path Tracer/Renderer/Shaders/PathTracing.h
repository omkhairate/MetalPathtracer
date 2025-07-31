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
        if (invD < 0.0) swap(t0, t1);
        tMin = max(tMin, t0);
        tMax = min(tMax, t1);
        if (tMax <= tMin) return false;
    }
    return true;
}

inline intersection firstHitBVHUnified(
    thread const ray& r,
    device const float4* bvhNodes,
    device const float4* primitives,
    uint32_t primitiveCount,
    uint32_t triangleCount)
{
    intersection hit;
    hit.t = INFINITY;
    hit.sphereId = -1;
    hit.triangleId = -1;

    constexpr int stackSize = 64;
    int stack[stackSize];
    int stackPtr = 0;
    stack[stackPtr++] = 0;

    while (stackPtr > 0)
    {
        int nodeIdx = stack[--stackPtr];

        float3 bmin = bvhNodes[2 * nodeIdx + 0].xyz;
        float3 bmax = bvhNodes[2 * nodeIdx + 1].xyz;
        int leftFirst = as_type<int>(bvhNodes[2 * nodeIdx + 0].w);
        int count = as_type<int>(bvhNodes[2 * nodeIdx + 1].w);

        if (!intersectAABB(r, bmin, bmax, 0.0001, hit.t)) continue;

        if (count > 0)
        {
            for (int i = 0; i < count; ++i)
            {
                int primIdx = leftFirst + i;
                int base = primIdx * 3;
                float type = primitives[base].w;

                if (type == 0) // Sphere
                {
                    float3 center = primitives[base].xyz;
                    float radius = primitives[base + 1].x;
                    float4 sphereData = float4(center, radius);
                    float root = sphereIntersection(r, sphereData);
                    if (root < hit.t && root != INFINITY)
                    {
                        hit.t = root;
                        hit.sphereId = primIdx;
                        hit.triangleId = -1;
                    }
                }
                else if (type == 1) // Triangle
                {
                    float3 v0 = primitives[base].xyz;
                    float3 v1 = primitives[base + 1].xyz;
                    float3 v2 = primitives[base + 2].xyz;

                    float tTri;
                    float3 bary;
                    if (triangleIntersection(r, v0, v1, v2, tTri, bary))
                    {
                        if (tTri < hit.t)
                        {
                            hit.t = tTri;
                            hit.sphereId = -1;
                            hit.triangleId = primIdx;
                            hit.normal = normalize(cross(v1 - v0, v2 - v0));
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

    if (hit.t == INFINITY) return hit;

    hit.point = r.origin + hit.t * r.direction;

    if (hit.sphereId >= 0)
    {
        int base = hit.sphereId * 3;
        hit.normal = normalize(hit.point - primitives[base].xyz);
    }

    hit.frontFace = dot(hit.normal, r.direction) < 0.0;
    if (!hit.frontFace)
        hit.normal = -hit.normal;

    return hit;
}

inline float4 rayColorBVH(
    ray ray,
    device const float4* bvhNodes,
    device const float4* primitives,
    device const float4* materials,
    uint primitiveCount,
    uint triangleCount,
    thread uint32_t& seed)
{
    constexpr size_t maxRayDepth = 32;

    float4 absorption = float4(1.0);
    float4 light = float4(0.0);

    for (size_t depth = 0; depth < maxRayDepth; ++depth)
    {
        intersection in = firstHitBVHUnified(ray, bvhNodes, primitives, primitiveCount, triangleCount);

        if (in.t == INFINITY)
        {
            float3 unitDir = normalize(ray.direction);
            float t = 0.5 * (unitDir.y + 1.0);
            float3 skyColor = mix(float3(1.0), float3(0.6, 0.7, 1.0), t);
            light += absorption * float4(skyColor, 1.0);
            break;
        }

        uint hitIndex = (in.sphereId >= 0) ? uint(in.sphereId) : uint(in.triangleId);
        int matIndex = int(hitIndex) * 2;

        float4 m0 = materials[matIndex + 0];
        float4 m1 = materials[matIndex + 1];

        float3 albedo = m0.xyz;
        float materialType = m0.w;
        float3 emissionColor = m1.xyz;
        float emissionPower = m1.w;

        if (emissionPower > 0.0 || materialType == 2)
        {
            light += absorption * float4(emissionColor, 1.0) * emissionPower;
        }

        float3 target = in.normal + randomUnitVector(seed);
        ray.origin = in.point + 0.0001 * in.normal;
        ray.direction = normalize(target);
        absorption *= float4(albedo, 1.0);
    }

    return clamp(light, 0.0, 1.0);
}

#endif
