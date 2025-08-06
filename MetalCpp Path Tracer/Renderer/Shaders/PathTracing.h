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
    thread const ray& r,
    device const float4* bvhNodes,
    device const float4* primitives,
    device const int* primitiveIndices)
{
    intersection in;
    in.t = INFINITY;
    in.primitiveId = -1;
    in.isTriangle = 0;

    constexpr int stackSize = 64;
    int stack[stackSize];
    int stackPtr = 0;
    stack[stackPtr++] = 0; // root

    while (stackPtr > 0)
    {
        int nodeIdx = stack[--stackPtr];

        float3 bmin = bvhNodes[2 * nodeIdx + 0].xyz;
        float3 bmax = bvhNodes[2 * nodeIdx + 1].xyz;
        int leftFirst = as_type<int>(bvhNodes[2 * nodeIdx + 0].w);
        int count = as_type<int>(bvhNodes[2 * nodeIdx + 1].w);

        if (!intersectAABB(r, bmin, bmax, 0.0001, in.t))
            continue;

        if (count > 0)
        {
            for (int i = 0; i < count; ++i)
            {
                int primIdx = primitiveIndices[leftFirst + i];
                int base = primIdx * 3;
                float4 p0 = primitives[base + 0];
                float4 p1 = primitives[base + 1];
                float4 p2 = primitives[base + 2];

                int primitiveType = int(p0.w);
                float tHit = INFINITY;
                float3 n = float3(0);
                float3 hit = float3(0);
                bool hitThis = false;

                if (primitiveType == 0)
                {
                    float3 center = p0.xyz;
                    float radius = p1.x;
                    float3 oc = r.origin - center;
                    float a = dot(r.direction, r.direction);
                    float b = dot(oc, r.direction);
                    float c = dot(oc, oc) - radius * radius;
                    float discriminant = b * b - a * c;

                    if (discriminant > 0.0)
                    {
                        float sqrtD = sqrt(discriminant);
                        float temp = (-b - sqrtD) / a;
                        if (temp < in.t && temp > 0.0001)
                        {
                            tHit = temp;
                            hit = r.origin + tHit * r.direction;
                            n = normalize(hit - center);
                            hitThis = true;
                        }
                    }
                }
                else if (primitiveType == 1)
                {
                    float3 v0 = p0.xyz;
                    float3 v1 = p1.xyz;
                    float3 v2 = p2.xyz;

                    float3 edge1 = v1 - v0;
                    float3 edge2 = v2 - v0;
                    float3 h = cross(r.direction, edge2);
                    float a = dot(edge1, h);
                    if (abs(a) > 1e-5)
                    {
                        float f = 1.0 / a;
                        float3 s = r.origin - v0;
                        float u = f * dot(s, h);
                        if (u >= 0.0 && u <= 1.0)
                        {
                            float3 q = cross(s, edge1);
                            float v = f * dot(r.direction, q);
                            if (v >= 0.0 && u + v <= 1.0)
                            {
                                float tt = f * dot(edge2, q);
                                if (tt > 0.0001 && tt < in.t)
                                {
                                    tHit = tt;
                                    hit = r.origin + tHit * r.direction;
                                    n = normalize(cross(edge1, edge2));
                                    hitThis = true;
                                    in.isTriangle = 1;
                                }
                            }
                        }
                    }
                }
                
                if (hitThis && tHit < in.t)
                {
                    in.t = tHit;
                    in.primitiveId = primIdx;
                    in.normal = n;
                    in.point = hit;
                    in.isTriangle = primitiveType;
                }
            }
        }
        else
        {
            stack[stackPtr++] = leftFirst;
            stack[stackPtr++] = leftFirst + 1;
        }
    }

    if (in.primitiveId != -1)
    {
        in.frontFace = dot(in.normal, r.direction) < 0.0;
        if (!in.frontFace)
            in.normal = -in.normal;
    }

    return in;
}


inline float4 rayColor(
    ray r,
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
        intersection hit = firstHitBVH(r, bvhNodes, primitives, primitiveIndices);

        if (hit.primitiveId == -1)
        {
            float3 unitDir = normalize(r.direction);
            float t = 0.5 * (unitDir.y + 1.0);
            float3 skyColor = mix(float3(1.0), float3(0.6, 0.7, 1.0), t);
            light += absorption * float4(skyColor, 1.0);
            break;
        }

        int matIndex = hit.primitiveId * 2;
        if (matIndex + 1 >= int(primitiveCount) * 2)
            break;
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
        float3 target = hit.normal + randomUnitVector(seed);
        r.origin = hit.point + 0.0001 * hit.normal;
        r.direction = normalize(target);
        absorption *= float4(albedo, 1.0);
    }

    return clamp(light, 0.0, 1.0);
}





#endif
