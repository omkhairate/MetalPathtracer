#ifndef STRUCTS_H
#define STRUCTS_H

#include <metal_stdlib>

struct v2f
{
    float4 position [[position]];
    float2 uv;
};

struct intersection
{
    float t = INFINITY;
    float3 point;
    float3 normal;
    bool frontFace;
    int primitiveId = -1;
    int isTriangle = 0;
};


struct UniformsData
{
    int primitiveIndex;
    simd::float3 cameraPosition;
    simd::float2 screenSize;

    simd::float3 viewportU;
    simd::float3 viewportV;
    simd::float3 firstPixelPosition;

    simd::float3 randomSeed;

    uint64_t primitiveCount;
    uint64_t triangleCount;
    uint64_t frameCount = 0;
    uint64_t totalPrimitiveCount;


};




#endif
