#include <metal_stdlib>
#include <metal_raytracing>
using namespace metal;
using metal::raytracing::ray;

#include "PathTracing.h"

float4 fragment fragmentMain(
    v2f in [[stage_in]],
    device const float4* bvhNodes [[buffer(0)]],          // <--- ADD THIS LINE
    device const float4* primitives [[buffer(1)]],
    device const float4* materials [[buffer(2)]],
    device const UniformsData* uniforms [[buffer(3)]],
    device const float3* vertexBuffer [[buffer(4)]],
    device const uint3* indexBuffer [[buffer(5)]],
    device const int* primitiveIndices [[buffer(6)]],
    device const float4* tlasNodes [[buffer(7)]],
    device const uint* activePrims [[buffer(8)]],
    device atomic_uint* hitCounts [[buffer(9)]],
    texture2d<float, access::read_write> lastFrame [[texture(0)]],
    texture2d<float, access::read_write> currentFrame [[texture(1)]])

{
    const device UniformsData& u = *uniforms;

    if (u.frameCount == 0)
    {
        uint2 coord = uint2(in.uv * u.screenSize);
        lastFrame.write(0, coord);
    }

    uint32_t seed = random(in.uv, u.randomSeed.xyz) * ((uint32_t)-1);

    float xOff = (randomFloat(seed) - 0.5) / u.screenSize.x;
    seed = random(seed);
    float yOff = (randomFloat(seed) - 0.5) / u.screenSize.y;
    seed = random(seed);

    float3 rayDir = (
        u.firstPixelPosition +
        (in.uv.x + xOff) * u.viewportU +
        (in.uv.y + yOff) * u.viewportV
    ) - u.cameraPosition;

    ray r{u.cameraPosition, normalize(rayDir)};
    r.min_distance = 0.0001;
    r.max_distance = INFINITY; // or INFINITY
    
    

    

    float4 color = rayColor(
        r,
        tlasNodes,
        u.tlasNodeCount,
        bvhNodes,
        primitives,       // <- Each primitive is 3 float4s
        materials,
        u.primitiveCount,
        primitiveIndices,
        activePrims,
        seed,
        u.maxRayDepth,
        u.debugAS,
        u.blasNodeCount,
        hitCounts
    );



    uint2 coord = uint2(in.uv * u.screenSize);
    uint64_t frameCount = u.frameCount + 1;

    color += lastFrame.read(coord) * (float)(frameCount - 1);
    color /= frameCount;
    color = clamp(color, 0.0, 1.0);

    currentFrame.write(color, coord);
    color.w = 1;
    return color;
}
