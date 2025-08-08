#ifndef SCENE_H
#define SCENE_H

#include "Material.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <simd/simd.h>
#include <cmath>
#include <fstream>

namespace MetalCppPathTracer {

enum class PrimitiveType {
    Sphere = 0,
    Triangle = 1
};

struct Primitive {
    PrimitiveType type;
    simd::float3 data0;
    simd::float3 data1;
    simd::float3 data2;
    Material material;
};

struct BVHNode {
    simd::float3 boundsMin;
    simd::float3 boundsMax;
    int leftFirst;
    int count; // >0: number of primitives (leaf). <=0: -right child index (internal)
};

class Scene {
public:
    Scene() = default;

    void clear() {
        primitives.clear();
        bvhNodes.clear();
        primitiveIndices.clear();
    }

    size_t addPrimitive(const Primitive& p) {
        primitives.push_back(p);
        return primitives.size() - 1;
    }

    size_t getPrimitiveCount() const {
        return primitives.size();
    }

    size_t getSphereCount() const {
        size_t count = 0;
        for (const auto& p : primitives)
            if (p.type == PrimitiveType::Sphere)
                count++;
        return count;
    }

    size_t getTriangleCount() const {
        size_t count = 0;
        for (const auto& p : primitives)
            if (p.type == PrimitiveType::Triangle)
                count++;
        return count;
    }

    const std::vector<size_t>& getPrimitiveIndices() const {
        return primitiveIndices;
    }

    void restoreVisiblePrimitives(const simd::float3& camPos,
                                  const simd::float3& camForward,
                                  float fovDegrees) {
        float cosFov = cosf(fovDegrees * 0.5f * (M_PI / 180.0f));
        auto it = hiddenPrimitives.begin();
        while (it != hiddenPrimitives.end()) {
            if (isPrimitiveVisible(*it, camPos, camForward, cosFov)) {
                primitives.push_back(*it);
                it = hiddenPrimitives.erase(it);
            } else {
                ++it;
            }
        }
    }

    void cullOffscreenPrimitives(const simd::float3& camPos,
                                 const simd::float3& camForward,
                                 float fovDegrees) {
        float cosFov = cosf(fovDegrees * 0.5f * (M_PI / 180.0f));
        auto it = primitives.begin();
        while (it != primitives.end()) {
            if (!isPrimitiveVisible(*it, camPos, camForward, cosFov)) {
                hiddenPrimitives.push_back(*it);
                it = primitives.erase(it);
            } else {
                ++it;
            }
        }
    }

    void buildBVH() {
        std::stable_sort(primitives.begin(), primitives.end(),
                         [](const Primitive& a, const Primitive& b) {
                             return static_cast<int>(a.type) < static_cast<int>(b.type);
                         });

        primitiveIndices.resize(primitives.size());
        for (size_t i = 0; i < primitives.size(); ++i)
            primitiveIndices[i] = i;

        bvhNodes.clear();
        buildBVHRecursive(0, primitives.size());

        for (const auto& p : primitives) {
            if (p.type == PrimitiveType::Sphere) {
                printf("Sphere -> Position: (%.2f, %.2f, %.2f), Radius: %.2f\n",
                       p.data0.x, p.data0.y, p.data0.z, p.data1.x);
            }
        }
        
        

    }

    size_t getBVHNodeCount() const {
        return bvhNodes.size();
    }

    simd::float4* createTransformsBuffer() {
        simd::float4* buffer = new simd::float4[primitives.size() * 3];
        for (size_t i = 0; i < primitives.size(); ++i) {
            const auto& p = primitives[i];
            buffer[3 * i + 0] = simd::make_float4(p.data0, static_cast<float>(p.type));
            buffer[3 * i + 1] = simd::make_float4(p.data1, 0);
            buffer[3 * i + 2] = simd::make_float4(p.data2, 0);
        }
        return buffer;
    }

    simd::float4* createMaterialsBuffer() {
        simd::float4* buffer = new simd::float4[2 * primitives.size()];
        for (size_t i = 0; i < primitives.size(); ++i) {
            const auto& m = primitives[i].material;
            buffer[2 * i + 0] = simd::make_float4(m.albedo, m.materialType);
            buffer[2 * i + 1] = simd::make_float4(m.emissionColor, m.emissionPower);
        }
        return buffer;
    }

    simd::float4* createSphereBuffer() {
        size_t sphereCount = getSphereCount();
        simd::float4* buffer = new simd::float4[sphereCount];

        size_t index = 0;
        for (const auto& p : primitives) {
            if (p.type == PrimitiveType::Sphere) {
                buffer[index++] = simd::make_float4(p.data0, p.data1.x); // pos.xyz + radius
            }
        }

        return buffer;
    }

    simd::float4* createSphereMaterialsBuffer() {
        size_t sphereCount = getSphereCount();
        simd::float4* buffer = new simd::float4[2 * sphereCount];

        size_t index = 0;
        for (const auto& p : primitives) {
            if (p.type == PrimitiveType::Sphere) {
                const auto& m = p.material;
                buffer[2 * index + 0] = simd::make_float4(m.albedo, m.materialType);
                buffer[2 * index + 1] = simd::make_float4(m.emissionColor, m.emissionPower);
                index++;
            }
        }

        return buffer;
    }

    simd::float4* createBVHBuffer() {
        simd::float4* buffer = new simd::float4[bvhNodes.size() * 2];
        for (size_t i = 0; i < bvhNodes.size(); ++i) {
            const auto& n = bvhNodes[i];
            buffer[2 * i + 0] = simd::make_float4(n.boundsMin, *(float*)&n.leftFirst);
            buffer[2 * i + 1] = simd::make_float4(n.boundsMax, *(float*)&n.count);
        }
        return buffer;
    }

    // Build a simple TLAS from the root's children. Returns number of TLAS nodes via outCount
    simd::float4* createTLASBuffer(size_t& outCount) {
        outCount = 0;
        if (bvhNodes.empty()) {
            return nullptr;
        }

        const BVHNode& root = bvhNodes[0];
        if (root.count > 0) {
            // Degenerate case: only a single leaf BVH, treat as one TLAS node
            outCount = 1;
            simd::float4* buffer = new simd::float4[2];
            int rootIndex = 0;
            buffer[0] = simd::make_float4(root.boundsMin, *(float*)&rootIndex);
            buffer[1] = simd::make_float4(root.boundsMax, 0.0f);
            return buffer;
        }

        int leftChild = root.leftFirst;
        int rightChild = -root.count;

        outCount = 2;
        simd::float4* buffer = new simd::float4[outCount * 2];

        const BVHNode& left = bvhNodes[leftChild];
        buffer[0] = simd::make_float4(left.boundsMin, *(float*)&leftChild);
        buffer[1] = simd::make_float4(left.boundsMax, 0.0f);

        const BVHNode& right = bvhNodes[rightChild];
        buffer[2] = simd::make_float4(right.boundsMin, *(float*)&rightChild);
        buffer[3] = simd::make_float4(right.boundsMax, 0.0f);

        return buffer;
    }

    int* createPrimitiveIndexBuffer() {
        int* buffer = new int[primitiveIndices.size()];
        for (size_t i = 0; i < primitiveIndices.size(); ++i) {
            buffer[i] = static_cast<int>(primitiveIndices[i]);
        }
        return buffer;
    }

    void dumpBVH(const char* path) const {
        std::ofstream out(path);
        for (size_t i = 0; i < bvhNodes.size(); ++i) {
            const auto& n = bvhNodes[i];
            out << i << " "
                << n.boundsMin.x << " " << n.boundsMin.y << " " << n.boundsMin.z << " "
                << n.boundsMax.x << " " << n.boundsMax.y << " " << n.boundsMax.z << "\n";
        }
    }

    static void dumpTLAS(const simd::float4* data, size_t count, const char* path) {
        std::ofstream out(path);
        for (size_t i = 0; i < count; ++i) {
            const simd::float4& mn = data[2 * i + 0];
            const simd::float4& mx = data[2 * i + 1];
            out << i << " "
                << mn.x << " " << mn.y << " " << mn.z << " "
                << mx.x << " " << mx.y << " " << mx.z << "\n";
        }
    }

    void createTriangleBuffers(
        std::vector<simd::float3>& outVertices,
        std::vector<simd::uint3>& outIndices)
    {
        outVertices.clear();
        outIndices.clear();
        uint32_t baseVertex = 0;

        for (const auto& p : primitives) {
            if (p.type != PrimitiveType::Triangle)
                continue;

            outVertices.push_back(p.data0);
            outVertices.push_back(p.data1);
            outVertices.push_back(p.data2);

            outIndices.push_back(simd::make_uint3(baseVertex, baseVertex + 1, baseVertex + 2));
            baseVertex += 3;
        }
    }

private:
    std::vector<Primitive> primitives;
    std::vector<Primitive> hiddenPrimitives;
    std::vector<size_t> primitiveIndices;
    std::vector<BVHNode> bvhNodes;

    bool isPrimitiveVisible(const Primitive& p,
                            const simd::float3& camPos,
                            const simd::float3& camForward,
                            float cosFov) const {
        simd::float3 center;
        float radius = 0.0f;

        if (p.type == PrimitiveType::Sphere) {
            center = p.data0;
            radius = p.data1.x;
        } else {
            center = (p.data0 + p.data1 + p.data2) / 3.0f;
            float d0 = simd::length(p.data0 - center);
            float d1 = simd::length(p.data1 - center);
            float d2 = simd::length(p.data2 - center);
            radius = std::max({d0, d1, d2});
        }

        simd::float3 toCenter = center - camPos;
        float dist = simd::length(toCenter);
        if (dist <= 0.0f) return true;

        simd::float3 dir = toCenter / dist;
        float cosAngle = simd::dot(dir, camForward);
        if (cosAngle >= cosFov) return true;

        float sinAngle = sqrtf(std::max(0.0f, 1.0f - cosAngle * cosAngle));
        return dist * sinAngle <= radius;
    }

    int buildBVHRecursive(size_t start, size_t end) {
        BVHNode node;
        simd::float3 bMin(std::numeric_limits<float>::max());
        simd::float3 bMax(-std::numeric_limits<float>::max());

        for (size_t i = start; i < end; ++i) {
            const auto& p = primitives[primitiveIndices[i]];
            simd::float3 pMin, pMax;
            if (p.type == PrimitiveType::Sphere) {
                float r = p.data1.x;
                pMin = p.data0 - r;
                pMax = p.data0 + r;
            } else {
                pMin = simd::min(p.data0, simd::min(p.data1, p.data2));
                pMax = simd::max(p.data0, simd::max(p.data1, p.data2));
            }
            bMin = simd::min(bMin, pMin);
            bMax = simd::max(bMax, pMax);
        }

        node.boundsMin = bMin;
        node.boundsMax = bMax;
        node.leftFirst = static_cast<int>(start);
        node.count = static_cast<int>(end - start);

        int nodeIndex = static_cast<int>(bvhNodes.size());
        bvhNodes.push_back(node);

        if (node.count <= 8)
            return nodeIndex;

        const int axisCount = 3;
        float bestCost = std::numeric_limits<float>::max();
        int bestAxis = -1;
        size_t bestSplit = start + (end - start) / 2;

        const float parentArea = surfaceArea(bMin, bMax);
        if (parentArea <= 0.0f) return nodeIndex;

        for (int axis = 0; axis < axisCount; ++axis) {
            std::sort(primitiveIndices.begin() + start, primitiveIndices.begin() + end,
                      [&](size_t a, size_t b) {
                          return primitives[a].data0[axis] < primitives[b].data0[axis];
                      });

            std::vector<simd::float3> leftMin(end - start);
            std::vector<simd::float3> leftMax(end - start);
            std::vector<simd::float3> rightMin(end - start);
            std::vector<simd::float3> rightMax(end - start);

            simd::float3 currMin(std::numeric_limits<float>::max());
            simd::float3 currMax(-std::numeric_limits<float>::max());
            for (size_t i = start; i < end; ++i) {
                const auto& p = primitives[primitiveIndices[i]];
                simd::float3 pMin, pMax;
                if (p.type == PrimitiveType::Sphere) {
                    float r = p.data1.x;
                    pMin = p.data0 - r;
                    pMax = p.data0 + r;
                } else {
                    pMin = simd::min(p.data0, simd::min(p.data1, p.data2));
                    pMax = simd::max(p.data0, simd::max(p.data1, p.data2));
                }
                currMin = simd::min(currMin, pMin);
                currMax = simd::max(currMax, pMax);
                leftMin[i - start] = currMin;
                leftMax[i - start] = currMax;
            }

            currMin = simd::float3(std::numeric_limits<float>::max());
            currMax = simd::float3(-std::numeric_limits<float>::max());
            for (size_t i = end; i-- > start;) {
                const auto& p = primitives[primitiveIndices[i]];
                simd::float3 pMin, pMax;
                if (p.type == PrimitiveType::Sphere) {
                    float r = p.data1.x;
                    pMin = p.data0 - r;
                    pMax = p.data0 + r;
                } else {
                    pMin = simd::min(p.data0, simd::min(p.data1, p.data2));
                    pMax = simd::max(p.data0, simd::max(p.data1, p.data2));
                }
                currMin = simd::min(currMin, pMin);
                currMax = simd::max(currMax, pMax);
                rightMin[i - start] = currMin;
                rightMax[i - start] = currMax;
            }

            for (size_t i = 1; i < (end - start); ++i) {
                float saLeft = surfaceArea(leftMin[i - 1], leftMax[i - 1]);
                float saRight = surfaceArea(rightMin[i], rightMax[i]);

                size_t leftCount = i;
                size_t rightCount = (end - start) - i;

                float cost = 0.125f + (
                    saLeft / parentArea) * leftCount + (
                    saRight / parentArea) * rightCount;

                if (cost < bestCost) {
                    bestCost = cost;
                    bestAxis = axis;
                    bestSplit = start + i;
                }
            }
        }

        if (bestAxis == -1)
            return nodeIndex;

        std::sort(primitiveIndices.begin() + start, primitiveIndices.begin() + end,
                  [&](size_t a, size_t b) {
                      return primitives[a].data0[bestAxis] < primitives[b].data0[bestAxis];
                  });

        int leftChild = buildBVHRecursive(start, bestSplit);
        int rightChild = buildBVHRecursive(bestSplit, end);

        bvhNodes[nodeIndex].leftFirst = leftChild;
        bvhNodes[nodeIndex].count = -rightChild;

        return nodeIndex;
    }

    float surfaceArea(const simd::float3& bmin, const simd::float3& bmax) {
        simd::float3 d = bmax - bmin;
        return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
};

}

#endif
