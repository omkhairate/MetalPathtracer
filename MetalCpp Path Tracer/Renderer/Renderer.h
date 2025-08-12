#ifndef RENDERER_H
#define RENDERER_H

#include <Metal/Metal.hpp>
#include <MetalKit/MetalKit.hpp>
#include <cstdint>
#include <simd/simd.h>
#include <vector>

#include "Scene.h"

namespace MetalCppPathTracer {

class Renderer {
public:
  Renderer(MTL::Device *pDevice);
  ~Renderer();

  void updateVisibleScene();
  void buildShaders();
  void buildBuffers();
  void buildTextures();

  void recalculateViewport();
  bool updateCamera();

  void updateUniforms();
  void draw(MTK::View *pView);
  void drawableSizeWillChange(MTK::View *pView, CGSize size);

  bool hasKeyframes() const;

  std::vector<std::pair<simd::float3, float>> _allSpheres;

  struct Chunk {
    std::vector<std::pair<simd::float4, simd::float4>>
        spheres; // (transform, material)
    simd::int3 chunkCoords;
  };

private:
  MTL::Device *_pDevice = nullptr;
  MTL::CommandQueue *_pCommandQueue = nullptr;
  MTL::RenderPipelineState *_pPSO = nullptr;

  // Core scene and geometry data
  Scene *_pScene = nullptr;

  // Buffers
  MTL::Buffer *_pSphereBuffer = nullptr;
  MTL::Buffer *_pSphereMaterialBuffer = nullptr;
  MTL::Buffer *_pTriangleVertexBuffer = nullptr;
  MTL::Buffer *_pTriangleIndexBuffer = nullptr;
  MTL::Buffer *_pUniformsBuffer = nullptr;
  MTL::Buffer *_pBVHBuffer = nullptr;
  MTL::Buffer *_pPrimitiveIndexBuffer = nullptr;
  MTL::Buffer *_pTLASBuffer = nullptr;
  MTL::Buffer *_pIntersectionCountBuffer = nullptr;
  size_t _blasNodeCount = 0;
  size_t _tlasNodeCount = 0;
  // Accumulation framebuffers
  MTL::Texture *_accumulationTargets[2] = {nullptr, nullptr};

  struct BoundingSphere {
    simd::float3 center;
    float radius;
  };

  std::vector<Primitive> _allPrimitives;
  std::vector<bool> _activePrimitive;
  std::vector<int> _inactiveFrames;
  std::vector<uint32_t> _lastIntersectionCount;
  std::vector<size_t> _activeToGlobalIndex;
  std::vector<BoundingSphere> _primitiveBounds;

  bool isInView(const BoundingSphere &b);
  void syncSceneWithActivePrimitives();
  void rebuildAccelerationStructures();
  void dumpAccelerationStructure(const std::string &path);
  void processIntersectionCounts();

  size_t _animationFrame = 0;
};

} // namespace MetalCppPathTracer

#endif // RENDERER_H
