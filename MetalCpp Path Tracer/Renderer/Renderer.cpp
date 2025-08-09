#include "Renderer.h"

#include "Camera.h"
#include "InputSystem.h"
#include "Scene.h"
#include "SceneLoader.h"
#include <cstdio>
#include <simd/simd.h>
#include <filesystem>
#include <cmath>
#include <algorithm>
#include <cstring>

using namespace MetalCppPathTracer;

struct UniformsData {
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
  uint64_t tlasNodeCount;
  uint64_t blasNodeCount;
  uint32_t maxRayDepth;
  uint32_t debugAS;
};

inline uint32_t bitm_random() {
  static uint32_t current_seed = 92407235;
  const uint32_t state = current_seed * 747796405u + 2891336453u;
  const uint32_t word = ((state >> ((state >> 28u) + 4u)) ^ state);
  return (current_seed = (word >> 22u) ^ word);
}

inline float randomFloat() {
  return (float)bitm_random() / (float)std::numeric_limits<uint32_t>::max();
}

bool Renderer::isInView(const BoundingSphere &b) {
  simd::float3 toCenter = b.center - Camera::position;
  float dist = simd::length(toCenter);
  if (dist < 1e-5f)
    return true;
  simd::float3 dir = toCenter / dist;
  float cosAngle = simd::dot(simd::normalize(Camera::forward), dir);
  if (cosAngle <= 0.0f)
    return false;
  float angle = acosf(cosAngle);
  float halfFov = (Camera::verticalFov * M_PI / 180.0f) * 0.5f;
  float radiusAngle = asinf(std::min(b.radius / dist, 1.0f));
  return angle <= halfFov + radiusAngle;
}

void Renderer::syncSceneWithActivePrimitives() {
  // Preserve camera path and settings while refreshing primitive list
  auto cameraPath = _pScene->cameraPath;
  auto size = _pScene->screenSize;
  auto depth = _pScene->maxRayDepth;
  _pScene->clear();
  _pScene->cameraPath = cameraPath;
  _pScene->screenSize = size;
  _pScene->maxRayDepth = depth;
  _activeToGlobalIndex.clear();
  for (size_t i = 0; i < _allPrimitives.size(); ++i) {
    if (_activePrimitive[i]) {
      _pScene->addPrimitive(_allPrimitives[i]);
      _activeToGlobalIndex.push_back(i);
    }
  }
  size_t activeCount = _activeToGlobalIndex.size();
  if (_pIntersectionCountBuffer)
    _pIntersectionCountBuffer->release();
  size_t bytes = sizeof(uint32_t) * activeCount;
  if (bytes == 0)
    bytes = sizeof(uint32_t);
  _pIntersectionCountBuffer =
      _pDevice->newBuffer(bytes, MTL::ResourceStorageModeManaged);
  memset(_pIntersectionCountBuffer->contents(), 0, bytes);
  _pIntersectionCountBuffer->didModifyRange(NS::Range::Make(0, bytes));
}

Renderer::Renderer(MTL::Device *pDevice)
    : _pDevice(pDevice->retain()), _pScene(new Scene()) {
  _pCommandQueue = _pDevice->newCommandQueue();

  Camera::reset();

  updateVisibleScene();
  buildShaders();
  buildTextures();

  recalculateViewport();
}

Renderer::~Renderer() {
  if (_pSphereBuffer)
    _pSphereBuffer->release();
  if (_pSphereMaterialBuffer)
    _pSphereMaterialBuffer->release();
  if (_pTriangleVertexBuffer)
    _pTriangleVertexBuffer->release();
  if (_pTriangleIndexBuffer)
    _pTriangleIndexBuffer->release();
  if (_pUniformsBuffer)
    _pUniformsBuffer->release();
  if (_pBVHBuffer)
    _pBVHBuffer->release();
  if (_pPrimitiveIndexBuffer)
    _pPrimitiveIndexBuffer->release();
  if (_pTLASBuffer)
    _pTLASBuffer->release();
  if (_pIntersectionCountBuffer)
    _pIntersectionCountBuffer->release();

  for (int i = 0; i < 2; i++)
    if (_accumulationTargets[i])
      _accumulationTargets[i]->release();

  if (_pPSO)
    _pPSO->release();
  if (_pCommandQueue)
    _pCommandQueue->release();
  if (_pDevice)
    _pDevice->release();

  delete _pScene;
}

void Renderer::buildShaders() {
  using NS::StringEncoding::UTF8StringEncoding;

  NS::Error *pError = nullptr;
  MTL::Library *pLibrary = _pDevice->newDefaultLibrary();

  if (!pLibrary) {
    __builtin_printf("Failed to load Metal library\n");
    assert(false);
  }

  MTL::Function *pVertexFn = pLibrary->newFunction(
      NS::String::string("vertexMain", UTF8StringEncoding));
  MTL::Function *pFragFn = pLibrary->newFunction(
      NS::String::string("fragmentMain", UTF8StringEncoding));

  MTL::RenderPipelineDescriptor *pDesc =
      MTL::RenderPipelineDescriptor::alloc()->init();
  pDesc->setVertexFunction(pVertexFn);
  pDesc->setFragmentFunction(pFragFn);
  pDesc->colorAttachments()->object(0)->setPixelFormat(
      MTL::PixelFormat::PixelFormatRGBA16Float);

  _pPSO = _pDevice->newRenderPipelineState(pDesc, &pError);
  if (!_pPSO) {
    __builtin_printf("%s\n", pError->localizedDescription()->utf8String());
    assert(false);
  }

  pVertexFn->release();
  pFragFn->release();
  pDesc->release();
  pLibrary->release();
}

void Renderer::updateVisibleScene() {
  if (!SceneLoader::LoadSceneFromXML("scene.xml", _pScene)) {
    std::filesystem::path alt =
        std::filesystem::path(__FILE__).parent_path() / "../scene.xml";
    SceneLoader::LoadSceneFromXML(alt.string(), _pScene);
  }

  Camera::screenSize = _pScene->screenSize;

  if (!_pScene->cameraPath.empty()) {
    const auto &k = _pScene->cameraPath.front();
    Camera::position = k.position;
    Camera::forward = simd::normalize(k.lookAt - k.position);
    Camera::up = {0, 1, 0};
  }

  printf(
      "Scene loaded: %zu total primitives (%zu spheres, %zu triangles, %zu rectangles)\n",
      _pScene->getPrimitiveCount(), _pScene->getSphereCount(),
      _pScene->getTriangleCount(), _pScene->getRectangleCount());

  // Store full primitive list and initialize tracking
  _allPrimitives = _pScene->getPrimitives();
  size_t primCount = _allPrimitives.size();
  _activePrimitive.assign(primCount, true);
  _inactiveFrames.assign(primCount, 0);
  _primitiveBounds.resize(primCount);
  for (size_t i = 0; i < primCount; ++i) {
    const Primitive &p = _allPrimitives[i];
    if (p.type == PrimitiveType::Sphere) {
      _primitiveBounds[i] = {p.sphere.center, p.sphere.radius};
    } else if (p.type == PrimitiveType::Triangle) {
      simd::float3 c = (p.triangle.v0 + p.triangle.v1 + p.triangle.v2) / 3.0f;
      float r = simd::length(p.triangle.v0 - c);
      r = std::max(r, (float)simd::length(p.triangle.v1 - c));
      r = std::max(r, (float)simd::length(p.triangle.v2 - c));
      _primitiveBounds[i] = {c, r};
    } else {
      float r = simd::length(p.rectangle.u) + simd::length(p.rectangle.v);
      _primitiveBounds[i] = {p.rectangle.center, r};
    }
  }

  syncSceneWithActivePrimitives();
  rebuildAccelerationStructures();
  buildBuffers();
}

void Renderer::recalculateViewport() {
  float aspectRatio = Camera::screenSize.x / Camera::screenSize.y;
  float fovRad = Camera::verticalFov * (M_PI / 180.0f);
  float halfHeight = tanf(fovRad * 0.5f);
  float halfWidth = aspectRatio * halfHeight;

  simd::float3 w = simd::normalize(-Camera::forward);
  simd::float3 u = simd::normalize(simd::cross(Camera::up, w));
  simd::float3 v = simd::cross(w, u);

  simd::float3 viewportU = u * (2.0f * halfWidth);
  simd::float3 viewportV = -v * (2.0f * halfHeight);

  simd::float3 firstPixelPosition =
      Camera::position - w - (viewportU * 0.5f) - (viewportV * 0.5f);

  UniformsData *uData = (UniformsData *)_pUniformsBuffer->contents();
  uData->cameraPosition = Camera::position;
  uData->viewportU = viewportU;
  uData->viewportV = viewportV;
  uData->firstPixelPosition = firstPixelPosition;
  uData->screenSize = Camera::screenSize;

  _pUniformsBuffer->didModifyRange(NS::Range::Make(0, sizeof(UniformsData)));

  printf("viewportU: (%f, %f, %f)\n", viewportU.x, viewportU.y, viewportU.z);
  printf("viewportV: (%f, %f, %f)\n", viewportV.x, viewportV.y, viewportV.z);
  printf("firstPixel: (%f, %f, %f)\n", firstPixelPosition.x,
         firstPixelPosition.y, firstPixelPosition.z);
}

void Renderer::buildBuffers() {
  const size_t primitiveCount = _pScene->getPrimitiveCount();
  const size_t uniformsDataSize = sizeof(UniformsData);

  // Uniforms
  if (_pUniformsBuffer)
    _pUniformsBuffer->release();
  _pUniformsBuffer =
      _pDevice->newBuffer(uniformsDataSize, MTL::ResourceStorageModeManaged);
  _pUniformsBuffer->didModifyRange(NS::Range::Make(0, uniformsDataSize));

  // Destroy previous
  if (_pSphereBuffer) {
    _pSphereBuffer->release();
    _pSphereBuffer = nullptr;
  }
  if (_pSphereMaterialBuffer) {
    _pSphereMaterialBuffer->release();
    _pSphereMaterialBuffer = nullptr;
  }

  // ✅ Unified buffer
  simd::float4 *primitiveBuffer =
      _pScene->createTransformsBuffer(); // 3 float4s per primitive
  simd::float4 *materialBuffer =
      _pScene->createMaterialsBuffer(); // 2 float4s per primitive

  const size_t primitiveSize = primitiveCount * 3 * sizeof(simd::float4);
  const size_t materialSize = primitiveCount * 2 * sizeof(simd::float4);

  _pSphereBuffer =
      _pDevice->newBuffer(primitiveSize, MTL::ResourceStorageModeManaged);
  _pSphereMaterialBuffer =
      _pDevice->newBuffer(materialSize, MTL::ResourceStorageModeManaged);

  memcpy(_pSphereBuffer->contents(), primitiveBuffer, primitiveSize);
  memcpy(_pSphereMaterialBuffer->contents(), materialBuffer, materialSize);

  _pSphereBuffer->didModifyRange(NS::Range::Make(0, primitiveSize));
  _pSphereMaterialBuffer->didModifyRange(NS::Range::Make(0, materialSize));

  delete[] primitiveBuffer;
  delete[] materialBuffer;

  // Dummy triangle buffer bindings
  simd::float3 dummyVertex = {0, 0, 0};
  simd::uint3 dummyIndex = {0, 0, 0};

  _pTriangleVertexBuffer = _pDevice->newBuffer(
      &dummyVertex, sizeof(simd::float3), MTL::ResourceStorageModeManaged);
  _pTriangleIndexBuffer = _pDevice->newBuffer(&dummyIndex, sizeof(simd::uint3),
                                              MTL::ResourceStorageModeManaged);
}

void Renderer::buildTextures() {
  MTL::TextureDescriptor *textureDescriptor =
      MTL::TextureDescriptor::alloc()->init();

  textureDescriptor->setPixelFormat(MTL::PixelFormat::PixelFormatRGBA32Float);
  textureDescriptor->setTextureType(MTL::TextureType::TextureType2D);
  textureDescriptor->setWidth(Camera::screenSize.x);
  textureDescriptor->setHeight(Camera::screenSize.y);
  textureDescriptor->setStorageMode(MTL::StorageMode::StorageModePrivate);
  textureDescriptor->setUsage(MTL::TextureUsageShaderRead |
                              MTL::TextureUsageShaderWrite);

  for (uint i = 0; i < 2; i++)
    _accumulationTargets[i] = _pDevice->newTexture(textureDescriptor);
}

bool Renderer::updateCamera() {
  bool changed = false;
  const auto &path = _pScene->cameraPath;
  if (!path.empty()) {
    if (_animationFrame <= path.front().frame) {
      Camera::position = path.front().position;
      simd::float3 look = path.front().lookAt;
      Camera::forward = simd::normalize(look - Camera::position);
    } else if (_animationFrame >= path.back().frame) {
      Camera::position = path.back().position;
      simd::float3 look = path.back().lookAt;
      Camera::forward = simd::normalize(look - Camera::position);
    } else {
      for (size_t i = 0; i + 1 < path.size(); ++i) {
        const auto &k0 = path[i];
        const auto &k1 = path[i + 1];
        if (_animationFrame >= k0.frame && _animationFrame <= k1.frame) {
          float t = float(_animationFrame - k0.frame) / float(k1.frame - k0.frame);
          Camera::position = k0.position + t * (k1.position - k0.position);
          simd::float3 look = k0.lookAt + t * (k1.lookAt - k0.lookAt);
          Camera::forward = simd::normalize(look - Camera::position);
          break;
        }
      }
    }
    Camera::up = {0, 1, 0};
    InputSystem::clearInputs();
    changed = true;
  } else {
    changed = Camera::transformWithInputs();
  }
  if (changed) {
    recalculateViewport();
    rebuildAccelerationStructures();
  }
  // Advance animation frame after processing camera path
  _animationFrame++;
  return changed;
}

void Renderer::updateUniforms() {
  UniformsData &u = *((UniformsData *)_pUniformsBuffer->contents());

  if (updateCamera()) {
    u.frameCount = 0;
    u.randomSeed = {randomFloat(), randomFloat(), randomFloat()};
  } else {
    u.frameCount++;
  }

  u.primitiveCount = _pScene->getPrimitiveCount();
  u.triangleCount = _pScene->getTriangleCount();
  u.tlasNodeCount = _tlasNodeCount;
  u.blasNodeCount = _blasNodeCount;
  u.maxRayDepth = _pScene->maxRayDepth;
  u.debugAS = InputSystem::debugAS;

  _pUniformsBuffer->didModifyRange(NS::Range::Make(0, sizeof(UniformsData)));
}

void Renderer::draw(MTK::View *pView) {
  updateUniforms();
  std::swap(_accumulationTargets[0], _accumulationTargets[1]);

  NS::AutoreleasePool *pPool = NS::AutoreleasePool::alloc()->init();

  MTL::CommandBuffer *pCmd = _pCommandQueue->commandBuffer();
  MTL::RenderPassDescriptor *pRpd = pView->currentRenderPassDescriptor();
  MTL::RenderCommandEncoder *pEnc = pCmd->renderCommandEncoder(pRpd);

  pEnc->setRenderPipelineState(_pPSO);

  // Always bind something for each slot
  pEnc->setFragmentBuffer(_pBVHBuffer, 0, 0); // New!
  pEnc->setFragmentBuffer(_pSphereBuffer, 0, 1);
  pEnc->setFragmentBuffer(_pSphereMaterialBuffer, 0, 2);
  pEnc->setFragmentBuffer(_pUniformsBuffer, 0, 3);
  pEnc->setFragmentBuffer(_pTriangleVertexBuffer, 0, 4);
  pEnc->setFragmentBuffer(_pTriangleIndexBuffer, 0, 5);
  pEnc->setFragmentBuffer(_pPrimitiveIndexBuffer, 0, 6);
  pEnc->setFragmentBuffer(_pTLASBuffer, 0, 7);
  pEnc->setFragmentBuffer(_pIntersectionCountBuffer, 0, 8);

  pEnc->setFragmentTexture(_accumulationTargets[0], 0);
  pEnc->setFragmentTexture(_accumulationTargets[1], 1);

  pEnc->drawPrimitives(MTL::PrimitiveType::PrimitiveTypeTriangle,
                       NS::UInteger(0), NS::UInteger(6));

  pEnc->endEncoding();
  pCmd->presentDrawable(pView->currentDrawable());
  pCmd->commit();
  pCmd->waitUntilCompleted();

  size_t activeCount = _activeToGlobalIndex.size();
  uint32_t *counts =
      _pIntersectionCountBuffer
          ? reinterpret_cast<uint32_t *>(_pIntersectionCountBuffer->contents())
          : nullptr;
  bool changed = false;
  // Reduce the number of consecutive empty frames required before a primitive
  // is offloaded. Lowering this threshold makes BLAS/TLAS rebuilds happen more
  // frequently so changes in node counts are easier to observe during
  // animation.
  const int OFFLOAD_THRESHOLD = 5;
  if (counts) {
    for (size_t i = 0; i < activeCount; ++i) {
      size_t g = _activeToGlobalIndex[i];
      uint32_t c = counts[i];
      if (c > 0) {
        _inactiveFrames[g] = 0;
      } else {
        _inactiveFrames[g]++;
        if (_inactiveFrames[g] > OFFLOAD_THRESHOLD && _activePrimitive[g]) {
          _activePrimitive[g] = false;
          changed = true;
        }
      }
      counts[i] = 0;
    }
    _pIntersectionCountBuffer->didModifyRange(
        NS::Range::Make(0, sizeof(uint32_t) * activeCount));
  }

  for (size_t g = 0; g < _allPrimitives.size(); ++g) {
    if (!_activePrimitive[g] && isInView(_primitiveBounds[g])) {
      _activePrimitive[g] = true;
      _inactiveFrames[g] = 0;
      changed = true;
    }
  }

  if (changed) {
    syncSceneWithActivePrimitives();
    rebuildAccelerationStructures();
    buildBuffers();
    recalculateViewport();
  }

  pPool->release();
}

void Renderer::drawableSizeWillChange(MTK::View *pView, CGSize size) {
  for (uint i = 0; i < 2; i++)
    if (_accumulationTargets[i])
      _accumulationTargets[i]->release();

  Camera::screenSize = {(float)size.width, (float)size.height};

  buildTextures();
  recalculateViewport();
}

void Renderer::rebuildAccelerationStructures() {
  _pScene->buildBVH();

  size_t newBlasCount = _pScene->getBVHNodeCount();
  if (newBlasCount != _blasNodeCount)
    printf("BLAS node count changed: %zu -> %zu\n", _blasNodeCount, newBlasCount);
  else
    printf("BLAS node count: %zu\n", newBlasCount);
  _blasNodeCount = newBlasCount;

  simd::float4 *bvhData = _pScene->createBVHBuffer();
  if (_pBVHBuffer)
    _pBVHBuffer->release();
  _pBVHBuffer = _pDevice->newBuffer(
      bvhData, sizeof(simd::float4) * newBlasCount * 2,
      MTL::ResourceStorageModeManaged);
  _pBVHBuffer->didModifyRange(NS::Range::Make(0, _pBVHBuffer->length()));
  delete[] bvhData;

  size_t tlasCount = 0;
  simd::float4 *tlasData = _pScene->createTLASBuffer(tlasCount);
  if (tlasCount != _tlasNodeCount)
    printf("TLAS node count changed: %zu -> %zu\n", _tlasNodeCount, tlasCount);
  else
    printf("TLAS node count: %zu\n", tlasCount);
  _tlasNodeCount = tlasCount;
  if (_pTLASBuffer)
    _pTLASBuffer->release();
  if (tlasData && tlasCount > 0) {
    _pTLASBuffer =
        _pDevice->newBuffer(tlasData, sizeof(simd::float4) * tlasCount * 2,
                            MTL::ResourceStorageModeManaged);
    _pTLASBuffer->didModifyRange(NS::Range::Make(0, _pTLASBuffer->length()));
  } else {
    _pTLASBuffer = _pDevice->newBuffer(1, MTL::ResourceStorageModeManaged);
  }
  delete[] tlasData;

  int *rawIndices = _pScene->createPrimitiveIndexBuffer();
  if (_pPrimitiveIndexBuffer)
    _pPrimitiveIndexBuffer->release();
  _pPrimitiveIndexBuffer = _pDevice->newBuffer(
      rawIndices, sizeof(int) * _pScene->getPrimitiveCount(),
      MTL::ResourceStorageModeManaged);
  _pPrimitiveIndexBuffer->didModifyRange(
      NS::Range::Make(0, sizeof(int) * _pScene->getPrimitiveCount()));
  delete[] rawIndices;
}
