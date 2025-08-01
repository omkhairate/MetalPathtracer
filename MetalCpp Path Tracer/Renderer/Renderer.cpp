#include "Renderer.h"

#include <simd/simd.h>
#include "Scene.h"
#include "InputSystem.h"
#include "Camera.h"
#include <cstdio>
#include "SceneLoader.h"

using namespace MetalCppPathTracer;

struct UniformsData
{
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

inline uint32_t bitm_random()
{
    static uint32_t current_seed = 92407235;
    const uint32_t state = current_seed * 747796405u + 2891336453u;
    const uint32_t word = ((state >> ((state >> 28u) + 4u)) ^ state);
    return (current_seed = (word >> 22u) ^ word);
}

inline float randomFloat()
{
    return (float)bitm_random() / (float)std::numeric_limits<uint32_t>::max();
}

Renderer::Renderer(MTL::Device* pDevice)
: _pDevice(pDevice->retain()), _pScene(new Scene())
{
    _pCommandQueue = _pDevice->newCommandQueue();

    Camera::reset();
    Camera::screenSize = {1280, 720};

    updateVisibleScene();
    buildShaders();
    buildBuffers();
    buildTextures();

    recalculateViewport();
}

Renderer::~Renderer()
{
    if (_pSphereBuffer) _pSphereBuffer->release();
    if (_pSphereMaterialBuffer) _pSphereMaterialBuffer->release();
    if (_pTriangleVertexBuffer) _pTriangleVertexBuffer->release();
    if (_pTriangleIndexBuffer) _pTriangleIndexBuffer->release();
    if (_pUniformsBuffer) _pUniformsBuffer->release();
    if (_pBVHBuffer) _pBVHBuffer->release();
    if (_pPrimitiveIndexBuffer) _pPrimitiveIndexBuffer->release();


    for (int i = 0; i < 2; i++)
        if (_accumulationTargets[i]) _accumulationTargets[i]->release();

    if (_pPSO) _pPSO->release();
    if (_pCommandQueue) _pCommandQueue->release();
    if (_pDevice) _pDevice->release();

    delete _pScene;
}

void Renderer::buildShaders()
{
    using NS::StringEncoding::UTF8StringEncoding;

    NS::Error* pError = nullptr;
    MTL::Library* pLibrary = _pDevice->newDefaultLibrary();

    if (!pLibrary)
    {
        __builtin_printf("Failed to load Metal library\n");
        assert(false);
    }

    MTL::Function* pVertexFn = pLibrary->newFunction(NS::String::string("vertexMain", UTF8StringEncoding));
    MTL::Function* pFragFn = pLibrary->newFunction(NS::String::string("fragmentMain", UTF8StringEncoding));

    MTL::RenderPipelineDescriptor* pDesc = MTL::RenderPipelineDescriptor::alloc()->init();
    pDesc->setVertexFunction(pVertexFn);
    pDesc->setFragmentFunction(pFragFn);
    pDesc->colorAttachments()->object(0)->setPixelFormat(MTL::PixelFormat::PixelFormatRGBA16Float);

    _pPSO = _pDevice->newRenderPipelineState(pDesc, &pError);
    if (!_pPSO)
    {
        __builtin_printf("%s\n", pError->localizedDescription()->utf8String());
        assert(false);
    }

    pVertexFn->release();
    pFragFn->release();
    pDesc->release();
    pLibrary->release();
}

void Renderer::updateVisibleScene()
{
    SceneLoader::LoadSceneFromXML("/Users/apollo/Downloads/MetalPathtracing-05e922c76da6c603e7840e71f7b563ad9b7eb4ea/MetalCpp Path Tracer/scene.xml", _pScene);

    printf("Scene loaded: %zu total primitives (%zu spheres, %zu triangles)\n",
           _pScene->getPrimitiveCount(),
           _pScene->getPrimitiveCount() - _pScene->getTriangleCount(),
           _pScene->getTriangleCount());

    _pScene->buildBVH();
    printf("BVH node count: %zu\n", _pScene->getBVHNodeCount());

    // BVH node buffer
    simd::float4* bvhData = _pScene->createBVHBuffer();
    if (_pBVHBuffer) _pBVHBuffer->release();
    _pBVHBuffer = _pDevice->newBuffer(
        bvhData,
        sizeof(simd::float4) * _pScene->getBVHNodeCount() * 2,
        MTL::ResourceStorageModeManaged
    );
    _pBVHBuffer->didModifyRange(NS::Range::Make(0, _pBVHBuffer->length()));
    delete[] bvhData; // optional if `createBVHBuffer` allocates on heap

    // 🆕 Primitive index buffer (for BVH leaf traversal)
    int* rawIndices = _pScene->createPrimitiveIndexBuffer();
    if (_pPrimitiveIndexBuffer) _pPrimitiveIndexBuffer->release();
    _pPrimitiveIndexBuffer = _pDevice->newBuffer(
        rawIndices,
        sizeof(int) * _pScene->getPrimitiveCount(),
        MTL::ResourceStorageModeManaged
    );
    _pPrimitiveIndexBuffer->didModifyRange(NS::Range::Make(0, sizeof(int) * _pScene->getPrimitiveCount()));
    delete[] rawIndices;

    buildBuffers();
}



void Renderer::recalculateViewport()
{
    float aspectRatio = Camera::screenSize.x / Camera::screenSize.y;
    float fovRad = Camera::verticalFov * (M_PI / 180.0f);
    float halfHeight = tanf(fovRad * 0.5f);
    float halfWidth = aspectRatio * halfHeight;

    simd::float3 w = simd::normalize(-Camera::forward);
    simd::float3 u = simd::normalize(simd::cross(Camera::up, w));
    simd::float3 v = simd::cross(w, u);

    simd::float3 viewportU = u * (2.0f * halfWidth);
    simd::float3 viewportV = -v * (2.0f * halfHeight);

    simd::float3 firstPixelPosition = Camera::position - w - (viewportU * 0.5f) - (viewportV * 0.5f);

    UniformsData* uData = (UniformsData*)_pUniformsBuffer->contents();
    uData->cameraPosition = Camera::position;
    uData->viewportU = viewportU;
    uData->viewportV = viewportV;
    uData->firstPixelPosition = firstPixelPosition;
    uData->screenSize = Camera::screenSize;

    _pUniformsBuffer->didModifyRange(NS::Range::Make(0, sizeof(UniformsData)));
    
    printf("viewportU: (%f, %f, %f)\n", viewportU.x, viewportU.y, viewportU.z);
    printf("viewportV: (%f, %f, %f)\n", viewportV.x, viewportV.y, viewportV.z);
    printf("firstPixel: (%f, %f, %f)\n", firstPixelPosition.x, firstPixelPosition.y, firstPixelPosition.z);

}

void Renderer::buildBuffers()
{
    const size_t primitiveCount = _pScene->getPrimitiveCount();
    const size_t uniformsDataSize = sizeof(UniformsData);

    // Uniforms
    if (_pUniformsBuffer) _pUniformsBuffer->release();
    _pUniformsBuffer = _pDevice->newBuffer(uniformsDataSize, MTL::ResourceStorageModeManaged);
    _pUniformsBuffer->didModifyRange(NS::Range::Make(0, uniformsDataSize));

    // Destroy previous
    if (_pSphereBuffer) { _pSphereBuffer->release(); _pSphereBuffer = nullptr; }
    if (_pSphereMaterialBuffer) { _pSphereMaterialBuffer->release(); _pSphereMaterialBuffer = nullptr; }

    // ✅ Unified buffer
    simd::float4* primitiveBuffer = _pScene->createTransformsBuffer();    // 3 float4s per primitive
    simd::float4* materialBuffer = _pScene->createMaterialsBuffer();      // 2 float4s per primitive

    const size_t primitiveSize = primitiveCount * 3 * sizeof(simd::float4);
    const size_t materialSize = primitiveCount * 2 * sizeof(simd::float4);

    _pSphereBuffer = _pDevice->newBuffer(primitiveSize, MTL::ResourceStorageModeManaged);
    _pSphereMaterialBuffer = _pDevice->newBuffer(materialSize, MTL::ResourceStorageModeManaged);

    memcpy(_pSphereBuffer->contents(), primitiveBuffer, primitiveSize);
    memcpy(_pSphereMaterialBuffer->contents(), materialBuffer, materialSize);

    _pSphereBuffer->didModifyRange(NS::Range::Make(0, primitiveSize));
    _pSphereMaterialBuffer->didModifyRange(NS::Range::Make(0, materialSize));

    delete[] primitiveBuffer;
    delete[] materialBuffer;

    // Dummy triangle buffer bindings
    simd::float3 dummyVertex = {0, 0, 0};
    simd::uint3 dummyIndex = {0, 0, 0};

    _pTriangleVertexBuffer = _pDevice->newBuffer(&dummyVertex, sizeof(simd::float3), MTL::ResourceStorageModeManaged);
    _pTriangleIndexBuffer = _pDevice->newBuffer(&dummyIndex, sizeof(simd::uint3), MTL::ResourceStorageModeManaged);
}




void Renderer::buildTextures()
{
    MTL::TextureDescriptor* textureDescriptor = MTL::TextureDescriptor::alloc()->init();

    textureDescriptor->setPixelFormat(MTL::PixelFormat::PixelFormatRGBA32Float);
    textureDescriptor->setTextureType(MTL::TextureType::TextureType2D);
    textureDescriptor->setWidth(Camera::screenSize.x);
    textureDescriptor->setHeight(Camera::screenSize.y);
    textureDescriptor->setStorageMode(MTL::StorageMode::StorageModePrivate);
    textureDescriptor->setUsage(MTL::TextureUsageShaderRead | MTL::TextureUsageShaderWrite);

    for (uint i = 0; i < 2; i++)
        _accumulationTargets[i] = _pDevice->newTexture(textureDescriptor);
}

bool Renderer::updateCamera()
{
    bool changed = Camera::transformWithInputs();
    if (changed)
        recalculateViewport();
    return changed;
}

void Renderer::updateUniforms()
{
    UniformsData& u = *((UniformsData*)_pUniformsBuffer->contents());

    if (updateCamera()) {
        u.frameCount = 0;
        u.randomSeed = {randomFloat(), randomFloat(), randomFloat()};
    } else {
        u.frameCount++;
    }

    u.primitiveCount = _pScene->getPrimitiveCount();
    u.triangleCount = _pScene->getTriangleCount();


    _pUniformsBuffer->didModifyRange(NS::Range::Make(0, sizeof(UniformsData)));
}

void Renderer::draw(MTK::View* pView)
{
    static int frameCounter = 0;
    frameCounter++;

    //if (frameCounter % 30 == 0)
    //    updateVisibleScene();

    updateUniforms();
    std::swap(_accumulationTargets[0], _accumulationTargets[1]);

    NS::AutoreleasePool* pPool = NS::AutoreleasePool::alloc()->init();

    MTL::CommandBuffer* pCmd = _pCommandQueue->commandBuffer();
    MTL::RenderPassDescriptor* pRpd = pView->currentRenderPassDescriptor();
    MTL::RenderCommandEncoder* pEnc = pCmd->renderCommandEncoder(pRpd);

    pEnc->setRenderPipelineState(_pPSO);

    // Always bind something for each slot
    pEnc->setFragmentBuffer(_pBVHBuffer, 0, 0);               // New!
    pEnc->setFragmentBuffer(_pSphereBuffer, 0, 1);
    pEnc->setFragmentBuffer(_pSphereMaterialBuffer, 0, 2);
    pEnc->setFragmentBuffer(_pUniformsBuffer, 0, 3);
    pEnc->setFragmentBuffer(_pTriangleVertexBuffer, 0, 4);
    pEnc->setFragmentBuffer(_pTriangleIndexBuffer, 0, 5);
    pEnc->setFragmentBuffer(_pPrimitiveIndexBuffer, 0, 6);




    pEnc->setFragmentTexture(_accumulationTargets[0], 0);
    pEnc->setFragmentTexture(_accumulationTargets[1], 1);

    pEnc->drawPrimitives(MTL::PrimitiveType::PrimitiveTypeTriangle, NS::UInteger(0), NS::UInteger(6));

    pEnc->endEncoding();
    pCmd->presentDrawable(pView->currentDrawable());
    pCmd->commit();

    pPool->release();
}

void Renderer::drawableSizeWillChange(MTK::View* pView, CGSize size)
{
    for (uint i = 0; i < 2; i++)
        if (_accumulationTargets[i]) _accumulationTargets[i]->release();

    Camera::screenSize = {(float)size.width, (float)size.height};

    buildTextures();
    recalculateViewport();
}
