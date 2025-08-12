#ifndef VIEW_DELEGATE_H
#define VIEW_DELEGATE_H

#include <Metal/Metal.hpp>
#include <MetalKit/MetalKit.hpp>
#include <cstddef>
#include <chrono>
#include <string>

#include "Renderer.h"

namespace MetalCppPathTracer
{

class ViewDelegate : public MTK::ViewDelegate
{
 public:
    ViewDelegate(MTL::Device *pDevice);
    virtual ~ViewDelegate() override;
    virtual void drawInMTKView(MTK::View* pView) override;
    virtual void drawableSizeWillChange(MTK::View *pView, CGSize size) override;

 private:
    Renderer* _pRenderer;
    std::size_t _frameCount = 0;
    std::size_t _maxFrames = 0;
    std::chrono::steady_clock::time_point _lastTime;
    std::string _dumpPath;
};

};

#endif  //  VIEW_DELEGATE_H
