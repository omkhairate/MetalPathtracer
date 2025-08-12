#include "ViewDelegate.h"
#include <AppKit/AppKit.hpp>
#include <cstdlib>

using namespace MetalCppPathTracer;

ViewDelegate::ViewDelegate(MTL::Device *pDevice)
    : MTK::ViewDelegate(), _pRenderer(new Renderer(pDevice)) {
  if (const char *env = std::getenv("MPT_MAX_FRAMES"))
    _maxFrames = std::strtoul(env, nullptr, 10);
}

ViewDelegate::~ViewDelegate() { delete _pRenderer; }

void ViewDelegate::drawInMTKView(MTK::View *pView) {
  _pRenderer->draw(pView);
  if (_maxFrames > 0 && _pRenderer->hasKeyframes() &&
      ++_frameCount >= _maxFrames)
    NS::Application::sharedApplication()->terminate(nullptr);
}

void ViewDelegate::drawableSizeWillChange(MTK::View *pView, CGSize size) {
  _pRenderer->drawableSizeWillChange(pView, size);
}
