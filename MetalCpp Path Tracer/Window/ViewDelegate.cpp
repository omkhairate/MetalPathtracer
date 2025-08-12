#include "ViewDelegate.h"
#include <AppKit/AppKit.hpp>
#include <cstdlib>
#include <chrono>
#include "ControllerView.hpp"

using namespace MetalCppPathTracer;

ViewDelegate::ViewDelegate(MTL::Device *pDevice)
    : MTK::ViewDelegate(), _pRenderer(new Renderer(pDevice)),
      _lastTime(std::chrono::steady_clock::now()) {
  if (const char *env = std::getenv("MPT_MAX_FRAMES"))
    _maxFrames = std::strtoul(env, nullptr, 10);
}

ViewDelegate::~ViewDelegate() { delete _pRenderer; }

void ViewDelegate::drawInMTKView(MTK::View *pView) {
  auto current = std::chrono::steady_clock::now();
  double fps =
      1.0 / std::chrono::duration<double>(current - _lastTime).count();
  _lastTime = current;
  updateFPS(fps);
  _pRenderer->draw(pView);
  if (_maxFrames > 0 && _pRenderer->hasKeyframes() &&
      ++_frameCount >= _maxFrames)
    NS::Application::sharedApplication()->terminate(nullptr);
}

void ViewDelegate::drawableSizeWillChange(MTK::View *pView, CGSize size) {
  _pRenderer->drawableSizeWillChange(pView, size);
}
