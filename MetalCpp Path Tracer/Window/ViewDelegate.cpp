#include "ViewDelegate.h"
#include <AppKit/AppKit.hpp>
#include <cstdlib>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include "ControllerView.hpp"
#include <mach/mach.h>

using namespace MetalCppPathTracer;

static double getMemoryUsageMB() {
  mach_task_basic_info_data_t info;
  mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
  if (task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
                reinterpret_cast<task_info_t>(&info), &count) != KERN_SUCCESS)
    return 0.0;
  return static_cast<double>(info.resident_size) / (1024.0 * 1024.0);
}

ViewDelegate::ViewDelegate(MTL::Device *pDevice)
    : MTK::ViewDelegate(), _pRenderer(new Renderer(pDevice)),
      _lastTime(std::chrono::steady_clock::now()) {
  if (const char *env = std::getenv("MPT_MAX_FRAMES"))
    _maxFrames = std::strtoul(env, nullptr, 10);
  if (const char *dump = std::getenv("MPT_DUMP_AS")) {
    _dumpPath = dump;
    std::filesystem::create_directories(_dumpPath);
  }
}

ViewDelegate::~ViewDelegate() { delete _pRenderer; }

void ViewDelegate::drawInMTKView(MTK::View *pView) {
  auto current = std::chrono::steady_clock::now();
  double fps =
      1.0 / std::chrono::duration<double>(current - _lastTime).count();
  _lastTime = current;
  updateFPS(fps);
  updateMemoryUsage(getMemoryUsageMB());
  _pRenderer->draw(pView);
  if (!_dumpPath.empty()) {
    char file[256];
    std::snprintf(file, sizeof(file), "%s/frame_%04zu.json", _dumpPath.c_str(),
                  _frameCount);
    _pRenderer->dumpAccelerationStructure(file);
  }
  ++_frameCount;
  if (_maxFrames > 0 && _pRenderer->hasKeyframes() &&
      _frameCount >= _maxFrames)
    NS::Application::sharedApplication()->terminate(nullptr);
}

void ViewDelegate::drawableSizeWillChange(MTK::View *pView, CGSize size) {
  _pRenderer->drawableSizeWillChange(pView, size);
}
