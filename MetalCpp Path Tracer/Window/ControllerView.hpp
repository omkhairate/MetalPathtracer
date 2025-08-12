#ifndef VIEW_ADAPTER_HPP
#define VIEW_ADAPTER_HPP

#import <MetalKit/MetalKit.hpp>

namespace MetalCppPathTracer
{

class ControllerView
{
public:
  virtual MTK::View* get(CGRect frame);
};

// Update the FPS text overlay on the rendering view.
void updateFPS(double fps);

};

#endif
