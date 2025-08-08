#include "ApplicationDelegate.h"
#include "SceneLoader.h"

using namespace MetalCppPathTracer;

ApplicationDelegate::~ApplicationDelegate() {
  if (_pMtkView)
    _pMtkView->release();
  if (_pWindow)
    _pWindow->release();
  if (_pDevice)
    _pDevice->release();
  if (_pViewDelegate)
    delete _pViewDelegate;
}

void ApplicationDelegate::applicationWillFinishLaunching(
    NS::Notification *pNotification) {
  NS::Application *pApp =
      reinterpret_cast<NS::Application *>(pNotification->object());
  pApp->setActivationPolicy(NS::ActivationPolicy::ActivationPolicyRegular);
}

void ApplicationDelegate::applicationDidFinishLaunching(
    NS::Notification *pNotification) {
  if (_initialized)
    return;
  _initialized = true;

  Scene tmpScene;
  SceneLoader::LoadSceneFromXML(
      "/Users/apollo/Downloads/"
      "MetalPathtracing-05e922c76da6c603e7840e71f7b563ad9b7eb4ea/MetalCpp Path "
      "Tracer/scene.xml",
      &tmpScene);

  CGRect frame = {{100.0, 100.0}, {tmpScene.screenSize.x, tmpScene.screenSize.y}};

  _pWindow = NS::Window::alloc()->init(frame,
                                       NS::WindowStyleMaskClosable |
                                           NS::WindowStyleMaskTitled |
                                           NS::WindowStyleMaskResizable,
                                       NS::BackingStoreBuffered, false);

  _pDevice = MTL::CreateSystemDefaultDevice();

  _pMtkView = _controllerView.get(frame);
  _pMtkView->setDevice(_pDevice);
  _pMtkView->setColorPixelFormat(MTL::PixelFormat::PixelFormatRGBA16Float);
  _pMtkView->setClearColor(MTL::ClearColor::Make(0.0, 0.0, 0.0, 1.0));
  _pMtkView->setPreferredFramesPerSecond(60);
  _pMtkView->setEnableSetNeedsDisplay(false);
  _pMtkView->setPaused(false);

  _pViewDelegate = new ViewDelegate(_pDevice);
  _pMtkView->setDelegate(_pViewDelegate);

  _pWindow->setContentView(_pMtkView);
  _pWindow->setTitle(NS::String::string(
      "MetalCpp Path Tracer", NS::StringEncoding::UTF8StringEncoding));
  _pWindow->makeKeyAndOrderFront(nullptr);

  NS::Application *pApp =
      reinterpret_cast<NS::Application *>(pNotification->object());
  pApp->activateIgnoringOtherApps(true);
}

bool ApplicationDelegate::applicationShouldTerminateAfterLastWindowClosed(
    NS::Application * /*pSender*/) {
  return true;
}

// hehehehe
