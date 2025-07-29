#ifndef SCENELOADER_H
#define SCENELOADER_H

#include "Scene.h"
#include <string>

namespace MetalCppPathTracer {

class SceneLoader {
public:
    static void LoadSceneFromXML(const std::string& path, Scene* scene);
};

}

#endif
