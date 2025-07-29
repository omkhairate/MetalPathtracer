#include "SceneLoader.h"
#include "Material.h"
#include <tinyxml2.h>
#include <simd/simd.h>
#include <cstdio>
#include <fstream>
#include <sstream>
#include "tiny_obj_loader.h"

using namespace tinyxml2;

namespace MetalCppPathTracer {

static simd::float3 parseVec3(const char* str) {
    float x=0,y=0,z=0;
    sscanf(str, "%f,%f,%f", &x,&y,&z);
    return simd::make_float3(x,y,z);
}

static void LoadOBJ(const std::string& path, std::vector<simd::float3>& verts, std::vector<simd::uint3>& tris) {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, path.c_str());

    if (!warn.empty()) {
        printf("tinyobjloader warning: %s\n", warn.c_str());
    }
    if (!err.empty()) {
        printf("tinyobjloader error: %s\n", err.c_str());
    }
    if (!ret) {
        printf("Failed to load OBJ: %s\n", path.c_str());
        return;
    }

    verts.reserve(attrib.vertices.size() / 3);
    for (size_t v = 0; v < attrib.vertices.size(); v += 3) {
        verts.push_back(simd::make_float3(
            attrib.vertices[v],
            attrib.vertices[v + 1],
            attrib.vertices[v + 2]
        ));
    }

    for (const auto& shape : shapes) {
        size_t indexOffset = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
            size_t fv = size_t(shape.mesh.num_face_vertices[f]);
            if (fv != 3) {
                indexOffset += fv;
                continue;
            }

            uint32_t idx0 = shape.mesh.indices[indexOffset + 0].vertex_index;
            uint32_t idx1 = shape.mesh.indices[indexOffset + 1].vertex_index;
            uint32_t idx2 = shape.mesh.indices[indexOffset + 2].vertex_index;

            if (idx0 >= verts.size() || idx1 >= verts.size() || idx2 >= verts.size()) {
                printf("Invalid triangle indices\n");
                indexOffset += fv;
                continue;
            }

            tris.push_back(simd::make_uint3(idx0, idx1, idx2));
            indexOffset += fv;
        }
    }

    printf("Loaded OBJ: %zu vertices, %zu triangles\n", verts.size(), tris.size());
}

void SceneLoader::LoadSceneFromXML(const std::string& path, Scene* scene) {
    XMLDocument doc;
    if (doc.LoadFile(path.c_str()) != XML_SUCCESS) {
        printf("Failed to load scene XML: %s\n", path.c_str());
        return;
    }

    scene->clear();

    auto* root = doc.FirstChildElement("Scene");
    if (!root) {
        printf("No <Scene> root.\n");
        return;
    }

    for (auto* e = root->FirstChildElement(); e; e = e->NextSiblingElement()) {
        std::string tag = e->Name();
        if (tag == "Sphere") {
            Primitive p;
            p.type = PrimitiveType::Sphere;
            p.data0 = parseVec3(e->Attribute("position"));
            float r = e->FloatAttribute("radius", 1.0f);
            p.data1 = simd::make_float3(r,0,0);
            p.data2 = simd::float3(0); // unused for spheres

            p.material.albedo = parseVec3(e->Attribute("albedo"));
            p.material.emissionColor = parseVec3(e->Attribute("emission"));
            p.material.materialType = e->FloatAttribute("materialType", 0);
            p.material.emissionPower = e->FloatAttribute("emissionPower", 0);

            scene->addPrimitive(p);
        }
        else if (tag == "Mesh") {
            std::vector<simd::float3> verts;
            std::vector<simd::uint3> tris;

            LoadOBJ(e->Attribute("file"), verts, tris);

            simd::float3 pos = parseVec3(e->Attribute("position"));
            float scale = e->FloatAttribute("scale", 1.0f);

            Material m;
            m.albedo = parseVec3(e->Attribute("albedo"));
            m.emissionColor = parseVec3(e->Attribute("emission"));
            m.materialType = e->FloatAttribute("materialType", 0);
            m.emissionPower = e->FloatAttribute("emissionPower", 0);

            for (const auto& tri : tris) {
                Primitive p;
                p.type = PrimitiveType::Triangle;
                p.data0 = pos + scale * verts[tri.x];
                p.data1 = pos + scale * verts[tri.y];
                p.data2 = pos + scale * verts[tri.z];
                p.material = m;
                scene->addPrimitive(p);
            }
        }
    }
}

}
