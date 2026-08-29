#pragma once

#include "core/Camera.h"
#include "gfx/Mesh.h"
#include "gfx/Texture.h"
#include "scene/GrassSystem.h"

#include <glm/glm.hpp>

#include <vector>

struct Material {
  Texture* albedo = nullptr;
  glm::vec4 baseColor{1.0f};
  float metallic = 0.0f;
  float roughness = 0.5f;
};

struct RenderObject {
  Mesh* mesh = nullptr;
  Material* material = nullptr;
  glm::mat4 transform{1.0f};
  bool castShadow = true;
};

struct DirectionalLight {
  glm::vec3 direction{0.45f, -1.0f, 0.35f};
  glm::vec3 color{1.0f, 0.96f, 0.9f};
  float intensity = 3.0f;
  glm::vec3 ambient{0.04f, 0.05f, 0.07f};
  float shadowBias = 0.0025f;
};

class Scene {
public:
  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void update(float dt);

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }
  DirectionalLight& light() { return light_; }
  const DirectionalLight& light() const { return light_; }
  const std::vector<RenderObject>& objects() const { return objects_; }
  GrassSystem& grass() { return grass_; }
  const GrassSystem& grass() const { return grass_; }
  float time() const { return time_; }

private:
  Camera camera_;
  DirectionalLight light_;
  GrassSystem grass_;

  Mesh cube_{};
  Mesh plane_{};
  Mesh sphere_{};
  Texture checker_{};
  Texture white_{};
  Texture rust_{};
  Texture gold_{};

  Material matFloor_{};
  Material matCube_{};
  Material matSphere_{};
  Material matGold_{};

  std::vector<RenderObject> objects_;
  float time_ = 0.0f;
};
