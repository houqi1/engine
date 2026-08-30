#pragma once

#include "core/Camera.h"
#include "gfx/Mesh.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

struct VoxelInstance {
  glm::vec3 position{0.0f};
  glm::vec3 color{1.0f};
};

class VoxelScene {
public:
  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void update(float dt);
  void rebuildVoxels();

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }
  Mesh& cubeMesh() { return cube_; }
  const Mesh& cubeMesh() const { return cube_; }
  const std::vector<VoxelInstance>& voxels() const { return voxels_; }

  int& gridSize() { return gridSize_; }
  float& voxelSize() { return voxelSize_; }
  glm::vec3& lightDir() { return lightDir_; }

private:
  Camera camera_;
  Mesh cube_{};
  std::vector<VoxelInstance> voxels_;

  int gridSize_ = 24;
  float voxelSize_ = 0.35f;
  glm::vec3 lightDir_{0.35f, -1.0f, 0.25f};
  float time_ = 0.0f;
};
