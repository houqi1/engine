#include "scene/VoxelScene.h"

#include "gfx/GfxDevice.h"

#include <cmath>

void VoxelScene::init(GfxDevice& gfx) {
  cube_ = MeshFactory::createCube(gfx);

  camera_.setOrbitTarget(glm::vec3(0.0f, 2.0f, 0.0f));
  camera_.setOrbitDistance(18.0f);
  camera_.setYawPitch(0.85f, 0.45f);

  rebuildVoxels();
}

void VoxelScene::cleanup(GfxDevice& gfx) {
  voxels_.clear();
  MeshFactory::destroy(gfx, cube_);
}

void VoxelScene::update(float dt) {
  time_ += dt;
}

void VoxelScene::rebuildVoxels() {
  voxels_.clear();
  const int n = std::max(4, gridSize_);
  const float half = 0.5f * static_cast<float>(n - 1);
  const float radius = 0.38f * static_cast<float>(n);

  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        const float fx = static_cast<float>(x) - half;
        const float fy = static_cast<float>(y) - half;
        const float fz = static_cast<float>(z) - half;
        const float dist = std::sqrt(fx * fx + fy * fy + fz * fz);
        // Hollow sphere shell as a placeholder volume.
        if (dist > radius || dist < radius - 1.6f) {
          continue;
        }

        VoxelInstance v{};
        v.position = glm::vec3(fx, fy + half * 0.35f, fz) * voxelSize_;
        const float t = static_cast<float>(y) / static_cast<float>(std::max(1, n - 1));
        v.color = glm::mix(glm::vec3(0.20f, 0.55f, 0.95f), glm::vec3(0.95f, 0.55f, 0.20f), t);
        voxels_.push_back(v);
      }
    }
  }
}
