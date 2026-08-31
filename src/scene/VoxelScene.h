#pragma once

#include "core/Camera.h"
#include "gfx/GpuTypes.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <vector>

struct GLFWwindow;
class GfxDevice;

struct VoxelHit {
  glm::ivec3 cell{0};
  glm::ivec3 normal{0};  // outward face normal of the hit voxel
  uint32_t material = 0;
};

class VoxelScene {
public:
  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void update(float dt);
  void handleEditInput(GLFWwindow* window, GfxDevice& gfx);
  void rebuildVoxels(GfxDevice& gfx);

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }

  const AllocatedBuffer& voxelBuffer() const { return voxelBuffer_; }
  uint32_t voxelCount() const { return static_cast<uint32_t>(voxelsCpu_.size()); }
  uint32_t occupiedCount() const { return occupiedCount_; }

  int& gridSize() { return gridSize_; }
  float& voxelSize() { return voxelSize_; }
  glm::vec3& lightDir() { return lightDir_; }
  glm::vec3 gridOrigin() const { return gridOrigin_; }
  glm::uvec3 gridDims() const {
    return glm::uvec3(static_cast<uint32_t>(gridSize_));
  }

  float& ambient() { return ambient_; }
  glm::vec3& skyColor() { return skyColor_; }
  uint32_t& maxSteps() { return maxSteps_; }
  int& renderMode() { return renderMode_; }
  int& brushMaterial() { return brushMaterial_; }
  float& brushRadius() { return brushRadius_; }

  std::optional<VoxelHit> lastHit() const { return lastHit_; }

private:
  bool inBounds(const glm::ivec3& p) const;
  uint32_t indexOf(const glm::ivec3& p) const;
  uint32_t getVoxel(const glm::ivec3& p) const;
  bool setVoxelCpu(const glm::ivec3& p, uint32_t material);
  void flushVoxels(GfxDevice& gfx);
  // Sphere brush in voxel units. radius=0 touches only the center cell.
  // placeOnlyEmpty: when true, never overwrite occupied cells (used for placing).
  int applySphereBrush(const glm::ivec3& center, float radius, uint32_t material,
                       bool placeOnlyEmpty);
  std::optional<VoxelHit> pickCenterRay() const;

  Camera camera_;
  AllocatedBuffer voxelBuffer_{};
  std::vector<uint32_t> voxelsCpu_;

  int gridSize_ = 64;
  float voxelSize_ = 0.35f;
  glm::vec3 gridOrigin_{0.0f};
  glm::vec3 lightDir_{0.35f, -1.0f, 0.25f};
  float ambient_ = 0.18f;
  glm::vec3 skyColor_{0.15f, 0.25f, 0.45f};
  uint32_t maxSteps_ = 192;
  uint32_t occupiedCount_ = 0;
  int renderMode_ = 0;  // 0 shaded, 1 albedo, 2 normal, 3 steps, 4 coord
  int brushMaterial_ = 1;
  float brushRadius_ = 0.0f;  // voxel units; 0 = single cell
  float time_ = 0.0f;

  bool prevLmb_ = false;
  bool prevF_ = false;
  std::optional<VoxelHit> lastHit_;
};
