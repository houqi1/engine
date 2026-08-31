#pragma once

#include "core/Camera.h"
#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <optional>
#include <vector>

struct GLFWwindow;
class GfxDevice;

struct VoxelHit {
  glm::ivec3 cell{0};       // coarse cell
  glm::ivec3 micro{0};      // micro cell inside coarse [0,8)
  glm::ivec3 normal{0};     // outward face normal in micro/world axes
  uint32_t material = 0;
  bool hasMicro = false;    // true when nested pick hit a micro voxel
};

class VoxelScene {
public:
  static constexpr int kMicroRes = 8;
  static constexpr int kMicroCount = kMicroRes * kMicroRes * kMicroRes;  // 512
  static constexpr int kMicroWords = kMicroCount / 32;                  // 16

  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void update(float dt);
  void handleEditInput(GLFWwindow* window, GfxDevice& gfx);
  void rebuildVoxels(GfxDevice& gfx);

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }

  const AllocatedBuffer& voxelBuffer() const { return voxelBuffer_; }
  const AllocatedBuffer& microBuffer() const { return microBuffer_; }
  uint32_t voxelCount() const { return static_cast<uint32_t>(voxelsCpu_.size()); }
  uint32_t occupiedCount() const { return occupiedCount_; }
  uint32_t occupiedMicroCount() const { return occupiedMicroCount_; }

  int& gridSize() { return gridSize_; }
  float& voxelSize() { return voxelSize_; }
  glm::vec3& lightDir() { return lightDir_; }
  glm::vec3 gridOrigin() const { return gridOrigin_; }
  glm::uvec3 gridDims() const {
    return glm::uvec3(static_cast<uint32_t>(gridSize_));
  }

  float& ambient() { return ambient_; }
  float& aoStrength() { return aoStrength_; }
  float& aoPower() { return aoPower_; }
  glm::vec3& skyColor() { return skyColor_; }
  bool& showSky() { return showSky_; }
  float& skyIntensity() { return skyIntensity_; }
  float& skyYaw() { return skyYaw_; }
  const Texture& sky() const { return sky_; }
  bool hasSky() const { return sky_.image.image != VK_NULL_HANDLE; }
  uint32_t& maxSteps() { return maxSteps_; }
  int& renderMode() { return renderMode_; }
  int& brushMaterial() { return brushMaterial_; }
  float& brushRadius() { return brushRadius_; }
  bool& nestedMicroVoxels() { return nestedMicroVoxels_; }

  std::optional<VoxelHit> lastHit() const { return lastHit_; }

private:
  bool inBounds(const glm::ivec3& p) const;
  uint32_t indexOf(const glm::ivec3& p) const;
  uint32_t getVoxel(const glm::ivec3& p) const;
  bool setVoxelCpu(const glm::ivec3& p, uint32_t material);

  bool microInBounds(const glm::ivec3& m) const;
  uint32_t microBitIndex(const glm::ivec3& m) const;
  bool getMicro(const glm::ivec3& coarse, const glm::ivec3& micro) const;
  bool setMicroCpu(const glm::ivec3& coarse, const glm::ivec3& micro, bool solid);
  void fillMicroBrickTemplate(uint32_t coarseIndex);
  void fillMicroBrickSolid(uint32_t coarseIndex);
  void clearMicroBrick(uint32_t coarseIndex);
  bool microBrickEmpty(uint32_t coarseIndex) const;
  void syncHasMicroFlag(uint32_t coarseIndex);
  void ensureCoarseBrick(const glm::ivec3& coarse, uint32_t material);

  // Packed in voxelsCpu_/GPU SSBO: low bits = material, bit31 = brick has any micro solid.
  static constexpr uint32_t kVoxelMatMask = 0x7FFFFFFFu;
  static constexpr uint32_t kVoxelHasMicroBit = 0x80000000u;
  static uint32_t materialOf(uint32_t packed) { return packed & kVoxelMatMask; }
  static bool hasMicroOf(uint32_t packed) { return (packed & kVoxelHasMicroBit) != 0u; }
  static uint32_t packVoxel(uint32_t material, bool hasMicro) {
    const uint32_t mat = material & kVoxelMatMask;
    return (mat == 0u) ? 0u : (mat | (hasMicro ? kVoxelHasMicroBit : 0u));
  }

  void flushAll(GfxDevice& gfx);
  int applyCoarseSphereBrush(const glm::ivec3& center, float radius, uint32_t material,
                             bool placeOnlyEmpty);
  int applyMicroSphereBrush(const glm::ivec3& coarse, const glm::ivec3& micro, float radius,
                            bool solid, uint32_t placeMaterial);
  std::optional<VoxelHit> pickCenterRay() const;
  std::optional<VoxelHit> pickCenterRayCoarse() const;
  std::optional<VoxelHit> pickCenterRayNested() const;

  Camera camera_;
  AllocatedBuffer voxelBuffer_{};
  AllocatedBuffer microBuffer_{};
  std::vector<uint32_t> voxelsCpu_;
  std::vector<uint32_t> microCpu_;  // coarseCount * 16 words, 512 occupancy bits each

  int gridSize_ = 48;  // default a bit smaller; each cell stores 8^3 micro bits
  float voxelSize_ = 0.35f;
  glm::vec3 gridOrigin_{0.0f};
  glm::vec3 lightDir_{0.35f, -1.0f, 0.25f};
  float ambient_ = 0.18f;
  float aoStrength_ = 1.0f;
  float aoPower_ = 1.0f / 3.0f;
  glm::vec3 skyColor_{0.15f, 0.25f, 0.45f};
  Texture sky_{};
  bool showSky_ = true;
  float skyIntensity_ = 1.0f;
  float skyYaw_ = 0.0f;
  uint32_t maxSteps_ = 192;
  uint32_t occupiedCount_ = 0;
  uint32_t occupiedMicroCount_ = 0;
  int renderMode_ = 0;
  int brushMaterial_ = 1;
  float brushRadius_ = 0.0f;         // coarse units when nested off; micro units when nested on
  bool nestedMicroVoxels_ = true;
  float time_ = 0.0f;

  bool prevLmb_ = false;
  bool prevF_ = false;
  std::optional<VoxelHit> lastHit_;
};
