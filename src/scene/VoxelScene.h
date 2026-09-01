#pragma once

#include "core/Camera.h"
#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <optional>
#include <vector>

struct GLFWwindow;
class GfxDevice;

struct VoxelHit {
  glm::ivec3 cell{0};     // coarse cell
  glm::ivec3 micro{0};    // micro cell inside coarse [0,8)
  glm::ivec3 normal{0};   // outward face normal in object-local axes
  uint32_t material = 0;
  bool hasMicro = false;  // true when nested pick hit a micro voxel
  int objectIndex = 0;
};

// Must match shaders/voxel_dda.comp std430 GpuVoxelObject (176 bytes).
struct GpuVoxelObject {
  float worldToObject[16];
  float objectToWorld[16];
  float voxelSize;
  float _pad0[3];
  uint32_t gridSize[3];
  uint32_t flags;  // bit0 = nestedMicro, bit1 = enabled
  uint32_t voxelOffset;
  uint32_t microOffset;
  uint32_t _pad1[2];
};
static_assert(sizeof(GpuVoxelObject) == 176, "GpuVoxelObject std430 size mismatch");

struct VoxelObject {
  static constexpr uint32_t kFlagNestedMicro = 1u;
  static constexpr uint32_t kFlagEnabled = 2u;

  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};  // w,x,y,z
  float voxelSize = 0.35f;
  int gridSize = 16;
  bool nestedMicro = true;
  bool editable = true;
  bool enabled = true;

  std::vector<uint32_t> voxelsCpu;
  std::vector<uint32_t> microCpu;

  uint32_t voxelOffset = 0;
  uint32_t microOffset = 0;

  // Maps local meters (grid corner = 0) -> world. Rotate about grid center.
  glm::mat4 objectToWorld() const;
  glm::mat4 worldToObject() const;
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

  // Upload object matrices to objectBuffer_ (call each frame from renderer).
  void uploadObjectTransforms(GfxDevice& gfx);

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }

  const AllocatedBuffer& voxelBuffer() const { return voxelBuffer_; }
  const AllocatedBuffer& microBuffer() const { return microBuffer_; }
  const AllocatedBuffer& objectBuffer() const { return objectBuffer_; }
  uint32_t objectCount() const { return static_cast<uint32_t>(objects_.size()); }
  const std::vector<GpuVoxelObject>& gpuObjects() const { return objectsGpu_; }

  uint32_t voxelCount() const { return static_cast<uint32_t>(voxelsCpu_.size()); }
  uint32_t occupiedCount() const { return occupiedCount_; }
  uint32_t occupiedMicroCount() const { return occupiedMicroCount_; }

  // Phase 1 ImGui: these control the ground object (objects_[0]).
  int& gridSize() { return gridSize_; }
  float& voxelSize() { return voxelSize_; }
  glm::vec3& lightDir() { return lightDir_; }
  glm::vec3 gridOrigin() const;
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
  float& spinSpeed() { return spinSpeed_; }
  bool& spinnerEnabled() { return spinnerEnabled_; }

  std::optional<VoxelHit> lastHit() const { return lastHit_; }

private:
  VoxelObject* ground();
  const VoxelObject* ground() const;

  bool inBounds(const VoxelObject& o, const glm::ivec3& p) const;
  uint32_t indexOf(const VoxelObject& o, const glm::ivec3& p) const;
  uint32_t getVoxel(const VoxelObject& o, const glm::ivec3& p) const;
  bool setVoxelCpu(VoxelObject& o, const glm::ivec3& p, uint32_t material);

  bool microInBounds(const glm::ivec3& m) const;
  uint32_t microBitIndex(const glm::ivec3& m) const;
  bool getMicro(const VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro) const;
  bool setMicroCpu(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro, bool solid);
  void fillMicroBrickTemplate(VoxelObject& o, uint32_t coarseIndex);
  void fillMicroBrickSolid(VoxelObject& o, uint32_t coarseIndex);
  void clearMicroBrick(VoxelObject& o, uint32_t coarseIndex);
  bool microBrickEmpty(const VoxelObject& o, uint32_t coarseIndex) const;
  void syncHasMicroFlag(VoxelObject& o, uint32_t coarseIndex);
  void ensureCoarseBrick(VoxelObject& o, const glm::ivec3& coarse, uint32_t material);

  // Packed in voxelsCpu_/GPU SSBO: low bits = material, bit31 = brick has any micro solid.
  static constexpr uint32_t kVoxelMatMask = 0x7FFFFFFFu;
  static constexpr uint32_t kVoxelHasMicroBit = 0x80000000u;
  static uint32_t materialOf(uint32_t packed) { return packed & kVoxelMatMask; }
  static bool hasMicroOf(uint32_t packed) { return (packed & kVoxelHasMicroBit) != 0u; }
  static uint32_t packVoxel(uint32_t material, bool hasMicro) {
    const uint32_t mat = material & kVoxelMatMask;
    return (mat == 0u) ? 0u : (mat | (hasMicro ? kVoxelHasMicroBit : 0u));
  }

  void buildGroundObject(VoxelObject& o) const;
  void buildSpinnerObject(VoxelObject& o) const;
  void packObjectPool();
  void fillGpuObjectRecords();
  void ensureGpuBuffers(GfxDevice& gfx);
  void flushObject(GfxDevice& gfx, int objectIndex);

  int applyCoarseSphereBrush(VoxelObject& o, const glm::ivec3& center, float radius,
                             uint32_t material, bool placeOnlyEmpty);
  int applyMicroSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                            float radius, bool solid, uint32_t placeMaterial);

  struct PickResult {
    VoxelHit hit{};
    float tWorld = 0.0f;
  };
  std::optional<PickResult> pickObject(const VoxelObject& o, int objectIndex, const glm::vec3& Ow,
                                       const glm::vec3& Dw) const;
  std::optional<VoxelHit> pickCenterRay() const;

  Camera camera_;
  AllocatedBuffer voxelBuffer_{};
  AllocatedBuffer microBuffer_{};
  AllocatedBuffer objectBuffer_{};

  std::vector<VoxelObject> objects_;
  std::vector<GpuVoxelObject> objectsGpu_;
  std::vector<uint32_t> voxelsCpu_;  // pooled
  std::vector<uint32_t> microCpu_;   // pooled

  int gridSize_ = 48;
  float voxelSize_ = 0.35f;
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
  float brushRadius_ = 0.0f;
  bool nestedMicroVoxels_ = true;
  float time_ = 0.0f;
  float spinSpeed_ = 0.8f;
  bool spinnerEnabled_ = true;

  bool prevLmb_ = false;
  bool prevF_ = false;
  std::optional<VoxelHit> lastHit_;
};
