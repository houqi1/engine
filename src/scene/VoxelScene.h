#pragma once

#include "core/Camera.h"
#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"
#include "voxel/MeshVoxelizer.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

struct GLFWwindow;
class GfxDevice;

struct VoxelHit {
  glm::ivec3 cell{0};
  glm::ivec3 micro{0};
  glm::ivec3 fine{0};
  glm::ivec3 normal{0};
  uint32_t material = 0;
  bool hasMicro = false;
  bool hasFine = false;
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
  uint32_t voxelOffset;  // grids[] texture index (0 = world, 1 = spinner)
  uint32_t occMipOffset;  // occupancy mip, in uints
  uint32_t occMipWords;
  uint32_t _pad1;
};
static_assert(sizeof(GpuVoxelObject) == 176, "GpuVoxelObject std430 size mismatch");

// Must match shaders/voxel_dda.comp std430 CoarseCell.
struct CoarseCell {
  uint32_t material = 0;                              // 0 = air
  uint32_t brickPage = 0xFFFFFFFFu;                   // INVALID = no brick page
};
static_assert(sizeof(CoarseCell) == 8, "CoarseCell size mismatch");

struct VoxelObject {
  static constexpr uint32_t kFlagNestedMicro = 1u;
  static constexpr uint32_t kFlagEnabled = 2u;
  static constexpr uint32_t kFlagImportPalette = 4u;

  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  float voxelSize = 0.35f;
  int gridSize = 16;
  bool nestedMicro = true;
  bool editable = true;
  bool enabled = true;
  bool useImportPalette = false;

  std::vector<CoarseCell> cells;
  uint32_t voxelOffset = 0;  // grids[] texture index (0 = world, 1 = spinner)
  uint32_t occMipOffset = 0;
  uint32_t occMipWords = 0;

  glm::mat4 objectToWorld() const;
  glm::mat4 worldToObject() const;
};

class VoxelScene {
public:
  static constexpr int kMicroRes = 8;
  static constexpr int kMicroCount = kMicroRes * kMicroRes * kMicroRes;
  static constexpr int kMicroWords = kMicroCount / 32;
  static constexpr int kFineRes = 2;
  static constexpr int kFineCount = kFineRes * kFineRes * kFineRes;
  static constexpr int kFineTableBytes = kMicroCount;  // one uint8 per 8^3 micro
  static constexpr int kFineWordsPerTable = kFineTableBytes / 4;
  static constexpr int kBrickPageWords = kMicroWords + kFineWordsPerTable;
  static constexpr uint32_t kInvalidBrickPage = 0xFFFFFFFFu;
  static constexpr uint32_t kPagesPerSlab = 1024u;
  static constexpr uint32_t kMaxBrickSlabs = 8u;
  static constexpr uint32_t kWordsPerSlab =
      kPagesPerSlab * static_cast<uint32_t>(kBrickPageWords);
  static constexpr int kOccMipRes = 4;
  static constexpr int kOccMipShift = 2;
  static constexpr uint32_t kGridTexCount = 2;
  static constexpr VkFormat kGridFormat = VK_FORMAT_R32G32_UINT;

  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void update(float dt);
  void handleEditInput(GLFWwindow* window, GfxDevice& gfx);
  void rebuildVoxels(GfxDevice& gfx);
  void rebuildOccupancyHull(GfxDevice& gfx);
  void uploadObjectTransforms(GfxDevice& gfx);
  bool importSurfaceMesh(GfxDevice& gfx, const std::string& path, const MeshVoxelizeConfig& cfg);
  void removeImportedMesh(GfxDevice& gfx);
  const std::string& importStatus() const { return importStatus_; }
  std::string& importPath() { return importPath_; }
  int& importGridN() { return importGridN_; }
  int& importPadding() { return importPadding_; }
  bool& importSampleColor() { return importSampleColor_; }
  const AllocatedBuffer& paletteBuffer() const { return paletteBuffer_; }
  const AllocatedBuffer& occMipBuffer() const { return occMipBuffer_; }
  uint32_t occMipBytes() const {
    return static_cast<uint32_t>(occMipCpu_.size() * sizeof(uint32_t));
  }

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }

  const AllocatedImage& gridImage(uint32_t i) const {
    return i < kGridTexCount ? grid3D_[i] : dummyGrid3D_;
  }
  const AllocatedImage& dummyGridImage() const { return dummyGrid3D_; }
  VkSampler gridSampler() const { return gridSampler_; }
  const AllocatedBuffer& dummyBrickSlabBuffer() const { return dummyBrickSlabBuffer_; }
  const AllocatedBuffer& hullVertexBuffer() const { return hullVertexBuffer_; }
  const AllocatedBuffer& hullIndexBuffer() const { return hullIndexBuffer_; }
  uint32_t hullIndexCount() const { return hullIndexCount_; }
  const AllocatedBuffer& spinnerObbVertexBuffer() const { return spinnerObbVertexBuffer_; }
  const AllocatedBuffer& spinnerObbIndexBuffer() const { return spinnerObbIndexBuffer_; }
  uint32_t spinnerObbIndexCount() const { return spinnerObbIndexCount_; }
  bool& hullConservativeDilate() { return hullDilate_; }
  const AllocatedBuffer& objectBuffer() const { return objectBuffer_; }
  uint32_t brickSlabCount() const { return static_cast<uint32_t>(slabs_.size()); }
  const AllocatedBuffer& brickSlabBuffer(uint32_t i) const { return slabs_[i].gpu; }
  uint32_t objectCount() const { return static_cast<uint32_t>(objectsGpu_.size()); }

  uint32_t voxelCount() const;
  uint32_t occupiedCount() const { return occupiedCount_; }
  uint32_t occupiedMicroCount() const { return occupiedMicroCount_; }
  uint32_t occupiedFineCount() const { return occupiedFineCount_; }
  uint32_t allocatedBrickPages() const { return allocatedPageCount_; }
  uint32_t brickPoolBytes() const {
    return static_cast<uint32_t>(slabs_.size() * kWordsPerSlab * sizeof(uint32_t));
  }

  int& gridSize() { return gridSize_; }
  float& voxelSize() { return voxelSize_; }
  glm::vec3& lightDir() { return lightDir_; }
  glm::vec3 gridOrigin() const;
  glm::uvec3 gridDims() const { return glm::uvec3(static_cast<uint32_t>(gridSize_)); }

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
  bool spinnerDrawEnabled() const {
    return spinnerEnabled_ && objects_.size() >= 2 && objects_[1].enabled;
  }
  glm::mat4 spinnerObjectToWorld() const {
    return objects_.size() >= 2 ? objects_[1].objectToWorld() : glm::mat4(1.0f);
  }
  bool cameraInsideWorldAabb() const;

  std::optional<VoxelHit> lastHit() const { return lastHit_; }

private:
  bool inBounds(const VoxelObject& o, const glm::ivec3& p) const;
  uint32_t indexOf(const VoxelObject& o, const glm::ivec3& p) const;
  uint32_t getVoxel(const VoxelObject& o, const glm::ivec3& p) const;
  CoarseCell& cellAt(VoxelObject& o, uint32_t idx);
  const CoarseCell& cellAt(const VoxelObject& o, uint32_t idx) const;

  bool microInBounds(const glm::ivec3& m) const;
  bool fineInBounds(const glm::ivec3& f) const;
  uint32_t microBitIndex(const glm::ivec3& m) const;
  uint32_t fineBitIndex(const glm::ivec3& f) const;
  bool getMicro(const VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro) const;
  bool getFine(const VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
               const glm::ivec3& fine) const;
  bool setVoxelCpu(VoxelObject& o, const glm::ivec3& p, uint32_t material);
  bool setMicroCpu(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro, bool solid);
  bool setFineCpu(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                  const glm::ivec3& fine, bool solid);
  bool brickPageEmpty(uint32_t page) const;
  uint32_t allocBrickPage(const uint32_t* words16);
  void freeBrickPage(uint32_t page);
  uint32_t ensureBrickPage(VoxelObject& o, uint32_t coarseIndex, bool fillSolid);
  void fillFineFromOccupancy(uint32_t page);
  void ensureCoarseBrick(VoxelObject& o, const glm::ivec3& coarse, uint32_t material);
  void recountOccupiedMicro();
  void recountOccupiedFine();

  void buildGroundObject(VoxelObject& o);
  void buildSpinnerObject(VoxelObject& o);
  uint32_t stampMeshIntoWorld(const MeshVoxelizeResult& r, bool sampleColor);
  void uploadWorldAndObjects(GfxDevice& gfx);
  void packObjectPool();
  void fillGpuObjectRecords();
  void fillOccMip(const VoxelObject& o, uint32_t* words, uint32_t wordCount) const;
  void uploadOccMip(GfxDevice& gfx);
  void uploadPalette(GfxDevice& gfx);
  uint32_t* brickPageWords(uint32_t page);
  const uint32_t* brickPageWords(uint32_t page) const;
  uint8_t readFineByte(uint32_t page, uint32_t microBit) const;
  void writeFineByte(uint32_t page, uint32_t microBit, uint8_t value);
  void ensureSlabCpu(uint32_t slabIndex);
  void ensureGpuBuffers(GfxDevice& gfx);
  void ensureCoarseGridFormat(GfxDevice& gfx);
  void ensureGridImages(GfxDevice& gfx);
  void uploadGridImage(GfxDevice& gfx, uint32_t objectIndex);
  void destroyGridImages(GfxDevice& gfx);
  void flushObject(GfxDevice& gfx, int objectIndex);
  void flushDirtyPages(GfxDevice& gfx);
  void destroyOccupancyHull(GfxDevice& gfx);

  int applyCoarseSphereBrush(VoxelObject& o, const glm::ivec3& center, float radius,
                             uint32_t material, bool placeOnlyEmpty);
  int applyMicroSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                            float radius, bool solid, uint32_t placeMaterial);
  int applyFineSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                           const glm::ivec3& fine, float radius, bool solid, uint32_t placeMaterial);

  struct PickResult {
    VoxelHit hit{};
    float tWorld = 0.0f;
  };
  std::optional<PickResult> pickObject(const VoxelObject& o, int objectIndex, const glm::vec3& Ow,
                                       const glm::vec3& Dw) const;
  std::optional<VoxelHit> pickCenterRay() const;

  struct BrickSlab {
    std::vector<uint32_t> words;
    AllocatedBuffer gpu{};
  };
  Camera camera_;
  std::array<AllocatedImage, kGridTexCount> grid3D_{};
  AllocatedImage dummyGrid3D_{};
  VkSampler gridSampler_ = VK_NULL_HANDLE;
  bool gridFormatChecked_ = false;
  AllocatedBuffer dummyBrickSlabBuffer_{};
  AllocatedBuffer objectBuffer_{};
  AllocatedBuffer paletteBuffer_{};
  AllocatedBuffer occMipBuffer_{};
  AllocatedBuffer hullVertexBuffer_{};
  AllocatedBuffer hullIndexBuffer_{};
  uint32_t hullIndexCount_ = 0;
  AllocatedBuffer spinnerObbVertexBuffer_{};
  AllocatedBuffer spinnerObbIndexBuffer_{};
  uint32_t spinnerObbIndexCount_ = 0;
  bool hullDilate_ = true;
  MeshVoxelizerGpu voxelizeGpu_{};
  std::array<glm::vec4, 256> importPalette_{};
  std::string importPath_;
  std::string lastImportedPath_;
  std::string importStatus_{"No import"};
  int importGridN_ = 48;
  int importPadding_ = 1;
  bool importSampleColor_ = false;
  bool importConservative_ = true;

  std::vector<VoxelObject> objects_;
  std::vector<GpuVoxelObject> objectsGpu_;
  std::vector<uint32_t> occMipCpu_;
  std::vector<BrickSlab> slabs_;
  std::vector<uint32_t> freePages_;
  std::unordered_set<uint32_t> dirtyPages_;
  uint32_t nextPage_ = 0;
  uint32_t allocatedPageCount_ = 0;

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
  uint32_t occupiedFineCount_ = 0;
  int renderMode_ = 0;
  int brushMaterial_ = 1;
  float brushRadius_ = 0.0f;
  bool nestedMicroVoxels_ = true;
  float time_ = 0.0f;
  float spinSpeed_ = 0.8f;
  bool spinnerEnabled_ = false;

  bool prevLmb_ = false;
  bool prevF_ = false;
  std::optional<VoxelHit> lastHit_;
};
