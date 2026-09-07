#pragma once

#include "core/Camera.h"
#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"
#include "physics/PhysicsWorld.h"
#include "physics/VoxelCollide.h"
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

// Must match shaders/voxel_dda.comp std430 GpuVoxelObject (208 bytes).
struct GpuVoxelObject {
  float worldToObject[16];
  float objectToWorld[16];
  float voxelSize;
  float _pad0[3];
  uint32_t gridSize[3];
  uint32_t flags;  // bit0 nestedMicro, bit1 enabled, bit2 import color, bit3 nestedFine
  uint32_t voxelOffset;  // coarsePool cell index (start of this object's N³)
  uint32_t occMipOffset;  // 4^3 coarse tiles: two uints (64 Morton bits) each
  uint32_t occMipWords;   // tile count * 2
  uint32_t cpuIndex;
  float occMin[3];  // coarse inclusive
  float _padOccMin;
  float occMax[3];  // coarse exclusive
  float _padOccMax;
};
static_assert(sizeof(GpuVoxelObject) == 208, "GpuVoxelObject std430 size mismatch");

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
  static constexpr uint32_t kFlagNestedFine = 8u;

  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  // Coarse cell meters. Default is 16 * 0.1 m so one fine cell matches Teardown.
  float voxelSize = 1.6f;
  int gridSize = 16;
  bool nestedMicro = true;
  bool editable = true;
  bool enabled = true;
  bool useImportPalette = false;

  std::vector<CoarseCell> cells;
  uint32_t voxelOffset = 0;  // coarsePool cell index (start of this object's N³)
  uint32_t occMipOffset = 0;
  uint32_t occMipWords = 0;
  glm::vec3 occMin{0.0f};
  glm::vec3 occMax{0.0f};
  // Packed coarse cells (x | y<<10 | z<<20) for vis raster. Occupancy AABB is too loose
  // after a cut: the box covers holes, vis commits this id, DDA misses, sky leaks.
  std::vector<uint32_t> occupiedCoarses;

  glm::mat4 objectToWorld() const;
  glm::mat4 worldToObject() const;
};

struct VisCoarseInstance {
  uint32_t gpuIndex = 0;
  uint32_t packedCoarse = 0;
};
static_assert(sizeof(VisCoarseInstance) == 8, "VisCoarseInstance vertex stride");

class VoxelScene {
public:
  static constexpr int kMicroRes = 8;
  static constexpr int kMicroCount = kMicroRes * kMicroRes * kMicroRes;
  static constexpr int kMicroWords = kMicroCount / 32;  // Morton-ordered occupancy bits
  static constexpr int kFineRes = 2;
  static constexpr int kFineCount = kFineRes * kFineRes * kFineRes;
  static constexpr int kFinePerCoarse = kMicroRes * kFineRes;  // 16
  // Visible shape is the fine cell. Physics must use the same 0.1 m grain as Teardown.
  static constexpr float kGameplayVoxelMeters = 0.1f;
  static constexpr float kDefaultVoxelSize =
      kGameplayVoxelMeters * static_cast<float>(kFinePerCoarse);
  static constexpr int kFineTableBytes = kMicroCount;  // one uint8 per 8^3 micro
  static constexpr int kFineWordsPerTable = kFineTableBytes / 4;
  static constexpr int kFinePerBrick = kFinePerCoarse * kFinePerCoarse * kFinePerCoarse;  // 4096
  // Occupancy then one 0xAARRGGBB per fine. Alpha != 0 means this fine has a sampled color.
  static constexpr int kFineColorOffset = kMicroWords + kFineWordsPerTable;
  static constexpr int kFineColorWords = kFinePerBrick;
  static constexpr int kBrickPageWords = kFineColorOffset + kFineColorWords;
  static constexpr uint32_t kInvalidBrickPage = 0xFFFFFFFFu;
  static constexpr uint32_t kPagesPerSlab = 2048u;
  static constexpr uint32_t kMaxBrickSlabs = 8u;
  static constexpr uint32_t kWordsPerSlab =
      kPagesPerSlab * static_cast<uint32_t>(kBrickPageWords);
  static constexpr int kOccMipRes = 4;
  static constexpr int kOccMipShift = 2;
  static constexpr uint32_t kMaxShapes = 256;

  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void update(float dt);
  void handleEditInput(GLFWwindow* window, GfxDevice& gfx);
  void rebuildVoxels(GfxDevice& gfx);
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
  const AllocatedBuffer& heatmapBuffer() const { return heatmapBuffer_; }
  void uploadBondHeatmap(GfxDevice& gfx);
  uint32_t poolCellIndex(int objectIndex, const glm::ivec3& coarse) const;
  uint32_t occMipBytes() const {
    return static_cast<uint32_t>(occMipCpu_.size() * sizeof(uint32_t));
  }

  Camera& camera() { return camera_; }
  const Camera& camera() const { return camera_; }

  const AllocatedBuffer& dummyBrickSlabBuffer() const { return dummyBrickSlabBuffer_; }
  const AllocatedBuffer& objectBuffer() const { return objectBuffer_; }
  const AllocatedBuffer& visInstanceBuffer() const { return visInstanceBuffer_; }
  uint32_t visInstanceCount() const { return static_cast<uint32_t>(uploadedVisInstances_.size()); }
  const AllocatedBuffer& coarsePoolBuffer() const { return coarsePoolBuffer_; }
  VkDeviceSize coarsePoolBytes() const { return coarsePoolBuffer_.size; }
  uint32_t brickSlabCount() const { return static_cast<uint32_t>(slabs_.size()); }
  const AllocatedBuffer& brickSlabBuffer(uint32_t i) const { return slabs_[i].gpu; }
  uint32_t objectCount() const { return static_cast<uint32_t>(objectsGpu_.size()); }
  uint32_t objectFlags(uint32_t i) const { return objectsGpu_.at(i).flags; }

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
  float microVoxelSize() const { return voxelSize_ / static_cast<float>(kMicroRes); }
  float fineVoxelSize() const { return voxelSize_ / static_cast<float>(kFinePerCoarse); }
  float gameplayVoxelSize() const { return fineVoxelSize(); }
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
  bool& solidColorOutput() { return solidColorOutput_; }
  glm::vec3& solidColor() { return solidColor_; }
  int& brushMaterial() { return brushMaterial_; }
  float& brushRadius() { return brushRadius_; }
  bool& nestedMicroVoxels() { return nestedMicroVoxels_; }
  bool& nestedFineVoxels() { return nestedFineVoxels_; }
  bool& collapseFullBricks() { return collapseFullBricks_; }
  float& spinSpeed() { return spinSpeed_; }
  bool& spinnerEnabled() { return spinnerEnabled_; }
  int debrisCount() const { return debrisCount_; }
  void setDebrisCount(GfxDevice& gfx, int count);

  bool simulate() const { return simulate_; }
  void setSimulate(GfxDevice& gfx, bool on);
  int cpuObjectCount() const { return static_cast<int>(objects_.size()); }
  const VoxelObject& cpuObject(int i) const { return objects_.at(static_cast<size_t>(i)); }
  VoxelObject& cpuObject(int i) { return objects_.at(static_cast<size_t>(i)); }
  bool occupancyFine(int objectIndex, const glm::ivec3& coarse, const glm::ivec3& micro,
                     const glm::ivec3& fine) const;
  uint32_t occupancyMaterial(int objectIndex, const glm::ivec3& coarse) const;
  uint32_t coarseBrickPage(int objectIndex, const glm::ivec3& coarse) const;
  void collectOccupiedFines(int objectIndex, const glm::ivec3& coarse,
                            std::vector<glm::ivec3>& out) const;
  uint32_t occupiedFineCount(int objectIndex, const glm::ivec3& coarse) const;
  void notifyOccupancyChanged(int objectIndex);
  bool fractureFromCuts(int objectIndex, const std::vector<glm::ivec3>& deletedAbsFines);
  int cutCoarseInterface(int objectIndex, const glm::ivec3& coarseA, const glm::ivec3& coarseB,
                         std::vector<glm::ivec3>* deletedOut);
  void peelCoarseIslands(int objectIndex, const std::vector<glm::ivec3>& coarses);
  void noteOccupancyGpuDirty();
  void flushOccupancyGpu(GfxDevice& gfx);
  const std::vector<physics::DebugBond>& structureDebugBonds() const;
  uint32_t physicsCornerCount(int objectIndex) const;
  uint32_t physicsEdgeCount(int objectIndex) const;
  physics::DebugSolve physicsDebug() const { return physics_.debugSolve(); }
  void gatherCornerNormals(int fromObj, int againstObj,
                           std::vector<physics::DebugCornerNormal>& out) const;

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
                  const glm::ivec3& fine, bool solid, bool writeRgb = false,
                  uint32_t rgb888 = 0);
  bool brickPageEmpty(uint32_t page) const;
  bool brickPageFull(uint32_t page) const;
  void tryCollapseFullBrick(VoxelObject& o, uint32_t coarseIndex);
  uint32_t allocBrickPage(const uint32_t* words16);
  void freeBrickPage(uint32_t page);
  uint32_t ensureBrickPage(VoxelObject& o, uint32_t coarseIndex, bool fillSolid);
  void fillFineFromOccupancy(uint32_t page);
  void ensureCoarseBrick(VoxelObject& o, const glm::ivec3& coarse, uint32_t material);
  void recountOccupiedMicro();
  void recountOccupiedFine();

  void buildGroundObject(VoxelObject& o);
  void buildSpinnerObject(VoxelObject& o);
  void buildTestBoxObject(VoxelObject& o);
  void buildDebrisObject(VoxelObject& o, int debrisIndex, int debrisTotal);
  void clearObjectPages(VoxelObject& o);
  void spawnDebrisObjects();
  uint32_t stampMeshIntoWorld(const MeshVoxelizeResult& r, bool sampleColor);
  void uploadWorldAndObjects(GfxDevice& gfx);
  void packObjectPool();
  void fillCoarseDirTiles();
  void fillGpuObjectRecords();
  void uploadOccMip(GfxDevice& gfx);
  void uploadPalette(GfxDevice& gfx);
  uint32_t* brickPageWords(uint32_t page);
  const uint32_t* brickPageWords(uint32_t page) const;
  uint8_t readFineByte(uint32_t page, uint32_t microBit) const;
  void writeFineByte(uint32_t page, uint32_t microBit, uint8_t value);
  uint32_t fineColorIndex(const glm::ivec3& micro, const glm::ivec3& fine) const;
  void writeFineRgb(uint32_t page, uint32_t colorIndex, uint32_t rgb888);
  void ensureSlabCpu(uint32_t slabIndex);
  void ensureGpuBuffers(GfxDevice& gfx);
  void uploadCoarsePool(GfxDevice& gfx);
  void uploadVisInstances(GfxDevice& gfx);
  void flushObject(GfxDevice& gfx, int objectIndex);
  void flushDirtyPages(GfxDevice& gfx);
  bool maybeFracture(int objectIndex, const std::vector<glm::ivec3>& deletedAbsFines);
  bool solidAbsFine(const VoxelObject& o, const glm::ivec3& absFine) const;
  void emitFracturePiece(int srcIndex, const std::vector<uint32_t>& packedFines);
  void clearPackedFines(VoxelObject& o, const std::vector<uint32_t>& packedFines);

  int applyCoarseSphereBrush(VoxelObject& o, const glm::ivec3& center, float radius,
                             uint32_t material, bool placeOnlyEmpty,
                             std::vector<glm::ivec3>* deletedAbsFines = nullptr);
  int applyMicroSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                            float radius, bool solid, uint32_t placeMaterial,
                            std::vector<glm::ivec3>* deletedAbsFines = nullptr);
  int applyFineSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                           const glm::ivec3& fine, float radius, bool solid, uint32_t placeMaterial,
                           std::vector<glm::ivec3>* deletedAbsFines = nullptr);

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
  AllocatedBuffer dummyBrickSlabBuffer_{};
  AllocatedBuffer objectBuffer_{};
  AllocatedBuffer visInstanceBuffer_{};
  AllocatedBuffer coarsePoolBuffer_{};
  std::vector<CoarseCell> coarsePoolCpu_{};
  AllocatedBuffer paletteBuffer_{};
  AllocatedBuffer occMipBuffer_{};
  AllocatedBuffer heatmapBuffer_{};
  std::vector<float> heatmapCpu_;
  MeshVoxelizerGpu voxelizeGpu_{};
  std::array<glm::vec4, 256> importPalette_{};
  std::string importPath_;
  std::string lastImportedPath_;
  std::string importStatus_{"No import"};
  int importGridN_ = 64;
  int importPadding_ = 1;
  bool importSampleColor_ = false;
  bool importConservative_ = true;

  std::vector<VoxelObject> objects_;
  std::vector<GpuVoxelObject> objectsGpu_;
  std::vector<GpuVoxelObject> uploadedObjectsGpu_;
  std::vector<VisCoarseInstance> visInstancesCpu_;
  std::vector<VisCoarseInstance> uploadedVisInstances_;
  std::vector<uint32_t> occMipCpu_;
  std::vector<BrickSlab> slabs_;
  std::vector<uint32_t> freePages_;
  std::unordered_set<uint32_t> dirtyPages_;
  uint32_t nextPage_ = 0;
  uint32_t allocatedPageCount_ = 0;

  int gridSize_ = 64;
  float voxelSize_ = kDefaultVoxelSize;
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
  bool solidColorOutput_ = false;
  glm::vec3 solidColor_{0.62f, 0.64f, 0.68f};
  int brushMaterial_ = 1;
  float brushRadius_ = 0.0f;
  bool nestedMicroVoxels_ = true;
  bool nestedFineVoxels_ = true;
  bool collapseFullBricks_ = true;
  float time_ = 0.0f;
  float spinSpeed_ = 0.8f;
  bool spinnerEnabled_ = false;
  bool simulate_ = false;
  int debrisCount_ = 8;
  physics::PhysicsWorld physics_;

  bool prevLmb_ = false;
  bool prevF_ = false;
  bool occupancyGpuDirty_ = false;
  std::optional<VoxelHit> lastHit_;
};
