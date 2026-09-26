#pragma once

#include "blast/GroundAnchors.h"
#include "blast/StructureWorld.h"
#include "core/Camera.h"
#include "gfx/GfxDevice.h"
#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"
#include "physics/PhysicsWorld.h"
#include "physics/VoxelCollide.h"
#include "scene/VoxelTypes.h"
#include "voxel/MeshVoxelizer.h"
#include "voxel/VoxelConnectivity.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

struct GLFWwindow;

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
  uint32_t voxelOffset;  // coarse cell index into shared CoarsePool
  uint32_t occMipOffset;  // 4^3 coarse tiles: two uints (64 Morton bits) each
  uint32_t occMipWords;   // tile count * 2
  uint32_t _pad1;
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
  // Slot is live in the object table (distinct from enabled, which is visibility/sim).
  bool slotOccupied = true;
  bool isScatter = false;
  bool isFineCollideProbe = false;
  MotionType motionType = MotionType::Dynamic;
  float density = 600.0f;
  // Bumped when occupancy, material connectivity rules, or anchors change.
  uint64_t topologyRevision = 1;
  // Parent-grid fine origin of local (0,0,0). Graph voxels stay in the family sample grid.
  glm::ivec3 structureFineOrigin{0};

  std::vector<CoarseCell> cells;
  uint32_t voxelOffset = 0;  // coarse cell index into shared CoarsePool
  uint32_t occMipOffset = 0;
  uint32_t occMipWords = 0;
  glm::vec3 occMin{0.0f};
  glm::vec3 occMax{0.0f};

  glm::mat4 objectToWorld() const;
  glm::mat4 worldToObject() const;
};

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
  static constexpr uint32_t kGridTexCount = 2;
  static constexpr VkFormat kGridFormat = VK_FORMAT_R32G32_UINT;
  // Soft cap for scatter / fracture objects (Phase 3 scale tests).
  static constexpr uint32_t kMaxVoxelObjects = 1024;

  VoxelScene() = default;
  ~VoxelScene();
  VoxelScene(const VoxelScene&) = delete;
  VoxelScene& operator=(const VoxelScene&) = delete;

  void init(GfxDevice& gfx, blast::BlastRuntime& runtime);
  void cleanup(GfxDevice& gfx);
  void update(float dt);
  void handleEditInput(GLFWwindow* window, GfxDevice& gfx);
  // Dig → connectivity → split commit. See advance/commit in the frame loop.
  void advanceFractureWork(float dt);
  void commitReadyFractures(GfxDevice& gfx);
  bool fractureEnabled() const { return fractureEnabled_; }
  bool& fractureEnabled() { return fractureEnabled_; }
  void rebuildVoxels(GfxDevice& gfx);
  // Keep ground (+ optional spinner/test box), append `count` small solid boxes for scale tests.
  void spawnScatterBoxes(GfxDevice& gfx, uint32_t count, bool pile = false);
  void clearScatterBoxes(GfxDevice& gfx);
  void uploadObjectTransforms(GfxDevice& gfx, uint32_t frameIndex);
  void uploadObjectTransforms(GfxDevice& gfx) { uploadObjectTransforms(gfx, 0); }
  void uploadCoarsePool(GfxDevice& gfx);
  bool importSurfaceMesh(GfxDevice& gfx, const std::string& path, const MeshVoxelizeConfig& cfg);
  void removeImportedMesh(GfxDevice& gfx);
  const std::string& importStatus() const { return importStatus_; }
  // E5.5: voxelize a mesh into its own static object standing on the ground (lowest
  // occupied layer on the ground top, centred in x/z). With importMount() it is mounted
  // as a stress structure anchored where it touches the ground.
  bool importMeshAsObject(GfxDevice& gfx, const std::string& path, const MeshVoxelizeConfig& cfg);
  bool& importAsObject() { return importAsObject_; }
  bool& importMount() { return importMount_; }
  float& importDensity() { return importDensity_; }
  float& importStrengthMPa() { return importStrengthMPa_; }  // the import's own strength
  int& importAgg() { return importAgg_; }  // 0 = auto (2, or 4 when over the node budget)
  VoxelObjectId importedObjectId() const { return importedObjectId_; }
  // Structure shown by the panel and stress / anchor colors: the last spawned demo or import.
  VoxelObjectId structureFocusId() const;
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
  const AllocatedBuffer& coarsePoolBuffer() const { return coarsePoolBuffer_; }
  const AllocatedBuffer& objectBuffer(uint32_t frameIndex) const {
    return objectFrameBuffers_[frameIndex % GfxDevice::kFramesInFlight];
  }
  const AllocatedBuffer& objectBuffer() const { return objectBuffer(0); }
  uint32_t brickSlabCount() const { return static_cast<uint32_t>(slabs_.size()); }
  const AllocatedBuffer& brickSlabBuffer(uint32_t i) const { return slabs_[i].gpu; }
  uint32_t objectCount() const { return static_cast<uint32_t>(objectsGpu_.size()); }
  uint32_t objectFlags(uint32_t i) const { return objectsGpu_.at(i).flags; }
  const std::vector<GpuVoxelObject>& gpuObjects() const { return objectsGpu_; }
  // Bumped when GPU buffer handles are recreated; renderer must refresh descriptors.
  uint32_t gpuResourceSerial() const { return gpuResourceSerial_; }

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

  bool simulate() const { return simulate_; }
  void setSimulate(GfxDevice& gfx, bool on);
  bool spawnTestBoxOnSimulate() const { return spawnTestBoxOnSimulate_; }
  void setSpawnTestBoxOnSimulate(GfxDevice& gfx, bool on);
  bool fineCollideProbes() const { return fineCollideProbes_; }
  void setFineCollideProbes(GfxDevice& gfx, bool on);
  bool getBodyState(VoxelObjectId id, physics::BodyState& out) const;
  int cpuObjectCount() const { return static_cast<int>(objects_.size()); }
  const VoxelObject& cpuObject(int i) const { return objects_.at(static_cast<size_t>(i)); }
  VoxelObject& cpuObject(int i) { return objects_.at(static_cast<size_t>(i)); }
  bool slotOccupied(int i) const {
    return i >= 0 && i < static_cast<int>(objects_.size()) && objects_[static_cast<size_t>(i)].slotOccupied;
  }
  VoxelObjectId objectIdAt(int slot) const;
  VoxelObjectId groundObjectId() const { return groundObjectId_; }
  VoxelObjectId testObjectId() const { return testObjectId_; }
  VoxelObject* tryGetObject(VoxelObjectId id);
  const VoxelObject* tryGetObject(VoxelObjectId id) const;
  bool occupancyFine(int objectIndex, const glm::ivec3& coarse, const glm::ivec3& micro,
                     const glm::ivec3& fine) const;
  uint32_t occupancyMaterial(int objectIndex, const glm::ivec3& coarse) const;
  uint32_t coarseBrickPage(int objectIndex, const glm::ivec3& coarse) const;
  void collectOccupiedFines(int objectIndex, const glm::ivec3& coarse,
                            std::vector<glm::ivec3>& out) const;
  void notifyOccupancyChanged(int objectIndex);
  uint32_t physicsCornerCount(int objectIndex) const;
  uint32_t physicsEdgeCount(int objectIndex) const;
  const physics::DebugSolve& physicsDebug() const { return physics_.debugSolve(); }
  blast::StructureWorld& structures() { return structures_; }
  const blast::StructureWorld& structures() const { return structures_; }
  VoxelObjectId stressCylinderId() const { return stressCylinderId_; }
  // Structure mounted for the focused object (stress demo or imported mesh).
  blast::StructureInstance* stressStructure() { return structures_.find(structureFocusId()); }
  const blast::StructureInstance* stressStructure() const { return structures_.find(structureFocusId()); }
  // Mounts one voxel object's whole fine grid as a stress structure, replacing any
  // structure already mounted for it. Density is the object's; anchors come from
  // anchorFine (object-local fine coordinates). Returns nullptr on failure.
  // allowFloating (E5.3): accept no world bonds and disconnected islands; islands are
  // split right away and become their own bodies on the next structure commit.
  blast::StructureInstance* mountObjectStructure(VoxelObjectId id, int agg,
                                                 const std::function<bool(int, int, int)>& anchorFine,
                                                 blast::StructureMountDesc desc, bool allowFloating = false);
  // E5.3: a dynamic object as a free body. No world bonds, no fake anchor; the solver
  // only sees centrifugal and impact loads.
  blast::StructureInstance* mountObjectFree(VoxelObjectId id, int agg, blast::StructureMountDesc desc);
  // Static objects anchor on the ground they touch, dynamic objects mount free.
  blast::StructureInstance* mountObjectAuto(VoxelObjectId id, int agg, blast::StructureMountDesc desc);
  // E5.3 demos on the stress demo slot (replace the cylinder / frame).
  bool spawnFreePlank(GfxDevice& gfx);
  bool spawnBlockWithFloatingPart(GfxDevice& gfx);
  // E5.4 contact-load demos. The beam rests on two static piers 3.2 m apart; with
  // anchoredReference it is a static beam anchored on the pier tops instead (T06 reference).
  bool spawnBeamOnPiers(GfxDevice& gfx, bool anchoredReference, float dropHeight = 0.01f);
  // Cuts the top half of the beam at mid-span, as digging would (structure rebuild).
  bool notchDemoBeam(GfxDevice& gfx);
  // A dynamic 0.4 m cube (not a structure) dropped on a four-column roof beam, and its removal.
  bool dropWeightOnRoof(GfxDevice& gfx, float density);
  void removeDemoWeight(GfxDevice& gfx);
  VoxelObjectId demoWeightId() const { return demoWeightId_; }
  // Resting beam peaks near 0.096 MPa, notched near 0.153 MPa: holds intact, breaks notched.
  static constexpr float kBeamDemoStrengthPa = 1.2e5f;
  // E5.2: fines whose exposed faces rest on unmounted static objects (the ground).
  // Faces against another mounted structure are counted as blocked.
  blast::GroundAnchorResult findGroundContactAnchors(VoxelObjectId id) const;
  // Mounts a static object anchored where it actually touches the ground. Refuses
  // objects that touch no ground or touch another mounted structure.
  blast::StructureInstance* mountObjectOnGround(VoxelObjectId id, int agg, blast::StructureMountDesc desc);
  // Stress demo spawns/cuts use ground-contact anchors instead of the explicit base rule.
  void setGroundContactAnchors(bool on) { groundContactAnchors_ = on; }
  bool groundContactAnchors() const { return groundContactAnchors_; }
  const std::string& structureMountStatus() const { return structureMountStatus_; }
  const blast::GroundAnchorResult& lastGroundAnchors() const { return lastGroundAnchors_; }
  void setAnchorDisplay(GfxDevice& gfx, bool on);
  bool anchorDisplay() const { return anchorDisplay_; }
  bool spawnStressCylinder(GfxDevice& gfx);
  bool resetStressCylinder(GfxDevice& gfx);
  bool cutStressCylinder270(GfxDevice& gfx);
  bool spawnStressFrame(GfxDevice& gfx, float columnHeightMeters = 4.0f);
  float frameColumnHeightMeters() const { return frameColumnHeightFines_ * 0.1f; }
  bool cutThreeColumns(GfxDevice& gfx);
  // Raise all dynamic structure pieces so the lowest returns near the original roof
  // height, zero velocities, and wake them to fall again (for accumulating Impact Damage).
  bool liftStructureForRedrop();
  bool setStressCylinderDoubleDensity(bool on);
  void setStressCylinderSolverIters(uint32_t iters);
  void setStressCylinderDisplay(GfxDevice& gfx, bool on);
  void setBondDamageDisplay(GfxDevice& gfx, bool on);
  bool bondDamageDisplay() const { return bondDamageDisplay_; }
  // After beginFrame: upload stress or bond-damage colors without waitIdle.
  void refreshStressColors(GfxDevice& gfx);
  void commitStructureSplits(GfxDevice& gfx);
  void commitStructureSplit(GfxDevice& gfx, blast::StructureInstance& inst);
  bool stressCylinderCut() const { return stressCylinderCut_; }
  bool stressCylinderDoubleDensity() const { return stressCylinderDoubleDensity_; }
  bool stressCylinderDisplay() const { return stressCylinderDisplay_; }
  void splitFineIndex(int fx, int fy, int fz, glm::ivec3& coarse, glm::ivec3& micro, glm::ivec3& fine) const;
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
  void fillTestSlot(VoxelObject& o);
  void clearFineCollideProbes(GfxDevice& gfx);
  void spawnFineCollideProbes(GfxDevice& gfx);
  void rebuildTestSlot(GfxDevice& gfx);
  void clearObjectPages(VoxelObject& o);
  uint32_t allocObjectSlot();
  void freeObjectSlot(uint32_t slot);
  void resetObjectTable(uint32_t reservedSlots);
  VoxelObjectId makeObjectId(uint32_t slot) const;
  VoxelObject* mutableObject(VoxelObjectId id);
  uint32_t stampMeshIntoWorld(const MeshVoxelizeResult& r, bool sampleColor);
  void uploadWorldAndObjects(GfxDevice& gfx);
  void packObjectPool();
  void packCoarsePool();
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
  void ensureCoarseGridFormat(GfxDevice& gfx);
  void ensureGridImages(GfxDevice& gfx);
  void uploadGridImage(GfxDevice& gfx, uint32_t objectIndex);
  void destroyGridImages(GfxDevice& gfx);
  void flushObject(GfxDevice& gfx, int objectIndex);
  void flushDirtyPages(GfxDevice& gfx);
  void bindStructureTicks(GfxDevice& gfx);
  GfxDevice* structureGfx_ = nullptr;
  void stopStructureTicks();
  void resetStructureSession();
  bool mountCylinderFromOccupancy(GfxDevice& gfx);
  bool mountFrameFromOccupancy(GfxDevice& gfx);
  void paintCylinderStress(GfxDevice& gfx);
  void paintBondDamage(GfxDevice& gfx);
  void paintAnchors(GfxDevice& gfx);
  // Allocates the stress demo object (replacing the previous one) filled by fill(x,y,z).
  VoxelObject* allocStressDemoObject(GfxDevice& gfx, int gridSize, MotionType motion, float density,
                                     const glm::vec3& position, const glm::quat& rotation,
                                     const std::function<bool(int, int, int)>& fill);
  // Upload, physics rebuild and color refresh after a demo structure mount.
  void finishDemoMount(GfxDevice& gfx);
  // Allocates and fills an object without touching the stress demo slot.
  uint32_t allocDemoSlot(int gridSize, MotionType motion, float density, const glm::vec3& position,
                         const glm::quat& rotation, const std::function<bool(int, int, int)>& fill);
  std::vector<VoxelObjectId> demoAuxIds_;  // piers, weights: freed with the demo structure
  VoxelObjectId demoWeightId_{};
  // Writes the anchor color over world-anchored fines of every object bound to inst.
  void overlayAnchorFines(const blast::StructureInstance& inst);
  void destroyStressCylinderObject();

  int applyCoarseSphereBrush(VoxelObject& o, const glm::ivec3& center, float radius,
                             uint32_t material, bool placeOnlyEmpty,
                             std::vector<voxel::FineCoord>* removedOut = nullptr);
  int applyMicroSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                            float radius, bool solid, uint32_t placeMaterial,
                            std::vector<voxel::FineCoord>* removedOut = nullptr);
  int applyFineSphereBrush(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                           const glm::ivec3& fine, float radius, bool solid, uint32_t placeMaterial,
                           std::vector<voxel::FineCoord>* removedOut = nullptr);

  struct FractureJob {
    enum class Phase : uint8_t { Searching, ReadyToCommit, Done, Cancelled };
    VoxelObjectId objectId{};
    uint64_t topologyRevision = 0;
    std::vector<voxel::FineCoord> removed;
    voxel::MultiSourceConnectivity search;
    voxel::ConnectivityResult result{};
    Phase phase = Phase::Searching;
  };

  class ObjectSolidView final : public voxel::SolidView {
  public:
    ObjectSolidView(const VoxelScene& scene, int objectIndex) : scene_(scene), objectIndex_(objectIndex) {}
    bool isSolid(voxel::FineCoord p) const override;

  private:
    const VoxelScene& scene_;
    int objectIndex_ = -1;
  };

  struct StructureRemoval {
    VoxelObjectId id{};
    std::vector<voxel::FineCoord> fines;
    std::vector<VoxelObjectId> children;
  };

  void enqueueFractureJob(VoxelObjectId id, std::vector<voxel::FineCoord> removed);
  void queueStructureRemoval(VoxelObjectId id, const std::vector<voxel::FineCoord>& fines);
  void noteStructureChild(VoxelObjectId parent, VoxelObjectId child);
  void commitStructureRemovals();
  bool commitFractureJob(GfxDevice& gfx, FractureJob& job);
  bool extractFragmentFromMasks(GfxDevice& gfx, VoxelObject& parent, int parentIndex,
                                const voxel::FinalizedComponent& comp, VoxelObjectId parentId,
                                const physics::BodyState& parentState, glm::dvec3 parentComLocal);
  bool extractIslandFromFines(GfxDevice& gfx, VoxelObject& parent, int parentIndex,
                              const std::vector<voxel::FineCoord>& fines, VoxelObjectId parentId,
                              const physics::BodyState& parentState, glm::dvec3 parentComLocal,
                              MotionType motion, VoxelObjectId* outId = nullptr);
  bool commitOccupancySplit(GfxDevice& gfx, VoxelObjectId parentId,
                            const std::vector<std::vector<voxel::FineCoord>>& islands,
                            const std::vector<uint8_t>& anchored,
                            const std::vector<NvBlastActor*>& actors, blast::StructureInstance& inst);
  VoxelObjectId objectOwningFamilyFine(const glm::ivec3& absFine) const;
  bool familyFinesOnObject(VoxelObjectId id, const std::vector<voxel::FineCoord>& fines) const;
  glm::dvec3 computeLocalCom(int objectIndex) const;
  uint32_t countSolidFines(int objectIndex) const;
  uint32_t readFineRgb(uint32_t page, uint32_t colorIndex) const;

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
  AllocatedBuffer coarsePoolBuffer_{};
  std::array<AllocatedBuffer, GfxDevice::kFramesInFlight> objectFrameBuffers_{};
  AllocatedBuffer paletteBuffer_{};
  AllocatedBuffer occMipBuffer_{};
  MeshVoxelizerGpu voxelizeGpu_{};
  std::array<glm::vec4, 256> importPalette_{};
  std::string importPath_;
  std::string lastImportedPath_;
  std::string importStatus_{"No import"};
  int importGridN_ = 64;
  int importPadding_ = 1;
  bool importSampleColor_ = false;
  bool importConservative_ = true;
  bool importAsObject_ = true;
  bool importMount_ = true;
  float importDensity_ = 600.0f;
  // The hut peaks near 2.4 MPa under its own weight: 4 MPa stands, loses its supports.
  float importStrengthMPa_ = 4.0f;
  int importAgg_ = 0;
  VoxelObjectId importedObjectId_{};
  VoxelObjectId structureFocusId_{};
  bool lastImportAsObject_ = false;
  // Nodes a mounted import may have before auto agg retries at 4 (and then refuses).
  static constexpr size_t kImportMaxNodes = 32768;
  bool voxelizeImport(GfxDevice& gfx, const std::string& path, const MeshVoxelizeConfig& cfg,
                      MeshVoxelizeResult& r);
  // Rebuilds the ground slab, dropping a mesh stamped into it by an earlier import.
  void clearGroundStamp();
  void releaseImportedObject();
  // Ground slab thickness in coarse cells; its top is where objects stand.
  static constexpr int kGroundThicknessCells = 2;
  float groundTopY() const;
  // Resamples the import's fine occupancy onto a destination fine grid, marking every
  // destination fine an occupied import fine overlaps. place() returns whether it set
  // a fine; the count of those is returned. Shared by the ground stamp and object import.
  uint32_t forEachImportWorldFine(const MeshVoxelizeResult& r, bool sampleColor, const glm::vec3& srcOrigin,
                                  const glm::vec3& dstOrigin, float dstFineVs, int dstFineN,
                                  const std::function<bool(int, int, int, bool, uint32_t)>& place) const;

  std::vector<VoxelObject> objects_;
  std::vector<uint32_t> objectGenerations_;
  std::vector<uint32_t> freeObjectSlots_;
  VoxelObjectId groundObjectId_{};
  VoxelObjectId testObjectId_{};
  std::vector<GpuVoxelObject> objectsGpu_;
  std::array<std::vector<GpuVoxelObject>, GfxDevice::kFramesInFlight> uploadedObjectsGpu_;
  std::vector<CoarseCell> coarsePoolCpu_;
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
  bool spawnTestBoxOnSimulate_ = false;
  bool fineCollideProbes_ = false;
  physics::PhysicsWorld physics_;
  blast::StructureWorld structures_;
  VoxelObjectId stressCylinderId_{};
  glm::vec3 frameSpawnPos_{0.0f};
  int frameColumnHeightFines_ = 40;
  float frameExtentY_ = 0.0f;
  bool frameSpawnPosValid_ = false;
  bool bondDamageDisplay_ = false;
  uint64_t lastBondDamagePaintEvents_ = 0;
  bool stressCylinderCut_ = false;
  bool stressCylinderDoubleDensity_ = false;
  bool stressCylinderDisplay_ = false;
  uint64_t lastStressPaintSolveEpoch_ = 0;
  bool groundContactAnchors_ = false;
  bool anchorDisplay_ = false;
  std::string structureMountStatus_;
  blast::GroundAnchorResult lastGroundAnchors_;

  bool prevLmb_ = false;
  bool prevF_ = false;
  std::optional<VoxelHit> lastHit_;

  bool fractureEnabled_ = true;
  std::vector<FractureJob> fractureJobs_;
  std::vector<StructureRemoval> structureRemovals_;
  // Debris smaller than this many fines is deleted instead of becoming a body.
  static constexpr uint32_t kMinFragmentFines = 8;
  uint32_t gpuResourceSerial_ = 1;
};
