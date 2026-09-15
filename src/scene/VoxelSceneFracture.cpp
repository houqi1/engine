#include "scene/VoxelScene.h"

#include "blast/CylinderVoxels.h"
#include "blast/HardFracture.h"
#include "blast/StructureWorld.h"

#include "NvBlast.h"
#include "NvBlastTypes.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <unordered_map>

namespace {

glm::ivec3 decodeFineInCoarse(uint32_t idx) {
  const int F = voxel::kFinePerCoarse;
  const int x = static_cast<int>(idx % static_cast<uint32_t>(F));
  const int y = static_cast<int>((idx / static_cast<uint32_t>(F)) % static_cast<uint32_t>(F));
  const int z = static_cast<int>(idx / static_cast<uint32_t>(F * F));
  return glm::ivec3(x, y, z);
}

void splitAbsFine(const glm::ivec3& absFine, glm::ivec3& coarse, glm::ivec3& micro, glm::ivec3& fine) {
  const int cellStride = VoxelScene::kMicroRes * VoxelScene::kFineRes;
  auto divFloor = [](int a, int b) {
    int q = a / b;
    int rem = a % b;
    if (rem != 0 && ((rem < 0) != (b < 0))) {
      --q;
    }
    return q;
  };
  coarse = glm::ivec3(divFloor(absFine.x, cellStride), divFloor(absFine.y, cellStride),
                      divFloor(absFine.z, cellStride));
  const glm::ivec3 rem = absFine - coarse * cellStride;
  micro = glm::ivec3(divFloor(rem.x, VoxelScene::kFineRes), divFloor(rem.y, VoxelScene::kFineRes),
                     divFloor(rem.z, VoxelScene::kFineRes));
  fine = rem - micro * VoxelScene::kFineRes;
}

}  // namespace

bool VoxelScene::ObjectSolidView::isSolid(voxel::FineCoord p) const {
  if (!scene_.slotOccupied(objectIndex_)) {
    return false;
  }
  const VoxelObject& o = scene_.cpuObject(objectIndex_);
  const int nFine = o.gridSize * voxel::kFinePerCoarse;
  if (p.x < 0 || p.y < 0 || p.z < 0 || p.x >= nFine || p.y >= nFine || p.z >= nFine) {
    return false;
  }
  glm::ivec3 coarse;
  glm::ivec3 micro;
  glm::ivec3 fine;
  splitAbsFine(glm::ivec3(p.x, p.y, p.z), coarse, micro, fine);
  return scene_.occupancyFine(objectIndex_, coarse, micro, fine);
}

uint32_t VoxelScene::readFineRgb(uint32_t page, uint32_t colorIndex) const {
  if (page == kInvalidBrickPage || colorIndex >= static_cast<uint32_t>(kFineColorWords)) {
    return 0;
  }
  const uint32_t si = page / kPagesPerSlab;
  if (si >= slabs_.size()) {
    return 0;
  }
  return brickPageWords(page)[kFineColorOffset + colorIndex];
}

uint32_t VoxelScene::countSolidFines(int objectIndex) const {
  if (!slotOccupied(objectIndex)) {
    return 0;
  }
  const VoxelObject& o = objects_[static_cast<size_t>(objectIndex)];
  uint32_t count = 0;
  std::vector<glm::ivec3> fines;
  fines.reserve(64);
  for (int z = 0; z < o.gridSize; ++z) {
    for (int y = 0; y < o.gridSize; ++y) {
      for (int x = 0; x < o.gridSize; ++x) {
        const glm::ivec3 c(x, y, z);
        if (occupancyMaterial(objectIndex, c) == 0u) {
          continue;
        }
        if (coarseBrickPage(objectIndex, c) == kInvalidBrickPage) {
          count += static_cast<uint32_t>(kFinePerBrick);
        } else {
          collectOccupiedFines(objectIndex, c, fines);
          count += static_cast<uint32_t>(fines.size());
        }
      }
    }
  }
  return count;
}

glm::dvec3 VoxelScene::computeLocalCom(int objectIndex) const {
  if (!slotOccupied(objectIndex)) {
    return glm::dvec3(0.0);
  }
  const VoxelObject& o = objects_[static_cast<size_t>(objectIndex)];
  const int F = kFinePerCoarse;
  const double s = static_cast<double>(o.voxelSize) / static_cast<double>(F);
  const double mi = static_cast<double>(physics::kDensityWood) * s * s * s;
  double mass = 0.0;
  glm::dvec3 moment(0.0);
  std::vector<glm::ivec3> fines;
  fines.reserve(64);
  for (int z = 0; z < o.gridSize; ++z) {
    for (int y = 0; y < o.gridSize; ++y) {
      for (int x = 0; x < o.gridSize; ++x) {
        const glm::ivec3 c(x, y, z);
        if (occupancyMaterial(objectIndex, c) == 0u) {
          continue;
        }
        if (coarseBrickPage(objectIndex, c) == kInvalidBrickPage) {
          const double mC = mi * static_cast<double>(F * F * F);
          const glm::dvec3 p((static_cast<double>(x) + 0.5) * static_cast<double>(o.voxelSize),
                             (static_cast<double>(y) + 0.5) * static_cast<double>(o.voxelSize),
                             (static_cast<double>(z) + 0.5) * static_cast<double>(o.voxelSize));
          mass += mC;
          moment += mC * p;
        } else {
          collectOccupiedFines(objectIndex, c, fines);
          for (const glm::ivec3& fp : fines) {
            const glm::dvec3 p((static_cast<double>(fp.x) + 0.5) * s,
                               (static_cast<double>(fp.y) + 0.5) * s,
                               (static_cast<double>(fp.z) + 0.5) * s);
            mass += mi;
            moment += mi * p;
          }
        }
      }
    }
  }
  if (mass <= 1e-12) {
    const double half = 0.5 * static_cast<double>(o.gridSize) * static_cast<double>(o.voxelSize);
    return glm::dvec3(half, half, half);
  }
  return moment / mass;
}

void VoxelScene::enqueueFractureJob(VoxelObjectId id, std::vector<voxel::FineCoord> removed) {
  VoxelObject* o = tryGetObject(id);
  if (!o || removed.empty()) {
    return;
  }
  for (FractureJob& job : fractureJobs_) {
    if (job.objectId == id && job.phase == FractureJob::Phase::Searching) {
      job.removed.insert(job.removed.end(), removed.begin(), removed.end());
      job.topologyRevision = o->topologyRevision;
      return;
    }
  }
  FractureJob job;
  job.objectId = id;
  job.topologyRevision = o->topologyRevision;
  job.removed = std::move(removed);
  job.phase = FractureJob::Phase::Searching;
  fractureJobs_.push_back(std::move(job));
}

void VoxelScene::advanceFractureWork(float /*dt*/) {
  if (!fractureEnabled_ || fractureJobs_.empty()) {
    return;
  }
  for (FractureJob& job : fractureJobs_) {
    if (job.phase != FractureJob::Phase::Searching) {
      continue;
    }
    VoxelObject* o = tryGetObject(job.objectId);
    if (!o) {
      job.phase = FractureJob::Phase::Cancelled;
      continue;
    }
    // Same-frame dig sets topologyRevision then enqueues; cancel only if a newer edit landed.
    if (o->topologyRevision != job.topologyRevision) {
      job.phase = FractureJob::Phase::Cancelled;
      continue;
    }

    // Stack view is valid for the entire synchronous search below.
    ObjectSolidView view(*this, static_cast<int>(job.objectId.slot));
    auto seeds = voxel::collectBoundarySeeds(view, job.removed);
    job.search.reset(&view, std::move(seeds));
    voxel::ConnectivityBudget budget;
    budget.maxExpansions = 0;  // v1: finish in one frame for gameplay-sized pieces
    budget.perRootQuota = 32;
    while (job.search.status() == voxel::ConnectivityStatus::Pending) {
      job.search.step(budget);
    }
    job.result = job.search.result();
    job.result.metrics.deletedFineCount = job.removed.size();

    if (job.result.status == voxel::ConnectivityStatus::ComponentsFound &&
        !job.result.components.empty()) {
      job.phase = FractureJob::Phase::ReadyToCommit;
    } else {
      physics::BodyState st;
      if (physics_.getBodyState(job.objectId, st)) {
        physics_.replaceShape(job.objectId, &st);
      } else {
        physics_.replaceShape(job.objectId, nullptr);
      }
      job.phase = FractureJob::Phase::Done;
    }
  }
}

bool VoxelScene::extractFragmentFromMasks(GfxDevice& /*gfx*/, VoxelObject& parent, int parentIndex,
                                          const voxel::FinalizedComponent& comp,
                                          VoxelObjectId parentId,
                                          const physics::BodyState& parentState,
                                          glm::dvec3 parentComLocal) {
  struct FineSample {
    glm::ivec3 absFine{0};
    uint32_t material = 1;
    uint32_t rgb = 0;
    bool hasRgb = false;
  };
  std::vector<FineSample> samples;
  samples.reserve(static_cast<size_t>(comp.voxelCount));

  for (const voxel::ComponentMask& mask : comp.masks) {
    const glm::ivec3 coarse(mask.coarse.x, mask.coarse.y, mask.coarse.z);
    if (!inBounds(parent, coarse)) {
      continue;
    }
    const uint32_t mat = occupancyMaterial(parentIndex, coarse);
    if (mat == 0u) {
      continue;
    }
    for (uint32_t i = 0; i < voxel::kLabelsPerSearchPage; ++i) {
      if (!mask.test(i)) {
        continue;
      }
      const glm::ivec3 local = decodeFineInCoarse(i);
      const glm::ivec3 absFine = coarse * kFinePerCoarse + local;
      glm::ivec3 c, m, f;
      splitAbsFine(absFine, c, m, f);
      if (!getFine(parent, c, m, f)) {
        continue;
      }
      FineSample sample;
      sample.absFine = absFine;
      sample.material = mat;
      const uint32_t srcPage = coarseBrickPage(parentIndex, c);
      if (srcPage != kInvalidBrickPage) {
        sample.rgb = readFineRgb(srcPage, fineColorIndex(m, f));
        sample.hasRgb = (sample.rgb & 0xFF000000u) != 0;
      }
      samples.push_back(sample);
    }
  }

  // Always carve the component out of the parent, even for tiny debris.
  for (const FineSample& sample : samples) {
    glm::ivec3 c, m, f;
    splitAbsFine(sample.absFine, c, m, f);
    setFineCpu(parent, c, m, f, false);
  }

  if (samples.size() < kMinFragmentFines) {
    return true;
  }

  glm::ivec3 fineMn(samples[0].absFine);
  glm::ivec3 fineMx(samples[0].absFine);
  for (const FineSample& sample : samples) {
    fineMn = glm::min(fineMn, sample.absFine);
    fineMx = glm::max(fineMx, sample.absFine);
  }
  const glm::ivec3 coarseMin(fineMn.x / kFinePerCoarse, fineMn.y / kFinePerCoarse,
                             fineMn.z / kFinePerCoarse);
  const glm::ivec3 coarseMax(fineMx.x / kFinePerCoarse, fineMx.y / kFinePerCoarse,
                             fineMx.z / kFinePerCoarse);
  const glm::ivec3 extent = coarseMax - coarseMin + glm::ivec3(1);
  const int newGridSize = std::max({1, extent.x, extent.y, extent.z});

  uint32_t slot = 0;
  try {
    slot = allocObjectSlot();
  } catch (const std::exception& ex) {
    std::cerr << "Fracture: allocObjectSlot failed: " << ex.what() << "\n";
    return false;
  }

  VoxelObject& frag = objects_[slot];
  frag.gridSize = newGridSize;
  frag.voxelSize = parent.voxelSize;
  frag.nestedMicro = parent.nestedMicro;
  frag.editable = true;
  frag.enabled = true;
  frag.slotOccupied = true;
  frag.isScatter = true;
  frag.motionType = MotionType::Dynamic;
  frag.topologyRevision = 1;
  frag.useImportPalette = parent.useImportPalette;
  frag.rotation = parent.rotation;

  // Keep world positions of preserved fines continuous under coarse-aligned shift.
  const glm::vec3 gridCenterParent =
      0.5f * static_cast<float>(parent.gridSize) * parent.voxelSize * glm::vec3(1.0f);
  const glm::vec3 gridCenterFrag =
      0.5f * static_cast<float>(newGridSize) * parent.voxelSize * glm::vec3(1.0f);
  const glm::vec3 shiftMeters = glm::vec3(coarseMin) * parent.voxelSize;
  frag.position = parent.position + parent.rotation * (shiftMeters + gridCenterFrag - gridCenterParent);

  const size_t cellCount = static_cast<size_t>(newGridSize) * static_cast<size_t>(newGridSize) *
                           static_cast<size_t>(newGridSize);
  frag.cells.assign(cellCount, CoarseCell{});

  const glm::ivec3 fineShift = coarseMin * kFinePerCoarse;
  glm::dvec3 fragMoment(0.0);
  double fragMass = 0.0;
  const double s = static_cast<double>(parent.voxelSize) / static_cast<double>(kFinePerCoarse);
  const double mi = static_cast<double>(parent.density > 0.0f ? parent.density : physics::kDensityWood) * s * s * s;
  uint32_t copied = 0;

  for (const FineSample& sample : samples) {
    const glm::ivec3 localFine = sample.absFine - fineShift;
    glm::ivec3 c, m, f;
    splitAbsFine(localFine, c, m, f);
    if (!inBounds(frag, c) || !microInBounds(m) || !fineInBounds(f)) {
      continue;
    }
    ensureCoarseBrick(frag, c, sample.material);
    if (sample.hasRgb) {
      setFineCpu(frag, c, m, f, true, true, sample.rgb & 0x00FFFFFFu);
    } else {
      setFineCpu(frag, c, m, f, true);
    }
    ++copied;
    const glm::dvec3 p((static_cast<double>(localFine.x) + 0.5) * s,
                       (static_cast<double>(localFine.y) + 0.5) * s,
                       (static_cast<double>(localFine.z) + 0.5) * s);
    fragMass += mi;
    fragMoment += mi * p;
  }

  if (copied < kMinFragmentFines) {
    freeObjectSlot(slot);
    return true;
  }

  // Velocity inheritance uses parent-local COM vs fragment COM expressed in parent frame.
  const glm::dvec3 fragComLocalParent =
      (fragMass > 1e-12 ? (fragMoment / fragMass) : glm::dvec3(gridCenterFrag)) +
      glm::dvec3(shiftMeters);
  const glm::vec3 deltaWorld =
      parent.rotation * glm::vec3(fragComLocalParent - parentComLocal);

  physics::BodyState childState = parentState;
  childState.x = frag.position;
  childState.q = frag.rotation;
  childState.v = parentState.v + glm::cross(parentState.w, deltaWorld);
  childState.w = parentState.w;
  childState.awake = true;
  childState.sleepTimer = 0.0f;

  const VoxelObjectId childId = makeObjectId(slot);
  if (!physics_.addBody(childId, childState)) {
    freeObjectSlot(slot);
    return false;
  }
  (void)parentId;
  return true;
}

bool VoxelScene::commitFractureJob(GfxDevice& gfx, FractureJob& job) {
  VoxelObject* parent = tryGetObject(job.objectId);
  if (!parent) {
    job.phase = FractureJob::Phase::Cancelled;
    return false;
  }
  if (parent->motionType != MotionType::Dynamic) {
    job.phase = FractureJob::Phase::Cancelled;
    return false;
  }

  physics::BodyState parentState{};
  if (!physics_.getBodyState(job.objectId, parentState)) {
    parentState.x = parent->position;
    parentState.q = parent->rotation;
    parentState.awake = true;
  }
  const glm::dvec3 parentCom = computeLocalCom(static_cast<int>(job.objectId.slot));
  const int parentIndex = static_cast<int>(job.objectId.slot);

  for (const voxel::FinalizedComponent& comp : job.result.components) {
    if (!extractFragmentFromMasks(gfx, *parent, parentIndex, comp, job.objectId, parentState,
                                  parentCom)) {
      // Allocation failure: stop extracting more; parent may be partially carved.
      std::cerr << "Fracture: fragment extract failed; leaving remaining as parent\n";
      break;
    }
  }

  parent->topologyRevision += 1;
  const uint32_t left = countSolidFines(parentIndex);
  if (left == 0) {
    parent->enabled = false;
    physics_.removeBody(job.objectId);
  } else {
    // Keep parent pose; refresh shape/mass and wake.
    parentState.awake = true;
    parentState.sleepTimer = 0.0f;
    // COM shift velocity for remainder.
    const glm::dvec3 newCom = computeLocalCom(parentIndex);
    const glm::vec3 deltaWorld = parent->rotation * glm::vec3(newCom - parentCom);
    parentState.v = parentState.v + glm::cross(parentState.w, deltaWorld);
    physics_.replaceShape(job.objectId, &parentState);
  }

  // Match scatter/import: drain in-flight frames before rebuilding shared GPU buffers.
  gfx.waitIdle();
  packObjectPool();
  fillGpuObjectRecords();
  ensureGpuBuffers(gfx);
  uploadCoarsePool(gfx);
  flushDirtyPages(gfx);
  uploadOccMip(gfx);
  for (uint32_t i = 0; i < GfxDevice::kFramesInFlight; ++i) {
    uploadObjectTransforms(gfx, i);
  }
  // Force descriptor refresh even if VkBuffer handles were recycled.
  ++gpuResourceSerial_;

  const glm::vec3 wakePad(2.0f);
  physics_.activateBodiesInBounds(parentState.x - wakePad, parentState.x + wakePad);

  std::cout << "Fracture: split object slot=" << job.objectId.slot
            << " fragments=" << job.result.components.size()
            << " visited=" << job.result.metrics.visitedFineCount
            << " remainderExit=" << (job.result.metrics.remainderEarlyExit ? 1 : 0) << "\n";
  job.phase = FractureJob::Phase::Done;
  return true;
}

void VoxelScene::commitReadyFractures(GfxDevice& gfx) {
  if (!fractureEnabled_ || fractureJobs_.empty()) {
    return;
  }
  for (FractureJob& job : fractureJobs_) {
    if (job.phase == FractureJob::Phase::ReadyToCommit) {
      commitFractureJob(gfx, job);
    }
  }
  fractureJobs_.erase(std::remove_if(fractureJobs_.begin(), fractureJobs_.end(),
                                     [](const FractureJob& j) {
                                       return j.phase == FractureJob::Phase::Done ||
                                              j.phase == FractureJob::Phase::Cancelled;
                                     }),
                      fractureJobs_.end());
}

namespace {

void sceneBlastLog(int type, const char* msg, const char* file, int line) {
  if (type <= NvBlastMessage::Error) {
    std::cerr << "NvBlastLL " << file << ":" << line << ": " << (msg ? msg : "") << "\n";
  }
}

}  // namespace

bool VoxelScene::extractIslandFromFines(GfxDevice& /*gfx*/, VoxelObject& parent, int parentIndex,
                                        const std::vector<voxel::FineCoord>& fines, VoxelObjectId parentId,
                                        const physics::BodyState& parentState, glm::dvec3 parentComLocal,
                                        MotionType motion, uint32_t minFines, VoxelObjectId* outId) {
  if (outId != nullptr) {
    *outId = {};
  }
  if (fines.empty()) {
    return true;
  }
  struct FineSample {
    glm::ivec3 absFine{0};
    uint32_t material = 1;
    uint32_t rgb = 0;
    bool hasRgb = false;
  };
  std::vector<FineSample> samples;
  samples.reserve(fines.size());
  const glm::ivec3 origin = parent.structureFineOrigin;
  auto toLocal = [&](const glm::ivec3& absFine) { return absFine - origin; };
  for (const voxel::FineCoord& fc : fines) {
    const glm::ivec3 absFine(fc.x, fc.y, fc.z);
    const glm::ivec3 local = toLocal(absFine);
    glm::ivec3 c, m, f;
    splitFineIndex(local.x, local.y, local.z, c, m, f);
    if (local.x < 0 || local.y < 0 || local.z < 0 || !getFine(parent, c, m, f)) {
      continue;
    }
    FineSample sample;
    sample.absFine = absFine;
    sample.material = occupancyMaterial(parentIndex, c);
    const uint32_t srcPage = coarseBrickPage(parentIndex, c);
    if (srcPage != kInvalidBrickPage) {
      sample.rgb = readFineRgb(srcPage, fineColorIndex(m, f));
      sample.hasRgb = (sample.rgb & 0xFF000000u) != 0;
    }
    samples.push_back(sample);
  }
  if (samples.empty()) {
    return true;
  }
  if (samples.size() < minFines) {
    return true;
  }

  glm::ivec3 fineMn = toLocal(samples[0].absFine);
  glm::ivec3 fineMx = fineMn;
  for (const FineSample& sample : samples) {
    const glm::ivec3 local = toLocal(sample.absFine);
    fineMn = glm::min(fineMn, local);
    fineMx = glm::max(fineMx, local);
  }
  const glm::ivec3 coarseMin(fineMn.x / kFinePerCoarse, fineMn.y / kFinePerCoarse, fineMn.z / kFinePerCoarse);
  const glm::ivec3 coarseMax(fineMx.x / kFinePerCoarse, fineMx.y / kFinePerCoarse, fineMx.z / kFinePerCoarse);
  const glm::ivec3 extent = coarseMax - coarseMin + glm::ivec3(1);
  const int newGridSize = std::max({1, extent.x, extent.y, extent.z});

  const float parentVoxelSize = parent.voxelSize;
  const bool parentNested = parent.nestedMicro;
  const float parentDensity = parent.density;
  const bool parentPalette = parent.useImportPalette;
  const glm::quat parentRot = parent.rotation;
  const glm::vec3 parentPos = parent.position;
  uint32_t slot = 0;
  try {
    slot = allocObjectSlot();
  } catch (const std::exception& ex) {
    std::cerr << "Structure split: allocObjectSlot failed: " << ex.what() << "\n";
    return false;
  }

  VoxelObject& parentObj = objects_[static_cast<size_t>(parentIndex)];
  VoxelObject& frag = objects_[slot];
  frag.gridSize = newGridSize;
  frag.voxelSize = parentVoxelSize;
  frag.nestedMicro = parentNested;
  frag.editable = true;
  frag.enabled = true;
  frag.slotOccupied = true;
  frag.isScatter = true;
  frag.motionType = motion;
  frag.density = parentDensity;
  frag.topologyRevision = 1;
  frag.structureFineOrigin = origin + coarseMin * kFinePerCoarse;
  frag.useImportPalette = parentPalette;
  frag.rotation = parentRot;
  const glm::vec3 gridCenterParent =
      0.5f * static_cast<float>(parentObj.gridSize) * parentVoxelSize * glm::vec3(1.0f);
  const glm::vec3 gridCenterFrag = 0.5f * static_cast<float>(newGridSize) * parentVoxelSize * glm::vec3(1.0f);
  const glm::vec3 shiftMeters = glm::vec3(coarseMin) * parentVoxelSize;
  frag.position = parentPos + parentRot * (shiftMeters + gridCenterFrag - gridCenterParent);
  frag.cells.assign(static_cast<size_t>(newGridSize) * static_cast<size_t>(newGridSize) *
                        static_cast<size_t>(newGridSize),
                    CoarseCell{});

  const glm::ivec3 fineShift = coarseMin * kFinePerCoarse;
  glm::dvec3 fragMoment(0.0);
  double fragMass = 0.0;
  const double s = static_cast<double>(parentVoxelSize) / static_cast<double>(kFinePerCoarse);
  const double mi = static_cast<double>(parentDensity > 0.0f ? parentDensity : physics::kDensityWood) * s * s * s;
  uint32_t copied = 0;
  for (const FineSample& sample : samples) {
    const glm::ivec3 localFine = toLocal(sample.absFine) - fineShift;
    glm::ivec3 c, m, f;
    splitFineIndex(localFine.x, localFine.y, localFine.z, c, m, f);
    if (!inBounds(frag, c) || !microInBounds(m) || !fineInBounds(f)) {
      continue;
    }
    ensureCoarseBrick(frag, c, sample.material);
    if (sample.hasRgb) {
      setFineCpu(frag, c, m, f, true, true, sample.rgb & 0x00FFFFFFu);
    } else {
      setFineCpu(frag, c, m, f, true);
    }
    ++copied;
    fragMass += mi;
    fragMoment += mi * glm::dvec3((static_cast<double>(localFine.x) + 0.5) * s,
                                  (static_cast<double>(localFine.y) + 0.5) * s,
                                  (static_cast<double>(localFine.z) + 0.5) * s);
  }
  if (copied == 0) {
    freeObjectSlot(slot);
    return true;
  }

  const glm::dvec3 fragComLocalParent =
      (fragMass > 1e-12 ? (fragMoment / fragMass) : glm::dvec3(gridCenterFrag)) + glm::dvec3(shiftMeters);
  const glm::vec3 deltaWorld = parentRot * glm::vec3(fragComLocalParent - parentComLocal);
  physics::BodyState childState = parentState;
  childState.x = frag.position;
  childState.q = frag.rotation;
  if (motion == MotionType::Dynamic) {
    childState.v = parentState.v + glm::cross(parentState.w, deltaWorld);
    childState.w = parentState.w;
    childState.awake = true;
  } else {
    childState.v = glm::vec3(0.0f);
    childState.w = glm::vec3(0.0f);
    childState.awake = false;
  }
  childState.sleepTimer = 0.0f;
  if (!physics_.addBody(makeObjectId(slot), childState)) {
    freeObjectSlot(slot);
    return false;
  }
  for (const voxel::FineCoord& fc : fines) {
    const glm::ivec3 local = toLocal(glm::ivec3(fc.x, fc.y, fc.z));
    if (local.x < 0 || local.y < 0 || local.z < 0) {
      continue;
    }
    glm::ivec3 c, m, f;
    splitFineIndex(local.x, local.y, local.z, c, m, f);
    setFineCpu(parentObj, c, m, f, false);
  }
  (void)parentId;
  if (outId != nullptr) {
    *outId = makeObjectId(slot);
  }
  return true;
}

bool VoxelScene::familyFinesOnObject(VoxelObjectId id, const std::vector<voxel::FineCoord>& fines) const {
  const VoxelObject* o = tryGetObject(id);
  if (o == nullptr || fines.empty()) {
    return false;
  }
  const int slot = static_cast<int>(id.slot);
  const size_t nCheck = fines.size() < 8 ? fines.size() : 8;
  for (size_t i = 0; i < nCheck; ++i) {
    const glm::ivec3 absFine(fines[i].x, fines[i].y, fines[i].z);
    const glm::ivec3 local = absFine - o->structureFineOrigin;
    if (local.x < 0 || local.y < 0 || local.z < 0) {
      continue;
    }
    glm::ivec3 c, m, f;
    splitFineIndex(local.x, local.y, local.z, c, m, f);
    if (occupancyFine(slot, c, m, f)) {
      return true;
    }
  }
  return false;
}

VoxelObjectId VoxelScene::objectOwningFamilyFine(const glm::ivec3& absFine) const {
  for (int i = 0; i < cpuObjectCount(); ++i) {
    if (groundObjectId_.valid() && static_cast<int>(groundObjectId_.slot) == i) {
      continue;
    }
    if (testObjectId_.valid() && static_cast<int>(testObjectId_.slot) == i) {
      continue;
    }
    if (!slotOccupied(i)) {
      continue;
    }
    const VoxelObject& o = cpuObject(i);
    const glm::ivec3 local = absFine - o.structureFineOrigin;
    if (local.x < 0 || local.y < 0 || local.z < 0) {
      continue;
    }
    glm::ivec3 c, m, f;
    splitFineIndex(local.x, local.y, local.z, c, m, f);
    if (!inBounds(o, c) || !occupancyFine(i, c, m, f)) {
      continue;
    }
    return objectIdAt(i);
  }
  return {};
}

bool VoxelScene::commitOccupancySplit(GfxDevice& gfx, VoxelObjectId parentId,
                                      const std::vector<std::vector<voxel::FineCoord>>& islands,
                                      const std::vector<uint8_t>& anchored, uint32_t minFines,
                                      const std::vector<NvBlastActor*>& actors) {
  VoxelObject* parent = tryGetObject(parentId);
  if (!parent || islands.empty()) {
    return false;
  }
  size_t keep = 0;
  size_t keepN = 0;
  bool haveAnchored = false;
  for (size_t i = 0; i < islands.size(); ++i) {
    const bool anc = i < anchored.size() && anchored[i] != 0;
    if (anc && !haveAnchored) {
      haveAnchored = true;
      keep = i;
      keepN = islands[i].size();
      continue;
    }
    if (anc == haveAnchored && islands[i].size() > keepN) {
      keepN = islands[i].size();
      keep = i;
    }
  }
  physics::BodyState parentState{};
  if (!physics_.getBodyState(parentId, parentState)) {
    parentState.x = parent->position;
    parentState.q = parent->rotation;
  }
  const glm::dvec3 parentCom = computeLocalCom(static_cast<int>(parentId.slot));
  const int parentIndex = static_cast<int>(parentId.slot);
  const uint32_t before = countSolidFines(parentIndex);

  std::vector<blast::ActorObjectLink> links(islands.size());
  links[keep].objectId = parentId;
  links[keep].fineOrigin = parent->structureFineOrigin;
  links[keep].fineN = parent->gridSize * kFinePerCoarse;
  if (keep < actors.size()) {
    links[keep].actor = actors[keep];
  }

  uint32_t created = 0;
  for (size_t i = 0; i < islands.size(); ++i) {
    if (i == keep || islands[i].empty() || islands[i].size() < minFines) {
      continue;
    }
    parent = tryGetObject(parentId);
    if (parent == nullptr) {
      return false;
    }
    const MotionType motion = (i < anchored.size() && anchored[i] != 0) ? MotionType::Static : MotionType::Dynamic;
    VoxelObjectId childId{};
    if (!extractIslandFromFines(gfx, *parent, parentIndex, islands[i], parentId, parentState, parentCom, motion,
                                minFines, &childId)) {
      std::cerr << "Structure split: island extract failed\n";
      return false;
    }
    links[i].objectId = childId;
    if (i < actors.size()) {
      links[i].actor = actors[i];
    }
    if (childId.valid()) {
      ++created;
    }
    if (const VoxelObject* child = tryGetObject(childId)) {
      links[i].fineOrigin = child->structureFineOrigin;
      links[i].fineN = child->gridSize * kFinePerCoarse;
    }
  }

  parent = tryGetObject(parentId);
  if (parent == nullptr) {
    return false;
  }
  const bool keepAnchored = keep < anchored.size() && anchored[keep] != 0;
  if (created > 0) {
    parent->motionType = keepAnchored ? MotionType::Static : MotionType::Dynamic;
  }
  parent->topologyRevision += 1;
  if (blast::StructureInstance* st = structures_.instance()) {
    if (st->objectId == parentId) {
      st->topologyRevision = parent->topologyRevision;
    }
  }
  const uint32_t left = countSolidFines(parentIndex);
  if (left == 0) {
    parent->enabled = false;
    physics_.removeBody(parentId);
  } else {
    parentState.awake = parent->motionType == MotionType::Dynamic;
    parentState.sleepTimer = 0.0f;
    if (parent->motionType == MotionType::Dynamic) {
      const glm::dvec3 newCom = computeLocalCom(parentIndex);
      parentState.v = parentState.v + glm::cross(parentState.w, glm::vec3(parent->rotation * glm::vec3(newCom - parentCom)));
    } else {
      parentState.v = glm::vec3(0.0f);
      parentState.w = glm::vec3(0.0f);
    }
    physics_.replaceShape(parentId, &parentState);
  }

  if (created == 0) {
    structures_.bindVisibleActors(links);
    return true;
  }
  lastStressPaintSolveEpoch_ = 0;
  gfx.waitIdle();
  packObjectPool();
  fillGpuObjectRecords();
  ensureGpuBuffers(gfx);
  uploadCoarsePool(gfx);
  flushDirtyPages(gfx);
  uploadOccMip(gfx);
  for (uint32_t i = 0; i < GfxDevice::kFramesInFlight; ++i) {
    uploadObjectTransforms(gfx, i);
  }
  ++gpuResourceSerial_;
  physics_.activateBodiesInBounds(parentState.x - glm::vec3(2.0f), parentState.x + glm::vec3(2.0f));
  structures_.bindVisibleActors(links);
  std::cout << "Structure split: parent slot=" << parentId.slot << " islands=" << islands.size()
            << " fines " << before << " -> " << left << " keepAnchored=" << (keepAnchored ? 1 : 0)
            << " bindings=" << structures_.bindings().size() << "\n";
  return true;
}

void VoxelScene::commitStructureSplits(GfxDevice& gfx) {
  blast::StructureInstance* inst = structures_.instance();
  if (inst == nullptr || inst->blast.actor == nullptr) {
    return;
  }
  if (!structures_.pendingFracture().valid || structures_.pendingFracture().candidates.empty()) {
    return;
  }
  if (!structures_.pendingSnapshotMatches(structures_.pendingFracture())) {
    structures_.recachePendingFromProbes();
    if (!structures_.pendingSnapshotMatches(structures_.pendingFracture()) ||
        !structures_.pendingFracture().valid || structures_.pendingFracture().candidates.empty()) {
      structures_.clearPendingFracture();
      return;
    }
  }

  const blast::PendingFracture& pending = structures_.pendingFracture();
  for (const blast::FractureCandidate& c : pending.candidates) {
    for (uint64_t f : c.faces) {
      inst->grid.brokenFaces.insert(f);
    }
    if (c.stableId != 0) {
      inst->grid.bondDamage[blast::packBondKey(c.nodeA, c.nodeB)] = 1.0f;
    }
  }

  const uint32_t nfrac = structures_.applyPendingCandidates(pending);
  inst->debug.fracturedBonds = nfrac;
  structures_.clearPendingFracture();
  if (nfrac == 0) {
    return;
  }

  const uint32_t actors = structures_.splitAllRequired();
  inst->debug.splitActors = actors;
  inst->debug.candidateCount = 0;

  if (actors <= 1) {
    std::cout << "Structure fracture: bonds=" << nfrac << " no new actor (graph updated)\n";
    return;
  }
  if (actors > 128) {
    std::cerr << "Structure fracture: bonds=" << nfrac << " actors=" << actors
              << " occupancy extract skipped (too many islands)\n";
    return;
  }

  const NvBlastChunk* chunks = NvBlastAssetGetChunks(inst->blast.asset, sceneBlastLog);
  const uint32_t nA = NvBlastFamilyGetActorCount(inst->blast.family, sceneBlastLog);
  std::vector<NvBlastActor*> actorList(nA, nullptr);
  NvBlastFamilyGetActors(actorList.data(), nA, inst->blast.family, sceneBlastLog);
  std::unordered_map<uint32_t, const blast::GraphNode*> nodeById;
  for (const blast::GraphNode& n : inst->graph.nodes) {
    nodeById[n.stableId] = &n;
  }

  struct Piece {
    NvBlastActor* actor = nullptr;
    std::vector<voxel::FineCoord> fines;
    uint8_t anchored = 0;
    VoxelObjectId owner{};
  };
  FILE* trace = nullptr;
  fopen_s(&trace, "docs/frame-split-trace.txt", "a");
  auto tr = [&](const std::string& s) {
    std::cout << s << "\n";
    if (trace) {
      std::fputs(s.c_str(), trace);
      std::fputc('\n', trace);
    }
  };

  const NvBlastSupportGraph support = NvBlastAssetGetSupportGraph(inst->blast.asset, sceneBlastLog);
  std::vector<Piece> pieces;
  for (uint32_t i = 0; i < nA; ++i) {
    NvBlastActor* a = actorList[i];
    if (a == nullptr) {
      continue;
    }
    const uint32_t nn = NvBlastActorGetGraphNodeCount(a, sceneBlastLog);
    if (nn == 0) {
      continue;
    }
    std::vector<uint32_t> gidx(nn);
    NvBlastActorGetGraphNodeIndices(gidx.data(), nn, a, sceneBlastLog);
    std::vector<voxel::FineCoord> fines;
    for (uint32_t gn : gidx) {
      if (gn >= support.nodeCount || support.chunkIndices[gn] == UINT32_MAX) {
        continue;
      }
      const uint32_t stable = chunks[support.chunkIndices[gn]].userData;
      const auto it = nodeById.find(stable);
      if (it == nodeById.end()) {
        continue;
      }
      for (const blast::VoxelCoord& p : it->second->voxels) {
        fines.push_back(voxel::FineCoord{p.x, p.y, p.z});
      }
    }
    if (fines.empty()) {
      tr("Structure split: actor " + std::to_string(i) + " graphNodes=" + std::to_string(nn) + " fines=0");
      continue;
    }
    tr("Structure split: actor " + std::to_string(i) + " graphNodes=" + std::to_string(nn) +
       " fines=" + std::to_string(fines.size()) +
       " anchored=" + std::to_string(NvBlastActorHasExternalBonds(a, sceneBlastLog) ? 1 : 0));
    Piece piece;
    piece.actor = a;
    piece.fines = std::move(fines);
    piece.anchored = NvBlastActorHasExternalBonds(a, sceneBlastLog) ? 1 : 0;
    piece.owner = objectOwningFamilyFine(glm::ivec3(piece.fines[0].x, piece.fines[0].y, piece.fines[0].z));
    if (!piece.owner.valid() ||
        (groundObjectId_.valid() && piece.owner.slot == groundObjectId_.slot) ||
        (testObjectId_.valid() && piece.owner.slot == testObjectId_.slot)) {
      piece.owner = inst->objectId;
    }
    tr("Structure split: ownerSlot=" + std::to_string(piece.owner.slot));
    pieces.push_back(std::move(piece));
  }

  std::unordered_map<uint32_t, std::vector<size_t>> bySlot;
  for (size_t i = 0; i < pieces.size(); ++i) {
    if (pieces[i].owner.valid()) {
      bySlot[pieces[i].owner.slot].push_back(i);
    }
  }
  for (const auto& kv : bySlot) {
    if (kv.second.size() <= 1) {
      continue;
    }
    VoxelObjectId owner = pieces[kv.second.front()].owner;
    std::vector<std::vector<voxel::FineCoord>> islands;
    std::vector<uint8_t> anchored;
    std::vector<NvBlastActor*> actors;
    islands.reserve(kv.second.size());
    anchored.reserve(kv.second.size());
    actors.reserve(kv.second.size());
    for (size_t idx : kv.second) {
      islands.push_back(std::move(pieces[idx].fines));
      anchored.push_back(pieces[idx].anchored);
      actors.push_back(pieces[idx].actor);
    }
    const uint32_t beforeSolids = countSolidFines(static_cast<int>(owner.slot));
    tr("Structure split: ownerSlot=" + std::to_string(owner.slot) +
       " islands=" + std::to_string(islands.size()) + " solidsBefore=" + std::to_string(beforeSolids));
    commitOccupancySplit(gfx, owner, islands, anchored, 64u, actors);
    tr("Structure split: solidsAfter=" +
       std::to_string(countSolidFines(static_cast<int>(owner.slot))));
  }
  if (pieces.size() < 2) {
    tr("Structure split: pieces=" + std::to_string(pieces.size()) + " occupancy skipped");
  }
  if (trace) {
    std::fclose(trace);
  }
}
