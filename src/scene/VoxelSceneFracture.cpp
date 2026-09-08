#include "scene/VoxelScene.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

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
  const double mi = static_cast<double>(physics::kDensityWood) * s * s * s;
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
