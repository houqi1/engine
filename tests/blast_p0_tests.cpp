#include "blast/BlastIds.h"
#include "blast/BlastMemory.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastGlobals.h"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

using Nv::Blast::ExtStressSolver;
using Nv::Blast::ExtStressSolverSettings;

namespace {

int gFailures = 0;

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAIL: " << msg << "\n";
    ++gFailures;
  }
}

void blastLog(int type, const char* msg, const char* file, int line) {
  if (type <= NvBlastMessage::Warning) {
    std::cerr << "NvBlastLL " << file << ":" << line << ": " << (msg ? msg : "") << "\n";
  }
  if (type == NvBlastMessage::Error) {
    ++gFailures;
  }
}

constexpr float kMeters = 1.0f;
constexpr float kDensity = 1000.0f;  // kg/m^3
constexpr float kGravityY = -9.81f;  // m/s^2
constexpr float kVolume = 1.0f;      // m^3
constexpr float kAreaEff = 1.0f;     // m^2 remaining bond area
constexpr float kStrengthPa = 1.0e12f;
constexpr int kResetCount = 50;

struct AlignedBlock {
  blast::TrackingAllocator* alloc = nullptr;
  void* ptr = nullptr;

  AlignedBlock() = default;
  AlignedBlock(const AlignedBlock&) = delete;
  AlignedBlock& operator=(const AlignedBlock&) = delete;
  AlignedBlock(AlignedBlock&& o) noexcept : alloc(o.alloc), ptr(o.ptr) {
    o.ptr = nullptr;
  }
  AlignedBlock& operator=(AlignedBlock&& o) noexcept {
    if (this != &o) {
      reset();
      alloc = o.alloc;
      ptr = o.ptr;
      o.ptr = nullptr;
    }
    return *this;
  }
  ~AlignedBlock() { reset(); }

  void reset() {
    if (alloc && ptr) {
      alloc->deallocate(ptr);
      ptr = nullptr;
    }
  }
};

AlignedBlock allocBlock(blast::TrackingAllocator& alloc, std::size_t bytes, const char* name) {
  AlignedBlock b;
  b.alloc = &alloc;
  b.ptr = alloc.allocate(bytes, name, __FILE__, __LINE__);
  return b;
}

struct TwoNodeScene {
  blast::IdMaps ids;
  AlignedBlock assetMem;
  AlignedBlock familyMem;
  NvBlastAsset* asset = nullptr;
  NvBlastFamily* family = nullptr;
  NvBlastActor* actor = nullptr;
  ExtStressSolver* solver = nullptr;
};

void fillChunk(NvBlastChunkDesc& c, float x, float y, float z, float volume, uint32_t parent, uint32_t flags,
               uint32_t userData) {
  c.centroid[0] = x;
  c.centroid[1] = y;
  c.centroid[2] = z;
  c.volume = volume;
  c.parentChunkDescIndex = parent;
  c.flags = flags;
  c.userData = userData;
}

void fillBond(NvBlastBondDesc& b, uint32_t a, uint32_t c, float nx, float ny, float nz, float area, float cx, float cy,
              float cz, uint32_t userData) {
  b.chunkIndices[0] = a;
  b.chunkIndices[1] = c;
  b.bond.normal[0] = nx;
  b.bond.normal[1] = ny;
  b.bond.normal[2] = nz;
  b.bond.area = area;
  b.bond.centroid[0] = cx;
  b.bond.centroid[1] = cy;
  b.bond.centroid[2] = cz;
  b.bond.userData = userData;
}

bool findBondNodes(const NvBlastSupportGraph& graph, uint32_t bondIndex, uint32_t& n0, uint32_t& n1) {
  for (uint32_t node = 0; node < graph.nodeCount; ++node) {
    for (uint32_t adj = graph.adjacencyPartition[node]; adj < graph.adjacencyPartition[node + 1]; ++adj) {
      if (graph.adjacentBondIndices[adj] == bondIndex) {
        n0 = node;
        n1 = graph.adjacentNodeIndices[adj];
        return true;
      }
    }
  }
  return false;
}

bool buildScene(blast::TrackingAllocator& alloc, TwoNodeScene& scene) {
  scene = TwoNodeScene{};
  const blast::StableIds stable = scene.ids.stable;

  NvBlastChunkDesc chunks[2];
  fillChunk(chunks[0], 0.0f, 0.5f * kMeters, 0.0f, kVolume, UINT32_MAX, NvBlastChunkDesc::SupportFlag, stable.baseNode);
  fillChunk(chunks[1], 0.0f, 1.5f * kMeters, 0.0f, kVolume, UINT32_MAX, NvBlastChunkDesc::SupportFlag, stable.topNode);

  NvBlastBondDesc bonds[2];
  fillBond(bonds[0], 0, 1, 0.0f, 1.0f, 0.0f, kAreaEff, 0.0f, 1.0f * kMeters, 0.0f, stable.internalBond);
  fillBond(bonds[1], 0, UINT32_MAX, 0.0f, 1.0f, 0.0f, kAreaEff, 0.0f, 0.0f, 0.0f, stable.worldBond);

  std::vector<char> scratch(4096);
  if (!NvBlastEnsureAssetExactSupportCoverage(chunks, 2, scratch.data(), blastLog)) {
    // Coverage may already be exact; continue either way.
  }

  uint32_t chunkReorderMap[2] = {0, 1};
  NvBlastReorderAssetDescChunks(chunks, 2, bonds, 2, chunkReorderMap, true, scratch.data(), blastLog);

  NvBlastAssetDesc desc{};
  desc.chunkCount = 2;
  desc.chunkDescs = chunks;
  desc.bondCount = 2;
  desc.bondDescs = bonds;

  const size_t createScratch = NvBlastGetRequiredScratchForCreateAsset(&desc, blastLog);
  if (scratch.size() < createScratch) {
    scratch.resize(createScratch);
  }
  const size_t assetBytes = NvBlastGetAssetMemorySize(&desc, blastLog);
  expect(assetBytes > 0, "asset memory size");
  scene.assetMem = allocBlock(alloc, assetBytes, "NvBlastAsset");
  scene.asset = NvBlastCreateAsset(scene.assetMem.ptr, &desc, scratch.data(), blastLog);
  expect(scene.asset != nullptr, "NvBlastCreateAsset");
  if (!scene.asset) {
    return false;
  }

  const uint32_t chunkCount = NvBlastAssetGetChunkCount(scene.asset, blastLog);
  const NvBlastChunk* builtChunks = NvBlastAssetGetChunks(scene.asset, blastLog);
  for (uint32_t i = 0; i < chunkCount; ++i) {
    if (builtChunks[i].userData == stable.baseNode) {
      scene.ids.chunkBase = i;
    } else if (builtChunks[i].userData == stable.topNode) {
      scene.ids.chunkTop = i;
    }
  }

  const uint32_t bondCount = NvBlastAssetGetBondCount(scene.asset, blastLog);
  const NvBlastBond* builtBonds = NvBlastAssetGetBonds(scene.asset, blastLog);
  for (uint32_t i = 0; i < bondCount; ++i) {
    if (builtBonds[i].userData == stable.internalBond) {
      scene.ids.bondInternal = i;
    } else if (builtBonds[i].userData == stable.worldBond) {
      scene.ids.bondWorld = i;
    }
  }

  const uint32_t* chunkToGraph = NvBlastAssetGetChunkToGraphNodeMap(scene.asset, blastLog);
  scene.ids.graphBase = chunkToGraph[scene.ids.chunkBase];
  scene.ids.graphTop = chunkToGraph[scene.ids.chunkTop];

  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(scene.asset, blastLog);
  for (uint32_t i = 0; i < graph.nodeCount; ++i) {
    if (graph.chunkIndices[i] == UINT32_MAX) {
      scene.ids.graphWorld = i;
    }
  }

  expect(scene.ids.chunkBase != blast::kInvalidIndex, "chunkBase mapped");
  expect(scene.ids.chunkTop != blast::kInvalidIndex, "chunkTop mapped");
  expect(scene.ids.graphBase != blast::kInvalidIndex, "graphBase mapped");
  expect(scene.ids.graphTop != blast::kInvalidIndex, "graphTop mapped");
  expect(scene.ids.graphWorld != blast::kInvalidIndex, "graphWorld mapped");
  expect(scene.ids.bondInternal != blast::kInvalidIndex, "bondInternal mapped");
  expect(scene.ids.bondWorld != blast::kInvalidIndex, "bondWorld mapped");
  expect(scene.ids.graphWorld != scene.ids.graphBase, "world graph node is not base");
  expect(scene.ids.graphWorld != scene.ids.graphTop, "world graph node is not top");
  expect(scene.ids.bondInternal != scene.ids.bondWorld, "internal and world SDK bond indices differ");

  std::vector<float> bondHealths(bondCount, kAreaEff);
  bondHealths[scene.ids.bondInternal] = kAreaEff;
  bondHealths[scene.ids.bondWorld] = Nv::Blast::kUnbreakableLimit * 2.0f;

  const size_t familyBytes = NvBlastAssetGetFamilyMemorySize(scene.asset, blastLog);
  scene.familyMem = allocBlock(alloc, familyBytes, "NvBlastFamily");
  scene.family = NvBlastAssetCreateFamily(scene.familyMem.ptr, scene.asset, blastLog);
  expect(scene.family != nullptr, "NvBlastAssetCreateFamily");
  if (!scene.family) {
    return false;
  }

  NvBlastActorDesc actorDesc{};
  actorDesc.uniformInitialBondHealth = kAreaEff;
  actorDesc.initialBondHealths = bondHealths.data();
  actorDesc.uniformInitialLowerSupportChunkHealth = kAreaEff;
  actorDesc.initialSupportChunkHealths = nullptr;

  const size_t actorScratch = NvBlastFamilyGetRequiredScratchForCreateFirstActor(scene.family, blastLog);
  if (scratch.size() < actorScratch) {
    scratch.resize(actorScratch);
  }
  scene.actor = NvBlastFamilyCreateFirstActor(scene.family, &actorDesc, scratch.data(), blastLog);
  expect(scene.actor != nullptr, "NvBlastFamilyCreateFirstActor");
  if (!scene.actor) {
    return false;
  }
  expect(NvBlastActorHasExternalBonds(scene.actor, blastLog), "actor has world/external bond");

  ExtStressSolverSettings settings;
  settings.maxSolverIterationsPerFrame = 25;
  settings.graphReductionLevel = 0;
  settings.compressionElasticLimit = kStrengthPa;
  settings.compressionFatalLimit = 2.0f * kStrengthPa;
  settings.tensionElasticLimit = kStrengthPa;
  settings.tensionFatalLimit = 2.0f * kStrengthPa;
  settings.shearElasticLimit = kStrengthPa;
  settings.shearFatalLimit = 2.0f * kStrengthPa;

  scene.solver = ExtStressSolver::create(*scene.family, settings);
  expect(scene.solver != nullptr, "ExtStressSolver::create");
  if (!scene.solver) {
    return false;
  }

  const float mass = kDensity * kVolume;
  scene.solver->setNodeInfo(scene.ids.graphBase, mass, kVolume, NvcVec3{0.0f, 0.5f * kMeters, 0.0f});
  scene.solver->setNodeInfo(scene.ids.graphTop, mass, kVolume, NvcVec3{0.0f, 1.5f * kMeters, 0.0f});
  scene.solver->setNodeInfo(scene.ids.graphWorld, 0.0f, 0.0f, NvcVec3{0.0f, 0.0f, 0.0f});
  expect(scene.solver->notifyActorCreated(*scene.actor), "notifyActorCreated");
  return gFailures == 0;
}

void destroyScene(TwoNodeScene& scene) {
  if (scene.solver) {
    scene.solver->release();
    scene.solver = nullptr;
  }
  scene.actor = nullptr;
  scene.family = nullptr;
  scene.asset = nullptr;
  scene.familyMem.reset();
  scene.assetMem.reset();
}

void runGravity(TwoNodeScene& scene, bool verbose) {
  const bool applied = scene.solver->addGravity(*scene.actor, NvcVec3{0.0f, kGravityY, 0.0f});
  expect(applied, "addGravity applied to at least one node");

  const auto t0 = std::chrono::steady_clock::now();
  scene.solver->update();
  const auto t1 = std::chrono::steady_clock::now();
  const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

  const float lin = scene.solver->getStressErrorLinear();
  const float ang = scene.solver->getStressErrorAngular();
  expect(std::isfinite(lin), "linear residual is finite");
  expect(std::isfinite(ang), "angular residual is finite");
  if (verbose) {
    std::cout << "stress.update " << ms << " ms  lin=" << lin << " ang=" << ang
              << " converged=" << (scene.solver->converged() ? 1 : 0)
              << " overstressed=" << scene.solver->getOverstressedBondCount() << "\n";
  }
}

void manualBreakInternal(TwoNodeScene& scene) {
  const float* healths = NvBlastActorGetBondHealths(scene.actor, blastLog);
  expect(healths != nullptr, "bond health array");
  const float internalHealth = healths[scene.ids.bondInternal];
  const float worldHealth = healths[scene.ids.bondWorld];
  expect(internalHealth > 0.0f, "internal bond health > 0 before break");
  expect(worldHealth >= Nv::Blast::kUnbreakableLimit, "world bond unbreakable before break");
  expect(canTakeDamage(internalHealth), "internal bond can take damage");
  expect(!canTakeDamage(worldHealth), "world bond cannot take damage");

  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(scene.asset, blastLog);
  uint32_t n0 = 0;
  uint32_t n1 = 0;
  expect(findBondNodes(graph, scene.ids.bondInternal, n0, n1), "internal bond in support graph");

  NvBlastBondFractureData frac{};
  frac.userdata = scene.ids.stable.internalBond;
  frac.nodeIndex0 = n0;
  frac.nodeIndex1 = n1;
  frac.health = internalHealth;

  NvBlastFractureBuffers cmds{};
  cmds.bondFractureCount = 1;
  cmds.bondFractures = &frac;
  cmds.chunkFractureCount = 0;
  cmds.chunkFractures = nullptr;
  NvBlastActorApplyFracture(nullptr, scene.actor, &cmds, blastLog, nullptr);

  const float* after = NvBlastActorGetBondHealths(scene.actor, blastLog);
  expect(after[scene.ids.bondInternal] <= 0.0f, "internal bond health is zero after manual break");
  expect(after[scene.ids.bondWorld] >= Nv::Blast::kUnbreakableLimit, "world bond still unbreakable");
  expect(NvBlastActorIsSplitRequired(scene.actor, blastLog), "split required after internal bond break");

  scene.solver->notifyActorDestroyed(*scene.actor);

  const uint32_t maxNew = NvBlastActorGetMaxActorCountForSplit(scene.actor, blastLog);
  std::vector<NvBlastActor*> created(maxNew, nullptr);
  NvBlastActorSplitEvent ev{};
  ev.newActors = created.data();
  std::vector<char> scratch(NvBlastActorGetRequiredScratchForSplit(scene.actor, blastLog));
  const uint32_t newCount =
      NvBlastActorSplit(&ev, scene.actor, maxNew, scratch.data(), blastLog, nullptr);
  expect(newCount >= 1, "split produced at least one actor");

  uint32_t active = NvBlastFamilyGetActorCount(scene.family, blastLog);
  expect(active >= 2, "family has at least two actors after split");

  bool sawWorld = false;
  bool sawFree = false;
  std::vector<NvBlastActor*> actors(active, nullptr);
  NvBlastFamilyGetActors(actors.data(), active, scene.family, blastLog);
  for (uint32_t i = 0; i < active; ++i) {
    NvBlastActor* a = actors[i];
    if (!a) {
      continue;
    }
    scene.solver->notifyActorCreated(*a);
    if (NvBlastActorHasExternalBonds(a, blastLog)) {
      sawWorld = true;
    } else {
      sawFree = true;
    }
  }
  expect(sawWorld, "one actor still has the world bond");
  expect(sawFree, "one actor is free of world bonds after split");
  (void)ev;
}

void printMaps(const blast::IdMaps& ids) {
  std::cout << "ID maps (stable != SDK index):\n"
            << "  stableNode " << ids.stable.baseNode << " -> chunkIndex " << ids.chunkBase << " graphNode "
            << ids.graphBase << "\n"
            << "  stableNode " << ids.stable.topNode << " -> chunkIndex " << ids.chunkTop << " graphNode "
            << ids.graphTop << "\n"
            << "  world graphNode " << ids.graphWorld << "\n"
            << "  stableBond " << ids.stable.internalBond << " -> sdkBond " << ids.bondInternal << "\n"
            << "  stableBond " << ids.stable.worldBond << " -> sdkBond " << ids.bondWorld << "\n";
}

void runOnce(blast::TrackingAllocator& alloc, bool verbose) {
  TwoNodeScene scene;
  if (!buildScene(alloc, scene)) {
    destroyScene(scene);
    return;
  }
  if (verbose) {
    printMaps(scene.ids);
  }
  runGravity(scene, verbose);
  manualBreakInternal(scene);
  destroyScene(scene);
}

}  // namespace

int main() {
  std::cout << "blast_p0_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  std::cout << "units: m, kg, s, N, Pa; density=" << kDensity << " kg/m^3 g=" << kGravityY << " m/s^2\n";

  blast::TrackingAllocator alloc;
  blast::LoggingErrorCallback errors;
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);

  const std::size_t live0 = alloc.liveBytes();
  runOnce(alloc, true);
  expect(alloc.liveBytes() == live0, "live bytes return to baseline after first teardown");
  expect(alloc.liveAllocs() == 0 || alloc.liveBytes() == live0, "no leftover allocs after first teardown");

  const std::size_t afterFirst = alloc.liveBytes();
  for (int i = 0; i < kResetCount; ++i) {
    runOnce(alloc, false);
    expect(alloc.liveBytes() == afterFirst, "reset live bytes stable at iteration " + std::to_string(i));
    if (gFailures > 0) {
      std::cerr << "stopping reset loop at iteration " << i << "\n";
      break;
    }
  }

  expect(errors.errorCount() == 0, "no NvBlast error-callback errors");

  if (gFailures == 0) {
    std::cout << "OK  resets=" << kResetCount << " liveBytes=" << alloc.liveBytes()
              << " totalAllocs=" << alloc.totalAllocs() << "\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
