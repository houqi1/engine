#include "blast/CylinderGraph.h"

#include "NvBlastGlobals.h"
#include "NvCTypes.h"

#include <cmath>
#include <cstring>
#include <vector>

namespace blast {
namespace {

constexpr float kPi = 3.14159265358979323846f;

void blastLog(int type, const char* msg, const char* file, int line) {
  Nv::Blast::logLL(type, msg, file, line);
}

AlignedBlock allocBlock(TrackingAllocator& alloc, std::size_t bytes, const char* name) {
  AlignedBlock b;
  b.alloc = &alloc;
  b.ptr = alloc.allocate(bytes, name, __FILE__, __LINE__);
  return b;
}

void fillChunk(NvBlastChunkDesc& c, float x, float y, float z, float volume, uint32_t userData) {
  c.centroid[0] = x;
  c.centroid[1] = y;
  c.centroid[2] = z;
  c.volume = volume;
  c.parentChunkDescIndex = UINT32_MAX;
  c.flags = NvBlastChunkDesc::SupportFlag;
  c.userData = userData;
}

void fillBond(NvBlastBondDesc& b, uint32_t a, uint32_t c, float nx, float ny, float nz, float area, float cx, float cy,
              float cz, uint32_t userData) {
  const float len = std::sqrt(nx * nx + ny * ny + nz * nz);
  b.chunkIndices[0] = a;
  b.chunkIndices[1] = c;
  b.bond.normal[0] = nx / len;
  b.bond.normal[1] = ny / len;
  b.bond.normal[2] = nz / len;
  b.bond.area = area;
  b.bond.centroid[0] = cx;
  b.bond.centroid[1] = cy;
  b.bond.centroid[2] = cz;
  b.bond.userData = userData;
}

void nodePos(int k, int j, float& x, float& y, float& z) {
  const float th = (2.0f * kPi * static_cast<float>(k)) / static_cast<float>(kCylinderCirc);
  x = kCylinderR * std::cos(th);
  y = (static_cast<float>(j) + 0.5f) * cylinderLayerHeight();
  z = kCylinderR * std::sin(th);
}

}  // namespace

int nodeIndex(int k, int j) { return j * kCylinderCirc + k; }

void nodeKJ(int node, int& k, int& j) {
  j = node / kCylinderCirc;
  k = node % kCylinderCirc;
}

void setCylinderNodeMasses(CylinderScene& scene, float density) {
  const float mass = density * cylinderNodeVolume();
  const float volume = cylinderNodeVolume();
  for (int j = 0; j < kCylinderLayers; ++j) {
    for (int k = 0; k < kCylinderCirc; ++k) {
      float x, y, z;
      nodePos(k, j, x, y, z);
      const uint32_t g = scene.ids.graphFromNode[static_cast<size_t>(nodeIndex(k, j))];
      scene.solver->setNodeInfo(g, mass, volume, NvcVec3{x, y, z});
    }
  }
  if (scene.ids.graphWorld != kInvalidIndex) {
    scene.solver->setNodeInfo(scene.ids.graphWorld, 0.0f, 0.0f, NvcVec3{0.0f, 0.0f, 0.0f});
  }
}

bool solveGravity(CylinderScene& scene, int extraIters) {
  if (!scene.solver || !scene.actor) {
    return false;
  }
  scene.solver->addGravity(*scene.actor, NvcVec3{0.0f, kGravityY, 0.0f});
  scene.solver->update();
  for (int i = 0; i < extraIters; ++i) {
    scene.solver->addGravity(*scene.actor, NvcVec3{0.0f, kGravityY, 0.0f});
    scene.solver->update();
  }
  return true;
}

void destroyCylinderScene(CylinderScene& scene) {
  if (scene.solver) {
    scene.solver->release();
    scene.solver = nullptr;
  }
  scene.actor = nullptr;
  scene.family = nullptr;
  scene.asset = nullptr;
  scene.familyMem.reset();
  scene.assetMem.reset();
  scene.ids = CylinderIdMaps{};
}

bool buildCylinderScene(TrackingAllocator& alloc, CylinderScene& scene, const CylinderBuildOpts& opts) {
  destroyCylinderScene(scene);

  const int vertCount = kCylinderCirc * (kCylinderLayers - 1);
  const int circCount = kCylinderCirc * kCylinderLayers;
  const int worldCount = opts.worldAnchor ? kCylinderCirc : 0;
  const int bondCount = vertCount + circCount + worldCount;
  const int chunkCount = kCylinderNodeCount;

  std::vector<NvBlastChunkDesc> chunks(static_cast<size_t>(chunkCount));
  std::vector<NvBlastBondDesc> bonds(static_cast<size_t>(bondCount));

  for (int j = 0; j < kCylinderLayers; ++j) {
    for (int k = 0; k < kCylinderCirc; ++k) {
      float x, y, z;
      nodePos(k, j, x, y, z);
      fillChunk(chunks[static_cast<size_t>(nodeIndex(k, j))], x, y, z, cylinderNodeVolume(),
                cylinderNodeStable(k, j));
    }
  }

  int bi = 0;
  for (int j = 0; j < kCylinderLayers - 1; ++j) {
    for (int k = 0; k < kCylinderCirc; ++k) {
      const int a = nodeIndex(k, j);
      const int c = nodeIndex(k, j + 1);
      float x, y0, y1, z;
      nodePos(k, j, x, y0, z);
      nodePos(k, j + 1, x, y1, z);
      fillBond(bonds[static_cast<size_t>(bi++)], static_cast<uint32_t>(a), static_cast<uint32_t>(c), 0.0f, 1.0f, 0.0f,
               cylinderVertArea(), x, 0.5f * (y0 + y1), z, cylinderVertBondStable(k, j));
    }
  }
  for (int j = 0; j < kCylinderLayers; ++j) {
    for (int k = 0; k < kCylinderCirc; ++k) {
      const int k1 = (k + 1) % kCylinderCirc;
      const int a = nodeIndex(k, j);
      const int c = nodeIndex(k1, j);
      float x0, y, z0, x1, z1;
      nodePos(k, j, x0, y, z0);
      nodePos(k1, j, x1, y, z1);
      float nx = x1 - x0;
      float nz = z1 - z0;
      fillBond(bonds[static_cast<size_t>(bi++)], static_cast<uint32_t>(a), static_cast<uint32_t>(c), nx, 0.0f, nz,
               cylinderCircArea(), 0.5f * (x0 + x1), y, 0.5f * (z0 + z1), cylinderCircBondStable(k, j));
    }
  }
  if (opts.worldAnchor) {
    for (int k = 0; k < kCylinderCirc; ++k) {
      float x, y, z;
      nodePos(k, 0, x, y, z);
      fillBond(bonds[static_cast<size_t>(bi++)], static_cast<uint32_t>(nodeIndex(k, 0)), UINT32_MAX, 0.0f, 1.0f, 0.0f,
               cylinderVertArea(), x, 0.0f, z, cylinderWorldBondStable(k));
    }
  }

  std::vector<char> scratch(static_cast<size_t>(chunkCount) * sizeof(NvBlastChunkDesc) + 4096);
  NvBlastEnsureAssetExactSupportCoverage(chunks.data(), static_cast<uint32_t>(chunkCount), scratch.data(), blastLog);
  std::vector<uint32_t> reorder(static_cast<size_t>(chunkCount));
  NvBlastReorderAssetDescChunks(chunks.data(), static_cast<uint32_t>(chunkCount), bonds.data(),
                                static_cast<uint32_t>(bondCount), reorder.data(), true, scratch.data(), blastLog);

  NvBlastAssetDesc desc{};
  desc.chunkCount = static_cast<uint32_t>(chunkCount);
  desc.chunkDescs = chunks.data();
  desc.bondCount = static_cast<uint32_t>(bondCount);
  desc.bondDescs = bonds.data();

  const size_t createScratch = NvBlastGetRequiredScratchForCreateAsset(&desc, blastLog);
  if (scratch.size() < createScratch) {
    scratch.resize(createScratch);
  }
  const size_t assetBytes = NvBlastGetAssetMemorySize(&desc, blastLog);
  scene.assetMem = allocBlock(alloc, assetBytes, "NvBlastAsset");
  scene.asset = NvBlastCreateAsset(scene.assetMem.ptr, &desc, scratch.data(), blastLog);
  if (!scene.asset) {
    return false;
  }

  const uint32_t builtChunks = NvBlastAssetGetChunkCount(scene.asset, blastLog);
  const NvBlastChunk* chunkArr = NvBlastAssetGetChunks(scene.asset, blastLog);
  for (uint32_t i = 0; i < builtChunks; ++i) {
    const uint32_t stable = chunkArr[i].userData;
    if (stable >= 10000u && stable < 10000u + static_cast<uint32_t>(kCylinderNodeCount)) {
      scene.ids.chunkFromNode[stable - 10000u] = i;
    }
  }
  const uint32_t* chunkToGraph = NvBlastAssetGetChunkToGraphNodeMap(scene.asset, blastLog);
  for (int n = 0; n < kCylinderNodeCount; ++n) {
    scene.ids.graphFromNode[n] = chunkToGraph[scene.ids.chunkFromNode[n]];
  }
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(scene.asset, blastLog);
  for (uint32_t i = 0; i < graph.nodeCount; ++i) {
    if (graph.chunkIndices[i] == UINT32_MAX) {
      scene.ids.graphWorld = i;
    }
  }

  const uint32_t builtBonds = NvBlastAssetGetBondCount(scene.asset, blastLog);
  const NvBlastBond* bondArr = NvBlastAssetGetBonds(scene.asset, blastLog);
  for (uint32_t i = 0; i < builtBonds; ++i) {
    const uint32_t u = bondArr[i].userData;
    if (u >= 20000u && u < 30000u) {
      scene.ids.sdkBondFromVert[u - 20000u] = i;
    } else if (u >= 30000u && u < 40000u) {
      scene.ids.sdkBondFromCirc[u - 30000u] = i;
    } else if (u >= 40000u && u < 40000u + static_cast<uint32_t>(kCylinderCirc)) {
      scene.ids.sdkBondFromWorld[u - 40000u] = i;
    }
  }

  std::vector<float> healths(builtBonds, cylinderVertArea());
  for (uint32_t i = 0; i < builtBonds; ++i) {
    const uint32_t u = bondArr[i].userData;
    if (u >= 30000u && u < 40000u) {
      healths[i] = cylinderCircArea();
    } else if (u >= 40000u) {
      healths[i] = Nv::Blast::kUnbreakableLimit * 2.0f;
    } else {
      healths[i] = cylinderVertArea();
    }
  }

  const size_t familyBytes = NvBlastAssetGetFamilyMemorySize(scene.asset, blastLog);
  scene.familyMem = allocBlock(alloc, familyBytes, "NvBlastFamily");
  scene.family = NvBlastAssetCreateFamily(scene.familyMem.ptr, scene.asset, blastLog);
  if (!scene.family) {
    return false;
  }

  NvBlastActorDesc actorDesc{};
  actorDesc.uniformInitialBondHealth = cylinderVertArea();
  actorDesc.initialBondHealths = healths.data();
  actorDesc.uniformInitialLowerSupportChunkHealth = cylinderVertArea();
  actorDesc.initialSupportChunkHealths = nullptr;

  const size_t actorScratch = NvBlastFamilyGetRequiredScratchForCreateFirstActor(scene.family, blastLog);
  if (scratch.size() < actorScratch) {
    scratch.resize(actorScratch);
  }
  scene.actor = NvBlastFamilyCreateFirstActor(scene.family, &actorDesc, scratch.data(), blastLog);
  if (!scene.actor) {
    return false;
  }

  Nv::Blast::ExtStressSolverSettings settings;
  settings.maxSolverIterationsPerFrame = opts.solverIters;
  settings.graphReductionLevel = 0;
  settings.compressionElasticLimit = opts.strengthPa;
  settings.compressionFatalLimit = 2.0f * opts.strengthPa;
  settings.tensionElasticLimit = opts.strengthPa;
  settings.tensionFatalLimit = 2.0f * opts.strengthPa;
  settings.shearElasticLimit = opts.strengthPa;
  settings.shearFatalLimit = 2.0f * opts.strengthPa;

  scene.solver = Nv::Blast::ExtStressSolver::create(*scene.family, settings);
  if (!scene.solver) {
    return false;
  }
  setCylinderNodeMasses(scene, opts.density);
  scene.solver->notifyActorCreated(*scene.actor);
  return true;
}

}  // namespace blast
