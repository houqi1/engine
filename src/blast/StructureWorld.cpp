#include "blast/StructureWorld.h"

#include "blast/CylinderVoxels.h"
#include "blast/HardFracture.h"
#include "blast/ImpactDamage.h"

#include "NvBlast.h"
#include "NvBlastExtDamageShaders.h"
#include "NvBlastTypes.h"
#include "NvCTypes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <map>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

#include <glm/gtc/quaternion.hpp>

namespace blast {
namespace {

void blastLog(int type, const char* msg, const char* file, int line) {
  (void)type;
  (void)msg;
  (void)file;
  (void)line;
}

float bondAzimuth(const GraphBond& b, float axisX, float axisZ) { return std::atan2(b.cz - axisZ, b.cx - axisX); }

glm::ivec3 unpackFineCoord(uint32_t i, int n) {
  if (n <= 0) {
    return glm::ivec3(0);
  }
  const int nn = n * n;
  const int z = static_cast<int>(i) / nn;
  const int y = (static_cast<int>(i) - z * nn) / n;
  const int x = static_cast<int>(i) - z * nn - y * n;
  return glm::ivec3(x, y, z);
}

uint32_t sdkBondFromNodes(const NvBlastSupportGraph& graph, uint32_t n0, uint32_t n1) {
  if (n0 > n1) {
    std::swap(n0, n1);
  }
  if (n0 >= graph.nodeCount) {
    return UINT32_MAX;
  }
  for (uint32_t adj = graph.adjacencyPartition[n0]; adj < graph.adjacencyPartition[n0 + 1]; ++adj) {
    if (graph.adjacentNodeIndices[adj] == n1) {
      return graph.adjacentBondIndices[adj];
    }
  }
  return UINT32_MAX;
}

}  // namespace

bool StructureWorld::init(BlastRuntime& runtime) {
  if (!runtime.initialized()) {
    return false;
  }
  shutdown();
  runtime_ = &runtime;
  initialized_ = true;
  clear();
  return true;
}

void StructureWorld::shutdown() {
  clear();
  initialized_ = false;
  runtime_ = nullptr;
}

void StructureWorld::clear() {
  instances_.clear();
  idle_ = StructureDebugSnapshot{};
  lastError_ = "ok";
  resetTickSession();
}

void StructureWorld::resetTickSession() {
  ticksReceived_ = 0;
  lastTickId_ = 0;
  lastDt_ = 0.0f;
  for (StructureInstance& inst : instances_) {
    inst.impactEvents.reset();
  }
}

void StructureWorld::unmount(VoxelObjectId objectId) {
  instances_.erase(std::remove_if(instances_.begin(), instances_.end(),
                                  [&](const StructureInstance& i) { return i.objectId == objectId; }),
                   instances_.end());
}

void StructureWorld::markCut(bool cut) {
  if (!instances_.empty()) {
    instances_.front().cutApplied = cut;
    instances_.front().debug.cut = cut;
  }
}

uint32_t StructureWorld::warmupGravity(uint32_t maxPasses) {
  StructureInstance* inst = instance();
  if (inst == nullptr || inst->blast.solver == nullptr || maxPasses == 0) {
    return 0;
  }
  if (inst->bindings.empty()) {
    rebuildBindingsFromFamily(*inst);
  }
  uint32_t n = 0;
  for (; n < maxPasses; ++n) {
    for (ActorBinding& b : inst->bindings) {
      if (b.actor == nullptr || !b.stressSolve || !b.anchored) {
        continue;
      }
      inst->blast.solver->addGravity(*b.actor, NvcVec3{0.0f, kE1GravityY, 0.0f});
    }
    inst->blast.solver->update();
    ++inst->solveEpoch;
    refreshDebug(*inst, 0.0f);
    if (inst->debug.converged) {
      break;
    }
  }
  collectPendingFracture(*inst);
  return n + 1;
}

void StructureWorld::setKeepColumnBox(float x0, float x1, float z0, float z1) {
  if (instances_.empty()) {
    return;
  }
  StructureInstance& inst = instances_.front();
  inst.keepBox = true;
  inst.keepX0 = x0;
  inst.keepX1 = x1;
  inst.keepZ0 = z0;
  inst.keepZ1 = z1;
  rebuildBondMeta(inst);
}

BlastError StructureWorld::mountSample(VoxelObjectId objectId, OccupancySample sample, uint32_t solverIters,
                                       float strengthPa, float axisX, float axisZ) {
  lastError_ = "ok";
  if (!initialized_ || runtime_ == nullptr) {
    lastError_ = "runtime not initialized";
    return BlastError::SolverCreateFailed;
  }
  if (sample.error != BlastError::Ok) {
    lastError_ = sample.message;
    return sample.error;
  }
  StructureInstance inst;
  inst.objectId = objectId;
  inst.grid = std::move(sample.grid);
  inst.graph = std::move(sample.graph);
  inst.axisX = axisX;
  inst.axisZ = axisZ;
  inst.keepAz0 = kE1KeepAz0;
  inst.keepAz1 = kE1KeepAz1;
  inst.solverIters = solverIters == 0 ? 200u : solverIters;
  inst.debug.extractMs = 0.0f;
  bool keepFrac = false;
  float keepS = strengthPa;
  uint64_t keepStrengthEpoch = 1;
  if (StructureInstance* old = instance()) {
    keepFrac = old->fractureEnabled;
    keepS = old->strengthPa;
    keepStrengthEpoch = old->strengthEpoch == 0 ? 1 : old->strengthEpoch;
  }
  const auto t0 = std::chrono::steady_clock::now();
  const BlastError ce =
      createVoxelBlast(runtime_->allocator(), inst.graph, inst.grid, inst.blast, keepS, inst.solverIters);
  const auto t1 = std::chrono::steady_clock::now();
  inst.debug.assetMs = std::chrono::duration<float, std::milli>(t1 - t0).count();
  if (ce != BlastError::Ok) {
    lastError_ = blastErrorMessage(ce);
    return ce;
  }
  const uint32_t nb = inst.blast.actor != nullptr ? familyAssetBondCount(inst.blast.actor, blastLog) : 0;
  if (inst.probes.size() < nb) {
    inst.probes.resize(nb);
  }
  rebuildBondMeta(inst);
  inst.occupiedCached = sample.occupied;
  inst.probeCount = 0;
  inst.debug.hasInstance = true;
  inst.debug.status = "mounted";
  inst.debug.occupied = inst.occupiedCached;
  inst.debug.nodes = static_cast<uint32_t>(inst.graph.nodes.size());
  inst.debug.bonds = static_cast<uint32_t>(inst.graph.bonds.size());
  inst.debug.worldBonds = sample.worldBonds;
  inst.debug.mass = sample.mass;
  inst.debug.weight = sample.mass * -kE1GravityY;
  inst.debug.density = inst.grid.density;
  inst.debug.solverIters = inst.solverIters;
  inst.debug.assetMs = inst.debug.assetMs;
  inst.debug.probeExportCount = 0;
  inst.fractureEnabled = keepFrac;
  inst.strengthPa = keepS;
  inst.strengthEpoch = keepStrengthEpoch;
  inst.debug.fractureEnabled = keepFrac;
  inst.debug.strengthPa = keepS;
  if (inst.blast.solver != nullptr) {
    applyExtStressStrength(*inst.blast.solver, keepS);
  }
  rebuildBindingsFromFamily(inst);
  if (!inst.bindings.empty()) {
    inst.bindings.front().objectId = objectId;
  }
  unmount(objectId);
  instances_.push_back(std::move(inst));
  return BlastError::Ok;
}

bool StructureWorld::setDensityScale(float scale) {
  StructureInstance* inst = instance();
  if (inst == nullptr || inst->blast.solver == nullptr) {
    return false;
  }
  const float s = scale > 0.0f ? scale : 1.0f;
  const float baseRho = kE1Density;
  inst->grid.density = baseRho * s;
  for (GraphNode& n : inst->graph.nodes) {
    n.mass = inst->grid.density * n.volume;
    const uint32_t g = inst->blast.graphFromStable[n.stableId];
    inst->blast.solver->setNodeInfo(g, n.mass, n.volume, NvcVec3{n.cx, n.cy, n.cz});
  }
  inst->debug.mass = 0.0f;
  for (const GraphNode& n : inst->graph.nodes) {
    inst->debug.mass += n.mass;
  }
  inst->debug.weight = inst->debug.mass * -kE1GravityY;
  inst->debug.density = inst->grid.density;
  return true;
}

void StructureWorld::setSolverIters(uint32_t iters) {
  StructureInstance* inst = instance();
  if (inst == nullptr || inst->blast.solver == nullptr) {
    return;
  }
  inst->solverIters = iters == 0 ? 25u : iters;
  auto st = inst->blast.solver->getSettings();
  st.maxSolverIterationsPerFrame = inst->solverIters;
  inst->blast.solver->setSettings(st);
  inst->debug.solverIters = inst->solverIters;
}

void StructureWorld::rebuildBondMeta(StructureInstance& inst) {
  const uint32_t nb = inst.blast.actor != nullptr ? familyAssetBondCount(inst.blast.actor, blastLog) : 0;
  inst.bondMeta.assign(nb, BondMeta{});
  for (uint32_t gi = 0; gi < static_cast<uint32_t>(inst.graph.bonds.size()); ++gi) {
    const GraphBond& b = inst.graph.bonds[gi];
    const auto it = inst.blast.sdkBondFromStable.find(b.stableId);
    if (it == inst.blast.sdkBondFromStable.end() || it->second >= nb) {
      continue;
    }
    BondMeta& m = inst.bondMeta[it->second];
    m.stableId = b.stableId;
    m.graphIndex = gi;
    m.world = b.world ? 1 : 0;
    if (inst.keepBox) {
      m.inStrip = (!b.world && b.cx >= inst.keepX0 && b.cx < inst.keepX1 && b.cz >= inst.keepZ0 &&
                   b.cz < inst.keepZ1)
                      ? 1
                      : 0;
    } else {
      const float az = bondAzimuth(b, inst.axisX, inst.axisZ);
      m.inStrip = (!b.world && az >= inst.keepAz0 && az < inst.keepAz1) ? 1 : 0;
    }
    const float y0 = static_cast<float>(kE1AnchorFines) * kE1FineMeters;
    const float y1 = static_cast<float>(kE1AnchorFines + kE1CutFines) * kE1FineMeters;
    m.inNeck = (m.inStrip != 0 && b.cy >= y0 - 0.05f && b.cy < y1 + 0.05f) ? 1 : 0;
  }
}

void StructureWorld::recacheOccupied() {
  StructureInstance* inst = instance();
  if (inst == nullptr) {
    return;
  }
  inst->occupiedCached = countSolidVoxels(inst->grid);
  inst->debug.occupied = inst->occupiedCached;
}

void StructureWorld::recachePendingFromProbes() {
  StructureInstance* inst = instance();
  if (inst != nullptr) {
    collectPendingFracture(*inst);
  }
}

void StructureWorld::refreshDebug(StructureInstance& inst, float solveMs) {
  inst.debug.hasInstance = true;
  inst.debug.solveMs = solveMs;
  inst.debug.probeMs = 0.0f;
  inst.debug.statsMs = 0.0f;
  inst.debug.occupied = inst.occupiedCached;
  inst.debug.nodes = static_cast<uint32_t>(inst.graph.nodes.size());
  inst.debug.bonds = static_cast<uint32_t>(inst.graph.bonds.size());
  inst.debug.cut = inst.cutApplied;
  inst.debug.solverIters = inst.solverIters;
  inst.debug.density = inst.grid.density;
  inst.debug.probeExportCount = 0;
  inst.debug.probeCount = inst.probeCount;
  if (inst.blast.solver == nullptr || inst.blast.actor == nullptr) {
    inst.debug.converged = false;
    inst.debug.status = "no solver";
    return;
  }
  inst.debug.converged = inst.blast.solver->converged();
  inst.debug.linErr = inst.blast.solver->getStressErrorLinear();
  inst.debug.angErr = inst.blast.solver->getStressErrorAngular();
  inst.debug.status = inst.debug.converged ? "converged" : "未收敛";
  const uint32_t nb = familyAssetBondCount(inst.blast.actor, blastLog);
  if (inst.probes.size() < nb) {
    inst.probes.resize(nb);
  }
  const auto tProbe0 = std::chrono::steady_clock::now();
  const uint32_t n = inst.blast.solver->copyBondProbes(inst.probes.data(), nb);
  const auto tProbe1 = std::chrono::steady_clock::now();
  inst.probeCount = n;
  inst.debug.probeCount = n;
  inst.debug.probeExportCount = 1;
  inst.debug.solveEpoch = inst.solveEpoch;
  inst.debug.probeMs = std::chrono::duration<float, std::milli>(tProbe1 - tProbe0).count();
  float maxT = 0, maxC = 0, maxS = 0, strip = 0, ry = 0;
  const uint32_t metaN = static_cast<uint32_t>(inst.bondMeta.size());
  for (uint32_t i = 0; i < n; ++i) {
    const auto& p = inst.probes[i];
    maxT = std::max(maxT, p.tension);
    maxC = std::max(maxC, p.compression);
    maxS = std::max(maxS, p.shear);
    const uint32_t sdk = p.blastBondIndex;
    if (sdk >= metaN) {
      continue;
    }
    const BondMeta& m = inst.bondMeta[sdk];
    if (m.world != 0) {
      float fy = p.forceLinear.y;
      if (inst.blast.graphWorld != kInvalidIndex && p.node0 == inst.blast.graphWorld) {
        fy = -fy;
      }
      ry += fy;
    } else if (m.inStrip != 0) {
      strip = std::max(strip, probeMaxStress(p));
    }
  }
  const auto tStats1 = std::chrono::steady_clock::now();
  inst.debug.statsMs = std::chrono::duration<float, std::milli>(tStats1 - tProbe1).count();
  inst.debug.maxTension = maxT;
  inst.debug.maxCompression = maxC;
  inst.debug.maxShear = maxS;
  inst.debug.stripMaxStress = strip;
  inst.debug.reactionY = ry;
  inst.debug.weight = inst.debug.mass * -kE1GravityY;
  inst.debug.fractureEnabled = inst.fractureEnabled;
  inst.debug.strengthPa = inst.strengthPa;
}

void StructureWorld::setFractureEnabled(bool on) {
  StructureInstance* inst = instance();
  if (inst == nullptr) {
    return;
  }
  if (inst->fractureEnabled == on) {
    return;
  }
  inst->fractureEnabled = on;
  inst->debug.fractureEnabled = on;
  if (!on) {
    inst->pending.valid = false;
    inst->pending.candidates.clear();
  }
}

void StructureWorld::setStressImpactImpulses(bool on) { stressImpactImpulses_ = on; }

void StructureWorld::setStressImpactScale(float scale) {
  stressImpactScale_ = std::max(0.0f, scale);
}

void StructureWorld::setImpactDamageEnabled(bool on) { impactDamageEnabled_ = on; }

void StructureWorld::setImpactSettings(const ImpactSettings& settings) { impactSettings_ = settings; }

void StructureWorld::setStrengthPa(float strengthPa) {
  StructureInstance* inst = instance();
  if (inst == nullptr) {
    return;
  }
  const bool changed = inst->strengthPa != strengthPa || inst->debug.strengthPa != strengthPa;
  inst->strengthPa = strengthPa;
  inst->debug.strengthPa = strengthPa;
  if (changed) {
    ++inst->strengthEpoch;
  }
  if (inst->blast.solver != nullptr) {
    applyExtStressStrength(*inst->blast.solver, strengthPa);
  }
}

PendingFracture StructureWorld::takePendingFracture() {
  PendingFracture out;
  if (instances_.empty()) {
    return out;
  }
  PendingFracture& src = instances_.front().pending;
  out.valid = src.valid;
  out.objectId = src.objectId;
  out.topologyRevision = src.topologyRevision;
  out.solverTopologyEpoch = src.solverTopologyEpoch;
  out.solveEpoch = src.solveEpoch;
  out.strengthEpoch = src.strengthEpoch;
  out.strengthPa = src.strengthPa;
  out.candidates.swap(src.candidates);
  src.valid = false;
  src.candidates.clear();
  return out;
}

void StructureWorld::clearPendingFracture() {
  if (instances_.empty()) {
    return;
  }
  instances_.front().pending.valid = false;
  instances_.front().pending.candidates.clear();
}

bool StructureWorld::pendingSnapshotMatches(const PendingFracture& pending) const {
  const StructureInstance* inst = instance();
  if (inst == nullptr || inst->blast.solver == nullptr) {
    return false;
  }
  return pending.valid && pending.objectId.slot == inst->objectId.slot &&
         pending.objectId.generation == inst->objectId.generation &&
         pending.topologyRevision == inst->topologyRevision &&
         pending.solverTopologyEpoch == inst->blast.solver->topologyEpoch() &&
         pending.solveEpoch == inst->solveEpoch && pending.strengthEpoch == inst->strengthEpoch &&
         pending.strengthPa == inst->strengthPa;
}

void StructureWorld::fillNodeOwners(StructureInstance& inst, const NvBlastSupportGraph& graph) {
  const uint32_t nA = NvBlastFamilyGetActorCount(inst.blast.family, blastLog);
  if (inst.actorScratch.size() < nA) {
    inst.actorScratch.resize(nA);
  }
  NvBlastFamilyGetActors(inst.actorScratch.data(), nA, inst.blast.family, blastLog);
  if (inst.nodeOwner.size() < graph.nodeCount) {
    inst.nodeOwner.resize(graph.nodeCount);
  }
  std::fill(inst.nodeOwner.begin(), inst.nodeOwner.begin() + graph.nodeCount, 0u);
  for (uint32_t i = 0; i < nA; ++i) {
    NvBlastActor* a = inst.actorScratch[i];
    if (a == nullptr) {
      continue;
    }
    const uint32_t nn = NvBlastActorGetGraphNodeCount(a, blastLog);
    if (inst.nodeIndexScratch.size() < nn) {
      inst.nodeIndexScratch.resize(nn);
    }
    NvBlastActorGetGraphNodeIndices(inst.nodeIndexScratch.data(), nn, a, blastLog);
    for (uint32_t k = 0; k < nn; ++k) {
      const uint32_t node = inst.nodeIndexScratch[k];
      if (node < graph.nodeCount) {
        inst.nodeOwner[node] = i + 1;
      }
    }
  }
}

void StructureWorld::collectPendingFracture(StructureInstance& inst) {
  const auto t0 = std::chrono::steady_clock::now();
  inst.pending.valid = false;
  inst.pending.candidates.clear();
  inst.debug.candidateCount = 0;
  inst.debug.candidateInStrip = 0;
  inst.debug.candidateMs = 0.0f;
  if (!inst.fractureEnabled || inst.blast.family == nullptr || inst.blast.solver == nullptr ||
      inst.blast.asset == nullptr) {
    inst.debug.candidateMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return;
  }
  if (inst.blast.solver->getOverstressedBondCount() == 0) {
    inst.debug.candidateMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return;
  }
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(inst.blast.asset, blastLog);
  fillNodeOwners(inst, graph);
  const uint32_t nA = NvBlastFamilyGetActorCount(inst.blast.family, blastLog);
  if (nA == 0) {
    inst.debug.candidateMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return;
  }
  const uint32_t metaN = static_cast<uint32_t>(inst.bondMeta.size());
  inst.pending.candidates.reserve(inst.probeCount);
  for (uint32_t ai = 0; ai < nA; ++ai) {
    NvBlastActor* actor = ai < inst.actorScratch.size() ? inst.actorScratch[ai] : nullptr;
    if (actor == nullptr) {
      continue;
    }
    NvBlastFractureBuffers cmds{};
    inst.blast.solver->generateFractureCommands(*actor, cmds);
    if (cmds.bondFractureCount == 0 || cmds.bondFractures == nullptr) {
      continue;
    }
    const uint32_t owner = ai + 1;
    const bool anchored = NvBlastActorHasExternalBonds(actor, blastLog);
    for (uint32_t b = 0; b < cmds.bondFractureCount; ++b) {
      const NvBlastBondFractureData& d = cmds.bondFractures[b];
      if (!(d.health > 0.0f)) {
        continue;
      }
      FractureCandidate c;
      c.node0 = d.nodeIndex0;
      c.node1 = d.nodeIndex1;
      c.damage = d.health;
      c.owner = owner;
      c.anchored = anchored;
      c.strength = inst.strengthPa;
      c.sdkIndex = sdkBondFromNodes(graph, d.nodeIndex0, d.nodeIndex1);
      if (c.owner == 0) {
        if (d.nodeIndex0 < inst.nodeOwner.size() && inst.nodeOwner[d.nodeIndex0] != 0) {
          c.owner = inst.nodeOwner[d.nodeIndex0];
        } else if (d.nodeIndex1 < inst.nodeOwner.size()) {
          c.owner = inst.nodeOwner[d.nodeIndex1];
        }
      }
      if (c.sdkIndex < inst.probeCount && inst.probes[c.sdkIndex].blastBondIndex == c.sdkIndex) {
        const auto& p = inst.probes[c.sdkIndex];
        c.stress = probeMaxStress(p);
        c.healthBefore = p.health;
      }
      if (c.sdkIndex < metaN) {
        const BondMeta& m = inst.bondMeta[c.sdkIndex];
        c.stableId = m.stableId;
        c.inStrip = m.inStrip != 0;
        if (m.graphIndex < inst.graph.bonds.size()) {
          const GraphBond& gb = inst.graph.bonds[m.graphIndex];
          c.nodeA = gb.nodeA;
          c.nodeB = gb.nodeB;
          c.cy = gb.cy;
          c.faces = gb.faces;
        }
      }
      inst.pending.candidates.push_back(std::move(c));
    }
  }
  inst.debug.candidateInStrip = 0;
  for (const FractureCandidate& c : inst.pending.candidates) {
    if (c.inStrip) {
      ++inst.debug.candidateInStrip;
    }
  }
  inst.pending.valid = !inst.pending.candidates.empty();
  inst.pending.objectId = inst.objectId;
  inst.pending.topologyRevision = inst.topologyRevision;
  inst.pending.solverTopologyEpoch = inst.blast.solver->topologyEpoch();
  inst.pending.solveEpoch = inst.solveEpoch;
  inst.pending.strengthEpoch = inst.strengthEpoch;
  inst.pending.strengthPa = inst.strengthPa;
  inst.debug.candidateCount = static_cast<uint32_t>(inst.pending.candidates.size());
  inst.debug.candidateMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

uint32_t StructureWorld::applyPendingCandidates(const PendingFracture& pending) {
  StructureInstance* inst = instance();
  if (inst == nullptr || inst->blast.family == nullptr || inst->blast.solver == nullptr ||
      inst->blast.asset == nullptr || !pending.valid || pending.candidates.empty()) {
    return 0;
  }
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(inst->blast.asset, blastLog);
  fillNodeOwners(*inst, graph);
  const uint32_t nA = NvBlastFamilyGetActorCount(inst->blast.family, blastLog);
  uint32_t nfrac = 0;
  for (uint32_t ai = 0; ai < nA; ++ai) {
    NvBlastActor* actor = ai < inst->actorScratch.size() ? inst->actorScratch[ai] : nullptr;
    if (actor == nullptr) {
      continue;
    }
    inst->cmdScratch.clear();
    for (const FractureCandidate& c : pending.candidates) {
      if (c.owner != ai + 1) {
        continue;
      }
      if (!(c.damage > 0.0f)) {
        continue;
      }
      NvBlastBondFractureData d{};
      d.userdata = 0;
      d.nodeIndex0 = c.node0;
      d.nodeIndex1 = c.node1;
      d.health = c.damage;
      inst->cmdScratch.push_back(d);
    }
    if (inst->cmdScratch.empty()) {
      continue;
    }
    NvBlastFractureBuffers buf{};
    buf.bondFractureCount = static_cast<uint32_t>(inst->cmdScratch.size());
    buf.bondFractures = inst->cmdScratch.data();
    buf.chunkFractureCount = 0;
    buf.chunkFractures = nullptr;
    NvBlastActorApplyFracture(nullptr, actor, &buf, blastLog, nullptr);
    nfrac += static_cast<uint32_t>(inst->cmdScratch.size());
  }
  if (nfrac > 0) {
    bool anyBroken = false;
    for (const FractureCandidate& c : pending.candidates) {
      if (c.healthBefore > 0.0f && c.damage >= c.healthBefore) {
        anyBroken = true;
        break;
      }
    }
    if (anyBroken) {
      inst->blast.solver->syncBrokenBonds();
    }
  }
  return nfrac;
}

uint32_t StructureWorld::applyPendingIfAny() {
  StructureInstance* inst = instance();
  if (inst == nullptr) {
    return 0;
  }
  const bool impactOnly = inst->impactAppliedThisTick &&
                          (!inst->pending.valid || inst->pending.candidates.empty());
  if (impactOnly) {
    const uint32_t before =
        inst->blast.family != nullptr ? NvBlastFamilyGetActorCount(inst->blast.family, blastLog) : 0;
    const uint32_t actors = splitAllRequired();
    inst->debug.splitActors = actors;
    if (actors > before) {
      occupancyDirty_ = true;
    }
    inst->impactAppliedThisTick = false;
    return inst->debug.impactDamageEvents;
  }
  if (!inst->pending.valid || inst->pending.candidates.empty()) {
    return 0;
  }
  if (!pendingSnapshotMatches(inst->pending)) {
    recachePendingFromProbes();
    if (!pendingSnapshotMatches(inst->pending) || !inst->pending.valid || inst->pending.candidates.empty()) {
      clearPendingFracture();
      // Stale stress work must not cancel damage already applied by Impact.
      return inst->impactAppliedThisTick ? applyPendingIfAny() : 0;
    }
  }
  for (const FractureCandidate& c : inst->pending.candidates) {
    if (!(c.damage > 0.0f) || !(c.healthBefore > 0.0f)) {
      continue;
    }
    const float frac = std::min(1.0f, c.damage / c.healthBefore);
    if (c.stableId != 0) {
      const uint64_t key = packBondKey(c.nodeA, c.nodeB);
      const auto it = inst->grid.bondDamage.find(key);
      const float prev = it != inst->grid.bondDamage.end() ? it->second : 0.0f;
      inst->grid.bondDamage[key] = std::max(prev, frac);
    }
    if (c.damage >= c.healthBefore) {
      for (uint64_t f : c.faces) {
        inst->grid.brokenFaces.insert(f);
      }
    }
  }
  const uint32_t nfrac = applyPendingCandidates(inst->pending);
  inst->debug.fracturedBonds = nfrac;
  const uint32_t candKeep = inst->debug.candidateCount;
  clearPendingFracture();
  inst->debug.candidateCount = candKeep;
  if (nfrac == 0 && !inst->impactAppliedThisTick) {
    return 0;
  }
  const uint32_t before =
      inst->blast.family != nullptr ? NvBlastFamilyGetActorCount(inst->blast.family, blastLog) : 0;
  const uint32_t actors = splitAllRequired();
  inst->debug.splitActors = actors;
  if (actors > before) {
    occupancyDirty_ = true;
  }
  inst->impactAppliedThisTick = false;
  return nfrac;
}

bool StructureWorld::takeOccupancyDirty() {
  const bool dirty = occupancyDirty_;
  occupancyDirty_ = false;
  return dirty;
}

uint32_t StructureWorld::splitAllRequired() {
  StructureInstance* inst = instance();
  if (inst == nullptr || inst->blast.family == nullptr || inst->blast.solver == nullptr) {
    return 0;
  }
  inst->loadSnapshots.clear();
  inst->blast.actor = nullptr;
  const uint32_t actors = blast::splitAllRequired(inst->blast.family, *inst->blast.solver, blastLog);
  rebuildBindingsFromFamily(*inst);
  inst->lastBoundTopologyEpoch = inst->blast.solver->topologyEpoch();
  inst->blast.actor = nullptr;
  for (ActorBinding& b : inst->bindings) {
    if (b.actor != nullptr && b.anchored) {
      inst->blast.actor = b.actor;
      break;
    }
  }
  if (inst->blast.actor == nullptr && !inst->bindings.empty()) {
    inst->blast.actor = inst->bindings.front().actor;
  }
  inst->debug.splitActors = actors;
  return actors;
}

ActorBinding* StructureWorld::bindingForObject(StructureInstance& inst, VoxelObjectId id) {
  if (!id.valid()) {
    return nullptr;
  }
  for (ActorBinding& b : inst.bindings) {
    if (b.objectId == id && b.actor != nullptr) {
      return &b;
    }
  }
  return nullptr;
}

uint32_t StructureWorld::applyImpactShaderToActor(StructureInstance& inst, NvBlastActor* actor,
                                                  const glm::vec3& localPos, const glm::vec3& localForce) {
  if (actor == nullptr || inst.blast.asset == nullptr) {
    return 0;
  }
  const float mag = glm::length(localForce);
  const float normalized = viewerNormalizedDamage(mag, impactSettings_, impactMaterial_);
  if (normalized <= 0.0f) {
    return 0;
  }
  const float falloff = std::min(32.0f, std::max(1.0f, impactSettings_.damageFalloffRadiusFactor));
  const float minDistance = impactSettings_.damageRadiusMax * normalized;
  const float maxDistance = minDistance * falloff;
  const uint32_t bondCount = familyAssetBondCount(actor, blastLog);
  const uint32_t chunkCount = NvBlastAssetGetChunkCount(inst.blast.asset, blastLog);
  if (bondCount == 0 && chunkCount == 0) {
    return 0;
  }
  inst.cmdScratch.assign(bondCount, NvBlastBondFractureData{});
  inst.chunkScratch.assign(chunkCount, NvBlastChunkFractureData{});
  NvBlastFractureBuffers buf{};
  buf.bondFractureCount = bondCount;
  buf.chunkFractureCount = chunkCount;
  buf.bondFractures = inst.cmdScratch.empty() ? nullptr : inst.cmdScratch.data();
  buf.chunkFractures = inst.chunkScratch.empty() ? nullptr : inst.chunkScratch.data();
  NvBlastExtProgramParams programParams(nullptr, &impactMaterial_, inst.blast.accelerator);
  NvBlastDamageProgram program{};
  glm::vec3 n = localForce;
  const float n2 = glm::dot(n, n);
  if (n2 > 1.0e-12f) {
    n /= std::sqrt(n2);
  } else {
    n = glm::vec3(0.0f, 1.0f, 0.0f);
  }
  if (impactSettings_.shearDamage) {
    // ExtImpactDamageManager shearDamage=true branch.
    NvBlastExtShearDamageDesc desc{};
    desc.damage = normalized;
    desc.normal[0] = n.x;
    desc.normal[1] = n.y;
    desc.normal[2] = n.z;
    desc.position[0] = localPos.x;
    desc.position[1] = localPos.y;
    desc.position[2] = localPos.z;
    desc.minRadius = minDistance;
    desc.maxRadius = maxDistance;
    programParams.damageDesc = &desc;
    program.graphShaderFunction = NvBlastExtShearGraphShader;
    program.subgraphShaderFunction = NvBlastExtShearSubgraphShader;
    NvBlastActorGenerateFracture(&buf, actor, program, &programParams, blastLog, nullptr);
  } else if (inst.blast.accelerator != nullptr) {
    // ExtImpactDamageManager shearDamage=false branch (ImpactSpread + accelerator).
    NvBlastExtImpactSpreadDamageDesc desc{};
    desc.damage = normalized;
    desc.position[0] = localPos.x;
    desc.position[1] = localPos.y;
    desc.position[2] = localPos.z;
    desc.minRadius = minDistance;
    desc.maxRadius = maxDistance;
    programParams.damageDesc = &desc;
    program.graphShaderFunction = NvBlastExtImpactSpreadGraphShader;
    program.subgraphShaderFunction = NvBlastExtImpactSpreadSubgraphShader;
    NvBlastActorGenerateFracture(&buf, actor, program, &programParams, blastLog, nullptr);
  } else {
    // Viewer ImpactSpread requires its accelerator; do not silently change shader.
    return 0;
  }
  if (buf.bondFractureCount == 0 && buf.chunkFractureCount == 0) {
    return 0;
  }
  viewerDamageToBondArea(buf, inst.blast.asset);
  NvBlastActorApplyFracture(nullptr, actor, &buf, blastLog, nullptr);
  inst.blast.solver->syncBrokenBonds();
  syncBondDamageFromHealth(inst);
  return buf.bondFractureCount + buf.chunkFractureCount;
}

void StructureWorld::syncBondDamageFromHealth(StructureInstance& inst) {
  if (inst.blast.asset == nullptr || inst.blast.family == nullptr || inst.bindings.empty()) {
    return;
  }
  NvBlastActor* any = nullptr;
  for (const ActorBinding& b : inst.bindings) {
    if (b.actor != nullptr) {
      any = b.actor;
      break;
    }
  }
  if (any == nullptr) {
    return;
  }
  const float* healths = NvBlastActorGetBondHealths(any, blastLog);
  if (healths == nullptr) {
    return;
  }
  const NvBlastBond* assetBonds = NvBlastAssetGetBonds(inst.blast.asset, blastLog);
  const uint32_t nb = NvBlastAssetGetBondCount(inst.blast.asset, blastLog);
  float maxDmg = 0.0f;
  uint32_t damaged = 0;
  const uint32_t metaN = static_cast<uint32_t>(inst.bondMeta.size());
  for (uint32_t sdk = 0; sdk < nb && sdk < metaN; ++sdk) {
    const BondMeta& m = inst.bondMeta[sdk];
    if (m.world != 0 || m.stableId == 0 || m.graphIndex >= inst.graph.bonds.size()) {
      continue;
    }
    const float initial = assetBonds != nullptr ? assetBonds[sdk].area : 0.0f;
    if (!(initial > 1.0e-8f) || !canTakeDamage(initial)) {
      continue;
    }
    const float h = healths[sdk];
    float frac = 1.0f - std::clamp(h / initial, 0.0f, 1.0f);
    if (h <= 0.0f) {
      frac = 1.0f;
    }
    const GraphBond& gb = inst.graph.bonds[m.graphIndex];
    const uint64_t key = packBondKey(gb.nodeA, gb.nodeB);
    if (frac > 0.0f) {
      const auto it = inst.grid.bondDamage.find(key);
      const float prev = it != inst.grid.bondDamage.end() ? it->second : 0.0f;
      inst.grid.bondDamage[key] = std::max(prev, frac);
    }
  }
  maxDmg = 0.0f;
  damaged = 0;
  for (const auto& kv : inst.grid.bondDamage) {
    if (kv.second > 0.0f) {
      maxDmg = std::max(maxDmg, kv.second);
      ++damaged;
    }
  }
  inst.debug.maxBondDamage = maxDmg;
  inst.debug.damagedBondCount = damaged;
}

void StructureWorld::applyViewerImpact(StructureInstance& inst, const WorldContactImpulse* impulses,
                                       uint32_t nImpulses) {
  if (!impactDamageEnabled_ || impulses == nullptr || nImpulses == 0) {
    return;
  }
  struct PairAccum {
    VoxelObjectId idA{};
    VoxelObjectId idB{};
    glm::vec3 forceA{0.0f};
    glm::vec3 forceB{0.0f};
    glm::vec3 pos{0.0f};
    glm::quat qA{1.0f, 0.0f, 0.0f, 0.0f};
    glm::quat qB{1.0f, 0.0f, 0.0f, 0.0f};
    glm::vec3 xA{0.0f};
    glm::vec3 xB{0.0f};
    uint32_t n = 0;
  };
  // Viewer averages within one PxShape pair, not across all shapes of an actor
  // or across simulation steps. A voxel support node is our collision subshape.
  using PairKey = std::tuple<uint32_t, uint32_t, uint32_t, uint32_t, int, uint32_t, uint32_t>;
  std::map<PairKey, PairAccum> pairs;
  for (uint32_t i = 0; i < nImpulses; ++i) {
    WorldContactImpulse imp = impulses[i];
    if (std::tie(imp.idB.slot, imp.idB.generation) < std::tie(imp.idA.slot, imp.idA.generation)) {
      std::swap(imp.idA, imp.idB);
      std::swap(imp.massA, imp.massB);
      std::swap(imp.velA, imp.velB);
      std::swap(imp.xA, imp.xB);
      std::swap(imp.qA, imp.qB);
      std::swap(imp.fineA, imp.fineB);
      std::swap(imp.fineNA, imp.fineNB);
      imp.n = -imp.n;
      imp.JA = -imp.JA;
    }
    ActorBinding* bA = bindingForObject(inst, imp.idA);
    ActorBinding* bB = bindingForObject(inst, imp.idB);
    // ExtPxActor marks terminal actors LEAF_CHUNK, and FilterShader suppresses
    // their contact notifications on either side (physics collision still runs).
    auto leaf = [](const ActorBinding* b) {
      return b && b->actor && !NvBlastActorCanFracture(b->actor, blastLog);
    };
    if (leaf(bA) || leaf(bB)) continue;
    glm::vec3 fA{0.0f};
    glm::vec3 fB{0.0f};
    if (!viewerPairForce(imp, fA, fB)) {
      continue;
    }
    auto shape = [&](ActorBinding* b, bool sideA) {
      NodeRef node;
      return b && pickContactNode(inst, *b, imp, sideA, node) ? node.graphNode : UINT32_MAX;
    };
    const PairKey key{imp.idA.slot, imp.idA.generation, imp.idB.slot, imp.idB.generation,
                      imp.substep, shape(bA, true), shape(bB, false)};
    PairAccum& acc = pairs[key];
    if (acc.n == 0) {
      acc.idA = imp.idA;
      acc.idB = imp.idB;
      acc.qA = imp.qA;
      acc.qB = imp.qB;
      acc.xA = imp.xA;
      acc.xB = imp.xB;
    }
    acc.forceA += fA;
    acc.forceB += fB;
    acc.pos += imp.worldPoint;
    ++acc.n;
  }
  auto applySide = [&](ActorBinding* b, const glm::vec3& worldPos, const glm::vec3& worldForce, const glm::quat& q,
                       const glm::vec3& x) {
    if (b == nullptr || b->actor == nullptr || glm::dot(worldForce, worldForce) <= 0.0f) {
      return;
    }
    BodyAssetFrame frame;
    frame.objectId = b->objectId;
    frame.assetCom = b->comAsset;
    frame.worldCom = x;
    frame.worldQ = q;
    const glm::vec3 localPos = worldToAsset(worldPos, frame);
    const glm::vec3 localForce = worldVecToAsset(worldForce, frame);
    if (stressImpactImpulses_ && b->stressSolve) {
      const glm::vec3 f = localForce * stressImpactScale_;
      inst.blast.solver->addForce(*b->actor, NvcVec3{localPos.x, localPos.y, localPos.z},
                                  NvcVec3{f.x, f.y, f.z}, Nv::Blast::ExtForceMode::FORCE);
      ++inst.debug.impactDamageEvents;
      return;
    }
    const uint32_t n = applyImpactShaderToActor(inst, b->actor, localPos, localForce);
    if (n > 0) {
      inst.impactAppliedThisTick = true;
      inst.debug.impactDamageEvents += n;
    }
  };
  uint32_t noBind = 0;
  uint32_t selfSkip = 0;
  for (auto& kv : pairs) {
    PairAccum& acc = kv.second;
    if (acc.n == 0) {
      continue;
    }
    const float invN = 1.0f / static_cast<float>(acc.n);
    acc.forceA *= invN;
    acc.forceB *= invN;
    acc.pos *= invN;
    ActorBinding* bA = bindingForObject(inst, acc.idA);
    ActorBinding* bB = bindingForObject(inst, acc.idB);
    if (bA != nullptr && bB != nullptr && !impactSettings_.selfCollisionEnabled) {
      ++selfSkip;
      continue;
    }
    if (bA == nullptr && bB == nullptr) {
      ++noBind;
    }
    applySide(bA, acc.pos, acc.forceA, acc.qA, acc.xA);
    applySide(bB, acc.pos, acc.forceB, acc.qB, acc.xB);
  }
  if (nImpulses > 0) {
    std::cout << "ImpactDamage impulses=" << nImpulses << " pairs=" << pairs.size()
              << " events=" << inst.debug.impactDamageEvents << " noBind=" << noBind
              << " selfSkip=" << selfSkip << " bindings=" << inst.bindings.size() << "\n";
  }
}

const BodyKinematics* StructureWorld::findKinematics(VoxelObjectId id, const BodyKinematics* kinematics,
                                                     uint32_t nKinematics) const {
  if (kinematics == nullptr || !id.valid()) {
    return nullptr;
  }
  for (uint32_t i = 0; i < nKinematics; ++i) {
    if (kinematics[i].objectId == id) {
      return &kinematics[i];
    }
  }
  return nullptr;
}

void StructureWorld::solveInstance(StructureInstance& inst, const WorldContactImpulse* impulses,
                                   uint32_t nImpulses, const BodyKinematics* kinematics, uint32_t nKinematics) {
  if (inst.blast.solver == nullptr || inst.blast.family == nullptr) {
    return;
  }
  inst.debug.probeExportCount = 0;
  inst.debug.gravityActors = 0;
  inst.debug.centrifugalActors = 0;
  inst.debug.impactDamageEvents = 0;
  inst.debug.impactDamageEnabled = impactDamageEnabled_;
  inst.debug.stressImpactImpulses = stressImpactImpulses_;
  inst.debug.stressImpactScale = stressImpactScale_;
  inst.impactAppliedThisTick = false;
  const auto tLoad0 = std::chrono::steady_clock::now();
  const uint32_t topo = inst.blast.solver->topologyEpoch();
  if (inst.bindings.empty() || inst.lastBoundTopologyEpoch != topo) {
    rebuildBindingsFromFamily(inst);
    inst.lastBoundTopologyEpoch = topo;
  }
  buildLoadSnapshots(inst, impulses, nImpulses, lastTickId_, lastDt_ > 0.0f ? lastDt_ : (1.0f / 60.0f));
  for (ActorLoadSnapshot& snap : inst.loadSnapshots) {
    snap.evaluated = true;
  }
  const glm::vec3 worldG(0.0f, kE1GravityY, 0.0f);
  for (ActorBinding& b : inst.bindings) {
    if (b.actor == nullptr || !b.stressSolve) {
      continue;
    }
    const BodyKinematics* kin = findKinematics(b.objectId, kinematics, nKinematics);
    const glm::quat q = kin != nullptr ? kin->worldQ : glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    if (b.anchored) {
      const glm::vec3 localG = glm::inverse(q) * worldG;
      inst.blast.solver->addGravity(*b.actor, NvcVec3{localG.x, localG.y, localG.z});
      ++inst.debug.gravityActors;
    } else {
      const glm::vec3 localW = kin != nullptr ? (glm::inverse(q) * kin->worldW) : glm::vec3(0.0f);
      inst.blast.solver->addCentrifugalAcceleration(
          *b.actor, NvcVec3{b.comAsset.x, b.comAsset.y, b.comAsset.z},
          NvcVec3{localW.x, localW.y, localW.z});
      ++inst.debug.centrifugalActors;
    }
  }
  applyViewerImpact(inst, impulses, nImpulses);
  // Keep bond-damage paint/HUD in sync even on ticks with no new Impact events.
  syncBondDamageFromHealth(inst);
  inst.debug.contactLoadMs =
      std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - tLoad0).count();
  const auto t0 = std::chrono::steady_clock::now();
  inst.blast.solver->update();
  ++inst.solveEpoch;
  const auto t1 = std::chrono::steady_clock::now();
  refreshDebug(inst, std::chrono::duration<float, std::milli>(t1 - t0).count());
  inst.debug.bindingCount = static_cast<uint32_t>(inst.bindings.size());
  inst.debug.impactDamageEnabled = impactDamageEnabled_;
  inst.debug.stressImpactImpulses = stressImpactImpulses_;
  inst.debug.stressImpactScale = stressImpactScale_;
  collectPendingFracture(inst);
}

void StructureWorld::onPhysicsTick(uint64_t tickId, float dt) { onPhysicsTick(tickId, dt, nullptr, 0, nullptr, 0); }

void StructureWorld::onPhysicsTick(uint64_t tickId, float dt, const WorldContactImpulse* impulses,
                                   uint32_t nImpulses) {
  onPhysicsTick(tickId, dt, impulses, nImpulses, nullptr, 0);
}

void StructureWorld::onPhysicsTick(uint64_t tickId, float dt, const WorldContactImpulse* impulses,
                                   uint32_t nImpulses, const BodyKinematics* kinematics, uint32_t nKinematics) {
  if (!initialized_) {
    return;
  }
  ++ticksReceived_;
  lastTickId_ = tickId;
  lastDt_ = dt;
  if (instances_.empty()) {
    return;
  }
  for (StructureInstance& inst : instances_) {
    solveInstance(inst, impulses, nImpulses, kinematics, nKinematics);
  }
}

std::size_t StructureWorld::blastLiveBytes() const {
  return runtime_ != nullptr ? runtime_->liveBytes() : 0;
}

std::size_t StructureWorld::blastRuntimeBaselineBytes() const {
  return runtime_ != nullptr ? runtime_->runtimeBaselineBytes() : 0;
}

int StructureWorld::blastErrorCount() const { return runtime_ != nullptr ? runtime_->errorCount() : 0; }

const StructureDebugSnapshot& StructureWorld::debug() const {
  if (instances_.empty()) {
    return idle_;
  }
  return instances_.front().debug;
}

void StructureWorld::fillBindingKinematics(StructureInstance& inst, ActorBinding& b) {
  b.nodeRefs.clear();
  if (b.actor == nullptr) {
    b.graphNodeCount = 0;
    b.anchored = false;
    b.stressSolve = false;
    b.comAsset = glm::vec3(0.0f);
    return;
  }
  b.graphNodeCount = NvBlastActorGetGraphNodeCount(b.actor, blastLog);
  b.anchored = NvBlastActorHasExternalBonds(b.actor, blastLog);
  b.stressSolve = b.graphNodeCount > 1;
  b.nodeRefs = actorNodeRefs(inst, b);
  float mass = 0.0f;
  glm::vec3 com(0.0f);
  std::vector<uint8_t> mine;
  if (inst.blast.asset != nullptr) {
    const NvBlastSupportGraph g = NvBlastAssetGetSupportGraph(inst.blast.asset, blastLog);
    mine.assign(g.nodeCount, 0);
    for (const NodeRef& r : b.nodeRefs) {
      if (r.graphNode < g.nodeCount) {
        mine[r.graphNode] = 1;
      }
    }
  }
  for (const GraphNode& n : inst.graph.nodes) {
    const auto it = inst.blast.graphFromStable.find(n.stableId);
    if (it == inst.blast.graphFromStable.end() || it->second >= mine.size() || mine[it->second] == 0) {
      continue;
    }
    mass += n.mass;
    com += n.mass * glm::vec3(n.cx, n.cy, n.cz);
  }
  b.comAsset = mass > 1e-8f ? com / mass : glm::vec3(0.0f);
}

std::vector<NodeRef> StructureWorld::actorNodeRefs(const StructureInstance& inst, const ActorBinding& b) const {
  std::vector<NodeRef> refs;
  if (b.actor == nullptr || inst.blast.asset == nullptr) {
    return refs;
  }
  const uint32_t nn = NvBlastActorGetGraphNodeCount(b.actor, blastLog);
  if (nn == 0) {
    return refs;
  }
  std::vector<uint32_t> idx(nn);
  NvBlastActorGetGraphNodeIndices(idx.data(), nn, b.actor, blastLog);
  const NvBlastSupportGraph g = NvBlastAssetGetSupportGraph(inst.blast.asset, blastLog);
  const NvBlastChunk* chunks = NvBlastAssetGetChunks(inst.blast.asset, blastLog);
  std::unordered_map<uint32_t, const GraphNode*> byStable;
  byStable.reserve(inst.graph.nodes.size() * 2 + 1);
  for (const GraphNode& n : inst.graph.nodes) {
    byStable.emplace(n.stableId, &n);
  }
  refs.reserve(nn);
  for (uint32_t i = 0; i < nn; ++i) {
    const uint32_t gn = idx[i];
    if (gn >= g.nodeCount || g.chunkIndices[gn] == UINT32_MAX) {
      continue;
    }
    const uint32_t stable = chunks[g.chunkIndices[gn]].userData;
    const auto it = byStable.find(stable);
    if (it == byStable.end()) {
      continue;
    }
    NodeRef r;
    r.graphNode = gn;
    r.center = glm::vec3(it->second->cx, it->second->cy, it->second->cz);
    refs.push_back(r);
  }
  return refs;
}

void StructureWorld::rebuildBindingsFromFamily(StructureInstance& inst) {
  std::unordered_map<NvBlastActor*, ActorBinding> prev;
  prev.reserve(inst.bindings.size());
  for (ActorBinding& b : inst.bindings) {
    if (b.actor != nullptr) {
      prev[b.actor] = b;
    }
  }
  inst.bindings.clear();
  if (inst.blast.family == nullptr) {
    return;
  }
  const uint32_t nA = NvBlastFamilyGetActorCount(inst.blast.family, blastLog);
  if (inst.actorScratch.size() < nA) {
    inst.actorScratch.resize(nA);
  }
  NvBlastFamilyGetActors(inst.actorScratch.data(), nA, inst.blast.family, blastLog);
  inst.bindings.reserve(nA);
  for (uint32_t i = 0; i < nA; ++i) {
    NvBlastActor* a = inst.actorScratch[i];
    if (a == nullptr) {
      continue;
    }
    if (NvBlastActorGetVisibleChunkCount(a, blastLog) == 0) {
      continue;
    }
    ActorBinding b;
    b.actor = a;
    b.objectId = inst.objectId;
    const auto it = prev.find(a);
    if (it != prev.end()) {
      b.objectId = it->second.objectId;
      b.fineOrigin = it->second.fineOrigin;
      b.fineN = it->second.fineN;
    }
    fillBindingKinematics(inst, b);
    inst.bindings.push_back(b);
  }
}

void StructureWorld::bindVisibleActors(const std::vector<ActorObjectLink>& links) {
  StructureInstance* inst = instance();
  if (inst == nullptr) {
    return;
  }
  rebuildBindingsFromFamily(*inst);
  if (inst->blast.solver != nullptr) {
    inst->lastBoundTopologyEpoch = inst->blast.solver->topologyEpoch();
  }
  for (ActorBinding& b : inst->bindings) {
    const ActorObjectLink* found = nullptr;
    for (const ActorObjectLink& L : links) {
      if (L.actor != nullptr && L.actor == b.actor) {
        found = &L;
        break;
      }
    }
    if (found == nullptr) {
      continue;
    }
    b.objectId = found->objectId;
    b.fineOrigin = found->fineOrigin;
    b.fineN = found->fineN;
    fillBindingKinematics(*inst, b);
  }
  inst->debug.bindingCount = static_cast<uint32_t>(inst->bindings.size());
  inst->loadSnapshots.clear();
}

bool StructureWorld::pickContactNode(const StructureInstance& inst, const ActorBinding& b,
                                     const WorldContactImpulse& imp, bool sideA, NodeRef& out) const {
  const std::vector<NodeRef>& nodes = b.nodeRefs;
  if (nodes.empty()) {
    return false;
  }
  const uint32_t fine = sideA ? imp.fineA : imp.fineB;
  const int nFine = sideA ? imp.fineNA : imp.fineNB;
  if (b.fineN > 0 && nFine > 0 && fine != 0xFFFFFFFFu) {
    const glm::ivec3 local = unpackFineCoord(fine, nFine);
    const VoxelCoord abs{local.x + b.fineOrigin.x, local.y + b.fineOrigin.y, local.z + b.fineOrigin.z};
    const auto it = inst.graph.voxelNode.find(abs);
    if (it != inst.graph.voxelNode.end()) {
      const auto gIt = inst.blast.graphFromStable.find(it->second);
      if (gIt != inst.blast.graphFromStable.end()) {
        for (const NodeRef& n : nodes) {
          if (n.graphNode == gIt->second) {
            out = n;
            return true;
          }
        }
      }
    }
  }
  BodyAssetFrame frame;
  frame.objectId = b.objectId;
  frame.assetCom = b.comAsset;
  frame.worldCom = sideA ? imp.xA : imp.xB;
  frame.worldQ = sideA ? imp.qA : imp.qB;
  PickedImpulse proj;
  if (!projectImpulseToActor(imp, b.objectId, frame, proj)) {
    return false;
  }
  return pickNearestNode(proj.pAsset, nodes, out);
}

void StructureWorld::buildLoadSnapshots(StructureInstance& inst, const WorldContactImpulse* impulses,
                                        uint32_t nImpulses, uint64_t tickId, float dt) {
  inst.loadSnapshots.clear();
  inst.debug.contactContrib = 0;
  inst.debug.mappedNodes = 0;
  inst.debug.invalidSnapshots = 0;
  if (impulses == nullptr || nImpulses == 0) {
    return;
  }
  ++inst.loadEpoch;
  for (ActorBinding& b : inst.bindings) {
    if (b.actor == nullptr || !b.stressSolve || !b.objectId.valid()) {
      continue;
    }
    std::vector<PickedImpulse> picked;
    picked.reserve(nImpulses);
    std::unordered_set<uint64_t> impactAccepted;
    std::unordered_set<uint64_t> impactRejected;
    for (uint32_t i = 0; i < nImpulses; ++i) {
      const WorldContactImpulse& imp = impulses[i];
      const bool sideA = imp.idA == b.objectId;
      const bool sideB = imp.idB == b.objectId;
      if (!sideA && !sideB) {
        continue;
      }
      const bool takeContact = !b.anchored || stressImpactImpulses_;
      if (!takeContact) {
        continue;
      }
      if (!imp.persistent) {
        const uint64_t eid =
            imp.eventId != 0 ? imp.eventId : (contactPairKey(imp.idA, imp.idB) ^ tickId);
        if (impactRejected.count(eid) != 0) {
          continue;
        }
        if (impactAccepted.count(eid) == 0) {
          if (!inst.impactEvents.consume(eid)) {
            impactRejected.insert(eid);
            continue;
          }
          impactAccepted.insert(eid);
        }
      }
      BodyAssetFrame frame;
      frame.objectId = b.objectId;
      frame.assetCom = b.comAsset;
      frame.worldCom = sideA ? imp.xA : imp.xB;
      frame.worldQ = sideA ? imp.qA : imp.qB;
      PickedImpulse p;
      if (!projectImpulseToActor(imp, b.objectId, frame, p)) {
        continue;
      }
      if (!pickContactNode(inst, b, imp, sideA, p.node)) {
        p.ok = false;
        picked.push_back(p);
        continue;
      }
      p.ok = true;
      picked.push_back(p);
    }
    if (picked.empty()) {
      continue;
    }
    const TickLoadResult folded = foldPickedImpulses(picked.data(), static_cast<uint32_t>(picked.size()), dt,
                                                     glm::vec3(0.0f));
    ActorLoadSnapshot snap;
    snap.tickId = tickId;
    snap.objectId = b.objectId;
    snap.topologyEpoch = inst.blast.solver != nullptr ? inst.blast.solver->topologyEpoch() : 0;
    snap.loadEpoch = inst.loadEpoch;
    snap.loads = folded.loads;
    snap.valid = folded.unmapped == 0 || !folded.loads.empty();
    snap.invalidReason = snap.valid ? "ok" : "unmapped contact";
    snap.contactContrib = folded.contactContrib;
    snap.mappedNodes = folded.mappedNodes;
    if (!snap.valid) {
      ++inst.debug.invalidSnapshots;
    }
    inst.debug.contactContrib += folded.contactContrib;
    inst.debug.mappedNodes += folded.mappedNodes;
    inst.loadSnapshots.push_back(std::move(snap));
  }
}

StructureInstance* StructureWorld::instance() { return instances_.empty() ? nullptr : &instances_.front(); }

const StructureInstance* StructureWorld::instance() const {
  return instances_.empty() ? nullptr : &instances_.front();
}

}  // namespace blast
