#pragma once

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastTypes.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace blast {

inline float probeMaxStress(const Nv::Blast::ExtStressSolver::BondProbe& p) {
  return std::max(p.compression, std::max(p.tension, p.shear));
}

// NVIDIA default: tension/shear inherit from compression when negative (see inheritSettingsLimits).
inline void applyExtStressStrength(Nv::Blast::ExtStressSolver& solver, float strengthPa) {
  auto st = solver.getSettings();
  st.graphReductionLevel = 0;
  st.compressionElasticLimit = strengthPa;
  st.compressionFatalLimit = 2.0f * strengthPa;
  st.tensionElasticLimit = -1.0f;
  st.tensionFatalLimit = -1.0f;
  st.shearElasticLimit = -1.0f;
  st.shearFatalLimit = -1.0f;
  solver.setSettings(st);
}

struct OverstressHit {
  uint32_t blastBondIndex = 0;
  uint32_t node0 = 0;
  uint32_t node1 = 0;
  float stress = 0.0f;
  float health = 0.0f;
};

inline uint32_t familyAssetBondCount(NvBlastActor* actor, NvBlastLog logFn) {
  if (actor == nullptr) {
    return 0;
  }
  NvBlastFamily* family = NvBlastActorGetFamily(actor, logFn);
  if (family == nullptr) {
    return 0;
  }
  const NvBlastAsset* asset = NvBlastFamilyGetAsset(family, logFn);
  if (asset == nullptr) {
    return 0;
  }
  return NvBlastAssetGetBondCount(asset, logFn);
}

inline bool isWorldGraphNode(const NvBlastSupportGraph& graph, uint32_t node) {
  return node >= graph.nodeCount || graph.chunkIndices[node] == UINT32_MAX;
}

// Family-level probes belong to `actor` when both support endpoints are this actor's
// graph nodes, or one is and the other is the world node. Bonds of a sibling actor
// are not submitted to NvBlastActorApplyFracture.
inline bool probeOwnedByActor(const Nv::Blast::ExtStressSolver::BondProbe& p, const NvBlastSupportGraph& graph,
                              const std::vector<uint8_t>& inActor) {
  const bool a0 = p.node0 < graph.nodeCount && inActor[p.node0] != 0;
  const bool a1 = p.node1 < graph.nodeCount && inActor[p.node1] != 0;
  if (a0 && a1) {
    return true;
  }
  if (a0 && isWorldGraphNode(graph, p.node1)) {
    return true;
  }
  if (a1 && isWorldGraphNode(graph, p.node0)) {
    return true;
  }
  return false;
}

inline std::vector<uint8_t> actorGraphNodeMask(NvBlastActor* actor, const NvBlastSupportGraph& graph, NvBlastLog logFn) {
  std::vector<uint8_t> inActor(graph.nodeCount, 0);
  if (actor == nullptr || graph.nodeCount == 0) {
    return inActor;
  }
  const uint32_t n = NvBlastActorGetGraphNodeCount(actor, logFn);
  std::vector<uint32_t> nodes(n, 0);
  NvBlastActorGetGraphNodeIndices(nodes.data(), n, actor, logFn);
  for (uint32_t i = 0; i < n; ++i) {
    if (nodes[i] < graph.nodeCount) {
      inActor[nodes[i]] = 1;
    }
  }
  return inActor;
}

inline uint32_t owningActorIndex(const Nv::Blast::ExtStressSolver::BondProbe& p, const NvBlastSupportGraph& graph,
                                 const uint32_t* nodeOwner) {
  const uint32_t o0 = p.node0 < graph.nodeCount ? nodeOwner[p.node0] : 0;
  const uint32_t o1 = p.node1 < graph.nodeCount ? nodeOwner[p.node1] : 0;
  if (o0 != 0 && o1 != 0 && o0 == o1) {
    return o0;
  }
  if (o0 != 0 && isWorldGraphNode(graph, p.node1)) {
    return o0;
  }
  if (o1 != 0 && isWorldGraphNode(graph, p.node0)) {
    return o1;
  }
  return 0;
}

inline uint32_t collectOverstressedFromProbes(NvBlastActor* actor,
                                              const Nv::Blast::ExtStressSolver::BondProbe* probes, uint32_t n,
                                              float strengthPa, NvBlastLog logFn, std::vector<OverstressHit>& out,
                                              bool requireConverged = true,
                                              const Nv::Blast::ExtStressSolver* solver = nullptr) {
  out.clear();
  if (actor == nullptr || probes == nullptr) {
    return 0;
  }
  if (requireConverged && solver != nullptr && !solver->converged()) {
    return 0;
  }
  NvBlastFamily* family = NvBlastActorGetFamily(actor, logFn);
  if (family == nullptr) {
    return 0;
  }
  const NvBlastAsset* asset = NvBlastFamilyGetAsset(family, logFn);
  if (asset == nullptr) {
    return 0;
  }
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, logFn);
  const std::vector<uint8_t> inActor = actorGraphNodeMask(actor, graph, logFn);
  for (uint32_t i = 0; i < n; ++i) {
    const auto& p = probes[i];
    if (!probeOwnedByActor(p, graph, inActor) || !canTakeDamage(p.health)) {
      continue;
    }
    const float s = probeMaxStress(p);
    if (!(s > strengthPa)) {
      continue;
    }
    OverstressHit h;
    h.blastBondIndex = p.blastBondIndex;
    h.node0 = p.node0;
    h.node1 = p.node1;
    h.stress = s;
    h.health = p.health;
    out.push_back(h);
  }
  return static_cast<uint32_t>(out.size());
}

inline uint32_t collectOverstressed(NvBlastActor* actor, Nv::Blast::ExtStressSolver& solver, float strengthPa,
                                    NvBlastLog logFn, std::vector<OverstressHit>& out,
                                    bool requireConverged = true) {
  out.clear();
  if (actor == nullptr || (requireConverged && !solver.converged())) {
    return 0;
  }
  const uint32_t bondCount = familyAssetBondCount(actor, logFn);
  if (bondCount == 0) {
    return 0;
  }
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> probes(bondCount);
  const uint32_t n = solver.copyBondProbes(probes.data(), bondCount);
  return collectOverstressedFromProbes(actor, probes.data(), n, strengthPa, logFn, out, requireConverged, &solver);
}

// Hard threshold: stress > S deducts current health; stress <= S does nothing.
// No-ops when the solver did not converge. Tests still export here; the engine
// commit path applies the pending candidate snapshot instead.
inline uint32_t applyHardThreshold(NvBlastActor* actor, Nv::Blast::ExtStressSolver& solver, float strengthPa,
                                   NvBlastLog logFn, bool requireConverged = true) {
  if (actor == nullptr || (requireConverged && !solver.converged())) {
    return 0;
  }
  NvBlastFamily* family = NvBlastActorGetFamily(actor, logFn);
  if (family == nullptr) {
    return 0;
  }
  const NvBlastAsset* asset = NvBlastFamilyGetAsset(family, logFn);
  if (asset == nullptr) {
    return 0;
  }
  const uint32_t bondCount = NvBlastAssetGetBondCount(asset, logFn);
  if (bondCount == 0) {
    return 0;
  }
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> probes(bondCount);
  const uint32_t n = solver.copyBondProbes(probes.data(), bondCount);
  if (n < bondCount) {
    probes.resize(n);
  }

  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, logFn);
  const std::vector<uint8_t> inActor = actorGraphNodeMask(actor, graph, logFn);

  std::vector<NvBlastBondFractureData> cmds;
  cmds.reserve(n);
  const float* healths = NvBlastActorGetBondHealths(actor, logFn);
  for (uint32_t i = 0; i < n; ++i) {
    const auto& p = probes[i];
    if (!probeOwnedByActor(p, graph, inActor)) {
      continue;
    }
    if (!canTakeDamage(p.health)) {
      continue;
    }
    if (!(probeMaxStress(p) > strengthPa)) {
      continue;
    }
    NvBlastBondFractureData d{};
    d.userdata = 0;
    d.nodeIndex0 = p.node0;
    d.nodeIndex1 = p.node1;
    d.health = healths != nullptr ? healths[p.blastBondIndex] : p.health;
    cmds.push_back(d);
  }
  if (cmds.empty()) {
    return 0;
  }
  NvBlastFractureBuffers buf{};
  buf.bondFractureCount = static_cast<uint32_t>(cmds.size());
  buf.bondFractures = cmds.data();
  buf.chunkFractureCount = 0;
  buf.chunkFractures = nullptr;
  NvBlastActorApplyFracture(nullptr, actor, &buf, logFn, nullptr);
  solver.syncBrokenBonds();
  return static_cast<uint32_t>(cmds.size());
}

inline void applySdkBondDamage(NvBlastActor* actor, const NvBlastAsset* asset, const uint32_t* sdkBonds,
                               uint32_t bondCount, NvBlastLog logFn) {
  if (actor == nullptr || asset == nullptr || sdkBonds == nullptr || bondCount == 0) {
    return;
  }
  const float* healths = NvBlastActorGetBondHealths(actor, logFn);
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, logFn);
  std::vector<NvBlastBondFractureData> cmds;
  cmds.reserve(bondCount);
  for (uint32_t i = 0; i < bondCount; ++i) {
    const uint32_t idx = sdkBonds[i];
    if (healths == nullptr || healths[idx] <= 0.0f || !canTakeDamage(healths[idx])) {
      continue;
    }
    NvBlastBondFractureData d{};
    d.health = healths[idx];
    bool found = false;
    for (uint32_t n0 = 0; n0 < graph.nodeCount && !found; ++n0) {
      for (uint32_t adj = graph.adjacencyPartition[n0]; adj < graph.adjacencyPartition[n0 + 1]; ++adj) {
        if (graph.adjacentBondIndices[adj] == idx) {
          d.nodeIndex0 = n0;
          d.nodeIndex1 = graph.adjacentNodeIndices[adj];
          found = true;
          break;
        }
      }
    }
    if (found) {
      cmds.push_back(d);
    }
  }
  if (cmds.empty()) {
    return;
  }
  NvBlastFractureBuffers buf{};
  buf.bondFractureCount = static_cast<uint32_t>(cmds.size());
  buf.bondFractures = cmds.data();
  NvBlastActorApplyFracture(nullptr, actor, &buf, logFn, nullptr);
}

inline uint32_t splitAllRequired(NvBlastFamily* family, Nv::Blast::ExtStressSolver& solver, NvBlastLog logFn) {
  if (family == nullptr) {
    return 0;
  }
  const uint32_t nA = NvBlastFamilyGetActorCount(family, logFn);
  std::vector<NvBlastActor*> list(nA, nullptr);
  NvBlastFamilyGetActors(list.data(), nA, family, logFn);
  for (uint32_t i = 0; i < nA; ++i) {
    NvBlastActor* actor = list[i];
    if (actor == nullptr || !NvBlastActorIsSplitRequired(actor, logFn)) {
      continue;
    }
    solver.notifyActorDestroyed(*actor);
    const uint32_t maxNew = NvBlastActorGetMaxActorCountForSplit(actor, logFn);
    std::vector<NvBlastActor*> created(maxNew, nullptr);
    NvBlastActorSplitEvent ev{};
    ev.newActors = created.data();
    std::vector<char> scratch(NvBlastActorGetRequiredScratchForSplit(actor, logFn));
    const uint32_t nNew = NvBlastActorSplit(&ev, actor, maxNew, scratch.data(), logFn, nullptr);
    for (uint32_t k = 0; k < nNew && k < maxNew; ++k) {
      if (created[k] != nullptr) {
        solver.notifyActorCreated(*created[k]);
      }
    }
  }
  return NvBlastFamilyGetActorCount(family, logFn);
}

inline uint32_t splitIfNeeded(NvBlastActor*& actor, NvBlastFamily* family, Nv::Blast::ExtStressSolver& solver,
                              NvBlastLog logFn) {
  if (actor == nullptr || !NvBlastActorIsSplitRequired(actor, logFn)) {
    return NvBlastFamilyGetActorCount(family, logFn);
  }
  solver.notifyActorDestroyed(*actor);
  const uint32_t maxNew = NvBlastActorGetMaxActorCountForSplit(actor, logFn);
  std::vector<NvBlastActor*> created(maxNew, nullptr);
  NvBlastActorSplitEvent ev{};
  ev.newActors = created.data();
  std::vector<char> scratch(NvBlastActorGetRequiredScratchForSplit(actor, logFn));
  NvBlastActorSplit(&ev, actor, maxNew, scratch.data(), logFn, nullptr);
  const uint32_t active = NvBlastFamilyGetActorCount(family, logFn);
  std::vector<NvBlastActor*> actors(active, nullptr);
  NvBlastFamilyGetActors(actors.data(), active, family, logFn);
  actor = nullptr;
  for (uint32_t i = 0; i < active; ++i) {
    if (actors[i] == nullptr) {
      continue;
    }
    solver.notifyActorCreated(*actors[i]);
    if (NvBlastActorHasExternalBonds(actors[i], logFn) && actor == nullptr) {
      actor = actors[i];
    }
  }
  if (actor == nullptr && active > 0) {
    actor = actors[0];
  }
  return active;
}

}  // namespace blast
