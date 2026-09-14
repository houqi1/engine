#include "blast/BlastMemory.h"
#include "blast/HardFracture.h"
#include "blast/VoxelGraph.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastGlobals.h"
#include "NvCTypes.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {

int gFailures = 0;

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAIL: " << msg << "\n";
    ++gFailures;
  }
}

void blastLog(int type, const char* msg, const char* file, int line) {
  if (type <= NvBlastMessage::Error) {
    std::cerr << "NvBlastLL " << file << ":" << line << ": " << (msg ? msg : "") << "\n";
    ++gFailures;
  }
}

void fillBox(blast::VoxelGrid& g, int x0, int y0, int z0, int x1, int y1, int z1) {
  for (int z = z0; z < z1; ++z) {
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        g.setSolid(x, y, z, true);
      }
    }
  }
}

const blast::GraphBond* findBond(const blast::VoxelStructureGraph& g, uint32_t a, uint32_t b) {
  const uint64_t k = blast::packBondKey(a, b);
  for (const auto& bond : g.bonds) {
    if (blast::packBondKey(bond.nodeA, bond.nodeB) == k) {
      return &bond;
    }
  }
  return nullptr;
}

bool idsUnique(const blast::VoxelStructureGraph& g) {
  std::set<uint32_t> nodes;
  std::set<uint32_t> bonds;
  for (const auto& n : g.nodes) {
    if (n.stableId == 0 || !nodes.insert(n.stableId).second) {
      return false;
    }
  }
  for (const auto& b : g.bonds) {
    if (b.stableId == 0 || !bonds.insert(b.stableId).second) {
      return false;
    }
  }
  return true;
}

void breakFace(blast::VoxelGrid& g, int x0, int y0, int z0, int x1, int y1, int z1) {
  int ax = x0, ay = y0, az = z0, axis = 0;
  if (x0 != x1) {
    axis = 0;
    ax = std::min(x0, x1);
  } else if (y0 != y1) {
    axis = 1;
    ay = std::min(y0, y1);
  } else {
    axis = 2;
    az = std::min(z0, z1);
  }
  g.brokenFaces.insert(blast::packFaceKey(ax, ay, az, axis));
}

void splitCoarseCellTwoWays(blast::VoxelGrid& g, int x0, int y0, int z0) {
  // Separate x=x0 from x=x0+1 inside a 2x2x2 coarse cell.
  for (int z = z0; z < z0 + 2; ++z) {
    for (int y = y0; y < y0 + 2; ++y) {
      breakFace(g, x0, y, z, x0 + 1, y, z);
    }
  }
}

void splitCoarseCellThreeWays(blast::VoxelGrid& g) {
  // 2x2x2: A=(x=0,y=0), B=(x=1,y=0), C=(y=1).
  for (int z = 0; z < 2; ++z) {
    breakFace(g, 0, 0, z, 1, 0, z);
    for (int x = 0; x < 2; ++x) {
      breakFace(g, x, 0, z, x, 1, z);
    }
  }
}

std::vector<NvBlastActor*> familyActors(NvBlastFamily* family) {
  const uint32_t n = NvBlastFamilyGetActorCount(family, blastLog);
  std::vector<NvBlastActor*> actors(n, nullptr);
  NvBlastFamilyGetActors(actors.data(), n, family, blastLog);
  return actors;
}

std::vector<uint32_t> actorInternalBonds(NvBlastActor* actor, const NvBlastAsset* asset) {
  std::vector<uint32_t> out;
  if (actor == nullptr || asset == nullptr) {
    return out;
  }
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, blastLog);
  const std::vector<uint8_t> inActor = blast::actorGraphNodeMask(actor, graph, blastLog);
  std::unordered_set<uint32_t> seen;
  for (uint32_t n0 = 0; n0 < graph.nodeCount; ++n0) {
    if (n0 >= inActor.size() || inActor[n0] == 0) {
      continue;
    }
    for (uint32_t adj = graph.adjacencyPartition[n0]; adj < graph.adjacencyPartition[n0 + 1]; ++adj) {
      const uint32_t n1 = graph.adjacentNodeIndices[adj];
      if (n0 >= n1) {
        continue;
      }
      if (n1 < inActor.size() && inActor[n1] != 0) {
        const uint32_t bi = graph.adjacentBondIndices[adj];
        if (seen.insert(bi).second) {
          out.push_back(bi);
        }
      }
    }
  }
  return out;
}

void addEndLoads(blast::VoxelBlast& vb, const blast::VoxelStructureGraph& graph, float mag) {
  const blast::GraphNode* lo = nullptr;
  const blast::GraphNode* hi = nullptr;
  for (const auto& n : graph.nodes) {
    if (lo == nullptr || n.cz < lo->cz) {
      lo = &n;
    }
    if (hi == nullptr || n.cz > hi->cz) {
      hi = &n;
    }
  }
  if (lo == nullptr || hi == nullptr || vb.solver == nullptr) {
    return;
  }
  const uint32_t g0 = vb.graphFromStable[lo->stableId];
  const uint32_t g1 = vb.graphFromStable[hi->stableId];
  vb.solver->addLoad(g0, NvcVec3{0.0f, 0.0f, mag}, NvcVec3{0.0f, 0.0f, 0.0f});
  vb.solver->addLoad(g1, NvcVec3{0.0f, 0.0f, -mag}, NvcVec3{0.0f, 0.0f, 0.0f});
}

void configureSolverIters(Nv::Blast::ExtStressSolver& solver, uint32_t iters) {
  auto st = solver.getSettings();
  st.maxSolverIterationsPerFrame = iters;
  solver.setSettings(st);
}

bool solveTension(blast::VoxelBlast& vb, const blast::VoxelStructureGraph& graph, float mag, int frames) {
  if (vb.solver == nullptr) {
    return false;
  }
  configureSolverIters(*vb.solver, 200);
  for (int i = 0; i < frames; ++i) {
    addEndLoads(vb, graph, mag);
    vb.solver->update();
    if (vb.solver->converged()) {
      return true;
    }
  }
  return vb.solver->converged();
}

bool buildTwoActorColumn(blast::TrackingAllocator& alloc, blast::VoxelGrid& grid, blast::VoxelStructureGraph& graph,
                         blast::VoxelBlast& vb) {
  blast::initGrid(grid, 2, 2, 8, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 8);
  if (blast::extractGraph(grid, graph, blast::kOwnerAll) != blast::BlastError::Ok) {
    return false;
  }
  if (graph.nodes.size() != 4 || graph.bonds.size() != 3) {
    return false;
  }
  if (blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) != blast::BlastError::Ok) {
    return false;
  }
  // Break the middle interface (nodes 1-2 along z).
  const blast::GraphBond* mid = nullptr;
  for (const auto& b : graph.bonds) {
    if (mid == nullptr) {
      mid = &b;
      continue;
    }
    if (std::abs(b.cz - 0.4f) < std::abs(mid->cz - 0.4f)) {
      mid = &b;
    }
  }
  if (mid == nullptr) {
    return false;
  }
  const uint32_t sdk = vb.sdkBondFromStable[mid->stableId];
  blast::applySdkBondDamage(vb.actor, vb.asset, &sdk, 1, blastLog);
  vb.solver->syncBrokenBonds();
  blast::splitIfNeeded(vb.actor, vb.family, *vb.solver, blastLog);
  blast::breakBondFaces(grid, *mid);
  return NvBlastFamilyGetActorCount(vb.family, blastLog) == 2;
}

void testLargeGraphFractureHighIndex(blast::TrackingAllocator& alloc) {
  std::cout << "R1 large graph fractures sdk bond >= 512\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 1, 1, 520, 1, 0.1f);
  fillBox(grid, 0, 0, 0, 1, 1, 520);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R1 extract");
  expect(graph.bonds.size() >= 512, "R1 graph has >= 512 bonds");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "R1 create");
  const uint32_t nb = blast::familyAssetBondCount(vb.actor, blastLog);
  std::cout << "  asset bonds=" << nb << " graph bonds=" << graph.bonds.size() << std::endl;
  expect(nb > 512, "R1 asset bond count > 512");
  expect(solveTension(vb, graph, 1000.0f, 40), "R1 chain converged");
  const float* h0 = NvBlastActorGetBondHealths(vb.actor, blastLog);
  expect(h0 != nullptr && h0[512] > 0.0f, "R1 bond 512 healthy before fracture");
  const float health512 = h0[512];
  const float health0 = h0[0];
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> probes(nb);
  const uint32_t nProbe = vb.solver->copyBondProbes(probes.data(), nb);
  expect(nProbe == nb, "R1 copyBondProbes returns every asset bond");
  const Nv::Blast::ExtStressSolver::BondProbe* p512 = nullptr;
  for (uint32_t i = 0; i < nProbe; ++i) {
    if (probes[i].blastBondIndex == 512) {
      p512 = &probes[i];
      break;
    }
  }
  expect(p512 != nullptr, "R1 probe exists for sdk index 512");
  float s512 = 0.0f;
  if (p512 != nullptr) {
    s512 = blast::probeMaxStress(*p512);
    expect(s512 > 0.0f, "R1 bond 512 has positive stress");
  }
  const uint32_t nfrac = blast::applyHardThreshold(vb.actor, *vb.solver, s512 * 0.5f, blastLog);
  expect(nfrac > 0, "R1 submitted fractures");
  const float* h1 = NvBlastActorGetBondHealths(vb.actor, blastLog);
  expect(h1[512] <= 0.0f, "R1 sdk bond 512 health is zero");
  expect(health512 > 0.0f, "R1 recorded pre-fracture health of 512");
  (void)health0;
  blast::destroyVoxelBlast(vb);
}

void testLargeGraphBelowThreshold(blast::TrackingAllocator& alloc) {
  std::cout << "R1 large graph below threshold leaves high-index health\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 1, 1, 520, 1, 0.1f);
  fillBox(grid, 0, 0, 0, 1, 1, 520);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R1b extract");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "R1b create");
  expect(solveTension(vb, graph, 1000.0f, 40), "R1b converged");
  const uint32_t nb = blast::familyAssetBondCount(vb.actor, blastLog);
  expect(nb > 512, "R1b bond count > 512");
  const float* h0 = NvBlastActorGetBondHealths(vb.actor, blastLog);
  const float health512 = h0[512];
  const float health0 = h0[0];
  const uint32_t nfrac = blast::applyHardThreshold(vb.actor, *vb.solver, 1.0e30f, blastLog);
  expect(nfrac == 0, "R1b no fracture far above stress");
  const float* h1 = NvBlastActorGetBondHealths(vb.actor, blastLog);
  expect(h1[512] == health512, "R1b bond 512 health unchanged");
  expect(h1[0] == health0, "R1b bond 0 health unchanged");
  blast::destroyVoxelBlast(vb);
}

void testMultiActorFilter(blast::TrackingAllocator& alloc) {
  std::cout << "R1 multi-actor applyHardThreshold stays on the target actor\n";
  blast::VoxelGrid grid;
  blast::VoxelStructureGraph graph;
  blast::VoxelBlast vb;
  expect(buildTwoActorColumn(alloc, grid, graph, vb), "R1c two actors");
  auto actors = familyActors(vb.family);
  expect(actors.size() == 2, "R1c actor count 2");
  NvBlastActor* a0 = actors[0];
  NvBlastActor* a1 = actors[1];
  auto bonds0 = actorInternalBonds(a0, vb.asset);
  auto bonds1 = actorInternalBonds(a1, vb.asset);
  expect(!bonds0.empty() && !bonds1.empty(), "R1c each actor has an internal bond");
  configureSolverIters(*vb.solver, 200);
  for (NvBlastActor* a : actors) {
    const uint32_t n = NvBlastActorGetGraphNodeCount(a, blastLog);
    std::vector<uint32_t> nodes(n, 0);
    NvBlastActorGetGraphNodeIndices(nodes.data(), n, a, blastLog);
    if (n >= 1) {
      vb.solver->addLoad(nodes[0], NvcVec3{0.0f, 0.0f, 2000.0f}, NvcVec3{0.0f, 0.0f, 0.0f});
    }
    if (n >= 2) {
      vb.solver->addLoad(nodes[n - 1], NvcVec3{0.0f, 0.0f, -2000.0f}, NvcVec3{0.0f, 0.0f, 0.0f});
    }
  }
  vb.solver->update();
  expect(vb.solver->converged(), "R1c converged");
  const float* h0 = NvBlastActorGetBondHealths(a0, blastLog);
  std::vector<float> healthB;
  healthB.reserve(bonds1.size());
  for (uint32_t bi : bonds1) {
    healthB.push_back(h0[bi]);
    expect(h0[bi] > 0.0f, "R1c actor B bond healthy before");
  }
  for (uint32_t bi : bonds0) {
    expect(h0[bi] > 0.0f, "R1c actor A bond healthy before");
  }
  const uint32_t nfrac = blast::applyHardThreshold(a0, *vb.solver, 0.0f, blastLog);
  expect(nfrac > 0, "R1c actor A fractured");
  const float* h1 = NvBlastActorGetBondHealths(a0, blastLog);
  for (uint32_t bi : bonds0) {
    expect(h1[bi] <= 0.0f, "R1c actor A specified bond health is zero");
  }
  for (size_t i = 0; i < bonds1.size(); ++i) {
    expect(h1[bonds1[i]] == healthB[i], "R1c actor B specified bond health unchanged");
  }
  blast::destroyVoxelBlast(vb);
}

void testThresholdBoundaries(blast::TrackingAllocator& alloc) {
  std::cout << "R1 threshold equality / slightly over / no-converge\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R1d extract");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "R1d create");
  expect(graph.bonds.size() == 1, "R1d one bond");
  const uint32_t sdk = vb.sdkBondFromStable[graph.bonds[0].stableId];
  configureSolverIters(*vb.solver, 200);
  addEndLoads(vb, graph, 500.0f);
  vb.solver->update();
  expect(vb.solver->converged(), "R1d converged");
  const uint32_t nb = blast::familyAssetBondCount(vb.actor, blastLog);
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> probes(nb);
  const uint32_t nProbe = vb.solver->copyBondProbes(probes.data(), nb);
  float sHot = 0.0f;
  for (uint32_t i = 0; i < nProbe; ++i) {
    if (probes[i].blastBondIndex == sdk) {
      sHot = blast::probeMaxStress(probes[i]);
    }
  }
  expect(sHot > 0.0f, "R1d target bond stressed");
  const float* h0 = NvBlastActorGetBondHealths(vb.actor, blastLog);
  const float healthHot = h0[sdk];
  expect(blast::applyHardThreshold(vb.actor, *vb.solver, sHot, blastLog) == 0, "R1d equal threshold no damage");
  expect(NvBlastActorGetBondHealths(vb.actor, blastLog)[sdk] == healthHot, "R1d health unchanged at equality");
  expect(blast::applyHardThreshold(vb.actor, *vb.solver, sHot * 0.999f, blastLog) > 0, "R1d slightly over fractures");
  expect(NvBlastActorGetBondHealths(vb.actor, blastLog)[sdk] <= 0.0f, "R1d one-shot full health");
  blast::destroyVoxelBlast(vb);

  blast::VoxelGrid chain;
  blast::initGrid(chain, 1, 1, 64, 1, 0.1f);
  fillBox(chain, 0, 0, 0, 1, 1, 64);
  blast::VoxelStructureGraph chainGraph;
  expect(blast::extractGraph(chain, chainGraph, blast::kOwnerAll) == blast::BlastError::Ok, "R1d chain extract");
  blast::VoxelBlast vb2;
  expect(blast::createVoxelBlast(alloc, chainGraph, chain, vb2, 1.0e7f) == blast::BlastError::Ok, "R1d chain create");
  const uint32_t sdk2 = vb2.sdkBondFromStable[chainGraph.bonds.front().stableId];
  configureSolverIters(*vb2.solver, 1);
  addEndLoads(vb2, chainGraph, 500.0f);
  vb2.solver->update();
  const float healthBefore = NvBlastActorGetBondHealths(vb2.actor, blastLog)[sdk2];
  if (!vb2.solver->converged()) {
    expect(blast::applyHardThreshold(vb2.actor, *vb2.solver, 0.0f, blastLog) == 0, "R1d no converge no fracture");
    expect(NvBlastActorGetBondHealths(vb2.actor, blastLog)[sdk2] == healthBefore, "R1d health unchanged if not converged");
  } else {
    std::cout << "  R1d 1-iter unexpectedly converged; skip no-fracture clause\n";
  }
  blast::destroyVoxelBlast(vb2);
}

void testSplitIdsUnique() {
  std::cout << "R2 split two and three nodes have unique ids\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 2, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 2);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2 two extract0");
  expect(graph.nodes.size() == 1, "R2 two start as one node");
  const uint32_t oldId = graph.nodes[0].stableId;
  splitCoarseCellTwoWays(grid, 0, 0, 0);
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2 two extract1");
  expect(graph.nodes.size() == 2, "R2 two nodes after split");
  expect(idsUnique(graph), "R2 two ids unique");
  bool kept = false;
  for (const auto& n : graph.nodes) {
    kept = kept || n.stableId == oldId;
  }
  expect(kept, "R2 one child kept the old id");

  blast::VoxelGrid g3;
  blast::initGrid(g3, 2, 2, 2, 2, 0.1f);
  fillBox(g3, 0, 0, 0, 2, 2, 2);
  blast::VoxelStructureGraph graph3;
  expect(blast::extractGraph(g3, graph3, blast::kOwnerAll) == blast::BlastError::Ok, "R2 three extract0");
  const uint32_t old3 = graph3.nodes[0].stableId;
  splitCoarseCellThreeWays(g3);
  expect(blast::extractGraph(g3, graph3, blast::kOwnerAll) == blast::BlastError::Ok, "R2 three extract1");
  expect(graph3.nodes.size() == 3, "R2 three nodes after split");
  expect(idsUnique(graph3), "R2 three ids unique");
  int keepCount = 0;
  for (const auto& n : graph3.nodes) {
    keepCount += n.stableId == old3 ? 1 : 0;
  }
  expect(keepCount == 1, "R2 old id assigned to exactly one of three");
}

void testRepeatedCutNoReuse() {
  std::cout << "R2 repeated cuts do not reuse ids\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2r extract0");
  std::set<uint32_t> retired;
  auto snapshot = [&]() {
    for (const auto& n : graph.nodes) {
      retired.insert(n.stableId);
    }
    for (const auto& b : graph.bonds) {
      retired.insert(b.stableId + 0x80000000u);
    }
  };
  snapshot();
  splitCoarseCellTwoWays(grid, 0, 0, 2);
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2r extract1");
  expect(idsUnique(graph), "R2r unique after first cut");
  std::set<uint32_t> live;
  for (const auto& n : graph.nodes) {
    live.insert(n.stableId);
  }
  splitCoarseCellTwoWays(grid, 0, 0, 0);
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2r extract2");
  expect(idsUnique(graph), "R2r unique after second cut");
  for (const auto& n : graph.nodes) {
    if (live.count(n.stableId) == 0) {
      expect(retired.count(n.stableId) == 0, "R2r new node id was not previously used");
    }
  }
}

void testDeterministicRepeat() {
  std::cout << "R2 identical inputs produce identical maps\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 4, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 4, 2, 4);
  splitCoarseCellTwoWays(grid, 0, 0, 0);
  blast::VoxelStructureGraph a;
  blast::VoxelStructureGraph b;
  expect(blast::extractGraph(grid, a, blast::kOwnerAll) == blast::BlastError::Ok, "R2d extract a");
  expect(blast::extractGraph(grid, b, blast::kOwnerAll) == blast::BlastError::Ok, "R2d extract b");
  expect(a.nodes.size() == b.nodes.size(), "R2d node count");
  expect(a.bonds.size() == b.bonds.size(), "R2d bond count");
  for (const auto& kv : a.voxelNode) {
    expect(b.voxelNode[kv.first] == kv.second, "R2d voxel id map matches");
  }
  blast::VoxelStructureGraph c;
  expect(blast::extractGraph(grid, c, blast::kOwnerAll) == blast::BlastError::Ok, "R2d extract c");
  expect(blast::extractGraph(grid, c, blast::kOwnerAll) == blast::BlastError::Ok, "R2d extract c again");
  for (const auto& kv : a.voxelNode) {
    expect(c.voxelNode[kv.first] == kv.second, "R2d rebuild on same graph keeps map");
  }
}

void testHighNodeBondIds() {
  std::cout << "R2 bond ids stay unique when node ids exceed 65535\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 6, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 6);
  blast::VoxelStructureGraph graph;
  graph.nextStable = 70000;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2h extract");
  expect(graph.nodes.size() == 3 && graph.bonds.size() == 2, "R2h 3 nodes 2 bonds");
  expect(idsUnique(graph), "R2h unique");
  for (const auto& n : graph.nodes) {
    expect(n.stableId >= 70000, "R2h node ids from high counter");
  }
  std::set<uint32_t> bondIds;
  for (const auto& b : graph.bonds) {
    expect(b.stableId != 0, "R2h bond id nonzero");
    expect(bondIds.insert(b.stableId).second, "R2h bond ids unique");
    const uint32_t packed = (b.nodeA << 16) | (b.nodeB & 0xFFFFu);
    expect(b.stableId != packed, "R2h bond id is not truncated node-pair packing");
  }
}

void testDamageMigratesOnSplit() {
  std::cout << "R2 partial damage migrates across a node split\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2m extract0");
  expect(graph.bonds.size() == 1 && graph.bonds[0].nFaces == 4, "R2m 4-face interface");
  grid.bondDamage[blast::packBondKey(graph.bonds[0].nodeA, graph.bonds[0].nodeB)] = 0.4f;
  splitCoarseCellTwoWays(grid, 0, 0, 2);
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2m extract1");
  expect(graph.nodes.size() == 3, "R2m three nodes");
  expect(graph.bonds.size() == 2, "R2m interface split into two bonds");
  for (const auto& b : graph.bonds) {
    expect(std::abs(b.d - 0.4f) < 1.0e-6f, "R2m damage d migrated");
    expect(b.nFaces == 2, "R2m remaining faces per child bond");
    const float ageo = blast::bondAgeo(b, grid.voxelSize);
    const float aeff = blast::bondAeff(b, grid.voxelSize);
    expect(std::abs(aeff - ageo * 0.6f) < 1.0e-8f, "R2m A_eff uses migrated d");
  }
}

void testBrokenFacesStayBroken() {
  std::cout << "R2 broken faces stay broken across rebuilds\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 6, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 6);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2b extract0");
  const uint32_t a = graph.nodes[0].stableId;
  const uint32_t b = graph.nodes[1].stableId;
  const blast::GraphBond* ab = findBond(graph, a, b);
  expect(ab != nullptr, "R2b ab exists");
  blast::breakBondFaces(grid, *ab);
  for (int i = 0; i < 3; ++i) {
    expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2b rebuild");
    expect(findBond(graph, a, b) == nullptr, "R2b broken interface does not heal");
  }
}

void testAmbiguousDamageRejected() {
  std::cout << "R2 ambiguous damage source is rejected\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 1, 4, 1, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 1, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2a extract0");
  expect(graph.nodes.size() == 8, "R2a agg=1 eight nodes");
  const size_t n0 = graph.nodes.size();
  const uint32_t next0 = graph.nextStable;
  grid.agg = 2;
  const blast::BlastError err = blast::extractGraph(grid, graph, blast::kOwnerAll);
  expect(err == blast::BlastError::AmbiguousDamage, "R2a merge of two old bonds is ambiguous");
  expect(graph.nodes.size() == n0, "R2a graph unchanged on failure");
  expect(graph.nextStable == next0, "R2a counters unchanged on failure");
}

void testIdExhaustion() {
  std::cout << "R2 id exhaustion fails instead of wrapping\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 2, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 2);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R2e extract0");
  expect(graph.nodes.size() == 1, "R2e one node");
  const uint32_t id0 = graph.nodes[0].stableId;
  splitCoarseCellThreeWays(grid);
  graph.nextStable = (std::numeric_limits<uint32_t>::max)();
  const blast::BlastError err = blast::extractGraph(grid, graph, blast::kOwnerAll);
  expect(err == blast::BlastError::IdExhausted, "R2e second new id refuses wrap");
  expect(graph.nodes.size() == 1 && graph.nodes[0].stableId == id0, "R2e graph restored");
}

void testOwnersExceed255(blast::TrackingAllocator& alloc) {
  std::cout << "R3 family with >255 actors stamps unique nonzero owners\n";
  constexpr int nx = 16;
  constexpr int ny = 17;
  blast::VoxelGrid grid;
  blast::initGrid(grid, nx, ny, 1, 1, 0.1f);
  fillBox(grid, 0, 0, 0, nx, ny, 1);
  const uint32_t solids0 = blast::countSolidVoxels(grid);
  expect(solids0 > 255, "R3 more than 255 voxels");
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R3 extract");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "R3 create");
  std::vector<uint32_t> allBonds;
  allBonds.reserve(vb.sdkBondFromStable.size());
  for (const auto& kv : vb.sdkBondFromStable) {
    allBonds.push_back(kv.second);
  }
  blast::applySdkBondDamage(vb.actor, vb.asset, allBonds.data(), static_cast<uint32_t>(allBonds.size()), blastLog);
  vb.solver->syncBrokenBonds();
  const uint32_t actors = blast::splitIfNeeded(vb.actor, vb.family, *vb.solver, blastLog);
  std::cout << "  actors=" << actors << " solids=" << solids0 << "\n";
  expect(actors > 255, "R3 split produced >255 actors");
  std::vector<blast::CompactFamily> families;
  expect(static_cast<bool>(blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f)), "R3 compact replace");
  expect(families.size() == actors, "R3 one family per actor");
  std::set<blast::OwnerId> owners;
  uint32_t voxelSum = 0;
  for (const auto& cf : families) {
    expect(cf.owner != blast::kUnowned, "R3 owner is not zero");
    expect(owners.insert(cf.owner).second, "R3 owner unique");
    expect(cf.blast.family != nullptr && cf.blast.actor != nullptr, "R3 geometry component exists");
    expect(NvBlastActorGetVisibleChunkCount(cf.blast.actor, blastLog) > 0, "R3 visible chunk present");
    for (const auto& n : cf.graph.nodes) {
      voxelSum += static_cast<uint32_t>(n.voxels.size());
      expect(n.owner == cf.owner, "R3 node owner matches family");
    }
  }
  expect(voxelSum == solids0, "R3 occupancy conserved");
  std::set<blast::OwnerId> gridOwners;
  uint32_t ownedSolids = 0;
  for (int y = 0; y < ny; ++y) {
    for (int x = 0; x < nx; ++x) {
      expect(grid.isSolid(x, y, 0), "R3 voxel still solid");
      const blast::OwnerId o = grid.ownerAt(x, y, 0);
      expect(o != blast::kUnowned, "R3 no unowned solid");
      gridOwners.insert(o);
      ++ownedSolids;
    }
  }
  expect(ownedSolids == solids0, "R3 every solid has an owner");
  expect(gridOwners.size() == families.size(), "R3 grid owners match families");
  for (auto& cf : families) {
    blast::destroyVoxelBlast(cf.blast);
  }
}

void testOwnerFilterDoesNotCross(blast::TrackingAllocator& alloc) {
  std::cout << "R3 owner filter does not absorb another actor\n";
  blast::VoxelGrid grid;
  blast::VoxelStructureGraph graph;
  blast::VoxelBlast vb;
  expect(buildTwoActorColumn(alloc, grid, graph, vb), "R3f two actors");
  std::vector<blast::CompactFamily> families;
  expect(static_cast<bool>(blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f)), "R3f replace");
  expect(families.size() == 2, "R3f two families");
  blast::VoxelStructureGraph allG;
  expect(blast::extractGraph(grid, allG, blast::kOwnerAll) == blast::BlastError::Ok, "R3f extract all");
  std::set<blast::OwnerId> nodeOwners;
  for (const auto& n : allG.nodes) {
    nodeOwners.insert(n.owner);
    expect(n.owner != blast::kUnowned, "R3f assigned owner on all-query nodes");
  }
  expect(nodeOwners.size() == 2, "R3f all-query does not merge owners");
  for (const auto& cf : families) {
    blast::VoxelStructureGraph part;
    expect(blast::extractGraph(grid, part, cf.owner) == blast::BlastError::Ok, "R3f extract owner");
    expect(!part.nodes.empty(), "R3f filtered graph nonempty");
    for (const auto& n : part.nodes) {
      expect(n.owner == cf.owner, "R3f filtered nodes match owner");
    }
    uint32_t vox = 0;
    for (const auto& n : part.nodes) {
      vox += static_cast<uint32_t>(n.voxels.size());
    }
    uint32_t expectV = 0;
    for (int z = 0; z < grid.nz; ++z) {
      for (int y = 0; y < grid.ny; ++y) {
        for (int x = 0; x < grid.nx; ++x) {
          if (grid.isSolid(x, y, z) && grid.ownerAt(x, y, z) == cf.owner) {
            ++expectV;
          }
        }
      }
    }
    expect(vox == expectV, "R3f filter coverage");
  }
  for (auto& cf : families) {
    blast::destroyVoxelBlast(cf.blast);
  }
}

bool gridsEqualOccupancy(const blast::VoxelGrid& a, const blast::VoxelGrid& b) {
  return a.nx == b.nx && a.ny == b.ny && a.nz == b.nz && a.solid == b.solid && a.brokenFaces == b.brokenFaces &&
         a.bondDamage.size() == b.bondDamage.size();
}

void testFailFirstCreate(blast::TrackingAllocator& alloc) {
  std::cout << "R4 first replacement create failure rolls back\n";
  blast::VoxelGrid grid;
  blast::VoxelStructureGraph graph;
  blast::VoxelBlast vb;
  expect(buildTwoActorColumn(alloc, grid, graph, vb), "R4a build");
  const std::size_t baseline = alloc.liveBytes();
  const auto liveAllocs = alloc.liveAllocs();
  blast::VoxelGrid gridCopy = grid;
  NvBlastFamily* oldFamily = vb.family;
  Nv::Blast::ExtStressSolver* oldSolver = vb.solver;
  std::vector<blast::CompactFamily> families;
  families.emplace_back();
  blast::CompactReplaceOpts opts;
  opts.failAfterCreates = 0;
  const blast::RebuildResult r = blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f, opts);
  expect(r.error == blast::BlastError::InjectedFailure, "R4a injected failure");
  expect(vb.family == oldFamily && vb.solver == oldSolver, "R4a old instance unchanged");
  expect(gridsEqualOccupancy(grid, gridCopy), "R4a caller grid occupancy unchanged");
  expect(alloc.liveBytes() == baseline, "R4a live bytes back to baseline");
  expect(alloc.liveAllocs() == liveAllocs, "R4a live allocs back to baseline");
  expect(families.size() == 1 && families[0].blast.solver == nullptr, "R4a output collection unchanged");
  configureSolverIters(*vb.solver, 20);
  vb.solver->addGravity(*vb.actor, NvcVec3{0.0f, -9.81f, 0.0f});
  vb.solver->update();
  expect(vb.family != nullptr, "R4a old family still solvable");
  blast::destroyVoxelBlast(vb);
}

void testFailSecondCreate(blast::TrackingAllocator& alloc) {
  std::cout << "R4 second replacement create failure releases the first temp\n";
  blast::VoxelGrid grid;
  blast::VoxelStructureGraph graph;
  blast::VoxelBlast vb;
  expect(buildTwoActorColumn(alloc, grid, graph, vb), "R4b build");
  const std::size_t baseline = alloc.liveBytes();
  Nv::Blast::ExtStressSolver* oldSolver = vb.solver;
  std::vector<blast::CompactFamily> families;
  blast::CompactReplaceOpts opts;
  opts.failAfterCreates = 1;
  const blast::RebuildResult r = blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f, opts);
  expect(r.error == blast::BlastError::InjectedFailure, "R4b injected failure");
  expect(vb.solver == oldSolver && vb.family != nullptr, "R4b old instance unchanged");
  expect(families.empty(), "R4b no commit");
  expect(alloc.liveBytes() == baseline, "R4b first temp released, baseline restored");
  expect(NvBlastFamilyGetActorCount(vb.family, blastLog) == 2, "R4b old family still has two actors");
  vb.solver->update();
  blast::destroyVoxelBlast(vb);
}

void testFailMapping(blast::TrackingAllocator& alloc) {
  std::cout << "R4 mapping check failure does not commit\n";
  blast::VoxelGrid grid;
  blast::VoxelStructureGraph graph;
  blast::VoxelBlast vb;
  expect(buildTwoActorColumn(alloc, grid, graph, vb), "R4c build");
  const std::size_t baseline = alloc.liveBytes();
  std::vector<blast::CompactFamily> families;
  blast::CompactReplaceOpts opts;
  opts.failMappingCheck = true;
  const blast::RebuildResult r = blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f, opts);
  expect(r.error == blast::BlastError::MappingInvalid, "R4c mapping error");
  expect(r.message != nullptr && std::string(r.message).find("mapping") != std::string::npos, "R4c reason string");
  expect(vb.family != nullptr && vb.solver != nullptr, "R4c old instance kept");
  expect(families.empty(), "R4c not committed");
  expect(alloc.liveBytes() == baseline, "R4c temps released");
  blast::destroyVoxelBlast(vb);
}

void testCommitSuccess(blast::TrackingAllocator& alloc) {
  std::cout << "R4 successful replace commits new instances and releases old\n";
  blast::VoxelGrid grid;
  blast::VoxelStructureGraph graph;
  blast::VoxelBlast vb;
  expect(buildTwoActorColumn(alloc, grid, graph, vb), "R4d build");
  std::vector<blast::CompactFamily> families;
  const blast::RebuildResult r = blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f);
  expect(r.error == blast::BlastError::Ok, "R4d ok");
  expect(vb.family == nullptr && vb.solver == nullptr, "R4d old released");
  expect(families.size() == 2, "R4d two new families");
  for (auto& cf : families) {
    expect(cf.blast.solver != nullptr && cf.blast.actor != nullptr, "R4d new instance usable");
    expect(NvBlastFamilyGetActorCount(cf.blast.family, blastLog) == 1, "R4d compact family one actor");
    blast::destroyVoxelBlast(cf.blast);
  }
}

void testCommitEmpty(blast::TrackingAllocator& alloc) {
  std::cout << "R4 deleting every voxel commits an empty structure\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "R4e extract");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "R4e create");
  const std::size_t afterCreate = alloc.liveBytes();
  expect(afterCreate > 0, "R4e instance holds memory");
  for (int z = 0; z < 4; ++z) {
    for (int y = 0; y < 2; ++y) {
      for (int x = 0; x < 2; ++x) {
        grid.setSolid(x, y, z, false);
      }
    }
  }
  expect(blast::countSolidVoxels(grid) == 0, "R4e no solids");
  std::vector<blast::CompactFamily> families;
  families.emplace_back();
  const blast::RebuildResult r = blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f);
  expect(r.error == blast::BlastError::Ok, "R4e empty commit is success");
  expect(families.empty(), "R4e output is empty");
  expect(vb.solver == nullptr && vb.family == nullptr, "R4e old released");
  expect(alloc.liveBytes() < afterCreate, "R4e old resources released");
}

}  // namespace

int main() {
  std::cout << "blast_graph_regression_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  blast::TrackingAllocator alloc;
  blast::LoggingErrorCallback errors;
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);

  testLargeGraphFractureHighIndex(alloc);
  testLargeGraphBelowThreshold(alloc);
  testMultiActorFilter(alloc);
  testThresholdBoundaries(alloc);

  testSplitIdsUnique();
  testRepeatedCutNoReuse();
  testDeterministicRepeat();
  testHighNodeBondIds();
  testDamageMigratesOnSplit();
  testBrokenFacesStayBroken();
  testAmbiguousDamageRejected();
  testIdExhaustion();

  testOwnersExceed255(alloc);
  testOwnerFilterDoesNotCross(alloc);

  testFailFirstCreate(alloc);
  testFailSecondCreate(alloc);
  testFailMapping(alloc);
  testCommitSuccess(alloc);
  testCommitEmpty(alloc);

  expect(errors.errorCount() == 0, "no NvBlast error-callback errors");
  if (gFailures == 0) {
    std::cout << "OK graph regression R1-R4\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
