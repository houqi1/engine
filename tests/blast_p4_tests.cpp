#include "blast/BlastMemory.h"
#include "blast/HardFracture.h"
#include "blast/VoxelGraph.h"

#include "NvBlast.h"
#include "NvBlastGlobals.h"

#include <cmath>
#include <iostream>
#include <string>

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

void testT14() {
  std::cout << "T14 coarse-cell cut-through\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 4, 4, 4, 4, 0.1f);
  fillBox(grid, 0, 0, 0, 4, 4, 4);
  for (int y = 0; y < 4; ++y) {
    for (int x = 0; x < 4; ++x) {
      grid.setSolid(x, y, 2, false);
    }
  }
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "T14 extract");
  std::cout << "  nodes=" << graph.nodes.size() << " bonds=" << graph.bonds.size() << "\n";
  expect(graph.nodes.size() == 2, "T14 two nodes after cut-through");
  expect(graph.bonds.empty(), "T14 no bond across the gap");
}

void testT15(blast::TrackingAllocator& alloc) {
  std::cout << "T15 rebuild keeps cuts and damage\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 6, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 6);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "T15 extract");
  expect(graph.nodes.size() == 3, "T15 three coarse nodes along z");
  expect(graph.bonds.size() == 2, "T15 two bonds");
  const uint32_t a = graph.nodes[0].stableId;
  const uint32_t b = graph.nodes[1].stableId;
  const uint32_t c = graph.nodes[2].stableId;
  const blast::GraphBond* ab = findBond(graph, a, b);
  const blast::GraphBond* bc = findBond(graph, b, c);
  expect(ab && bc, "T15 both bonds exist");
  grid.bondDamage[blast::packBondKey(bc->nodeA, bc->nodeB)] = 0.4f;
  blast::breakBondFaces(grid, *ab);
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "T15 rebuild");
  expect(findBond(graph, a, b) == nullptr, "T15 broken faces do not heal");
  const blast::GraphBond* bc2 = findBond(graph, b, c);
  expect(bc2 != nullptr, "T15 uncut bond remains");
  expect(std::abs(bc2->d - 0.4f) < 1.0e-6f, "T15 damage d preserved");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "T15 blast create");
  const auto it = vb.sdkBondFromStable.find(bc2->stableId);
  expect(it != vb.sdkBondFromStable.end(), "T15 sdk bond mapped");
  if (it != vb.sdkBondFromStable.end()) {
    const float* h = NvBlastActorGetBondHealths(vb.actor, blastLog);
    const float aeff = blast::bondAeff(*bc2, grid.voxelSize);
    expect(std::abs(h[it->second] - aeff) / aeff < 0.01f, "T15 health is A_eff not 1");
  }
  blast::destroyVoxelBlast(vb);
}

void testT16(blast::TrackingAllocator& alloc) {
  std::cout << "T16 thin + delete voxels\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "T16 extract");
  expect(graph.nodes.size() == 2, "T16 two nodes");
  expect(graph.bonds.size() == 1, "T16 one bond");
  const int faces0 = graph.bonds[0].nFaces;
  const float mass0 = graph.nodes[0].mass + graph.nodes[1].mass;
  const float d0 = graph.bonds[0].d;
  expect(faces0 == 4, "T16 2x2 interface");
  grid.setSolid(0, 0, 1, false);
  grid.setSolid(1, 0, 1, false);
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "T16 rebuild");
  expect(graph.bonds.size() == 1, "T16 bond remains");
  expect(graph.bonds[0].nFaces == 2, "T16 A_geo faces halved");
  expect(graph.bonds[0].d == d0, "T16 d not increased by the cut");
  const float mass1 = graph.nodes[0].mass + graph.nodes[1].mass;
  expect(mass1 < mass0, "T16 mass dropped");
  const float ageo = blast::bondAgeo(graph.bonds[0], grid.voxelSize);
  const float aeff = blast::bondAeff(graph.bonds[0], grid.voxelSize);
  expect(std::abs(aeff - ageo * (1.0f - d0)) < 1.0e-8f, "T16 A_eff = A_geo*(1-d)");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "T16 blast");
  blast::destroyVoxelBlast(vb);
}

void testT17(blast::TrackingAllocator& alloc) {
  std::cout << "T17 multi-actor compact replace\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 2, 2, 4, 2, 0.1f);
  fillBox(grid, 0, 0, 0, 2, 2, 4);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "T17 extract");
  expect(graph.bonds.size() == 1, "T17 one bond");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "T17 create");
  const uint32_t sdk = vb.sdkBondFromStable[graph.bonds[0].stableId];
  const uint32_t* bonds = &sdk;
  blast::applySdkBondDamage(vb.actor, vb.asset, bonds, 1, blastLog);
  vb.solver->syncBrokenBonds();
  blast::splitIfNeeded(vb.actor, vb.family, *vb.solver, blastLog);
  const uint32_t actorsBefore = NvBlastFamilyGetActorCount(vb.family, blastLog);
  std::cout << "  actors after split=" << actorsBefore << "\n";
  expect(actorsBefore == 2, "T17 split into two actors");
  blast::breakBondFaces(grid, graph.bonds[0]);
  const float volB = graph.nodes[1].volume;
  grid.setSolid(0, 0, 0, false);
  std::vector<blast::CompactFamily> families;
  expect(blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f).error == blast::BlastError::Ok,
         "T17 compact replace");
  expect(families.size() == 2, "T17 two compact families");
  expect(vb.family == nullptr && vb.solver == nullptr, "T17 old family released");
  bool sawEdited = false;
  bool sawKept = false;
  for (const auto& cf : families) {
    float vol = 0.0f;
    for (const auto& n : cf.graph.nodes) {
      vol += n.volume;
    }
    if (std::abs(vol - volB) < 1.0e-8f) {
      sawKept = true;
      expect(cf.graph.nodes.size() == 1, "T17 unedited actor one node");
    } else {
      sawEdited = true;
      expect(vol < volB, "T17 edited actor lost mass");
    }
    expect(cf.blast.family != nullptr, "T17 compact family alive");
    expect(NvBlastFamilyGetActorCount(cf.blast.family, blastLog) == 1, "T17 each compact family one actor");
  }
  expect(sawEdited && sawKept, "T17 both edited and preserved actors");
  for (auto& cf : families) {
    blast::destroyVoxelBlast(cf.blast);
  }
}

}  // namespace

int main() {
  std::cout << "blast_p4_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  blast::TrackingAllocator alloc;
  blast::LoggingErrorCallback errors;
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);

  testT14();
  testT15(alloc);
  testT16(alloc);
  testT17(alloc);

  expect(errors.errorCount() == 0, "no NvBlast error-callback errors");
  if (gFailures == 0) {
    std::cout << "OK P4 T14 T15 T16 T17\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
