#include "blast/BlastMemory.h"
#include "blast/HardFracture.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
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
    for (int pass = 0; pass < 3; ++pass) {
      uint32_t covered = 0;
      for (const auto& node : cf.graph.nodes) covered += static_cast<uint32_t>(node.voxels.size());
      expect(blast::countSolidVoxels(cf.grid) == covered, "T17 family occupancy excludes siblings");
      const auto voxel = cf.graph.nodes.front().voxels.front();
      expect(blast::markRemovedVoxel(cf.grid, voxel.x, voxel.y, voxel.z), "T17 repeated edit");
      std::vector<blast::CompactFamily> next;
      const auto result = blast::compactReplace(alloc, cf.grid, cf.graph, cf.blast, next, 1.0e7f);
      expect(static_cast<bool>(result), "T17 repeated compact succeeds");
      if (!result || next.size() != 1) break;
      cf = std::move(next.front());
    }
  }
  for (auto& cf : families) {
    blast::destroyVoxelBlast(cf.blast);
  }
}

void testRebuildKeepsAnchorAndCut(blast::TrackingAllocator& alloc) {
  std::cout << "E4 rebuild keeps anchor and cut\n";
  blast::VoxelGrid grid;
  blast::initGrid(grid, 1, 4, 1, 1, 0.1f);
  fillBox(grid, 0, 0, 0, 1, 4, 1);
  grid.setAnchor(0, 0, 0, true);
  blast::VoxelStructureGraph graph;
  expect(blast::extractGraph(grid, graph, blast::kOwnerAll) == blast::BlastError::Ok, "E4 extract");
  expect(blast::attachWorldBonds(grid, graph) == blast::BlastError::Ok, "E4 anchor");
  blast::VoxelBlast vb;
  expect(blast::createVoxelBlast(alloc, graph, grid, vb, 1.0e7f) == blast::BlastError::Ok, "E4 create");
  const uint64_t saw = blast::packFaceKey(0, 1, 0, 1);
  grid.brokenFaces.insert(saw);
  expect(blast::markRemovedVoxel(grid, 0, 3, 0), "E4 remove top");
  std::vector<blast::CompactFamily> families;
  expect(static_cast<bool>(blast::compactReplace(alloc, grid, graph, vb, families, 1.0e7f)), "E4 replace");
  expect(families.size() == 1, "E4 one family");
  if (!families.empty()) {
    bool world = false;
    bool sawFace = false;
    for (const blast::GraphBond& b : families[0].graph.bonds) {
      if (b.world) {
        world = true;
      }
      for (uint64_t f : b.faces) {
        if (f == saw) {
          sawFace = true;
        }
      }
    }
    expect(world, "E4 world bond restored");
    expect(!sawFace, "E4 cut face stays open");
    expect(!families[0].grid.isSolid(0, 3, 0), "E4 top stays empty");
    blast::destroyVoxelBlast(families[0].blast);
  }
}

blast::OccupancySample columnSample() {
  blast::OccupancySample sample;
  blast::initGrid(sample.grid, 1, 7, 1, 1, 0.1f);
  fillBox(sample.grid, 0, 0, 0, 1, 7, 1);
  sample.grid.setAnchor(0, 0, 0, true);
  sample.error = blast::extractGraph(sample.grid, sample.graph, blast::kOwnerAll);
  expect(sample.error == blast::BlastError::Ok, "E4 column graph");
  expect(blast::attachWorldBonds(sample.grid, sample.graph) == blast::BlastError::Ok, "E4 column anchor");
  sample.occupied = 7;
  for (const auto& node : sample.graph.nodes) sample.mass += node.mass;
  sample.worldBonds = 1;
  return sample;
}

void testWorldRemovalAndSecondInstance(blast::BlastRuntime& runtime) {
  std::cout << "E4 bridge removal retains detached stress actors and repeated edits\n";
  const auto baseline = runtime.liveBytes();
  {
    blast::StructureWorld world;
    expect(world.init(runtime), "E4 world init");
    const VoxelObjectId parent{10, 1}, child{11, 1};
    expect(world.mountSample(parent, columnSample(), 50, 1.0e7f, 0, 0) == blast::BlastError::Ok,
           "E4 mount column");
    const glm::ivec3 bridge(0, 3, 0);
    expect(world.applyOccupancyRemoval(parent, &bridge, 1) == blast::BlastError::Ok, "E4 remove bridge");
    auto* inst = world.instance();
    expect(inst->bindings.size() == 2, "E4 cut immediately creates two actors without stress threshold");
    expect(world.takeOccupancyDirty(inst), "E4 removal requests scene split");
    std::vector<blast::ActorObjectLink> links;
    for (const auto& binding : inst->bindings) {
      blast::ActorObjectLink link;
      link.actor = binding.actor;
      link.objectId = binding.anchored ? parent : child;
      link.fineOrigin = binding.anchored ? glm::ivec3(0) : glm::ivec3(0, 4, 0);
      link.fineN = 8;
      links.push_back(link);
      expect(binding.stressSolve, "E4 both pieces remain stress-capable");
    }
    world.bindVisibleActors(links, inst);
    expect(world.ownsObject(child), "E4 child is still mounted");
    const glm::ivec3 tip(0, 2, 0);
    expect(world.applyOccupancyRemoval(child, &tip, 1) == blast::BlastError::Ok, "E4 edit rebased child");
    expect(world.instanceCount() == 2, "E4 compact separates owners into instances");
    blast::StructureInstance* detached = nullptr;
    blast::StructureInstance* anchored = nullptr;
    for (uint32_t i = 0; i < world.instanceCount(); ++i) {
      auto* part = world.instanceAt(i);
      if (part->objectId == child) detached = part;
      if (part->objectId == parent) anchored = part;
      for (const auto& binding : part->bindings) {
        expect(binding.fineN == 8, "E4 rebuild retains fine dimensions");
      }
    }
    expect(detached && anchored, "E4 both owner identities preserved");
    if (detached && anchored) {
      expect(blast::countSolidVoxels(detached->grid) == 2, "E4 child has only surviving child fines");
      expect(blast::countSolidVoxels(anchored->grid) == 3, "E4 unedited parent keeps mass");
      expect(!world.takeOccupancyDirty(anchored), "E4 unedited owner needs no body replacement or wake");
      expect(detached->bindings.front().fineOrigin == glm::ivec3(0, 4, 0), "E4 child origin preserved");
      const glm::ivec3 another(0, 1, 0);
      expect(world.applyOccupancyRemoval(child, &another, 1) == blast::BlastError::Ok, "E4 second compact edit succeeds");
    }
    world.clear();

    std::cout << "E4 explicit second-instance fracture submission\n";
    expect(world.mountSample(parent, columnSample(), 50, 1.0e7f, 0, 0) == blast::BlastError::Ok, "E4 first mount");
    expect(world.mountSample(child, columnSample(), 50, 1.0e7f, 0, 0) == blast::BlastError::Ok, "E4 second mount");
    auto* first = world.instanceAt(0);
    auto* second = world.instanceAt(1);
    // The synthetic command below exercises routing, not stress generation.
    // Establish an accepted solve before submitting it through the stress gate.
    second->blast.solver->update();
    expect(second->blast.solver->converged(), "E4 command fixture has accepted solve");
    const auto& bond = second->graph.bonds.front();
    blast::FractureCandidate candidate;
    candidate.stableId = bond.stableId;
    candidate.nodeA = bond.nodeA;
    candidate.nodeB = bond.nodeB;
    candidate.node0 = second->blast.graphFromStable.at(bond.nodeA);
    candidate.node1 = second->blast.graphFromStable.at(bond.nodeB);
    candidate.owner = 1;
    candidate.healthBefore = blast::bondAeff(bond, second->grid.voxelSize);
    candidate.damage = candidate.healthBefore;
    candidate.faces = bond.faces;
    auto& pending = second->pending;
    pending.valid = true;
    pending.objectId = child;
    pending.topologyRevision = second->topologyRevision;
    pending.solverTopologyEpoch = second->blast.solver->topologyEpoch();
    pending.solveEpoch = second->solveEpoch;
    pending.strengthEpoch = second->strengthEpoch;
    pending.strengthPa = second->material.strengthPa;
    pending.candidates.push_back(candidate);
    expect(world.pendingSnapshotMatches(pending, second), "E4 second pending snapshot matches");
    expect(!world.pendingSnapshotMatches(pending, first), "E4 first rejects second snapshot");
    expect(world.applyPendingIfAny(second) == 1, "E4 second commits its fracture");
    expect(NvBlastFamilyGetActorCount(second->blast.family, blastLog) == 2, "E4 second family split");
    expect(NvBlastFamilyGetActorCount(first->blast.family, blastLog) == 1, "E4 first family untouched");
    expect(!world.takeOccupancyDirty(first), "E4 first dirty flag untouched");
    expect(world.takeOccupancyDirty(second), "E4 second dirty flag delivered");
    std::vector<blast::ActorObjectLink> secondLinks;
    for (const auto& b : second->bindings) secondLinks.push_back({b.actor, child, glm::ivec3(0), 8});
    world.bindVisibleActors(secondLinks, second);
    expect(first->bindings.front().objectId == parent, "E4 second rebind leaves first alone");
    world.onPhysicsTick(1, 1.0f / 60.0f);
    expect(first->solveEpoch == 1 && second->solveEpoch == 1, "E4 both instances still solve");
  }
  expect(runtime.liveBytes() == baseline, "E4 world teardown releases rebuilt families");
  expect(runtime.errorCount() == 0, "E4 world no SDK errors");
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
  testRebuildKeepsAnchorAndCut(alloc);

  expect(errors.errorCount() == 0, "no NvBlast error-callback errors");
  {
    blast::BlastRuntime runtime;
    expect(runtime.init(), "E4 runtime init");
    testWorldRemovalAndSecondInstance(runtime);
  }
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);
  if (gFailures == 0) {
    std::cout << "OK P4 T14 T15 T16 T17 E4\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
