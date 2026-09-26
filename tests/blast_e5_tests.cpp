// E5.0 multi-instance StructureWorld and E5.1 generic object mount.
#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/FrameVoxels.h"
#include "blast/GroundAnchors.h"
#include "blast/ObjectOccupancyView.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "blast/VoxelGraph.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAIL: " << msg << "\n";
    ++gFailures;
  }
}

bool sameBits(float a, float b) { return std::memcmp(&a, &b, sizeof(float)) == 0; }

// Engine voxel objects: 1.6 m coarse cells, 16 fines per cell.
constexpr float kCoarseMeters = 1.6f;
constexpr int kFinesPerCoarse = 16;

class DenseBits {
public:
  explicit DenseBits(int n) : n_(n), bits_(static_cast<size_t>(n) * n * n, 0) {}
  void set(int x, int y, int z, bool v) { bits_[index(x, y, z)] = v ? 1 : 0; }
  bool get(int x, int y, int z) const {
    return x >= 0 && y >= 0 && z >= 0 && x < n_ && y < n_ && z < n_ && bits_[index(x, y, z)] != 0;
  }
  int n() const { return n_; }

private:
  size_t index(int x, int y, int z) const { return static_cast<size_t>(x + n_ * (y + n_ * z)); }
  int n_;
  std::vector<uint8_t> bits_;
};

// Verbatim semantics of the scene's former dedicated CylinderObjectView / FrameObjectView.
class LegacySceneView final : public blast::OccupancyView {
public:
  enum class Kind { Cylinder, Frame };
  LegacySceneView(Kind kind, const DenseBits& occ) : kind_(kind), occ_(occ) {}
  int nx() const override { return kind_ == Kind::Cylinder ? spec_.nx : blast::kFrameGridFines; }
  int ny() const override { return kind_ == Kind::Cylinder ? spec_.ny : blast::kFrameGridFines; }
  int nz() const override { return kind_ == Kind::Cylinder ? spec_.nz : blast::kFrameGridFines; }
  float voxelSize() const override { return kind_ == Kind::Cylinder ? spec_.voxelSize : blast::kE1FineMeters; }
  float density() const override { return blast::kE1Density; }
  bool solid(int x, int y, int z) const override { return occ_.get(x, y, z); }
  bool anchor(int x, int y, int z) const override {
    return solid(x, y, z) &&
           (kind_ == Kind::Cylinder ? blast::isBaseAnchorFine(x, y, z, spec_) : blast::isFrameAnchorFine(x, y, z));
  }

private:
  Kind kind_;
  const DenseBits& occ_;
  blast::CylinderRaster spec_{};
};

DenseBits cylinderOccupancy(bool cut) {
  const blast::CylinderRaster spec{};
  DenseBits occ(spec.nx);
  blast::forEachCylinderFine(spec, [&](int x, int y, int z) {
    if (!(cut && blast::inBaseCut270(x, y, z, spec))) {
      occ.set(x, y, z, true);
    }
  });
  return occ;
}

DenseBits frameOccupancy(int columnHeight, bool cutThree) {
  blast::FrameRaster spec{};
  spec.columnHeight = columnHeight;
  DenseBits occ(spec.nx);
  blast::forEachFrameFine(spec, [&](int x, int y, int z) {
    if (!(cutThree && blast::inThreeColumnCut(x, y, z, spec.columnHeight))) {
      occ.set(x, y, z, true);
    }
  });
  return occ;
}

// Same construction as VoxelScene::mountObjectStructure: whole object grid, object density.
blast::OccupancySample genericSample(const DenseBits& occ, float density, int agg,
                                     const blast::ObjectOccupancyView::FinePredicate& anchor) {
  const int gridSize = occ.n() / kFinesPerCoarse;
  const blast::ObjectOccupancyView view(
      gridSize * kFinesPerCoarse, kCoarseMeters / static_cast<float>(kFinesPerCoarse), density,
      [&occ](int x, int y, int z) { return occ.get(x, y, z); }, anchor);
  blast::OccupancySampleOpts opts;
  opts.agg = agg;
  return blast::sampleOccupancy(view, opts);
}

blast::OccupancySample legacySample(LegacySceneView::Kind kind, const DenseBits& occ, int agg) {
  const LegacySceneView view(kind, occ);
  blast::OccupancySampleOpts opts;
  opts.agg = agg;
  return blast::sampleOccupancy(view, opts);
}

void compareSamples(const blast::OccupancySample& a, const blast::OccupancySample& b, const std::string& tag) {
  expect(a.error == blast::BlastError::Ok && b.error == blast::BlastError::Ok, tag + " both samples valid");
  expect(a.occupied == b.occupied, tag + " occupied");
  expect(a.worldBonds == b.worldBonds, tag + " world bonds");
  expect(sameBits(a.mass, b.mass), tag + " mass");
  expect(a.grid.nx == b.grid.nx && a.grid.ny == b.grid.ny && a.grid.nz == b.grid.nz && a.grid.agg == b.grid.agg,
         tag + " grid dims");
  expect(sameBits(a.grid.voxelSize, b.grid.voxelSize) && sameBits(a.grid.density, b.grid.density) &&
             sameBits(a.grid.ox, b.grid.ox) && sameBits(a.grid.oy, b.grid.oy) && sameBits(a.grid.oz, b.grid.oz),
         tag + " grid scale / origin");
  expect(a.grid.solid == b.grid.solid && a.grid.anchor == b.grid.anchor, tag + " solid and anchor masks");
  expect(a.graph.nodes.size() == b.graph.nodes.size(), tag + " node count");
  expect(a.graph.bonds.size() == b.graph.bonds.size(), tag + " bond count");
  if (a.graph.nodes.size() != b.graph.nodes.size() || a.graph.bonds.size() != b.graph.bonds.size()) {
    return;
  }
  uint32_t nodeDiffs = 0;
  for (size_t i = 0; i < a.graph.nodes.size(); ++i) {
    const blast::GraphNode& n = a.graph.nodes[i];
    const blast::GraphNode& m = b.graph.nodes[i];
    const bool same = n.stableId == m.stableId && n.voxels == m.voxels && sameBits(n.cx, m.cx) &&
                      sameBits(n.cy, m.cy) && sameBits(n.cz, m.cz) && sameBits(n.volume, m.volume) &&
                      sameBits(n.mass, m.mass);
    nodeDiffs += same ? 0u : 1u;
  }
  uint32_t bondDiffs = 0;
  for (size_t i = 0; i < a.graph.bonds.size(); ++i) {
    const blast::GraphBond& p = a.graph.bonds[i];
    const blast::GraphBond& q = b.graph.bonds[i];
    const bool same = p.stableId == q.stableId && p.nodeA == q.nodeA && p.nodeB == q.nodeB && p.world == q.world &&
                      p.nFaces == q.nFaces && p.faces == q.faces && sameBits(p.d, q.d) && sameBits(p.nx, q.nx) &&
                      sameBits(p.ny, q.ny) && sameBits(p.nz, q.nz) && sameBits(p.cx, q.cx) &&
                      sameBits(p.cy, q.cy) && sameBits(p.cz, q.cz);
    bondDiffs += same ? 0u : 1u;
  }
  expect(nodeDiffs == 0, tag + " nodes bit-identical (" + std::to_string(nodeDiffs) + " differ)");
  expect(bondDiffs == 0, tag + " bonds bit-identical (" + std::to_string(bondDiffs) + " differ)");
  std::cout << "  " << tag << ": nodes=" << a.graph.nodes.size() << " bonds=" << a.graph.bonds.size()
            << " world=" << a.worldBonds << " identical\n";
}

void compareSolves(const blast::StructureDebugSnapshot& a, const blast::StructureDebugSnapshot& b,
                   const std::string& tag) {
  expect(a.converged == b.converged, tag + " converged");
  expect(a.nodes == b.nodes && a.bonds == b.bonds && a.worldBonds == b.worldBonds, tag + " counts");
  expect(a.solverIters == b.solverIters, tag + " solver iterations");
  expect(sameBits(a.strengthPa, b.strengthPa) && a.fractureEnabled == b.fractureEnabled, tag + " material");
  expect(sameBits(a.reactionY, b.reactionY), tag + " reaction");
  expect(sameBits(a.maxTension, b.maxTension) && sameBits(a.maxCompression, b.maxCompression) &&
             sameBits(a.maxShear, b.maxShear),
         tag + " max stresses");
  expect(sameBits(a.stripMaxStress, b.stripMaxStress), tag + " strip stress");
  expect(sameBits(a.linErr, b.linErr) && sameBits(a.angErr, b.angErr), tag + " residuals");
  expect(a.candidateCount == b.candidateCount && a.candidateInStrip == b.candidateInStrip, tag + " candidates");
  expect(a.cut == b.cut, tag + " cut flag");
}

// E5.1: the generic whole-object view reproduces the dedicated scene views exactly,
// and mount(desc) as built by the scene reproduces the legacy mountSample path.
void testGenericViewMatchesLegacy(blast::BlastRuntime& rt) {
  std::cout << "E5.1 generic object view == dedicated scene views\n";
  expect(sameBits(kCoarseMeters / static_cast<float>(kFinesPerCoarse), blast::kE1FineMeters),
         "1.6 m / 16 is exactly the 0.1 m fine size");

  const blast::CylinderRaster cyl{};
  for (const bool cut : {false, true}) {
    const std::string tag = cut ? "cylinder cut" : "cylinder intact";
    const DenseBits occ = cylinderOccupancy(cut);
    blast::OccupancySample legacy = legacySample(LegacySceneView::Kind::Cylinder, occ, blast::kE1Agg);
    blast::OccupancySample generic =
        genericSample(occ, blast::kE1Density, blast::kE1Agg,
                      [cyl](int x, int y, int z) { return blast::isBaseAnchorFine(x, y, z, cyl); });
    compareSamples(legacy, generic, tag);

    blast::StructureWorld a;
    blast::StructureWorld b;
    expect(a.init(rt) && b.init(rt), tag + " worlds");
    const VoxelObjectId id{1, 1};
    const uint32_t iters = cut ? 400u : 200u;
    expect(a.mountSample(id, std::move(legacy), iters, blast::kE1StrengthHoldPa, blast::cylinderAxisX(cyl),
                         blast::cylinderAxisZ(cyl)) == blast::BlastError::Ok,
           tag + " legacy mount");
    a.markCut(cut);
    blast::StructureMountDesc desc;
    desc.objectId = id;
    desc.material.strengthPa = blast::kE1StrengthHoldPa;
    desc.material.solverIters = iters;
    desc.diag.kind = blast::DiagnosticProfile::Kind::CylinderStrip;
    desc.diag.axisX = blast::cylinderAxisX(cyl);
    desc.diag.axisZ = blast::cylinderAxisZ(cyl);
    desc.diag.keepAz0 = blast::kE1KeepAz0;
    desc.diag.keepAz1 = blast::kE1KeepAz1;
    desc.diag.cutApplied = cut;
    desc.keepMaterialOfReplaced = true;
    blast::StructureHandle h;
    expect(b.mount(desc, std::move(generic), &h) == blast::BlastError::Ok && h.valid(), tag + " generic mount");
    a.warmupGravity(cut ? 24u : 8u);
    b.warmupGravity(cut ? 24u : 8u, b.find(h));
    compareSolves(a.debug(), b.debug(h.valid() ? b.find(h) : nullptr), tag + " warmup");
  }

  for (const bool cut : {false, true}) {
    const std::string tag = cut ? "frame cut" : "frame intact";
    const DenseBits occ = frameOccupancy(blast::kFrameColH, cut);
    blast::OccupancySample legacy = legacySample(LegacySceneView::Kind::Frame, occ, blast::kFrameAgg);
    blast::OccupancySample generic =
        genericSample(occ, blast::kE1Density, blast::kFrameAgg,
                      [](int x, int y, int z) { return blast::isFrameAnchorFine(x, y, z); });
    compareSamples(legacy, generic, tag);

    blast::StructureWorld a;
    blast::StructureWorld b;
    expect(a.init(rt) && b.init(rt), tag + " worlds");
    const VoxelObjectId id{2, 1};
    const blast::FrameRaster spec{};
    float x0, x1, z0, z1;
    blast::keepColumnWorldBox(spec, x0, x1, z0, z1);
    expect(a.mountSample(id, std::move(legacy), 400, blast::kFrameStrengthHoldPa, 0.0f, 0.0f) ==
               blast::BlastError::Ok,
           tag + " legacy mount");
    a.setKeepColumnBox(x0, x1, z0, z1);
    a.markCut(cut);
    blast::StructureMountDesc desc;
    desc.objectId = id;
    desc.material.strengthPa = blast::kFrameStrengthHoldPa;
    desc.material.solverIters = 400;
    desc.diag.kind = blast::DiagnosticProfile::Kind::ColumnBox;
    blast::keepColumnWorldBox(spec, desc.diag.keepX0, desc.diag.keepX1, desc.diag.keepZ0, desc.diag.keepZ1);
    desc.diag.cutApplied = cut;
    desc.keepMaterialOfReplaced = true;
    blast::StructureHandle h;
    expect(b.mount(desc, std::move(generic), &h) == blast::BlastError::Ok && h.valid(), tag + " generic mount");
    a.warmupGravity(cut ? 16u : 8u);
    b.warmupGravity(cut ? 16u : 8u, b.find(h));
    compareSolves(a.debug(), b.debug(h.valid() ? b.find(h) : nullptr), tag + " warmup");
    // Same fracture decision under fail strength after the warmup.
    a.setFractureEnabled(true);
    b.setFractureEnabled(true);
    a.setStrengthPa(blast::kFrameStrengthFailPa);
    b.setStrengthPa(blast::kFrameStrengthFailPa);
    for (uint64_t t = 1; t <= 4; ++t) {
      a.onPhysicsTick(t, 1.0f / 60.0f);
      b.onPhysicsTick(t, 1.0f / 60.0f);
    }
    compareSolves(a.debug(), b.debug(b.find(h)), tag + " fail strength");
  }
}

// Small anchored column: 1 x 7 x 1 fines, anchored at the bottom fine.
blast::OccupancySample column(float density = 1000.0f) {
  class Column final : public blast::OccupancyView {
  public:
    explicit Column(float rho) : rho_(rho) {}
    int nx() const override { return 1; }
    int ny() const override { return 7; }
    int nz() const override { return 1; }
    float density() const override { return rho_; }
    bool solid(int x, int y, int z) const override { return x == 0 && z == 0 && y >= 0 && y < 7; }
    bool anchor(int x, int y, int z) const override { return solid(x, y, z) && y == 0; }

  private:
    float rho_;
  } view(density);
  blast::OccupancySampleOpts opts;
  opts.agg = 1;
  return blast::sampleOccupancy(view, opts);
}

blast::StructureMountDesc descFor(VoxelObjectId id, float strengthPa, bool fracture, uint32_t iters = 50) {
  blast::StructureMountDesc desc;
  desc.objectId = id;
  desc.material.strengthPa = strengthPa;
  desc.material.fractureEnabled = fracture;
  desc.material.solverIters = iters;
  return desc;
}

float totalMass(const blast::StructureInstance& inst) {
  float m = 0.0f;
  for (const blast::GraphNode& n : inst.graph.nodes) m += n.mass;
  return m;
}

void testMultiInstanceIsolation(blast::BlastRuntime& rt) {
  std::cout << "E5.0 multi-instance isolation\n";
  const auto baseline = rt.liveBytes();
  {
    blast::StructureWorld world;
    expect(world.init(rt), "init");
    const VoxelObjectId a{20, 1}, b{21, 1}, c{22, 1};

    blast::StructureHandle ha, hb;
    expect(world.mount(descFor(a, 1.0e6f, true, 60), column(1000.0f), &ha) == blast::BlastError::Ok, "mount A");
    blast::StructureInstance* instA = world.find(ha);
    expect(instA != nullptr && world.find(a) == instA, "A by handle and object");
    // A second, unrelated structure never copies A's material.
    expect(world.mount(descFor(b, 5.0e7f, false, 90), column(600.0f), &hb) == blast::BlastError::Ok, "mount B");
    blast::StructureInstance* instB = world.find(hb);
    expect(ha.valid() && hb.valid() && ha != hb, "distinct handles");
    expect(instB != nullptr && instB->material.strengthPa == 5.0e7f && !instB->material.fractureEnabled &&
               instB->material.solverIters == 90,
           "B keeps its own material");
    expect(instA->material.strengthPa == 1.0e6f && instA->material.fractureEnabled &&
               instA->material.solverIters == 60,
           "A unchanged by B");
    expect(instB->material.baseDensity == 600.0f && instA->material.baseDensity == 1000.0f,
           "base density comes from each sample");
    // Legacy mountSample of a different object also no longer inherits the first instance.
    expect(world.mountSample(c, column(), 70, 2.0e7f, 0.0f, 0.0f) == blast::BlastError::Ok, "legacy mount C");
    const blast::StructureInstance* instC = world.find(c);
    expect(instC != nullptr && instC->material.strengthPa == 2.0e7f && !instC->material.fractureEnabled,
           "legacy mount of another object uses its own strength");

    // Instance addresses stay valid while more structures are mounted.
    for (uint32_t i = 0; i < 12; ++i) {
      expect(world.mount(descFor({static_cast<uint32_t>(100 + i), 1}, 1.0e7f, false), column()) ==
                 blast::BlastError::Ok,
             "mount filler");
    }
    expect(world.find(ha) == instA && instA->objectId == a, "A pointer stable across mounts");

    // Targeted setters touch only their instance.
    const float massA = totalMass(*instA);
    const float massB = totalMass(*instB);
    expect(world.setDensityScale(2.0f, instB), "scale B density");
    expect(instB->grid.density == 1200.0f && std::abs(totalMass(*instB) - 2.0f * massB) < 1.0e-4f * massB,
           "B density doubles from its own 600 base");
    expect(totalMass(*instA) == massA && instA->grid.density == 1000.0f, "A density untouched");
    world.setSolverIters(33, instB);
    expect(instB->material.solverIters == 33 && instA->material.solverIters == 60, "solver iterations per instance");
    world.markCut(true, instB);
    expect(instB->diag.cutApplied && !instA->diag.cutApplied, "cut flag per instance");
    expect(world.warmupGravity(4, instB) > 0 && instB->solveEpoch > 0 && instA->solveEpoch == 0,
           "warmup solves only its instance");
    expect(&world.debug(b) == &instB->debug && world.debug(VoxelObjectId{77, 1}).hasInstance == false,
           "debug by object, idle when unmounted");

    // Remounting the same object keeps its current material and strength epoch.
    const uint64_t epochA = instA->strengthEpoch;
    blast::StructureHandle ha2;
    blast::StructureMountDesc again = descFor(a, 9.0e9f, false, 60);
    again.keepMaterialOfReplaced = true;
    expect(world.mount(again, column(), &ha2) == blast::BlastError::Ok, "remount A");
    expect(world.find(ha) == nullptr, "old A handle is stale after remount");
    const blast::StructureInstance* instA2 = world.find(ha2);
    expect(instA2 != nullptr && instA2->material.strengthPa == 1.0e6f && instA2->material.fractureEnabled &&
               instA2->strengthEpoch == epochA,
           "remount keeps A's material");
    blast::StructureHandle ha3;
    expect(world.mount(descFor(a, 3.0e6f, false), column(), &ha3) == blast::BlastError::Ok, "fresh remount A");
    expect(world.find(ha3)->material.strengthPa == 3.0e6f, "remount without keep uses the new material");
    expect(world.find(b) == instB, "B survives A's remounts");

    world.unmount(b);
    expect(world.find(hb) == nullptr && world.find(b) == nullptr && !world.ownsObject(b), "unmount B");
  }
  expect(rt.liveBytes() == baseline, "multi-instance world releases all families");
}

void testRebuildHandlesAndUnmountBinding(blast::BlastRuntime& rt) {
  std::cout << "E5.0 rebuild keeps the root handle; unmount detaches fragment bindings\n";
  const auto baseline = rt.liveBytes();
  {
    blast::StructureWorld world;
    expect(world.init(rt), "init");
    const VoxelObjectId other{30, 1}, parent{31, 1}, child{32, 1};
    blast::StructureHandle hOther, hParent;
    expect(world.mount(descFor(other, 4.0e6f, true), column(), &hOther) == blast::BlastError::Ok, "mount other");
    expect(world.mount(descFor(parent, 1.0e7f, false), column(), &hParent) == blast::BlastError::Ok, "mount parent");
    blast::StructureInstance* otherInst = world.find(hOther);

    // Cutting the bridge fine rebuilds the parent; the other structure is untouched.
    const glm::ivec3 bridge(0, 3, 0);
    expect(world.applyOccupancyRemoval(parent, &bridge, 1) == blast::BlastError::Ok, "remove bridge");
    blast::StructureInstance* parentInst = world.find(hParent);
    expect(parentInst != nullptr && parentInst->objectId == parent, "root piece keeps the parent handle");
    expect(world.find(hOther) == otherInst && otherInst->material.strengthPa == 4.0e6f &&
               otherInst->material.fractureEnabled,
           "other instance unchanged by the rebuild");
    expect(parentInst != nullptr && parentInst->material.strengthPa == 1.0e7f, "rebuilt piece keeps its material");
    if (parentInst == nullptr) return;
    expect(parentInst->bindings.size() == 2, "bridge cut yields two actors");

    std::vector<blast::ActorObjectLink> links;
    for (const blast::ActorBinding& bnd : parentInst->bindings) {
      links.push_back({bnd.actor, bnd.anchored ? parent : child, bnd.anchored ? glm::ivec3(0) : glm::ivec3(0, 4, 0), 8});
    }
    world.bindVisibleActors(links, parentInst);
    expect(world.ownsObject(child), "fragment bound");
    // Editing the fragment splits it into its own instance with a fresh handle.
    const glm::ivec3 tip(0, 2, 0);
    expect(world.applyOccupancyRemoval(child, &tip, 1) == blast::BlastError::Ok, "edit fragment");
    const blast::StructureInstance* childInst = world.find(child);
    expect(childInst != nullptr && childInst->handle.valid() && childInst->handle != hParent &&
               childInst->handle != hOther,
           "fragment instance gets a new handle");
    expect(world.find(hParent) != nullptr && world.find(hParent)->objectId == parent, "parent handle still resolves");

    // Freeing the fragment object releases its instance only.
    world.unmount(child);
    expect(!world.ownsObject(child) && world.find(child) == nullptr, "fragment unmounted");
    expect(world.find(hParent) != nullptr && world.find(hOther) != nullptr, "parent and other survive");

    // A fragment bound inside another instance is detached, not left dangling.
    const VoxelObjectId p2{33, 1}, frag{34, 1};
    blast::StructureHandle hP2;
    expect(world.mount(descFor(p2, 1.0e7f, false), column(), &hP2) == blast::BlastError::Ok, "mount p2");
    expect(world.applyOccupancyRemoval(p2, &bridge, 1) == blast::BlastError::Ok, "cut p2");
    blast::StructureInstance* p2Inst = world.find(hP2);
    std::vector<blast::ActorObjectLink> links2;
    for (const blast::ActorBinding& bnd : p2Inst->bindings) {
      links2.push_back({bnd.actor, bnd.anchored ? p2 : frag, bnd.anchored ? glm::ivec3(0) : glm::ivec3(0, 4, 0), 8});
    }
    world.bindVisibleActors(links2, p2Inst);
    expect(world.ownsObject(frag), "frag bound in p2");
    world.unmount(frag);
    expect(!world.ownsObject(frag) && world.find(hP2) == p2Inst, "unmount detaches frag binding, keeps p2");
    world.onPhysicsTick(1, 1.0f / 60.0f);
  }
  expect(rt.liveBytes() == baseline, "rebuild world releases all families");
  expect(rt.errorCount() == 0, "no SDK errors");
}

void testDiagnosticProfile(blast::BlastRuntime& rt) {
  std::cout << "E5.0 diagnostics only for regression profiles\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  const blast::CylinderRaster cyl{};
  const DenseBits occ = cylinderOccupancy(false);
  auto countStrip = [](const blast::StructureInstance& inst) {
    uint32_t n = 0;
    for (const blast::BondMeta& m : inst.bondMeta) n += m.inStrip;
    return n;
  };
  blast::StructureHandle plain, strip;
  auto anchor = [cyl](int x, int y, int z) { return blast::isBaseAnchorFine(x, y, z, cyl); };
  expect(world.mount(descFor({40, 1}, 5.0e7f, false),
                     genericSample(occ, blast::kE1Density, blast::kE1Agg, anchor), &plain) == blast::BlastError::Ok,
         "mount without profile");
  blast::StructureMountDesc desc = descFor({41, 1}, 5.0e7f, false);
  desc.diag.kind = blast::DiagnosticProfile::Kind::CylinderStrip;
  desc.diag.axisX = blast::cylinderAxisX(cyl);
  desc.diag.axisZ = blast::cylinderAxisZ(cyl);
  desc.diag.keepAz0 = blast::kE1KeepAz0;
  desc.diag.keepAz1 = blast::kE1KeepAz1;
  expect(world.mount(desc, genericSample(occ, blast::kE1Density, blast::kE1Agg, anchor), &strip) ==
             blast::BlastError::Ok,
         "mount with cylinder profile");
  expect(countStrip(*world.find(plain)) == 0, "no strip bonds without a profile");
  expect(countStrip(*world.find(strip)) > 0, "cylinder profile marks the strip");
  world.clear();
}

// E5.0 acceptance as worded: the cylinder and the four-column frame mounted at the same
// time, with different materials, evolve exactly as each does alone.
void testCylinderAndFrameTogether(blast::BlastRuntime& rt) {
  std::cout << "E5.0 cylinder and frame mounted together\n";
  const blast::CylinderRaster cyl{};
  const DenseBits cylOcc = cylinderOccupancy(false);
  const DenseBits frameOcc = frameOccupancy(blast::kFrameColH, true);
  auto cylSample = [&] {
    return genericSample(cylOcc, blast::kE1Density, blast::kE1Agg,
                         [cyl](int x, int y, int z) { return blast::isBaseAnchorFine(x, y, z, cyl); });
  };
  auto frameSample = [&] {
    return genericSample(frameOcc, blast::kE1Density, blast::kFrameAgg,
                         [](int x, int y, int z) { return blast::isFrameAnchorFine(x, y, z); });
  };
  const VoxelObjectId cylId{50, 1}, frameId{51, 1};
  const blast::StructureMountDesc cylDesc = descFor(cylId, blast::kE1StrengthHoldPa, false, 200);
  const blast::StructureMountDesc frameDesc = descFor(frameId, blast::kFrameStrengthFailPa, true, 400);
  constexpr uint64_t kTicks = 6;

  blast::StructureWorld solo;
  blast::StructureWorld pair;
  expect(solo.init(rt) && pair.init(rt), "worlds");
  // Solo runs, one structure at a time.
  blast::StructureHandle h;
  expect(solo.mount(cylDesc, cylSample(), &h) == blast::BlastError::Ok, "solo cylinder");
  for (uint64_t t = 1; t <= kTicks; ++t) solo.onPhysicsTick(t, 1.0f / 60.0f);
  const blast::StructureDebugSnapshot cylAlone = solo.debug(solo.find(h));
  solo.clear();
  expect(solo.mount(frameDesc, frameSample(), &h) == blast::BlastError::Ok, "solo frame");
  for (uint64_t t = 1; t <= kTicks; ++t) solo.onPhysicsTick(t, 1.0f / 60.0f);
  const blast::StructureDebugSnapshot frameAlone = solo.debug(solo.find(h));
  solo.clear();

  blast::StructureHandle hc, hf;
  expect(pair.mount(cylDesc, cylSample(), &hc) == blast::BlastError::Ok, "pair cylinder");
  expect(pair.mount(frameDesc, frameSample(), &hf) == blast::BlastError::Ok, "pair frame");
  expect(pair.instanceCount() == 2, "two instances");
  for (uint64_t t = 1; t <= kTicks; ++t) pair.onPhysicsTick(t, 1.0f / 60.0f);
  const blast::StructureInstance* c = pair.find(hc);
  const blast::StructureInstance* f = pair.find(hf);
  expect(c != nullptr && f != nullptr, "both resolve");
  if (c == nullptr || f == nullptr) return;
  expect(c->material.strengthPa == blast::kE1StrengthHoldPa && !c->material.fractureEnabled,
         "cylinder keeps hold strength, fracture off");
  expect(f->material.strengthPa == blast::kFrameStrengthFailPa && f->material.fractureEnabled,
         "frame keeps fail strength, fracture on");
  compareSolves(cylAlone, c->debug, "cylinder beside frame");
  compareSolves(frameAlone, f->debug, "frame beside cylinder");
  expect(c->debug.candidateCount == 0, "cylinder at hold strength has no candidates");
  expect(f->debug.candidateCount > 0, "cut frame at fail strength has candidates");
  std::cout << "  cylinder Ry=" << c->debug.reactionY << " cand=" << c->debug.candidateCount
            << " | frame cand=" << f->debug.candidateCount << "\n";
}

// Scene ground: 64 coarse cells of 1.6 m, 2 cells thick, grid centred at x=z=0, top at y=3.2 m.
constexpr int kGroundCoarse = 64;
constexpr int kGroundFines = kGroundCoarse * kFinesPerCoarse;
constexpr int kGroundTopFine = 2 * kFinesPerCoarse;
constexpr float kGroundTopMeters = 3.2f;

blast::AnchorSource sceneGround() {
  blast::AnchorSource g;
  g.frame.fineN = kGroundFines;
  g.frame.fineSize = kCoarseMeters / static_cast<float>(kFinesPerCoarse);
  g.frame.position = glm::vec3(0.0f, 0.5f * static_cast<float>(kGroundCoarse) * kCoarseMeters, 0.0f);
  g.solid = [](int, int y, int) { return y < kGroundTopFine; };
  return g;
}

// Grid of gridSize coarse cells whose bottom face rests on the ground (the demo spawn pose).
blast::AnchorGridFrame onGround(int gridSize, float lift = 0.0f) {
  blast::AnchorGridFrame f;
  f.fineN = gridSize * kFinesPerCoarse;
  f.fineSize = kCoarseMeters / static_cast<float>(kFinesPerCoarse);
  f.position = glm::vec3(0.0f, kGroundTopMeters + lift + 0.5f * static_cast<float>(gridSize) * kCoarseMeters, 0.0f);
  return f;
}

std::vector<uint32_t> worldBondNodes(const blast::OccupancySample& s) {
  std::vector<uint32_t> nodes;
  for (const blast::GraphBond& b : s.graph.bonds) {
    if (b.world) nodes.push_back(b.nodeA);
  }
  std::sort(nodes.begin(), nodes.end());
  return nodes;
}

float settledReaction(blast::StructureWorld& world, blast::StructureInstance* inst, bool& converged) {
  world.warmupGravity(96, inst);
  converged = inst->debug.converged;
  return inst->debug.reactionY;
}

// E5.2 acceptance on the regression geometry: ground-contact anchors mark only the
// contact layer, anchor exactly the same nodes as the explicit base rule, and give the
// same total reaction within kE1ReactionRelTol of the weight.
void testGroundAnchorsMatchExplicitNodes(blast::BlastRuntime& rt) {
  std::cout << "E5.2 ground-contact anchors vs explicit base rule\n";
  const blast::CylinderRaster cyl{};
  struct Case {
    std::string tag;
    DenseBits occ;
    int gridSize;
    int agg;
    std::function<bool(int, int, int)> explicitAnchor;
  };
  std::vector<Case> cases;
  cases.push_back({"cylinder", cylinderOccupancy(false), blast::kE1GridFines / kFinesPerCoarse, blast::kE1Agg,
                   [cyl](int x, int y, int z) { return blast::isBaseAnchorFine(x, y, z, cyl); }});
  cases.push_back({"frame", frameOccupancy(blast::kFrameColH, false), blast::kFrameGridFines / kFinesPerCoarse,
                   blast::kFrameAgg, [](int x, int y, int z) { return blast::isFrameAnchorFine(x, y, z); }});
  cases.push_back({"frame cut", frameOccupancy(blast::kFrameColH, true), blast::kFrameGridFines / kFinesPerCoarse,
                   blast::kFrameAgg, [](int x, int y, int z) { return blast::isFrameAnchorFine(x, y, z); }});
  const std::vector<blast::AnchorSource> sources{sceneGround()};
  for (Case& k : cases) {
    const DenseBits& occ = k.occ;
    const blast::GroundAnchorResult r =
        blast::findGroundAnchors(onGround(k.gridSize), [&occ](int x, int y, int z) { return occ.get(x, y, z); },
                                 sources);
    uint32_t mismatched = 0;
    uint32_t bottom = 0;
    for (int z = 0; z < occ.n(); ++z) {
      for (int y = 0; y < occ.n(); ++y) {
        for (int x = 0; x < occ.n(); ++x) {
          const bool expected = occ.get(x, y, z) && y == 0;
          bottom += expected ? 1u : 0u;
          mismatched += r.isAnchor(x, y, z) != expected ? 1u : 0u;
        }
      }
    }
    expect(mismatched == 0, k.tag + " anchors are exactly the ground contact layer");
    expect(r.anchorFines == bottom && r.contactFaces == bottom && r.blockedFaces == 0,
           k.tag + " one contact face per bottom fine");

    blast::OccupancySample expl = genericSample(occ, blast::kE1Density, k.agg, k.explicitAnchor);
    blast::OccupancySample autoS =
        genericSample(occ, blast::kE1Density, k.agg, [&r](int x, int y, int z) { return r.isAnchor(x, y, z); });
    expect(expl.error == blast::BlastError::Ok && autoS.error == blast::BlastError::Ok, k.tag + " samples");
    const size_t anchoredNodes = worldBondNodes(expl).size();
    expect(anchoredNodes > 0 && worldBondNodes(expl) == worldBondNodes(autoS), k.tag + " same anchored node set");
    uint32_t explFaces = 0, autoFaces = 0;
    for (const blast::GraphBond& b : expl.graph.bonds) explFaces += b.world ? static_cast<uint32_t>(b.nFaces) : 0u;
    for (const blast::GraphBond& b : autoS.graph.bonds) autoFaces += b.world ? static_cast<uint32_t>(b.nFaces) : 0u;
    expect(autoFaces == bottom && explFaces == 2 * bottom, k.tag + " anchor area = contact area (half the 2-fine rule)");

    blast::StructureWorld world;
    expect(world.init(rt), k.tag + " world");
    blast::StructureHandle he, ha;
    expect(world.mount(descFor({60, 1}, blast::kE1StrengthHoldPa, false, 400), std::move(expl), &he) ==
               blast::BlastError::Ok,
           k.tag + " mount explicit");
    expect(world.mount(descFor({61, 1}, blast::kE1StrengthHoldPa, false, 400), std::move(autoS), &ha) ==
               blast::BlastError::Ok,
           k.tag + " mount ground");
    bool convE = false, convA = false;
    const float re = settledReaction(world, world.find(he), convE);
    const float ra = settledReaction(world, world.find(ha), convA);
    const float weight = world.find(ha)->debug.weight;
    expect(convE && convA, k.tag + " both converge");
    const float relPair = std::abs(ra - re) / weight;
    const float relWeight = std::abs(ra - weight) / weight;
    expect(relPair <= blast::kE1ReactionRelTol, k.tag + " reaction matches explicit within tolerance");
    expect(relWeight <= blast::kE1ReactionRelTol, k.tag + " ground reaction balances the weight");
    std::cout << "  " << k.tag << ": anchors=" << r.anchorFines << " nodes=" << anchoredNodes
              << " W=" << weight << " Ry explicit=" << re << " ground=" << ra << " rel=" << relPair << "\n";
  }
}

// Small solid box (x,z in [4,12), y in [0,6)) in a one-coarse-cell grid.
bool boxSolid(int x, int y, int z) { return x >= 4 && x < 12 && z >= 4 && z < 12 && y >= 0 && y < 6; }

uint32_t anchorsWhere(const blast::GroundAnchorResult& r, const std::function<bool(int, int, int)>& pred) {
  uint32_t n = 0;
  for (int z = 0; z < r.fineN; ++z)
    for (int y = 0; y < r.fineN; ++y)
      for (int x = 0; x < r.fineN; ++x) n += (r.isAnchor(x, y, z) && pred(x, y, z)) ? 1u : 0u;
  return n;
}

void testGroundAnchorEdges() {
  std::cout << "E5.2 ground anchor edge cases\n";
  const std::vector<blast::AnchorSource> ground{sceneGround()};
  auto bottomLayer = [](int, int y, int) { return y == 0; };

  const auto resting = blast::findGroundAnchors(onGround(1), boxSolid, ground);
  expect(resting.anchorFines == 64 && resting.contactFaces == 64 && anchorsWhere(resting, bottomLayer) == 64,
         "resting box anchors its 8x8 footprint only");

  // Contact tolerance is half a fine: the probe is the neighbouring cell centre.
  expect(blast::findGroundAnchors(onGround(1, 0.04f), boxSolid, ground).anchorFines == 64,
         "0.04 m gap is still contact");
  expect(blast::findGroundAnchors(onGround(1, 0.06f), boxSolid, ground).anchorFines == 0,
         "0.06 m gap is not contact");
  const auto lifted = blast::findGroundAnchors(onGround(1, 0.1f), boxSolid, ground);
  expect(lifted.anchorFines == 0 && lifted.contactFaces == 0, "box one fine above ground is not anchored");

  blast::AnchorGridFrame yaw = onGround(1);
  yaw.rotation = glm::angleAxis(glm::radians(30.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const auto yawed = blast::findGroundAnchors(yaw, boxSolid, ground);
  expect(yawed.anchorFines == 64 && anchorsWhere(yawed, bottomLayer) == 64, "30 deg yaw keeps the footprint");

  // Tipped 90 deg about Z: local -X faces down, so the x=4 layer rests on the ground.
  blast::AnchorGridFrame tipped = onGround(1);
  tipped.rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
  tipped.position.y = kGroundTopMeters + 0.4f;
  const auto side = blast::findGroundAnchors(tipped, boxSolid, ground);
  expect(side.anchorFines == 48 && anchorsWhere(side, [](int x, int, int) { return x == 4; }) == 48,
         "box tipped on its side anchors the x=4 face");

  // Touching a mounted structure never anchors and is reported.
  blast::AnchorSource mountedSlab = sceneGround();
  mountedSlab.blocksMount = true;
  const auto onStructure = blast::findGroundAnchors(onGround(1), boxSolid, {mountedSlab});
  expect(onStructure.anchorFines == 0 && onStructure.blockedFaces == 64, "resting on a mounted structure is blocked");
  blast::AnchorSource halfSlab = sceneGround();
  halfSlab.blocksMount = true;
  halfSlab.solid = [](int x, int y, int) { return y < kGroundTopFine && x < kGroundFines / 2; };  // world x < 0
  const auto half = blast::findGroundAnchors(onGround(1), boxSolid, {sceneGround(), halfSlab});
  expect(half.blockedFaces == 32 && half.anchorFines == 32 &&
             anchorsWhere(half, [](int x, int, int) { return x >= 8; }) == 32,
         "half on a mounted structure: that half blocked, the other half anchored");

  blast::AnchorGridFrame far = onGround(1);
  far.position.x = 1000.0f;
  expect(blast::findGroundAnchors(far, boxSolid, ground).anchorFines == 0, "no source nearby, no anchors");
}

}  // namespace

int main() {
  std::cout << "blast_e5_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  testMultiInstanceIsolation(rt);
  testRebuildHandlesAndUnmountBinding(rt);
  testDiagnosticProfile(rt);
  testGenericViewMatchesLegacy(rt);
  testCylinderAndFrameTogether(rt);
  testGroundAnchorEdges();
  testGroundAnchorsMatchExplicitNodes(rt);
  expect(rt.errorCount() == 0, "no NvBlast errors");
  rt.shutdown();
  if (gFailures == 0) {
    std::cout << "OK E5.0 multi-instance, E5.1 generic object mount, E5.2 ground-contact anchors\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
