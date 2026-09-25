// E5.0 multi-instance StructureWorld and E5.1 generic object mount.
#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/FrameVoxels.h"
#include "blast/ObjectOccupancyView.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "blast/VoxelGraph.h"

#include <cmath>
#include <cstring>
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

}  // namespace

int main() {
  std::cout << "blast_e5_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  testMultiInstanceIsolation(rt);
  testRebuildHandlesAndUnmountBinding(rt);
  testDiagnosticProfile(rt);
  testGenericViewMatchesLegacy(rt);
  expect(rt.errorCount() == 0, "no NvBlast errors");
  rt.shutdown();
  if (gFailures == 0) {
    std::cout << "OK E5.0 multi-instance, E5.1 generic object mount\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
