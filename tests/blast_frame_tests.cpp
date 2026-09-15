#include "blast/FrameVoxels.h"
#include "blast/HardFracture.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include <cmath>
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

class DenseFrameView final : public blast::OccupancyView {
public:
  explicit DenseFrameView(bool cutThree)
      : solid_(static_cast<size_t>(spec_.nx * spec_.ny * spec_.nz), 0),
        anc_(static_cast<size_t>(spec_.nx * spec_.ny * spec_.nz), 0) {
    blast::forEachFrameFine(spec_, [&](int x, int y, int z) {
      if (cutThree && blast::inThreeColumnCut(x, y, z)) {
        return;
      }
      solid_[idx(x, y, z)] = 1;
      if (blast::isFrameAnchorFine(x, y, z)) {
        anc_[idx(x, y, z)] = 1;
      }
    });
  }
  int nx() const override { return spec_.nx; }
  int ny() const override { return spec_.ny; }
  int nz() const override { return spec_.nz; }
  float voxelSize() const override { return spec_.voxelSize; }
  float density() const override { return blast::kE1Density; }
  bool solid(int x, int y, int z) const override {
    if (x < 0 || y < 0 || z < 0 || x >= spec_.nx || y >= spec_.ny || z >= spec_.nz) {
      return false;
    }
    return solid_[idx(x, y, z)] != 0;
  }
  bool anchor(int x, int y, int z) const override {
    if (x < 0 || y < 0 || z < 0 || x >= spec_.nx || y >= spec_.ny || z >= spec_.nz) {
      return false;
    }
    return anc_[idx(x, y, z)] != 0;
  }
  uint32_t occupied() const {
    uint32_t n = 0;
    for (uint8_t v : solid_) {
      n += v != 0 ? 1u : 0u;
    }
    return n;
  }

private:
  size_t idx(int x, int y, int z) const {
    return static_cast<size_t>(x + spec_.nx * (y + spec_.ny * z));
  }
  blast::FrameRaster spec_{};
  std::vector<uint8_t> solid_;
  std::vector<uint8_t> anc_;
};

blast::OccupancySample build(bool cutThree) {
  DenseFrameView view(cutThree);
  blast::OccupancySampleOpts opts;
  opts.agg = blast::kFrameAgg;
  return blast::sampleOccupancy(view, opts);
}

void solveWorld(blast::StructureWorld& world, int ticks) {
  physics::FixedStepClock clock;
  for (int i = 0; i < ticks; ++i) {
    clock.advance(physics::kDt, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  }
}

void driveUntilConverged(blast::StructureWorld& world, int maxTicks) {
  for (int i = 0; i < maxTicks; ++i) {
    solveWorld(world, 1);
    if (world.debug().converged) {
      return;
    }
  }
}

bool mount(blast::StructureWorld& world, bool cutThree, uint32_t iters, float strengthPa) {
  auto sample = build(cutThree);
  if (sample.error != blast::BlastError::Ok) {
    std::cerr << sample.message << "\n";
    return false;
  }
  VoxelObjectId id{1, 1};
  if (world.mountSample(id, std::move(sample), iters, strengthPa, 0.0f, 0.0f) != blast::BlastError::Ok) {
    std::cerr << world.lastError() << "\n";
    return false;
  }
  blast::FrameRaster spec{};
  float x0, x1, z0, z1;
  blast::keepColumnWorldBox(spec, x0, x1, z0, z1);
  world.setKeepColumnBox(x0, x1, z0, z1);
  world.markCut(cutThree);
  return true;
}

void testRaster() {
  std::cout << "Frame raster four columns + roof\n";
  DenseFrameView intact(false);
  DenseFrameView cut(true);
  expect(intact.occupied() > 1000, "occupied frame voxels");
  expect(cut.occupied() < intact.occupied(), "cutting three columns removes voxels");
  expect(cut.occupied() > 500, "keep column and roof remain");
  auto s = build(false);
  std::cout << "  occupied=" << s.occupied << " nodes=" << s.graph.nodes.size()
            << " bonds=" << s.graph.bonds.size() << " world=" << s.worldBonds << "\n";
}

void testIntactHolds(blast::BlastRuntime& rt) {
  std::cout << "T01 intact four-column holds\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, false, 400, blast::kFrameStrengthHoldPa), "mount intact");
  driveUntilConverged(world, 16);
  expect(world.debug().converged, "intact converged");
  const float rel = std::abs(world.debug().reactionY - world.debug().weight) /
                    std::max(world.debug().weight, 1.0f);
  expect(rel < 1.0e-2f, "reaction matches weight");
  world.setStrengthPa(blast::kFrameStrengthFailPa);
  world.setFractureEnabled(true);
  solveWorld(world, 2);
  expect(world.debug().candidateCount == 0, "intact does not fracture at 1.0 MPa");
  expect(world.debug().stripMaxStress < blast::kFrameStrengthFailPa, "intact keep-column below fail S");
  std::cout << "  W=" << world.debug().weight << " Ry=" << world.debug().reactionY << " rel=" << rel
            << " keep-col=" << world.debug().stripMaxStress << " cand=" << world.debug().candidateCount
            << " maxTCS=" << world.debug().maxTension << "/" << world.debug().maxCompression << "/"
            << world.debug().maxShear << "\n";
  world.clear();
}

void testCutStressAndStrength(blast::BlastRuntime& rt) {
  std::cout << "T02 cut three columns\n";
  blast::StructureWorld hold;
  expect(hold.init(rt), "hold init");
  expect(mount(hold, false, 400, blast::kFrameStrengthHoldPa), "intact");
  driveUntilConverged(hold, 16);
  const float before = hold.debug().stripMaxStress;
  hold.clear();

  blast::StructureWorld cutHold;
  expect(cutHold.init(rt), "cut hold init");
  expect(mount(cutHold, true, 400, blast::kFrameStrengthHoldPa), "cut hold");
  driveUntilConverged(cutHold, 24);
  std::cout << "  cut conv=" << cutHold.debug().converged << " lin=" << cutHold.debug().linErr
            << " keep-col=" << cutHold.debug().stripMaxStress << "\n";
  expect(cutHold.debug().converged, "cut converged");
  const float after = cutHold.debug().stripMaxStress;
  expect(after > before, "remaining column stress rises");
  {
    blast::StructureInstance* inst = cutHold.instance();
    const float th[] = {2.5e5f, 5.0e5f, 1.0e6f, 1.5e6f, 2.0e6f, 2.5e6f};
    std::cout << "  bond counts above S (keep-col vs other):\n";
    for (float s0 : th) {
      uint32_t keep = 0;
      uint32_t other = 0;
      uint32_t n = inst != nullptr ? inst->probeCount : 0;
      for (uint32_t i = 0; i < n; ++i) {
        const auto& p = inst->probes[i];
        if (!canTakeDamage(p.health)) {
          continue;
        }
        if (!(blast::probeMaxStress(p) > s0)) {
          continue;
        }
        bool inKeep = false;
        if (p.blastBondIndex < inst->bondMeta.size()) {
          inKeep = inst->bondMeta[p.blastBondIndex].inStrip != 0;
        }
        if (inKeep) {
          ++keep;
        } else {
          ++other;
        }
      }
      std::cout << "    S=" << s0 << " Pa  keep-col=" << keep << " other=" << other << "\n";
    }
  }
  cutHold.setFractureEnabled(true);
  solveWorld(cutHold, 2);
  expect(cutHold.debug().candidateCount == 0, "high strength does not break");
  std::cout << "  keep-col before=" << before << " after=" << after << "\n";
  cutHold.clear();

  blast::StructureWorld cutFail;
  expect(cutFail.init(rt), "cut fail init");
  expect(mount(cutFail, true, 400, blast::kFrameStrengthHoldPa), "cut fail");
  cutFail.setStrengthPa(blast::kFrameStrengthFailPa);
  cutFail.setFractureEnabled(true);
  driveUntilConverged(cutFail, 24);
  expect(cutFail.debug().converged, "cut-fail converged");
  solveWorld(cutFail, 1);
  const uint32_t cand = cutFail.debug().candidateCount;
  const uint32_t nfrac = cutFail.applyPendingCandidates(cutFail.pendingFracture());
  const uint32_t actors = cutFail.splitAllRequired();
  expect(nfrac > 0, "low strength breaks remaining path");
  expect(actors > 1, "hot slice splits the remaining column");
  std::cout << "  fail cand=" << cand << " fractured=" << nfrac << " actors=" << actors << "\n";
  cutFail.clear();
}

void testConvergeGate(blast::BlastRuntime& rt) {
  std::cout << "T06 converge gate\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, true, 1, blast::kFrameStrengthHoldPa), "mount 1 iter");
  world.setStrengthPa(blast::kFrameStrengthFailPa);
  world.setFractureEnabled(true);
  solveWorld(world, 1);
  if (!world.debug().converged) {
    expect(world.debug().candidateCount == 0, "no fracture when not converged");
  } else {
    std::cout << "  1 iter unexpectedly converged; skip no-fracture clause\n";
  }
  world.setSolverIters(400);
  driveUntilConverged(world, 24);
  expect(world.debug().converged, "recovered budget converges");
  solveWorld(world, 1);
  expect(world.debug().candidateCount > 0, "fracture resumes after convergence");
  world.clear();
}

void testFreeBody(blast::BlastRuntime& rt) {
  std::cout << "T05 free body (no anchors sampled)\n";
  blast::FrameRaster spec{};
  std::vector<uint8_t> solid(static_cast<size_t>(spec.nx * spec.ny * spec.nz), 0);
  blast::forEachFrameFine(spec, [&](int x, int y, int z) {
    solid[static_cast<size_t>(x + spec.nx * (y + spec.ny * z))] = 1;
  });
  class NoAnchorView final : public blast::OccupancyView {
  public:
    explicit NoAnchorView(const std::vector<uint8_t>& s) : s_(s) {}
    int nx() const override { return blast::kFrameGridFines; }
    int ny() const override { return blast::kFrameGridFines; }
    int nz() const override { return blast::kFrameGridFines; }
    float voxelSize() const override { return blast::kE1FineMeters; }
    float density() const override { return blast::kE1Density; }
    bool solid(int x, int y, int z) const override {
      return s_[static_cast<size_t>(x + nx() * (y + ny() * z))] != 0;
    }
    bool anchor(int, int, int) const override { return false; }

  private:
    const std::vector<uint8_t>& s_;
  };
  NoAnchorView view(solid);
  blast::OccupancySampleOpts opts;
  opts.agg = blast::kFrameAgg;
  auto sample = blast::sampleOccupancy(view, opts);
  std::cout << "  unanchored err=" << static_cast<uint32_t>(sample.error) << " world=" << sample.worldBonds
            << " " << sample.message << "\n";
  expect(sample.error != blast::BlastError::Ok, "unanchored occupancy is rejected");
  (void)rt;
}

}  // namespace

int main() {
  std::cout << "blast_frame_tests four-column roof\n";
  blast::BlastRuntime rt;
  if (!rt.init()) {
    std::cerr << "FAIL: BlastRuntime init\n";
    return 1;
  }
  testRaster();
  testIntactHolds(rt);
  testCutStressAndStrength(rt);
  testConvergeGate(rt);
  testFreeBody(rt);
  rt.shutdown();
  if (gFailures != 0) {
    std::cerr << "FAILED " << gFailures << "\n";
    return 1;
  }
  std::cout << "OK four-column T01/T02/T05/T06\n";
  return 0;
}
