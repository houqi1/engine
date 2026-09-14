#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/HardFracture.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include "NvBlast.h"

#include <algorithm>
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

void blastLog(int type, const char* msg, const char* file, int line) {
  if (type <= NvBlastMessage::Error) {
    std::cerr << "NvBlastLL " << file << ":" << line << ": " << (msg ? msg : "") << "\n";
  }
}

class DenseCylinderView final : public blast::OccupancyView {
public:
  explicit DenseCylinderView(const blast::CylinderRaster& spec, bool cut)
      : spec_(spec), solid_(static_cast<size_t>(spec.nx * spec.ny * spec.nz), 0),
        anc_(static_cast<size_t>(spec.nx * spec.ny * spec.nz), 0) {
    blast::forEachCylinderFine(spec_, [&](int x, int y, int z) {
      if (cut && blast::inBaseCut270(x, y, z, spec_)) {
        return;
      }
      solid_[idx(x, y, z)] = 1;
      if (blast::isBaseAnchorFine(x, y, z, spec_)) {
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
    return x >= 0 && y >= 0 && z >= 0 && x < spec_.nx && y < spec_.ny && z < spec_.nz &&
           solid_[idx(x, y, z)] != 0;
  }
  bool anchor(int x, int y, int z) const override {
    return x >= 0 && y >= 0 && z >= 0 && x < spec_.nx && y < spec_.ny && z < spec_.nz &&
           anc_[idx(x, y, z)] != 0;
  }

private:
  size_t idx(int x, int y, int z) const {
    return static_cast<size_t>(x + spec_.nx * (y + spec_.ny * z));
  }
  blast::CylinderRaster spec_;
  std::vector<uint8_t> solid_;
  std::vector<uint8_t> anc_;
};

blast::OccupancySample build(bool cut) {
  blast::CylinderRaster spec{};
  DenseCylinderView view(spec, cut);
  blast::OccupancySampleOpts opts;
  opts.agg = blast::kE1Agg;
  return blast::sampleOccupancy(view, opts);
}

void drive(blast::StructureWorld& world, int ticks) {
  physics::FixedStepClock clock;
  for (int i = 0; i < ticks; ++i) {
    clock.advance(physics::kDt, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  }
}

bool mount(blast::StructureWorld& world, bool cut, uint32_t iters) {
  auto sample = build(cut);
  if (sample.error != blast::BlastError::Ok) {
    return false;
  }
  blast::CylinderRaster spec{};
  VoxelObjectId id{1, 1};
  return world.mountSample(id, std::move(sample), iters, blast::kE1StrengthHoldPa, blast::cylinderAxisX(spec),
                           blast::cylinderAxisZ(spec)) == blast::BlastError::Ok;
}

void testIntactHolds(blast::BlastRuntime& rt) {
  std::cout << "E2 intact holds at fail strength\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, false, 400), "mount intact");
  world.setStrengthPa(blast::kE2StrengthFailPa);
  world.setFractureEnabled(true);
  drive(world, 8);
  expect(world.debug().converged, "intact converged");
  expect(world.debug().maxTension < blast::kE2StrengthFailPa ||
             world.debug().maxCompression < blast::kE2StrengthFailPa * 2.0f,
         "intact stresses not far above fail S");
  expect(world.debug().candidateCount == 0, "intact has no fracture candidates");
  expect(world.debug().stripMaxStress < blast::kE2StrengthFailPa, "intact strip below fail S");
  std::cout << "  strip=" << world.debug().stripMaxStress << " maxT=" << world.debug().maxTension
            << " maxC=" << world.debug().maxCompression << " cand=" << world.debug().candidateCount << "\n";
  world.clear();
}

void testCutCandidatesAndSplit(blast::BlastRuntime& rt) {
  std::cout << "E2 cut produces strip candidates and split islands\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, true, 400), "mount cut");
  world.setStrengthPa(blast::kE2StrengthFailPa);
  world.setFractureEnabled(true);
  drive(world, 8);
  std::cout << "  conv=" << world.debug().converged << " strip=" << world.debug().stripMaxStress
            << " cand=" << world.debug().candidateCount << " stripCand=" << world.debug().candidateInStrip << "\n";
  expect(world.debug().stripMaxStress > blast::kE2StrengthFailPa, "cut strip exceeds fail S");
  blast::StructureInstance* inst = world.instance();
  expect(inst != nullptr && inst->blast.actor != nullptr, "actor");
  std::vector<blast::OverstressHit> hits;
  if (world.debug().converged) {
    blast::collectOverstressed(inst->blast.actor, *inst->blast.solver, blast::kE2StrengthFailPa, blastLog, hits);
  } else {
    for (const auto& p : inst->probes) {
      if (!canTakeDamage(p.health)) {
        continue;
      }
      if (blast::probeMaxStress(p) > blast::kE2StrengthFailPa) {
        blast::OverstressHit h;
        h.blastBondIndex = p.blastBondIndex;
        h.node0 = p.node0;
        h.node1 = p.node1;
        h.stress = blast::probeMaxStress(p);
        hits.push_back(h);
      }
    }
  }
  expect(!hits.empty(), "overstressed bonds exist on the cut cylinder");
  uint32_t stripHits = 0;
  for (const auto& h : hits) {
    for (const auto& b : inst->graph.bonds) {
      const auto it = inst->blast.sdkBondFromStable.find(b.stableId);
      if (it != inst->blast.sdkBondFromStable.end() && it->second == h.blastBondIndex && !b.world) {
        const float az = std::atan2(b.cz - inst->axisZ, b.cx - inst->axisX);
        if (az >= inst->keepAz0 && az < inst->keepAz1) {
          ++stripHits;
        }
      }
    }
  }
  std::cout << "  hits=" << hits.size() << " inStrip=" << stripHits << "\n";
  expect(stripHits > 0, "some candidates lie on the remaining strip");

  std::vector<uint32_t> sdk;
  sdk.reserve(hits.size());
  for (const auto& h : hits) {
    sdk.push_back(h.blastBondIndex);
  }
  const uint32_t occupied = blast::countSolidVoxels(inst->grid);
  blast::applySdkBondDamage(inst->blast.actor, inst->blast.asset, sdk.data(), static_cast<uint32_t>(sdk.size()),
                            blastLog);
  inst->blast.solver->syncBrokenBonds();
  const uint32_t actors =
      blast::splitIfNeeded(inst->blast.actor, inst->blast.family, *inst->blast.solver, blastLog);
  std::cout << "  actors after split=" << actors << " occupied=" << occupied << "\n";
  expect(actors >= 2, "split produced at least two actors");
  bool sawAnchor = false;
  bool sawFree = false;
  uint32_t visFines = 0;
  const NvBlastChunk* chunks = NvBlastAssetGetChunks(inst->blast.asset, blastLog);
  const uint32_t nA = NvBlastFamilyGetActorCount(inst->blast.family, blastLog);
  std::vector<NvBlastActor*> list(nA, nullptr);
  NvBlastFamilyGetActors(list.data(), nA, inst->blast.family, blastLog);
  for (uint32_t i = 0; i < nA; ++i) {
    if (list[i] == nullptr) {
      continue;
    }
    const bool ext = NvBlastActorHasExternalBonds(list[i], blastLog);
    sawAnchor = sawAnchor || ext;
    sawFree = sawFree || !ext;
    const uint32_t nv = NvBlastActorGetVisibleChunkCount(list[i], blastLog);
    std::vector<uint32_t> vis(nv);
    NvBlastActorGetVisibleChunkIndices(vis.data(), nv, list[i], blastLog);
    for (uint32_t ci : vis) {
      const uint32_t sid = chunks[ci].userData;
      for (const auto& n : inst->graph.nodes) {
        if (n.stableId == sid) {
          visFines += static_cast<uint32_t>(n.voxels.size());
        }
      }
    }
  }
  expect(sawAnchor, "one actor keeps world anchor");
  expect(sawFree, "one actor has no ghost anchor");
  expect(visFines == occupied, "visible chunk voxels cover occupancy");
  world.clear();
}

void testHoldStrength(blast::BlastRuntime& rt) {
  std::cout << "E2 hold strength does not candidate the cut\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, true, 400), "mount cut");
  world.setStrengthPa(blast::kE2StrengthHoldPa);
  world.setFractureEnabled(true);
  drive(world, 4);
  expect(world.debug().stripMaxStress < blast::kE2StrengthHoldPa, "cut strip below hold S");
  expect(world.debug().candidateCount == 0, "hold strength yields no candidates");
  world.clear();
}

void testReset(blast::BlastRuntime& rt) {
  std::cout << "E2 reset releases instance\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  const std::size_t base = rt.liveBytes();
  expect(mount(world, true, 50), "mount");
  world.clear();
  expect(rt.liveBytes() <= base + 64, "live bytes near baseline");
}

}  // namespace

int main() {
  std::cout << "blast_e2_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  std::cout << "S_fail=" << blast::kE2StrengthFailPa << " S_hold=" << blast::kE2StrengthHoldPa << "\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  testIntactHolds(rt);
  testCutCandidatesAndSplit(rt);
  testHoldStrength(rt);
  testReset(rt);
  rt.shutdown();
  if (gFailures == 0) {
    std::cout << "OK E2 fracture candidates split islands\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
