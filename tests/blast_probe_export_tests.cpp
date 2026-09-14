#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/HardFracture.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include "NvBlast.h"
#include "NvCTypes.h"

#include <cmath>
#include <cstdint>
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

void drive(blast::StructureWorld& world, int ticks) {
  physics::FixedStepClock clock;
  for (int i = 0; i < ticks; ++i) {
    clock.advance(physics::kDt, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  }
}

bool sameF(float a, float b) { return a == b || (std::isnan(a) && std::isnan(b)); }

bool sameV(const NvcVec3& a, const NvcVec3& b) { return sameF(a.x, b.x) && sameF(a.y, b.y) && sameF(a.z, b.z); }

void expectProbeEq(const Nv::Blast::ExtStressSolver::BondProbe& a, const Nv::Blast::ExtStressSolver::BondProbe& b,
                   const std::string& tag) {
  expect(a.blastBondIndex == b.blastBondIndex, tag + " idx");
  expect(a.node0 == b.node0, tag + " node0");
  expect(a.node1 == b.node1, tag + " node1");
  expect(sameF(a.health, b.health), tag + " health");
  expect(sameF(a.compression, b.compression), tag + " compression");
  expect(sameF(a.tension, b.tension), tag + " tension");
  expect(sameF(a.shear, b.shear), tag + " shear");
  expect(sameV(a.forceLinear, b.forceLinear), tag + " force");
  expect(sameV(a.moment, b.moment), tag + " moment");
}

void compareCopies(Nv::Blast::ExtStressSolver& solver, uint32_t nb, const char* label) {
  expect(solver.copyBondProbes(nullptr, nb) == 0, std::string(label) + " null out");
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> dummy(1);
  expect(solver.copyBondProbes(dummy.data(), 0) == 0, std::string(label) + " zero cap");

  std::vector<Nv::Blast::ExtStressSolver::BondProbe> neu(nb);
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> old(nb);
  const uint32_t nNew = solver.copyBondProbes(neu.data(), nb);
  const uint32_t nOld = solver.copyBondProbesLegacy(old.data(), nb);
  expect(nNew == nOld, std::string(label) + " count");
  expect(nNew == nb, std::string(label) + " full capacity");
  const uint32_t n = nNew < nOld ? nNew : nOld;
  uint32_t worldN = 0, deadN = 0;
  for (uint32_t i = 0; i < n; ++i) {
    expectProbeEq(neu[i], old[i], std::string(label) + " bond " + std::to_string(i));
    expect(neu[i].blastBondIndex == i, std::string(label) + " index order");
    if (!canTakeDamage(neu[i].health)) {
      ++worldN;
    }
    if (neu[i].health <= 0.0f) {
      ++deadN;
    }
  }
  (void)worldN;
  (void)deadN;

  const uint32_t cap = nb > 7 ? 7u : 1u;
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> neuCap(cap);
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> oldCap(cap);
  const uint32_t cNew = solver.copyBondProbes(neuCap.data(), cap);
  const uint32_t cOld = solver.copyBondProbesLegacy(oldCap.data(), cap);
  expect(cNew == cOld && cNew == cap, std::string(label) + " truncated count");
  for (uint32_t i = 0; i < cNew; ++i) {
    expectProbeEq(neuCap[i], oldCap[i], std::string(label) + " trunc " + std::to_string(i));
  }
}

void compareCandidates(blast::StructureInstance& inst, float strength, bool requireConv, const char* label) {
  std::vector<blast::OverstressHit> fromCopy;
  std::vector<blast::OverstressHit> fromBuf;
  blast::collectOverstressed(inst.blast.actor, *inst.blast.solver, strength, blastLog, fromCopy, requireConv);
  blast::collectOverstressedFromProbes(inst.blast.actor, inst.probes.data(), inst.probeCount, strength, blastLog,
                                       fromBuf, requireConv, inst.blast.solver);
  expect(fromCopy.size() == fromBuf.size(), std::string(label) + " candidate count");
  const size_t n = fromCopy.size() < fromBuf.size() ? fromCopy.size() : fromBuf.size();
  for (size_t i = 0; i < n; ++i) {
    expect(fromCopy[i].blastBondIndex == fromBuf[i].blastBondIndex, std::string(label) + " cand idx");
    expect(fromCopy[i].node0 == fromBuf[i].node0, std::string(label) + " cand n0");
    expect(fromCopy[i].node1 == fromBuf[i].node1, std::string(label) + " cand n1");
  }
}

void testIntact(blast::BlastRuntime& rt) {
  std::cout << "probe export intact\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, false, 200), "mount");
  world.setFractureEnabled(true);
  world.setStrengthPa(blast::kE2StrengthFailPa);
  drive(world, 4);
  expect(world.debug().probeExportCount == 1, "intact one export / tick");
  blast::StructureInstance* inst = world.instance();
  expect(inst != nullptr, "inst");
  const uint32_t nb = blast::familyAssetBondCount(inst->blast.actor, blastLog);
  compareCopies(*inst->blast.solver, nb, "intact");
  compareCandidates(*inst, blast::kE2StrengthFailPa, true, "intact hold-fail");
  world.setStrengthPa(blast::kE2StrengthHoldPa);
  drive(world, 1);
  compareCandidates(*inst, blast::kE2StrengthHoldPa, true, "intact hold");
  expect(world.debug().candidateCount == 0, "hold no candidates");
  world.clear();
}

void testCutAndDamage(blast::BlastRuntime& rt) {
  std::cout << "probe export cut / damaged / split\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, true, 400), "mount cut");
  world.setFractureEnabled(true);
  world.setStrengthPa(blast::kE2StrengthFailPa);
  drive(world, 8);
  expect(world.debug().probeExportCount == 1, "cut one export / tick");
  blast::StructureInstance* inst = world.instance();
  expect(inst != nullptr, "inst");
  const uint32_t nb = blast::familyAssetBondCount(inst->blast.actor, blastLog);
  compareCopies(*inst->blast.solver, nb, "cut");
  compareCandidates(*inst, blast::kE2StrengthFailPa, false, "cut fail");

  const float* healths = NvBlastActorGetBondHealths(inst->blast.actor, blastLog);
  NvBlastBondFractureData one{};
  bool haveOne = false;
  std::vector<NvBlastBondFractureData> all;
  for (uint32_t i = 0; i < inst->probeCount; ++i) {
    const auto& p = inst->probes[i];
    if (!canTakeDamage(p.health) || !(blast::probeMaxStress(p) > blast::kE2StrengthFailPa)) {
      continue;
    }
    NvBlastBondFractureData d{};
    d.nodeIndex0 = p.node0;
    d.nodeIndex1 = p.node1;
    d.health = healths != nullptr ? healths[p.blastBondIndex] : p.health;
    if (!haveOne) {
      one = d;
      haveOne = true;
    }
    all.push_back(d);
  }
  expect(haveOne && !all.empty(), "cut overstressed bonds");
  NvBlastFractureBuffers oneBuf{};
  oneBuf.bondFractureCount = 1;
  oneBuf.bondFractures = &one;
  NvBlastActorApplyFracture(nullptr, inst->blast.actor, &oneBuf, blastLog, nullptr);
  inst->blast.solver->syncBrokenBonds();
  expect(NvBlastFamilyGetActorCount(inst->blast.family, blastLog) == 1, "single bond break did not split");
  compareCopies(*inst->blast.solver, nb, "damaged no split");

  NvBlastFractureBuffers allBuf{};
  allBuf.bondFractureCount = static_cast<uint32_t>(all.size());
  allBuf.bondFractures = all.data();
  NvBlastActorApplyFracture(nullptr, inst->blast.actor, &allBuf, blastLog, nullptr);
  inst->blast.solver->syncBrokenBonds();
  const uint32_t actors = blast::splitIfNeeded(inst->blast.actor, inst->blast.family, *inst->blast.solver, blastLog);
  std::cout << "  actors after split=" << actors << " bonds=" << all.size() << "\n";
  expect(actors >= 2, "split");
  drive(world, 2);
  expect(world.debug().probeExportCount == 1, "split one export / tick");
  compareCopies(*inst->blast.solver, nb, "after split");
  world.clear();
}

}  // namespace

int main() {
  std::cout << "blast_probe_export_tests linear vs legacy copyBondProbes\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  testIntact(rt);
  testCutAndDamage(rt);
  rt.shutdown();
  if (gFailures != 0) {
    std::cerr << gFailures << " failure(s)\n";
    return 1;
  }
  std::cout << "OK probe export\n";
  return 0;
}
