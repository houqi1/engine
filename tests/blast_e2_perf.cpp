#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/HardFracture.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include "NvBlast.h"
#include "NvCTypes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

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

struct Ms {
  double min = 0;
  double max = 0;
  double mean = 0;
  double median = 0;
  double p95 = 0;
};

Ms summarize(std::vector<double> v) {
  Ms out{};
  if (v.empty()) {
    return out;
  }
  std::sort(v.begin(), v.end());
  out.min = v.front();
  out.max = v.back();
  out.mean = std::accumulate(v.begin(), v.end(), 0.0) / static_cast<double>(v.size());
  const size_t n = v.size();
  out.median = 0.5 * (v[(n - 1) / 2] + v[n / 2]);
  out.p95 = v[static_cast<size_t>(std::ceil(0.95 * static_cast<double>(n))) - 1];
  return out;
}

double msSince(Clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

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

void addGravityAndUpdate(blast::StructureInstance& inst) {
  const uint32_t nA = NvBlastFamilyGetActorCount(inst.blast.family, blastLog);
  std::vector<NvBlastActor*> actors(nA, nullptr);
  NvBlastFamilyGetActors(actors.data(), nA, inst.blast.family, blastLog);
  for (uint32_t i = 0; i < nA; ++i) {
    if (actors[i] != nullptr) {
      inst.blast.solver->addGravity(*actors[i], NvcVec3{0.0f, blast::kE1GravityY, 0.0f});
    }
  }
  inst.blast.solver->update();
}

uint32_t copyProbes(blast::StructureInstance& inst) {
  const uint32_t nb = blast::familyAssetBondCount(inst.blast.actor, blastLog);
  if (inst.probes.size() < nb) {
    inst.probes.resize(nb);
  }
  return inst.blast.solver->copyBondProbes(inst.probes.data(), nb);
}

float bondAzimuth(const blast::GraphBond& b, float axisX, float axisZ) {
  return std::atan2(b.cz - axisZ, b.cx - axisX);
}

void currentNestedStats(blast::StructureInstance& inst, uint32_t n, float& maxT, float& maxC, float& maxS,
                        float& strip, float& ry) {
  maxT = maxC = maxS = strip = ry = 0.0f;
  std::unordered_map<uint32_t, bool> worldSdk;
  for (const blast::GraphBond& b : inst.graph.bonds) {
    if (!b.world) {
      continue;
    }
    const auto it = inst.blast.sdkBondFromStable.find(b.stableId);
    if (it != inst.blast.sdkBondFromStable.end()) {
      worldSdk[it->second] = true;
    }
  }
  for (uint32_t i = 0; i < n; ++i) {
    const auto& p = inst.probes[i];
    maxT = std::max(maxT, p.tension);
    maxC = std::max(maxC, p.compression);
    maxS = std::max(maxS, p.shear);
    if (worldSdk.count(p.blastBondIndex) != 0) {
      float fy = p.forceLinear.y;
      if (inst.blast.graphWorld != blast::kInvalidIndex && p.node0 == inst.blast.graphWorld) {
        fy = -fy;
      }
      ry += fy;
    }
  }
  for (const blast::GraphBond& b : inst.graph.bonds) {
    if (b.world) {
      continue;
    }
    const auto it = inst.blast.sdkBondFromStable.find(b.stableId);
    if (it == inst.blast.sdkBondFromStable.end()) {
      continue;
    }
    const uint32_t sdk = it->second;
    const Nv::Blast::ExtStressSolver::BondProbe* pr = nullptr;
    for (uint32_t i = 0; i < n; ++i) {
      if (inst.probes[i].blastBondIndex == sdk) {
        pr = &inst.probes[i];
        break;
      }
    }
    if (pr == nullptr) {
      continue;
    }
    const float az = bondAzimuth(b, inst.axisX, inst.axisZ);
    if (az >= inst.keepAz0 && az < inst.keepAz1) {
      strip = std::max(strip, blast::probeMaxStress(*pr));
    }
  }
}

void linearStats(blast::StructureInstance& inst, uint32_t n, float& maxT, float& maxC, float& maxS, float& strip,
                 float& ry) {
  maxT = maxC = maxS = strip = ry = 0.0f;
  std::unordered_map<uint32_t, uint8_t> flag;
  flag.reserve(inst.graph.bonds.size());
  for (const blast::GraphBond& b : inst.graph.bonds) {
    const auto it = inst.blast.sdkBondFromStable.find(b.stableId);
    if (it == inst.blast.sdkBondFromStable.end()) {
      continue;
    }
    uint8_t f = 0;
    if (b.world) {
      f = 1;
    } else if (bondAzimuth(b, inst.axisX, inst.axisZ) >= inst.keepAz0 &&
               bondAzimuth(b, inst.axisX, inst.axisZ) < inst.keepAz1) {
      f = 2;
    }
    flag[it->second] = f;
  }
  for (uint32_t i = 0; i < n; ++i) {
    const auto& p = inst.probes[i];
    maxT = std::max(maxT, p.tension);
    maxC = std::max(maxC, p.compression);
    maxS = std::max(maxS, p.shear);
    const auto it = flag.find(p.blastBondIndex);
    if (it == flag.end()) {
      continue;
    }
    if (it->second == 1) {
      float fy = p.forceLinear.y;
      if (inst.blast.graphWorld != blast::kInvalidIndex && p.node0 == inst.blast.graphWorld) {
        fy = -fy;
      }
      ry += fy;
    } else if (it->second == 2) {
      strip = std::max(strip, blast::probeMaxStress(p));
    }
  }
}

uint32_t extraCopyCandidates(blast::StructureInstance& inst, float strengthPa) {
  std::vector<blast::OverstressHit> hits;
  blast::collectOverstressed(inst.blast.actor, *inst.blast.solver, strengthPa, blastLog, hits, false);
  return static_cast<uint32_t>(hits.size());
}

uint32_t incrementalCandidates(blast::StructureInstance& inst, uint32_t n, float strengthPa) {
  uint32_t count = 0;
  for (uint32_t i = 0; i < n; ++i) {
    const auto& p = inst.probes[i];
    if (!canTakeDamage(p.health)) {
      continue;
    }
    if (blast::probeMaxStress(p) > strengthPa) {
      ++count;
    }
  }
  return count;
}

void printMs(const char* name, const Ms& s) {
  std::cout << "| " << std::left << std::setw(40) << name << " | " << std::right << std::setw(8)
            << std::fixed << std::setprecision(3) << s.median << " | " << std::setw(8) << s.p95 << " | "
            << std::setw(8) << s.max << " | " << std::setw(8) << s.mean << " | " << std::setw(8) << s.min
            << " |\n";
}

void runVariant(blast::BlastRuntime& rt, bool cut, uint32_t iters, int warmup, int samples) {
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(mount(world, cut, iters), cut ? "mount cut" : "mount intact");
  blast::StructureInstance* inst = world.instance();
  expect(inst != nullptr && inst->blast.solver != nullptr, "instance");
  if (inst == nullptr) {
    return;
  }
  world.setStrengthPa(blast::kE2StrengthFailPa);
  world.setFractureEnabled(true);

  for (int i = 0; i < warmup; ++i) {
    addGravityAndUpdate(*inst);
  }

  std::vector<double> aMs, probeMs, nestedMs, extraCopyMs, linearMs, incrCandMs, prodMs, prodProbe, prodStats,
      prodCand;
  aMs.reserve(static_cast<size_t>(samples));
  uint32_t nProbe = 0;
  uint32_t extraHits = 0;
  uint32_t incrHits = 0;
  uint32_t exportCount = 0;
  float strip = 0;
  for (int i = 0; i < samples; ++i) {
    auto t = Clock::now();
    addGravityAndUpdate(*inst);
    aMs.push_back(msSince(t));

    t = Clock::now();
    nProbe = copyProbes(*inst);
    probeMs.push_back(msSince(t));

    float maxT = 0, maxC = 0, maxS = 0, ry = 0;
    t = Clock::now();
    currentNestedStats(*inst, nProbe, maxT, maxC, maxS, strip, ry);
    nestedMs.push_back(msSince(t));

    t = Clock::now();
    extraHits = extraCopyCandidates(*inst, blast::kE2StrengthFailPa);
    extraCopyMs.push_back(msSince(t));

    t = Clock::now();
    linearStats(*inst, nProbe, maxT, maxC, maxS, strip, ry);
    linearMs.push_back(msSince(t));

    t = Clock::now();
    incrHits = incrementalCandidates(*inst, nProbe, blast::kE2StrengthFailPa);
    incrCandMs.push_back(msSince(t));
  }

  physics::FixedStepClock clock;
  for (int i = 0; i < samples; ++i) {
    const auto t = Clock::now();
    clock.advance(physics::kDt, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
    prodMs.push_back(msSince(t));
    prodProbe.push_back(world.debug().probeMs);
    prodStats.push_back(world.debug().statsMs);
    prodCand.push_back(world.debug().candidateMs);
    exportCount = world.debug().probeExportCount;
  }

  std::vector<double> bMs, cProdMs;
  bMs.reserve(static_cast<size_t>(samples));
  for (int i = 0; i < samples; ++i) {
    bMs.push_back(aMs[static_cast<size_t>(i)] + probeMs[static_cast<size_t>(i)]);
    cProdMs.push_back(bMs.back() + linearMs[static_cast<size_t>(i)] + incrCandMs[static_cast<size_t>(i)]);
  }

  const Ms A = summarize(aMs);
  const Ms B = summarize(bMs);
  const Ms Cprod = summarize(cProdMs);
  const Ms P = summarize(probeMs);
  const Ms N = summarize(nestedMs);
  const Ms E = summarize(extraCopyMs);
  const Ms L = summarize(linearMs);
  const Ms I = summarize(incrCandMs);
  const Ms Prod = summarize(prodMs);
  const Ms PP = summarize(prodProbe);
  const Ms PS = summarize(prodStats);
  const Ms PC = summarize(prodCand);
  const double wrap = P.median + L.median + I.median;

  std::cout << "\n## " << (cut ? "cut" : "intact") << "  iters=" << iters << "  warmup=" << warmup
            << "  samples=" << samples << "\n";
  std::cout << "nodes=" << inst->graph.nodes.size() << " bonds=" << inst->graph.bonds.size()
            << " probes=" << nProbe << " conv=" << inst->blast.solver->converged()
            << " lin=" << inst->blast.solver->getStressErrorLinear() << " strip=" << strip
            << " extraHits=" << extraHits << " incrHits=" << incrHits << " exports/tick=" << exportCount << "\n";
  std::cout << "| layer | median | p95 | max | mean | min |\n";
  std::cout << "|---|---:|---:|---:|---:|---:|\n";
  printMs("A gravity+update", A);
  printMs("B A + copyBondProbes", B);
  printMs("C production (B + linear + filter)", Cprod);
  printMs("  copyBondProbes alone", P);
  printMs("  linear strip+maxTCS", L);
  printMs("  candidate filter on existing probes", I);
  printMs("  nested strip (not production)", N);
  printMs("  extra collectOverstressed copy", E);
  printMs("production tick (solveInstance)", Prod);
  printMs("  production probeMs", PP);
  printMs("  production statsMs", PS);
  printMs("  production candidateMs", PC);
  std::cout << std::setprecision(3) << "delta B-A median=" << (B.median - A.median)
            << "  C-B median=" << (Cprod.median - B.median) << "  export+stats+cand median=" << wrap
            << "  exports/tick=" << exportCount << "\n";
  expect(exportCount == 1, "production path exports once per tick");
  if (wrap > 2.0) {
    std::cout << "NOTE: export+stats+cand median " << wrap << " ms exceeds 1-2 ms target\n";
  }

  world.clear();
}

}  // namespace

int main() {
  std::cout << "blast_e2_perf same E1 cylinder; layers A/B/C (D is vulkan_engine_voxel --e2-perf)\n";
  std::cout << "A = addGravity + solver.update()\n";
  std::cout << "B = A + copyBondProbes (linear)\n";
  std::cout << "C = B + linear stats + candidate filter on existing probes (one export)\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  runVariant(rt, false, 200, 4, 12);
  runVariant(rt, true, 200, 4, 12);
  runVariant(rt, false, 400, 3, 8);
  runVariant(rt, true, 400, 3, 8);
  rt.shutdown();
  if (gFailures != 0) {
    std::cerr << gFailures << " failure(s)\n";
    return 1;
  }
  std::cout << "OK blast_e2_perf\n";
  return 0;
}
