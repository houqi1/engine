#include "blast/BlastMemory.h"
#include "blast/CylinderGraph.h"
#include "blast/HardFracture.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastGlobals.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using blast::CylinderBuildOpts;
using blast::CylinderScene;
using Nv::Blast::ExtStressSolver;

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

std::vector<ExtStressSolver::BondProbe> probesOf(ExtStressSolver& solver) {
  std::vector<ExtStressSolver::BondProbe> p(512);
  const uint32_t n = solver.copyBondProbes(p.data(), static_cast<uint32_t>(p.size()));
  p.resize(n);
  return p;
}

const ExtStressSolver::BondProbe* findProbe(const std::vector<ExtStressSolver::BondProbe>& p, uint32_t sdkBond) {
  for (const auto& x : p) {
    if (x.blastBondIndex == sdkBond) {
      return &x;
    }
  }
  return nullptr;
}

void zeroBonds(CylinderScene& scene, const std::vector<uint32_t>& sdkBonds) {
  std::vector<NvBlastBondFractureData> cmds;
  const float* healths = NvBlastActorGetBondHealths(scene.actor, blastLog);
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(scene.asset, blastLog);
  for (uint32_t idx : sdkBonds) {
    if (healths[idx] <= 0.0f || !canTakeDamage(healths[idx])) {
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
  NvBlastActorApplyFracture(nullptr, scene.actor, &buf, blastLog, nullptr);
  scene.solver->syncBrokenBonds();
  blast::splitIfNeeded(scene.actor, scene.family, *scene.solver, blastLog);
}

std::vector<uint32_t> baseSectorBonds(const blast::CylinderIdMaps& ids, const std::vector<int>& cutK) {
  std::vector<uint32_t> out;
  for (int k : cutK) {
    out.push_back(ids.sdkBondFromVert[static_cast<size_t>(k)]);  // j=0 vertical
    out.push_back(ids.sdkBondFromCirc[static_cast<size_t>(k)]);  // j=0 circ
  }
  return out;
}

std::vector<int> rangeK(int a, int bExclusive) {
  std::vector<int> v;
  for (int k = a; k < bExclusive; ++k) {
    v.push_back(k);
  }
  return v;
}

float remainingStripMaxStress(const CylinderScene& scene, const std::vector<ExtStressSolver::BondProbe>& p,
                              const std::vector<int>& keepK) {
  float m = 0.0f;
  for (int k : keepK) {
    const uint32_t sdk = scene.ids.sdkBondFromVert[static_cast<size_t>(k)];
    const auto* pr = findProbe(p, sdk);
    if (pr) {
      m = std::max(m, blast::probeMaxStress(*pr));
    }
  }
  return m;
}

float remainingStripMaxTension(const CylinderScene& scene, const std::vector<ExtStressSolver::BondProbe>& p,
                               const std::vector<int>& keepK) {
  float m = 0.0f;
  for (int k : keepK) {
    const uint32_t sdk = scene.ids.sdkBondFromVert[static_cast<size_t>(k)];
    const auto* pr = findProbe(p, sdk);
    if (pr) {
      m = std::max(m, pr->tension);
    }
  }
  return m;
}

NvcVec3 worldReaction(const CylinderScene& scene, const std::vector<ExtStressSolver::BondProbe>& p) {
  NvcVec3 r{0.0f, 0.0f, 0.0f};
  for (int k = 0; k < blast::kCylinderCirc; ++k) {
    const uint32_t sdk = scene.ids.sdkBondFromWorld[static_cast<size_t>(k)];
    const auto* pr = findProbe(p, sdk);
    if (!pr) {
      continue;
    }
    NvcVec3 f = pr->forceLinear;
    if (pr->node0 == scene.ids.graphWorld) {
      f.x = -f.x;
      f.y = -f.y;
      f.z = -f.z;
    }
    r.x += f.x;
    r.y += f.y;
    r.z += f.z;
  }
  return r;
}

bool finiteProbe(const ExtStressSolver::BondProbe& p) {
  return std::isfinite(p.compression) && std::isfinite(p.tension) && std::isfinite(p.shear) &&
         std::isfinite(p.forceLinear.x) && std::isfinite(p.forceLinear.y) && std::isfinite(p.forceLinear.z);
}

void testT01T24(blast::TrackingAllocator& alloc) {
  std::cout << "T01/T24 intact cylinder\n";
  CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 200;
  CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "build intact cylinder");
  expect(scene.actor && NvBlastActorHasExternalBonds(scene.actor, blastLog), "world anchor present");
  expect(blast::solveGravity(scene, 3), "solve gravity");
  expect(scene.solver->converged(), "T01 converged");
  expect(std::isfinite(scene.solver->getStressErrorLinear()), "T01 lin residual finite");
  expect(std::isfinite(scene.solver->getStressErrorAngular()), "T01 ang residual finite");

  const auto p = probesOf(*scene.solver);
  expect(!p.empty(), "bond probes");
  for (const auto& x : p) {
    expect(finiteProbe(x), "probe finite");
  }
  const uint32_t fractured = blast::applyHardThreshold(scene.actor, *scene.solver, blast::kStrengthHoldPa, blastLog);
  expect(fractured == 0, "T01 no hard-threshold fracture at hold strength");
  expect(NvBlastFamilyGetActorCount(scene.family, blastLog) == 1, "T01 still one actor");

  const float W = blast::cylinderTotalMass() * -blast::kGravityY;
  const NvcVec3 R = worldReaction(scene, p);
  const float residual = std::sqrt((R.x) * (R.x) + (R.y - W) * (R.y - W) + (R.z) * (R.z));
  const float rel = residual / W;
  std::cout << "  T24 W=" << W << " N  Ry=" << R.y << "  rel=" << rel << "  lin=" << scene.solver->getStressErrorLinear()
            << " ang=" << scene.solver->getStressErrorAngular() << "\n";
  expect(rel <= 0.001f, "T24 reaction vs weight <= 0.1%");
  blast::destroyCylinderScene(scene);
}

void testT05(blast::TrackingAllocator& alloc) {
  std::cout << "T05 free body\n";
  CylinderBuildOpts opts;
  opts.worldAnchor = false;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 200;
  CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "build free cylinder");
  expect(blast::solveGravity(scene, 3), "free gravity");
  const float W = blast::cylinderTotalMass() * -blast::kGravityY;
  const auto p = probesOf(*scene.solver);
  float maxF = 0.0f;
  for (const auto& x : p) {
    const float mag =
        std::sqrt(x.forceLinear.x * x.forceLinear.x + x.forceLinear.y * x.forceLinear.y + x.forceLinear.z * x.forceLinear.z);
    maxF = std::max(maxF, mag);
  }
  const float rel = maxF / W;
  std::cout << "  T05 max|F|/W=" << rel << " converged=" << scene.solver->converged() << "\n";
  expect(rel <= 0.001f, "T05 internal force < 0.1% of weight");
  blast::destroyCylinderScene(scene);
}

void testT02(blast::TrackingAllocator& alloc) {
  std::cout << "T02 base 270 cut\n";
  const std::vector<int> keep = {0, 1, 2, 3};
  const std::vector<int> cut = rangeK(4, 16);

  CylinderBuildOpts hold;
  hold.strengthPa = blast::kStrengthHoldPa;
  hold.solverIters = 200;

  CylinderScene intact;
  expect(blast::buildCylinderScene(alloc, intact, hold), "T02 intact");
  blast::solveGravity(intact, 3);
  const float stressBefore = remainingStripMaxStress(intact, probesOf(*intact.solver), keep);
  blast::destroyCylinderScene(intact);

  CylinderScene cutHold;
  expect(blast::buildCylinderScene(alloc, cutHold, hold), "T02 cut-hold build");
  zeroBonds(cutHold, baseSectorBonds(cutHold.ids, cut));
  expect(NvBlastFamilyGetActorCount(cutHold.family, blastLog) == 1, "T02 upper shell still one actor after cut");
  blast::solveGravity(cutHold, 3);
  expect(cutHold.solver->converged(), "T02 cut-hold converged");
  const float stressAfter = remainingStripMaxStress(cutHold, probesOf(*cutHold.solver), keep);
  std::cout << "  remaining-strip stress before=" << stressBefore << " after=" << stressAfter << "\n";
  expect(stressAfter > stressBefore, "T02 remaining-strip stress rises");
  const uint32_t brokenHold =
      blast::applyHardThreshold(cutHold.actor, *cutHold.solver, blast::kStrengthHoldPa, blastLog);
  expect(brokenHold == 0, "T02 high strength does not break");
  expect(NvBlastFamilyGetActorCount(cutHold.family, blastLog) == 1, "T02 high strength still one actor");
  blast::destroyCylinderScene(cutHold);

  CylinderBuildOpts fail = hold;
  fail.strengthPa = blast::kStrengthFailPa;
  CylinderScene cutFail;
  expect(blast::buildCylinderScene(alloc, cutFail, fail), "T02 cut-fail build");
  zeroBonds(cutFail, baseSectorBonds(cutFail.ids, cut));
  blast::solveGravity(cutFail, 3);
  const uint32_t brokenFail =
      blast::applyHardThreshold(cutFail.actor, *cutFail.solver, blast::kStrengthFailPa, blastLog);
  blast::splitIfNeeded(cutFail.actor, cutFail.family, *cutFail.solver, blastLog);
  std::cout << "  T02 low-S fractures=" << brokenFail
            << " actors=" << NvBlastFamilyGetActorCount(cutFail.family, blastLog) << "\n";
  expect(brokenFail > 0 || NvBlastFamilyGetActorCount(cutFail.family, blastLog) > 1, "T02 low strength breaks");
  blast::destroyCylinderScene(cutFail);
}

void testT04(blast::TrackingAllocator& alloc) {
  std::cout << "T04 symmetric walls\n";
  CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 200;

  CylinderScene side;
  expect(blast::buildCylinderScene(alloc, side, opts), "T04 side build");
  zeroBonds(side, baseSectorBonds(side.ids, rangeK(4, 16)));
  blast::solveGravity(side, 3);
  const float tenSide = remainingStripMaxTension(side, probesOf(*side.solver), {0, 1, 2, 3});
  blast::destroyCylinderScene(side);

  CylinderScene sym;
  expect(blast::buildCylinderScene(alloc, sym, opts), "T04 sym build");
  std::vector<int> cutSym;
  for (int k = 0; k < blast::kCylinderCirc; ++k) {
    const bool keep = (k == 0 || k == 1 || k == 8 || k == 9);
    if (!keep) {
      cutSym.push_back(k);
    }
  }
  zeroBonds(sym, baseSectorBonds(sym.ids, cutSym));
  blast::solveGravity(sym, 3);
  const float tenSym = remainingStripMaxTension(sym, probesOf(*sym.solver), {0, 1, 8, 9});
  std::cout << "  T04 tension side90=" << tenSide << " sym45+45=" << tenSym << "\n";
  expect(tenSym < tenSide, "T04 bending/tension smaller than single 90 strip");
  blast::destroyCylinderScene(sym);
}

void testT03(blast::TrackingAllocator& alloc) {
  std::cout << "T03 weight scan\n";
  CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 200;
  const std::vector<int> cut = rangeK(4, 16);
  const std::vector<int> keep = {0, 1, 2, 3};

  CylinderScene a;
  opts.density = blast::kCylinderRho;
  expect(blast::buildCylinderScene(alloc, a, opts), "T03 rho");
  zeroBonds(a, baseSectorBonds(a.ids, cut));
  blast::solveGravity(a, 3);
  const float sA = remainingStripMaxStress(a, probesOf(*a.solver), keep);
  blast::destroyCylinderScene(a);

  CylinderScene b;
  opts.density = 2.0f * blast::kCylinderRho;
  expect(blast::buildCylinderScene(alloc, b, opts), "T03 2x rho");
  blast::setCylinderNodeMasses(b, opts.density);
  zeroBonds(b, baseSectorBonds(b.ids, cut));
  blast::solveGravity(b, 3);
  const float sB = remainingStripMaxStress(b, probesOf(*b.solver), keep);
  std::cout << "  T03 stress rho=" << sA << " 2rho=" << sB << "\n";
  expect(sB >= sA * 1.2f, "T03 heavier is not less dangerous");
  blast::destroyCylinderScene(b);
}

void testT06(blast::TrackingAllocator& alloc) {
  std::cout << "T06 converge gate\n";
  CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthFailPa;
  opts.solverIters = 1;
  CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "T06 build");
  zeroBonds(scene, baseSectorBonds(scene.ids, rangeK(4, 16)));
  scene.solver->addGravity(*scene.actor, NvcVec3{0.0f, blast::kGravityY, 0.0f});
  scene.solver->update();
  const bool conv = scene.solver->converged();
  const uint32_t n = blast::applyHardThreshold(scene.actor, *scene.solver, blast::kStrengthFailPa, blastLog);
  if (!conv) {
    expect(n == 0, "T06 no fracture when not converged");
  } else {
    std::cout << "  T06 iter=1 unexpectedly converged; skip no-fracture clause\n";
  }
  Nv::Blast::ExtStressSolverSettings st = scene.solver->getSettings();
  st.maxSolverIterationsPerFrame = 200;
  scene.solver->setSettings(st);
  blast::solveGravity(scene, 2);
  const uint32_t n2 = blast::applyHardThreshold(scene.actor, *scene.solver, blast::kStrengthFailPa, blastLog);
  expect(scene.solver->converged(), "T06 recovered budget converges");
  expect(n2 > 0, "T06 fracture resumes after budget restored");
  blast::destroyCylinderScene(scene);
}

void testT21(blast::TrackingAllocator& alloc) {
  std::cout << "T21 threshold boundary\n";
  CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 200;
  CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "T21 build");
  zeroBonds(scene, baseSectorBonds(scene.ids, rangeK(4, 16)));
  blast::solveGravity(scene, 3);
  const auto p = probesOf(*scene.solver);
  float sMax = 0.0f;
  uint32_t hot = 0;
  for (const auto& x : p) {
    if (!canTakeDamage(x.health)) {
      continue;
    }
    const float s = blast::probeMaxStress(x);
    if (s > sMax) {
      sMax = s;
      hot = x.blastBondIndex;
    }
  }
  expect(sMax > 0.0f, "T21 has positive stress");
  const float* h0 = NvBlastActorGetBondHealths(scene.actor, blastLog);
  const float healthHot = h0[hot];
  expect(blast::applyHardThreshold(scene.actor, *scene.solver, sMax, blastLog) == 0, "T21 equal threshold no damage");
  expect(NvBlastActorGetBondHealths(scene.actor, blastLog)[hot] == healthHot, "T21 health unchanged at equality");
  const uint32_t n = blast::applyHardThreshold(scene.actor, *scene.solver, sMax * 0.999f, blastLog);
  expect(n > 0, "T21 slightly over threshold fractures");
  const float h1 = NvBlastActorGetBondHealths(scene.actor, blastLog)[hot];
  expect(h1 <= 0.0f, "T21 one-shot full health");
  blast::solveGravity(scene, 0);
  const uint32_t n2 = blast::applyHardThreshold(scene.actor, *scene.solver, sMax * 0.999f, blastLog);
  expect(n2 == 0 || NvBlastActorGetBondHealths(scene.actor, blastLog)[hot] <= 0.0f,
         "T21 repeat generate is not partial damage");
  blast::destroyCylinderScene(scene);
}

void testT22(blast::TrackingAllocator& alloc) {
  std::cout << "T22 redundant cut no split\n";
  CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 200;
  CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "T22 build");
  const uint32_t midCirc = scene.ids.sdkBondFromCirc[static_cast<size_t>(4 + 4 * blast::kCylinderCirc)];  // j=4, k=4
  zeroBonds(scene, {midCirc});
  expect(NvBlastFamilyGetActorCount(scene.family, blastLog) == 1, "T22 no actor split");
  blast::solveGravity(scene, 1);
  const auto p = probesOf(*scene.solver);
  const auto* pr = findProbe(p, midCirc);
  expect(pr != nullptr, "T22 probe still listed");
  if (pr) {
    expect(pr->health <= 0.0f, "T22 bond health zero");
    expect(blast::probeMaxStress(*pr) == 0.0f, "T22 dead bond carries no stress next solve");
  }
  blast::destroyCylinderScene(scene);
}

void iterationScan(blast::TrackingAllocator& alloc) {
  std::cout << "iteration scan\n";
  const uint32_t iters[] = {25, 50, 100, 200};
  for (uint32_t it : iters) {
    CylinderBuildOpts opts;
    opts.strengthPa = blast::kStrengthHoldPa;
    opts.solverIters = it;
    CylinderScene scene;
    expect(blast::buildCylinderScene(alloc, scene, opts), "scan build");
    blast::solveGravity(scene, 0);
    std::cout << "  iters=" << it << " conv=" << scene.solver->converged()
              << " lin=" << scene.solver->getStressErrorLinear() << " ang=" << scene.solver->getStressErrorAngular()
              << "\n";
    expect(std::isfinite(scene.solver->getStressErrorLinear()), "scan lin finite");
    blast::destroyCylinderScene(scene);
  }
}

}  // namespace

int main() {
  std::cout << "blast_p1_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  std::cout << "cylinder R=" << blast::kCylinderR << " t=" << blast::kCylinderT << " H=" << blast::kCylinderH
            << " rho=" << blast::kCylinderRho << " S_hold=" << blast::kStrengthHoldPa
            << " S_fail=" << blast::kStrengthFailPa << "\n";
  std::cout << "mass=" << blast::cylinderTotalMass() << " kg  W=" << blast::cylinderTotalMass() * -blast::kGravityY
            << " N\n";

  blast::TrackingAllocator alloc;
  blast::LoggingErrorCallback errors;
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);

  testT01T24(alloc);
  testT05(alloc);
  testT02(alloc);
  testT03(alloc);
  testT04(alloc);
  testT06(alloc);
  testT21(alloc);
  testT22(alloc);
  iterationScan(alloc);

  expect(errors.errorCount() == 0, "no NvBlast error-callback errors");
  if (gFailures == 0) {
    std::cout << "OK P1 T01-T06 T21 T22 T24\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
