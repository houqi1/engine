#include "blast/FrameVoxels.h"
#include "blast/HardFracture.h"
#include "blast/ImpactDamage.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include "NvBlast.h"

#include <glm/gtc/quaternion.hpp>

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
      : spec_{88},  // Preserve the original tall-frame stress regression fixture.
        solid_(static_cast<size_t>(spec_.nx * spec_.ny * spec_.nz), 0),
        anc_(static_cast<size_t>(spec_.nx * spec_.ny * spec_.nz), 0) {
    blast::forEachFrameFine(spec_, [&](int x, int y, int z) {
      if (cutThree && blast::inThreeColumnCut(x, y, z, spec_.columnHeight)) {
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
  for (int height : {10, 20, 40, 88, 92}) {
    blast::FrameRaster r{};
    r.columnHeight = height;
    int top = -1;
    int roofFines = 0;
    int removed = 0;
    blast::forEachFrameFine(r, [&](int x, int y, int z) {
      top = std::max(top, y);
      const bool cut = blast::inThreeColumnCut(x, y, z, height);
      if (cut) ++removed;
      if (y >= height) {
        ++roofFines;
        expect(!cut, "height-aware column cut preserves roof");
      }
    });
    expect(top == height + blast::kFrameRoofT - 1, "roof follows column height");
    expect(roofFines == 3072, "roof volume stays constant across heights");
    expect(removed == 3 * blast::kFrameColW * blast::kFrameColW *
                          (height - blast::kFrameAnchorFines), "cut reaches selected column height");
  }
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
    const uint32_t n = inst != nullptr ? inst->probeCount : 0;
    const float th[] = {2.5e5f, 5.0e5f, 1.0e6f, 1.5e6f, 2.0e6f, 2.5e6f};
    std::cout << "  bond counts above S (keep-col vs other):\n";
    for (float s0 : th) {
      uint32_t keep = 0;
      uint32_t other = 0;
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
    const float failS = blast::kFrameStrengthFailPa;
    const float binM = 0.5f;
    const int nBin = 20;
    uint32_t yOver[20] = {};
    uint32_t yAll[20] = {};
    float yMin = 1.0e9f;
    float yMax = -1.0e9f;
    uint32_t keepOver = 0;
    uint32_t keepTotal = 0;
    for (uint32_t i = 0; i < n; ++i) {
      const auto& p = inst->probes[i];
      if (p.blastBondIndex >= inst->bondMeta.size()) {
        continue;
      }
      const blast::BondMeta& m = inst->bondMeta[p.blastBondIndex];
      if (m.inStrip == 0 || m.world != 0 || m.graphIndex >= inst->graph.bonds.size()) {
        continue;
      }
      if (!canTakeDamage(p.health)) {
        continue;
      }
      const float cy = inst->graph.bonds[m.graphIndex].cy;
      yMin = std::min(yMin, cy);
      yMax = std::max(yMax, cy);
      const int b = std::max(0, std::min(nBin - 1, static_cast<int>(cy / binM)));
      ++yAll[b];
      ++keepTotal;
      if (blast::probeMaxStress(p) > failS) {
        ++yOver[b];
        ++keepOver;
      }
    }
    std::cout << "  keep-col bonds σ>" << failS << " Pa: " << keepOver << "/" << keepTotal
              << "  y=[" << yMin << "," << yMax << "]\n";
    for (int b = 0; b < nBin; ++b) {
      if (yAll[b] == 0) {
        continue;
      }
      std::cout << "    y " << (b * binM) << "-" << ((b + 1) * binM) << " m  over=" << yOver[b]
                << " / " << yAll[b] << "\n";
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
  uint32_t nfrac = 0;
  uint32_t actors = 1;
  uint32_t firstCand = 0;
  for (int step = 0; step < 256; ++step) {
    solveWorld(cutFail, 1);
    const uint32_t cand = cutFail.debug().candidateCount;
    if (step == 0) {
      firstCand = cand;
      bool anyPartial = false;
      for (const blast::FractureCandidate& c : cutFail.pendingFracture().candidates) {
        if (c.healthBefore > 0.0f && c.damage > 0.0f && c.damage < c.healthBefore) {
          anyPartial = true;
        }
      }
      expect(anyPartial, "below fatal ExtStress damage is fractional");
    }
    if (cand == 0) {
      if (cutFail.debug().converged &&
          cutFail.debug().stripMaxStress < blast::kFrameStrengthFailPa) {
        break;
      }
      continue;
    }
    nfrac += cutFail.applyPendingCandidates(cutFail.pendingFracture());
    actors = cutFail.splitAllRequired();
    if (actors > 1) {
      break;
    }
  }
  expect(nfrac > 0, "low strength applies ExtStress damage");
  expect(actors > 1, "ExtPxStressSolver-style commands split the family");
  expect(firstCand > 0, "first solve emits ExtStress fracture commands");
  expect(firstCand >= 36, "all over-elastic keep-col bonds are submitted, no peak filter");
  std::cout << "  fail firstCand=" << firstCand << " fractured=" << nfrac << " actors=" << actors << "\n";
  driveUntilConverged(cutFail, 24);
  solveWorld(cutFail, 1);
  std::cout << "  after-split conv=" << cutFail.debug().converged
            << " strip=" << cutFail.debug().stripMaxStress
            << " cand=" << cutFail.debug().candidateCount
            << " actors=" << cutFail.debug().splitActors << "\n";
  blast::WorldContactImpulse smash{};
  smash.idA = VoxelObjectId{1, 1};
  smash.worldPoint = glm::vec3(0.0f, 1.0f, 0.0f);
  smash.JA = glm::vec3(0.0f, 1.0e6f, 0.0f);
  smash.xA = glm::vec3(0.0f, 1.0f, 0.0f);
  smash.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  smash.persistent = false;
  smash.eventId = 7;
  smash.tickId = 500;
  const float stripBeforeHit = cutFail.debug().stripMaxStress;
  cutFail.onPhysicsTick(500, physics::kDt, &smash, 1);
  expect(!cutFail.stressImpactImpulses(), "anchored impact-to-stress stays off");
  expect(cutFail.debug().stripMaxStress <= stripBeforeHit * 1.01f + 1.0f,
         "anchored contact is not added into ExtStress");
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
  std::cout << "  1 iter conv=" << world.debug().converged << " cand=" << world.debug().candidateCount << "\n";
  world.setSolverIters(400);
  driveUntilConverged(world, 48);
  std::cout << "  recovered conv=" << world.debug().converged << " cand=" << world.debug().candidateCount
            << " strip=" << world.debug().stripMaxStress << "\n";
  expect(world.debug().converged, "recovered budget converges");
  solveWorld(world, 1);
  expect(world.debug().candidateCount > 0 || world.debug().stripMaxStress <= blast::kFrameStrengthFailPa,
         "cut column is over-elastic or already unloaded");
  world.clear();
}

void blastLogQuiet(int type, const char* msg, const char* file, int line) {
  if (type <= NvBlastMessage::Error) {
    std::cerr << "NvBlastLL " << file << ":" << line << ": " << (msg ? msg : "") << "\n";
  }
}

// Diagnose why Fail's falling roof stays one visible piece after landing.
// Mirrors the real scene after occupancy extract: each Blast actor has its own objectId.
void testRoofLandingSecondarySplit(blast::BlastRuntime& rt) {
  std::cout << "DIAG Fail roof landing secondary Impact\n";
  blast::StructureWorld world;
  expect(world.init(rt), "diag init");
  expect(mount(world, true, 400, blast::kFrameStrengthHoldPa), "diag mount cut");
  world.setStrengthPa(blast::kFrameStrengthFailPa);
  world.setFractureEnabled(true);
  world.setImpactDamageEnabled(true);
  world.setStressImpactImpulses(false);
  blast::ImpactSettings impact = world.impactSettings();
  impact.shearDamage = false;  // Explicitly exercise Viewer's ImpactSpread option.
  world.setImpactSettings(impact);
  driveUntilConverged(world, 24);

  uint32_t actors = 1;
  for (int step = 0; step < 256; ++step) {
    solveWorld(world, 1);
    if (world.debug().candidateCount == 0) {
      if (world.debug().converged && world.debug().stripMaxStress < blast::kFrameStrengthFailPa) {
        break;
      }
      continue;
    }
    world.applyPendingCandidates(world.pendingFracture());
    actors = world.splitAllRequired();
    if (actors > 1) {
      break;
    }
  }
  expect(actors > 1, "diag Fail split");
  std::cout << "  after Fail actors=" << actors << " bindings=" << world.bindings().size() << "\n";
  // Tick once with no impulses so syncBondDamageFromHealth runs after Fail only.
  world.onPhysicsTick(800, physics::kDt, nullptr, 0);
  std::cout << "  AFTER_FAIL_ONLY maxBondDamage=" << world.debug().maxBondDamage
            << " damagedBonds=" << world.debug().damagedBondCount
            << " events=" << world.debug().impactDamageEvents << "\n";

  // Real scene: bindVisibleActors gives each island its own objectId.
  const VoxelObjectId stumpId{2, 1};
  const VoxelObjectId roofId{3, 1};
  const VoxelObjectId chipAId{4, 1};
  const VoxelObjectId chipBId{5, 1};
  std::vector<blast::ActorObjectLink> links;
  const blast::ActorBinding* roofBind = nullptr;
  uint32_t freeIdx = 0;
  for (const blast::ActorBinding& b : world.bindings()) {
    blast::ActorObjectLink L;
    L.actor = b.actor;
    if (b.anchored) {
      L.objectId = stumpId;
    } else if (b.graphNodeCount > 8) {
      L.objectId = roofId;
      roofBind = &b;
    } else if (freeIdx == 0) {
      L.objectId = chipAId;
      ++freeIdx;
    } else {
      L.objectId = chipBId;
      ++freeIdx;
    }
    links.push_back(L);
  }
  world.bindVisibleActors(links);
  expect(roofBind != nullptr, "diag found free multi-node roof actor");
  // Re-find roof after rebind (pointers in links are stable; binding vector rebuilt).
  const blast::ActorBinding* roof = nullptr;
  const blast::ActorBinding* stump = nullptr;
  for (const blast::ActorBinding& b : world.bindings()) {
    if (b.objectId == roofId) {
      roof = &b;
    }
    if (b.objectId == stumpId) {
      stump = &b;
    }
  }
  expect(roof != nullptr && stump != nullptr, "diag roof/stump bindings after relink");
  std::cout << "  roof nodes=" << roof->graphNodeCount << " comAsset=(" << roof->comAsset.x << ","
            << roof->comAsset.y << "," << roof->comAsset.z << ") stump nodes=" << stump->graphNodeCount
            << "\n";

  const uint32_t actorsBefore =
      world.instance() && world.instance()->blast.family
          ? NvBlastFamilyGetActorCount(world.instance()->blast.family, blastLogQuiet)
          : 0;

  // A) Roof vs stump (same family): must selfSkip — no Impact on this pair.
  {
    blast::WorldContactImpulse hit{};
    hit.idA = roofId;
    hit.idB = stumpId;
    hit.worldPoint = glm::vec3(roof->comAsset.x, 0.5f, roof->comAsset.z);
    hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
    hit.massA = 2000.0f;
    hit.massB = 0.0f;
    hit.velA = glm::vec3(0.0f, -12.0f, 0.0f);
    hit.velB = glm::vec3(0.0f);
    hit.xA = roof->comAsset;
    hit.xB = stump->comAsset;
    hit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    hit.qB = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    hit.persistent = false;
    hit.eventId = 9001;
    hit.tickId = 900;
    world.onPhysicsTick(900, physics::kDt, &hit, 1);
    world.applyPendingIfAny();
    std::cout << "  A roof-vs-stump events=" << world.debug().impactDamageEvents
              << " actors=" << world.debug().splitActors << "\n";
    expect(world.debug().impactDamageEvents == 0, "diag same-family roof-stump applies no Impact");
  }

  // B) Roof vs ground (ground has no Blast binding): Viewer Impact path.
  {
    blast::WorldContactImpulse hit{};
    hit.idA = roofId;
    hit.idB = {};  // ground
    hit.worldPoint = glm::vec3(roof->comAsset.x, roof->comAsset.y - 0.2f, roof->comAsset.z);
    hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
    hit.massA = 2000.0f;
    hit.massB = 0.0f;
    hit.velA = glm::vec3(0.0f, -12.0f, 0.0f);
    hit.velB = glm::vec3(0.0f);
    hit.xA = roof->comAsset;
    hit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    hit.persistent = false;
    hit.eventId = 9002;
    hit.tickId = 901;
    // onPhysicsTick already ends with collectPending; inspect split-required before applyPending.
    // Re-run Impact-only path pieces by calling solve then checking actor flags before apply.
    world.onPhysicsTick(901, physics::kDt, &hit, 1);
    const uint32_t events = world.debug().impactDamageEvents;
    uint32_t splitReq = 0;
    for (const blast::ActorBinding& b : world.bindings()) {
      if (b.actor != nullptr && NvBlastActorIsSplitRequired(b.actor, blastLogQuiet)) {
        ++splitReq;
      }
    }
    world.applyPendingIfAny();
    const uint32_t actorsAfter = world.debug().splitActors;
    const bool dirty = world.takeOccupancyDirty();
    std::cout << "  B roof-vs-ground events=" << events << " splitRequiredBeforeApply=" << splitReq
              << " actorsBefore=" << actorsBefore << " actorsAfter=" << actorsAfter
              << " occupancyDirty=" << (dirty ? 1 : 0) << " cand=" << world.debug().candidateCount << "\n";
    expect(events > 0, "diag roof-ground Impact produces events");
    expect(splitReq > 0, "diag Impact leaves IsSplitRequired before applyPending");
    expect(actorsAfter > actorsBefore, "diag roof-ground Impact increases Blast actor count");
    expect(dirty, "diag roof-ground Impact sets occupancyDirty for extract");
  }

  // B2) Same geometry, but dilute force like many averaged contacts / soft landing.
  // User report: events>0 while actors almost unchanged — reproduce partial damage.
  {
    // Fresh Fail world for isolation.
    blast::StructureWorld w2;
    expect(w2.init(rt), "B2 init");
    expect(mount(w2, true, 400, blast::kFrameStrengthHoldPa), "B2 mount");
    w2.setStrengthPa(blast::kFrameStrengthFailPa);
    w2.setFractureEnabled(true);
    w2.setImpactDamageEnabled(true);
    w2.setStressImpactImpulses(false);
    blast::ImpactSettings is = w2.impactSettings();
    is.shearDamage = false;
    w2.setImpactSettings(is);
    driveUntilConverged(w2, 24);
    for (int step = 0; step < 256; ++step) {
      solveWorld(w2, 1);
      if (w2.debug().candidateCount == 0) {
        if (w2.debug().converged && w2.debug().stripMaxStress < blast::kFrameStrengthFailPa) {
          break;
        }
        continue;
      }
      w2.applyPendingCandidates(w2.pendingFracture());
      if (w2.splitAllRequired() > 1) {
        break;
      }
    }
    std::vector<blast::ActorObjectLink> linksB2;
    const blast::ActorBinding* roofB2pre = nullptr;
    for (const blast::ActorBinding& b : w2.bindings()) {
      blast::ActorObjectLink L;
      L.actor = b.actor;
      if (b.anchored) {
        L.objectId = stumpId;
      } else if (b.graphNodeCount > 8) {
        L.objectId = roofId;
        roofB2pre = &b;
      } else {
        L.objectId = chipAId;
      }
      linksB2.push_back(L);
    }
    w2.bindVisibleActors(linksB2);
    const blast::ActorBinding* roofB2 = nullptr;
    for (const blast::ActorBinding& b : w2.bindings()) {
      if (b.objectId == roofId) {
        roofB2 = &b;
      }
    }
    expect(roofB2 != nullptr || roofB2pre != nullptr, "B2 roof");
    if (roofB2 != nullptr) {
      const uint32_t before =
          w2.instance() && w2.instance()->blast.family
              ? NvBlastFamilyGetActorCount(w2.instance()->blast.family, blastLogQuiet)
              : 0;
      // Soft hit: |force| = m*|v| = 2000*0.6 = 1200 → damage=120, normalized=1 still.
      // Need below material health*threshold: damage < 10 for normalized 0 after min filter?
      // damage=force/hardness; normalized=damage/100; need damage < 10 → force < 100.
      // Use vel so force = 80 → damage=8 → normalized=0 (filtered). That's events=0.
      // Use force≈150 → damage=15 → normalized=0.15 → small bond damage, may not split.
      blast::WorldContactImpulse hit{};
      hit.idA = roofId;
      hit.worldPoint = glm::vec3(roofB2->comAsset.x, roofB2->comAsset.y - 0.2f, roofB2->comAsset.z);
      hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
      hit.massA = 2000.0f;
      hit.massB = 0.0f;
      hit.velA = glm::vec3(0.0f, -0.08f, 0.0f);  // force=160, damage=16, norm=0.16
      hit.xA = roofB2->comAsset;
      hit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      hit.persistent = false;
      hit.eventId = 9100;
      hit.tickId = 910;
      // |dv|^2 = 0.0064 < 1 → viewerPairForce filters → events=0. Use vel -1.05 to pass filter
      // with still-modest force: force=2100, damage=210, norm=1 → full.
      // To get events>0 without split: pass filter (|v|>=1) but tiny reduced mass.
      hit.massA = 5.0f;             // force = 5*1.05 ≈ 5.25 → damage 0.525 → norm 0 (filtered)
      hit.velA = glm::vec3(0.0f, -1.05f, 0.0f);
      w2.onPhysicsTick(910, physics::kDt, &hit, 1);
      std::cout << "  B2a tiny mass events=" << w2.debug().impactDamageEvents << "\n";
      // Mid: mass=80, |v|=1.05 → force≈84 → damage=8.4 → still filtered (<10).
      hit.massA = 120.0f;  // force=126, damage=12.6, norm=0.126
      hit.eventId = 9101;
      hit.tickId = 911;
      w2.onPhysicsTick(911, physics::kDt, &hit, 1);
      const uint32_t evMid = w2.debug().impactDamageEvents;
      uint32_t splitReqMid = 0;
      for (const blast::ActorBinding& b : w2.bindings()) {
        if (b.actor != nullptr && NvBlastActorIsSplitRequired(b.actor, blastLogQuiet)) {
          ++splitReqMid;
        }
      }
      w2.applyPendingIfAny();
      const uint32_t afterMid = w2.debug().splitActors;
      const bool dirtyMid = w2.takeOccupancyDirty();
      std::cout << "  B2b mid-hit events=" << evMid << " splitReq=" << splitReqMid << " actors " << before
                << "->" << afterMid << " dirty=" << (dirtyMid ? 1 : 0) << "\n";
      // This is the user-visible pattern if real landing force is only modestly above threshold:
      // events can be >0 while actors stay flat when damage does not open a support cut.
      if (evMid > 0 && afterMid <= before) {
        std::cout << "  B2 PATTERN MATCH: events>0 and actors unchanged (damage without split)\n";
      }
    }
    w2.clear();
  }

  // B3) Repeat mid-strength hits: prove whether NvBlast health / bondDamage accumulate to a split.
  {
    blast::StructureWorld w3;
    expect(w3.init(rt), "B3 init");
    expect(mount(w3, true, 400, blast::kFrameStrengthHoldPa), "B3 mount");
    w3.setStrengthPa(blast::kFrameStrengthFailPa);
    w3.setFractureEnabled(true);
    w3.setImpactDamageEnabled(true);
    w3.setStressImpactImpulses(false);
    blast::ImpactSettings is = w3.impactSettings();
    is.shearDamage = false;
    w3.setImpactSettings(is);
    driveUntilConverged(w3, 24);
    for (int step = 0; step < 256; ++step) {
      solveWorld(w3, 1);
      if (w3.debug().candidateCount == 0) {
        if (w3.debug().converged && w3.debug().stripMaxStress < blast::kFrameStrengthFailPa) {
          break;
        }
        continue;
      }
      w3.applyPendingCandidates(w3.pendingFracture());
      if (w3.splitAllRequired() > 1) {
        break;
      }
    }
    std::vector<blast::ActorObjectLink> links3;
    for (const blast::ActorBinding& b : w3.bindings()) {
      blast::ActorObjectLink L;
      L.actor = b.actor;
      if (b.anchored) {
        L.objectId = stumpId;
      } else if (b.graphNodeCount > 8) {
        L.objectId = roofId;
      } else {
        L.objectId = chipAId;
      }
      links3.push_back(L);
    }
    w3.bindVisibleActors(links3);
    const blast::ActorBinding* roof3 = nullptr;
    for (const blast::ActorBinding& b : w3.bindings()) {
      if (b.objectId == roofId) {
        roof3 = &b;
      }
    }
    expect(roof3 != nullptr, "B3 roof");
    if (roof3 != nullptr) {
      const uint32_t actors0 =
          w3.instance() && w3.instance()->blast.family
              ? NvBlastFamilyGetActorCount(w3.instance()->blast.family, blastLogQuiet)
              : 0;
      float maxDmg = 0.0f;
      uint32_t lastEvents = 0;
      uint32_t actorsN = actors0;
      for (int hitI = 0; hitI < 30; ++hitI) {
        blast::WorldContactImpulse hit{};
        hit.idA = roofId;
        hit.worldPoint = glm::vec3(roof3->comAsset.x, roof3->comAsset.y - 0.2f, roof3->comAsset.z);
        hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
        hit.massA = 120.0f;
        hit.massB = 0.0f;
        hit.velA = glm::vec3(0.0f, -1.05f, 0.0f);
        hit.xA = roof3->comAsset;
        hit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        hit.persistent = false;
        hit.eventId = 9200u + static_cast<uint32_t>(hitI);
        hit.tickId = 920u + static_cast<uint32_t>(hitI);
        w3.onPhysicsTick(hit.tickId, physics::kDt, &hit, 1);
        lastEvents = w3.debug().impactDamageEvents;
        w3.applyPendingIfAny();
        actorsN = w3.debug().splitActors;
        maxDmg = w3.debug().maxBondDamage;
        if (actorsN > actors0) {
          break;
        }
      }
      std::cout << "  B3 30x mid-hit: lastEvents=" << lastEvents << " maxBondDamage=" << maxDmg
                << " actors " << actors0 << "->" << actorsN
                << " damagedBonds=" << w3.debug().damagedBondCount << "\n";
      expect(lastEvents > 0 || maxDmg > 0.0f, "B3 mid-hits apply some Impact damage");
      if (actorsN <= actors0) {
        std::cout << "  B3 PROOF: 30 mid-strength landings did not increase actors "
                     "(damage without split — matches HUD actors flat)\n";
      } else {
        std::cout << "  B3 PROOF: repeated mid-hits eventually split\n";
      }
    }
    w3.clear();
  }

  // C) Same as B but with Stress fracture still collecting pending — check split still runs.
  {
    driveUntilConverged(world, 8);
    // Re-bind: split may have destroyed actors; map remaining free multi-node to roofId.
    std::vector<blast::ActorObjectLink> links2;
    VoxelObjectId nextChip{6, 1};
    for (const blast::ActorBinding& b : world.bindings()) {
      blast::ActorObjectLink L;
      L.actor = b.actor;
      if (b.anchored) {
        L.objectId = stumpId;
      } else if (b.graphNodeCount > 4 && b.stressSolve) {
        L.objectId = roofId;
      } else {
        L.objectId = nextChip;
        nextChip.slot += 1;
      }
      links2.push_back(L);
    }
    world.bindVisibleActors(links2);
    const blast::ActorBinding* roof2 = nullptr;
    for (const blast::ActorBinding& b : world.bindings()) {
      if (b.objectId == roofId) {
        roof2 = &b;
        break;
      }
    }
    if (roof2 == nullptr || roof2->graphNodeCount <= 1) {
      std::cout << "  C skipped (no multi-node free roof left after B)\n";
    } else {
      world.setFractureEnabled(true);
      const uint32_t before =
          world.instance() && world.instance()->blast.family
              ? NvBlastFamilyGetActorCount(world.instance()->blast.family, blastLogQuiet)
              : 0;
      blast::WorldContactImpulse hit{};
      hit.idA = roofId;
      hit.worldPoint = glm::vec3(roof2->comAsset.x, roof2->comAsset.y - 0.2f, roof2->comAsset.z);
      hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
      hit.massA = 1500.0f;
      hit.massB = 0.0f;
      hit.velA = glm::vec3(0.0f, -10.0f, 0.0f);
      hit.xA = roof2->comAsset;
      hit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
      hit.persistent = false;
      hit.eventId = 9003;
      hit.tickId = 902;
      world.onPhysicsTick(902, physics::kDt, &hit, 1);
      const uint32_t events = world.debug().impactDamageEvents;
      const uint32_t cand = world.debug().candidateCount;
      world.applyPendingIfAny();
      const uint32_t after = world.debug().splitActors;
      const bool dirty = world.takeOccupancyDirty();
      std::cout << "  C stressOn+landing events=" << events << " cand=" << cand << " actors " << before
                << "->" << after << " dirty=" << (dirty ? 1 : 0) << "\n";
      expect(events > 0, "diag C Impact still fires with Stress fracture on");
      expect(after > before || dirty, "diag C still splits or dirties with Stress pending present");
    }
  }
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
  testRoofLandingSecondarySplit(rt);
  testConvergeGate(rt);
  testFreeBody(rt);
  rt.shutdown();
  if (gFailures != 0) {
    std::cerr << "FAILED " << gFailures << "\n";
    return 1;
  }
  std::cout << "OK four-column T01/T02/T05/T06 + landing diag\n";
  return 0;
}
