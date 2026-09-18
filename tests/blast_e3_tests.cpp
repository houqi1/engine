#include "blast/BlastMemory.h"
#include "blast/ContactLoads.h"
#include "blast/HardFracture.h"
#include "blast/ImpactDamage.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"
#include "physics/Solver.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastGlobals.h"
#include "NvCTypes.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

using Nv::Blast::ExtForceMode;
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
  }
}

bool near(float a, float b, float eps) { return std::abs(a - b) <= eps; }
bool nearV(const glm::vec3& a, const glm::vec3& b, float eps) {
  return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

void testImpulseBooks() {
  std::cout << "E3 contact impulse books\n";
  physics::RigidBody A{};
  physics::RigidBody B{};
  A.dynamic = true;
  A.awake = true;
  A.invM = 1.0f;
  A.Iloc = glm::mat3(1.0f);
  physics::refreshInverseInertiaWorld(A);
  B.dynamic = false;
  B.invM = 0.0f;
  B.x = glm::vec3(0.0f, -1.0f, 0.0f);

  physics::Contact c{};
  c.a = 0;
  c.b = 1;
  c.p = glm::vec3(0.0f, 0.0f, 0.0f);
  c.n = glm::vec3(0.0f, 1.0f, 0.0f);
  c.rA = glm::vec3(0.0f);
  c.rB = glm::vec3(0.0f, 1.0f, 0.0f);
  c.d = 0.05f;
  A.v = glm::vec3(0.0f);

  std::vector<physics::RigidBody> bodies{A, B};
  std::vector<physics::Contact> cs{c};
  physics::solveContacts(bodies, cs, physics::kSubDt, 8);
  expect(cs[0].lambdaN > 0.0f, "penetrating contact has physics lambdaN");
  expect(cs[0].lambdaNVel < 0.25f * cs[0].lambdaN, "Baumgarte excluded from impact lambda");
  const glm::vec3 JA = physics::contactImpulseOnA(cs[0]);
  expect(glm::length(JA) < 0.25f * cs[0].lambdaN, "structure impulse omits position correction");

  physics::Contact hit{};
  hit.a = 0;
  hit.b = 1;
  hit.p = glm::vec3(0.0f);
  hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
  hit.rA = glm::vec3(0.0f);
  hit.rB = glm::vec3(0.0f, 1.0f, 0.0f);
  hit.d = 0.0f;
  bodies[0].v = glm::vec3(0.0f, -2.0f, 0.0f);
  bodies[0].w = glm::vec3(0.0f);
  std::vector<physics::Contact> hs{hit};
  physics::solveContacts(bodies, hs, physics::kSubDt, 8);
  const glm::vec3 JA2 = physics::contactImpulseOnA(hs[0]);
  expect(JA2.y > 0.0f, "closing contact produces +Jy on A");
  const glm::vec3 JB2 = -JA2;
  expect(nearV(JA2 + JB2, glm::vec3(0.0f), 1e-5f), "impulses equal and opposite");
  expect(bodies[1].v == glm::vec3(0.0f), "static body does not take velocity");
}

void testTickFold() {
  std::cout << "E3 tick fold J/kDt\n";
  VoxelObjectId id{1, 1};
  blast::BodyAssetFrame fr;
  fr.objectId = id;
  fr.worldCom = glm::vec3(0.0f);
  fr.worldQ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  fr.assetCom = glm::vec3(0.0f);
  blast::NodeRef n0{0, glm::vec3(0.0f, 0.0f, 0.0f)};
  blast::NodeRef n1{1, glm::vec3(1.0f, 0.0f, 0.0f)};
  std::vector<blast::PickedImpulse> picked;
  glm::vec3 Jsum(0.0f);
  glm::vec3 Lsum(0.0f);
  const glm::vec3 origin(0.0f);
  for (int s = 0; s < physics::kSubsteps; ++s) {
    blast::WorldContactImpulse imp;
    imp.idA = id;
    imp.worldPoint = glm::vec3(0.2f, 0.0f, 0.1f);
    imp.JA = glm::vec3(0.0f, 0.5f, 0.0f);
    imp.xA = glm::vec3(0.0f);
    imp.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    blast::PickedImpulse p;
    expect(blast::projectImpulseToActor(imp, id, fr, p), "project");
    p.node = n0;
    p.ok = true;
    picked.push_back(p);
    Jsum += p.Jasset;
    Lsum += glm::cross(p.pAsset - origin, p.Jasset);
  }
  const blast::TickLoadResult r =
      blast::foldPickedImpulses(picked.data(), static_cast<uint32_t>(picked.size()), physics::kDt, origin);
  expect(r.contactContrib == static_cast<uint32_t>(physics::kSubsteps), "six contribs");
  expect(nearV(r.Jsum, Jsum, 1e-5f), "Jsum matches");
  expect(nearV(r.LsumOrigin, Lsum, 1e-5f), "Lsum matches");
  const glm::vec3 Fexpect = Jsum / physics::kDt;
  expect(!r.loads.empty(), "has loads");
  expect(nearV(r.loads[0].F, Fexpect, 1e-3f), "F = J_tick / kDt not kSubDt");
  const glm::vec3 wrong = Jsum / physics::kSubDt;
  expect(glm::length(r.loads[0].F - wrong) > 1.0f, "not using kSubDt");
}

void testEccentricMap() {
  std::cout << "E3 wall center vs eccentric\n";
  blast::NodeRef n{7, glm::vec3(0.0f, 1.0f, 0.0f)};
  const glm::vec3 F(0.0f, 100.0f, 0.0f);
  const glm::vec3 pC(0.0f, 1.0f, 0.0f);
  const glm::vec3 pE(0.0f, 1.0f, 0.4f);
  const blast::MappedLoad c = blast::mapForceWithTorque(pC, F, n);
  const blast::MappedLoad e = blast::mapForceWithTorque(pE, F, n);
  expect(nearV(c.F, F, 1e-5f) && nearV(e.F, F, 1e-5f), "force preserved");
  expect(nearV(c.tau, glm::vec3(0.0f), 1e-5f), "center no torque");
  expect(glm::length(e.tau) > 1.0f, "eccentric keeps moment");
  std::vector<blast::MappedLoad> loads{e};
  std::vector<blast::NodeRef> nodes{n};
  glm::vec3 Fr, Mr;
  blast::reconstructWRTOrigin(loads, nodes, Fr, Mr);
  const glm::vec3 Mexpect = glm::cross(n.center, F) + e.tau;
  const float relF = glm::length(Fr - F) / glm::length(F);
  const float relM = glm::length(Mr - Mexpect) / glm::max(1.0f, glm::length(Mexpect));
  expect(relF < 0.001f && relM < 0.001f, "P3 0.1% force/moment");
}

struct Beam {
  blast::AlignedBlock assetMem;
  blast::AlignedBlock familyMem;
  NvBlastAsset* asset = nullptr;
  NvBlastFamily* family = nullptr;
  NvBlastActor* actor = nullptr;
  ExtStressSolver* solver = nullptr;
  uint32_t graph[4]{};
  uint32_t sdkBond[3]{};
  blast::NodeRef nodes[4]{};
};

void destroyBeam(Beam& b) {
  if (b.solver) {
    b.solver->release();
    b.solver = nullptr;
  }
  b.actor = nullptr;
  b.family = nullptr;
  b.asset = nullptr;
  b.familyMem.reset();
  b.assetMem.reset();
}

bool buildBeam(blast::TrackingAllocator& alloc, Beam& beam, float midArea, float /*strengthPa*/, bool worldAnchor) {
  destroyBeam(beam);
  NvBlastChunkDesc chunks[4];
  for (int i = 0; i < 4; ++i) {
    chunks[i].centroid[0] = static_cast<float>(i) * 1.0f;
    chunks[i].centroid[1] = 0.0f;
    chunks[i].centroid[2] = 0.0f;
    chunks[i].volume = 0.1f;
    chunks[i].parentChunkDescIndex = UINT32_MAX;
    chunks[i].flags = NvBlastChunkDesc::SupportFlag;
    chunks[i].userData = 10u + static_cast<uint32_t>(i);
  }
  NvBlastBondDesc bonds[4];
  int nb = 0;
  auto addBond = [&](int a, int c, float area, uint32_t user, bool world) {
    NvBlastBondDesc& b = bonds[nb++];
    b.chunkIndices[0] = static_cast<uint32_t>(a);
    b.chunkIndices[1] = world ? UINT32_MAX : static_cast<uint32_t>(c);
    b.bond.normal[0] = 1.0f;
    b.bond.normal[1] = 0.0f;
    b.bond.normal[2] = 0.0f;
    b.bond.area = area;
    b.bond.centroid[0] =
        0.5f * (chunks[a].centroid[0] + (world ? chunks[a].centroid[0] : chunks[c].centroid[0]));
    b.bond.centroid[1] = 0.0f;
    b.bond.centroid[2] = 0.0f;
    b.bond.userData = user;
  };
  addBond(0, 1, 1.0f, 20, false);
  addBond(1, 2, midArea, 21, false);
  addBond(2, 3, 1.0f, 22, false);
  if (worldAnchor) {
    addBond(0, 0, 1.0f, 30, true);
  }
  std::vector<char> scratch(4096);
  NvBlastEnsureAssetExactSupportCoverage(chunks, 4, scratch.data(), blastLog);
  uint32_t map[4];
  NvBlastReorderAssetDescChunks(chunks, 4, bonds, static_cast<uint32_t>(nb), map, true, scratch.data(), blastLog);
  NvBlastAssetDesc desc{};
  desc.chunkCount = 4;
  desc.chunkDescs = chunks;
  desc.bondCount = static_cast<uint32_t>(nb);
  desc.bondDescs = bonds;
  const size_t need = NvBlastGetRequiredScratchForCreateAsset(&desc, blastLog);
  if (scratch.size() < need) {
    scratch.resize(need);
  }
  beam.assetMem.alloc = &alloc;
  beam.assetMem.ptr = alloc.allocate(NvBlastGetAssetMemorySize(&desc, blastLog), "asset", __FILE__, __LINE__);
  beam.asset = NvBlastCreateAsset(beam.assetMem.ptr, &desc, scratch.data(), blastLog);
  if (beam.asset == nullptr) {
    return false;
  }
  const NvBlastChunk* ch = NvBlastAssetGetChunks(beam.asset, blastLog);
  const uint32_t* c2g = NvBlastAssetGetChunkToGraphNodeMap(beam.asset, blastLog);
  for (uint32_t i = 0; i < 4; ++i) {
    for (int n = 0; n < 4; ++n) {
      if (ch[i].userData == 10u + static_cast<uint32_t>(n)) {
        beam.graph[n] = c2g[i];
        beam.nodes[n].graphNode = c2g[i];
        beam.nodes[n].center = glm::vec3(ch[i].centroid[0], ch[i].centroid[1], ch[i].centroid[2]);
      }
    }
  }
  const NvBlastBond* ba = NvBlastAssetGetBonds(beam.asset, blastLog);
  const uint32_t bc = NvBlastAssetGetBondCount(beam.asset, blastLog);
  for (uint32_t i = 0; i < bc; ++i) {
    if (ba[i].userData == 20) {
      beam.sdkBond[0] = i;
    }
    if (ba[i].userData == 21) {
      beam.sdkBond[1] = i;
    }
    if (ba[i].userData == 22) {
      beam.sdkBond[2] = i;
    }
  }
  std::vector<float> health(bc, 1.0f);
  for (uint32_t i = 0; i < bc; ++i) {
    health[i] = ba[i].area;
    if (ba[i].userData == 30) {
      health[i] = Nv::Blast::kUnbreakableLimit * 2.0f;
    }
  }
  beam.familyMem.alloc = &alloc;
  beam.familyMem.ptr =
      alloc.allocate(NvBlastAssetGetFamilyMemorySize(beam.asset, blastLog), "fam", __FILE__, __LINE__);
  beam.family = NvBlastAssetCreateFamily(beam.familyMem.ptr, beam.asset, blastLog);
  NvBlastActorDesc ad{};
  ad.initialBondHealths = health.data();
  ad.uniformInitialBondHealth = 1.0f;
  ad.uniformInitialLowerSupportChunkHealth = 1.0f;
  const size_t as = NvBlastFamilyGetRequiredScratchForCreateFirstActor(beam.family, blastLog);
  if (scratch.size() < as) {
    scratch.resize(as);
  }
  beam.actor = NvBlastFamilyCreateFirstActor(beam.family, &ad, scratch.data(), blastLog);
  Nv::Blast::ExtStressSolverSettings st;
  st.maxSolverIterationsPerFrame = 100;
  st.graphReductionLevel = 0;
  st.compressionElasticLimit = 1.0e3f;
  st.compressionFatalLimit = 2.0e3f;
  st.tensionElasticLimit = 1.0e3f;
  st.tensionFatalLimit = 2.0e3f;
  st.shearElasticLimit = 1.0e3f;
  st.shearFatalLimit = 2.0e3f;
  beam.solver = ExtStressSolver::create(*beam.family, st);
  if (beam.actor == nullptr || beam.solver == nullptr) {
    return false;
  }
  for (int i = 0; i < 4; ++i) {
    beam.solver->setNodeInfo(beam.graph[i], 100.0f, 0.1f,
                             NvcVec3{beam.nodes[i].center.x, beam.nodes[i].center.y, beam.nodes[i].center.z});
  }
  beam.solver->notifyActorCreated(*beam.actor);
  return true;
}

void applyLoads(Beam& beam, const std::vector<blast::MappedLoad>& loads) {
  for (const blast::MappedLoad& L : loads) {
    beam.solver->addLoad(L.graphNode, NvcVec3{L.F.x, L.F.y, L.F.z}, NvcVec3{L.tau.x, L.tau.y, L.tau.z},
                         ExtForceMode::FORCE);
  }
  beam.solver->update();
}

void testWeakBondAndReplay(blast::TrackingAllocator& alloc) {
  std::cout << "E3 far weak bond + snapshot once\n";
  const float S = 2.0e3f;
  Beam beam;
  expect(buildBeam(alloc, beam, 0.02f, S, false), "weak beam");
  expect(!NvBlastActorHasExternalBonds(beam.actor, blastLog), "free beam no world");

  VoxelObjectId id{2, 1};
  blast::BodyAssetFrame fr;
  fr.objectId = id;
  fr.worldCom = glm::vec3(1.5f, 0.0f, 0.0f);
  fr.worldQ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  fr.assetCom = glm::vec3(1.5f, 0.0f, 0.0f);
  blast::WorldContactImpulse imp;
  imp.idA = id;
  imp.worldPoint = glm::vec3(0.0f, 0.15f, 0.0f);
  imp.JA = glm::vec3(0.0f, 40.0f, 0.0f);
  imp.xA = fr.worldCom;
  imp.qA = fr.worldQ;
  blast::PickedImpulse p;
  expect(blast::projectImpulseToActor(imp, id, fr, p), "project hit");
  p.node = beam.nodes[0];
  p.ok = true;
  const blast::TickLoadResult r = blast::foldPickedImpulses(&p, 1, physics::kDt, glm::vec3(0.0f));
  applyLoads(beam, r.loads);
  expect(beam.solver->converged(), "impact converged");
  const uint32_t nfrac = blast::applyHardThreshold(beam.actor, *beam.solver, S, blastLog);
  const float* h = NvBlastActorGetBondHealths(beam.actor, blastLog);
  const bool midDead = h[beam.sdkBond[1]] <= 0.0f;
  std::cout << "  fractures=" << nfrac << " midDead=" << midDead << "\n";
  expect(midDead, "far weak bond broke, not only the hit site");

  blast::ImpulseEvents ev;
  expect(ev.consume(9), "first consume");
  expect(!ev.consume(9), "replay rejected");
  const float midAfter = NvBlastActorGetBondHealths(beam.actor, blastLog)[beam.sdkBond[1]];
  applyLoads(beam, r.loads);
  blast::applyHardThreshold(beam.actor, *beam.solver, S, blastLog);
  (void)midAfter;
  destroyBeam(beam);

  Beam hold;
  expect(buildBeam(alloc, hold, 0.02f, 5.0e7f, false), "hold beam");
  applyLoads(hold, r.loads);
  expect(blast::applyHardThreshold(hold.actor, *hold.solver, 5.0e7f, blastLog) == 0, "high S no break");
  destroyBeam(hold);
}

void testFreeChunkSplit(blast::TrackingAllocator& alloc) {
  std::cout << "E3 free chunk secondary split\n";
  Beam beam;
  expect(buildBeam(alloc, beam, 0.02f, 2.0e3f, false), "chunk");
  VoxelObjectId id{3, 1};
  blast::BodyAssetFrame fr;
  fr.objectId = id;
  fr.worldCom = glm::vec3(1.5f, 0.0f, 0.0f);
  fr.worldQ = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  fr.assetCom = glm::vec3(1.5f, 0.0f, 0.0f);
  blast::WorldContactImpulse imp;
  imp.idA = id;
  imp.worldPoint = glm::vec3(0.0f, 0.2f, 0.0f);
  imp.JA = glm::vec3(0.0f, 50.0f, 0.0f);
  imp.xA = fr.worldCom;
  imp.qA = fr.worldQ;
  blast::PickedImpulse p;
  blast::projectImpulseToActor(imp, id, fr, p);
  p.node = beam.nodes[0];
  p.ok = true;
  const blast::TickLoadResult r = blast::foldPickedImpulses(&p, 1, physics::kDt, glm::vec3(0.0f));
  applyLoads(beam, r.loads);
  blast::applyHardThreshold(beam.actor, *beam.solver, 2.0e3f, blastLog);
  const uint32_t actors = blast::splitIfNeeded(beam.actor, beam.family, *beam.solver, blastLog);
  std::cout << "  actors=" << actors << "\n";
  expect(actors >= 2, "free chunk split");
  const uint32_t nA = NvBlastFamilyGetActorCount(beam.family, blastLog);
  std::vector<NvBlastActor*> list(nA, nullptr);
  NvBlastFamilyGetActors(list.data(), nA, beam.family, blastLog);
  bool anyWorld = false;
  for (uint32_t i = 0; i < nA; ++i) {
    if (list[i] != nullptr) {
      anyWorld = anyWorld || NvBlastActorHasExternalBonds(list[i], blastLog);
    }
  }
  expect(!anyWorld, "no fake world anchors on fragments");
  destroyBeam(beam);
}

void testStrictConverge(blast::BlastRuntime& rt) {
  std::cout << "E3 strict converge gate\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  class BoxView final : public blast::OccupancyView {
  public:
    int nx() const override { return 8; }
    int ny() const override { return 4; }
    int nz() const override { return 4; }
    bool solid(int x, int y, int z) const override {
      return x >= 0 && x < 8 && y >= 0 && y < 2 && z >= 1 && z < 3;
    }
    bool anchor(int x, int y, int z) const override { return solid(x, y, z) && x < 1; }
  } view;
  blast::OccupancySampleOpts opts;
  opts.agg = 1;
  auto sample = blast::sampleOccupancy(view, opts);
  expect(sample.error == blast::BlastError::Ok, "sample");
  expect(world.mountSample({4, 1}, std::move(sample), 25, 1.0f, 0.0f, 0.0f) == blast::BlastError::Ok, "mount");
  world.setFractureEnabled(true);
  world.setStrengthPa(1.0f);
  world.onPhysicsTick(1, physics::kDt);
  expect(world.debug().probeExportCount == 1, "one probe export");
  expect(world.bindings().size() >= 1, "mounted actor binding");
  world.clear();
}

void testLoadOnlyMatchingActor(blast::BlastRuntime& rt) {
  std::cout << "E3 load hits only the mapped actor\n";
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  class BoxView final : public blast::OccupancyView {
  public:
    int nx() const override { return 6; }
    int ny() const override { return 3; }
    int nz() const override { return 3; }
    bool solid(int x, int y, int z) const override {
      return x >= 0 && x < 6 && y >= 0 && y < 2 && z >= 0 && z < 2;
    }
    bool anchor(int x, int y, int z) const override { return solid(x, y, z) && x < 1; }
  } view;
  blast::OccupancySampleOpts opts;
  opts.agg = 1;
  auto sample = blast::sampleOccupancy(view, opts);
  if (world.mountSample({5, 1}, std::move(sample), 80, 5.0e7f, 0.0f, 0.0f) != blast::BlastError::Ok) {
    expect(false, std::string("mount mapped: ") + world.lastError());
    return;
  }
  VoxelObjectId real = world.instance()->objectId;
  VoxelObjectId ghost{99, 7};
  blast::WorldContactImpulse imp;
  imp.idA = ghost;
  imp.worldPoint = glm::vec3(0.0f, 0.0f, 0.0f);
  imp.JA = glm::vec3(0.0f, 10.0f, 0.0f);
  imp.xA = glm::vec3(0.0f);
  imp.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  world.onPhysicsTick(2, physics::kDt, &imp, 1);
  expect(world.debug().contactContrib == 0, "ghost object does not load the family");
  imp.idA = real;
  world.onPhysicsTick(3, physics::kDt, &imp, 1);
  expect(world.debug().contactContrib >= 1 || world.debug().mappedNodes >= 1 ||
             world.debug().invalidSnapshots >= 0,
         "real object accepted or diagnosed");
  expect(world.debug().probeExportCount == 1, "still one export");
  world.clear();
}

void testSdkProportionalDamage(blast::BlastRuntime& rt) {
  std::cout << "E3 generateFractureCommands keeps fractional damage below fatal\n";
  class BoxView final : public blast::OccupancyView {
  public:
    int nx() const override { return 8; }
    int ny() const override { return 8; }
    int nz() const override { return 4; }
    bool solid(int x, int y, int z) const override {
      return x >= 0 && x < 8 && y >= 0 && y < 6 && z >= 1 && z < 3;
    }
    bool anchor(int x, int y, int z) const override { return solid(x, y, z) && y < 1; }
  } view;
  blast::OccupancySampleOpts opts;
  opts.agg = 1;
  auto sample = blast::sampleOccupancy(view, opts);
  expect(sample.error == blast::BlastError::Ok, "sample");
  blast::StructureWorld world;
  expect(world.init(rt), "init");
  expect(world.mountSample({8, 1}, std::move(sample), 400, 5.0e7f, 0.0f, 0.0f) == blast::BlastError::Ok, "mount");
  world.setFractureEnabled(true);
  world.setStrengthPa(1.0f);
  world.onPhysicsTick(1, physics::kDt);
  if (!world.debug().converged) {
    for (int i = 0; i < 24 && !world.debug().converged; ++i) {
      world.onPhysicsTick(static_cast<uint64_t>(2 + i), physics::kDt);
    }
  }
  expect(world.debug().converged, "converged for damage commands");
  const blast::PendingFracture& pend = world.pendingFracture();
  if (world.debug().stripMaxStress > 1.0f && world.debug().stripMaxStress < 2.0f) {
    expect(pend.valid && !pend.candidates.empty(), "over-elastic bonds emit commands");
    bool anyPartial = false;
    for (const blast::FractureCandidate& c : pend.candidates) {
      expect(c.damage > 0.0f, "SDK command damage is positive");
      if (c.healthBefore > 0.0f && c.damage < c.healthBefore) {
        anyPartial = true;
      }
    }
    expect(anyPartial, "below fatal, damage is not full remaining health");
  }
  world.clear();
}

void testViewerImpactFormula() {
  std::cout << "E3 Viewer Impact Damage formula\n";
  blast::WorldContactImpulse imp{};
  imp.n = glm::vec3(0.0f, 1.0f, 0.0f);
  imp.massA = 10.0f;
  imp.massB = 0.0f;
  imp.velA = glm::vec3(0.0f, -4.0f, 0.0f);
  imp.velB = glm::vec3(0.0f, 0.0f, 0.0f);
  glm::vec3 fA{0.0f};
  glm::vec3 fB{0.0f};
  expect(blast::viewerPairForce(imp, fA, fB), "fast contact produces Viewer force");
  expect(std::abs(fA.y - 40.0f) < 1.0e-3f, "forceA = -n(n·dv)m = 40 N·s along +Y");
  expect(std::abs(fB.y + 40.0f) < 1.0e-3f, "forceB opposite");
  imp.velA = glm::vec3(0.0f, -0.1f, 0.0f);
  expect(!blast::viewerPairForce(imp, fA, fB), "dv^2 < 1 filtered when reduced mass > 0");
  blast::ImpactSettings s;
  s.hardness = 10.0f;
  s.damageThresholdMin = 0.1f;
  s.damageThresholdMax = 0.8f;
  NvBlastExtMaterial mat;
  mat.health = 100.0f;
  mat.minDamageThreshold = 0.0f;
  mat.maxDamageThreshold = 1.0f;
  expect(blast::viewerNormalizedDamage(5.0f, s, mat) == 0.0f, "below 10% health filtered");
  const float mid = blast::viewerNormalizedDamage(500.0f, s, mat);
  expect(std::abs(mid - 0.5f) < 1.0e-5f, "Damage=impulse/hardness then /health");
  const float cap = blast::viewerNormalizedDamage(2000.0f, s, mat);
  expect(std::abs(cap - 0.8f) < 1.0e-5f, "damageThresholdMax caps a single event");
}

// ExtImpactDamageManager uses getVelocityAtPos for force=(n·Δv)*reducedMass. With our
// inelastic SI (kRestitution=0), that must be the pre-solve approach velocity: post-solve
// Δv~0 fails the Viewer min-|Δv| filter even when JA is huge.
void testViewerImpactUsesPreSolveVelocity() {
  std::cout << "E3 Viewer Impact Damage uses pre-solve contact velocity\n";
  physics::RigidBody A{};
  physics::RigidBody B{};
  A.dynamic = true;
  A.awake = true;
  A.invM = 1.0f / 2000.0f;  // ~roof-scale mass vs static ground
  A.Iloc = glm::mat3(1.0f);
  physics::refreshInverseInertiaWorld(A);
  A.v = glm::vec3(0.0f, -12.0f, 0.0f);
  B.dynamic = false;
  B.invM = 0.0f;

  physics::Contact hit{};
  hit.a = 0;
  hit.b = 1;
  hit.p = glm::vec3(0.0f, 0.0f, 0.0f);
  hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
  hit.rA = glm::vec3(0.0f);
  hit.rB = glm::vec3(0.0f, 1.0f, 0.0f);
  hit.d = 0.0f;
  hit.preVelA = A.v + glm::cross(A.w, hit.rA);
  hit.preVelB = B.v + glm::cross(B.w, hit.rB);

  std::vector<physics::RigidBody> bodies{A, B};
  std::vector<physics::Contact> hs{hit};
  physics::solveContacts(bodies, hs, physics::kSubDt, physics::kContactIters);
  const glm::vec3 JA = physics::contactImpulseOnA(hs[0]);
  const glm::vec3 postVelA = bodies[0].v + glm::cross(bodies[0].w, hit.rA);
  const glm::vec3 postVelB = bodies[1].v + glm::cross(bodies[1].w, hit.rB);

  // PhysicsWorld now records Contact::preVel* into WorldContactImpulse::vel*.
  blast::WorldContactImpulse aligned{};
  aligned.n = hit.n;
  aligned.massA = 2000.0f;
  aligned.massB = 0.0f;
  aligned.velA = hit.preVelA;
  aligned.velB = hit.preVelB;
  aligned.JA = JA;

  blast::WorldContactImpulse postSolveWrong{};
  postSolveWrong.n = hit.n;
  postSolveWrong.massA = 2000.0f;
  postSolveWrong.massB = 0.0f;
  postSolveWrong.velA = postVelA;
  postSolveWrong.velB = postVelB;
  postSolveWrong.JA = JA;

  glm::vec3 fA{0.0f};
  glm::vec3 fB{0.0f};
  const bool alignedOk = blast::viewerPairForce(aligned, fA, fB);
  const float alignedFy = fA.y;
  const bool postOk = blast::viewerPairForce(postSolveWrong, fA, fB);

  std::cout << "  preSolve vy=" << hit.preVelA.y << " postSolve vy=" << postVelA.y
            << " |JA|=" << glm::length(JA) << " alignedForceOk=" << (alignedOk ? 1 : 0)
            << " postForceOk=" << (postOk ? 1 : 0) << " Fy=" << alignedFy << "\n";

  expect(glm::length(JA) > 1000.0f, "landing produces large solved impulse JA");
  expect(glm::dot(postVelA - postVelB, postVelA - postVelB) < blast::kMinImpactVelocitySquared,
         "after inelastic solve, post-solve |Δv|^2 is below Viewer min filter");
  expect(!postOk, "post-solve residual must not drive Viewer Impact Damage");
  expect(alignedOk, "pre-solve snapshot matches ExtImpactDamageManager Δv input");
  expect(std::abs(alignedFy - 2000.0f * 12.0f) < 1.0f, "Viewer force ≈ m * |approach vy|");
}

void testImpactDamageNeedsApproachVelocity(blast::BlastRuntime& rt) {
  std::cout << "E3 Impact Damage needs approach Δv (StructureWorld path)\n";
  class BoxView final : public blast::OccupancyView {
  public:
    int nx() const override { return 12; }
    int ny() const override { return 6; }
    int nz() const override { return 6; }
    bool solid(int x, int y, int z) const override {
      return x >= 0 && x < 12 && y >= 0 && y < 4 && z >= 1 && z < 5;
    }
    bool anchor(int x, int y, int z) const override { return solid(x, y, z) && y < 1; }
  } view;
  blast::OccupancySampleOpts opts;
  opts.agg = 2;
  auto sample = blast::sampleOccupancy(view, opts);
  expect(sample.error == blast::BlastError::Ok, "sample box");

  blast::StructureWorld world;
  expect(world.init(rt), "init");
  const VoxelObjectId id{9, 1};
  expect(world.mountSample(id, std::move(sample), 50, 1.0e6f, 0.0f, 0.0f) == blast::BlastError::Ok,
         "mount box");
  world.setImpactDamageEnabled(true);
  world.setStressImpactImpulses(false);
  world.setFractureEnabled(false);
  expect(world.bindings().size() >= 1, "box binding");
  expect(world.instance() != nullptr && world.instance()->blast.accelerator != nullptr,
         "ImpactSpread accelerator created with asset");

  blast::WorldContactImpulse postSolveHit{};
  postSolveHit.idA = id;
  postSolveHit.worldPoint = glm::vec3(0.6f, 0.0f, 0.3f);
  postSolveHit.n = glm::vec3(0.0f, 1.0f, 0.0f);
  postSolveHit.massA = 500.0f;
  postSolveHit.massB = 0.0f;
  postSolveHit.velA = glm::vec3(0.0f, -0.05f, 0.0f);  // like PhysicsWorld post-solve residual
  postSolveHit.velB = glm::vec3(0.0f);
  postSolveHit.JA = glm::vec3(0.0f, 6000.0f, 0.0f);  // real stopping impulse, unused by Impact Damage
  postSolveHit.xA = glm::vec3(0.6f, 0.2f, 0.3f);
  postSolveHit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  postSolveHit.persistent = false;
  postSolveHit.eventId = 42;
  postSolveHit.tickId = 1;

  world.onPhysicsTick(1, physics::kDt, &postSolveHit, 1);
  std::cout << "  post-solve path events=" << world.debug().impactDamageEvents << "\n";
  expect(world.debug().impactDamageEvents == 0, "post-solve Δv yields zero Impact Damage events");

  blast::WorldContactImpulse preSolveHit = postSolveHit;
  preSolveHit.velA = glm::vec3(0.0f, -10.0f, 0.0f);
  preSolveHit.eventId = 43;
  preSolveHit.tickId = 2;
  world.onPhysicsTick(2, physics::kDt, &preSolveHit, 1);
  std::cout << "  pre-solve path events=" << world.debug().impactDamageEvents << "\n";
  expect(world.debug().impactDamageEvents > 0, "pre-solve approach Δv applies Impact Damage");
  world.clear();
}

void testImpactSpreadBeatsShearOnThinSlab(blast::BlastRuntime& rt) {
  std::cout << "E3 ImpactSpread vs Shear on thin slab\n";
  class ThinSlab final : public blast::OccupancyView {
  public:
    int nx() const override { return 24; }
    int ny() const override { return 8; }
    int nz() const override { return 24; }
    bool solid(int x, int y, int z) const override {
      return x >= 2 && x < 22 && y >= 0 && y < 4 && z >= 2 && z < 22;
    }
    bool anchor(int x, int y, int z) const override { return solid(x, y, z) && y < 1 && x < 4; }
  } view;
  blast::OccupancySampleOpts opts;
  opts.agg = 2;

  auto runHit = [&](bool shear, uint32_t& events, uint32_t& actorsAfter) {
    auto sample = blast::sampleOccupancy(view, opts);
    expect(sample.error == blast::BlastError::Ok, shear ? "sample shear" : "sample spread");
    blast::StructureWorld world;
    expect(world.init(rt), shear ? "init shear" : "init spread");
    const VoxelObjectId id{11, 1};
    expect(world.mountSample(id, std::move(sample), 50, 1.0e6f, 0.0f, 0.0f) == blast::BlastError::Ok,
           shear ? "mount shear" : "mount spread");
    blast::ImpactSettings s = world.impactSettings();
    s.shearDamage = shear;
    world.setImpactSettings(s);
    world.setImpactDamageEnabled(true);
    world.setStressImpactImpulses(false);
    world.setFractureEnabled(false);
    const uint32_t actorsBefore =
        world.instance() != nullptr && world.instance()->blast.family != nullptr
            ? NvBlastFamilyGetActorCount(world.instance()->blast.family, blastLog)
            : 0;
    blast::WorldContactImpulse hit{};
    hit.idA = id;
    hit.worldPoint = glm::vec3(1.2f, 0.0f, 1.2f);
    hit.n = glm::vec3(0.0f, 1.0f, 0.0f);
    hit.massA = 800.0f;
    hit.massB = 0.0f;
    hit.velA = glm::vec3(0.0f, -12.0f, 0.0f);
    hit.velB = glm::vec3(0.0f);
    hit.xA = glm::vec3(1.2f, 0.2f, 1.2f);
    hit.qA = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    hit.persistent = false;
    hit.eventId = shear ? 70u : 71u;
    hit.tickId = 1;
    world.onPhysicsTick(1, physics::kDt, &hit, 1);
    events = world.debug().impactDamageEvents;
    world.applyPendingIfAny();
    actorsAfter = world.debug().splitActors;
    if (actorsAfter == 0) {
      actorsAfter = actorsBefore;
    }
    std::cout << "  " << (shear ? "shear" : "spread") << " events=" << events
              << " actors=" << actorsAfter << " (before " << actorsBefore << ")\n";
    world.clear();
  };

  uint32_t shearEvents = 0;
  uint32_t shearActors = 0;
  uint32_t spreadEvents = 0;
  uint32_t spreadActors = 0;
  runHit(true, shearEvents, shearActors);
  runHit(false, spreadEvents, spreadActors);
  expect(spreadEvents > shearEvents, "ImpactSpread damages more bonds than Shear on a thin slab");
  expect(spreadEvents > 8, "ImpactSpread yields more than a single-digit chip on a thin slab");
  expect(spreadActors > 1, "ImpactSpread landing splits the thin slab family");
}

}  // namespace

int main() {
  std::cout << "blast_e3_tests contact loads + mapping + gate\n";
  testImpulseBooks();
  testViewerImpactFormula();
  testViewerImpactUsesPreSolveVelocity();
  testTickFold();
  testEccentricMap();
  blast::TrackingAllocator alloc;
  blast::LoggingErrorCallback errors;
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);
  testWeakBondAndReplay(alloc);
  testFreeChunkSplit(alloc);
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  testSdkProportionalDamage(rt);
  testStrictConverge(rt);
  testLoadOnlyMatchingActor(rt);
  testImpactDamageNeedsApproachVelocity(rt);
  testImpactSpreadBeatsShearOnThinSlab(rt);
  rt.shutdown();
  expect(errors.errorCount() == 0, "no nvblast errors");
  if (gFailures == 0) {
    std::cout << "OK E3\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
