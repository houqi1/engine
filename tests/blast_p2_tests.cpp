#include "blast/ActorBodies.h"
#include "blast/BlastMemory.h"
#include "blast/CylinderGraph.h"
#include "blast/HardFracture.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastGlobals.h"

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
    ++gFailures;
  }
}

blast::BodySnapshot snapshotFree(const blast::SplitWorld& world) {
  blast::BodySnapshot s;
  for (const auto& b : world.bodies) {
    if (!b.anchored) {
      s.x = b.x;
      s.q = b.q;
      s.v = b.v;
      s.w = b.w;
      s.comAsset = b.x;
      return s;
    }
  }
  if (!world.bodies.empty()) {
    const auto& b = world.bodies[0];
    s.x = b.x;
    s.q = b.q;
    s.v = b.v;
    s.w = b.w;
    s.comAsset = b.x;
  }
  return s;
}

void cutBonds(blast::CylinderScene& scene, const std::vector<uint32_t>& sdk) {
  blast::applySdkBondDamage(scene.actor, scene.asset, sdk.data(), static_cast<uint32_t>(sdk.size()), blastLog);
  if (scene.solver) {
    scene.solver->syncBrokenBonds();
  }
  blast::splitIfNeeded(scene.actor, scene.family, *scene.solver, blastLog);
}

void testT07(blast::TrackingAllocator& alloc) {
  std::cout << "T07 last anchor path\n";
  blast::CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.solverIters = 50;
  blast::CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "T07 build");

  blast::SplitWorld world;
  blast::buildCylinderChunkShapes(scene.asset, opts.density, world.shapes);
  blast::rebuildBodiesFromFamily(scene.family, scene.asset, world, nullptr, blastLog);
  expect(world.bodies.size() == 1, "T07 start one body");
  expect(world.bodies[0].anchored, "T07 start anchored");
  const float m0 = blast::totalMass(world);

  std::vector<uint32_t> baseVert;
  for (int k = 0; k < blast::kCylinderCirc; ++k) {
    baseVert.push_back(scene.ids.sdkBondFromVert[static_cast<size_t>(k)]);
  }
  cutBonds(scene, baseVert);
  blast::rebuildBodiesFromFamily(scene.family, scene.asset, world, nullptr, blastLog);

  int nFree = 0;
  int nAnc = 0;
  float freeMass = 0.0f;
  glm::vec3 freeY0(0.0f);
  for (const auto& b : world.bodies) {
    if (b.anchored) {
      ++nAnc;
    } else {
      ++nFree;
      freeMass += b.mass;
      freeY0 = b.x;
    }
  }
  std::cout << "  actors=" << world.bodies.size() << " free=" << nFree << " anchored=" << nAnc
            << " freeMass=" << freeMass << "\n";
  expect(nFree >= 1, "T07 has a dynamic body");
  expect(nAnc >= 1, "T07 base ring still anchored");
  expect(!blast::hasGhostAnchor(world, blastLog), "T07 no ghost anchor");
  expect(std::abs(blast::totalMass(world) - m0) / m0 <= 0.001f, "T07 mass conserved");
  expect(std::abs(freeMass - (7.0f / 8.0f) * m0) / m0 <= 0.02f, "T07 free mass is upper shell");

  std::vector<glm::vec3> ancX0;
  for (const auto& b : world.bodies) {
    if (b.anchored) {
      ancX0.push_back(b.x);
    }
  }
  const glm::vec3 g(0.0f, blast::kGravityY, 0.0f);
  const float dt = 1.0f / 60.0f;
  for (int i = 0; i < 10; ++i) {
    blast::integrateDynamic(world, dt, g);
  }
  glm::vec3 freeY1 = freeY0;
  size_t ancI = 0;
  for (const auto& b : world.bodies) {
    if (!b.anchored) {
      freeY1 = b.x;
    } else {
      expect(glm::length(b.v) < 1.0e-6f, "T07 anchored velocity stays zero");
      if (ancI < ancX0.size()) {
        expect(glm::length(b.x - ancX0[ancI]) < 1.0e-6f, "T07 anchored does not move");
      }
      ++ancI;
    }
  }
  expect(freeY1.y < freeY0.y - 0.05f, "T07 free body falls");
  blast::destroyCylinderScene(scene);
}

void testT08T09(blast::TrackingAllocator& alloc) {
  std::cout << "T08/T09 rotating split + birth overlap\n";
  blast::CylinderBuildOpts opts;
  opts.strengthPa = blast::kStrengthHoldPa;
  opts.worldAnchor = false;
  blast::CylinderScene scene;
  expect(blast::buildCylinderScene(alloc, scene, opts), "T08 build");

  blast::SplitWorld world;
  blast::buildCylinderChunkShapes(scene.asset, opts.density, world.shapes);
  blast::rebuildBodiesFromFamily(scene.family, scene.asset, world, nullptr, blastLog);
  expect(world.bodies.size() == 1, "T08 one body");

  world.bodies[0].v = glm::vec3(0.4f, -0.2f, 0.1f);
  world.bodies[0].w = glm::vec3(0.3f, 1.2f, -0.5f);
  world.bodies[0].anchored = false;
  const blast::BodySnapshot parent = snapshotFree(world);
  const float m0 = blast::totalMass(world);
  const glm::vec3 p0 = blast::totalLinearMomentum(world);
  const glm::vec3 L0 = blast::totalAngularMomentumAbout(world, parent.x);

  std::vector<uint32_t> mid;
  const int jCut = 3;
  for (int k = 0; k < blast::kCylinderCirc; ++k) {
    mid.push_back(scene.ids.sdkBondFromVert[static_cast<size_t>(jCut * blast::kCylinderCirc + k)]);
  }
  cutBonds(scene, mid);
  blast::rebuildBodiesFromFamily(scene.family, scene.asset, world, &parent, blastLog);

  expect(world.bodies.size() >= 2, "T08 split into >= 2 bodies");
  const float m1 = blast::totalMass(world);
  const glm::vec3 p1 = blast::totalLinearMomentum(world);
  const glm::vec3 L1 = blast::totalAngularMomentumAbout(world, parent.x);
  const float relM = std::abs(m1 - m0) / m0;
  const float relP = glm::length(p1 - p0) / std::max(glm::length(p0), 1.0e-6f);
  const float relL = glm::length(L1 - L0) / std::max(glm::length(L0), 1.0e-6f);
  std::cout << "  bodies=" << world.bodies.size() << " relM=" << relM << " relP=" << relP << " relL=" << relL << "\n";
  expect(relM <= 0.001f, "T08 mass rel <= 0.1%");
  expect(relP <= 0.001f, "T08 linear momentum rel <= 0.1%");
  expect(relL <= 0.001f, "T08 angular momentum rel <= 0.1%");

  const float pen = blast::maxBirthPenetration(world);
  std::cout << "  T09 max birth penetration=" << pen << " m\n";
  expect(pen <= blast::kBirthOverlapSlop, "T09 no deep birth overlap");
  expect(!blast::hasGhostAnchor(world, blastLog), "T08/T09 no ghost anchor on unhooked pieces");

  // No explosion: relative COM velocity is only rigid-field inheritance.
  if (world.bodies.size() >= 2) {
    const auto& a = world.bodies[0];
    const auto& b = world.bodies[1];
    const glm::vec3 r = b.x - a.x;
    const glm::vec3 vRel = b.v - a.v;
    const glm::vec3 vRigid = glm::cross(parent.w, r);
    const float boom = glm::length(vRel - vRigid);
    std::cout << "  extra separation speed=" << boom << " m/s\n";
    expect(boom <= 1.0e-4f, "T09 no added explosion velocity");
  }
  blast::destroyCylinderScene(scene);
}

}  // namespace

int main() {
  std::cout << "blast_p2_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";

  blast::TrackingAllocator alloc;
  blast::LoggingErrorCallback errors;
  NvBlastGlobalSetAllocatorCallback(&alloc);
  NvBlastGlobalSetErrorCallback(&errors);

  testT07(alloc);
  testT08T09(alloc);

  expect(errors.errorCount() == 0, "no NvBlast error-callback errors");
  if (gFailures == 0) {
    std::cout << "OK P2 T07 T08 T09\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
