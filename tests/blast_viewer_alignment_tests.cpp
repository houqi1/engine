#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace {
int failures = 0;
void expect(bool value, const char* message) {
  if (!value) { ++failures; std::cerr << "FAIL: " << message << '\n'; }
}

// Partial boundary cells produce different bond areas: the same Viewer damage
// fraction must remove the same fraction of each, regardless of discretization.
class Bar final : public blast::OccupancyView {
 public:
  int nx() const override { return 8; }
  int ny() const override { return 4; }
  int nz() const override { return 4; }
  bool solid(int x, int y, int z) const override {
    return x >= 0 && x < 8 && y >= 0 && y < 3 && z >= 0 && z < 3;
  }
  bool anchor(int x, int y, int z) const override { return solid(x, y, z) && x == 0; }
};

constexpr VoxelObjectId body{3, 1};
bool mount(blast::StructureWorld& world, blast::BlastRuntime& rt, bool shear = false) {
  Bar bar;
  blast::OccupancySampleOpts opts;
  opts.agg = 2;
  if (!world.init(rt) || world.mountSample(body, blast::sampleOccupancy(bar, opts), 100,
                                          5.0e7f, 0.0f, 0.0f) != blast::BlastError::Ok) return false;
  blast::ImpactSettings settings;
  settings.shearDamage = shear;
  settings.damageRadiusMax = 100.0f; // All bonds receive the same radial fraction.
  world.setImpactSettings(settings);
  world.setFractureEnabled(false);
  return true;
}

blast::WorldContactImpulse hit(const blast::StructureWorld& world, float x = 0.5f) {
  blast::WorldContactImpulse contact;
  contact.idA = body;
  contact.idB = {0, 1};
  contact.worldPoint = {x, 0.1f, 0.1f};
  contact.xA = world.bindings().front().comAsset;
  contact.massA = 125.0f;
  contact.velA = {0.0f, -2.0f, 0.0f}; // 250 / hardness 10 / health 100 = 0.25.
  contact.n = {0.0f, 1.0f, 0.0f};
  return contact;
}

void checkFractions(blast::StructureWorld& world, float remaining) {
  const auto* inst = world.instance();
  const auto* bonds = NvBlastAssetGetBonds(inst->blast.asset, nullptr);
  const float* health = NvBlastActorGetBondHealths(inst->blast.actor, nullptr);
  int checked = 0;
  float minArea = 1.0f, maxArea = 0.0f;
  for (uint32_t i = 0; i < inst->bondMeta.size(); ++i) {
    if (inst->bondMeta[i].world) continue;
    expect(std::abs(health[i] / bonds[i].area - remaining) < 0.0001f,
           "Viewer normalized damage preserves remaining-health fraction");
    minArea = std::min(minArea, bonds[i].area);
    maxArea = std::max(maxArea, bonds[i].area);
    ++checked;
  }
  expect(checked > 1 && minArea < maxArea, "fixture covers unequal bond areas");
}

void testContactGroups(blast::BlastRuntime& rt) {
  for (int scenario = 0; scenario < 5; ++scenario) {
    blast::StructureWorld world;
    expect(mount(world, rt), "mount contact fixture");
    auto a = hit(world);
    auto b = a;
    if (scenario == 1) b = hit(world, 0.7f); // Separate chunk/shape: separate damage event.
    if (scenario == 2) b.substep = 1;        // Separate simulation step: do not average away.
    if (scenario == 3) {
      std::swap(b.idA, b.idB);
      std::swap(b.massA, b.massB);
      std::swap(b.velA, b.velB);
      std::swap(b.xA, b.xB);
      std::swap(b.qA, b.qB);
      b.n = -b.n;
    }
    if (scenario == 4) b.velA = {0.0f, -0.1f, 0.0f}; // Rejected points don't dilute mean.
    const blast::WorldContactImpulse contacts[]{a, b};
    world.onPhysicsTick(1, physics::kDt, contacts, 2);
    checkFractions(world, scenario == 1 || scenario == 2 ? 0.5f : 0.75f);
    world.clear();
  }
}

void testAccumulationAndRouting(blast::BlastRuntime& rt) {
  blast::StructureWorld world;
  expect(mount(world, rt), "mount accumulation fixture");
  auto contact = hit(world);
  world.onPhysicsTick(1, physics::kDt, &contact, 1);
  checkFractions(world, 0.75f);
  world.onPhysicsTick(2, physics::kDt, &contact, 1);
  checkFractions(world, 0.5f); // Multiply by initial area, not already damaged health.
  world.setStressImpactImpulses(true);
  world.setStressImpactScale(0.0f);
  expect(world.stressImpactScale() == 0.0f, "Viewer stress multiplier accepts zero");
  world.onPhysicsTick(3, physics::kDt, &contact, 1);
  checkFractions(world, 0.5f); // Stress callback replaces shader.
  world.setImpactDamageEnabled(false);
  world.setStressImpactImpulses(false);
  world.onPhysicsTick(4, physics::kDt, &contact, 1);
  checkFractions(world, 0.5f);
  world.clear();
}

void testShaderFractionEquivalence(blast::BlastRuntime& rt) {
    // Run the unmodified NVIDIA shader to obtain normalized damage, then compare
  // our area-health adapter for both programs on exactly the same geometry.
  for (bool shear : {false, true}) {
    blast::StructureWorld world;
    expect(mount(world, rt, shear), "mount shader fixture");
    auto* inst = world.instance();
    const uint32_t nb = NvBlastAssetGetBondCount(inst->blast.asset, nullptr);
    const uint32_t nc = NvBlastAssetGetChunkCount(inst->blast.asset, nullptr);
    std::vector<NvBlastBondFractureData> bonds(nb);
    std::vector<NvBlastChunkFractureData> chunks(nc);
    NvBlastFractureBuffers commands{nb, nc, bonds.data(), chunks.data()};
    const size_t familySize = NvBlastAssetGetFamilyMemorySize(inst->blast.asset, nullptr);
    std::vector<char> familyMemory(familySize + 15);
    void* aligned = reinterpret_cast<void*>((reinterpret_cast<uintptr_t>(familyMemory.data()) + 15) & ~uintptr_t(15));
    NvBlastFamily* referenceFamily = NvBlastAssetCreateFamily(aligned, inst->blast.asset, nullptr);
    std::vector<float> initialHealth(nb, 1.0f);
    const float* originalHealth = NvBlastActorGetBondHealths(inst->blast.actor, nullptr);
    for (uint32_t i = 0; i < nb; ++i)
      if (inst->bondMeta[i].world) initialHealth[i] = originalHealth[i];
    NvBlastActorDesc actorDesc{};
    actorDesc.initialBondHealths = initialHealth.data();
    actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;
    std::vector<char> scratch(NvBlastFamilyGetRequiredScratchForCreateFirstActor(referenceFamily, nullptr));
    NvBlastActor* referenceActor = NvBlastFamilyCreateFirstActor(referenceFamily, &actorDesc, scratch.data(), nullptr);
    auto contact = hit(world);
    NvBlastExtShearDamageDesc sd{0.25f, {0, 1, 0}, {0.5f, 0.1f, 0.1f}, 25.0f, 50.0f};
    NvBlastExtImpactSpreadDamageDesc rd{0.25f, {0.5f, 0.1f, 0.1f}, 25.0f, 50.0f};
    NvBlastExtProgramParams params(shear ? static_cast<void*>(&sd) : static_cast<void*>(&rd),
                                 &world.impactMaterial(), inst->blast.accelerator);
    NvBlastDamageProgram program{};
    program.graphShaderFunction = shear ? NvBlastExtShearGraphShader : NvBlastExtImpactSpreadGraphShader;
    program.subgraphShaderFunction = shear ? NvBlastExtShearSubgraphShader : NvBlastExtImpactSpreadSubgraphShader;
    NvBlastActorGenerateFracture(&commands, referenceActor, program, &params, nullptr, nullptr);
    NvBlastActorApplyFracture(nullptr, referenceActor, &commands, nullptr, nullptr);
    const float* expected = NvBlastActorGetBondHealths(referenceActor, nullptr);
    world.onPhysicsTick(1, physics::kDt, &contact, 1);
    const auto* assetBonds = NvBlastAssetGetBonds(inst->blast.asset, nullptr);
    const auto* health = NvBlastActorGetBondHealths(inst->blast.actor, nullptr);
    for (uint32_t i = 0; i < nb; ++i) {
      if (!inst->bondMeta[i].world) {
        if (std::abs(health[i] / assetBonds[i].area - std::max(0.0f, expected[i])) >= 0.0001f)
          std::cerr << "shader=" << shear << " bond=" << i << " actual=" << health[i] / assetBonds[i].area
                    << " expected=" << expected[i] << '\n';
        expect(std::abs(health[i] / assetBonds[i].area - std::max(0.0f, expected[i])) < 0.0001f,
               "NVIDIA shader and area-health adapter agree bond by bond");
      }
    }
    NvBlastActorDeactivate(referenceActor, nullptr);
    world.clear();
  }
}
} // namespace

int main() {
  blast::ImpactSettings defaults;
  expect(defaults.shearDamage && !defaults.selfCollisionEnabled, "Viewer default shader and self collision");
  blast::BlastRuntime rt;
  if (!rt.init()) return 1;
  testContactGroups(rt);
  testAccumulationAndRouting(rt);
  testShaderFractionEquivalence(rt);
  expect(rt.errorCount() == 0, "no Blast errors");
  rt.shutdown();
  if (!failures) std::cout << "OK Viewer damage fractions, shape pairs, substeps, routing and shaders\n";
  return failures ? 1 : 0;
}
