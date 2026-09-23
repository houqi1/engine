#pragma once

#include "blast/ContactLoads.h"

#include "NvBlastExtDamageShaders.h"

#include <cstdint>

namespace blast {

enum class ImpactModel : uint8_t {
  // SampleAssetViewer: one event per (body pair, substep, touched node pair), each using the
  // whole-body reduced-mass force / hardness. Damage grows with contact count and body mass.
  Viewer,
  // One event per body pair per tick at the hardest-hit contact. Damage is the velocity change
  // of the damaged actor (force / actor mass) over fullDamageDeltaV, so it is scale invariant.
  BodyPair,
};

// NVIDIA SampleAssetViewer / ExtImpactSettings defaults (Blast 1.1.10), plus BodyPair model.
struct ImpactSettings {
  ImpactModel model = ImpactModel::BodyPair;
  // BodyPair only: actor velocity change (m/s) that maps to full material health damage.
  float fullDamageDeltaV = 25.0f;
  bool selfCollisionEnabled = false;
  bool shearDamage = true;
  float hardness = 10.0f;
  float damageRadiusMax = 2.0f;
  float damageThresholdMin = 0.1f;
  float damageThresholdMax = 1.0f;
  float damageFalloffRadiusFactor = 2.0f;
};

struct ImpactMaterial {
  NvBlastExtMaterial material{};
};

// ExtImpactDamageManager.cpp MIN_IMPACT_VELOCITY_SQUARED.
constexpr float kMinImpactVelocitySquared = 1.0f;

// ExtImpactDamageManager::onContact: force = (n·Δv) n * reducedMass, averaged per pair.
// velA/velB must be the contact-point velocities used for that formula (pre-solve
// approach velocity in this engine; post-solve residual is ~0 with kRestitution=0).
bool viewerPairForce(const WorldContactImpulse& imp, glm::vec3& forceA, glm::vec3& forceB);

float viewerNormalizedDamage(float forceMag, const ImpactSettings& settings, const NvBlastExtMaterial& material);

// BodyPair model: deltaV = |force| / damaged actor mass. deltaV == fullDamageDeltaV means
// damage equal to material.health; then the same material and settings thresholds apply.
float bodyPairNormalizedDamage(float deltaV, const ImpactSettings& settings, const NvBlastExtMaterial& material);

// Viewer bonds start at health 1. Blast 5 ExtStress instead stores remaining
// cross-sectional area in health. Preserve the Viewer's fractional health loss.
void viewerDamageToBondArea(NvBlastFractureBuffers& commands, const NvBlastAsset* asset);

}  // namespace blast
