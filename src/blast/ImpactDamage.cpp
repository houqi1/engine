#include "blast/ImpactDamage.h"
#include "NvBlast.h"

#include <algorithm>
#include <cmath>

namespace blast {

bool viewerPairForce(const WorldContactImpulse& imp, glm::vec3& forceA, glm::vec3& forceB) {
  const float m0 = imp.massA;
  const float m1 = imp.massB;
  float reducedMass = 0.0f;
  if (m0 == 0.0f) {
    reducedMass = m1;
  } else if (m1 == 0.0f) {
    reducedMass = m0;
  } else {
    reducedMass = m0 * m1 / (m0 + m1);
  }
  const glm::vec3 dv = imp.velA - imp.velB;
  if (glm::dot(dv, dv) < kMinImpactVelocitySquared && reducedMass != 0.0f) {
    return false;
  }
  glm::vec3 n = imp.n;
  const float n2 = glm::dot(n, n);
  if (n2 > 1.0e-12f) {
    n /= std::sqrt(n2);
  }
  const glm::vec3 fn = n * glm::dot(n, dv) * reducedMass;
  forceA = -fn;
  forceB = fn;
  return true;
}

float viewerNormalizedDamage(float forceMag, const ImpactSettings& settings, const NvBlastExtMaterial& material) {
  const float damage = settings.hardness > 0.0f ? forceMag / settings.hardness : 0.0f;
  float normalized = material.getNormalizedDamage(damage);
  if (normalized == 0.0f || normalized < settings.damageThresholdMin) {
    return 0.0f;
  }
  if (normalized > settings.damageThresholdMax) {
    normalized = settings.damageThresholdMax;
  }
  if (normalized < 0.0f) {
    normalized = 0.0f;
  }
  return normalized;
}

void viewerDamageToBondArea(NvBlastFractureBuffers& commands, const NvBlastAsset* asset) {
  const NvBlastSupportGraph graph = NvBlastAssetGetSupportGraph(asset, nullptr);
  const NvBlastBond* bonds = NvBlastAssetGetBonds(asset, nullptr);
  for (uint32_t i = 0; i < commands.bondFractureCount; ++i) {
    auto& command = commands.bondFractures[i];
    bool found = false;
    if (command.nodeIndex0 < graph.nodeCount) {
      for (uint32_t adj = graph.adjacencyPartition[command.nodeIndex0];
           adj < graph.adjacencyPartition[command.nodeIndex0 + 1]; ++adj) {
        if (graph.adjacentNodeIndices[adj] == command.nodeIndex1) {
          command.health *= bonds[graph.adjacentBondIndices[adj]].area;
          found = true;
          break;
        }
      }
    }
    if (!found) command.health = 0.0f;
  }
  // Support chunk health already uses the Viewer's unit-health convention.
}

}  // namespace blast
