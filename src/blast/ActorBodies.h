#pragma once

#include "blast/CylinderGraph.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <vector>

namespace blast {

struct ChunkObb {
  uint32_t chunkIndex = 0;
  glm::vec3 center{0.0f};
  glm::vec3 axis[3]{glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1)};
  glm::vec3 half{0.0f};
  float mass = 0.0f;
  float volume = 0.0f;
};

struct ActorBody {
  uint32_t actorIndex = 0;
  NvBlastActor* actor = nullptr;
  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  float mass = 0.0f;
  glm::mat3 Ibody{1.0f};
  bool anchored = false;
  std::vector<uint32_t> chunks;
};

struct BodySnapshot {
  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  glm::vec3 comAsset{0.0f};
};

struct SplitWorld {
  std::vector<ChunkObb> shapes;
  std::vector<ActorBody> bodies;
};

void buildCylinderChunkShapes(const NvBlastAsset* asset, float density, std::vector<ChunkObb>& shapes);
void rebuildBodiesFromFamily(NvBlastFamily* family, const NvBlastAsset* asset, SplitWorld& world,
                             const BodySnapshot* inheritFrom, NvBlastLog logFn);
void inheritVelocity(ActorBody& child, const BodySnapshot& parent);
void integrateDynamic(SplitWorld& world, float dt, const glm::vec3& gravity);
float totalMass(const SplitWorld& world);
glm::vec3 totalLinearMomentum(const SplitWorld& world);
glm::vec3 totalAngularMomentumAbout(const SplitWorld& world, const glm::vec3& origin);
float maxBirthPenetration(const SplitWorld& world);
bool hasGhostAnchor(const SplitWorld& world, NvBlastLog logFn);

constexpr float kBirthOverlapSlop = 0.02f;
constexpr float kNoBoomSpeed = 0.0f;

}  // namespace blast
