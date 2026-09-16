#pragma once

#include "physics/PhysicsTypes.h"

#include <vector>

namespace physics {

struct RigidBody {
  int shapeIndex = -1;
  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  // Occupancy COM in object local meters (grid origin). x is this point in world.
  glm::vec3 comLocal{0.0f};
  float invM = 0.0f;
  glm::mat3 Iloc{1.0f};
  glm::mat3 IinvW{0.0f};
  bool dynamic = false;
  bool awake = false;
  bool enableSleep = true;
  float sleepTimer = 0.0f;
  float extent = 0.0f;
  int sleepIsland = -1;
  glm::vec3 xTick0{0.0f};
  glm::quat qTick0{1.0f, 0.0f, 0.0f, 0.0f};
  std::vector<SleepSupport> supports;
  uint32_t occupiedFine = 0;
};

}  // namespace physics
