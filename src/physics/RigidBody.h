#pragma once

#include "physics/PhysicsTypes.h"

namespace physics {

struct RigidBody {
  int shapeIndex = -1;
  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  float invM = 0.0f;
  glm::mat3 Iloc{1.0f};
  glm::mat3 IinvW{0.0f};
  bool dynamic = false;
  bool awake = false;
  float sleepTimer = 0.0f;
  uint32_t occupiedFine = 0;
};

}  // namespace physics
