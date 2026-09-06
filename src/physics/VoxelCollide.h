#pragma once

#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"

#include <vector>

class VoxelScene;

namespace physics {

void collidePair(VoxelScene& scene, const RigidBody& a, const RigidBody& b, const ShapeClass& ca,
                 const ShapeClass& cb, std::vector<Contact>& out);

struct DebugCornerNormal {
  glm::vec3 p{0.0f};
  glm::vec3 n{0.0f, 1.0f, 0.0f};
  float d = 0.0f;
  bool hit = false;
};

void gatherCornerNormals(const VoxelScene& scene, const ShapeClass& fromClass, int fromObj,
                         int againstObj, std::vector<DebugCornerNormal>& out);

}  // namespace physics
