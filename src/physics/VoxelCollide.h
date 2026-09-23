#pragma once

#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"

#include <vector>

class VoxelScene;

namespace physics {

struct CollisionPose {
  glm::vec3 min{0}, max{0};
  glm::mat4 toWorld{1}, toLocal{1};
  float fineSize = 0;
};

void collidePair(VoxelScene& scene, const RigidBody& a, const RigidBody& b, const ShapeClass& ca,
                 const ShapeClass& cb, std::vector<Contact>& out,
                 const CollisionPose* poseA = nullptr, const CollisionPose* poseB = nullptr);

bool worldAabb(const VoxelScene& scene, int objectIndex, const ShapeClass& sc, glm::vec3& wmn,
               glm::vec3& wmx);

bool worldPointHitsOccupancy(const VoxelScene& scene, int objectIndex, const glm::vec3& worldP);

struct DebugCornerNormal {
  glm::vec3 p{0.0f};
  glm::vec3 n{0.0f, 1.0f, 0.0f};
  float d = 0.0f;
  bool hit = false;
};

void gatherCornerNormals(const VoxelScene& scene, const ShapeClass& fromClass, int fromObj,
                         int againstObj, std::vector<DebugCornerNormal>& out);

}  // namespace physics
