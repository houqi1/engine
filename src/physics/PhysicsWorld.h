#pragma once

#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"
#include "scene/VoxelTypes.h"

#include <glm/glm.hpp>

#include <vector>

class VoxelScene;

namespace physics {

struct BodyState {
  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  bool awake = true;
  float sleepTimer = 0.0f;
};

class PhysicsWorld {
public:
  void attach(VoxelScene& scene);
  void rebuildFromScene();
  void markDirty(int objectIndex);
  void step(float frameDt);
  void syncTransformsToScene();

  bool getBodyState(VoxelObjectId id, BodyState& out) const;
  bool addBody(VoxelObjectId id, const BodyState& state);
  bool removeBody(VoxelObjectId id);
  // Reclassify/mass for an existing body. If stateOrNull is non-null, replaces pose/velocity.
  bool replaceShape(VoxelObjectId id, const BodyState* stateOrNull);
  void activateBodiesInBounds(const glm::vec3& worldMin, const glm::vec3& worldMax);

  const ShapeClass* shapeClass(int objectIndex) const;
  DebugSolve debugSolve() const { return debug_; }

private:
  void ensureBodyCapacity(int n);
  void initBodyFromObject(int objectIndex, RigidBody& body, bool preserveMotion);
  void rebuildDirty();
  void substep();
  void updateSleep(float h);

  VoxelScene* scene_ = nullptr;
  std::vector<RigidBody> bodies_;
  std::vector<ShapeClass> classes_;
  float accumulator_ = 0.0f;
  DebugSolve debug_{};
};

}  // namespace physics
