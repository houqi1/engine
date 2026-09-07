#pragma once

#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"
#include "physics/Structure.h"

#include <vector>

class VoxelScene;

namespace physics {

class PhysicsWorld {
public:
  void attach(VoxelScene& scene);
  void rebuildFromScene();
  void markDirty(int objectIndex);
  void markDirtyCells(int objectIndex, const std::vector<glm::ivec3>& coarses);
  // src stays; dst is a newly appended CPU object. Copies velocity and adds w × r.
  void onSplit(int srcIndex, int dstIndex, const glm::vec3& newCenterWorld);
  void rebuildDirty();
  void step(float frameDt);
  void syncTransformsToScene();

  const ShapeClass* shapeClass(int objectIndex) const;
  DebugSolve debugSolve() const { return debug_; }
  const std::vector<DebugBond>& debugBonds() const { return structure_.debugBonds(); }
  void fillCoarsePhi(std::vector<float>& dst) const { structure_.fillCoarsePhi(dst); }

private:
  void substep();
  void updateSleep(float h);

  VoxelScene* scene_ = nullptr;
  StructureWorld structure_;
  std::vector<RigidBody> bodies_;
  std::vector<ShapeClass> classes_;
  float accumulator_ = 0.0f;
  DebugSolve debug_{};
};

}  // namespace physics
