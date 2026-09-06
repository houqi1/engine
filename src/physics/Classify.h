#pragma once

#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"

class VoxelScene;

namespace physics {

void rebuildShapeClass(VoxelScene& scene, int objectIndex, ShapeClass& out);
void computeMassProperties(VoxelScene& scene, int objectIndex, RigidBody& body);

}  // namespace physics
