#include "physics/PhysicsWorld.h"

#include "physics/Classify.h"
#include "physics/Solver.h"
#include "physics/VoxelCollide.h"
#include "scene/VoxelScene.h"

#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

namespace physics {

namespace {

glm::vec3 gridCenterLocal(const VoxelObject& o) {
  return 0.5f * static_cast<float>(o.gridSize) * o.voxelSize * glm::vec3(1.0f);
}

}  // namespace

void PhysicsWorld::attach(VoxelScene& scene) {
  scene_ = &scene;
  structure_.attach(scene);
}

void PhysicsWorld::rebuildFromScene() {
  if (!scene_) {
    return;
  }
  const int n = scene_->cpuObjectCount();
  bodies_.assign(static_cast<size_t>(n), RigidBody{});
  classes_.assign(static_cast<size_t>(n), ShapeClass{});
  for (int i = 0; i < n; ++i) {
    RigidBody& b = bodies_[static_cast<size_t>(i)];
    const VoxelObject& o = scene_->cpuObject(i);
    b.shapeIndex = i;
    b.x = o.position;
    b.q = o.rotation;
    b.dynamic = (i != 0);
    b.awake = b.dynamic;
    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);
    b.sleepTimer = 0.0f;
    classes_[static_cast<size_t>(i)].dirty = true;
  }
  rebuildDirty();
  structure_.rebuildAll();
  syncTransformsToScene();
  accumulator_ = 0.0f;
}

void PhysicsWorld::markDirty(int objectIndex) {
  if (objectIndex >= 0 && objectIndex < static_cast<int>(classes_.size())) {
    classes_[static_cast<size_t>(objectIndex)].dirty = true;
  }
  structure_.markDirty(objectIndex);
}

void PhysicsWorld::onSplit(int srcIndex, int dstIndex, const glm::vec3& /*newCenterWorld*/) {
  if (!scene_ || srcIndex < 0 || dstIndex < 0) {
    return;
  }
  const int n = scene_->cpuObjectCount();
  if (dstIndex >= n || srcIndex >= n) {
    return;
  }
  while (static_cast<int>(bodies_.size()) < n) {
    bodies_.push_back(RigidBody{});
    classes_.push_back(ShapeClass{});
    classes_.back().dirty = true;
  }
  RigidBody srcCopy{};
  if (srcIndex < static_cast<int>(bodies_.size())) {
    srcCopy = bodies_[static_cast<size_t>(srcIndex)];
  }
  RigidBody& dst = bodies_[static_cast<size_t>(dstIndex)];
  dst.shapeIndex = dstIndex;
  dst.x = scene_->cpuObject(dstIndex).position;
  dst.q = scene_->cpuObject(dstIndex).rotation;
  dst.comLocal = gridCenterLocal(scene_->cpuObject(dstIndex));
  dst.dynamic = (dstIndex != 0) || srcCopy.dynamic;
  dst.awake = dst.dynamic;
  dst.sleepTimer = 0.0f;
  dst.invM = 0.0f;
  dst.v = glm::vec3(0.0f);
  dst.w = srcCopy.w;
  classes_[static_cast<size_t>(srcIndex)].dirty = true;
  classes_[static_cast<size_t>(dstIndex)].dirty = true;
  rebuildDirty();
  // v_point = v_com + w × (p - com). src.x is the pre-split COM.
  dst.v = srcCopy.v + glm::cross(srcCopy.w, dst.x - srcCopy.x);
  dst.w = srcCopy.w;
  dst.awake = dst.dynamic;
  structure_.onSplit(srcIndex, dstIndex);
  syncTransformsToScene();
}

void PhysicsWorld::rebuildDirty() {
  if (!scene_) {
    return;
  }
  for (size_t i = 0; i < bodies_.size(); ++i) {
    if (i >= classes_.size() || !classes_[i].dirty) {
      continue;
    }
    rebuildShapeClass(*scene_, static_cast<int>(i), classes_[i]);
    computeMassProperties(*scene_, static_cast<int>(i), bodies_[i]);
    refreshInverseInertiaWorld(bodies_[i]);
  }
}

const ShapeClass* PhysicsWorld::shapeClass(int objectIndex) const {
  if (objectIndex < 0 || objectIndex >= static_cast<int>(classes_.size())) {
    return nullptr;
  }
  return &classes_[static_cast<size_t>(objectIndex)];
}

void PhysicsWorld::substep() {
  if (!scene_) {
    return;
  }
  for (RigidBody& b : bodies_) {
    if (b.awake && b.invM > 0.0f && b.dynamic) {
      b.v += kGravity * kSubDt;
    }
  }
  structure_.substep();
  std::vector<Contact> contacts;
  const int n = static_cast<int>(bodies_.size());
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      RigidBody& A = bodies_[static_cast<size_t>(i)];
      RigidBody& B = bodies_[static_cast<size_t>(j)];
      if (!A.awake && !B.awake) {
        continue;
      }
      if (!scene_->cpuObject(i).enabled || !scene_->cpuObject(j).enabled) {
        continue;
      }
      collidePair(*scene_, A, B, classes_[static_cast<size_t>(i)], classes_[static_cast<size_t>(j)],
                  contacts);
    }
  }
  for (const Contact& c : contacts) {
    if (c.a >= 0 && c.a < n && bodies_[static_cast<size_t>(c.a)].dynamic) {
      bodies_[static_cast<size_t>(c.a)].awake = true;
      bodies_[static_cast<size_t>(c.a)].sleepTimer = 0.0f;
    }
    if (c.b >= 0 && c.b < n && bodies_[static_cast<size_t>(c.b)].dynamic) {
      bodies_[static_cast<size_t>(c.b)].awake = true;
      bodies_[static_cast<size_t>(c.b)].sleepTimer = 0.0f;
    }
  }
  structure_.wakeFromContacts(contacts);
  solveContacts(bodies_, contacts, kSubDt, kContactIters);
  integrateBodies(bodies_, kSubDt);
  syncTransformsToScene();
  debug_.contacts = static_cast<int>(contacts.size());
  debug_.lastContacts = contacts;
  structure_.fillDebug(debug_);
  debug_.maxD = 0.0f;
  debug_.minNy = 1.0f;
  for (const Contact& c : contacts) {
    debug_.maxD = std::max(debug_.maxD, c.d);
    debug_.minNy = std::min(debug_.minNy, c.n.y);
  }
  if (bodies_.size() > 1) {
    debug_.v = bodies_[1].v;
    debug_.w = bodies_[1].w;
  }
}

void PhysicsWorld::updateSleep(float h) {
  for (RigidBody& b : bodies_) {
    if (!b.dynamic || b.invM <= 0.0f) {
      b.awake = false;
      continue;
    }
    if (glm::length(b.v) < kSleepLin && glm::length(b.w) < kSleepAng) {
      b.sleepTimer += h;
      if (b.sleepTimer >= kSleepTime) {
        b.awake = false;
        b.v = glm::vec3(0.0f);
        b.w = glm::vec3(0.0f);
      }
    } else {
      b.sleepTimer = 0.0f;
      b.awake = true;
    }
  }
}

void PhysicsWorld::step(float frameDt) {
  if (!scene_) {
    return;
  }
  if (static_cast<int>(bodies_.size()) != scene_->cpuObjectCount()) {
    rebuildFromScene();
  }
  rebuildDirty();
  syncTransformsToScene();
  accumulator_ =
      std::min(accumulator_ + std::max(frameDt, 0.0f), kDt * static_cast<float>(kMaxStepsPerFrame));
  int guard = 0;
  while (accumulator_ >= kDt && guard < kMaxStepsPerFrame) {
    structure_.beginDt();
    for (int s = 0; s < kSubsteps; ++s) {
      substep();
    }
    structure_.updateSleep(kDt);
    updateSleep(kDt);
    accumulator_ -= kDt;
    ++guard;
  }
}

void PhysicsWorld::syncTransformsToScene() {
  if (!scene_) {
    return;
  }
  for (RigidBody& b : bodies_) {
    if (b.shapeIndex < 0 || b.shapeIndex >= scene_->cpuObjectCount()) {
      continue;
    }
    VoxelObject& o = scene_->cpuObject(b.shapeIndex);
    o.rotation = b.q;
    o.position = b.x + b.q * (gridCenterLocal(o) - b.comLocal);
  }
}

}  // namespace physics
