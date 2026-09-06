#include "physics/PhysicsWorld.h"

#include "physics/Classify.h"
#include "physics/Solver.h"
#include "physics/VoxelCollide.h"
#include "scene/VoxelScene.h"

#include <algorithm>

namespace physics {

void PhysicsWorld::attach(VoxelScene& scene) { scene_ = &scene; }

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
  accumulator_ = 0.0f;
}

void PhysicsWorld::markDirty(int objectIndex) {
  if (objectIndex >= 0 && objectIndex < static_cast<int>(classes_.size())) {
    classes_[static_cast<size_t>(objectIndex)].dirty = true;
  }
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
  solveContacts(bodies_, contacts, kSubDt, kContactIters);
  integrateBodies(bodies_, kSubDt);
  syncTransformsToScene();
  debug_.contacts = static_cast<int>(contacts.size());
  debug_.lastContacts = contacts;
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
  accumulator_ =
      std::min(accumulator_ + std::max(frameDt, 0.0f), kDt * static_cast<float>(kMaxStepsPerFrame));
  int guard = 0;
  while (accumulator_ >= kDt && guard < kMaxStepsPerFrame) {
    for (int s = 0; s < kSubsteps; ++s) {
      substep();
    }
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
    o.position = b.x;
    o.rotation = b.q;
  }
}

}  // namespace physics
