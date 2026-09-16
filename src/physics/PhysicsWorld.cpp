#include "physics/PhysicsWorld.h"

#include "physics/Classify.h"
#include "physics/Solver.h"
#include "physics/VoxelCollide.h"
#include "scene/VoxelScene.h"

#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>
#include <vector>

namespace physics {
namespace {

glm::vec3 gridCenterLocal(const VoxelObject& o) {
  const float e = 0.5f * static_cast<float>(o.gridSize) * o.voxelSize;
  return glm::vec3(e, e, e);
}

void snapBodyComFromObject(const VoxelObject& o, RigidBody& body) {
  const glm::vec3 gc = gridCenterLocal(o);
  body.x = o.position + o.rotation * (body.comLocal - gc);
  body.q = o.rotation;
}

void writeObjectPoseFromCom(VoxelObject& o, const RigidBody& body) {
  const glm::vec3 gc = gridCenterLocal(o);
  o.rotation = body.q;
  o.position = body.x - body.q * (body.comLocal - gc);
}

bool isStaticEnvironment(const VoxelScene& scene, const RigidBody& b) {
  if (b.dynamic) {
    return false;
  }
  if (b.shapeIndex < 0 || !scene.slotOccupied(b.shapeIndex)) {
    return true;
  }
  return scene.cpuObject(b.shapeIndex).motionType != MotionType::Kinematic;
}

bool skipCollidePair(const VoxelScene& scene, const RigidBody& a, const RigidBody& b) {
  if (!a.dynamic && !b.dynamic) {
    return true;
  }
  const bool aKin = !a.dynamic && !isStaticEnvironment(scene, a);
  const bool bKin = !b.dynamic && !isStaticEnvironment(scene, b);
  if (!a.awake && !b.awake && !aKin && !bKin) {
    return true;
  }
  if (!a.awake && a.dynamic && isStaticEnvironment(scene, b)) {
    return true;
  }
  if (!b.awake && b.dynamic && isStaticEnvironment(scene, a)) {
    return true;
  }
  return false;
}

float quatAngle(const glm::quat& a, const glm::quat& b) {
  float d = std::abs(glm::dot(glm::normalize(a), glm::normalize(b)));
  d = std::min(d, 1.0f);
  return 2.0f * std::acos(d);
}

float sleepVelocity(const RigidBody& b, float h) {
  const float extent = std::max(b.extent, kSphereRadius);
  const float maxV = glm::length(b.v) + glm::length(b.w) * extent;
  const float maxDx = glm::length(b.x - b.xTick0) + quatAngle(b.qTick0, b.q) * extent;
  const float invH = h > 1e-8f ? 1.0f / h : 0.0f;
  return std::max(maxV, kSleepPositionFactor * maxDx * invH);
}

int ufFind(std::vector<int>& parent, int i) {
  while (parent[static_cast<size_t>(i)] != i) {
    parent[static_cast<size_t>(i)] = parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
    i = parent[static_cast<size_t>(i)];
  }
  return i;
}

void ufUnite(std::vector<int>& parent, int a, int b) {
  a = ufFind(parent, a);
  b = ufFind(parent, b);
  if (a != b) {
    parent[static_cast<size_t>(b)] = a;
  }
}

}  // namespace

void PhysicsWorld::attach(VoxelScene& scene) { scene_ = &scene; }

void PhysicsWorld::ensureBodyCapacity(int n) {
  if (n < 0) {
    return;
  }
  const size_t need = static_cast<size_t>(n);
  if (bodies_.size() < need) {
    bodies_.resize(need);
  }
  if (classes_.size() < need) {
    classes_.resize(need);
  }
}

void PhysicsWorld::initBodyFromObject(int objectIndex, RigidBody& body, bool preserveMotion) {
  const VoxelObject& o = scene_->cpuObject(objectIndex);
  const glm::vec3 prevV = body.v;
  const glm::vec3 prevW = body.w;
  const bool prevAwake = body.awake;
  const float prevSleep = body.sleepTimer;

  body.shapeIndex = objectIndex;
  body.x = o.position;
  body.q = o.rotation;
  const bool dynamic = o.slotOccupied && o.motionType == MotionType::Dynamic;
  body.dynamic = dynamic;
  if (!o.slotOccupied) {
    body.v = glm::vec3(0.0f);
    body.w = glm::vec3(0.0f);
    body.awake = false;
    body.sleepTimer = 0.0f;
    body.sleepIsland = -1;
    body.supports.clear();
    body.invM = 0.0f;
    body.Iloc = glm::mat3(1.0f);
    body.IinvW = glm::mat3(0.0f);
    return;
  }
  if (preserveMotion && dynamic) {
    body.v = prevV;
    body.w = prevW;
    body.awake = prevAwake;
    body.sleepTimer = prevSleep;
  } else {
    body.v = glm::vec3(0.0f);
    body.w = glm::vec3(0.0f);
    body.awake = dynamic;
    body.sleepTimer = 0.0f;
    body.sleepIsland = -1;
    body.supports.clear();
  }
}

void PhysicsWorld::rebuildFromScene() {
  if (!scene_) {
    return;
  }
  const int n = scene_->cpuObjectCount();
  bodies_.assign(static_cast<size_t>(n), RigidBody{});
  classes_.assign(static_cast<size_t>(n), ShapeClass{});
  for (int i = 0; i < n; ++i) {
    initBodyFromObject(i, bodies_[static_cast<size_t>(i)], false);
    classes_[static_cast<size_t>(i)].dirty = scene_->slotOccupied(i);
  }
  rebuildDirty();
  clock_.accumulator = 0.0f;
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
    const int slot = static_cast<int>(i);
    if (!scene_->slotOccupied(slot)) {
      wakeSleepersSupportedBy(slot);
      RigidBody& b = bodies_[i];
      b.shapeIndex = slot;
      b.dynamic = false;
      b.awake = false;
      b.sleepIsland = -1;
      b.supports.clear();
      b.invM = 0.0f;
      b.IinvW = glm::mat3(0.0f);
      b.v = glm::vec3(0.0f);
      b.w = glm::vec3(0.0f);
      classes_[i].dirty = false;
      classes_[i].corners.clear();
      classes_[i].edges.clear();
      classes_[i].occValid = false;
      continue;
    }
    RigidBody& b = bodies_[i];
    if (!b.dynamic) {
      wakeSleepersSupportedBy(slot);
    } else if (!b.awake) {
      wakeBodyAndIsland(slot, 0.0f);
    }
    rebuildShapeClass(*scene_, slot, classes_[i]);
    computeMassProperties(*scene_, slot, b);
    snapBodyComFromObject(scene_->cpuObject(slot), b);
    refreshInverseInertiaWorld(b);
  }
}

const ShapeClass* PhysicsWorld::shapeClass(int objectIndex) const {
  if (objectIndex < 0 || objectIndex >= static_cast<int>(classes_.size())) {
    return nullptr;
  }
  return &classes_[static_cast<size_t>(objectIndex)];
}

bool PhysicsWorld::getBodyState(VoxelObjectId id, BodyState& out) const {
  if (!scene_ || !scene_->tryGetObject(id)) {
    return false;
  }
  const int i = static_cast<int>(id.slot);
  if (i < 0 || i >= static_cast<int>(bodies_.size())) {
    return false;
  }
  const RigidBody& b = bodies_[static_cast<size_t>(i)];
  out.x = b.x;
  out.q = b.q;
  out.v = b.v;
  out.w = b.w;
  out.comLocal = b.comLocal;
  out.awake = b.awake;
  out.sleepTimer = b.sleepTimer;
  return true;
}

bool PhysicsWorld::addBody(VoxelObjectId id, const BodyState& state) {
  if (!scene_ || !scene_->tryGetObject(id)) {
    return false;
  }
  const int i = static_cast<int>(id.slot);
  ensureBodyCapacity(i + 1);
  RigidBody& b = bodies_[static_cast<size_t>(i)];
  initBodyFromObject(i, b, false);
  b.x = state.x;
  b.q = state.q;
  b.v = state.v;
  b.w = state.w;
  b.awake = state.awake && b.dynamic;
  b.sleepTimer = state.sleepTimer;
  classes_[static_cast<size_t>(i)] = ShapeClass{};
  classes_[static_cast<size_t>(i)].dirty = true;
  rebuildShapeClass(*scene_, i, classes_[static_cast<size_t>(i)]);
  computeMassProperties(*scene_, i, b);
  snapBodyComFromObject(scene_->cpuObject(i), b);
  b.v = state.v;
  b.w = state.w;
  b.awake = state.awake && b.dynamic;
  b.sleepTimer = state.sleepTimer;
  refreshInverseInertiaWorld(b);
  return true;
}

bool PhysicsWorld::removeBody(VoxelObjectId id) {
  if (!id.valid()) {
    return false;
  }
  const int i = static_cast<int>(id.slot);
  if (i < 0 || i >= static_cast<int>(bodies_.size())) {
    return false;
  }
  wakeSleepersSupportedBy(i);
  for (const std::pair<int, int>& t : lastTouching_) {
    if (t.first == i) {
      wakeBodyAndIsland(t.second, 0.0f);
    } else if (t.second == i) {
      wakeBodyAndIsland(t.first, 0.0f);
    }
  }
  RigidBody& b = bodies_[static_cast<size_t>(i)];
  b = RigidBody{};
  b.shapeIndex = i;
  b.dynamic = false;
  b.awake = false;
  if (i < static_cast<int>(classes_.size())) {
    classes_[static_cast<size_t>(i)] = ShapeClass{};
    classes_[static_cast<size_t>(i)].dirty = false;
  }
  return true;
}

bool PhysicsWorld::replaceShape(VoxelObjectId id, const BodyState* stateOrNull) {
  if (!scene_ || !scene_->tryGetObject(id)) {
    return false;
  }
  const int i = static_cast<int>(id.slot);
  ensureBodyCapacity(i + 1);
  RigidBody& b = bodies_[static_cast<size_t>(i)];
  const bool preserve = stateOrNull == nullptr;
  initBodyFromObject(i, b, preserve);
  if (stateOrNull) {
    b.x = stateOrNull->x;
    b.q = stateOrNull->q;
    b.v = stateOrNull->v;
    b.w = stateOrNull->w;
    b.awake = stateOrNull->awake && b.dynamic;
    b.sleepTimer = stateOrNull->sleepTimer;
  }
  classes_[static_cast<size_t>(i)].dirty = true;
  rebuildShapeClass(*scene_, i, classes_[static_cast<size_t>(i)]);
  computeMassProperties(*scene_, i, b);
  snapBodyComFromObject(scene_->cpuObject(i), b);
  if (stateOrNull) {
    b.v = stateOrNull->v;
    b.w = stateOrNull->w;
    b.awake = stateOrNull->awake && b.dynamic;
    b.sleepTimer = stateOrNull->sleepTimer;
  }
  refreshInverseInertiaWorld(b);
  return true;
}

void PhysicsWorld::activateBodiesInBounds(const glm::vec3& worldMin, const glm::vec3& worldMax) {
  if (!scene_) {
    return;
  }
  const int n = static_cast<int>(bodies_.size());
  for (int i = 0; i < n; ++i) {
    if (!scene_->slotOccupied(i)) {
      continue;
    }
    RigidBody& b = bodies_[static_cast<size_t>(i)];
    if (!b.dynamic || i >= static_cast<int>(classes_.size())) {
      continue;
    }
    glm::vec3 wmn;
    glm::vec3 wmx;
    if (!worldAabb(*scene_, i, classes_[static_cast<size_t>(i)], wmn, wmx)) {
      continue;
    }
    if (wmx.x < worldMin.x || wmn.x > worldMax.x || wmx.y < worldMin.y || wmn.y > worldMax.y ||
        wmx.z < worldMin.z || wmn.z > worldMax.z) {
      continue;
    }
    wakeBodyAndIsland(i, 0.0f);
  }
}

void PhysicsWorld::snapshotTickPoses() {
  for (RigidBody& b : bodies_) {
    if (!b.dynamic) {
      continue;
    }
    b.xTick0 = b.x;
    b.qTick0 = b.q;
  }
}

void PhysicsWorld::wakeBodyAndIsland(int objectIndex, float gravityDt) {
  if (objectIndex < 0 || objectIndex >= static_cast<int>(bodies_.size())) {
    return;
  }
  const int island = bodies_[static_cast<size_t>(objectIndex)].sleepIsland;
  auto wakeOne = [&](RigidBody& b) {
    if (!b.dynamic) {
      return;
    }
    const bool wasSleeping = !b.awake;
    b.awake = true;
    b.sleepTimer = 0.0f;
    b.sleepIsland = -1;
    b.supports.clear();
    if (wasSleeping && gravityDt > 0.0f && b.invM > 0.0f) {
      b.v += kGravity * gravityDt;
    }
  };
  if (island < 0) {
    wakeOne(bodies_[static_cast<size_t>(objectIndex)]);
    return;
  }
  for (RigidBody& b : bodies_) {
    if (b.sleepIsland == island) {
      wakeOne(b);
    }
  }
}

void PhysicsWorld::wakeSleepersSupportedBy(int staticSlot) {
  for (size_t i = 0; i < bodies_.size(); ++i) {
    RigidBody& b = bodies_[i];
    if (b.awake || !b.dynamic) {
      continue;
    }
    for (const SleepSupport& s : b.supports) {
      if (s.staticSlot == staticSlot) {
        wakeBodyAndIsland(static_cast<int>(i), 0.0f);
        break;
      }
    }
  }
}

void PhysicsWorld::wakeLostStaticSupport() {
  if (!scene_) {
    return;
  }
  const int n = static_cast<int>(bodies_.size());
  for (int i = 0; i < n; ++i) {
    RigidBody& b = bodies_[static_cast<size_t>(i)];
    if (b.awake || !b.dynamic || b.supports.empty()) {
      continue;
    }
    bool anyLost = false;
    for (const SleepSupport& s : b.supports) {
      if (!worldPointHitsOccupancy(*scene_, s.staticSlot, s.worldP)) {
        anyLost = true;
        break;
      }
    }
    if (anyLost) {
      wakeBodyAndIsland(i, 0.0f);
    }
  }
}

void PhysicsWorld::wakeFromTouchingContacts(const std::vector<Contact>& contacts, float gravityDt) {
  const int n = static_cast<int>(bodies_.size());
  for (const Contact& c : contacts) {
    if (!contactIsTouching(c) || c.a < 0 || c.b < 0 || c.a >= n || c.b >= n) {
      continue;
    }
    RigidBody& A = bodies_[static_cast<size_t>(c.a)];
    RigidBody& B = bodies_[static_cast<size_t>(c.b)];
    if (A.dynamic && !A.awake && B.dynamic && B.awake) {
      wakeBodyAndIsland(c.a, gravityDt);
    }
    if (B.dynamic && !B.awake && A.dynamic && A.awake) {
      wakeBodyAndIsland(c.b, gravityDt);
    }
    if (A.dynamic && !A.awake && !isStaticEnvironment(*scene_, B)) {
      wakeBodyAndIsland(c.a, gravityDt);
    }
    if (B.dynamic && !B.awake && !isStaticEnvironment(*scene_, A)) {
      wakeBodyAndIsland(c.b, gravityDt);
    }
  }
}

void PhysicsWorld::substep() {
  if (!scene_) {
    return;
  }
  const auto t0 = std::chrono::steady_clock::now();
  for (RigidBody& b : bodies_) {
    if (b.awake && b.invM > 0.0f && b.dynamic) {
      b.v += kGravity * kSubDt;
    }
  }
  std::vector<Contact> contacts;
  const int n = static_cast<int>(bodies_.size());
  for (int i = 0; i < n; ++i) {
    if (!scene_->slotOccupied(i) || !scene_->cpuObject(i).enabled) {
      continue;
    }
    for (int j = i + 1; j < n; ++j) {
      if (!scene_->slotOccupied(j) || !scene_->cpuObject(j).enabled) {
        continue;
      }
      RigidBody& A = bodies_[static_cast<size_t>(i)];
      RigidBody& B = bodies_[static_cast<size_t>(j)];
      if (skipCollidePair(*scene_, A, B)) {
        continue;
      }
      collidePair(*scene_, A, B, classes_[static_cast<size_t>(i)], classes_[static_cast<size_t>(j)],
                  contacts);
    }
  }
  wakeFromTouchingContacts(contacts, kSubDt);
  const auto t1 = std::chrono::steady_clock::now();
  solveContacts(bodies_, contacts, kSubDt, kContactIters);
  recordSubstepImpulses(contacts, substepIndex_);
  ++substepIndex_;
  const auto t2 = std::chrono::steady_clock::now();
  integrateBodies(bodies_, kSubDt);
  syncTransformsToScene();
  const auto t3 = std::chrono::steady_clock::now();
  debug_.collideMs += std::chrono::duration<float, std::milli>(t1 - t0).count();
  debug_.contactSolveMs += std::chrono::duration<float, std::milli>(t2 - t1).count();
  debug_.integrateMs += std::chrono::duration<float, std::milli>(t3 - t2).count();
  debug_.contacts = static_cast<int>(contacts.size());
  debug_.lastContacts = contacts;
  debug_.maxD = 0.0f;
  debug_.minNy = 1.0f;
  for (const Contact& c : contacts) {
    debug_.maxD = std::max(debug_.maxD, c.d);
    debug_.minNy = std::min(debug_.minNy, c.n.y);
  }
  if (scene_) {
    const VoxelObjectId testId = scene_->testObjectId();
    if (testId.valid() && static_cast<int>(testId.slot) < n) {
      const RigidBody& tb = bodies_[testId.slot];
      debug_.v = tb.v;
      debug_.w = tb.w;
    }
  }
}

void PhysicsWorld::updateSleep(float h) {
  const int n = static_cast<int>(bodies_.size());
  if (n <= 0) {
    lastTouching_.clear();
    return;
  }

  lastTouching_.clear();
  std::vector<int> parent(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i) {
    parent[static_cast<size_t>(i)] = i;
  }
  std::vector<char> penetrating(static_cast<size_t>(n), 0);

  for (const Contact& c : debug_.lastContacts) {
    if (c.a < 0 || c.b < 0 || c.a >= n || c.b >= n || !contactIsTouching(c)) {
      continue;
    }
    lastTouching_.push_back({c.a, c.b});
    RigidBody& A = bodies_[static_cast<size_t>(c.a)];
    RigidBody& B = bodies_[static_cast<size_t>(c.b)];
    if (A.dynamic && B.dynamic && A.awake && B.awake) {
      ufUnite(parent, c.a, c.b);
    }
    if (c.d > kSlop) {
      if (A.dynamic && A.awake) {
        penetrating[static_cast<size_t>(c.a)] = 1;
      }
      if (B.dynamic && B.awake) {
        penetrating[static_cast<size_t>(c.b)] = 1;
      }
    }
  }

  struct IslandAcc {
    float minTimer = 1.0e30f;
    bool canSleep = true;
  };
  std::vector<IslandAcc> acc(static_cast<size_t>(n));
  std::vector<char> used(static_cast<size_t>(n), 0);

  for (int i = 0; i < n; ++i) {
    RigidBody& b = bodies_[static_cast<size_t>(i)];
    if (!b.dynamic || b.invM <= 0.0f) {
      b.awake = false;
      continue;
    }
    if (!b.awake) {
      continue;
    }
    const bool motionSleepy =
        sleepVelocity(b, h) <= kSleepLin && glm::length(b.w) <= kSleepAng;
    if (!b.enableSleep || !motionSleepy) {
      b.sleepTimer = 0.0f;
      b.awake = true;
    } else {
      b.sleepTimer += h;
    }
    const int root = ufFind(parent, i);
    used[static_cast<size_t>(root)] = 1;
    IslandAcc& a = acc[static_cast<size_t>(root)];
    a.minTimer = std::min(a.minTimer, b.sleepTimer);
    if (!b.enableSleep || !motionSleepy || penetrating[static_cast<size_t>(i)] != 0) {
      a.canSleep = false;
    }
  }

  for (int root = 0; root < n; ++root) {
    if (used[static_cast<size_t>(root)] == 0) {
      continue;
    }
    const IslandAcc& a = acc[static_cast<size_t>(root)];
    if (!a.canSleep || a.minTimer < kSleepTime) {
      continue;
    }
    const int islandId = nextSleepIsland_++;
    for (int i = 0; i < n; ++i) {
      if (!bodies_[static_cast<size_t>(i)].dynamic || !bodies_[static_cast<size_t>(i)].awake) {
        continue;
      }
      if (ufFind(parent, i) != ufFind(parent, root)) {
        continue;
      }
      RigidBody& b = bodies_[static_cast<size_t>(i)];
      b.awake = false;
      b.v = glm::vec3(0.0f);
      b.w = glm::vec3(0.0f);
      b.sleepIsland = islandId;
      b.supports.clear();
    }
    for (const Contact& c : debug_.lastContacts) {
      if (c.a < 0 || c.b < 0 || c.a >= n || c.b >= n || !contactIsTouching(c)) {
        continue;
      }
      RigidBody& A = bodies_[static_cast<size_t>(c.a)];
      RigidBody& B = bodies_[static_cast<size_t>(c.b)];
      if (A.sleepIsland == islandId && isStaticEnvironment(*scene_, B)) {
        A.supports.push_back(SleepSupport{c.b, c.p});
      }
      if (B.sleepIsland == islandId && isStaticEnvironment(*scene_, A)) {
        B.supports.push_back(SleepSupport{c.a, c.p});
      }
    }
  }
}

void PhysicsWorld::recordSubstepImpulses(const std::vector<Contact>& contacts, int substep) {
  if (!scene_) {
    return;
  }
  const uint64_t tick = clock_.tickId;
  for (const Contact& c : contacts) {
    if (c.a < 0 || c.b < 0 || c.a >= static_cast<int>(bodies_.size()) ||
        c.b >= static_cast<int>(bodies_.size())) {
      continue;
    }
    const glm::vec3 JA = contactImpulseOnA(c);
    if (glm::dot(JA, JA) <= 1e-20f) {
      continue;
    }
    const RigidBody& A = bodies_[static_cast<size_t>(c.a)];
    const RigidBody& B = bodies_[static_cast<size_t>(c.b)];
    if (!A.dynamic && !B.dynamic) {
      continue;
    }
    blast::WorldContactImpulse rec;
    rec.idA = scene_->objectIdAt(c.a);
    rec.idB = scene_->objectIdAt(c.b);
    rec.worldPoint = c.p;
    rec.JA = JA;
    rec.xA = A.x;
    rec.xB = B.x;
    rec.qA = A.q;
    rec.qB = B.q;
    rec.fineA = c.fineA;
    rec.fineB = c.fineB;
    rec.fineNA = (c.a < static_cast<int>(classes_.size())) ? classes_[static_cast<size_t>(c.a)].fineN : 0;
    rec.fineNB = (c.b < static_cast<int>(classes_.size())) ? classes_[static_cast<size_t>(c.b)].fineN : 0;
    rec.tickId = tick;
    rec.substep = substep;
    tickImpulses_.push_back(rec);
  }
}

void PhysicsWorld::setFixedTickCallback(FixedPhysicsTickFn fn, void* user) {
  tickFn_ = fn;
  tickUser_ = user;
}

void PhysicsWorld::resetTickSession() { clock_.resetSession(); }

void PhysicsWorld::step(float frameDt) {
  const auto tRebuild0 = std::chrono::steady_clock::now();
  if (scene_) {
    const int n = scene_->cpuObjectCount();
    // Grow body tables when the scene adds slots. Do not wipe velocities of existing bodies.
    if (static_cast<int>(bodies_.size()) < n) {
      const int oldN = static_cast<int>(bodies_.size());
      ensureBodyCapacity(n);
      for (int i = oldN; i < n; ++i) {
        initBodyFromObject(i, bodies_[static_cast<size_t>(i)], false);
        classes_[static_cast<size_t>(i)].dirty = scene_->slotOccupied(i);
      }
    } else if (static_cast<int>(bodies_.size()) > n) {
      bodies_.resize(static_cast<size_t>(n));
      classes_.resize(static_cast<size_t>(n));
    }
    rebuildDirty();
  }
  debug_.rebuildMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - tRebuild0).count();
  debug_.physicsTicksThisFrame = 0;
  clock_.advance(frameDt, [this](uint64_t id, float dt) {
    debug_.collideMs = 0.0f;
    debug_.contactSolveMs = 0.0f;
    debug_.integrateMs = 0.0f;
    debug_.structureCallbackMs = 0.0f;
    debug_.awakeBodies = 0;
    debug_.occupiedBodies = 0;
    tickImpulses_.clear();
    substepIndex_ = 0;
    if (scene_) {
      wakeLostStaticSupport();
      snapshotTickPoses();
      for (int s = 0; s < kSubsteps; ++s) {
        substep();
      }
      updateSleep(dt);
      for (size_t i = 0; i < bodies_.size(); ++i) {
        if (scene_->slotOccupied(static_cast<int>(i))) {
          ++debug_.occupiedBodies;
          if (bodies_[i].awake) {
            ++debug_.awakeBodies;
          }
        }
      }
    }
    const auto tCb0 = std::chrono::steady_clock::now();
    if (tickFn_ != nullptr) {
      tickFn_(tickUser_, id, dt);
    }
    debug_.structureCallbackMs =
        std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - tCb0).count();
    ++debug_.physicsTicksThisFrame;
  });
}

void PhysicsWorld::syncTransformsToScene() {
  if (!scene_) {
    return;
  }
  for (RigidBody& b : bodies_) {
    if (b.shapeIndex < 0 || b.shapeIndex >= scene_->cpuObjectCount()) {
      continue;
    }
    if (!scene_->slotOccupied(b.shapeIndex)) {
      continue;
    }
    VoxelObject& o = scene_->cpuObject(b.shapeIndex);
    // Kinematic objects are authored by the scene (spinner); do not overwrite.
    if (o.motionType == MotionType::Kinematic) {
      snapBodyComFromObject(o, b);
      continue;
    }
    if (!b.dynamic) {
      continue;
    }
    writeObjectPoseFromCom(o, b);
  }
}

}  // namespace physics
