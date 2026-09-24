#pragma once

#include "blast/ContactLoads.h"
#include "physics/PhysicsTypes.h"
#include "physics/DynamicAabbTree.h"
#include "physics/ContactCache.h"
#include "physics/VoxelCollide.h"
#include "physics/RigidBody.h"
#include "scene/VoxelTypes.h"

#include <glm/glm.hpp>

#include <utility>
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
  glm::vec3 comLocal{0.0f};
};

class PhysicsWorld {
public:
  using FixedPhysicsTickFn = void (*)(void* user, uint64_t tickId, float dt);

  void attach(VoxelScene& scene);
  void rebuildFromScene();
  void markDirty(int objectIndex);
  void step(float frameDt);
  void syncTransformsToScene();
  // Non-owning. Physics does not include Blast; the scene forwards to StructureWorld.
  void setFixedTickCallback(FixedPhysicsTickFn fn, void* user);
  void resetTickSession();
  uint64_t tickId() const { return clock_.tickId; }

  bool getBodyState(VoxelObjectId id, BodyState& out) const;
  bool addBody(VoxelObjectId id, const BodyState& state);
  bool removeBody(VoxelObjectId id);
  // Reclassify/mass for an existing body. If stateOrNull is non-null, replaces pose/velocity.
  bool replaceShape(VoxelObjectId id, const BodyState* stateOrNull);
  void activateBodiesInBounds(const glm::vec3& worldMin, const glm::vec3& worldMax);

  const ShapeClass* shapeClass(int objectIndex) const;
  // By reference: lastContacts can hold thousands of contacts in a pile-up.
  const DebugSolve& debugSolve() const { return debug_; }
  const std::vector<blast::WorldContactImpulse>& tickImpulses() const { return tickImpulses_; }

private:
  void ensureBodyCapacity(int n);
  void initBodyFromObject(int objectIndex, RigidBody& body, bool preserveMotion);
  void rebuildDirty();
  void substep();
  void snapshotTickPoses();
  void wakeLostStaticSupport();
  void wakeBodyAndIsland(int objectIndex, float gravityDt);
  void wakeSleepersSupportedBy(int staticSlot);
  void wakeFromTouchingContacts(const std::vector<Contact>& contacts, float gravityDt);
  void updateSleep(float h);
  void recordSubstepImpulses(const std::vector<Contact>& contacts, int substep);

  VoxelScene* scene_ = nullptr;
  std::vector<RigidBody> bodies_;
  std::vector<ShapeClass> classes_;
  DynamicAabbTree broadPhase_;
  ContactCache contactCache_;
  std::vector<int> broadProxies_;
  std::vector<PhysicsAabb> broadBounds_;
  std::vector<CollisionPose> collisionPoses_;
  std::vector<std::pair<int, int>> broadPairs_;
  FixedStepClock clock_{};
  FixedPhysicsTickFn tickFn_ = nullptr;
  void* tickUser_ = nullptr;
  DebugSolve debug_{};
  std::vector<blast::WorldContactImpulse> tickImpulses_;
  std::vector<std::pair<int, int>> lastTouching_;
  // Sorted, unique pairKey(lo, hi) of lastTouching_ for O(log n) lookups.
  std::vector<uint64_t> lastTouchingKeys_;
  std::vector<Contact> contactScratch_;
  // Narrow phase output per broad pair, merged in pair order (deterministic).
  std::vector<std::vector<Contact>> pairContacts_;
  std::vector<char> queriers_;
  std::vector<std::vector<std::pair<int, int>>> bodyPairs_;
  std::vector<ContactCache::Pair*> pairSlots_;
  std::vector<ContactCache::Stats> pairStats_;
  int substepIndex_ = 0;
  int nextSleepIsland_ = 1;
};

}  // namespace physics
