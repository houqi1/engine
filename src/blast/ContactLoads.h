#pragma once

#include "scene/VoxelTypes.h"

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <unordered_set>
#include <vector>

namespace blast {

struct NodeRef {
  uint32_t graphNode = 0;
  glm::vec3 center{0.0f};
};

struct MappedLoad {
  uint32_t graphNode = 0;
  glm::vec3 F{0.0f};
  glm::vec3 tau{0.0f};
};

struct ContactImpulse {
  glm::vec3 point{0.0f};
  glm::vec3 impulse{0.0f};  // net J this substep, including friction
  uint64_t eventId = 0;
  bool persistent = false;
};

struct LoadSnapshot {
  uint64_t snapshotId = 0;
  float dt = 1.0f / 60.0f;
  std::vector<MappedLoad> loads;
  bool valid = true;
};

MappedLoad mapForceWithTorque(const glm::vec3& point, const glm::vec3& F, const NodeRef& node);
MappedLoad mapForceWithTorqueNearest(const glm::vec3& point, const glm::vec3& F, const std::vector<NodeRef>& nodes);

void reconstructWRTOrigin(const std::vector<MappedLoad>& loads, const std::vector<NodeRef>& nodes, glm::vec3& F,
                           glm::vec3& M);

LoadSnapshot snapshotFromContacts(const std::vector<ContactImpulse>& contacts, const std::vector<NodeRef>& nodes,
                                  float dt, uint64_t snapshotId);

class ImpulseEvents {
public:
  // First time: true and record. Replay: false.
  bool consume(uint64_t eventId);
  void invalidateSnapshot(LoadSnapshot& snap);
  bool consumed(uint64_t eventId) const;

private:
  std::unordered_set<uint64_t> consumed_;
};

// One substep contact contribution after SI, before pose integration.
// JA is the world impulse on idA; idB receives -JA. Static bodies still receive
// the reaction even when invM == 0.
struct WorldContactImpulse {
  VoxelObjectId idA{};
  VoxelObjectId idB{};
  glm::vec3 worldPoint{0.0f};
  glm::vec3 JA{0.0f};
  glm::vec3 xA{0.0f};
  glm::vec3 xB{0.0f};
  glm::quat qA{1.0f, 0.0f, 0.0f, 0.0f};
  glm::quat qB{1.0f, 0.0f, 0.0f, 0.0f};
  uint32_t fineA = 0xFFFFFFFFu;
  uint32_t fineB = 0xFFFFFFFFu;
  int fineNA = 0;
  int fineNB = 0;
  uint64_t tickId = 0;
  int substep = 0;
};

struct BodyAssetFrame {
  VoxelObjectId objectId{};
  glm::vec3 worldCom{0.0f};
  glm::quat worldQ{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 assetCom{0.0f};
};

struct NodeLoadAccum {
  uint32_t graphNode = 0;
  glm::vec3 J{0.0f};
  glm::vec3 L{0.0f};
};

struct TickLoadResult {
  std::vector<MappedLoad> loads;
  glm::vec3 Jsum{0.0f};
  glm::vec3 LsumOrigin{0.0f};
  uint32_t contactContrib = 0;
  uint32_t mappedNodes = 0;
  uint32_t unmapped = 0;
};

glm::vec3 worldToAsset(const glm::vec3& pWorld, const BodyAssetFrame& frame);
glm::vec3 worldVecToAsset(const glm::vec3& vWorld, const BodyAssetFrame& frame);

struct PickedImpulse {
  glm::vec3 pAsset{0.0f};
  glm::vec3 Jasset{0.0f};
  NodeRef node{};
  bool ok = false;
};

// Convert a world impulse on `objectId` into asset-local J at the contact point.
// sideA true: impulse is JA on idA. sideA false: impulse on B is -JA.
bool projectImpulseToActor(const WorldContactImpulse& imp, VoxelObjectId objectId, const BodyAssetFrame& frame,
                           PickedImpulse& out);

// Fold many (possibly different-point) impulses onto nodes. F = J/kDt, tau = L/kDt.
TickLoadResult foldPickedImpulses(const PickedImpulse* picked, uint32_t n, float tickDt,
                                  const glm::vec3& originAsset);

bool pickNearestNode(const glm::vec3& pAsset, const std::vector<NodeRef>& nodes, NodeRef& out);

}  // namespace blast
