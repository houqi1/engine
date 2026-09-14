#include "blast/ContactLoads.h"

#include <limits>

namespace blast {

MappedLoad mapForceWithTorque(const glm::vec3& point, const glm::vec3& F, const NodeRef& node) {
  MappedLoad L;
  L.graphNode = node.graphNode;
  L.F = F;
  L.tau = glm::cross(point - node.center, F);
  return L;
}

MappedLoad mapForceWithTorqueNearest(const glm::vec3& point, const glm::vec3& F, const std::vector<NodeRef>& nodes) {
  MappedLoad L;
  if (nodes.empty()) {
    return L;
  }
  float best = std::numeric_limits<float>::max();
  const NodeRef* pick = &nodes[0];
  for (const NodeRef& n : nodes) {
    const float d2 = glm::dot(point - n.center, point - n.center);
    if (d2 < best) {
      best = d2;
      pick = &n;
    }
  }
  return mapForceWithTorque(point, F, *pick);
}

void reconstructWRTOrigin(const std::vector<MappedLoad>& loads, const std::vector<NodeRef>& nodes, glm::vec3& F,
                          glm::vec3& M) {
  F = glm::vec3(0.0f);
  M = glm::vec3(0.0f);
  for (const MappedLoad& L : loads) {
    glm::vec3 c(0.0f);
    for (const NodeRef& n : nodes) {
      if (n.graphNode == L.graphNode) {
        c = n.center;
        break;
      }
    }
    F += L.F;
    M += glm::cross(c, L.F) + L.tau;
  }
}

LoadSnapshot snapshotFromContacts(const std::vector<ContactImpulse>& contacts, const std::vector<NodeRef>& nodes,
                                  float dt, uint64_t snapshotId) {
  LoadSnapshot snap;
  snap.snapshotId = snapshotId;
  snap.dt = dt > 0.0f ? dt : 1.0f / 60.0f;
  snap.valid = true;
  for (const ContactImpulse& c : contacts) {
    const glm::vec3 F = c.impulse / snap.dt;
    snap.loads.push_back(mapForceWithTorqueNearest(c.point, F, nodes));
  }
  return snap;
}

bool ImpulseEvents::consume(uint64_t eventId) {
  if (eventId == 0) {
    return true;
  }
  if (consumed_.count(eventId) != 0) {
    return false;
  }
  consumed_.insert(eventId);
  return true;
}

void ImpulseEvents::invalidateSnapshot(LoadSnapshot& snap) { snap.valid = false; }

bool ImpulseEvents::consumed(uint64_t eventId) const { return consumed_.count(eventId) != 0; }

glm::vec3 worldToAsset(const glm::vec3& pWorld, const BodyAssetFrame& frame) {
  const glm::vec3 r = glm::inverse(frame.worldQ) * (pWorld - frame.worldCom);
  return frame.assetCom + r;
}

glm::vec3 worldVecToAsset(const glm::vec3& vWorld, const BodyAssetFrame& frame) {
  return glm::inverse(frame.worldQ) * vWorld;
}

bool projectImpulseToActor(const WorldContactImpulse& imp, VoxelObjectId objectId, const BodyAssetFrame& frame,
                           PickedImpulse& out) {
  out = {};
  if (!objectId.valid() || !frame.objectId.valid() || objectId != frame.objectId) {
    return false;
  }
  bool sideA = false;
  glm::vec3 Jw(0.0f);
  if (imp.idA == objectId) {
    sideA = true;
    Jw = imp.JA;
  } else if (imp.idB == objectId) {
    Jw = -imp.JA;
  } else {
    return false;
  }
  (void)sideA;
  out.pAsset = worldToAsset(imp.worldPoint, frame);
  out.Jasset = worldVecToAsset(Jw, frame);
  out.ok = true;
  return true;
}

bool pickNearestNode(const glm::vec3& pAsset, const std::vector<NodeRef>& nodes, NodeRef& out) {
  if (nodes.empty()) {
    return false;
  }
  float best = std::numeric_limits<float>::max();
  const NodeRef* pick = nullptr;
  for (const NodeRef& n : nodes) {
    const glm::vec3 d = pAsset - n.center;
    const float d2 = glm::dot(d, d);
    if (d2 < best) {
      best = d2;
      pick = &n;
    }
  }
  if (pick == nullptr) {
    return false;
  }
  out = *pick;
  return true;
}

TickLoadResult foldPickedImpulses(const PickedImpulse* picked, uint32_t n, float tickDt,
                                  const glm::vec3& originAsset) {
  TickLoadResult r;
  const float dt = tickDt > 0.0f ? tickDt : (1.0f / 60.0f);
  std::vector<NodeLoadAccum> acc;
  acc.reserve(n);
  auto findAcc = [&](uint32_t graphNode) -> NodeLoadAccum& {
    for (NodeLoadAccum& a : acc) {
      if (a.graphNode == graphNode) {
        return a;
      }
    }
    NodeLoadAccum a;
    a.graphNode = graphNode;
    acc.push_back(a);
    return acc.back();
  };
  for (uint32_t i = 0; i < n; ++i) {
    const PickedImpulse& p = picked[i];
    if (!p.ok) {
      ++r.unmapped;
      continue;
    }
    ++r.contactContrib;
    r.Jsum += p.Jasset;
    r.LsumOrigin += glm::cross(p.pAsset - originAsset, p.Jasset);
    NodeLoadAccum& a = findAcc(p.node.graphNode);
    a.J += p.Jasset;
    a.L += glm::cross(p.pAsset - p.node.center, p.Jasset);
  }
  r.mappedNodes = static_cast<uint32_t>(acc.size());
  r.loads.reserve(acc.size());
  for (const NodeLoadAccum& a : acc) {
    MappedLoad L;
    L.graphNode = a.graphNode;
    L.F = a.J / dt;
    L.tau = a.L / dt;
    r.loads.push_back(L);
  }
  return r;
}

}  // namespace blast
