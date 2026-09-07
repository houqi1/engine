#include "physics/Structure.h"

#include "scene/VoxelScene.h"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace physics {
namespace {

constexpr float kShearAlpha = 1.2f;
constexpr float kBondBeta = 0.12f;
constexpr float kBondCfm = 0.05f;


glm::vec3 axisUnit(int axis) {
  glm::vec3 n(0.0f);
  n[axis] = 1.0f;
  return n;
}

glm::vec3 tangent1(int axis) {
  return axis == 0 ? glm::vec3(0, 1, 0) : glm::vec3(1, 0, 0);
}

glm::vec3 tangent2(int axis) {
  return glm::cross(axisUnit(axis), tangent1(axis));
}

float angTerm(const BondNode& n, const glm::vec3& r, const glm::vec3& dir) {
  if (n.anchored || n.mass <= 1e-8f) {
    return 0.0f;
  }
  const glm::vec3 t = glm::cross(r, dir);
  const glm::vec3 iinv(1.0f / std::max(n.Idiag.x, 1e-6f), 1.0f / std::max(n.Idiag.y, 1e-6f),
                       1.0f / std::max(n.Idiag.z, 1e-6f));
  return glm::dot(t, iinv * t);
}

}  // namespace

void StructureWorld::attach(VoxelScene& scene) { scene_ = &scene; }

void StructureWorld::rebuildAll() {
  nodes_.clear();
  bonds_.clear();
  nodeIndex_.clear();
  graph_.clear();
  islands_.clear();
  debugBonds_.clear();
  if (!scene_) {
    return;
  }
  for (int i = 0; i < scene_->cpuObjectCount(); ++i) {
    rebuildShape(i);
  }
  rebuildIslands();
  snapshotDebug();
}

void StructureWorld::markDirty(int shapeIndex) {
  dirtyFull_.insert(shapeIndex);
  const auto snap = snapshotIslandSleep();
  rebuildShape(shapeIndex);
  rebuildIslands(&snap);
  snapshotDebug();
}

void StructureWorld::markDirtyCells(int shapeIndex, const std::vector<glm::ivec3>& coarses) {
  if (coarses.empty() || dirtyFull_.count(shapeIndex) != 0) {
    if (coarses.empty()) {
      markDirty(shapeIndex);
    }
    return;
  }
  rebuildNeighborhood(shapeIndex, coarses);
  snapshotDebug();
}

void StructureWorld::onSplit(int srcIndex, int dstIndex) {
  const auto snap = snapshotIslandSleep();
  rebuildShape(srcIndex);
  rebuildShape(dstIndex);
  rebuildIslands(&snap);
  snapshotDebug();
  (void)dstIndex;
}

void StructureWorld::beginDt() {
  splitConsumedThisDt_ = false;
  brokenThisDt_ = 0;
  for (Island& is : islands_.all()) {
    is.brokenThisDt = 0;
  }
  splitOneIslandIfNeeded();
}

void StructureWorld::wakeShape(int shapeIndex) {
  for (Island& is : islands_.all()) {
    if (is.shapeIndex == shapeIndex) {
      islands_.wake(is.id);
    }
  }
}

void StructureWorld::wakeFromContacts(const std::vector<Contact>& contacts,
                                      const std::vector<RigidBody>& bodies) {
  if (!scene_ || contacts.empty()) {
    return;
  }
  auto speed = [&](int shape) -> float {
    if (shape < 0 || shape >= static_cast<int>(bodies.size())) {
      return 0.0f;
    }
    const RigidBody& b = bodies[static_cast<size_t>(shape)];
    return glm::length(b.v) + glm::length(b.w) * 0.5f;
  };
  auto hitsStructure = [&](int shape, uint32_t packedFine) -> bool {
    if (shape != 0 || shape >= scene_->cpuObjectCount()) {
      return false;
    }
    const VoxelObject& o = scene_->cpuObject(shape);
    const int n = o.gridSize * VoxelScene::kFinePerCoarse;
    if (n <= 0) {
      return false;
    }
    const glm::ivec3 p = unpackFine(packedFine, n);
    return (p.y / VoxelScene::kFinePerCoarse) > VoxelScene::kGroundCoarseY;
  };
  for (const Contact& c : contacts) {
    if (speed(c.a) < kImpactSpeed && speed(c.b) < kImpactSpeed) {
      continue;
    }
    if (hitsStructure(c.a, c.fineA)) {
      const VoxelObject& o = scene_->cpuObject(c.a);
      const int n = o.gridSize * VoxelScene::kFinePerCoarse;
      wakeIslandOfCoarse(c.a, unpackFine(c.fineA, n) / VoxelScene::kFinePerCoarse);
    }
    if (hitsStructure(c.b, c.fineB)) {
      const VoxelObject& o = scene_->cpuObject(c.b);
      const int n = o.gridSize * VoxelScene::kFinePerCoarse;
      wakeIslandOfCoarse(c.b, unpackFine(c.fineB, n) / VoxelScene::kFinePerCoarse);
    }
  }
}

uint64_t StructureWorld::nodeKey(int shapeIndex, const glm::ivec3& c) const {
  return (static_cast<uint64_t>(static_cast<uint32_t>(shapeIndex)) << 32) |
         (static_cast<uint32_t>(c.x) & 1023u) |
         ((static_cast<uint32_t>(c.y) & 1023u) << 10) |
         ((static_cast<uint32_t>(c.z) & 1023u) << 20);
}

uint64_t StructureWorld::bondKey(const glm::ivec3& lower, int axis) const {
  return (static_cast<uint64_t>(static_cast<uint32_t>(lower.x) & 1023u)) |
         (static_cast<uint64_t>(static_cast<uint32_t>(lower.y) & 1023u) << 10) |
         (static_cast<uint64_t>(static_cast<uint32_t>(lower.z) & 1023u) << 20) |
         (static_cast<uint64_t>(axis & 3) << 30);
}

int StructureWorld::findNode(int shapeIndex, const glm::ivec3& c) const {
  const auto it = nodeIndex_.find(nodeKey(shapeIndex, c));
  if (it == nodeIndex_.end()) {
    return -1;
  }
  return it->second;
}

BondStrength StructureWorld::strengthForMaterial(uint32_t mat) const {
  BondStrength s;
  if (mat >= 2u) {
    s.E = 3.0e8f;
    s.G = 1.2e8f;
    s.sigmaT = 2.0e5f;
    s.sigmaC = 8.0e6f;
    s.tauU = 1.2e6f;
    s.density = kDensityStone;
    s.jBreak = 120.0f;
  }
  return s;
}

bool StructureWorld::isStructural(int shapeIndex) const {
  return shapeIndex == 0;
}

std::unordered_map<uint64_t, bool> StructureWorld::snapshotIslandSleep() const {
  std::unordered_map<uint64_t, bool> snap;
  snap.reserve(nodes_.size());
  for (const Island& is : islands_.all()) {
    for (int ni : is.nodes) {
      if (ni < 0 || ni >= static_cast<int>(nodes_.size())) {
        continue;
      }
      const BondNode& n = nodes_[static_cast<size_t>(ni)];
      snap[nodeKey(n.shapeIndex, n.coarse)] = is.asleep;
    }
  }
  return snap;
}

bool StructureWorld::islandAwake(int islandId) const {
  const Island* is = islands_.byId(islandId);
  return is != nullptr && !is->asleep;
}

void StructureWorld::wakeIslandOfCoarse(int shapeIndex, const glm::ivec3& c) {
  const int ni = findNode(shapeIndex, c);
  if (ni < 0) {
    return;
  }
  islands_.wake(nodes_[static_cast<size_t>(ni)].islandId);
}

void StructureWorld::rebuildIslands(const std::unordered_map<uint64_t, bool>* prevSleep) {
  std::unordered_map<uint64_t, bool> owned;
  if (prevSleep == nullptr) {
    owned = snapshotIslandSleep();
    prevSleep = &owned;
  }
  const bool hadSnap = !prevSleep->empty();
  std::unordered_set<uint64_t> prevKeys;
  prevKeys.reserve(prevSleep->size());
  for (const auto& kv : *prevSleep) {
    prevKeys.insert(kv.first);
  }

  islands_.clear();
  for (BondNode& n : nodes_) {
    n.islandId = -1;
  }
  for (Bond& b : bonds_) {
    b.islandId = -1;
  }

  std::vector<int> unanchored;
  unanchored.reserve(nodes_.size());
  std::vector<int> local(nodes_.size(), -1);
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
    if (nodes_[static_cast<size_t>(i)].anchored) {
      continue;
    }
    local[static_cast<size_t>(i)] = static_cast<int>(unanchored.size());
    unanchored.push_back(i);
  }
  if (unanchored.empty()) {
    rebuildGraphColors();
    return;
  }

  std::vector<int> parent(unanchored.size());
  for (int i = 0; i < static_cast<int>(parent.size()); ++i) {
    parent[static_cast<size_t>(i)] = i;
  }
  auto find = [&](int x) {
    while (parent[static_cast<size_t>(x)] != x) {
      parent[static_cast<size_t>(x)] = parent[static_cast<size_t>(parent[static_cast<size_t>(x)])];
      x = parent[static_cast<size_t>(x)];
    }
    return x;
  };
  auto unite = [&](int a, int b) {
    a = find(a);
    b = find(b);
    if (a != b) {
      parent[static_cast<size_t>(b)] = a;
    }
  };
  for (const Bond& b : bonds_) {
    if (!b.alive || b.a < 0 || b.b < 0 || b.a >= static_cast<int>(local.size()) ||
        b.b >= static_cast<int>(local.size())) {
      continue;
    }
    const int la = local[static_cast<size_t>(b.a)];
    const int lb = local[static_cast<size_t>(b.b)];
    if (la >= 0 && lb >= 0) {
      unite(la, lb);
    }
  }

  std::unordered_map<int, int> rootToIsland;
  for (int li = 0; li < static_cast<int>(unanchored.size()); ++li) {
    const int r = find(li);
    auto it = rootToIsland.find(r);
    if (it == rootToIsland.end()) {
      Island& created = islands_.add(nodes_[static_cast<size_t>(unanchored[static_cast<size_t>(li)])].shapeIndex);
      it = rootToIsland.emplace(r, created.id).first;
    }
    Island* is = islands_.byId(it->second);
    const int ni = unanchored[static_cast<size_t>(li)];
    nodes_[static_cast<size_t>(ni)].islandId = is->id;
    is->nodes.push_back(ni);
  }

  const glm::ivec3 face[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (Island& is : islands_.all()) {
    bool allAsleep = hadSnap;
    bool lostNeighbor = false;
    for (int ni : is.nodes) {
      const BondNode& n = nodes_[static_cast<size_t>(ni)];
      const uint64_t key = nodeKey(n.shapeIndex, n.coarse);
      const auto it = prevSleep->find(key);
      if (it == prevSleep->end() || !it->second) {
        allAsleep = false;
      }
      for (const glm::ivec3& d : face) {
        const uint64_t nk = nodeKey(n.shapeIndex, n.coarse + d);
        if (prevKeys.count(nk) != 0 && findNode(n.shapeIndex, n.coarse + d) < 0) {
          lostNeighbor = true;
        }
      }
    }
    if (!hadSnap) {
      is.asleep = true;
      is.settleLeft = 0;
      is.sleepTimer = kSleepTime;
    } else if (allAsleep && !lostNeighbor) {
      is.asleep = true;
      is.settleLeft = 0;
      is.sleepTimer = kSleepTime;
    } else {
      is.asleep = false;
      is.settleLeft = kBondSettleSteps;
      is.sleepTimer = 0.0f;
    }
    is.needsSplit = false;
    is.brokenThisDt = 0;
    is.vmax = 0.0f;
  }

  for (int bi = 0; bi < static_cast<int>(bonds_.size()); ++bi) {
    Bond& b = bonds_[static_cast<size_t>(bi)];
    if (!b.alive || b.a < 0 || b.b < 0) {
      continue;
    }
    const int ia = nodes_[static_cast<size_t>(b.a)].islandId;
    const int ib = nodes_[static_cast<size_t>(b.b)].islandId;
    const int id = ia >= 0 ? ia : ib;
    b.islandId = id;
    if (id >= 0) {
      if (Island* is = islands_.byId(id)) {
        is->bonds.push_back(bi);
      }
    }
  }
  rebuildGraphColors();
}

void StructureWorld::splitOneIslandIfNeeded() {
  if (splitConsumedThisDt_ || islands_.firstNeedsSplit() < 0) {
    return;
  }
  const auto snap = snapshotIslandSleep();
  rebuildIslands(&snap);
  splitConsumedThisDt_ = true;
}

void StructureWorld::dropShapeRegion(int shapeIndex, const std::unordered_set<uint64_t>* dropKeys) {
  std::vector<BondNode> keptNodes;
  std::unordered_map<uint64_t, int> keptIndex;
  keptNodes.reserve(nodes_.size());
  for (const BondNode& n : nodes_) {
    if (n.shapeIndex == shapeIndex) {
      const uint64_t key = nodeKey(n.shapeIndex, n.coarse);
      if (dropKeys == nullptr || dropKeys->count(key) != 0) {
        continue;
      }
    }
    keptIndex[nodeKey(n.shapeIndex, n.coarse)] = static_cast<int>(keptNodes.size());
    keptNodes.push_back(n);
  }
  std::vector<Bond> keptBonds;
  keptBonds.reserve(bonds_.size());
  auto remap = [&](int oldIdx) -> int {
    if (oldIdx < 0 || oldIdx >= static_cast<int>(nodes_.size())) {
      return -1;
    }
    const BondNode& n = nodes_[static_cast<size_t>(oldIdx)];
    const auto it = keptIndex.find(nodeKey(n.shapeIndex, n.coarse));
    return it == keptIndex.end() ? -1 : it->second;
  };
  for (Bond& b : bonds_) {
    const int na = remap(b.a);
    const int nb = remap(b.b);
    if (na < 0 || nb < 0) {
      continue;
    }
    b.a = na;
    b.b = nb;
    keptBonds.push_back(b);
  }
  nodes_.swap(keptNodes);
  bonds_.swap(keptBonds);
  nodeIndex_.swap(keptIndex);
}

bool StructureWorld::addNode(int shapeIndex, const glm::ivec3& c, const BondNode* prev) {
  if (!scene_ || findNode(shapeIndex, c) >= 0) {
    return false;
  }
  if (VoxelScene::isTerrainCoarse(shapeIndex, c)) {
    return false;
  }
  const uint32_t mat = scene_->occupancyMaterial(shapeIndex, c);
  if (mat == 0u) {
    return false;
  }
  const uint32_t nFine = scene_->occupiedFineCount(shapeIndex, c);
  if (nFine == 0u) {
    return false;
  }
  const VoxelObject& o = scene_->cpuObject(shapeIndex);
  const float a = o.voxelSize;
  const float fineV = kVoxel * kVoxel * kVoxel;
  const BondStrength st = strengthForMaterial(mat);
  BondNode node;
  node.shapeIndex = shapeIndex;
  node.coarse = c;
  node.mass = std::max(1e-3f, st.density * fineV * static_cast<float>(nFine));
  const float ii = node.mass * a * a / 6.0f;
  node.Idiag = glm::vec3(std::max(ii, 1e-4f));
  const glm::vec3 local = (glm::vec3(c) + glm::vec3(0.5f)) * a;
  node.rest = glm::vec3(o.objectToWorld() * glm::vec4(local, 1.0f));
  node.anchored = (shapeIndex == 0 && c.y <= VoxelScene::kGroundCoarseY);
  if (prev != nullptr) {
    node.u = prev->u;
    node.theta = prev->theta;
    node.v = prev->v;
    node.w = prev->w;
  } else {
    node.u = glm::vec3(0.0f);
    node.theta = glm::vec3(0.0f);
    node.v = glm::vec3(0.0f);
    node.w = glm::vec3(0.0f);
  }
  if (node.anchored) {
    node.u = glm::vec3(0.0f);
    node.theta = glm::vec3(0.0f);
    node.v = glm::vec3(0.0f);
    node.w = glm::vec3(0.0f);
  }
  const int idx = static_cast<int>(nodes_.size());
  nodeIndex_[nodeKey(shapeIndex, c)] = idx;
  nodes_.push_back(node);
  return true;
}

void StructureWorld::addBondIfMissing(int shapeIndex, const glm::ivec3& lower, int axis,
                                      const std::unordered_map<uint64_t, Bond>* warm,
                                      std::unordered_set<uint64_t>* createdKeys) {
  glm::ivec3 upper = lower;
  upper[axis] += 1;
  const int i = findNode(shapeIndex, lower);
  const int j = findNode(shapeIndex, upper);
  if (i < 0 || j < 0) {
    return;
  }
  const uint64_t key = bondKey(lower, axis);
  if (createdKeys != nullptr && !createdKeys->insert(key).second) {
    return;
  }
  if (createdKeys == nullptr) {
    for (const Bond& existing : bonds_) {
      if (existing.alive && existing.a == i && existing.b == j && existing.axis == axis) {
        return;
      }
    }
  }
  const BondNode& na = nodes_[static_cast<size_t>(i)];
  const BondNode& nb = nodes_[static_cast<size_t>(j)];
  if (na.anchored && nb.anchored) {
    return;
  }
  const VoxelObject& o = scene_->cpuObject(shapeIndex);
  const float a = o.voxelSize;
  const uint32_t matA = scene_->occupancyMaterial(shapeIndex, lower);
  const uint32_t matB = scene_->occupancyMaterial(shapeIndex, upper);
  const BondStrength st = strengthForMaterial(std::min(matA, matB));
  const uint32_t nFace = scene_->countSharedFaceFines(shapeIndex, lower, axis);
  if (nFace == 0u) {
    return;
  }
  const float aFine = kVoxel * kVoxel;
  const float iFine = aFine * aFine / 12.0f;
  Bond bond;
  bond.a = i;
  bond.b = j;
  bond.axis = axis;
  bond.L = a;
  bond.A = std::max(1e-6f, aFine * static_cast<float>(nFace));
  bond.I = std::max(1e-12f, iFine * static_cast<float>(nFace));
  bond.J = 2.0f * bond.I;
  bond.kn = st.E * bond.A / bond.L;
  bond.kv = st.G * bond.A / (kShearAlpha * bond.L);
  bond.kt = st.G * bond.J / bond.L;
  bond.km = st.E * bond.I / bond.L;
  bond.strength = st;
  bond.alive = true;
  if (warm != nullptr) {
    const auto it = warm->find(bondKey(lower, axis));
    if (it != warm->end() && it->second.alive) {
      for (int k = 0; k < 6; ++k) {
        bond.lambda[k] = it->second.lambda[k];
      }
      bond.phi = it->second.phi;
    }
  }
  bonds_.push_back(bond);
}

void StructureWorld::rebuildShape(int shapeIndex) {
  if (!scene_ || shapeIndex < 0 || shapeIndex >= scene_->cpuObjectCount()) {
    return;
  }
  std::unordered_map<uint64_t, Bond> warm;
  std::unordered_map<uint64_t, BondNode> prevNodes;
  warm.reserve(bonds_.size());
  for (const Bond& b : bonds_) {
    if (b.a < 0 || b.b < 0 || b.a >= static_cast<int>(nodes_.size()) ||
        b.b >= static_cast<int>(nodes_.size())) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    if (na.shapeIndex != shapeIndex) {
      continue;
    }
    warm[bondKey(na.coarse, b.axis)] = b;
  }
  for (const BondNode& n : nodes_) {
    if (n.shapeIndex == shapeIndex) {
      prevNodes[nodeKey(n.shapeIndex, n.coarse)] = n;
    }
  }

  dropShapeRegion(shapeIndex, nullptr);

  const VoxelObject& o = scene_->cpuObject(shapeIndex);
  if (!isStructural(shapeIndex) || !o.enabled || o.cells.empty() || o.gridSize <= 0) {
    return;
  }

  auto prevOf = [&](const glm::ivec3& c) -> const BondNode* {
    const auto it = prevNodes.find(nodeKey(shapeIndex, c));
    return it == prevNodes.end() ? nullptr : &it->second;
  };

  if (!o.occupiedCoarses.empty()) {
    for (uint32_t packed : o.occupiedCoarses) {
      const glm::ivec3 c(static_cast<int>(packed & 1023u),
                         static_cast<int>((packed >> 10) & 1023u),
                         static_cast<int>((packed >> 20) & 1023u));
      addNode(shapeIndex, c, prevOf(c));
    }
  } else {
    for (int z = 0; z < o.gridSize; ++z) {
      for (int y = 0; y < o.gridSize; ++y) {
        for (int x = 0; x < o.gridSize; ++x) {
          const glm::ivec3 c(x, y, z);
          addNode(shapeIndex, c, prevOf(c));
        }
      }
    }
  }

  std::unordered_set<uint64_t> created;
  created.reserve(nodes_.size() * 3u);
  for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
    if (nodes_[static_cast<size_t>(i)].shapeIndex != shapeIndex) {
      continue;
    }
    for (int axis = 0; axis < 3; ++axis) {
      addBondIfMissing(shapeIndex, nodes_[static_cast<size_t>(i)].coarse, axis, &warm, &created);
    }
  }
}

void StructureWorld::rebuildNeighborhood(int shapeIndex, const std::vector<glm::ivec3>& seeds) {
  if (!scene_ || !isStructural(shapeIndex) || seeds.empty()) {
    return;
  }
  const VoxelObject& o = scene_->cpuObject(shapeIndex);
  if (!o.enabled || o.cells.empty()) {
    return;
  }
  const auto snap = snapshotIslandSleep();

  std::unordered_set<uint64_t> dropKeys;
  std::vector<glm::ivec3> region;
  region.reserve(seeds.size() * 7);
  auto consider = [&](glm::ivec3 c) {
    if (c.x < 0 || c.y < 0 || c.z < 0 || c.x >= o.gridSize || c.y >= o.gridSize ||
        c.z >= o.gridSize) {
      return;
    }
    const uint64_t key = nodeKey(shapeIndex, c);
    if (dropKeys.insert(key).second) {
      region.push_back(c);
    }
  };
  const glm::ivec3 face[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const glm::ivec3& s : seeds) {
    consider(s);
    for (const glm::ivec3& d : face) {
      consider(s + d);
    }
  }
  if (region.size() * 4u > nodes_.size() && nodes_.size() > 64) {
    const auto snap = snapshotIslandSleep();
    rebuildShape(shapeIndex);
    rebuildIslands(&snap);
    for (const glm::ivec3& s : seeds) {
      wakeIslandOfCoarse(shapeIndex, s);
    }
    return;
  }

  std::unordered_map<uint64_t, Bond> warm;
  std::unordered_map<uint64_t, BondNode> prevNodes;
  for (const Bond& b : bonds_) {
    if (b.a < 0 || b.b < 0 || b.a >= static_cast<int>(nodes_.size()) ||
        b.b >= static_cast<int>(nodes_.size())) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    if (na.shapeIndex != shapeIndex) {
      continue;
    }
    if (dropKeys.count(nodeKey(shapeIndex, na.coarse)) != 0 ||
        dropKeys.count(nodeKey(shapeIndex, nodes_[static_cast<size_t>(b.b)].coarse)) != 0) {
      warm[bondKey(na.coarse, b.axis)] = b;
    }
  }
  for (const BondNode& n : nodes_) {
    if (n.shapeIndex == shapeIndex && dropKeys.count(nodeKey(shapeIndex, n.coarse)) != 0) {
      prevNodes[nodeKey(shapeIndex, n.coarse)] = n;
    }
  }

  dropShapeRegion(shapeIndex, &dropKeys);

  for (const glm::ivec3& c : region) {
    const auto it = prevNodes.find(nodeKey(shapeIndex, c));
    addNode(shapeIndex, c, it == prevNodes.end() ? nullptr : &it->second);
  }
  std::unordered_set<uint64_t> created;
  created.reserve(region.size() * 6u);
  for (const Bond& b : bonds_) {
    if (b.alive && b.a >= 0 && b.a < static_cast<int>(nodes_.size())) {
      const BondNode& na = nodes_[static_cast<size_t>(b.a)];
      if (na.shapeIndex == shapeIndex) {
        created.insert(bondKey(na.coarse, b.axis));
      }
    }
  }
  for (const glm::ivec3& c : region) {
    for (int axis = 0; axis < 3; ++axis) {
      addBondIfMissing(shapeIndex, c, axis, &warm, &created);
      glm::ivec3 lower = c;
      lower[axis] -= 1;
      addBondIfMissing(shapeIndex, lower, axis, &warm, &created);
    }
  }
  rebuildIslands(&snap);
  for (const glm::ivec3& s : seeds) {
    wakeIslandOfCoarse(shapeIndex, s);
  }
}

void StructureWorld::rebuildGraphColors() {
  graph_.beginAssign(static_cast<int>(nodes_.size()));
  for (int i = 0; i < kGraphBucketCount; ++i) {
    graph_.bucket(i).clear();
  }
  for (int bi = 0; bi < static_cast<int>(bonds_.size()); ++bi) {
    Bond& b = bonds_[static_cast<size_t>(bi)];
    if (!b.alive || b.a < 0 || b.b < 0) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    const BondNode& nb = nodes_[static_cast<size_t>(b.b)];
    b.color = graph_.assignBond(b.a, b.b, na.anchored, nb.anchored);
    graph_.bucket(b.color).push_back(bi);
  }
}

void StructureWorld::applyGhostGravity() {
  // Quasi-static: kinetic energy is not state. Each substep applies a fresh
  // gravity impulse onto v=0; TGS turns it into λ. Sleeping islands are not
  // in the solver (Box3D).
  for (BondNode& n : nodes_) {
    if (n.anchored) {
      n.u = glm::vec3(0.0f);
      n.theta = glm::vec3(0.0f);
      n.v = glm::vec3(0.0f);
      n.w = glm::vec3(0.0f);
    }
  }
  for (const Island& is : islands_.all()) {
    if (is.asleep) {
      continue;
    }
    for (int ni : is.nodes) {
      if (ni < 0 || ni >= static_cast<int>(nodes_.size())) {
        continue;
      }
      BondNode& n = nodes_[static_cast<size_t>(ni)];
      n.v = glm::vec3(0.0f);
      n.w = glm::vec3(0.0f);
      if (n.anchored) {
        continue;
      }
      n.v += kGravity * kSubDt;
    }
  }
}

void StructureWorld::solveAxis(BondNode& na, BondNode& nb, const glm::vec3& n, const glm::vec3& ra,
                               const glm::vec3& rb, float C, float& lambda, float kn, float h) {
  if (na.anchored && nb.anchored) {
    return;
  }
  const glm::vec3 va = na.v + glm::cross(na.w, ra);
  const glm::vec3 vb = nb.v + glm::cross(nb.w, rb);
  const float Cdot = glm::dot(n, vb - va);
  const float invMa = na.anchored ? 0.0f : 1.0f / std::max(na.mass, 1e-6f);
  const float invMb = nb.anchored ? 0.0f : 1.0f / std::max(nb.mass, 1e-6f);
  const float Keff = invMa + invMb + angTerm(na, ra, n) + angTerm(nb, rb, n);
  const float cfm = kBondCfm + 1.0f / (h * h * std::max(kn, 1.0f) + 1.0f);
  const float denom = Keff + cfm;
  if (denom <= 1e-12f) {
    return;
  }
  const float dLam = -(Cdot + kBondBeta * C / h) / denom;
  lambda += dLam;
  const glm::vec3 J = dLam * n;
  if (!na.anchored) {
    na.v -= J * invMa;
    const glm::vec3 iinv(1.0f / std::max(na.Idiag.x, 1e-6f), 1.0f / std::max(na.Idiag.y, 1e-6f),
                         1.0f / std::max(na.Idiag.z, 1e-6f));
    na.w -= iinv * glm::cross(ra, J);
  }
  if (!nb.anchored) {
    nb.v += J * invMb;
    const glm::vec3 iinv(1.0f / std::max(nb.Idiag.x, 1e-6f), 1.0f / std::max(nb.Idiag.y, 1e-6f),
                         1.0f / std::max(nb.Idiag.z, 1e-6f));
    nb.w += iinv * glm::cross(rb, J);
  }
}

void StructureWorld::solveBond(Bond& bond, float h) {
  if (!bond.alive || bond.a < 0 || bond.b < 0) {
    return;
  }
  BondNode& na = nodes_[static_cast<size_t>(bond.a)];
  BondNode& nb = nodes_[static_cast<size_t>(bond.b)];
  const glm::vec3 n = axisUnit(bond.axis);
  const glm::vec3 t1 = tangent1(bond.axis);
  const glm::vec3 t2 = tangent2(bond.axis);
  const glm::vec3 xa = na.rest + na.u;
  const glm::vec3 xb = nb.rest + nb.u;
  const glm::vec3 ra = 0.5f * bond.L * n;
  const glm::vec3 rb = -0.5f * bond.L * n;
  const glm::vec3 d = xb - xa;
  const glm::vec3 dAng = nb.theta - na.theta;

  solveAxis(na, nb, n, ra, rb, glm::dot(d, n) - bond.L, bond.lambda[0], bond.kn, h);
  solveAxis(na, nb, t1, ra, rb, glm::dot(d, t1), bond.lambda[1], bond.kv, h);
  solveAxis(na, nb, t2, ra, rb, glm::dot(d, t2), bond.lambda[2], bond.kv, h);
  solveAxis(na, nb, n, ra, rb, glm::dot(dAng, n), bond.lambda[3], bond.kt, h);
  solveAxis(na, nb, t1, ra, rb, glm::dot(dAng, t1), bond.lambda[4], bond.km, h);
  solveAxis(na, nb, t2, ra, rb, glm::dot(dAng, t2), bond.lambda[5], bond.km, h);

  if (na.anchored) {
    na.v = glm::vec3(0.0f);
    na.w = glm::vec3(0.0f);
    na.u = glm::vec3(0.0f);
    na.theta = glm::vec3(0.0f);
  }
  if (nb.anchored) {
    nb.v = glm::vec3(0.0f);
    nb.w = glm::vec3(0.0f);
    nb.u = glm::vec3(0.0f);
    nb.theta = glm::vec3(0.0f);
  }
}

void StructureWorld::clampBondLambda(Bond& bond, float h) const {
  if (!bond.alive || h <= 1e-12f) {
    return;
  }
  const float lamT = bond.A * bond.strength.sigmaT * h;
  const float lamC = bond.A * bond.strength.sigmaC * h;
  bond.lambda[0] = std::clamp(bond.lambda[0], -lamC, lamT);
}

void StructureWorld::solveBonds() {
  for (int iter = 0; iter < kBondIters; ++iter) {
    for (int c = 0; c < kGraphBucketCount; ++c) {
      for (int bi : graph_.bucket(c)) {
        if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
          continue;
        }
        Bond& bond = bonds_[static_cast<size_t>(bi)];
        if (!islandAwake(bond.islandId)) {
          continue;
        }
        solveBond(bond, kSubDt);
      }
    }
  }
}

float StructureWorld::bondPhi(const Bond& bond, float h) const {
  if (!bond.alive || h <= 1e-12f) {
    return 0.0f;
  }
  const BondNode& na = nodes_[static_cast<size_t>(bond.a)];
  const BondNode& nb = nodes_[static_cast<size_t>(bond.b)];
  if (na.anchored && nb.anchored) {
    return 0.0f;
  }
  const float N = bond.lambda[0] / h;
  const float V = std::sqrt(bond.lambda[1] * bond.lambda[1] + bond.lambda[2] * bond.lambda[2]) / h;
  const float T = std::abs(bond.lambda[3]) / h;
  const float M = std::sqrt(bond.lambda[4] * bond.lambda[4] + bond.lambda[5] * bond.lambda[5]) / h;
  const float Mu = std::max(bond.strength.sigmaT * bond.I / std::max(0.5f * bond.L, 1e-4f), 1e-4f);
  const float Tu = std::max(bond.strength.tauU * bond.J / std::max(0.5f * bond.L, 1e-4f), 1e-4f);
  float phiNm = 0.0f;
  if (N > 0.0f) {
    phiNm = N / std::max(bond.A * bond.strength.sigmaT, 1e-4f) + std::abs(M) / Mu;
  } else {
    phiNm = std::abs(N) / std::max(bond.A * bond.strength.sigmaC, 1e-4f) + std::abs(M) / Mu;
  }
  (void)T;
  (void)Tu;
  return phiNm + std::abs(V) / std::max(bond.A * bond.strength.tauU, 1e-4f);
}

int StructureWorld::breakOverstressed() {
  maxPhi_ = 0.0f;
  newlyBroken_.clear();
  std::vector<int> over;
  over.reserve(16);
  for (Island& is : islands_.all()) {
    if (is.asleep) {
      continue;
    }
    const bool settling = is.settleLeft > 0;
    for (int bi : is.bonds) {
      if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
        continue;
      }
      Bond& b = bonds_[static_cast<size_t>(bi)];
      if (!b.alive) {
        continue;
      }
      b.phi = bondPhi(b, kSubDt);
      maxPhi_ = std::max(maxPhi_, b.phi);
      const BondNode& na = nodes_[static_cast<size_t>(b.a)];
      const BondNode& nb = nodes_[static_cast<size_t>(b.b)];
      if (na.anchored && nb.anchored) {
        continue;
      }
      if (!settling && b.phi >= 1.0f) {
        over.push_back(bi);
      }
    }
  }
  std::sort(over.begin(), over.end(), [&](int i, int j) {
    return bonds_[static_cast<size_t>(i)].phi > bonds_[static_cast<size_t>(j)].phi;
  });
  int n = 0;
  const int dtLeft = std::max(0, kMaxBrokenBondsPerDt - brokenThisDt_);
  const int cap = std::min(kMaxBrokenBondsPerSubstep, dtLeft);
  for (int i : over) {
    if (n >= cap) {
      break;
    }
    Bond& b = bonds_[static_cast<size_t>(i)];
    b.alive = false;
    newlyBroken_.push_back(i);
    if (Island* is = islands_.byId(b.islandId)) {
      is->needsSplit = true;
      ++is->brokenThisDt;
    }
    ++n;
  }
  for (Bond& b : bonds_) {
    if (b.alive && islandAwake(b.islandId)) {
      clampBondLambda(b, kSubDt);
    }
  }
  return n;
}

void StructureWorld::fractureDeadBonds() {
  if (!scene_ || newlyBroken_.empty() || splitConsumedThisDt_) {
    return;
  }
  int shape = -1;
  std::vector<glm::ivec3> cuts;
  cuts.reserve(newlyBroken_.size() * 16);
  for (int bi : newlyBroken_) {
    if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
      continue;
    }
    const Bond& b = bonds_[static_cast<size_t>(bi)];
    if (b.a < 0 || b.b < 0 || b.a >= static_cast<int>(nodes_.size()) ||
        b.b >= static_cast<int>(nodes_.size())) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    const BondNode& nb = nodes_[static_cast<size_t>(b.b)];
    if (shape < 0) {
      shape = na.shapeIndex;
    }
    if (na.shapeIndex != shape || nb.shapeIndex != shape) {
      continue;
    }
    if (VoxelScene::isTerrainCoarse(shape, na.coarse) &&
        VoxelScene::isTerrainCoarse(shape, nb.coarse)) {
      continue;
    }
    scene_->cutCoarseInterface(shape, na.coarse, nb.coarse, &cuts);
  }
  if (shape < 0 || cuts.empty()) {
    return;
  }
  const bool split = scene_->fractureFromCuts(shape, cuts);
  scene_->noteOccupancyGpuDirty();
  std::vector<glm::ivec3> dirty;
  dirty.reserve(newlyBroken_.size() * 2);
  for (int bi : newlyBroken_) {
    if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
      continue;
    }
    const Bond& b = bonds_[static_cast<size_t>(bi)];
    if (b.a < 0 || b.b < 0 || b.a >= static_cast<int>(nodes_.size()) ||
        b.b >= static_cast<int>(nodes_.size())) {
      continue;
    }
    dirty.push_back(nodes_[static_cast<size_t>(b.a)].coarse);
    dirty.push_back(nodes_[static_cast<size_t>(b.b)].coarse);
  }
  if (split) {
    splitConsumedThisDt_ = true;
    markDirty(shape);
  } else {
    rebuildNeighborhood(shape, dirty);
  }
}

void StructureWorld::applyContactImpulses(const std::vector<Contact>& contacts,
                                          const std::vector<RigidBody>& bodies) {
  if (!scene_ || contacts.empty() || splitConsumedThisDt_) {
    return;
  }
  newlyBroken_.clear();
  auto bodySpeed = [&](int shape) -> float {
    if (shape < 0 || shape >= static_cast<int>(bodies.size())) {
      return 0.0f;
    }
    const RigidBody& b = bodies[static_cast<size_t>(shape)];
    return glm::length(b.v) + glm::length(b.w) * 0.5f;
  };
  auto consider = [&](int shape, uint32_t packedFine, float impulse, float impactSpeed) {
    if (shape != 0 || impulse < 0.0f || shape >= scene_->cpuObjectCount()) {
      return;
    }
    if (impactSpeed < kImpactSpeed) {
      return;
    }
    const VoxelObject& o = scene_->cpuObject(shape);
    const int n = o.gridSize * VoxelScene::kFinePerCoarse;
    if (n <= 0) {
      return;
    }
    const glm::ivec3 p = unpackFine(packedFine, n);
    const int F = VoxelScene::kFinePerCoarse;
    const glm::ivec3 c(p.x / F, p.y / F, p.z / F);
    if (c.y < VoxelScene::kGroundCoarseY) {
      return;
    }
    const uint32_t mat = scene_->occupancyMaterial(shape, c);
    if (mat == 0u) {
      return;
    }
    const BondStrength st = strengthForMaterial(mat);
    if (impulse < st.jBreak) {
      return;
    }
    const int ni = findNode(shape, c);
    if (ni < 0) {
      return;
    }
    for (int bi = 0; bi < static_cast<int>(bonds_.size()); ++bi) {
      Bond& b = bonds_[static_cast<size_t>(bi)];
      if (!b.alive || (b.a != ni && b.b != ni)) {
        continue;
      }
      b.alive = false;
      newlyBroken_.push_back(bi);
    }
  };
  for (const Contact& c : contacts) {
    const float sp = std::max(bodySpeed(c.a), bodySpeed(c.b));
    consider(c.a, c.fineA, c.lambdaN, sp);
    consider(c.b, c.fineB, c.lambdaN, sp);
  }
  if (!newlyBroken_.empty()) {
    for (int bi : newlyBroken_) {
      if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
        continue;
      }
      islands_.wake(bonds_[static_cast<size_t>(bi)].islandId);
    }
    fractureDeadBonds();
  }
}

void StructureWorld::integrateGhost() {
  for (Island& is : islands_.all()) {
    if (is.asleep) {
      continue;
    }
    float vmax = 0.0f;
    for (int ni : is.nodes) {
      if (ni < 0 || ni >= static_cast<int>(nodes_.size())) {
        continue;
      }
      BondNode& n = nodes_[static_cast<size_t>(ni)];
      if (n.anchored) {
        n.u = glm::vec3(0.0f);
        n.theta = glm::vec3(0.0f);
        n.v = glm::vec3(0.0f);
        n.w = glm::vec3(0.0f);
        continue;
      }
      vmax = std::max(vmax, glm::length(n.v) + glm::length(n.w) * 0.5f);
      n.u += n.v * kSubDt;
      n.theta += n.w * kSubDt;
      const float ulen = glm::length(n.u);
      if (ulen > kBondUMax) {
        n.u *= kBondUMax / ulen;
      }
      const float tlen = glm::length(n.theta);
      if (tlen > kBondThetaMax) {
        n.theta *= kBondThetaMax / tlen;
      }
      n.v = glm::vec3(0.0f);
      n.w = glm::vec3(0.0f);
    }
    is.vmax = vmax;
  }
}

void StructureWorld::fillCoarsePhi(std::vector<float>& dst) const {
  if (!scene_ || dst.empty()) {
    return;
  }
  for (const Bond& b : bonds_) {
    if (!b.alive || b.a < 0 || b.b < 0) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    const BondNode& nb = nodes_[static_cast<size_t>(b.b)];
    if (na.anchored && nb.anchored) {
      continue;
    }
    const float phi = std::max(b.phi, 0.0f);
    auto scatter = [&](const BondNode& n) {
      const uint32_t idx = scene_->poolCellIndex(n.shapeIndex, n.coarse);
      if (idx >= dst.size()) {
        return;
      }
      dst[idx] = std::max(dst[idx], phi);
    };
    scatter(na);
    scatter(nb);
  }
}

void StructureWorld::snapshotDebug() {
  debugBonds_.clear();
  debugBonds_.reserve(bonds_.size());
  for (const Bond& b : bonds_) {
    if (b.a < 0 || b.b < 0) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    const BondNode& nb = nodes_[static_cast<size_t>(b.b)];
    DebugBond d;
    d.a = na.rest + na.u;
    d.b = nb.rest + nb.u;
    d.phi = b.phi;
    d.alive = b.alive;
    debugBonds_.push_back(d);
  }
}

void StructureWorld::substep() {
  brokenThisStep_ = 0;
  if (!scene_ || nodes_.empty()) {
    return;
  }
  bool anyAwake = false;
  for (const Island& is : islands_.all()) {
    if (!is.asleep && !is.nodes.empty()) {
      anyAwake = true;
      break;
    }
  }
  if (!anyAwake) {
    return;
  }

  applyGhostGravity();
  solveBonds();
  brokenThisStep_ = breakOverstressed();
  brokenThisDt_ += brokenThisStep_;
  if (brokenThisStep_ > 0) {
    solveBonds();
    fractureDeadBonds();
  }
  integrateGhost();
}

void StructureWorld::updateSleep(float h) {
  for (Island& is : islands_.all()) {
    if (is.settleLeft > 0) {
      --is.settleLeft;
    }
    if (is.nodes.empty()) {
      is.asleep = true;
      continue;
    }
    if (is.brokenThisDt > 0 || is.settleLeft > 0) {
      is.sleepTimer = 0.0f;
      is.asleep = false;
      continue;
    }
    if (is.vmax < kSleepLin) {
      is.sleepTimer += h;
      if (is.sleepTimer >= kSleepTime) {
        is.asleep = true;
        for (int ni : is.nodes) {
          if (ni < 0 || ni >= static_cast<int>(nodes_.size())) {
            continue;
          }
          BondNode& n = nodes_[static_cast<size_t>(ni)];
          n.v = glm::vec3(0.0f);
          n.w = glm::vec3(0.0f);
        }
      }
    } else {
      is.sleepTimer = 0.0f;
      is.asleep = false;
    }
  }
  maxPhiPrev_ = maxPhi_;
  snapshotDebug();
}

void StructureWorld::fillDebug(DebugSolve& debug) const {
  debug.bondsAlive = 0;
  debug.bondsBrokenThisStep = brokenThisDt_;
  debug.maxPhi = maxPhi_;
  for (const Bond& b : bonds_) {
    if (b.alive) {
      ++debug.bondsAlive;
    }
  }
}

}  // namespace physics
