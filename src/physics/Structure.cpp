#include "physics/Structure.h"

#include "scene/VoxelScene.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace physics {
namespace {

constexpr int kGroundCoarseY = 2;
constexpr float kShearAlpha = 1.2f;
constexpr float kBondBeta = 0.12f;
constexpr float kBondCfm = 0.05f;
constexpr float kGhostDamp = 0.82f;

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
  rebuildGraphColors();
  snapshotDebug();
}

void StructureWorld::markDirty(int shapeIndex) {
  rebuildShape(shapeIndex);
  rebuildGraphColors();
  islands_.wakeShape(shapeIndex);
  snapshotDebug();
}

void StructureWorld::onSplit(int srcIndex, int dstIndex) {
  rebuildShape(srcIndex);
  rebuildShape(dstIndex);
  rebuildGraphColors();
  islands_.wakeShape(srcIndex);
  snapshotDebug();
  (void)dstIndex;
}

void StructureWorld::beginDt() {
  splitConsumedThisDt_ = false;
  brokenThisDt_ = 0;
}

void StructureWorld::wakeShape(int shapeIndex) { islands_.wakeShape(shapeIndex); }

void StructureWorld::wakeFromContacts(const std::vector<Contact>& contacts) {
  if (!scene_) {
    return;
  }
  for (const Contact& c : contacts) {
    if (c.a >= 0) {
      islands_.wakeShape(c.a);
    }
    if (c.b >= 0) {
      islands_.wakeShape(c.b);
    }
  }
}

uint64_t StructureWorld::nodeKey(int shapeIndex, const glm::ivec3& c) const {
  return (static_cast<uint64_t>(static_cast<uint32_t>(shapeIndex)) << 32) |
         (static_cast<uint32_t>(c.x) & 1023u) |
         ((static_cast<uint32_t>(c.y) & 1023u) << 10) |
         ((static_cast<uint32_t>(c.z) & 1023u) << 20);
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
  }
  return s;
}

void StructureWorld::rebuildShape(int shapeIndex) {
  if (!scene_ || shapeIndex < 0 || shapeIndex >= scene_->cpuObjectCount()) {
    return;
  }
  // Drop existing nodes/bonds for this shape.
  std::vector<BondNode> keptNodes;
  std::unordered_map<uint64_t, int> keptIndex;
  keptNodes.reserve(nodes_.size());
  for (const BondNode& n : nodes_) {
    if (n.shapeIndex == shapeIndex) {
      continue;
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
    if (n.shapeIndex == shapeIndex) {
      return -1;
    }
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

  const VoxelObject& o = scene_->cpuObject(shapeIndex);
  const bool staticShape = shapeIndex == 0;
  if (!staticShape || !o.enabled || o.cells.empty() || o.gridSize <= 0) {
    Island* is = islands_.findByShape(shapeIndex);
    if (is) {
      is->nodes.clear();
    }
    return;
  }

  Island& island = islands_.ensureForShape(shapeIndex);
  island.nodes.clear();
  island.asleep = false;
  island.sleepTimer = 0.0f;
  island.settleLeft = std::max(island.settleLeft, 6);

  const glm::mat4 o2w = o.objectToWorld();
  const float a = o.voxelSize;
  const float fineV = kVoxel * kVoxel * kVoxel;

  auto addNode = [&](const glm::ivec3& c) {
    const uint32_t mat = scene_->occupancyMaterial(shapeIndex, c);
    if (mat == 0u) {
      return;
    }
    const uint32_t nFine = scene_->occupiedFineCount(shapeIndex, c);
    if (nFine == 0u) {
      return;
    }
    const BondStrength st = strengthForMaterial(mat);
    BondNode node;
    node.shapeIndex = shapeIndex;
    node.coarse = c;
    node.mass = std::max(1e-3f, st.density * fineV * static_cast<float>(nFine));
    const float ii = node.mass * a * a / 6.0f;
    node.Idiag = glm::vec3(std::max(ii, 1e-4f));
    const glm::vec3 local = (glm::vec3(c) + glm::vec3(0.5f)) * a;
    node.rest = glm::vec3(o2w * glm::vec4(local, 1.0f));
    node.anchored = (shapeIndex == 0 && c.y < kGroundCoarseY);
    node.u = glm::vec3(0.0f);
    node.theta = glm::vec3(0.0f);
    node.v = glm::vec3(0.0f);
    node.w = glm::vec3(0.0f);
    const int idx = static_cast<int>(nodes_.size());
    nodeIndex_[nodeKey(shapeIndex, c)] = idx;
    nodes_.push_back(node);
    island.nodes.push_back(idx);
  };

  if (!o.occupiedCoarses.empty()) {
    for (uint32_t packed : o.occupiedCoarses) {
      addNode(glm::ivec3(static_cast<int>(packed & 1023u),
                         static_cast<int>((packed >> 10) & 1023u),
                         static_cast<int>((packed >> 20) & 1023u)));
    }
  } else {
    for (int z = 0; z < o.gridSize; ++z) {
      for (int y = 0; y < o.gridSize; ++y) {
        for (int x = 0; x < o.gridSize; ++x) {
          addNode(glm::ivec3(x, y, z));
        }
      }
    }
  }

  auto solidFrac = [&](const glm::ivec3& c) -> float {
    return static_cast<float>(scene_->occupiedFineCount(shapeIndex, c)) /
           static_cast<float>(VoxelScene::kFinePerBrick);
  };

  for (int i : island.nodes) {
    const BondNode& na = nodes_[static_cast<size_t>(i)];
    for (int axis = 0; axis < 3; ++axis) {
      glm::ivec3 nbC = na.coarse;
      nbC[axis] += 1;
      const int j = findNode(shapeIndex, nbC);
      if (j < 0) {
        continue;
      }
      const BondNode& nb = nodes_[static_cast<size_t>(j)];
      const uint32_t matA = scene_->occupancyMaterial(shapeIndex, na.coarse);
      const uint32_t matB = scene_->occupancyMaterial(shapeIndex, nb.coarse);
      const BondStrength st = strengthForMaterial(std::min(matA, matB));
      const float fill = std::min(solidFrac(na.coarse), solidFrac(nb.coarse));
      Bond bond;
      bond.a = i;
      bond.b = j;
      bond.axis = axis;
      bond.L = a;
      bond.A = std::max(1e-4f, a * a * std::max(fill, 1.0f / 16.0f));
      bond.I = bond.A * bond.A / 12.0f;
      bond.J = bond.A * bond.A / 6.0f;
      bond.kn = st.E * bond.A / bond.L;
      bond.kv = st.G * bond.A / (kShearAlpha * bond.L);
      bond.kt = st.G * bond.J / bond.L;
      bond.km = st.E * bond.I / bond.L;
      bond.strength = st;
      bond.alive = true;
      bonds_.push_back(bond);
      (void)nb;
    }
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
  for (BondNode& n : nodes_) {
    if (n.anchored) {
      n.v = glm::vec3(0.0f);
      n.w = glm::vec3(0.0f);
      continue;
    }
    n.v += kGravity * kSubDt;
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

void StructureWorld::solveBonds() {
  for (int iter = 0; iter < kBondIters; ++iter) {
    for (int c = 0; c < kGraphBucketCount; ++c) {
      for (int bi : graph_.bucket(c)) {
        if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
          continue;
        }
        solveBond(bonds_[static_cast<size_t>(bi)], kSubDt);
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
  const float M = std::sqrt(bond.lambda[4] * bond.lambda[4] + bond.lambda[5] * bond.lambda[5]) / h;
  const float Mu = std::max(bond.strength.sigmaT * bond.I / std::max(0.5f * bond.L, 1e-4f), 1e-4f);
  float phiNm = 0.0f;
  if (N > 0.0f) {
    phiNm = N / std::max(bond.A * bond.strength.sigmaT, 1e-4f) + std::abs(M) / Mu;
  } else {
    phiNm = std::abs(N) / std::max(bond.A * bond.strength.sigmaC, 1e-4f) + std::abs(M) / Mu;
  }
  return phiNm + std::abs(V) / std::max(bond.A * bond.strength.tauU, 1e-4f);
}

int StructureWorld::breakOverstressed() {
  maxPhi_ = 0.0f;
  newlyBroken_.clear();
  bool settling = false;
  float ghostSpeed = 0.0f;
  for (const Island& is : islands_.all()) {
    if (is.settleLeft > 0) {
      settling = true;
    }
  }
  for (const BondNode& n : nodes_) {
    if (!n.anchored) {
      ghostSpeed = std::max(ghostSpeed, glm::length(n.v));
    }
  }
  std::vector<int> over;
  over.reserve(16);
  for (int i = 0; i < static_cast<int>(bonds_.size()); ++i) {
    Bond& b = bonds_[static_cast<size_t>(i)];
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
    if (!settling && ghostSpeed < 0.4f && b.phi >= 1.0f) {
      over.push_back(i);
    }
  }
  std::sort(over.begin(), over.end(), [&](int i, int j) {
    return bonds_[static_cast<size_t>(i)].phi > bonds_[static_cast<size_t>(j)].phi;
  });
  int n = 0;
  for (int i : over) {
    if (n >= kMaxBrokenBondsPerSubstep) {
      break;
    }
    bonds_[static_cast<size_t>(i)].alive = false;
    newlyBroken_.push_back(i);
    ++n;
  }
  return n;
}

void StructureWorld::cutBrokenBonds() {
  if (newlyBroken_.empty()) {
    return;
  }
  for (int bi : newlyBroken_) {
    if (bi < 0 || bi >= static_cast<int>(bonds_.size())) {
      continue;
    }
    const Bond& b = bonds_[static_cast<size_t>(bi)];
    if (b.a < 0 || b.b < 0) {
      continue;
    }
    const BondNode& na = nodes_[static_cast<size_t>(b.a)];
    Island* is = islands_.findByShape(na.shapeIndex);
    if (is) {
      is->needsSplit = true;
    }
  }
}

void StructureWorld::peelUnanchored() {
  if (!scene_ || splitConsumedThisDt_) {
    return;
  }
  for (Island& island : islands_.all()) {
    if (island.shapeIndex < 0 || island.nodes.empty()) {
      continue;
    }
    if (!islands_.consumeSplit(island.shapeIndex) && brokenThisStep_ == 0) {
      continue;
    }
    splitConsumedThisDt_ = true;

    std::unordered_map<int, int> local;
    std::vector<int> ids;
    ids.reserve(island.nodes.size());
    for (int ni : island.nodes) {
      if (ni < 0 || ni >= static_cast<int>(nodes_.size())) {
        continue;
      }
      if (nodes_[static_cast<size_t>(ni)].shapeIndex != island.shapeIndex) {
        continue;
      }
      local[ni] = static_cast<int>(ids.size());
      ids.push_back(ni);
    }
    if (ids.empty()) {
      return;
    }
    std::vector<int> parent(ids.size());
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
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
      if (!b.alive) {
        continue;
      }
      const auto ia = local.find(b.a);
      const auto ib = local.find(b.b);
      if (ia == local.end() || ib == local.end()) {
        continue;
      }
      unite(ia->second, ib->second);
    }

    std::vector<uint8_t> anchoredRoot(ids.size(), 0);
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
      if (nodes_[static_cast<size_t>(ids[static_cast<size_t>(i)])].anchored) {
        anchoredRoot[static_cast<size_t>(find(i))] = 1;
      }
    }
    std::unordered_map<int, std::vector<glm::ivec3>> pieces;
    for (int i = 0; i < static_cast<int>(ids.size()); ++i) {
      const int r = find(i);
      if (anchoredRoot[static_cast<size_t>(r)]) {
        continue;
      }
      pieces[r].push_back(nodes_[static_cast<size_t>(ids[static_cast<size_t>(i)])].coarse);
    }
    for (auto& kv : pieces) {
      if (kv.second.empty()) {
        continue;
      }
      scene_->peelCoarseIslands(island.shapeIndex, kv.second);
      scene_->noteOccupancyGpuDirty();
    }
    markDirty(island.shapeIndex);
    return;  // at most one island split per dt
  }
}

void StructureWorld::integrateGhost() {
  for (BondNode& n : nodes_) {
    if (n.anchored) {
      n.u = glm::vec3(0.0f);
      n.theta = glm::vec3(0.0f);
      n.v = glm::vec3(0.0f);
      n.w = glm::vec3(0.0f);
      continue;
    }
    n.u += n.v * kSubDt;
    n.theta += n.w * kSubDt;
    n.v *= kGhostDamp;
    n.w *= kGhostDamp;
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
    cutBrokenBonds();
    peelUnanchored();
    rebuildGraphColors();
  }
  integrateGhost();
  snapshotDebug();
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
    float vmax = 0.0f;
    for (int ni : is.nodes) {
      if (ni < 0 || ni >= static_cast<int>(nodes_.size())) {
        continue;
      }
      const BondNode& n = nodes_[static_cast<size_t>(ni)];
      if (n.anchored) {
        continue;
      }
      vmax = std::max(vmax, glm::length(n.v) + glm::length(n.w) * 0.5f);
    }
    if (vmax < kSleepLin && brokenThisDt_ == 0) {
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
