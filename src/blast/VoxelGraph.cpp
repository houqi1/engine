#include "blast/VoxelGraph.h"

#include "NvBlastGlobals.h"
#include "NvBlastTypes.h"
#include "NvCTypes.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <queue>
#include <unordered_set>

namespace blast {
namespace {

void blastLog(int type, const char* msg, const char* file, int line) { Nv::Blast::logLL(type, msg, file, line); }

const int kDx[6] = {1, -1, 0, 0, 0, 0};
const int kDy[6] = {0, 0, 1, -1, 0, 0};
const int kDz[6] = {0, 0, 0, 0, 1, -1};

uint64_t faceBetween(int x, int y, int z, int nx, int ny, int nz) {
  int ax = x, ay = y, az = z, axis = 0;
  if (nx != x) {
    axis = 0;
    ax = std::min(x, nx);
  } else if (ny != y) {
    axis = 1;
    ay = std::min(y, ny);
  } else {
    axis = 2;
    az = std::min(z, nz);
  }
  return packFaceKey(ax, ay, az, axis);
}

bool faceOpen(const VoxelGrid& g, int x, int y, int z, int nx, int ny, int nz) {
  return g.brokenFaces.count(faceBetween(x, y, z, nx, ny, nz)) == 0;
}

AlignedBlock allocBlock(TrackingAllocator& alloc, size_t n, const char* name) {
  AlignedBlock b;
  b.alloc = &alloc;
  b.ptr = alloc.allocate(n, name, __FILE__, __LINE__);
  return b;
}

bool allocStableId(uint32_t& next, uint32_t& out) {
  if (next == 0) {
    return false;
  }
  out = next;
  if (next == (std::numeric_limits<uint32_t>::max)()) {
    next = 0;
  } else {
    ++next;
  }
  return true;
}

void bumpNextPast(uint32_t& next, uint32_t id) {
  if (next == 0) {
    return;
  }
  if (id < next) {
    return;
  }
  if (id == (std::numeric_limits<uint32_t>::max)()) {
    next = 0;
  } else {
    next = id + 1;
  }
}

struct PendingNode {
  std::vector<VoxelCoord> voxels;
  std::unordered_map<uint32_t, int> overlap;
  OwnerId owner = kUnowned;
  VoxelCoord minCoord{};
};

struct IdCandidate {
  int overlap = 0;
  VoxelCoord minCoord{};
  uint32_t newIndex = 0;
  uint32_t oldId = 0;
};

bool candidateLess(const IdCandidate& a, const IdCandidate& b) {
  if (a.overlap != b.overlap) {
    return a.overlap > b.overlap;
  }
  if (a.minCoord < b.minCoord) {
    return true;
  }
  if (b.minCoord < a.minCoord) {
    return false;
  }
  if (a.newIndex != b.newIndex) {
    return a.newIndex < b.newIndex;
  }
  return a.oldId < b.oldId;
}

void finishNodeGeometry(GraphNode& node, const VoxelGrid& grid) {
  double sx = 0, sy = 0, sz = 0;
  for (const VoxelCoord& p : node.voxels) {
    sx += (static_cast<double>(p.x) + 0.5) * grid.voxelSize + grid.ox;
    sy += (static_cast<double>(p.y) + 0.5) * grid.voxelSize + grid.oy;
    sz += (static_cast<double>(p.z) + 0.5) * grid.voxelSize + grid.oz;
  }
  const float n = static_cast<float>(node.voxels.size());
  node.cx = static_cast<float>(sx / n);
  node.cy = static_cast<float>(sy / n);
  node.cz = static_cast<float>(sz / n);
  node.volume = n * grid.voxelSize * grid.voxelSize * grid.voxelSize;
  node.mass = grid.density * node.volume;
}

void finishBondGeometry(GraphBond& b) {
  if (b.nFaces <= 0) {
    return;
  }
  const float inv = 1.0f / static_cast<float>(b.nFaces);
  b.cx *= inv;
  b.cy *= inv;
  b.cz *= inv;
  const float len = std::sqrt(b.nx * b.nx + b.ny * b.ny + b.nz * b.nz);
  if (len > 1.0e-8f) {
    b.nx /= len;
    b.ny /= len;
    b.nz /= len;
  }
}

BlastError assignBondIdentities(const VoxelGrid& grid, const std::vector<GraphBond>& oldBonds, uint32_t& nextBondId,
                                std::vector<GraphBond>& bonds) {
  std::unordered_map<uint64_t, uint32_t> faceOldBond;
  for (uint32_t i = 0; i < static_cast<uint32_t>(oldBonds.size()); ++i) {
    for (uint64_t f : oldBonds[i].faces) {
      const auto it = faceOldBond.find(f);
      if (it != faceOldBond.end() && it->second != i) {
        return BlastError::AmbiguousDamage;
      }
      faceOldBond[f] = i;
    }
  }

  std::vector<int> source(bonds.size(), -1);
  std::vector<int> claimCount(oldBonds.size(), 0);
  for (size_t i = 0; i < bonds.size(); ++i) {
    std::unordered_set<uint32_t> src;
    for (uint64_t f : bonds[i].faces) {
      const auto it = faceOldBond.find(f);
      if (it != faceOldBond.end()) {
        src.insert(it->second);
      }
    }
    if (src.size() > 1) {
      return BlastError::AmbiguousDamage;
    }
    if (src.size() == 1) {
      const uint32_t s = *src.begin();
      source[i] = static_cast<int>(s);
      claimCount[s] += 1;
    }
  }

  std::unordered_set<uint32_t> usedIds;
  usedIds.reserve(bonds.size() * 2);
  for (size_t i = 0; i < bonds.size(); ++i) {
    GraphBond& b = bonds[i];
    float d = 0.0f;
    uint32_t keepId = 0;
    if (source[i] >= 0) {
      const GraphBond& old = oldBonds[static_cast<size_t>(source[i])];
      d = old.d;
      const auto oldKey = packBondKey(old.nodeA, old.nodeB);
      const auto dmgOld = grid.bondDamage.find(oldKey);
      if (dmgOld != grid.bondDamage.end()) {
        d = dmgOld->second;
      }
      if (claimCount[static_cast<size_t>(source[i])] == 1 && old.nodeA == b.nodeA && old.nodeB == b.nodeB) {
        keepId = old.stableId;
      }
    }
    const auto dmgNew = grid.bondDamage.find(packBondKey(b.nodeA, b.nodeB));
    if (dmgNew != grid.bondDamage.end()) {
      d = dmgNew->second;
    }
    b.d = std::clamp(d, 0.0f, 1.0f);
    if (keepId != 0 && usedIds.count(keepId) == 0) {
      b.stableId = keepId;
      usedIds.insert(keepId);
      bumpNextPast(nextBondId, keepId);
    } else {
      uint32_t id = 0;
      if (!allocStableId(nextBondId, id) || usedIds.count(id) != 0) {
        return BlastError::IdExhausted;
      }
      b.stableId = id;
      usedIds.insert(id);
    }
  }
  return BlastError::Ok;
}

BlastError validateFamilyGraph(const VoxelGrid& grid, const CompactFamily& cf) {
  std::unordered_set<uint32_t> nodeIds;
  nodeIds.reserve(cf.graph.nodes.size());
  std::unordered_set<VoxelCoord, VoxelCoordHash> seenVoxels;
  uint32_t voxelCount = 0;
  for (const GraphNode& n : cf.graph.nodes) {
    if (n.stableId == 0 || !nodeIds.insert(n.stableId).second) {
      return BlastError::MappingInvalid;
    }
    if (cf.blast.chunkFromStable.find(n.stableId) == cf.blast.chunkFromStable.end() ||
        cf.blast.graphFromStable.find(n.stableId) == cf.blast.graphFromStable.end()) {
      return BlastError::MappingInvalid;
    }
    for (const VoxelCoord& p : n.voxels) {
      if (!grid.isSolid(p.x, p.y, p.z) || grid.ownerAt(p.x, p.y, p.z) != cf.owner) {
        return BlastError::OwnerConflict;
      }
      if (!seenVoxels.insert(p).second) {
        return BlastError::OwnerConflict;
      }
      ++voxelCount;
    }
  }
  uint32_t expected = 0;
  for (int z = 0; z < grid.nz; ++z) {
    for (int y = 0; y < grid.ny; ++y) {
      for (int x = 0; x < grid.nx; ++x) {
        if (grid.isSolid(x, y, z) && grid.ownerAt(x, y, z) == cf.owner) {
          ++expected;
        }
      }
    }
  }
  if (voxelCount != expected) {
    return BlastError::OwnerConflict;
  }
  const float voxVol = grid.voxelSize * grid.voxelSize * grid.voxelSize;
  float mass = 0.0f;
  for (const GraphNode& n : cf.graph.nodes) {
    mass += n.mass;
  }
  const float expectMass = grid.density * voxVol * static_cast<float>(expected);
  if (expectMass > 0.0f && std::abs(mass - expectMass) > 1.0e-4f * expectMass) {
    return BlastError::MassMismatch;
  }

  std::unordered_set<uint32_t> bondIds;
  bondIds.reserve(cf.graph.bonds.size());
  for (const GraphBond& b : cf.graph.bonds) {
    if (b.stableId == 0 || !bondIds.insert(b.stableId).second) {
      return BlastError::MappingInvalid;
    }
    if (cf.blast.sdkBondFromStable.find(b.stableId) == cf.blast.sdkBondFromStable.end()) {
      return BlastError::MappingInvalid;
    }
    for (uint64_t f : b.faces) {
      if (grid.brokenFaces.count(f) != 0) {
        return BlastError::BrokenFaceReconnected;
      }
    }
  }
  return BlastError::Ok;
}

}  // namespace

const char* blastErrorMessage(BlastError e) {
  switch (e) {
    case BlastError::Ok:
      return "ok";
    case BlastError::EmptyGraph:
      return "empty graph";
    case BlastError::IdExhausted:
      return "stable id counter exhausted";
    case BlastError::AmbiguousDamage:
      return "bond damage source is ambiguous";
    case BlastError::OwnerConflict:
      return "voxel owner conflict or incomplete coverage";
    case BlastError::OwnerCapacity:
      return "owner id capacity exhausted";
    case BlastError::AssetCreateFailed:
      return "NvBlastCreateAsset failed";
    case BlastError::FamilyCreateFailed:
      return "NvBlastAssetCreateFamily failed";
    case BlastError::ActorCreateFailed:
      return "NvBlastFamilyCreateFirstActor failed";
    case BlastError::SolverCreateFailed:
      return "ExtStressSolver::create failed";
    case BlastError::MappingInvalid:
      return "id or sdk mapping invalid";
    case BlastError::MassMismatch:
      return "replacement mass does not match occupancy";
    case BlastError::BrokenFaceReconnected:
      return "broken face reconnected";
    case BlastError::InjectedFailure:
      return "injected rebuild failure";
    case BlastError::NotAnchored:
      return "no world-anchor voxels";
    case BlastError::DisconnectedFromAnchor:
      return "node cannot reach a world anchor";
    case BlastError::CavityBond:
      return "bond spans an empty cavity";
  }
  return "unknown blast error";
}

void initGrid(VoxelGrid& g, int nx, int ny, int nz, int agg, float voxelSize) {
  g.nx = nx;
  g.ny = ny;
  g.nz = nz;
  g.agg = agg;
  g.voxelSize = voxelSize;
  g.solid.assign(static_cast<size_t>(nx * ny * nz), 0);
  g.anchor.assign(static_cast<size_t>(nx * ny * nz), 0);
  g.owner.assign(static_cast<size_t>(nx * ny * nz), kUnowned);
  g.brokenFaces.clear();
  g.bondDamage.clear();
  g.ox = 0.0f;
  g.oy = 0.0f;
  g.oz = 0.0f;
}

uint32_t countSolidVoxels(const VoxelGrid& g) {
  uint32_t n = 0;
  for (uint8_t s : g.solid) {
    n += s != 0 ? 1u : 0u;
  }
  return n;
}

BlastError extractGraph(const VoxelGrid& grid, VoxelStructureGraph& graph, OwnerId ownerFilter) {
  const std::unordered_map<VoxelCoord, uint32_t, VoxelCoordHash> oldVoxelNode = graph.voxelNode;
  const std::vector<GraphBond> oldBonds = graph.bonds;
  uint32_t next = graph.nextStable;
  uint32_t nextBond = graph.nextBondId;
  if (next == 0) {
    next = 1;
  }
  if (nextBond == 0) {
    nextBond = 1;
  }

  const int G = std::max(1, grid.agg);
  const int cxn = (grid.nx + G - 1) / G;
  const int cyn = (grid.ny + G - 1) / G;
  const int czn = (grid.nz + G - 1) / G;
  std::vector<uint8_t> seen(static_cast<size_t>(grid.nx * grid.ny * grid.nz), 0);
  std::vector<PendingNode> pending;

  auto allowed = [&](int x, int y, int z, OwnerId seedOwner) -> bool {
    if (!grid.isSolid(x, y, z) || seen[static_cast<size_t>(grid.index(x, y, z))]) {
      return false;
    }
    const OwnerId o = grid.ownerAt(x, y, z);
    if (ownerFilter != kOwnerAll) {
      return o == ownerFilter;
    }
    return o == seedOwner;
  };

  for (int cz = 0; cz < czn; ++cz) {
    for (int cy = 0; cy < cyn; ++cy) {
      for (int cx = 0; cx < cxn; ++cx) {
        const int x0 = cx * G, y0 = cy * G, z0 = cz * G;
        const int x1 = std::min(x0 + G, grid.nx);
        const int y1 = std::min(y0 + G, grid.ny);
        const int z1 = std::min(z0 + G, grid.nz);
        for (int z = z0; z < z1; ++z) {
          for (int y = y0; y < y1; ++y) {
            for (int x = x0; x < x1; ++x) {
              if (!grid.isSolid(x, y, z) || seen[static_cast<size_t>(grid.index(x, y, z))]) {
                continue;
              }
              const OwnerId seedOwner = grid.ownerAt(x, y, z);
              if (ownerFilter != kOwnerAll && seedOwner != ownerFilter) {
                continue;
              }
              PendingNode node;
              node.owner = ownerFilter != kOwnerAll ? ownerFilter : seedOwner;
              node.minCoord = {x, y, z};
              std::queue<VoxelCoord> q;
              q.push({x, y, z});
              seen[static_cast<size_t>(grid.index(x, y, z))] = 1;
              while (!q.empty()) {
                const VoxelCoord p = q.front();
                q.pop();
                node.voxels.push_back(p);
                if (p < node.minCoord) {
                  node.minCoord = p;
                }
                const auto it = oldVoxelNode.find(p);
                if (it != oldVoxelNode.end() && it->second != 0) {
                  node.overlap[it->second]++;
                }
                for (int d = 0; d < 6; ++d) {
                  const int nx = p.x + kDx[d];
                  const int ny = p.y + kDy[d];
                  const int nz = p.z + kDz[d];
                  if (nx < x0 || ny < y0 || nz < z0 || nx >= x1 || ny >= y1 || nz >= z1) {
                    continue;
                  }
                  if (!allowed(nx, ny, nz, seedOwner)) {
                    continue;
                  }
                  if (!faceOpen(grid, p.x, p.y, p.z, nx, ny, nz)) {
                    continue;
                  }
                  seen[static_cast<size_t>(grid.index(nx, ny, nz))] = 1;
                  q.push({nx, ny, nz});
                }
              }
              pending.push_back(std::move(node));
            }
          }
        }
      }
    }
  }

  std::vector<uint32_t> assigned(pending.size(), 0);
  std::unordered_set<uint32_t> usedOld;
  std::vector<IdCandidate> cands;
  cands.reserve(pending.size() * 2);
  for (uint32_t i = 0; i < static_cast<uint32_t>(pending.size()); ++i) {
    for (const auto& v : pending[i].overlap) {
      IdCandidate c;
      c.overlap = v.second;
      c.minCoord = pending[i].minCoord;
      c.newIndex = i;
      c.oldId = v.first;
      cands.push_back(c);
    }
  }
  std::sort(cands.begin(), cands.end(), candidateLess);
  for (const IdCandidate& c : cands) {
    if (assigned[c.newIndex] != 0 || usedOld.count(c.oldId) != 0) {
      continue;
    }
    assigned[c.newIndex] = c.oldId;
    usedOld.insert(c.oldId);
  }

  VoxelStructureGraph built;
  built.nodes.reserve(pending.size());
  for (uint32_t i = 0; i < static_cast<uint32_t>(pending.size()); ++i) {
    GraphNode node;
    node.owner = pending[i].owner;
    node.voxels = std::move(pending[i].voxels);
    if (assigned[i] != 0) {
      node.stableId = assigned[i];
      bumpNextPast(next, node.stableId);
    } else if (!allocStableId(next, node.stableId)) {
      return BlastError::IdExhausted;
    }
    finishNodeGeometry(node, grid);
    for (const VoxelCoord& p : node.voxels) {
      built.voxelNode[p] = node.stableId;
    }
    built.nodes.push_back(std::move(node));
  }

  std::unordered_map<uint64_t, int> bondIndex;
  const float s = grid.voxelSize;
  for (int z = 0; z < grid.nz; ++z) {
    for (int y = 0; y < grid.ny; ++y) {
      for (int x = 0; x < grid.nx; ++x) {
        if (!grid.isSolid(x, y, z)) {
          continue;
        }
        const auto ia = built.voxelNode.find({x, y, z});
        if (ia == built.voxelNode.end()) {
          continue;
        }
        for (int d = 0; d < 6; ++d) {
          if (kDx[d] + kDy[d] + kDz[d] < 0) {
            continue;
          }
          const int nx = x + kDx[d];
          const int ny = y + kDy[d];
          const int nz = z + kDz[d];
          if (!grid.isSolid(nx, ny, nz)) {
            continue;
          }
          const auto ib = built.voxelNode.find({nx, ny, nz});
          if (ib == built.voxelNode.end() || ib->second == ia->second) {
            continue;
          }
          if (!faceOpen(grid, x, y, z, nx, ny, nz)) {
            continue;
          }
          const uint64_t key = packBondKey(ia->second, ib->second);
          auto it = bondIndex.find(key);
          if (it == bondIndex.end()) {
            GraphBond b;
            b.nodeA = ia->second < ib->second ? ia->second : ib->second;
            b.nodeB = ia->second < ib->second ? ib->second : ia->second;
            bondIndex[key] = static_cast<int>(built.bonds.size());
            built.bonds.push_back(b);
            it = bondIndex.find(key);
          }
          GraphBond& b = built.bonds[static_cast<size_t>(it->second)];
          b.nFaces += 1;
          b.faces.push_back(faceBetween(x, y, z, nx, ny, nz));
          b.cx += (static_cast<float>(x + nx) * 0.5f + 0.5f) * s + grid.ox;
          b.cy += (static_cast<float>(y + ny) * 0.5f + 0.5f) * s + grid.oy;
          b.cz += (static_cast<float>(z + nz) * 0.5f + 0.5f) * s + grid.oz;
          b.nx += static_cast<float>(kDx[d]);
          b.ny += static_cast<float>(kDy[d]);
          b.nz += static_cast<float>(kDz[d]);
        }
      }
    }
  }
  for (GraphBond& b : built.bonds) {
    finishBondGeometry(b);
  }

  const BlastError ids = assignBondIdentities(grid, oldBonds, nextBond, built.bonds);
  if (ids != BlastError::Ok) {
    return ids;
  }

  built.nextStable = next;
  built.nextBondId = nextBond;
  graph.nodes = std::move(built.nodes);
  graph.bonds = std::move(built.bonds);
  graph.voxelNode = std::move(built.voxelNode);
  graph.nextStable = built.nextStable;
  graph.nextBondId = built.nextBondId;
  return BlastError::Ok;
}

void breakBondFaces(VoxelGrid& grid, const GraphBond& bond) {
  for (uint64_t f : bond.faces) {
    grid.brokenFaces.insert(f);
  }
  grid.bondDamage[packBondKey(bond.nodeA, bond.nodeB)] = 1.0f;
}

float bondAgeo(const GraphBond& b, float voxelSize) { return static_cast<float>(b.nFaces) * voxelSize * voxelSize; }

float bondAeff(const GraphBond& b, float voxelSize) { return bondAgeo(b, voxelSize) * (1.0f - b.d); }

void destroyVoxelBlast(VoxelBlast& b) {
  if (b.solver) {
    b.solver->release();
    b.solver = nullptr;
  }
  b.actor = nullptr;
  b.family = nullptr;
  b.asset = nullptr;
  b.familyMem.reset();
  b.assetMem.reset();
  b.chunkFromStable.clear();
  b.sdkBondFromStable.clear();
  b.graphFromStable.clear();
  b.graphWorld = kInvalidIndex;
}

VoxelBlast::VoxelBlast(VoxelBlast&& o) noexcept { *this = std::move(o); }

VoxelBlast& VoxelBlast::operator=(VoxelBlast&& o) noexcept {
  if (this != &o) {
    destroyVoxelBlast(*this);
    assetMem = std::move(o.assetMem);
    familyMem = std::move(o.familyMem);
    asset = o.asset;
    family = o.family;
    actor = o.actor;
    solver = o.solver;
    chunkFromStable = std::move(o.chunkFromStable);
    sdkBondFromStable = std::move(o.sdkBondFromStable);
    graphFromStable = std::move(o.graphFromStable);
    graphWorld = o.graphWorld;
    o.asset = nullptr;
    o.family = nullptr;
    o.actor = nullptr;
    o.solver = nullptr;
    o.graphWorld = kInvalidIndex;
  }
  return *this;
}

VoxelBlast::~VoxelBlast() { destroyVoxelBlast(*this); }

BlastError createVoxelBlast(TrackingAllocator& alloc, const VoxelStructureGraph& graph, const VoxelGrid& grid,
                            VoxelBlast& out, float strengthPa, uint32_t solverIters) {
  destroyVoxelBlast(out);
  if (graph.nodes.empty()) {
    return BlastError::EmptyGraph;
  }
  const uint32_t nN = static_cast<uint32_t>(graph.nodes.size());
  const uint32_t nB = static_cast<uint32_t>(graph.bonds.size());
  std::vector<NvBlastChunkDesc> chunks(nN);
  for (uint32_t i = 0; i < nN; ++i) {
    const GraphNode& n = graph.nodes[i];
    chunks[i].centroid[0] = n.cx;
    chunks[i].centroid[1] = n.cy;
    chunks[i].centroid[2] = n.cz;
    chunks[i].volume = n.volume;
    chunks[i].parentChunkDescIndex = UINT32_MAX;
    chunks[i].flags = NvBlastChunkDesc::SupportFlag;
    chunks[i].userData = n.stableId;
  }
  std::vector<NvBlastBondDesc> bonds(nB);
  std::unordered_map<uint32_t, uint32_t> chunkOfStable;
  for (uint32_t i = 0; i < nN; ++i) {
    chunkOfStable[graph.nodes[i].stableId] = i;
  }
  for (uint32_t i = 0; i < nB; ++i) {
    const GraphBond& gb = graph.bonds[i];
    bonds[i].chunkIndices[0] = chunkOfStable[gb.nodeA];
    bonds[i].chunkIndices[1] = gb.world ? UINT32_MAX : chunkOfStable[gb.nodeB];
    bonds[i].bond.normal[0] = gb.nx;
    bonds[i].bond.normal[1] = gb.ny;
    bonds[i].bond.normal[2] = gb.nz;
    bonds[i].bond.area = gb.world ? bondAgeo(gb, grid.voxelSize) : bondAeff(gb, grid.voxelSize);
    bonds[i].bond.centroid[0] = gb.cx;
    bonds[i].bond.centroid[1] = gb.cy;
    bonds[i].bond.centroid[2] = gb.cz;
    bonds[i].bond.userData = gb.stableId;
  }
  std::vector<char> scratch(static_cast<size_t>(nN) * sizeof(NvBlastChunkDesc) + 4096);
  NvBlastEnsureAssetExactSupportCoverage(chunks.data(), nN, scratch.data(), blastLog);
  std::vector<uint32_t> reorder(nN);
  NvBlastReorderAssetDescChunks(chunks.data(), nN, bonds.data(), nB, reorder.data(), true, scratch.data(), blastLog);
  NvBlastAssetDesc desc{};
  desc.chunkCount = nN;
  desc.chunkDescs = chunks.data();
  desc.bondCount = nB;
  desc.bondDescs = bonds.data();
  const size_t need = NvBlastGetRequiredScratchForCreateAsset(&desc, blastLog);
  if (scratch.size() < need) {
    scratch.resize(need);
  }
  const size_t bytes = NvBlastGetAssetMemorySize(&desc, blastLog);
  out.assetMem = allocBlock(alloc, bytes, "NvBlastAsset");
  out.asset = NvBlastCreateAsset(out.assetMem.ptr, &desc, scratch.data(), blastLog);
  if (!out.asset) {
    destroyVoxelBlast(out);
    return BlastError::AssetCreateFailed;
  }
  const uint32_t builtN = NvBlastAssetGetChunkCount(out.asset, blastLog);
  const NvBlastChunk* ch = NvBlastAssetGetChunks(out.asset, blastLog);
  const uint32_t* c2g = NvBlastAssetGetChunkToGraphNodeMap(out.asset, blastLog);
  for (uint32_t i = 0; i < builtN; ++i) {
    out.chunkFromStable[ch[i].userData] = i;
    out.graphFromStable[ch[i].userData] = c2g[i];
  }
  const uint32_t builtB = NvBlastAssetGetBondCount(out.asset, blastLog);
  const NvBlastBond* ba = NvBlastAssetGetBonds(out.asset, blastLog);
  std::unordered_set<uint32_t> worldStable;
  for (const GraphBond& gb : graph.bonds) {
    if (gb.world) {
      worldStable.insert(gb.stableId);
    }
  }
  std::vector<float> health(builtB, 0.0f);
  for (uint32_t i = 0; i < builtB; ++i) {
    out.sdkBondFromStable[ba[i].userData] = i;
    health[i] = worldStable.count(ba[i].userData) != 0 ? Nv::Blast::kUnbreakableLimit * 2.0f : ba[i].area;
  }
  const size_t famBytes = NvBlastAssetGetFamilyMemorySize(out.asset, blastLog);
  out.familyMem = allocBlock(alloc, famBytes, "NvBlastFamily");
  out.family = NvBlastAssetCreateFamily(out.familyMem.ptr, out.asset, blastLog);
  if (!out.family) {
    destroyVoxelBlast(out);
    return BlastError::FamilyCreateFailed;
  }
  NvBlastActorDesc ad{};
  ad.initialBondHealths = health.data();
  ad.uniformInitialBondHealth = 1.0f;
  ad.uniformInitialLowerSupportChunkHealth = 1.0f;
  const size_t as = NvBlastFamilyGetRequiredScratchForCreateFirstActor(out.family, blastLog);
  if (scratch.size() < as) {
    scratch.resize(as);
  }
  out.actor = NvBlastFamilyCreateFirstActor(out.family, &ad, scratch.data(), blastLog);
  if (!out.actor) {
    destroyVoxelBlast(out);
    return BlastError::ActorCreateFailed;
  }
  Nv::Blast::ExtStressSolverSettings st;
  st.maxSolverIterationsPerFrame = solverIters == 0 ? 50u : solverIters;
  st.graphReductionLevel = 0;
  st.compressionElasticLimit = strengthPa;
  st.compressionFatalLimit = 2.0f * strengthPa;
  st.tensionElasticLimit = strengthPa;
  st.tensionFatalLimit = 2.0f * strengthPa;
  st.shearElasticLimit = strengthPa;
  st.shearFatalLimit = 2.0f * strengthPa;
  out.solver = Nv::Blast::ExtStressSolver::create(*out.family, st);
  if (!out.solver) {
    destroyVoxelBlast(out);
    return BlastError::SolverCreateFailed;
  }
  for (const GraphNode& n : graph.nodes) {
    const uint32_t g = out.graphFromStable[n.stableId];
    out.solver->setNodeInfo(g, n.mass, n.volume, NvcVec3{n.cx, n.cy, n.cz});
  }
  const NvBlastSupportGraph sg = NvBlastAssetGetSupportGraph(out.asset, blastLog);
  for (uint32_t i = 0; i < sg.nodeCount; ++i) {
    if (sg.chunkIndices[i] == UINT32_MAX) {
      out.graphWorld = i;
      out.solver->setNodeInfo(i, 0.0f, 0.0f, NvcVec3{0.0f, 0.0f, 0.0f});
    }
  }
  out.solver->notifyActorCreated(*out.actor);
  return BlastError::Ok;
}

BlastError stampOwnersFromActors(VoxelGrid& grid, const VoxelStructureGraph& graph, NvBlastFamily* family,
                                 const VoxelBlast& blastObj) {
  if (family == nullptr || blastObj.asset == nullptr) {
    return BlastError::OwnerConflict;
  }
  const uint32_t nA = NvBlastFamilyGetActorCount(family, blastLog);
  std::vector<NvBlastActor*> actors(nA, nullptr);
  NvBlastFamilyGetActors(actors.data(), nA, family, blastLog);
  std::unordered_map<uint32_t, OwnerId> nodeOwner;
  OwnerId next = 1;
  for (uint32_t i = 0; i < nA; ++i) {
    if (actors[i] == nullptr) {
      continue;
    }
    const uint32_t nv = NvBlastActorGetVisibleChunkCount(actors[i], blastLog);
    if (nv == 0) {
      continue;
    }
    if (next == 0) {
      return BlastError::OwnerCapacity;
    }
    std::vector<uint32_t> chunks(nv);
    NvBlastActorGetVisibleChunkIndices(chunks.data(), nv, actors[i], blastLog);
    const NvBlastChunk* ch = NvBlastAssetGetChunks(blastObj.asset, blastLog);
    const OwnerId own = next;
    if (next == (std::numeric_limits<OwnerId>::max)()) {
      next = 0;
    } else {
      ++next;
    }
    for (uint32_t c : chunks) {
      nodeOwner[ch[c].userData] = own;
    }
  }
  std::fill(grid.owner.begin(), grid.owner.end(), kUnowned);
  for (const GraphNode& n : graph.nodes) {
    const auto it = nodeOwner.find(n.stableId);
    if (it == nodeOwner.end()) {
      continue;
    }
    const OwnerId o = it->second;
    for (const VoxelCoord& p : n.voxels) {
      if (!grid.isSolid(p.x, p.y, p.z)) {
        continue;
      }
      const OwnerId prev = grid.ownerAt(p.x, p.y, p.z);
      if (prev != kUnowned && prev != o) {
        return BlastError::OwnerConflict;
      }
      grid.setOwner(p.x, p.y, p.z, o);
    }
  }
  for (int z = 0; z < grid.nz; ++z) {
    for (int y = 0; y < grid.ny; ++y) {
      for (int x = 0; x < grid.nx; ++x) {
        if (grid.isSolid(x, y, z) && grid.ownerAt(x, y, z) == kUnowned) {
          return BlastError::OwnerConflict;
        }
      }
    }
  }
  return BlastError::Ok;
}

RebuildResult compactReplace(TrackingAllocator& alloc, VoxelGrid& grid, VoxelStructureGraph& oldGraph,
                             VoxelBlast& oldBlast, std::vector<CompactFamily>& out, float strengthPa,
                             const CompactReplaceOpts& opts) {
  RebuildResult result;
  if (oldBlast.family == nullptr || oldBlast.asset == nullptr) {
    result.error = BlastError::FamilyCreateFailed;
    result.message = "old family missing";
    return result;
  }

  VoxelGrid working = grid;
  const BlastError stamped = stampOwnersFromActors(working, oldGraph, oldBlast.family, oldBlast);
  if (stamped != BlastError::Ok) {
    result.error = stamped;
    result.message = blastErrorMessage(stamped);
    return result;
  }

  std::vector<OwnerId> owners;
  {
    std::unordered_set<OwnerId> uniq;
    for (int z = 0; z < working.nz; ++z) {
      for (int y = 0; y < working.ny; ++y) {
        for (int x = 0; x < working.nx; ++x) {
          if (!working.isSolid(x, y, z)) {
            continue;
          }
          const OwnerId o = working.ownerAt(x, y, z);
          if (o != kUnowned) {
            uniq.insert(o);
          }
        }
      }
    }
    owners.assign(uniq.begin(), uniq.end());
    std::sort(owners.begin(), owners.end());
  }

  if (owners.empty()) {
    destroyVoxelBlast(oldBlast);
    grid = std::move(working);
    out.clear();
    result.error = BlastError::Ok;
    result.message = "empty structure";
    return result;
  }

  std::vector<CompactFamily> built;
  built.reserve(owners.size());
  uint32_t nextNode = oldGraph.nextStable;
  uint32_t nextBond = oldGraph.nextBondId;
  uint32_t created = 0;
  for (OwnerId o : owners) {
    if (created == opts.failAfterCreates) {
      result.error = BlastError::InjectedFailure;
      result.message = blastErrorMessage(BlastError::InjectedFailure);
      return result;
    }
    CompactFamily cf;
    cf.owner = o;
    // Full dense copy per owner. Sparse local rebuild is intentionally not this round.
    cf.grid = working;
    cf.graph = oldGraph;
    cf.graph.nextStable = nextNode;
    cf.graph.nextBondId = nextBond;
    const BlastError ge = extractGraph(cf.grid, cf.graph, o);
    if (ge != BlastError::Ok) {
      result.error = ge;
      result.message = blastErrorMessage(ge);
      return result;
    }
    if (cf.graph.nodes.empty()) {
      continue;
    }
    const BlastError ce = createVoxelBlast(alloc, cf.graph, cf.grid, cf.blast, strengthPa);
    if (ce != BlastError::Ok) {
      result.error = ce;
      result.message = blastErrorMessage(ce);
      return result;
    }
    const BlastError ve = opts.failMappingCheck ? BlastError::MappingInvalid : validateFamilyGraph(working, cf);
    if (ve != BlastError::Ok) {
      result.error = ve;
      result.message = blastErrorMessage(ve);
      return result;
    }
    nextNode = cf.graph.nextStable;
    nextBond = cf.graph.nextBondId;
    built.push_back(std::move(cf));
    ++created;
  }

  uint32_t graphVoxels = 0;
  for (const CompactFamily& cf : built) {
    for (const GraphNode& n : cf.graph.nodes) {
      graphVoxels += static_cast<uint32_t>(n.voxels.size());
    }
  }
  if (graphVoxels != countSolidVoxels(working)) {
    result.error = BlastError::OwnerConflict;
    result.message = "replacement voxel count does not match occupancy";
    return result;
  }

  grid = std::move(working);
  destroyVoxelBlast(oldBlast);
  out = std::move(built);
  result.error = BlastError::Ok;
  result.message = "ok";
  return result;
}

}  // namespace blast
