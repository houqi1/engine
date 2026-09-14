#include "blast/OccupancySampler.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace blast {
namespace {

bool allocBondId(uint32_t& next, uint32_t& out) {
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

}  // namespace

BlastError attachWorldBonds(const VoxelGrid& grid, VoxelStructureGraph& graph) {
  std::unordered_set<uint32_t> used;
  used.reserve(graph.bonds.size() * 2);
  for (const GraphBond& b : graph.bonds) {
    used.insert(b.stableId);
  }
  uint32_t next = graph.nextBondId == 0 ? 1 : graph.nextBondId;
  uint32_t added = 0;
  for (const GraphNode& n : graph.nodes) {
    int nAnc = 0;
    double sx = 0, sy = 0, sz = 0;
    for (const VoxelCoord& p : n.voxels) {
      if (!grid.isAnchor(p.x, p.y, p.z)) {
        continue;
      }
      ++nAnc;
      sx += (static_cast<double>(p.x) + 0.5) * grid.voxelSize + grid.ox;
      sy += (static_cast<double>(p.y) + 0.5) * grid.voxelSize + grid.oy;
      sz += (static_cast<double>(p.z) + 0.5) * grid.voxelSize + grid.oz;
    }
    if (nAnc == 0) {
      continue;
    }
    GraphBond b;
    b.world = true;
    b.nodeA = n.stableId;
    b.nodeB = 0;
    b.nFaces = nAnc;
    b.nx = 0.0f;
    b.ny = 1.0f;
    b.nz = 0.0f;
    b.cx = static_cast<float>(sx / nAnc);
    b.cy = static_cast<float>(sy / nAnc);
    b.cz = static_cast<float>(sz / nAnc);
    if (!allocBondId(next, b.stableId) || used.count(b.stableId) != 0) {
      return BlastError::IdExhausted;
    }
    used.insert(b.stableId);
    graph.bonds.push_back(b);
    ++added;
  }
  graph.nextBondId = next;
  if (added == 0) {
    return BlastError::NotAnchored;
  }
  return BlastError::Ok;
}

bool nodeReachesAnchor(const VoxelStructureGraph& graph, uint32_t startId) {
  std::unordered_set<uint32_t> anchored;
  std::unordered_map<uint32_t, std::vector<uint32_t>> adj;
  for (const GraphBond& b : graph.bonds) {
    if (b.world) {
      anchored.insert(b.nodeA);
      continue;
    }
    adj[b.nodeA].push_back(b.nodeB);
    adj[b.nodeB].push_back(b.nodeA);
  }
  if (anchored.count(startId) != 0) {
    return true;
  }
  std::queue<uint32_t> q;
  std::unordered_set<uint32_t> seen;
  q.push(startId);
  seen.insert(startId);
  while (!q.empty()) {
    const uint32_t u = q.front();
    q.pop();
    if (anchored.count(u) != 0) {
      return true;
    }
    const auto it = adj.find(u);
    if (it == adj.end()) {
      continue;
    }
    for (uint32_t v : it->second) {
      if (seen.insert(v).second) {
        q.push(v);
      }
    }
  }
  return false;
}

BlastError validateStructureGraph(const VoxelGrid& grid, const VoxelStructureGraph& graph) {
  if (graph.nodes.empty()) {
    return BlastError::EmptyGraph;
  }
  std::unordered_set<uint32_t> nodeIds;
  std::unordered_set<VoxelCoord, VoxelCoordHash> seenVox;
  uint32_t covered = 0;
  float mass = 0.0f;
  for (const GraphNode& n : graph.nodes) {
    if (n.stableId == 0 || !nodeIds.insert(n.stableId).second) {
      return BlastError::MappingInvalid;
    }
    for (const VoxelCoord& p : n.voxels) {
      if (!grid.isSolid(p.x, p.y, p.z) || !seenVox.insert(p).second) {
        return BlastError::OwnerConflict;
      }
      ++covered;
    }
    mass += n.mass;
  }
  const uint32_t solids = countSolidVoxels(grid);
  if (covered != solids) {
    return BlastError::OwnerConflict;
  }
  const float voxVol = grid.voxelSize * grid.voxelSize * grid.voxelSize;
  const float expectMass = grid.density * voxVol * static_cast<float>(solids);
  if (expectMass > 0.0f && std::abs(mass - expectMass) > 1.0e-4f * expectMass) {
    return BlastError::MassMismatch;
  }
  std::unordered_set<uint32_t> bondIds;
  uint32_t worldBonds = 0;
  for (const GraphBond& b : graph.bonds) {
    if (b.stableId == 0 || !bondIds.insert(b.stableId).second) {
      return BlastError::MappingInvalid;
    }
    if (b.world) {
      ++worldBonds;
      continue;
    }
    for (uint64_t f : b.faces) {
      if (grid.brokenFaces.count(f) != 0) {
        return BlastError::BrokenFaceReconnected;
      }
    }
  }
  if (worldBonds == 0) {
    return BlastError::NotAnchored;
  }
  for (const GraphNode& n : graph.nodes) {
    if (!nodeReachesAnchor(graph, n.stableId)) {
      return BlastError::DisconnectedFromAnchor;
    }
  }
  return BlastError::Ok;
}

OccupancySample sampleOccupancy(const OccupancyView& view, const OccupancySampleOpts& opts) {
  OccupancySample out;
  initGrid(out.grid, view.nx(), view.ny(), view.nz(), opts.agg, view.voxelSize());
  out.grid.density = view.density();
  out.grid.ox = view.originX();
  out.grid.oy = view.originY();
  out.grid.oz = view.originZ();
  for (int z = 0; z < view.nz(); ++z) {
    for (int y = 0; y < view.ny(); ++y) {
      for (int x = 0; x < view.nx(); ++x) {
        if (!view.solid(x, y, z)) {
          continue;
        }
        out.grid.setSolid(x, y, z, true);
        if (view.anchor(x, y, z)) {
          out.grid.setAnchor(x, y, z, true);
        }
        ++out.occupied;
      }
    }
  }
  if (out.occupied == 0) {
    out.error = BlastError::EmptyGraph;
    out.message = blastErrorMessage(out.error);
    return out;
  }
  out.error = extractGraph(out.grid, out.graph, kOwnerAll);
  if (out.error != BlastError::Ok) {
    out.message = blastErrorMessage(out.error);
    return out;
  }
  out.error = attachWorldBonds(out.grid, out.graph);
  if (out.error != BlastError::Ok) {
    out.message = blastErrorMessage(out.error);
    return out;
  }
  out.error = validateStructureGraph(out.grid, out.graph);
  out.message = blastErrorMessage(out.error);
  out.worldBonds = 0;
  for (const GraphBond& b : out.graph.bonds) {
    if (b.world) {
      ++out.worldBonds;
    }
  }
  for (const GraphNode& n : out.graph.nodes) {
    out.mass += n.mass;
  }
  return out;
}

}  // namespace blast
