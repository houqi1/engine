#pragma once

#include "blast/BlastIds.h"
#include "blast/BlastMemory.h"

#include "NvBlast.h"
#include "NvBlastExtDamageShaders.h"
#include "NvBlastExtStressSolver.h"

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace blast {

// Geometric owner of a voxel / graph node / compact family.
// 0 is reserved. Call sites decide which of the two meanings applies:
//   VoxelGrid::owner / GraphNode::owner / CompactFamily::owner → kUnowned (no actor).
//   extractGraph ownerFilter → kOwnerAll (visit every owner, including unowned).
using OwnerId = uint32_t;
constexpr OwnerId kUnowned = 0;
constexpr OwnerId kOwnerAll = 0;

enum class BlastError : uint32_t {
  Ok = 0,
  EmptyGraph,
  IdExhausted,
  AmbiguousDamage,
  OwnerConflict,
  OwnerCapacity,
  AssetCreateFailed,
  FamilyCreateFailed,
  ActorCreateFailed,
  SolverCreateFailed,
  MappingInvalid,
  MassMismatch,
  BrokenFaceReconnected,
  InjectedFailure,
  NotAnchored,
  DisconnectedFromAnchor,
  CavityBond,
};

inline bool ok(BlastError e) { return e == BlastError::Ok; }
const char* blastErrorMessage(BlastError e);

struct VoxelCoord {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator==(const VoxelCoord& o) const { return x == o.x && y == o.y && z == o.z; }
  bool operator<(const VoxelCoord& o) const {
    return x < o.x || (x == o.x && (y < o.y || (y == o.y && z < o.z)));
  }
};

struct VoxelCoordHash {
  size_t operator()(const VoxelCoord& p) const noexcept {
    const uint64_t h = static_cast<uint64_t>(static_cast<uint32_t>(p.x)) * 73856093ull ^
                       static_cast<uint64_t>(static_cast<uint32_t>(p.y)) * 19349663ull ^
                       static_cast<uint64_t>(static_cast<uint32_t>(p.z)) * 83492791ull;
    return static_cast<size_t>(h);
  }
};

inline uint64_t packFaceKey(int x, int y, int z, int axis) {
  return static_cast<uint64_t>(static_cast<uint32_t>(x)) |
         (static_cast<uint64_t>(static_cast<uint32_t>(y)) << 16) |
         (static_cast<uint64_t>(static_cast<uint32_t>(z)) << 32) |
         (static_cast<uint64_t>(axis & 3) << 48);
}

inline uint64_t packBondKey(uint32_t a, uint32_t b) {
  const uint32_t lo = a < b ? a : b;
  const uint32_t hi = a < b ? b : a;
  return (static_cast<uint64_t>(hi) << 32) | lo;
}

struct VoxelGrid {
  int nx = 0;
  int ny = 0;
  int nz = 0;
  int agg = 2;
  float voxelSize = 0.1f;
  float density = 1000.0f;
  float ox = 0.0f;
  float oy = 0.0f;
  float oz = 0.0f;
  std::vector<uint8_t> solid;
  std::vector<uint8_t> anchor;
  std::vector<OwnerId> owner;  // 0 = unowned
  std::unordered_set<uint64_t> brokenFaces;
  std::unordered_map<uint64_t, float> bondDamage;

  int index(int x, int y, int z) const { return x + nx * (y + ny * z); }
  bool inBounds(int x, int y, int z) const { return x >= 0 && y >= 0 && z >= 0 && x < nx && y < ny && z < nz; }
  bool isSolid(int x, int y, int z) const { return inBounds(x, y, z) && solid[static_cast<size_t>(index(x, y, z))] != 0; }
  void setSolid(int x, int y, int z, bool v) {
    if (inBounds(x, y, z)) {
      solid[static_cast<size_t>(index(x, y, z))] = v ? 1 : 0;
    }
  }
  OwnerId ownerAt(int x, int y, int z) const {
    return inBounds(x, y, z) ? owner[static_cast<size_t>(index(x, y, z))] : kUnowned;
  }
  void setOwner(int x, int y, int z, OwnerId o) {
    if (inBounds(x, y, z)) {
      owner[static_cast<size_t>(index(x, y, z))] = o;
    }
  }
  bool isAnchor(int x, int y, int z) const {
    return inBounds(x, y, z) && !anchor.empty() && anchor[static_cast<size_t>(index(x, y, z))] != 0;
  }
  void setAnchor(int x, int y, int z, bool v) {
    if (inBounds(x, y, z) && !anchor.empty()) {
      anchor[static_cast<size_t>(index(x, y, z))] = v ? 1 : 0;
    }
  }
};

void initGrid(VoxelGrid& g, int nx, int ny, int nz, int agg, float voxelSize);
uint32_t countSolidVoxels(const VoxelGrid& g);

struct GraphNode {
  uint32_t stableId = 0;
  OwnerId owner = kUnowned;
  std::vector<VoxelCoord> voxels;
  float cx = 0;
  float cy = 0;
  float cz = 0;
  float volume = 0;
  float mass = 0;
};

struct GraphBond {
  uint32_t stableId = 0;
  uint32_t nodeA = 0;
  uint32_t nodeB = 0;
  int nFaces = 0;
  float d = 0;
  float nx = 0;
  float ny = 1;
  float nz = 0;
  float cx = 0;
  float cy = 0;
  float cz = 0;
  bool world = false;
  std::vector<uint64_t> faces;
};

struct VoxelStructureGraph {
  std::vector<GraphNode> nodes;
  std::vector<GraphBond> bonds;
  std::unordered_map<VoxelCoord, uint32_t, VoxelCoordHash> voxelNode;
  uint32_t nextStable = 1;
  uint32_t nextBondId = 1;
};

// Rebuilds graph in `graph`. On failure, `graph` is left unchanged and a reason is returned.
// ownerFilter == kOwnerAll visits every owner; any other value keeps voxels of that owner only.
BlastError extractGraph(const VoxelGrid& grid, VoxelStructureGraph& graph, OwnerId ownerFilter);

void breakBondFaces(VoxelGrid& grid, const GraphBond& bond);
float bondAgeo(const GraphBond& b, float voxelSize);
float bondAeff(const GraphBond& b, float voxelSize);

struct VoxelBlast {
  AlignedBlock assetMem;
  AlignedBlock familyMem;
  NvBlastAsset* asset = nullptr;
  NvBlastFamily* family = nullptr;
  NvBlastActor* actor = nullptr;
  Nv::Blast::ExtStressSolver* solver = nullptr;
  // ExtImpactDamageManager ImpactSpread path needs this (type!=0 AABB tree).
  NvBlastExtDamageAccelerator* accelerator = nullptr;
  std::unordered_map<uint32_t, uint32_t> chunkFromStable;
  std::unordered_map<uint32_t, uint32_t> sdkBondFromStable;
  std::unordered_map<uint32_t, uint32_t> graphFromStable;
  uint32_t graphWorld = kInvalidIndex;

  VoxelBlast() = default;
  VoxelBlast(const VoxelBlast&) = delete;
  VoxelBlast& operator=(const VoxelBlast&) = delete;
  VoxelBlast(VoxelBlast&& o) noexcept;
  VoxelBlast& operator=(VoxelBlast&& o) noexcept;
  ~VoxelBlast();
};

void destroyVoxelBlast(VoxelBlast& b);
BlastError createVoxelBlast(TrackingAllocator& alloc, const VoxelStructureGraph& graph, const VoxelGrid& grid,
                            VoxelBlast& out, float strengthPa, uint32_t solverIters = 50);
BlastError stampOwnersFromActors(VoxelGrid& grid, const VoxelStructureGraph& graph, NvBlastFamily* family,
                                 const VoxelBlast& blast);

struct CompactFamily {
  VoxelGrid grid;
  VoxelStructureGraph graph;
  VoxelBlast blast;
  OwnerId owner = 1;
};

struct CompactReplaceOpts {
  // Test-only: fail after this many successful replacement creates. kInvalidIndex = off.
  uint32_t failAfterCreates = kInvalidIndex;
  bool failMappingCheck = false;
};

struct RebuildResult {
  BlastError error = BlastError::Ok;
  const char* message = "ok";
  explicit operator bool() const { return error == BlastError::Ok; }
};

// Prepare replacements without touching the caller's grid / graph / blast / out.
// Commit is a swap of complete state. An empty structure (all voxels deleted) is success.
// Each owner currently copies the full dense grid; sparse local rebuild is not this round.
RebuildResult compactReplace(TrackingAllocator& alloc, VoxelGrid& grid, VoxelStructureGraph& oldGraph,
                             VoxelBlast& oldBlast, std::vector<CompactFamily>& out, float strengthPa,
                             const CompactReplaceOpts& opts = {});

}  // namespace blast
