#pragma once

#include "blast/VoxelGraph.h"

#include <cstdint>

namespace blast {

class OccupancyView {
public:
  virtual ~OccupancyView() = default;
  virtual int nx() const = 0;
  virtual int ny() const = 0;
  virtual int nz() const = 0;
  virtual bool solid(int x, int y, int z) const = 0;
  virtual bool anchor(int x, int y, int z) const = 0;
  virtual float voxelSize() const { return 0.1f; }
  virtual float density() const { return 1000.0f; }
  virtual float originX() const { return 0.0f; }
  virtual float originY() const { return 0.0f; }
  virtual float originZ() const { return 0.0f; }
};

struct OccupancySampleOpts {
  int agg = 2;
  // E5.3: accept graphs without world bonds (free bodies) and nodes that cannot reach
  // an anchor (floating islands, split off right after mounting).
  bool allowFloating = false;
};

struct OccupancySample {
  BlastError error = BlastError::Ok;
  const char* message = "ok";
  VoxelGrid grid;
  VoxelStructureGraph graph;
  uint32_t occupied = 0;
  uint32_t worldBonds = 0;
  float mass = 0.0f;
};

OccupancySample sampleOccupancy(const OccupancyView& view, const OccupancySampleOpts& opts);
BlastError attachWorldBonds(const VoxelGrid& grid, VoxelStructureGraph& graph);
// requireAnchored: at least one world bond and every node reaches one.
BlastError validateStructureGraph(const VoxelGrid& grid, const VoxelStructureGraph& graph,
                                  bool requireAnchored = true);
bool nodeReachesAnchor(const VoxelStructureGraph& graph, uint32_t startId);

}  // namespace blast
