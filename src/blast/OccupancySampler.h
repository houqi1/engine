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
BlastError validateStructureGraph(const VoxelGrid& grid, const VoxelStructureGraph& graph);
bool nodeReachesAnchor(const VoxelStructureGraph& graph, uint32_t startId);

}  // namespace blast
