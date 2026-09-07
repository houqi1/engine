#pragma once

#include <vector>

namespace physics {

// Connected component of unanchored BondNodes joined by live bonds.
// Anchored nodes are static (Box3D): they support islands but do not merge
// two islands into one. Sleep/wake is per island, not per node.
struct Island {
  int id = -1;
  int shapeIndex = -1;
  std::vector<int> nodes;
  std::vector<int> bonds;
  bool asleep = true;
  bool needsSplit = false;
  float sleepTimer = 0.0f;
  int settleLeft = 0;
  int brokenThisDt = 0;
  float vmax = 0.0f;
};

class IslandSet {
public:
  void clear();
  Island& add(int shapeIndex);
  Island* byId(int id);
  const Island* byId(int id) const;
  std::vector<Island>& all() { return islands_; }
  const std::vector<Island>& all() const { return islands_; }
  void wake(int islandId);
  int firstNeedsSplit() const;

private:
  std::vector<Island> islands_;
};

}  // namespace physics
