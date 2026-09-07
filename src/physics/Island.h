#pragma once

#include <vector>

namespace physics {

struct Island {
  int id = -1;
  int shapeIndex = -1;
  std::vector<int> nodes;
  bool asleep = false;
  bool needsSplit = false;
  float sleepTimer = 0.0f;
  int settleLeft = 0;
};

// One island per static shape that has a bond graph. Box3D: merge fast, split
// at most one island per dt, sleep the whole island.
class IslandSet {
public:
  void clear();
  Island& ensureForShape(int shapeIndex);
  Island* findByShape(int shapeIndex);
  const Island* findByShape(int shapeIndex) const;
  std::vector<Island>& all() { return islands_; }
  const std::vector<Island>& all() const { return islands_; }
  void wakeShape(int shapeIndex);
  // Returns true if a split was consumed this call.
  bool consumeSplit(int shapeIndex);

private:
  std::vector<Island> islands_;
};

}  // namespace physics
