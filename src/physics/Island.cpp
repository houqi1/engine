#include "physics/Island.h"

namespace physics {

void IslandSet::clear() { islands_.clear(); }

Island& IslandSet::add(int shapeIndex) {
  Island is;
  is.id = static_cast<int>(islands_.size());
  is.shapeIndex = shapeIndex;
  is.asleep = true;
  is.needsSplit = false;
  is.sleepTimer = 0.0f;
  is.settleLeft = 0;
  is.brokenThisDt = 0;
  is.vmax = 0.0f;
  islands_.push_back(is);
  return islands_.back();
}

Island* IslandSet::byId(int id) {
  if (id < 0 || id >= static_cast<int>(islands_.size())) {
    return nullptr;
  }
  return &islands_[static_cast<size_t>(id)];
}

const Island* IslandSet::byId(int id) const {
  if (id < 0 || id >= static_cast<int>(islands_.size())) {
    return nullptr;
  }
  return &islands_[static_cast<size_t>(id)];
}

void IslandSet::wake(int islandId) {
  Island* is = byId(islandId);
  if (!is) {
    return;
  }
  is->asleep = false;
  is->sleepTimer = 0.0f;
}

int IslandSet::firstNeedsSplit() const {
  for (const Island& is : islands_) {
    if (is.needsSplit && !is.nodes.empty()) {
      return is.id;
    }
  }
  return -1;
}

}  // namespace physics
