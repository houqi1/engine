#include "physics/Island.h"

#include <algorithm>

namespace physics {

void IslandSet::clear() { islands_.clear(); }

Island& IslandSet::ensureForShape(int shapeIndex) {
  for (Island& is : islands_) {
    if (is.shapeIndex == shapeIndex) {
      return is;
    }
  }
  Island is;
  is.id = static_cast<int>(islands_.size());
  is.shapeIndex = shapeIndex;
  is.asleep = false;
  is.needsSplit = false;
  is.sleepTimer = 0.0f;
  is.settleLeft = 12;
  islands_.push_back(is);
  return islands_.back();
}

Island* IslandSet::findByShape(int shapeIndex) {
  for (Island& is : islands_) {
    if (is.shapeIndex == shapeIndex) {
      return &is;
    }
  }
  return nullptr;
}

const Island* IslandSet::findByShape(int shapeIndex) const {
  for (const Island& is : islands_) {
    if (is.shapeIndex == shapeIndex) {
      return &is;
    }
  }
  return nullptr;
}

void IslandSet::wakeShape(int shapeIndex) {
  Island* is = findByShape(shapeIndex);
  if (!is) {
    return;
  }
  is->asleep = false;
  is->sleepTimer = 0.0f;
  is->settleLeft = std::max(is->settleLeft, 6);
}

bool IslandSet::consumeSplit(int shapeIndex) {
  Island* is = findByShape(shapeIndex);
  if (!is || !is->needsSplit) {
    return false;
  }
  is->needsSplit = false;
  return true;
}

}  // namespace physics
