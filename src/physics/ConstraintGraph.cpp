#include "physics/ConstraintGraph.h"

#include <algorithm>

namespace physics {

void ConstraintGraph::clear() {
  usedMask_.clear();
  for (int i = 0; i < kGraphBucketCount; ++i) {
    buckets_[static_cast<size_t>(i)].clear();
  }
}

void ConstraintGraph::beginAssign(int nodeCount) {
  clear();
  usedMask_.assign(static_cast<size_t>(std::max(nodeCount, 0)), 0u);
}

int ConstraintGraph::assignBond(int nodeA, int nodeB, bool aAnchored, bool bAnchored) {
  const bool staticDynamic = aAnchored != bAnchored;
  const int start = staticDynamic ? 0 : kDynamicColorStart;
  auto tryColor = [&](int color) -> bool {
    const uint32_t bit = 1u << color;
    bool freeA = aAnchored || nodeA < 0 || nodeA >= static_cast<int>(usedMask_.size()) ||
                 (usedMask_[static_cast<size_t>(nodeA)] & bit) == 0u;
    bool freeB = bAnchored || nodeB < 0 || nodeB >= static_cast<int>(usedMask_.size()) ||
                 (usedMask_[static_cast<size_t>(nodeB)] & bit) == 0u;
    if (!freeA || !freeB) {
      return false;
    }
    if (!aAnchored && nodeA >= 0 && nodeA < static_cast<int>(usedMask_.size())) {
      usedMask_[static_cast<size_t>(nodeA)] |= bit;
    }
    if (!bAnchored && nodeB >= 0 && nodeB < static_cast<int>(usedMask_.size())) {
      usedMask_[static_cast<size_t>(nodeB)] |= bit;
    }
    return true;
  };

  for (int c = start; c < kGraphColorCount; ++c) {
    if (tryColor(c)) {
      return c;
    }
  }
  for (int c = 0; c < start; ++c) {
    if (tryColor(c)) {
      return c;
    }
  }
  return kOverflowColor;
}

}  // namespace physics
