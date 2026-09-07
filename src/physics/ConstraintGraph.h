#pragma once

#include <cstdint>
#include <vector>

namespace physics {

// Box3D-style graph coloring. Same color => no shared dynamic node.
constexpr int kGraphColorCount = 12;
constexpr int kOverflowColor = kGraphColorCount;  // 13th serial bucket
constexpr int kGraphBucketCount = kGraphColorCount + 1;
constexpr int kDynamicColorStart = 4;  // 0..3 prefer static-dynamic (anchor) bonds

class ConstraintGraph {
public:
  void clear();
  void beginAssign(int nodeCount);
  // Returns color in [0, kOverflowColor]. Static-dynamic searches low colors first.
  int assignBond(int nodeA, int nodeB, bool aAnchored, bool bAnchored);
  const std::vector<int>& bucket(int color) const { return buckets_[static_cast<size_t>(color)]; }
  std::vector<int>& bucket(int color) { return buckets_[static_cast<size_t>(color)]; }

private:
  std::vector<uint32_t> usedMask_;
  std::vector<int> buckets_[kGraphBucketCount];
};

}  // namespace physics
