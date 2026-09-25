#pragma once

#include "blast/OccupancySampler.h"

#include <functional>
#include <utility>

namespace blast {

// One voxel object's whole fine grid in object-local fine coordinates (origin 0).
// Solid comes from the object's occupancy; anchors come from an explicit
// object-local predicate and are only reported on solid fines.
class ObjectOccupancyView final : public OccupancyView {
public:
  using FinePredicate = std::function<bool(int x, int y, int z)>;

  ObjectOccupancyView(int fineN, float fineSize, float density, FinePredicate solid, FinePredicate anchor)
      : fineN_(fineN), fineSize_(fineSize), density_(density), solid_(std::move(solid)),
        anchor_(std::move(anchor)) {}

  int nx() const override { return fineN_; }
  int ny() const override { return fineN_; }
  int nz() const override { return fineN_; }
  float voxelSize() const override { return fineSize_; }
  float density() const override { return density_; }
  bool solid(int x, int y, int z) const override { return solid_(x, y, z); }
  bool anchor(int x, int y, int z) const override { return anchor_ && solid(x, y, z) && anchor_(x, y, z); }

private:
  int fineN_;
  float fineSize_;
  float density_;
  FinePredicate solid_;
  FinePredicate anchor_;
};

}  // namespace blast
