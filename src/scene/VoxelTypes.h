#pragma once

#include <cstdint>

enum class MotionType : uint8_t {
  Static = 0,
  Dynamic = 1,
  Kinematic = 2,
};

struct VoxelObjectId {
  uint32_t slot = 0xFFFFFFFFu;
  uint32_t generation = 0;

  bool valid() const { return slot != 0xFFFFFFFFu && generation != 0; }
  bool operator==(const VoxelObjectId& o) const {
    return slot == o.slot && generation == o.generation;
  }
  bool operator!=(const VoxelObjectId& o) const { return !(*this == o); }
};
