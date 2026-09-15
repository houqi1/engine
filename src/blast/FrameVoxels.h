#pragma once

#include "blast/CylinderVoxels.h"

#include <cstdint>

namespace blast {

// Four corner columns + a roof slab. Fine 0.1 m, same 64^3 object as the cylinder demo.
constexpr int kFrameGridFines = 64;
constexpr int kFrameColW = 6;
constexpr int kFrameColH = 40;
constexpr int kFrameRoofT = 4;
constexpr int kFrameAnchorFines = 2;
constexpr int kFrameInset = 8;
constexpr int kFrameKeepCol = 3;
constexpr int kFrameAgg = 4;
constexpr float kFrameStrengthFailPa = 1.0e6f;
constexpr float kFrameStrengthHoldPa = 5.0e7f;

struct FrameRaster {
  int nx = kFrameGridFines;
  int ny = kFrameGridFines;
  int nz = kFrameGridFines;
  float voxelSize = kE1FineMeters;
  float ox = 0.0f;
  float oy = 0.0f;
  float oz = 0.0f;
};

inline void frameColumnRange(int col, int& x0, int& z0) {
  const int span = 32;
  const int ix = (col == 2 || col == 3) ? 1 : 0;
  const int iz = (col == 1 || col == 3) ? 1 : 0;
  x0 = kFrameInset + ix * span;
  z0 = kFrameInset + iz * span;
}

inline bool inFrameColumn(int col, int x, int y, int z) {
  if (y < 0 || y >= kFrameColH) {
    return false;
  }
  int x0 = 0;
  int z0 = 0;
  frameColumnRange(col, x0, z0);
  return x >= x0 && x < x0 + kFrameColW && z >= z0 && z < z0 + kFrameColW;
}

inline bool inAnyFrameColumn(int x, int y, int z) {
  for (int c = 0; c < 4; ++c) {
    if (inFrameColumn(c, x, y, z)) {
      return true;
    }
  }
  return false;
}

inline bool inFrameRoof(int x, int y, int z) {
  if (y < kFrameColH || y >= kFrameColH + kFrameRoofT) {
    return false;
  }
  const int x0 = kFrameInset;
  const int x1 = kFrameInset + 32 + kFrameColW;
  const int z0 = kFrameInset;
  const int z1 = kFrameInset + 32 + kFrameColW;
  const int t = kFrameColW;
  const bool xBeam = x >= x0 && x < x1 && ((z >= z0 && z < z0 + t) || (z >= z1 - t && z < z1));
  const bool zBeam = z >= z0 && z < z1 && ((x >= x0 && x < x0 + t) || (x >= x1 - t && x < x1));
  return xBeam || zBeam;
}

inline bool inFrameSolid(int x, int y, int z) { return inAnyFrameColumn(x, y, z) || inFrameRoof(x, y, z); }

inline bool isFrameAnchorFine(int x, int y, int z) {
  return y >= 0 && y < kFrameAnchorFines && inAnyFrameColumn(x, y, z);
}

inline bool inThreeColumnCut(int x, int y, int z) {
  if (y < kFrameAnchorFines) {
    return false;
  }
  for (int c = 0; c < 4; ++c) {
    if (c == kFrameKeepCol) {
      continue;
    }
    if (inFrameColumn(c, x, y, z)) {
      return true;
    }
  }
  return false;
}

inline void keepColumnWorldBox(const FrameRaster& r, float& x0, float& x1, float& z0, float& z1) {
  int cx = 0;
  int cz = 0;
  frameColumnRange(kFrameKeepCol, cx, cz);
  x0 = static_cast<float>(cx) * r.voxelSize + r.ox;
  x1 = static_cast<float>(cx + kFrameColW) * r.voxelSize + r.ox;
  z0 = static_cast<float>(cz) * r.voxelSize + r.oz;
  z1 = static_cast<float>(cz + kFrameColW) * r.voxelSize + r.oz;
}

template <typename Fn>
void forEachFrameFine(const FrameRaster& r, Fn&& fn) {
  for (int z = 0; z < r.nz; ++z) {
    for (int y = 0; y < r.ny; ++y) {
      for (int x = 0; x < r.nx; ++x) {
        if (inFrameSolid(x, y, z)) {
          fn(x, y, z);
        }
      }
    }
  }
}

}  // namespace blast
