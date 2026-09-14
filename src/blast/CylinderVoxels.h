#pragma once

#include <cmath>
#include <cstdint>

namespace blast {

constexpr float kE1Radius = 2.0f;
constexpr float kE1Thickness = 0.2f;
constexpr float kE1Height = 6.0f;
constexpr float kE1FineMeters = 0.1f;
constexpr int kE1Agg = 2;
constexpr float kE1Density = 1000.0f;
constexpr float kE1GravityY = -9.81f;
constexpr int kE1FineHeight = 60;
constexpr int kE1GridFines = 64;
constexpr int kE1AxisFine = 32;
constexpr int kE1AnchorFines = 2;
// One P1 layer (~0.8 m) above the 0.2 m anchor ring. Two fines (0.2 m) sat on the
// ground and was not visible; still far from a full-height slot.
constexpr int kE1CutFines = 8;
constexpr float kE1KeepAz0 = 0.0f;
constexpr float kE1KeepAz1 = 1.5707963267948966f;
constexpr float kE1StressColorMaxPa = 2.0e6f;
constexpr float kE1ReactionRelTol = 1.0e-3f;
constexpr float kE1StrengthHoldPa = 5.0e7f;
// E2: intact strip ~0.12 MPa, cut strip ~2.7 MPa (E1). Hold is the P1 high-S control.
constexpr float kE2StrengthFailPa = 2.5e5f;
constexpr float kE2StrengthHoldPa = 5.0e7f;

struct CylinderRaster {
  int nx = kE1GridFines;
  int ny = kE1GridFines;
  int nz = kE1GridFines;
  int axisX = kE1AxisFine;
  int axisZ = kE1AxisFine;
  int y0 = 0;
  int y1 = kE1FineHeight;
  float voxelSize = kE1FineMeters;
  float R = kE1Radius;
  float t = kE1Thickness;
  float ox = 0.0f;
  float oy = 0.0f;
  float oz = 0.0f;
};

inline float cylinderAxisX(const CylinderRaster& r) {
  return (static_cast<float>(r.axisX) + 0.5f) * r.voxelSize + r.ox;
}
inline float cylinderAxisZ(const CylinderRaster& r) {
  return (static_cast<float>(r.axisZ) + 0.5f) * r.voxelSize + r.oz;
}

inline float voxelCenter(int i, float voxelSize, float origin) {
  return (static_cast<float>(i) + 0.5f) * voxelSize + origin;
}

inline float voxelAzimuth(float x, float z, const CylinderRaster& r) {
  return std::atan2(z - cylinderAxisZ(r), x - cylinderAxisX(r));
}

inline bool inCylinderWall(int x, int y, int z, const CylinderRaster& r) {
  if (y < r.y0 || y >= r.y1) {
    return false;
  }
  const float px = voxelCenter(x, r.voxelSize, r.ox);
  const float pz = voxelCenter(z, r.voxelSize, r.oz);
  const float dx = px - cylinderAxisX(r);
  const float dz = pz - cylinderAxisZ(r);
  const float rad = std::sqrt(dx * dx + dz * dz);
  const float inner = r.R - 0.5f * r.t;
  const float outer = r.R + 0.5f * r.t;
  return rad >= inner && rad < outer;
}

inline bool inKeepStrip(int x, int z, const CylinderRaster& r) {
  const float az = voxelAzimuth(voxelCenter(x, r.voxelSize, r.ox), voxelCenter(z, r.voxelSize, r.oz), r);
  return az >= kE1KeepAz0 && az < kE1KeepAz1;
}

inline bool inBaseCut270(int x, int y, int z, const CylinderRaster& r) {
  if (!inCylinderWall(x, y, z, r)) {
    return false;
  }
  if (y < r.y0 + kE1AnchorFines || y >= r.y0 + kE1AnchorFines + kE1CutFines) {
    return false;
  }
  return !inKeepStrip(x, z, r);
}

inline bool isBaseAnchorFine(int x, int y, int z, const CylinderRaster& r) {
  return inCylinderWall(x, y, z, r) && y >= r.y0 && y < r.y0 + kE1AnchorFines;
}

template <typename Fn>
void forEachCylinderFine(const CylinderRaster& r, Fn&& fn) {
  for (int z = 0; z < r.nz; ++z) {
    for (int y = r.y0; y < r.y1; ++y) {
      for (int x = 0; x < r.nx; ++x) {
        if (inCylinderWall(x, y, z, r)) {
          fn(x, y, z);
        }
      }
    }
  }
}

}  // namespace blast
