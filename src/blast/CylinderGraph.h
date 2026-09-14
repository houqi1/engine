#pragma once

#include "blast/BlastIds.h"
#include "blast/BlastMemory.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"

#include <cstdint>
#include <vector>

namespace blast {

constexpr int kCylinderCirc = 16;
constexpr int kCylinderLayers = 8;
constexpr int kCylinderNodeCount = kCylinderCirc * kCylinderLayers;

constexpr float kCylinderR = 2.0f;
constexpr float kCylinderT = 0.2f;
constexpr float kCylinderH = 6.0f;
constexpr float kCylinderRho = 1000.0f;
constexpr float kGravityY = -9.81f;

// Locked P1 strengths (Pa). Hold >> estimated remaining-strip axial ~0.3 MPa.
constexpr float kStrengthHoldPa = 5.0e7f;
constexpr float kStrengthFailPa = 1.0e3f;

inline uint32_t cylinderNodeStable(int k, int j) {
  return 10000u + static_cast<uint32_t>(j) * static_cast<uint32_t>(kCylinderCirc) + static_cast<uint32_t>(k);
}
inline uint32_t cylinderVertBondStable(int k, int j) {
  return 20000u + static_cast<uint32_t>(j) * static_cast<uint32_t>(kCylinderCirc) + static_cast<uint32_t>(k);
}
inline uint32_t cylinderCircBondStable(int k, int j) {
  return 30000u + static_cast<uint32_t>(j) * static_cast<uint32_t>(kCylinderCirc) + static_cast<uint32_t>(k);
}
inline uint32_t cylinderWorldBondStable(int k) { return 40000u + static_cast<uint32_t>(k); }

inline float cylinderArcLength() { return 6.28318530718f * kCylinderR / static_cast<float>(kCylinderCirc); }
inline float cylinderLayerHeight() { return kCylinderH / static_cast<float>(kCylinderLayers); }
inline float cylinderVertArea() { return cylinderArcLength() * kCylinderT; }
inline float cylinderCircArea() { return cylinderLayerHeight() * kCylinderT; }
inline float cylinderNodeVolume() { return cylinderArcLength() * cylinderLayerHeight() * kCylinderT; }
inline float cylinderNodeMass() { return kCylinderRho * cylinderNodeVolume(); }
inline float cylinderTotalMass() { return cylinderNodeMass() * static_cast<float>(kCylinderNodeCount); }

struct CylinderBondKind {
  enum Enum { Vertical = 0, Circumferential = 1, World = 2 };
};

struct CylinderIdMaps {
  uint32_t chunkFromNode[kCylinderNodeCount]{};
  uint32_t graphFromNode[kCylinderNodeCount]{};
  uint32_t graphWorld = kInvalidIndex;
  uint32_t sdkBondFromVert[kCylinderCirc * (kCylinderLayers - 1)]{};
  uint32_t sdkBondFromCirc[kCylinderCirc * kCylinderLayers]{};
  uint32_t sdkBondFromWorld[kCylinderCirc]{};
};

struct CylinderScene {
  CylinderIdMaps ids;
  AlignedBlock assetMem;
  AlignedBlock familyMem;
  NvBlastAsset* asset = nullptr;
  NvBlastFamily* family = nullptr;
  NvBlastActor* actor = nullptr;
  Nv::Blast::ExtStressSolver* solver = nullptr;
};

struct CylinderBuildOpts {
  bool worldAnchor = true;
  float density = kCylinderRho;
  float strengthPa = kStrengthHoldPa;
  uint32_t solverIters = 25;
};

bool buildCylinderScene(TrackingAllocator& alloc, CylinderScene& scene, const CylinderBuildOpts& opts);
void destroyCylinderScene(CylinderScene& scene);
void setCylinderNodeMasses(CylinderScene& scene, float density);
bool solveGravity(CylinderScene& scene, int extraIters = 0);

int nodeIndex(int k, int j);
void nodeKJ(int node, int& k, int& j);

}  // namespace blast
