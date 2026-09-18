#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <algorithm>
#include <cstdint>
#include <vector>

namespace physics {

constexpr float kDt = 1.0f / 60.0f;
constexpr int kMaxStepsPerFrame = 2;
constexpr int kSubsteps = 6;
constexpr float kSubDt = kDt / static_cast<float>(kSubsteps);
constexpr glm::vec3 kGravity{0.0f, -9.81f, 0.0f};

// Accumulator used by PhysicsWorld::step. Tests must drive this, not a raw tick loop.
struct FixedStepClock {
  float accumulator = 0.0f;
  uint64_t tickId = 0;

  template <typename Fn>
  int advance(float frameDt, Fn&& onFixedTick) {
    accumulator = (std::min)(accumulator + (std::max)(frameDt, 0.0f),
                             kDt * static_cast<float>(kMaxStepsPerFrame));
    int steps = 0;
    while (accumulator >= kDt && steps < kMaxStepsPerFrame) {
      accumulator -= kDt;
      ++tickId;
      onFixedTick(tickId, kDt);
      ++steps;
    }
    return steps;
  }

  void resetSession() {
    accumulator = 0.0f;
    tickId = 0;
  }
};

constexpr float kVoxel = 0.1f;
constexpr float kSphereRadius = 0.5f * kVoxel;
// Penetration tolerance: how deep before Baumgarte position correction. Not hover height.
constexpr float kSlop = 0.01f;
// Speculative contact range (d >= -lookAhead). Matches 6-neighbor fine probe reach.
constexpr float kContactLookAhead = kVoxel;
constexpr float kBaumgarte = 0.2f;
constexpr float kRestitution = 0.0f;
constexpr float kFriction = 0.5f;
// One TGS substep still needs several SI passes so 4 simultaneous
// corners can share the impact (Catto). Collision is still rebuilt each hSub.
constexpr int kContactIters = 8;

constexpr float kSleepLin = 0.05f;
constexpr float kSleepAng = 0.2f;
constexpr float kSleepTime = 0.5f;
// Box2D v3: position correction folded into sleep velocity (Catto solver.c).
constexpr float kSleepPositionFactor = 0.5f;

constexpr float kDensityWood = 600.0f;

enum class FineClass : uint8_t { Empty = 0, Inside, Face, Edge, Corner };

struct Contact {
  int a = 0;
  int b = 0;
  glm::vec3 p{0.0f};
  glm::vec3 n{0.0f, 1.0f, 0.0f};
  glm::vec3 rA{0.0f};
  glm::vec3 rB{0.0f};
  float d = 0.0f;
  int nFace = 2;
  uint32_t fineA = 0;
  uint32_t fineB = 0;
  float lambdaN = 0.0f;
  float lambdaT = 0.0f;
  // Collision-only normal impulse on A (excludes Baumgarte position correction).
  float lambdaNVel = 0.0f;
  // Accumulated world-space friction impulse on A (sum of dLamT * t each iteration).
  glm::vec3 JtWorld{0.0f};
  // Contact-point velocities before solveContacts. ExtImpactDamageManager builds
  // Viewer force from getVelocityAtPos; with kRestitution=0, post-solve Δv~0, so
  // Impact Damage must see this pre-solve approach velocity.
  glm::vec3 preVelA{0.0f};
  glm::vec3 preVelB{0.0f};
};

inline bool contactIsTouching(const Contact& c) { return c.d >= -kSlop; }

struct SleepSupport {
  int staticSlot = -1;
  glm::vec3 worldP{0.0f};
};

struct ShapeClass {
  int fineN = 0;
  std::vector<uint32_t> corners;
  std::vector<uint32_t> edges;
  glm::ivec3 occFineMn{0};
  glm::ivec3 occFineMx{-1};
  bool occValid = false;
  bool dirty = true;
};

struct DebugSolve {
  int contacts = 0;
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  float maxD = 0.0f;
  float minNy = 1.0f;
  std::vector<Contact> lastContacts;
  // Last physics tick only. Structure callback is timed separately from collide/solve.
  float rebuildMs = 0.0f;
  float collideMs = 0.0f;
  float contactSolveMs = 0.0f;
  float integrateMs = 0.0f;
  float structureCallbackMs = 0.0f;
  int awakeBodies = 0;
  int occupiedBodies = 0;
  int physicsTicksThisFrame = 0;
};

inline uint32_t packFine(int x, int y, int z, int n) {
  return static_cast<uint32_t>(x) + static_cast<uint32_t>(y) * static_cast<uint32_t>(n) +
         static_cast<uint32_t>(z) * static_cast<uint32_t>(n) * static_cast<uint32_t>(n);
}

inline glm::ivec3 unpackFine(uint32_t i, int n) {
  const int nn = n * n;
  const int z = static_cast<int>(i) / nn;
  const int y = (static_cast<int>(i) - z * nn) / n;
  const int x = static_cast<int>(i) - z * nn - y * n;
  return glm::ivec3(x, y, z);
}

}  // namespace physics
