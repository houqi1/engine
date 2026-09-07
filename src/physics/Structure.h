#pragma once

#include "physics/ConstraintGraph.h"
#include "physics/Island.h"
#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class VoxelScene;

namespace physics {

struct BondStrength {
  float E = 5.0e7f;
  float G = 1.9e7f;
  float sigmaT = 4.0e5f;
  float sigmaC = 1.2e6f;
  float tauU = 5.0e5f;
  float density = kDensityWood;
  // Impulse to fail one fine-sized patch. Independent of h (not F = J/Δt).
  float jBreak = 40.0f;
};

struct BondNode {
  int shapeIndex = -1;
  int islandId = -1;
  glm::ivec3 coarse{0};
  float mass = 1.0f;
  glm::vec3 Idiag{1.0f};
  glm::vec3 rest{0.0f};
  glm::vec3 u{0.0f};
  glm::vec3 theta{0.0f};
  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};
  bool anchored = false;
};

struct Bond {
  int a = -1;
  int b = -1;
  int axis = 0;
  int islandId = -1;
  int color = 0;
  float A = 1.0f;
  float I = 1.0f;
  float J = 1.0f;
  float L = 1.0f;
  float kn = 1.0f;
  float kv = 1.0f;
  float kt = 1.0f;
  float km = 1.0f;
  float lambda[6] = {0, 0, 0, 0, 0, 0};
  BondStrength strength{};
  bool alive = true;
  float phi = 0.0f;
};

struct DebugBond {
  glm::vec3 a{0.0f};
  glm::vec3 b{0.0f};
  float phi = 0.0f;
  bool alive = true;
};

class StructureWorld {
public:
  void attach(VoxelScene& scene);
  void rebuildAll();
  void markDirty(int shapeIndex);
  void markDirtyCells(int shapeIndex, const std::vector<glm::ivec3>& coarses);
  void onSplit(int srcIndex, int dstIndex);
  void beginDt();
  void substep();
  void updateSleep(float h);
  void wakeShape(int shapeIndex);
  void wakeFromContacts(const std::vector<Contact>& contacts,
                        const std::vector<RigidBody>& bodies);
  void applyContactImpulses(const std::vector<Contact>& contacts,
                            const std::vector<RigidBody>& bodies);
  void fillDebug(DebugSolve& debug) const;
  const std::vector<DebugBond>& debugBonds() const { return debugBonds_; }
  void fillCoarsePhi(std::vector<float>& dst) const;

private:
  void rebuildShape(int shapeIndex);
  void rebuildNeighborhood(int shapeIndex, const std::vector<glm::ivec3>& seeds);
  void rebuildGraphColors();
  void rebuildIslands(const std::unordered_map<uint64_t, bool>* prevSleep = nullptr);
  void splitOneIslandIfNeeded();
  void wakeIslandOfCoarse(int shapeIndex, const glm::ivec3& c);
  bool islandAwake(int islandId) const;
  std::unordered_map<uint64_t, bool> snapshotIslandSleep() const;
  void applyGhostGravity();
  void solveBonds();
  void solveBond(Bond& bond, float h);
  void solveAxis(BondNode& na, BondNode& nb, const glm::vec3& n, const glm::vec3& ra,
                 const glm::vec3& rb, float C, float& lambda, float kn, float h);
  void clampBondLambda(Bond& bond, float h) const;
  int breakOverstressed();
  void fractureDeadBonds();
  void integrateGhost();
  bool addNode(int shapeIndex, const glm::ivec3& c, const BondNode* prev);
  void addBondIfMissing(int shapeIndex, const glm::ivec3& lower, int axis,
                        const std::unordered_map<uint64_t, Bond>* warm,
                        std::unordered_set<uint64_t>* createdKeys);
  void dropShapeRegion(int shapeIndex, const std::unordered_set<uint64_t>* dropKeys);
  uint64_t bondKey(const glm::ivec3& lower, int axis) const;
  void snapshotDebug();
  float bondPhi(const Bond& bond, float h) const;
  BondStrength strengthForMaterial(uint32_t mat) const;
  uint64_t nodeKey(int shapeIndex, const glm::ivec3& c) const;
  int findNode(int shapeIndex, const glm::ivec3& c) const;
  bool isStructural(int shapeIndex) const;

  VoxelScene* scene_ = nullptr;
  std::vector<BondNode> nodes_;
  std::vector<Bond> bonds_;
  std::unordered_map<uint64_t, int> nodeIndex_;
  ConstraintGraph graph_;
  IslandSet islands_;
  std::vector<DebugBond> debugBonds_;
  bool splitConsumedThisDt_ = false;
  int brokenThisStep_ = 0;
  int brokenThisDt_ = 0;
  float maxPhi_ = 0.0f;
  float maxPhiPrev_ = 0.0f;
  std::vector<int> newlyBroken_;
  std::unordered_set<int> dirtyFull_;
};

}  // namespace physics
