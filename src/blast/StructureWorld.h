#pragma once

#include "blast/BlastMemory.h"
#include "blast/ContactLoads.h"
#include "blast/OccupancySampler.h"
#include "blast/VoxelGraph.h"

#include "scene/VoxelTypes.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastTypes.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

namespace blast {

struct StructureDebugSnapshot {
  bool hasInstance = false;
  bool converged = false;
  const char* status = "idle";
  uint32_t occupied = 0;
  uint32_t nodes = 0;
  uint32_t bonds = 0;
  uint32_t worldBonds = 0;
  uint64_t topologyRevision = 0;
  float mass = 0.0f;
  float weight = 0.0f;
  float reactionY = 0.0f;
  float maxTension = 0.0f;
  float maxCompression = 0.0f;
  float maxShear = 0.0f;
  float stripMaxStress = 0.0f;
  float linErr = 0.0f;
  float angErr = 0.0f;
  float extractMs = 0.0f;
  float assetMs = 0.0f;
  float solveMs = 0.0f;
  float probeMs = 0.0f;
  float statsMs = 0.0f;
  float candidateMs = 0.0f;
  uint32_t solverIters = 200;
  float density = 1000.0f;
  bool cut = false;
  bool fractureEnabled = false;
  float strengthPa = 5.0e7f;
  uint32_t candidateCount = 0;
  uint32_t candidateInStrip = 0;
  uint32_t fracturedBonds = 0;
  uint32_t splitActors = 1;
  uint32_t probeExportCount = 0;
  uint32_t probeCount = 0;
  uint64_t solveEpoch = 0;
  uint32_t contactContrib = 0;
  uint32_t mappedNodes = 0;
  uint32_t invalidSnapshots = 0;
  uint32_t bindingCount = 0;
  float contactLoadMs = 0.0f;
};

struct FractureCandidate {
  uint32_t stableId = 0;
  uint32_t sdkIndex = 0;
  uint32_t nodeA = 0;
  uint32_t nodeB = 0;
  uint32_t node0 = 0;
  uint32_t node1 = 0;
  uint32_t owner = 0;
  float stress = 0.0f;
  float strength = 0.0f;
  bool inStrip = false;
  bool anchored = false;
  std::vector<uint64_t> faces;
};

// One ExtStress snapshot is only valid until the first bonds break. Applying every
// over-S bond from that snapshot deletes members that would be under S after
// unload. Each solveEpoch submits at most this many hottest over-S bonds per actor;
// the next solve (same gravity, new topology) decides the rest.
constexpr uint32_t kFractureBondsPerSolve = 1;

void keepHottestCandidates(std::vector<FractureCandidate>& candidates,
                           uint32_t maxPerActor = kFractureBondsPerSolve);

struct PendingFracture {
  bool valid = false;
  VoxelObjectId objectId{};
  uint64_t topologyRevision = 0;
  uint32_t solverTopologyEpoch = 0;
  uint64_t solveEpoch = 0;
  uint64_t strengthEpoch = 0;
  float strengthPa = 0.0f;
  std::vector<FractureCandidate> candidates;
};

struct BondMeta {
  uint32_t stableId = 0;
  uint32_t graphIndex = 0xFFFFFFFFu;
  uint8_t world = 0;
  uint8_t inStrip = 0;
  uint8_t inNeck = 0;
};

struct ActorBinding {
  NvBlastActor* actor = nullptr;
  VoxelObjectId objectId{};
  bool anchored = false;
  bool stressSolve = false;
  uint32_t graphNodeCount = 0;
  glm::vec3 comAsset{0.0f};
  glm::ivec3 fineOrigin{0};
  int fineN = 0;
  std::vector<NodeRef> nodeRefs;
};

struct ActorObjectLink {
  NvBlastActor* actor = nullptr;
  VoxelObjectId objectId{};
  glm::ivec3 fineOrigin{0};
  int fineN = 0;
};

struct ActorLoadSnapshot {
  uint64_t tickId = 0;
  VoxelObjectId objectId{};
  uint32_t topologyEpoch = 0;
  uint64_t loadEpoch = 0;
  std::vector<MappedLoad> loads;
  bool evaluated = false;
  bool valid = true;
  const char* invalidReason = "ok";
  uint32_t contactContrib = 0;
  uint32_t mappedNodes = 0;
};

struct StructureInstance {
  VoxelObjectId objectId{};
  uint64_t topologyRevision = 0;
  uint64_t solveEpoch = 0;
  uint64_t strengthEpoch = 0;
  uint32_t occupiedCached = 0;
  uint32_t probeCount = 0;
  VoxelGrid grid;
  VoxelStructureGraph graph;
  VoxelBlast blast;
  std::vector<Nv::Blast::ExtStressSolver::BondProbe> probes;
  std::vector<BondMeta> bondMeta;
  std::vector<NvBlastActor*> actorScratch;
  std::vector<uint32_t> nodeOwner;
  std::vector<uint32_t> nodeIndexScratch;
  std::vector<NvBlastBondFractureData> cmdScratch;
  StructureDebugSnapshot debug{};
  float axisX = 0.0f;
  float axisZ = 0.0f;
  float keepAz0 = 0.0f;
  float keepAz1 = 1.5707963267948966f;
  bool keepBox = false;
  float keepX0 = 0.0f;
  float keepX1 = 0.0f;
  float keepZ0 = 0.0f;
  float keepZ1 = 0.0f;
  uint32_t solverIters = 200;
  bool cutApplied = false;
  bool fractureEnabled = false;
  float strengthPa = 5.0e7f;
  PendingFracture pending{};
  std::vector<ActorBinding> bindings;
  std::vector<ActorLoadSnapshot> loadSnapshots;
  uint64_t loadEpoch = 0;
  uint64_t lastLoadTick = 0;
  uint32_t lastBoundTopologyEpoch = 0xFFFFFFFFu;
};

class StructureWorld {
public:
  StructureWorld() = default;
  ~StructureWorld() { shutdown(); }
  StructureWorld(const StructureWorld&) = delete;
  StructureWorld& operator=(const StructureWorld&) = delete;
  StructureWorld(StructureWorld&&) = delete;
  StructureWorld& operator=(StructureWorld&&) = delete;

  bool init(BlastRuntime& runtime);
  void shutdown();
  void clear();
  void resetTickSession();

  void onPhysicsTick(uint64_t tickId, float dt);
  void onPhysicsTick(uint64_t tickId, float dt, const WorldContactImpulse* impulses, uint32_t nImpulses);
  void bindVisibleActors(const std::vector<ActorObjectLink>& links);
  const std::vector<ActorBinding>& bindings() const {
    static const std::vector<ActorBinding> kEmpty;
    return instances_.empty() ? kEmpty : instances_.front().bindings;
  }

  BlastError mountSample(VoxelObjectId objectId, OccupancySample sample, uint32_t solverIters, float strengthPa,
                         float axisX, float axisZ);
  void unmount(VoxelObjectId objectId);
  bool setDensityScale(float scale);
  void setSolverIters(uint32_t iters);
  void markCut(bool cut);
  void setKeepColumnBox(float x0, float x1, float z0, float z1);
  uint32_t warmupGravity(uint32_t maxPasses);
  void setFractureEnabled(bool on);
  void setStrengthPa(float strengthPa);
  PendingFracture takePendingFracture();
  const PendingFracture& pendingFracture() const { return instances_.empty() ? idlePending_ : instances_.front().pending; }
  void clearPendingFracture();
  bool pendingSnapshotMatches(const PendingFracture& pending) const;
  uint32_t applyPendingCandidates(const PendingFracture& pending);
  uint32_t splitAllRequired();
  void recacheOccupied();
  void recachePendingFromProbes();

  bool initialized() const { return initialized_; }
  uint32_t instanceCount() const { return static_cast<uint32_t>(instances_.size()); }
  uint64_t physicsTicksReceived() const { return ticksReceived_; }
  uint64_t lastTickId() const { return lastTickId_; }
  float lastDt() const { return lastDt_; }
  std::size_t blastLiveBytes() const;
  std::size_t blastRuntimeBaselineBytes() const;
  int blastErrorCount() const;
  const StructureDebugSnapshot& debug() const;
  StructureInstance* instance();
  const StructureInstance* instance() const;
  const char* lastError() const { return lastError_; }

private:
  void solveInstance(StructureInstance& inst, const WorldContactImpulse* impulses, uint32_t nImpulses);
  void refreshDebug(StructureInstance& inst, float solveMs);
  void collectPendingFracture(StructureInstance& inst);
  void rebuildBondMeta(StructureInstance& inst);
  void fillNodeOwners(StructureInstance& inst, const NvBlastSupportGraph& graph);
  void rebuildBindingsFromFamily(StructureInstance& inst);
  void fillBindingKinematics(StructureInstance& inst, ActorBinding& b);
  std::vector<NodeRef> actorNodeRefs(const StructureInstance& inst, const ActorBinding& b) const;
  bool pickContactNode(const StructureInstance& inst, const ActorBinding& b, const WorldContactImpulse& imp,
                       bool sideA, NodeRef& out) const;
  void buildLoadSnapshots(StructureInstance& inst, const WorldContactImpulse* impulses, uint32_t nImpulses,
                          uint64_t tickId, float dt);

  BlastRuntime* runtime_ = nullptr;
  bool initialized_ = false;
  uint64_t ticksReceived_ = 0;
  uint64_t lastTickId_ = 0;
  float lastDt_ = 0.0f;
  std::vector<StructureInstance> instances_;
  StructureDebugSnapshot idle_{};
  PendingFracture idlePending_{};
  const char* lastError_ = "ok";
};

}  // namespace blast
