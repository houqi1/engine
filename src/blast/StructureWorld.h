#pragma once

#include "blast/BlastMemory.h"
#include "blast/ContactLoads.h"
#include "blast/ImpactDamage.h"
#include "blast/OccupancySampler.h"
#include "blast/VoxelGraph.h"

#include "scene/VoxelTypes.h"

#include "NvBlast.h"
#include "NvBlastExtStressSolver.h"
#include "NvBlastTypes.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <memory>
#include <unordered_map>
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
  // E5.4 contact loads, per tick.
  uint32_t contactPairs = 0;       // smoothed pairs applied
  uint32_t frozenPairs = 0;        // held because every body in the pair sleeps
  uint32_t contactLoadNodes = 0;   // addLoad calls
  uint32_t persistentContacts = 0; // persistent impulses routed to addLoad
  uint32_t impactSkippedPersistent = 0;
  float contactForceN = 0.0f;      // sum of |F| applied
  bool stressImpactImpulses = false;
  float stressImpactScale = 0.01f;
  uint32_t gravityActors = 0;
  uint32_t centrifugalActors = 0;
  bool impactDamageEnabled = true;
  uint32_t impactDamageEvents = 0;
  float maxBondDamage = 0.0f;
  uint32_t damagedBondCount = 0;
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
  float damage = 0.0f;
  float healthBefore = 0.0f;
  float cy = 0.0f;
  bool inStrip = false;
  bool anchored = false;
  std::vector<uint64_t> faces;
};

constexpr float kExtStressImpactImpulseFactor = 0.01f;

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

// E5.4: smoothed persistent contact load from one other body, in asset space.
struct PairContactLoad {
  VoxelObjectId other{};
  std::vector<MappedLoad> loads;
  uint64_t lastSeenTick = 0;
};

struct ActorBinding {
  NvBlastActor* actor = nullptr;
  VoxelObjectId objectId{};
  bool anchored = false;
  bool stressSolve = false;
  uint32_t graphNodeCount = 0;
  float mass = 0.0f;  // Sum of this actor's Blast node masses.
  glm::vec3 comAsset{0.0f};
  glm::ivec3 fineOrigin{0};
  int fineN = 0;
  std::vector<NodeRef> nodeRefs;
  std::unordered_map<uint32_t, uint32_t> nodeRefIndex;  // Blast graph node -> nodeRefs index
  std::vector<PairContactLoad> contactLoads;            // kept while the actor survives
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

// Stable identity of a mounted structure. Ids are never reused, so a handle to a
// released or rebuilt instance resolves to nullptr instead of another structure.
struct StructureHandle {
  uint32_t id = 0;
  bool valid() const { return id != 0; }
  bool operator==(const StructureHandle& o) const { return id == o.id; }
  bool operator!=(const StructureHandle& o) const { return id != o.id; }
};

struct StructureMaterial {
  float baseDensity = 1000.0f;  // kg/m^3 of the sampled occupancy; setDensityScale multiplies it
  float strengthPa = 5.0e7f;
  uint32_t solverIters = 200;
  bool fractureEnabled = false;
  // > 0: this structure keeps its own strength; the global setStrengthPa (UI hold /
  // fail switch for the demos) leaves it alone.
  float ownStrengthPa = 0.0f;
};

// Cylinder / four-column regression diagnostics: which bonds form the surviving
// strip and neck. Only affects debug statistics, never fracture decisions.
struct DiagnosticProfile {
  enum class Kind : uint8_t { None, CylinderStrip, ColumnBox };
  Kind kind = Kind::None;
  float axisX = 0.0f;
  float axisZ = 0.0f;
  float keepAz0 = 0.0f;
  float keepAz1 = 1.5707963267948966f;
  float keepX0 = 0.0f;
  float keepX1 = 0.0f;
  float keepZ0 = 0.0f;
  float keepZ1 = 0.0f;
  bool cutApplied = false;
};

struct StructureMountDesc {
  VoxelObjectId objectId{};
  StructureMaterial material{};
  DiagnosticProfile diag{};
  // Remounting the same object keeps its current strength, fracture switch and
  // strength epoch instead of desc.material's (other instances never donate).
  bool keepMaterialOfReplaced = false;
};

struct StructureInstance {
  StructureHandle handle{};
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
  std::vector<NvBlastChunkFractureData> chunkScratch;
  StructureDebugSnapshot debug{};
  StructureMaterial material{};
  DiagnosticProfile diag{};
  PendingFracture pending{};
  std::vector<ActorBinding> bindings;
  std::vector<ActorLoadSnapshot> loadSnapshots;
  uint64_t loadEpoch = 0;
  uint64_t lastLoadTick = 0;
  uint32_t lastBoundTopologyEpoch = 0xFFFFFFFFu;
  ImpulseEvents impactEvents{};
  bool impactAppliedThisTick = false;
  bool occupancyDirty = false;
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
  void onPhysicsTick(uint64_t tickId, float dt, const WorldContactImpulse* impulses, uint32_t nImpulses,
                     const BodyKinematics* kinematics, uint32_t nKinematics);
  void bindVisibleActors(const std::vector<ActorObjectLink>& links, StructureInstance* target = nullptr);
  const std::vector<ActorBinding>& bindings() const;
  // Visits every instance's bindings without copying them (per-tick callers).
  template <typename Fn>
  void forEachBinding(Fn&& fn) const {
    for (const auto& inst : instances_) {
      for (const ActorBinding& b : inst->bindings) {
        fn(b);
      }
    }
  }

  // Replaces any instance already mounted for desc.objectId.
  BlastError mount(const StructureMountDesc& desc, OccupancySample sample, StructureHandle* outHandle = nullptr);
  // Legacy cylinder-strip mount used by headless tests; remounts keep material.
  BlastError mountSample(VoxelObjectId objectId, OccupancySample sample, uint32_t solverIters, float strengthPa,
                         float axisX, float axisZ);
  // Releases instances mounted for this object and detaches it from any other
  // instance's actor bindings (e.g. a freed fragment slot).
  void unmount(VoxelObjectId objectId);
  bool ownsObject(VoxelObjectId objectId) const;
  // Drop these object-local fines from the mounted structure and rebuild the family.
  // Failure leaves the previous family in place.
  BlastError applyOccupancyRemoval(VoxelObjectId objectId, const glm::ivec3* localFines, uint32_t count);
  // Omitted targets below mean the first instance (single-structure tests only);
  // scene code always passes the instance it means.
  bool setDensityScale(float scale, StructureInstance* target = nullptr);
  void setSolverIters(uint32_t iters, StructureInstance* target = nullptr);
  void markCut(bool cut, StructureInstance* target = nullptr);
  void setKeepColumnBox(float x0, float x1, float z0, float z1, StructureInstance* target = nullptr);
  uint32_t warmupGravity(uint32_t maxPasses, StructureInstance* target = nullptr);
  void setFractureEnabled(bool on);
  void setStrengthPa(float strengthPa);
  // E5.4: persistent contacts load the stress solver (addLoad); one-shot impacts keep
  // the Viewer route. Off keeps the pre-E5.4 behavior.
  void setContactLoadsEnabled(bool on) { contactLoadsEnabled_ = on; }
  bool contactLoadsEnabled() const { return contactLoadsEnabled_; }
  // SampleAssetViewer: pass impact to stress instead of the damage shader.
  // addForce(contact, ViewerForce * 0.01). Default off.
  void setStressImpactImpulses(bool on);
  void setStressImpactScale(float scale);
  bool stressImpactImpulses() const { return stressImpactImpulses_; }
  float stressImpactScale() const { return stressImpactScale_; }
  void setImpactDamageEnabled(bool on);
  bool impactDamageEnabled() const { return impactDamageEnabled_; }
  void setImpactSettings(const ImpactSettings& settings);
  const ImpactSettings& impactSettings() const { return impactSettings_; }
  void setImpactMaterial(const NvBlastExtMaterial& material) { impactMaterial_ = material; }
  const NvBlastExtMaterial& impactMaterial() const { return impactMaterial_; }
  PendingFracture takePendingFracture();
  const PendingFracture& pendingFracture() const { return instances_.empty() ? idlePending_ : instances_.front()->pending; }
  // Scene commits pass an explicit instance. Omitting it preserves the original
  // single-structure API used by diagnostics and headless tests.
  void clearPendingFracture(StructureInstance* target = nullptr);
  bool pendingSnapshotMatches(const PendingFracture& pending, const StructureInstance* target = nullptr) const;
  uint32_t applyPendingCandidates(const PendingFracture& pending, StructureInstance* target = nullptr);
  uint32_t applyPendingIfAny(StructureInstance* target = nullptr);
  uint32_t splitAllRequired(StructureInstance* target = nullptr, bool force = false);
  bool takeOccupancyDirty(StructureInstance* target = nullptr);
  void recacheOccupied(StructureInstance* target = nullptr);
  void recachePendingFromProbes(StructureInstance* target = nullptr);

  bool initialized() const { return initialized_; }
  uint32_t instanceCount() const { return static_cast<uint32_t>(instances_.size()); }
  uint64_t physicsTicksReceived() const { return ticksReceived_; }
  uint64_t lastTickId() const { return lastTickId_; }
  float lastDt() const { return lastDt_; }
  std::size_t blastLiveBytes() const;
  std::size_t blastRuntimeBaselineBytes() const;
  int blastErrorCount() const;
  const StructureDebugSnapshot& debug(const StructureInstance* target = nullptr) const;
  // Idle snapshot when nothing is mounted for this object.
  const StructureDebugSnapshot& debug(VoxelObjectId objectId) const {
    const StructureInstance* inst = find(objectId);
    return inst != nullptr ? inst->debug : idle_;
  }
  // First instance; single-structure tests and diagnostics only.
  StructureInstance* instance();
  const StructureInstance* instance() const;
  StructureInstance* instanceAt(uint32_t index) {
    return index < instances_.size() ? instances_[index].get() : nullptr;
  }
  StructureInstance* find(StructureHandle handle);
  const StructureInstance* find(StructureHandle handle) const;
  // Instance mounted for this object (its root objectId, not a fragment binding).
  StructureInstance* find(VoxelObjectId objectId);
  const StructureInstance* find(VoxelObjectId objectId) const;
  const char* lastError() const { return lastError_; }

private:
  void solveInstance(StructureInstance& inst, const WorldContactImpulse* impulses, uint32_t nImpulses,
                     const BodyKinematics* kinematics, uint32_t nKinematics);
  const BodyKinematics* findKinematics(VoxelObjectId id, const BodyKinematics* kinematics, uint32_t nKinematics) const;
  void refreshDebug(StructureInstance& inst, float solveMs);
  void collectPendingFracture(StructureInstance& inst);
  void rebuildBondMeta(StructureInstance& inst);
  void fillNodeOwners(StructureInstance& inst, const NvBlastSupportGraph& graph);
  void rebuildBindingsFromFamily(StructureInstance& inst);
  using NodesByStable = std::unordered_map<uint32_t, const GraphNode*>;
  // Built once per rebind; per-actor work is then proportional to the actor's own nodes.
  static NodesByStable nodesByStable(const StructureInstance& inst);
  void fillBindingKinematics(StructureInstance& inst, ActorBinding& b, const NodesByStable& byStable);
  // E5.4: fold this tick's persistent contacts into each binding's smoothed pair loads.
  void updateContactLoads(StructureInstance& inst, const WorldContactImpulse* impulses, uint32_t nImpulses,
                          const BodyKinematics* kinematics, uint32_t nKinematics, uint64_t tickId, float dt);
  void applyContactLoads(StructureInstance& inst);
  // Node refs of b's actor; nodesOut (optional) receives the matching graph nodes.
  std::vector<NodeRef> actorNodeRefs(const StructureInstance& inst, const ActorBinding& b, const NodesByStable& byStable,
                                     std::vector<const GraphNode*>* nodesOut = nullptr) const;
  bool pickContactNode(const StructureInstance& inst, const ActorBinding& b, const WorldContactImpulse& imp,
                       bool sideA, NodeRef& out) const;
  void buildLoadSnapshots(StructureInstance& inst, const WorldContactImpulse* impulses, uint32_t nImpulses,
                          uint64_t tickId, float dt);
  void applyViewerImpact(StructureInstance& inst, const WorldContactImpulse* impulses, uint32_t nImpulses);
  uint32_t applyImpactShaderToActor(StructureInstance& inst, NvBlastActor* actor, const glm::vec3& localPos,
                                    const glm::vec3& localForce, float normalized);
  void applyImpactSide(StructureInstance& inst, ActorBinding* b, const glm::vec3& worldPos,
                       const glm::vec3& worldForce, const glm::quat& q, const glm::vec3& x);
  void syncBondDamageFromHealth(StructureInstance& inst);
  ActorBinding* bindingForObject(StructureInstance& inst, VoxelObjectId id);

  BlastRuntime* runtime_ = nullptr;
  bool initialized_ = false;
  uint64_t ticksReceived_ = 0;
  uint64_t lastTickId_ = 0;
  float lastDt_ = 0.0f;
  // unique_ptr keeps instance addresses stable across mounts and rebuilds.
  std::vector<std::unique_ptr<StructureInstance>> instances_;
  uint32_t nextHandleId_ = 1;
  mutable std::vector<ActorBinding> bindingCache_;
  StructureDebugSnapshot idle_{};
  PendingFracture idlePending_{};
  const char* lastError_ = "ok";
  bool stressImpactImpulses_ = false;
  bool contactLoadsEnabled_ = false;
  float contactLoadTau_ = 0.1f;  // s, smoothing of solver contact impulses
  float stressImpactScale_ = 0.01f;
  bool impactDamageEnabled_ = true;
  ImpactSettings impactSettings_{};
  NvBlastExtMaterial impactMaterial_{};
};

}  // namespace blast
