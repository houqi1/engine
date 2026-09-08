#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace voxel {

struct FineCoord {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;

  bool operator==(const FineCoord& o) const { return x == o.x && y == o.y && z == o.z; }
  bool operator!=(const FineCoord& o) const { return !(*this == o); }
};

struct FineCoordHash {
  size_t operator()(const FineCoord& p) const noexcept {
    // 3D hash mixed into size_t; good enough for sparse test grids.
    const uint64_t x = static_cast<uint32_t>(p.x);
    const uint64_t y = static_cast<uint32_t>(p.y);
    const uint64_t z = static_cast<uint32_t>(p.z);
    uint64_t h = x * 73856093ull ^ y * 19349663ull ^ z * 83492791ull;
    return static_cast<size_t>(h);
  }
};

struct CoarseCoord {
  int32_t x = 0;
  int32_t y = 0;
  int32_t z = 0;
  bool operator==(const CoarseCoord& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct CoarseCoordHash {
  size_t operator()(const CoarseCoord& p) const noexcept {
    const uint64_t x = static_cast<uint32_t>(p.x);
    const uint64_t y = static_cast<uint32_t>(p.y);
    const uint64_t z = static_cast<uint32_t>(p.z);
    return static_cast<size_t>(x * 73856093ull ^ y * 19349663ull ^ z * 83492791ull);
  }
};

inline constexpr int kFinePerCoarse = 16;
inline constexpr uint32_t kUnvisitedLabel = 0xFFFFFFFFu;
inline constexpr uint32_t kLabelsPerSearchPage = 4096u;  // 16^3

inline CoarseCoord fineToCoarse(FineCoord p) {
  auto divFloor = [](int32_t v, int32_t d) -> int32_t {
    return v >= 0 ? v / d : -((-v + d - 1) / d);
  };
  return CoarseCoord{divFloor(p.x, kFinePerCoarse), divFloor(p.y, kFinePerCoarse),
                     divFloor(p.z, kFinePerCoarse)};
}

inline uint32_t fineIndexInCoarse(FineCoord p) {
  auto modPositive = [](int32_t v, int32_t d) -> int32_t {
    const int32_t m = v % d;
    return m >= 0 ? m : m + d;
  };
  const int32_t lx = modPositive(p.x, kFinePerCoarse);
  const int32_t ly = modPositive(p.y, kFinePerCoarse);
  const int32_t lz = modPositive(p.z, kFinePerCoarse);
  return static_cast<uint32_t>(lx + ly * kFinePerCoarse + lz * kFinePerCoarse * kFinePerCoarse);
}

// Read-only occupancy for connectivity. Engine wraps VoxelObject + edit patches.
class SolidView {
public:
  virtual ~SolidView() = default;
  virtual bool isSolid(FineCoord p) const = 0;
};

struct ConnectivityBudget {
  // Max fine expansions this call. 0 = unlimited.
  uint64_t maxExpansions = 0;
  // Fair share: expand up to this many frontier voxels per active root per round.
  uint32_t perRootQuota = 32;
};

struct ConnectivityMetrics {
  uint64_t deletedFineCount = 0;
  uint64_t seedCount = 0;
  uint64_t visitedFineCount = 0;
  uint64_t allocatedSearchPageCount = 0;
  uint64_t mergedGroupCount = 0;
  uint64_t finalizedComponentCount = 0;
  bool remainderEarlyExit = false;
  uint64_t expansionCount = 0;
};

enum class ConnectivityStatus : uint8_t {
  Pending = 0,
  Connected,          // all seeds in one active group (may not be exhausted)
  ComponentsFound,    // one or more finalized components (+ optional remainder)
  Empty,              // no surviving solid near deletions
  Failed,
};

struct ComponentMask {
  CoarseCoord coarse{};
  // Bit i set => fineIndexInCoarse i belongs to this component.
  std::array<uint64_t, 64> bits{};  // 4096 bits

  void set(uint32_t fineInCoarse) {
    bits[fineInCoarse >> 6] |= (uint64_t{1} << (fineInCoarse & 63u));
  }
  bool test(uint32_t fineInCoarse) const {
    return (bits[fineInCoarse >> 6] & (uint64_t{1} << (fineInCoarse & 63u))) != 0;
  }
};

struct FinalizedComponent {
  uint32_t rootLabel = 0;
  uint64_t voxelCount = 0;
  std::vector<ComponentMask> masks;
};

struct ConnectivityResult {
  ConnectivityStatus status = ConnectivityStatus::Pending;
  // Finalized components discovered by exhausting frontiers.
  std::vector<FinalizedComponent> components;
  // True when search stopped because only one active group remained.
  bool hasRemainder = false;
  // When Connected: the single surviving root label (for tests).
  uint32_t connectedRoot = kUnvisitedLabel;
  ConnectivityMetrics metrics{};
};

// Full BFS oracle: returns component id per solid voxel (dense map). Component ids are dense from 0.
std::unordered_map<FineCoord, uint32_t, FineCoordHash> fullBfsComponents(
    const SolidView& view, const std::vector<FineCoord>& seeds);

// Collect unique surviving face-neighbors of deleted voxels.
std::vector<FineCoord> collectBoundarySeeds(const SolidView& view,
                                            const std::vector<FineCoord>& removed);

class MultiSourceConnectivity {
public:
  void reset(const SolidView* view, std::vector<FineCoord> seeds);
  // Advance search under budget. Returns true when status is no longer Pending.
  bool step(ConnectivityBudget budget);
  const ConnectivityResult& result() const { return result_; }
  ConnectivityStatus status() const { return result_.status; }

  // For tests: label of a visited fine after findRoot, or Unvisited.
  uint32_t labelAt(FineCoord p) const;

private:
  struct SearchGroup {
    uint32_t parent = 0;
    uint32_t size = 1;
    std::vector<FineCoord> frontier;
    uint64_t voxelCount = 0;
    bool finalized = false;
    bool active = true;
  };

  using SearchPage = std::array<uint32_t, kLabelsPerSearchPage>;

  const SolidView* view_ = nullptr;
  std::vector<SearchGroup> groups_;
  std::unordered_map<CoarseCoord, SearchPage, CoarseCoordHash> pages_;
  std::vector<FineCoord> seeds_;
  ConnectivityResult result_{};

  uint32_t findRoot(uint32_t label) const;
  uint32_t findRootMutable(uint32_t label);
  void mergeGroups(uint32_t a, uint32_t b);
  uint32_t* labelPtr(FineCoord p, bool create);
  uint32_t lookupLabel(FineCoord p) const;
  void assignLabel(FineCoord p, uint32_t label);
  void expandOne(uint32_t root);
  void tryFinalize(uint32_t root);
  void checkTermination();
  void buildMasksForRoot(uint32_t root, FinalizedComponent& out) const;
  uint32_t activeRootCount() const;
};

}  // namespace voxel
