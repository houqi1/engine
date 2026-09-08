#include "voxel/VoxelConnectivity.h"

#include <cstdlib>
#include <iostream>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

using namespace voxel;

namespace {

struct DenseGridView final : SolidView {
  int size = 0;
  std::vector<uint8_t> solid;  // size^3

  explicit DenseGridView(int n) : size(n), solid(static_cast<size_t>(n) * n * n, 0) {}

  size_t index(FineCoord p) const {
    return static_cast<size_t>(p.x) + static_cast<size_t>(p.y) * static_cast<size_t>(size) +
           static_cast<size_t>(p.z) * static_cast<size_t>(size) * static_cast<size_t>(size);
  }

  bool inBounds(FineCoord p) const {
    return p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < size && p.y < size && p.z < size;
  }

  bool isSolid(FineCoord p) const override {
    if (!inBounds(p)) {
      return false;
    }
    return solid[index(p)] != 0;
  }

  void set(FineCoord p, bool v) {
    if (inBounds(p)) {
      solid[index(p)] = v ? 1 : 0;
    }
  }

  std::vector<FineCoord> allSolid() const {
    std::vector<FineCoord> out;
    for (int z = 0; z < size; ++z) {
      for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
          FineCoord p{x, y, z};
          if (isSolid(p)) {
            out.push_back(p);
          }
        }
      }
    }
    return out;
  }
};

int gFailures = 0;

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAIL: " << msg << "\n";
    ++gFailures;
  }
}

bool samePartition(const std::unordered_map<FineCoord, uint32_t, FineCoordHash>& oracle,
                   MultiSourceConnectivity& search, const std::vector<FineCoord>& solids) {
  // Pairwise: two voxels same component in oracle iff same root label in search (when visited).
  // For Connected status without full visit, only seed-reachable solids are labeled; compare on
  // the union of oracle keys that are also visited OR use oracle on all solids from seeds.
  for (size_t i = 0; i < solids.size(); ++i) {
    for (size_t j = i + 1; j < solids.size(); ++j) {
      const FineCoord a = solids[i];
      const FineCoord b = solids[j];
      const auto ia = oracle.find(a);
      const auto ib = oracle.find(b);
      if (ia == oracle.end() || ib == oracle.end()) {
        continue;
      }
      const bool sameOracle = ia->second == ib->second;
      const uint32_t la = search.labelAt(a);
      const uint32_t lb = search.labelAt(b);
      // If either unvisited, skip pairwise (remainder early exit may leave large body unlabeled).
      if (la == kUnvisitedLabel || lb == kUnvisitedLabel) {
        continue;
      }
      const bool sameSearch = la == lb;
      if (sameOracle != sameSearch) {
        return false;
      }
    }
  }
  return true;
}

ConnectivityResult runSearch(const SolidView& view, const std::vector<FineCoord>& removed,
                             ConnectivityBudget budget) {
  auto seeds = collectBoundarySeeds(view, removed);
  MultiSourceConnectivity search;
  search.reset(&view, seeds);
  while (search.status() == ConnectivityStatus::Pending) {
    search.step(budget);
  }
  return search.result();
}

void testSurfaceDigNoSplit() {
  DenseGridView g(8);
  for (int z = 2; z <= 5; ++z)
    for (int y = 2; y <= 5; ++y)
      for (int x = 2; x <= 5; ++x)
        g.set({x, y, z}, true);
  const FineCoord dig{2, 3, 3};
  g.set(dig, false);
  auto seeds = collectBoundarySeeds(g, {dig});
  MultiSourceConnectivity search;
  search.reset(&g, seeds);
  ConnectivityBudget b;
  b.maxExpansions = 1;
  while (search.status() == ConnectivityStatus::Pending) {
    search.step(b);
  }
  expect(search.status() == ConnectivityStatus::Connected, "surface dig stays connected");
}

void testBridgeCut() {
  DenseGridView g(16);
  // Two 3x3x3 blocks linked by one voxel bridge.
  for (int z = 2; z <= 4; ++z)
    for (int y = 2; y <= 4; ++y)
      for (int x = 1; x <= 3; ++x)
        g.set({x, y, z}, true);
  for (int z = 2; z <= 4; ++z)
    for (int y = 2; y <= 4; ++y)
      for (int x = 7; x <= 9; ++x)
        g.set({x, y, z}, true);
  g.set({4, 3, 3}, true);
  g.set({5, 3, 3}, true);
  g.set({6, 3, 3}, true);
  const FineCoord cut{5, 3, 3};
  g.set(cut, false);

  auto seeds = collectBoundarySeeds(g, {cut});
  expect(seeds.size() >= 2, "bridge cut has multiple seeds");
  MultiSourceConnectivity search;
  search.reset(&g, seeds);
  ConnectivityBudget b;
  b.maxExpansions = 4;
  b.perRootQuota = 2;
  while (search.status() == ConnectivityStatus::Pending) {
    search.step(b);
  }
  expect(search.status() == ConnectivityStatus::ComponentsFound, "bridge cut splits");
  expect(search.result().components.size() >= 1, "at least one finalized component");
  expect(search.result().hasRemainder || search.result().components.size() >= 2,
         "remainder or two finalized");

  auto oracle = fullBfsComponents(g, seeds);
  std::unordered_set<uint32_t> ids;
  for (const auto& kv : oracle) {
    ids.insert(kv.second);
  }
  expect(ids.size() == 2, "oracle sees two components");
}

void testCornerTouchNotConnected() {
  DenseGridView g(8);
  g.set({2, 2, 2}, true);
  g.set({3, 3, 3}, true);
  // Delete nothing between them — they are not face-adjacent.
  // Simulate "removed" empty set with artificial seeds at both.
  std::vector<FineCoord> seeds{{2, 2, 2}, {3, 3, 3}};
  MultiSourceConnectivity search;
  search.reset(&g, seeds);
  ConnectivityBudget b;
  b.maxExpansions = 8;
  while (search.status() == ConnectivityStatus::Pending) {
    search.step(b);
  }
  expect(search.status() == ConnectivityStatus::ComponentsFound, "corner-only are separate");
  expect(search.result().components.size() + (search.result().hasRemainder ? 1u : 0u) == 2,
         "two components for corner touch");
}

void testBudgetInvariant() {
  DenseGridView g(12);
  for (int z = 1; z <= 10; ++z)
    for (int y = 1; y <= 10; ++y)
      for (int x = 1; x <= 10; ++x)
        g.set({x, y, z}, true);
  // Cut a thin wall at x=5 for yz in [1,10] except a tunnel — actually remove a plane to split.
  std::vector<FineCoord> removed;
  for (int z = 1; z <= 10; ++z) {
    for (int y = 1; y <= 10; ++y) {
      removed.push_back({5, y, z});
      g.set({5, y, z}, false);
    }
  }
  auto seeds = collectBoundarySeeds(g, removed);
  auto oracle = fullBfsComponents(g, seeds);

  auto runWithBudget = [&](uint64_t maxExp, uint32_t quota) {
    MultiSourceConnectivity search;
    search.reset(&g, seeds);
    ConnectivityBudget b;
    b.maxExpansions = maxExp;
    b.perRootQuota = quota;
    while (search.status() == ConnectivityStatus::Pending) {
      search.step(b);
    }
    return search;
  };

  MultiSourceConnectivity a = runWithBudget(1, 1);
  MultiSourceConnectivity b = runWithBudget(0, 64);
  expect(a.status() == b.status(), "budget=1 and unlimited same status");

  const auto solids = g.allSolid();
  // Rebuild searchable states for pairwise compare.
  MultiSourceConnectivity sa;
  sa.reset(&g, seeds);
  ConnectivityBudget tiny;
  tiny.maxExpansions = 1;
  tiny.perRootQuota = 1;
  while (sa.status() == ConnectivityStatus::Pending) {
    sa.step(tiny);
  }
  MultiSourceConnectivity sb;
  sb.reset(&g, seeds);
  ConnectivityBudget big;
  big.maxExpansions = 0;
  while (sb.status() == ConnectivityStatus::Pending) {
    sb.step(big);
  }
  expect(samePartition(oracle, sa, solids), "tiny budget partition matches oracle on visited");
  expect(samePartition(oracle, sb, solids), "unlimited partition matches oracle on visited");
}

void testRandomDifferential(uint32_t seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> nDist(6, 14);
  const int n = nDist(rng);
  DenseGridView g(n);

  // Random connected blob via DFS growth.
  std::uniform_int_distribution<int> coord(1, n - 2);
  FineCoord start{coord(rng), coord(rng), coord(rng)};
  g.set(start, true);
  std::vector<FineCoord> frontier{start};
  static const FineCoord dirs[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                    {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  std::uniform_int_distribution<int> dirPick(0, 5);
  const int target = n * n;
  while (static_cast<int>(g.allSolid().size()) < target && !frontier.empty()) {
    std::uniform_int_distribution<size_t> pick(0, frontier.size() - 1);
    const size_t fi = pick(rng);
    const FineCoord cur = frontier[fi];
    const FineCoord d = dirs[dirPick(rng)];
    const FineCoord npos{cur.x + d.x, cur.y + d.y, cur.z + d.z};
    if (!g.inBounds(npos) || g.isSolid(npos)) {
      if ((rng() % 5) == 0) {
        frontier.erase(frontier.begin() + static_cast<std::ptrdiff_t>(fi));
      }
      continue;
    }
    g.set(npos, true);
    frontier.push_back(npos);
  }

  auto solidsBefore = g.allSolid();
  if (solidsBefore.size() < 8) {
    return;
  }
  std::uniform_int_distribution<size_t> delPick(0, solidsBefore.size() - 1);
  const size_t delCount = std::max<size_t>(1, solidsBefore.size() / 20);
  std::vector<FineCoord> removed;
  removed.reserve(delCount);
  for (size_t i = 0; i < delCount; ++i) {
    const FineCoord p = solidsBefore[delPick(rng)];
    if (g.isSolid(p)) {
      g.set(p, false);
      removed.push_back(p);
    }
  }
  if (removed.empty()) {
    return;
  }

  auto seeds = collectBoundarySeeds(g, removed);
  if (seeds.size() < 2) {
    return;
  }
  auto oracle = fullBfsComponents(g, seeds);
  const auto solids = g.allSolid();

  for (uint64_t budget : {uint64_t{1}, uint64_t{3}, uint64_t{17}, uint64_t{0}}) {
    MultiSourceConnectivity search;
    search.reset(&g, seeds);
    ConnectivityBudget b;
    b.maxExpansions = budget;
    b.perRootQuota = 1 + static_cast<uint32_t>(rng() % 8);
    while (search.status() == ConnectivityStatus::Pending) {
      search.step(b);
    }
    expect(search.status() != ConnectivityStatus::Pending, "search completed");
    expect(samePartition(oracle, search, solids),
           "random differential seed=" + std::to_string(seed) + " budget=" + std::to_string(budget));
  }
}

}  // namespace

int main() {
  testSurfaceDigNoSplit();
  testBridgeCut();
  testCornerTouchNotConnected();
  testBudgetInvariant();
  for (uint32_t s = 1; s <= 40; ++s) {
    testRandomDifferential(s);
  }
  if (gFailures == 0) {
    std::cout << "voxel_connectivity_tests: all passed\n";
    return 0;
  }
  std::cerr << "voxel_connectivity_tests: " << gFailures << " failure(s)\n";
  return 1;
}
