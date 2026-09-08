#include "voxel/VoxelConnectivity.h"

#include <algorithm>
#include <queue>

namespace voxel {
namespace {

constexpr FineCoord kFaceNeighbors[6] = {
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
};

}  // namespace

std::vector<FineCoord> collectBoundarySeeds(const SolidView& view,
                                            const std::vector<FineCoord>& removed) {
  std::unordered_map<FineCoord, uint8_t, FineCoordHash> unique;
  unique.reserve(removed.size() * 4);
  for (const FineCoord& r : removed) {
    for (const FineCoord& d : kFaceNeighbors) {
      const FineCoord n{r.x + d.x, r.y + d.y, r.z + d.z};
      if (view.isSolid(n)) {
        unique.emplace(n, 1);
      }
    }
  }
  std::vector<FineCoord> seeds;
  seeds.reserve(unique.size());
  for (const auto& kv : unique) {
    seeds.push_back(kv.first);
  }
  return seeds;
}

std::unordered_map<FineCoord, uint32_t, FineCoordHash> fullBfsComponents(
    const SolidView& view, const std::vector<FineCoord>& seeds) {
  std::unordered_map<FineCoord, uint32_t, FineCoordHash> labels;
  uint32_t nextId = 0;
  for (const FineCoord& seed : seeds) {
    if (!view.isSolid(seed) || labels.find(seed) != labels.end()) {
      continue;
    }
    const uint32_t id = nextId++;
    std::queue<FineCoord> q;
    q.push(seed);
    labels.emplace(seed, id);
    while (!q.empty()) {
      const FineCoord cur = q.front();
      q.pop();
      for (const FineCoord& d : kFaceNeighbors) {
        const FineCoord n{cur.x + d.x, cur.y + d.y, cur.z + d.z};
        if (!view.isSolid(n) || labels.find(n) != labels.end()) {
          continue;
        }
        labels.emplace(n, id);
        q.push(n);
      }
    }
  }
  return labels;
}

void MultiSourceConnectivity::reset(const SolidView* view, std::vector<FineCoord> seeds) {
  view_ = view;
  seeds_ = std::move(seeds);
  groups_.clear();
  pages_.clear();
  result_ = ConnectivityResult{};
  result_.metrics.seedCount = seeds_.size();

  if (!view_ || seeds_.empty()) {
    result_.status = ConnectivityStatus::Empty;
    return;
  }

  groups_.reserve(seeds_.size());
  for (uint32_t i = 0; i < seeds_.size(); ++i) {
    SearchGroup g;
    g.parent = i;
    g.size = 1;
    g.frontier.push_back(seeds_[i]);
    g.voxelCount = 1;
    g.finalized = false;
    g.active = true;
    groups_.push_back(std::move(g));
    assignLabel(seeds_[i], i);
    ++result_.metrics.visitedFineCount;
  }

  if (seeds_.size() == 1) {
    result_.status = ConnectivityStatus::Connected;
    result_.connectedRoot = 0;
    return;
  }

  result_.status = ConnectivityStatus::Pending;
  checkTermination();
}

uint32_t MultiSourceConnectivity::findRoot(uint32_t label) const {
  while (label < groups_.size() && groups_[label].parent != label) {
    label = groups_[label].parent;
  }
  return label;
}

uint32_t MultiSourceConnectivity::findRootMutable(uint32_t label) {
  uint32_t root = label;
  while (root < groups_.size() && groups_[root].parent != root) {
    root = groups_[root].parent;
  }
  // Path compression.
  uint32_t cur = label;
  while (cur < groups_.size() && groups_[cur].parent != cur) {
    const uint32_t next = groups_[cur].parent;
    groups_[cur].parent = root;
    cur = next;
  }
  return root;
}

void MultiSourceConnectivity::mergeGroups(uint32_t a, uint32_t b) {
  a = findRootMutable(a);
  b = findRootMutable(b);
  if (a == b) {
    return;
  }
  if (groups_[a].finalized || groups_[b].finalized) {
    return;
  }
  // Union by size: keep larger as root.
  if (groups_[a].size < groups_[b].size) {
    std::swap(a, b);
  }
  groups_[b].parent = a;
  groups_[a].size += groups_[b].size;
  groups_[a].voxelCount += groups_[b].voxelCount;
  if (!groups_[b].frontier.empty()) {
    groups_[a].frontier.insert(groups_[a].frontier.end(), groups_[b].frontier.begin(),
                               groups_[b].frontier.end());
    groups_[b].frontier.clear();
    groups_[b].frontier.shrink_to_fit();
  }
  groups_[b].active = false;
  ++result_.metrics.mergedGroupCount;
}

uint32_t* MultiSourceConnectivity::labelPtr(FineCoord p, bool create) {
  const CoarseCoord c = fineToCoarse(p);
  auto it = pages_.find(c);
  if (it == pages_.end()) {
    if (!create) {
      return nullptr;
    }
    SearchPage page;
    page.fill(kUnvisitedLabel);
    it = pages_.emplace(c, page).first;
    ++result_.metrics.allocatedSearchPageCount;
  }
  return &it->second[fineIndexInCoarse(p)];
}

uint32_t MultiSourceConnectivity::lookupLabel(FineCoord p) const {
  const CoarseCoord c = fineToCoarse(p);
  const auto it = pages_.find(c);
  if (it == pages_.end()) {
    return kUnvisitedLabel;
  }
  return it->second[fineIndexInCoarse(p)];
}

void MultiSourceConnectivity::assignLabel(FineCoord p, uint32_t label) {
  uint32_t* slot = labelPtr(p, true);
  if (slot) {
    *slot = label;
  }
}

uint32_t MultiSourceConnectivity::labelAt(FineCoord p) const {
  const uint32_t raw = lookupLabel(p);
  if (raw == kUnvisitedLabel) {
    return kUnvisitedLabel;
  }
  return findRoot(raw);
}

void MultiSourceConnectivity::expandOne(uint32_t root) {
  root = findRootMutable(root);
  SearchGroup& g = groups_[root];
  if (!g.active || g.finalized || g.frontier.empty()) {
    return;
  }
  const FineCoord cur = g.frontier.back();
  g.frontier.pop_back();
  ++result_.metrics.expansionCount;

  for (const FineCoord& d : kFaceNeighbors) {
    const FineCoord n{cur.x + d.x, cur.y + d.y, cur.z + d.z};
    if (!view_->isSolid(n)) {
      continue;
    }
    const uint32_t existing = lookupLabel(n);
    if (existing == kUnvisitedLabel) {
      assignLabel(n, root);
      g.frontier.push_back(n);
      ++g.voxelCount;
      ++result_.metrics.visitedFineCount;
    } else {
      mergeGroups(root, existing);
      root = findRootMutable(root);
    }
  }
}

void MultiSourceConnectivity::tryFinalize(uint32_t root) {
  root = findRootMutable(root);
  SearchGroup& g = groups_[root];
  if (!g.active || g.finalized) {
    return;
  }
  if (!g.frontier.empty()) {
    return;
  }
  // Exhausted: only finalize if another active group still exists.
  if (activeRootCount() <= 1) {
    return;
  }
  g.finalized = true;
  g.active = false;
  FinalizedComponent comp;
  comp.rootLabel = root;
  comp.voxelCount = g.voxelCount;
  buildMasksForRoot(root, comp);
  result_.components.push_back(std::move(comp));
  ++result_.metrics.finalizedComponentCount;
}

uint32_t MultiSourceConnectivity::activeRootCount() const {
  uint32_t n = 0;
  for (uint32_t i = 0; i < groups_.size(); ++i) {
    if (groups_[i].parent == i && groups_[i].active && !groups_[i].finalized) {
      ++n;
    }
  }
  return n;
}

void MultiSourceConnectivity::buildMasksForRoot(uint32_t root, FinalizedComponent& out) const {
  root = findRoot(root);
  std::unordered_map<CoarseCoord, size_t, CoarseCoordHash> index;
  for (const auto& kv : pages_) {
    const CoarseCoord& c = kv.first;
    const SearchPage& page = kv.second;
    ComponentMask* mask = nullptr;
    for (uint32_t i = 0; i < kLabelsPerSearchPage; ++i) {
      const uint32_t raw = page[i];
      if (raw == kUnvisitedLabel) {
        continue;
      }
      if (findRoot(raw) != root) {
        continue;
      }
      if (!mask) {
        const size_t idx = out.masks.size();
        index.emplace(c, idx);
        out.masks.push_back(ComponentMask{});
        out.masks.back().coarse = c;
        mask = &out.masks.back();
      }
      mask->set(i);
    }
  }
}

void MultiSourceConnectivity::checkTermination() {
  if (result_.status != ConnectivityStatus::Pending) {
    return;
  }

  // Finalize any newly exhausted groups while >1 active remains.
  for (uint32_t i = 0; i < groups_.size(); ++i) {
    if (groups_[i].parent == i && groups_[i].active && !groups_[i].finalized &&
        groups_[i].frontier.empty()) {
      tryFinalize(i);
    }
  }

  const uint32_t active = activeRootCount();
  if (active == 0) {
    result_.status = ConnectivityStatus::ComponentsFound;
    result_.hasRemainder = false;
    return;
  }
  if (active == 1) {
    // If we already finalized other components, remaining group is remainder (no full traverse).
    if (!result_.components.empty()) {
      result_.status = ConnectivityStatus::ComponentsFound;
      result_.hasRemainder = true;
      result_.metrics.remainderEarlyExit = true;
      uint32_t root = kUnvisitedLabel;
      for (uint32_t i = 0; i < groups_.size(); ++i) {
        if (groups_[i].parent == i && groups_[i].active && !groups_[i].finalized) {
          root = i;
          break;
        }
      }
      result_.connectedRoot = root;
      return;
    }
    // No finalized components and one active root: all seeds merged => connected.
    // Need not exhaust the frontier.
    uint32_t root = kUnvisitedLabel;
    for (uint32_t i = 0; i < groups_.size(); ++i) {
      if (groups_[i].parent == i && groups_[i].active && !groups_[i].finalized) {
        root = i;
        break;
      }
    }
    result_.status = ConnectivityStatus::Connected;
    result_.connectedRoot = root;
    result_.hasRemainder = false;
  }
}

bool MultiSourceConnectivity::step(ConnectivityBudget budget) {
  if (!view_ || result_.status != ConnectivityStatus::Pending) {
    return result_.status != ConnectivityStatus::Pending;
  }

  const uint64_t limit =
      budget.maxExpansions == 0 ? UINT64_MAX : result_.metrics.expansionCount + budget.maxExpansions;
  const uint32_t quota = std::max(1u, budget.perRootQuota);

  while (result_.status == ConnectivityStatus::Pending && result_.metrics.expansionCount < limit) {
    bool anyWork = false;
    for (uint32_t i = 0; i < groups_.size(); ++i) {
      if (groups_[i].parent != i || !groups_[i].active || groups_[i].finalized) {
        continue;
      }
      uint32_t did = 0;
      while (did < quota && result_.metrics.expansionCount < limit) {
        const uint32_t root = findRootMutable(i);
        if (root != i || !groups_[root].active || groups_[root].finalized ||
            groups_[root].frontier.empty()) {
          break;
        }
        expandOne(root);
        ++did;
        anyWork = true;
      }
    }
    checkTermination();
    if (!anyWork) {
      // All frontiers empty but still pending: finalize remaining empties.
      checkTermination();
      if (result_.status == ConnectivityStatus::Pending && activeRootCount() <= 1) {
        checkTermination();
      }
      if (result_.status == ConnectivityStatus::Pending && activeRootCount() == 0) {
        result_.status = ConnectivityStatus::ComponentsFound;
      }
      break;
    }
  }
  return result_.status != ConnectivityStatus::Pending;
}

}  // namespace voxel
