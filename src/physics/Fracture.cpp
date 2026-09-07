#include "physics/Fracture.h"

#include "scene/VoxelScene.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

constexpr int kFinePerCoarse = VoxelScene::kFinePerCoarse;
constexpr int kFineRes = VoxelScene::kFineRes;
constexpr int kMicroRes = VoxelScene::kMicroRes;
constexpr uint32_t kEmptyKey = 0xFFFFFFFFu;

const glm::ivec3 kFace[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                             {0, 0, 1}, {0, 0, -1}};

uint32_t packFine(int x, int y, int z) {
  return static_cast<uint32_t>(x) | (static_cast<uint32_t>(y) << 10) |
         (static_cast<uint32_t>(z) << 20);
}

glm::ivec3 unpackFine(uint32_t p) {
  return glm::ivec3(static_cast<int>(p & 1023u), static_cast<int>((p >> 10) & 1023u),
                    static_cast<int>((p >> 20) & 1023u));
}

struct UnionFind {
  std::vector<int> parent;
  std::vector<int> rank;
  std::vector<int> queueCount;

  void reset(int n) {
    parent.resize(static_cast<size_t>(n));
    rank.assign(static_cast<size_t>(n), 0);
    queueCount.assign(static_cast<size_t>(n), 0);
    for (int i = 0; i < n; ++i) {
      parent[static_cast<size_t>(i)] = i;
    }
  }

  int find(int x) {
    int r = x;
    while (parent[static_cast<size_t>(r)] != r) {
      r = parent[static_cast<size_t>(r)];
    }
    while (parent[static_cast<size_t>(x)] != r) {
      const int n = parent[static_cast<size_t>(x)];
      parent[static_cast<size_t>(x)] = r;
      x = n;
    }
    return r;
  }

  bool unite(int a, int b) {
    a = find(a);
    b = find(b);
    if (a == b) {
      return false;
    }
    if (rank[static_cast<size_t>(a)] < rank[static_cast<size_t>(b)]) {
      std::swap(a, b);
    }
    parent[static_cast<size_t>(b)] = a;
    queueCount[static_cast<size_t>(a)] += queueCount[static_cast<size_t>(b)];
    queueCount[static_cast<size_t>(b)] = 0;
    if (rank[static_cast<size_t>(a)] == rank[static_cast<size_t>(b)]) {
      ++rank[static_cast<size_t>(a)];
    }
    return true;
  }
};

struct FineVisit {
  std::vector<uint32_t> key;
  std::vector<int> lab;
  uint32_t mask = 0;
  uint32_t used = 0;

  void reset(uint32_t minCap) {
    uint32_t cap = 64;
    while (cap < minCap) {
      cap <<= 1;
    }
    key.assign(cap, kEmptyKey);
    lab.assign(cap, -1);
    mask = cap - 1u;
    used = 0;
  }

  void grow() {
    std::vector<uint32_t> oldKey = std::move(key);
    std::vector<int> oldLab = std::move(lab);
    const uint32_t cap = static_cast<uint32_t>(oldKey.size()) << 1;
    key.assign(cap, kEmptyKey);
    lab.assign(cap, -1);
    mask = cap - 1u;
    used = 0;
    for (size_t i = 0; i < oldKey.size(); ++i) {
      if (oldKey[i] == kEmptyKey) {
        continue;
      }
      uint32_t s = oldKey[i] * 0x9e3779b1u;
      for (;;) {
        s &= mask;
        if (key[s] == kEmptyKey) {
          key[s] = oldKey[i];
          lab[s] = oldLab[i];
          ++used;
          break;
        }
        ++s;
      }
    }
  }

  bool insertOrGet(uint32_t packed, int newLab, int* outLab) {
    if (key.empty()) {
      reset(64);
    }
    if (used * 2u > mask) {
      grow();
    }
    uint32_t s = packed * 0x9e3779b1u;
    for (;;) {
      s &= mask;
      if (key[s] == kEmptyKey) {
        key[s] = packed;
        lab[s] = newLab;
        *outLab = newLab;
        ++used;
        return true;
      }
      if (key[s] == packed) {
        *outLab = lab[s];
        return false;
      }
      ++s;
    }
  }
};

int nextGridN(int coarseExtent) {
  if (coarseExtent <= 8) {
    return 8;
  }
  if (coarseExtent <= 16) {
    return 16;
  }
  if (coarseExtent <= 32) {
    return 32;
  }
  return 64;
}

}  // namespace

bool VoxelScene::solidAbsFine(const VoxelObject& o, const glm::ivec3& absFine) const {
  const int nFine = o.gridSize * kFinePerCoarse;
  if (absFine.x < 0 || absFine.y < 0 || absFine.z < 0 || absFine.x >= nFine ||
      absFine.y >= nFine || absFine.z >= nFine) {
    return false;
  }
  const glm::ivec3 c = absFine / kFinePerCoarse;
  const glm::ivec3 rem = absFine - c * kFinePerCoarse;
  const glm::ivec3 m = rem / kFineRes;
  const glm::ivec3 f = rem - m * kFineRes;
  return getFine(o, c, m, f);
}

void VoxelScene::clearPackedFines(VoxelObject& o, const std::vector<uint32_t>& packedFines) {
  for (uint32_t p : packedFines) {
    const glm::ivec3 abs = unpackFine(p);
    const glm::ivec3 c = abs / kFinePerCoarse;
    const glm::ivec3 rem = abs - c * kFinePerCoarse;
    const glm::ivec3 m = rem / kFineRes;
    const glm::ivec3 f = rem - m * kFineRes;
    setFineCpu(o, c, m, f, false);
  }
}

void VoxelScene::emitFracturePiece(int srcIndex, const std::vector<uint32_t>& packedFines) {
  if (srcIndex < 0 || srcIndex >= static_cast<int>(objects_.size()) || packedFines.empty()) {
    return;
  }
  if (static_cast<uint32_t>(objects_.size()) >= kMaxShapes) {
    clearPackedFines(objects_[static_cast<size_t>(srcIndex)], packedFines);
    return;
  }
  if (packedFines.size() < physics::kResidualFineLimit) {
    clearPackedFines(objects_[static_cast<size_t>(srcIndex)], packedFines);
    return;
  }

  VoxelObject& src = objects_[static_cast<size_t>(srcIndex)];
  glm::ivec3 mn(1024);
  glm::ivec3 mx(-1);
  for (uint32_t p : packedFines) {
    const glm::ivec3 abs = unpackFine(p);
    mn = glm::min(mn, abs);
    mx = glm::max(mx, abs);
  }
  const glm::ivec3 c0 = mn / kFinePerCoarse;
  const glm::ivec3 c1 = mx / kFinePerCoarse;
  const glm::ivec3 coarseExt = c1 - c0 + 1;
  const int newN = nextGridN(std::max({coarseExt.x, coarseExt.y, coarseExt.z}));

  VoxelObject dst;
  dst.voxelSize = src.voxelSize;
  dst.gridSize = newN;
  dst.nestedMicro = src.nestedMicro;
  dst.editable = true;
  dst.enabled = true;
  dst.useImportPalette = src.useImportPalette;
  dst.rotation = src.rotation;
  dst.cells.assign(static_cast<size_t>(newN) * static_cast<size_t>(newN) * static_cast<size_t>(newN),
                   CoarseCell{});

  const glm::vec3 newCornerOldLocal = glm::vec3(c0) * src.voxelSize;
  const glm::vec3 newCenterOldLocal =
      newCornerOldLocal + 0.5f * static_cast<float>(newN) * src.voxelSize;
  dst.position = glm::vec3(src.objectToWorld() * glm::vec4(newCenterOldLocal, 1.0f));

  struct CoarseBucket {
    glm::ivec3 oldC{0};
    uint32_t count = 0;
  };
  std::vector<CoarseBucket> buckets;
  buckets.reserve(packedFines.size() / 16 + 1);
  auto bucketOf = [&](const glm::ivec3& oldC) -> CoarseBucket& {
    for (CoarseBucket& b : buckets) {
      if (b.oldC == oldC) {
        return b;
      }
    }
    buckets.push_back(CoarseBucket{oldC, 0});
    return buckets.back();
  };
  for (uint32_t p : packedFines) {
    ++bucketOf(unpackFine(p) / kFinePerCoarse).count;
  }

  auto solidInCoarse = [&](const VoxelObject& o, const glm::ivec3& c) -> uint32_t {
    if (!inBounds(o, c) || o.cells.empty()) {
      return 0;
    }
    const CoarseCell& cell = cellAt(o, indexOf(o, c));
    if (cell.material == 0u) {
      return 0;
    }
    if (cell.brickPage == kInvalidBrickPage) {
      return static_cast<uint32_t>(kFinePerBrick);
    }
    uint32_t n = 0;
    for (int i = 0; i < kFineTableBytes; ++i) {
      n += static_cast<uint32_t>(
          std::popcount(static_cast<unsigned>(readFineByte(cell.brickPage, static_cast<uint32_t>(i)))));
    }
    return n;
  };

  for (const CoarseBucket& b : buckets) {
    const glm::ivec3 nc = b.oldC - c0;
    if (nc.x < 0 || nc.y < 0 || nc.z < 0 || nc.x >= newN || nc.y >= newN || nc.z >= newN) {
      continue;
    }
    const uint32_t remain = solidInCoarse(src, b.oldC);
    if (remain == 0u) {
      continue;
    }
    const uint32_t dstIdx = indexOf(dst, nc);
    CoarseCell& old = cellAt(src, indexOf(src, b.oldC));
    if (b.count >= remain) {
      // Whole remaining coarse moves, including virtual-full pages.
      dst.cells[dstIdx] = old;
      old.material = 0;
      old.brickPage = kInvalidBrickPage;
    } else {
      // Split coarse: dest never inherits INVALID (that would ghost a 16³ cube).
      dst.cells[dstIdx].material = old.material == 0u ? 1u : old.material;
      uint32_t empty16[16] = {};
      dst.cells[dstIdx].brickPage = allocBrickPage(empty16);
    }
  }

  for (uint32_t p : packedFines) {
    const glm::ivec3 abs = unpackFine(p);
    const glm::ivec3 oldC = abs / kFinePerCoarse;
    const glm::ivec3 rem = abs - oldC * kFinePerCoarse;
    const glm::ivec3 m = rem / kFineRes;
    const glm::ivec3 f = rem - m * kFineRes;
    const glm::ivec3 nc = oldC - c0;
    if (nc.x < 0 || nc.y < 0 || nc.z < 0 || nc.x >= newN || nc.y >= newN || nc.z >= newN) {
      setFineCpu(src, oldC, m, f, false);
      continue;
    }
    CoarseCell& dstCell = cellAt(dst, indexOf(dst, nc));
    CoarseCell& srcCell = cellAt(src, indexOf(src, oldC));
    if (dstCell.material != 0u && dstCell.brickPage != kInvalidBrickPage &&
        dstCell.brickPage != srcCell.brickPage) {
      uint32_t rgbWord = 0;
      if (srcCell.material != 0u && srcCell.brickPage != kInvalidBrickPage) {
        rgbWord = brickPageWords(srcCell.brickPage)[static_cast<uint32_t>(kFineColorOffset) +
                                                    fineColorIndex(m, f)];
      }
      setFineCpu(dst, nc, m, f, true);
      if (rgbWord != 0u && dstCell.brickPage != kInvalidBrickPage) {
        brickPageWords(dstCell.brickPage)[static_cast<uint32_t>(kFineColorOffset) +
                                          fineColorIndex(m, f)] = rgbWord;
        dirtyPages_.insert(dstCell.brickPage);
      }
    }
    setFineCpu(src, oldC, m, f, false);
  }

  objects_.push_back(std::move(dst));
  const int dstIndex = static_cast<int>(objects_.size()) - 1;
  physics_.onSplit(srcIndex, dstIndex, objects_.back().position);
}

bool VoxelScene::maybeFracture(int objectIndex, const std::vector<glm::ivec3>& deletedAbsFines) {
  if (objectIndex < 0 || objectIndex >= static_cast<int>(objects_.size()) ||
      deletedAbsFines.empty()) {
    return false;
  }
  VoxelObject& o = objects_[static_cast<size_t>(objectIndex)];
  if (!o.enabled || o.cells.empty() || o.gridSize <= 0) {
    return false;
  }
  const int nFine = o.gridSize * kFinePerCoarse;
  if (nFine > 1024) {
    return false;
  }

  FineVisit visit;
  visit.reset(64);
  std::vector<uint32_t> seeds;
  seeds.reserve(deletedAbsFines.size() * 6);
  for (const glm::ivec3& d : deletedAbsFines) {
    for (const glm::ivec3& dir : kFace) {
      const glm::ivec3 n = d + dir;
      if (!solidAbsFine(o, n)) {
        continue;
      }
      const uint32_t packed = packFine(n.x, n.y, n.z);
      int lab = 0;
      if (visit.insertOrGet(packed, static_cast<int>(seeds.size()), &lab)) {
        seeds.push_back(packed);
      }
    }
  }
  if (seeds.size() <= 1) {
    return false;
  }

  UnionFind uf;
  uf.reset(static_cast<int>(seeds.size()));
  std::vector<std::vector<uint32_t>> cells(seeds.size());
  std::vector<uint32_t> qPos;
  std::vector<int> qLab;
  qPos.reserve(256);
  qLab.reserve(256);
  for (int i = 0; i < static_cast<int>(seeds.size()); ++i) {
    qPos.push_back(seeds[static_cast<size_t>(i)]);
    qLab.push_back(i);
    uf.queueCount[static_cast<size_t>(i)] = 1;
    cells[static_cast<size_t>(i)].push_back(seeds[static_cast<size_t>(i)]);
  }

  std::vector<uint8_t> coarseJumped(
      static_cast<size_t>(o.gridSize) * static_cast<size_t>(o.gridSize) *
          static_cast<size_t>(o.gridSize),
      0);

  auto enqueue = [&](uint32_t packed, int lab) {
    lab = uf.find(lab);
    int existing = -1;
    if (!visit.insertOrGet(packed, lab, &existing)) {
      uf.unite(lab, existing);
      return;
    }
    qPos.push_back(packed);
    qLab.push_back(lab);
    ++uf.queueCount[static_cast<size_t>(lab)];
    cells[static_cast<size_t>(lab)].push_back(packed);
  };

  size_t head = 0;
  std::vector<int> finished;
  bool split = false;
  while (head < qPos.size()) {
    const uint32_t packed = qPos[head];
    int lab = uf.find(qLab[head]);
    ++head;
    --uf.queueCount[static_cast<size_t>(lab)];
    const glm::ivec3 p = unpackFine(packed);

    const glm::ivec3 c = p / kFinePerCoarse;
    if (inBounds(o, c)) {
      const uint32_t cidx = indexOf(o, c);
      const CoarseCell& cell = cellAt(o, cidx);
      if (cell.material != 0u && cell.brickPage == kInvalidBrickPage &&
          coarseJumped[cidx] == 0) {
        coarseJumped[cidx] = 1;
        const glm::ivec3 base = c * kFinePerCoarse;
        for (int z = 0; z < kFinePerCoarse; ++z) {
          for (int y = 0; y < kFinePerCoarse; ++y) {
            for (int x = 0; x < kFinePerCoarse; ++x) {
              const glm::ivec3 fp = base + glm::ivec3(x, y, z);
              const uint32_t pk = packFine(fp.x, fp.y, fp.z);
              int existing = -1;
              if (visit.insertOrGet(pk, lab, &existing)) {
                cells[static_cast<size_t>(lab)].push_back(pk);
              } else {
                uf.unite(lab, existing);
                lab = uf.find(lab);
              }
            }
          }
        }
        for (int face = 0; face < 6; ++face) {
          const glm::ivec3 d = kFace[face];
          glm::ivec3 nCoarse = c + d;
          if (!inBounds(o, nCoarse)) {
            continue;
          }
          for (int u = 0; u < kFinePerCoarse; ++u) {
            for (int v = 0; v < kFinePerCoarse; ++v) {
              glm::ivec3 fp = base;
              if (d.x == 1) {
                fp.x += kFinePerCoarse - 1;
                fp.y += u;
                fp.z += v;
              } else if (d.x == -1) {
                fp.y += u;
                fp.z += v;
              } else if (d.y == 1) {
                fp.y += kFinePerCoarse - 1;
                fp.x += u;
                fp.z += v;
              } else if (d.y == -1) {
                fp.x += u;
                fp.z += v;
              } else if (d.z == 1) {
                fp.z += kFinePerCoarse - 1;
                fp.x += u;
                fp.y += v;
              } else {
                fp.x += u;
                fp.y += v;
              }
              const glm::ivec3 nb(fp.x + d.x, fp.y + d.y, fp.z + d.z);
              if (solidAbsFine(o, nb)) {
                enqueue(packFine(nb.x, nb.y, nb.z), lab);
              }
            }
          }
        }
      } else {
        for (const glm::ivec3& d : kFace) {
          const glm::ivec3 n = p + d;
          if (!solidAbsFine(o, n)) {
            continue;
          }
          enqueue(packFine(n.x, n.y, n.z), lab);
        }
      }
    }

    lab = uf.find(lab);
    int active = 0;
    int roots = 0;
    for (int i = 0; i < static_cast<int>(seeds.size()); ++i) {
      if (uf.parent[static_cast<size_t>(i)] != i) {
        continue;
      }
      ++roots;
      if (uf.queueCount[static_cast<size_t>(i)] > 0) {
        ++active;
      }
    }
    if (roots <= 1) {
      split = false;
      break;
    }
    if (uf.queueCount[static_cast<size_t>(lab)] == 0) {
      bool already = false;
      for (int f : finished) {
        if (uf.find(f) == lab) {
          already = true;
          break;
        }
      }
      if (!already) {
        finished.push_back(lab);
        split = true;
      }
      if (active <= 1) {
        break;
      }
    }
  }

  if (!split) {
    return false;
  }

  bool changed = false;
  std::vector<uint8_t> emitted(seeds.size(), 0);
  for (int root : finished) {
    const int r = uf.find(root);
    if (r < 0 || r >= static_cast<int>(emitted.size()) || emitted[static_cast<size_t>(r)]) {
      continue;
    }
    emitted[static_cast<size_t>(r)] = 1;
    std::vector<uint32_t> piece;
    for (int i = 0; i < static_cast<int>(seeds.size()); ++i) {
      if (uf.find(i) == r) {
        piece.insert(piece.end(), cells[static_cast<size_t>(i)].begin(),
                     cells[static_cast<size_t>(i)].end());
      }
    }
    std::sort(piece.begin(), piece.end());
    piece.erase(std::unique(piece.begin(), piece.end()), piece.end());
    if (piece.empty()) {
      continue;
    }
    emitFracturePiece(objectIndex, piece);
    changed = true;
  }
  return changed;
}
