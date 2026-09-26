#include "blast/GroundAnchors.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace blast {
namespace {

float halfExtent(const AnchorGridFrame& f) { return 0.5f * static_cast<float>(f.fineN) * f.fineSize; }

glm::vec3 toWorld(const AnchorGridFrame& f, const glm::vec3& localMeters) {
  return f.position + f.rotation * (localMeters - glm::vec3(halfExtent(f)));
}

void worldBounds(const AnchorGridFrame& f, glm::vec3& lo, glm::vec3& hi) {
  const float e = static_cast<float>(f.fineN) * f.fineSize;
  lo = glm::vec3(std::numeric_limits<float>::max());
  hi = glm::vec3(-std::numeric_limits<float>::max());
  for (int c = 0; c < 8; ++c) {
    const glm::vec3 corner((c & 1) ? e : 0.0f, (c & 2) ? e : 0.0f, (c & 4) ? e : 0.0f);
    const glm::vec3 w = toWorld(f, corner);
    lo = glm::min(lo, w);
    hi = glm::max(hi, w);
  }
}

bool sourceSolidAt(const AnchorSource& s, const glm::vec3& world) {
  const glm::vec3 local = glm::inverse(s.frame.rotation) * (world - s.frame.position) + glm::vec3(halfExtent(s.frame));
  const glm::vec3 cell = glm::floor(local / s.frame.fineSize);
  const int n = s.frame.fineN;
  if (cell.x < 0.0f || cell.y < 0.0f || cell.z < 0.0f || cell.x >= n || cell.y >= n || cell.z >= n) {
    return false;
  }
  return s.solid(static_cast<int>(cell.x), static_cast<int>(cell.y), static_cast<int>(cell.z));
}

}  // namespace

GroundAnchorResult findGroundAnchors(const AnchorGridFrame& object, const FineSolidFn& solid,
                                     const std::vector<AnchorSource>& sources, float paddingFines) {
  GroundAnchorResult out;
  const int n = object.fineN;
  out.fineN = n;
  if (n <= 0 || !solid) {
    return out;
  }
  const size_t count = static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n);
  out.anchor.assign(count, 0);

  // Only sources overlapping the object's bounds (grown by the probe distance) can touch it.
  glm::vec3 objLo, objHi;
  worldBounds(object, objLo, objHi);
  const glm::vec3 grow((1.0f + paddingFines) * object.fineSize);
  std::vector<const AnchorSource*> nearby;
  for (const AnchorSource& s : sources) {
    if (s.frame.fineN <= 0 || !s.solid) {
      continue;
    }
    glm::vec3 lo, hi;
    worldBounds(s.frame, lo, hi);
    if (glm::all(glm::lessThanEqual(lo, objHi + grow)) && glm::all(glm::lessThanEqual(objLo - grow, hi))) {
      nearby.push_back(&s);
    }
  }
  if (nearby.empty()) {
    return out;
  }

  std::vector<uint8_t> occ(count, 0);
  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        occ[static_cast<size_t>(x + n * (y + n * z))] = solid(x, y, z) ? 1 : 0;
      }
    }
  }
  auto occupied = [&](int x, int y, int z) {
    return x >= 0 && y >= 0 && z >= 0 && x < n && y < n && z < n &&
           occ[static_cast<size_t>(x + n * (y + n * z))] != 0;
  };

  static constexpr int kDirs[6][3] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  const float reach = (0.5f + paddingFines) * object.fineSize;
  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        if (!occupied(x, y, z)) {
          continue;
        }
        const glm::vec3 center((static_cast<float>(x) + 0.5f) * object.fineSize,
                               (static_cast<float>(y) + 0.5f) * object.fineSize,
                               (static_cast<float>(z) + 0.5f) * object.fineSize);
        bool anchored = false;
        for (const auto& d : kDirs) {
          if (occupied(x + d[0], y + d[1], z + d[2])) {
            continue;
          }
          const glm::vec3 probe = toWorld(object, center + reach * glm::vec3(d[0], d[1], d[2]));
          bool contact = false;
          bool blocked = false;
          for (const AnchorSource* s : nearby) {
            if (!sourceSolidAt(*s, probe)) {
              continue;
            }
            if (s->blocksMount) {
              blocked = true;
            } else {
              contact = true;
            }
          }
          if (blocked) {
            ++out.blockedFaces;
          } else if (contact) {
            ++out.contactFaces;
            anchored = true;
          }
        }
        if (anchored) {
          out.anchor[static_cast<size_t>(x + n * (y + n * z))] = 1;
          ++out.anchorFines;
        }
      }
    }
  }
  return out;
}

}  // namespace blast
