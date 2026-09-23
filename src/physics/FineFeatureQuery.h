#pragma once

#include "physics/PhysicsTypes.h"

namespace physics {

// IDs are sorted x + n*y + n*n*z. Walk populated rows, jumping directly over
// absent rows/slices. Emits exactly the old z/y/x order without testing empty
// rows in a large sparse overlap box. Returns row visits for regression tests.
template<class Visit>
int queryFineFeatures(const std::vector<uint32_t>& ids, int n, glm::ivec3 mn, glm::ivec3 mx, Visit&& visit) {
  if (ids.empty() || n <= 0 || mx.x < mn.x || mx.y < mn.y || mx.z < mn.z) return 0;
  auto it = std::lower_bound(ids.begin(), ids.end(), packFine(mn.x, mn.y, mn.z, n));
  const uint32_t end = packFine(mx.x, mx.y, mx.z, n);
  int rows = 0;
  while (it != ids.end() && *it <= end) {
    const glm::ivec3 p = unpackFine(*it, n);
    ++rows;
    if (p.y < mn.y) {
      it = std::lower_bound(it, ids.end(), packFine(mn.x, mn.y, p.z, n));
      continue;
    }
    if (p.y > mx.y) {
      if (p.z == mx.z) break;
      it = std::lower_bound(it, ids.end(), packFine(mn.x, mn.y, p.z + 1, n));
      continue;
    }
    const uint32_t lo = packFine(mn.x, p.y, p.z, n);
    const uint32_t hi = packFine(mx.x, p.y, p.z, n);
    it = std::lower_bound(it, ids.end(), lo);
    for (; it != ids.end() && *it <= hi; ++it) visit(*it);
    // Advance to the next admissible row, not each intervening empty row.
    if (p.y == mx.y && p.z == mx.z) break;
    const uint32_t next = p.y == mx.y ? packFine(mn.x, mn.y, p.z + 1, n)
                                         : packFine(mn.x, p.y + 1, p.z, n);
    it = std::lower_bound(it, ids.end(), next);
  }
  return rows;
}
} // namespace physics
