#include "physics/Classify.h"

#include "scene/VoxelScene.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace physics {
namespace {

int finePerAxis(const VoxelObject& o) { return o.gridSize * VoxelScene::kFinePerCoarse; }

bool occupiedFine(VoxelScene& scene, int objectIndex, int n, const glm::ivec3& p) {
  if (p.x < 0 || p.y < 0 || p.z < 0 || p.x >= n || p.y >= n || p.z >= n) {
    return false;
  }
  const int F = VoxelScene::kFinePerCoarse;
  const glm::ivec3 c(p.x / F, p.y / F, p.z / F);
  const glm::ivec3 rem = p - c * F;
  const glm::ivec3 m(rem.x / VoxelScene::kFineRes, rem.y / VoxelScene::kFineRes,
                     rem.z / VoxelScene::kFineRes);
  const glm::ivec3 f = rem - m * VoxelScene::kFineRes;
  return scene.occupancyFine(objectIndex, c, m, f);
}

void classifyFine(VoxelScene& scene, int objectIndex, int n, const glm::ivec3& p, ShapeClass& out) {
  int clamped = 0;
  for (int axis = 0; axis < 3; ++axis) {
    glm::ivec3 neg = p;
    glm::ivec3 pos = p;
    --neg[axis];
    ++pos[axis];
    if (occupiedFine(scene, objectIndex, n, neg) && occupiedFine(scene, objectIndex, n, pos)) {
      ++clamped;
    }
  }
  if (clamped >= 2) {
    return;
  }
  const uint32_t idx = packFine(p.x, p.y, p.z, n);
  if (clamped == 0) {
    out.corners.push_back(idx);
  } else {
    out.edges.push_back(idx);
  }
}

void emitAabbEdges(VoxelScene& scene, int objectIndex, int n, const glm::ivec3& mn,
                   const glm::ivec3& mx, ShapeClass& out) {
  const int ys[2] = {mn.y, mx.y};
  const int zs[2] = {mn.z, mx.z};
  const int xs[2] = {mn.x, mx.x};
  for (int iy = 0; iy < 2; ++iy) {
    for (int iz = 0; iz < 2; ++iz) {
      for (int x = mn.x; x <= mx.x; ++x) {
        classifyFine(scene, objectIndex, n, glm::ivec3(x, ys[iy], zs[iz]), out);
      }
    }
  }
  for (int ix = 0; ix < 2; ++ix) {
    for (int iz = 0; iz < 2; ++iz) {
      for (int y = mn.y; y <= mx.y; ++y) {
        classifyFine(scene, objectIndex, n, glm::ivec3(xs[ix], y, zs[iz]), out);
      }
    }
  }
  for (int ix = 0; ix < 2; ++ix) {
    for (int iy = 0; iy < 2; ++iy) {
      for (int z = mn.z; z <= mx.z; ++z) {
        classifyFine(scene, objectIndex, n, glm::ivec3(xs[ix], ys[iy], z), out);
      }
    }
  }
}

bool uniformSolidCoarse(VoxelScene& scene, int objectIndex, const glm::ivec3& c) {
  return scene.occupancyMaterial(objectIndex, c) != 0u &&
         scene.coarseBrickPage(objectIndex, c) == VoxelScene::kInvalidBrickPage;
}

glm::ivec3 unpackOccupiedCoarse(uint32_t packed) {
  return glm::ivec3(static_cast<int>(packed & 1023u), static_cast<int>((packed >> 10) & 1023u),
                    static_cast<int>((packed >> 20) & 1023u));
}

template <typename Fn>
void forEachOccupiedCoarse(VoxelScene& scene, int objectIndex, const VoxelObject& o, Fn&& fn) {
  if (!o.occupiedCoarses.empty()) {
    for (uint32_t packed : o.occupiedCoarses) {
      fn(unpackOccupiedCoarse(packed));
    }
    return;
  }
  for (int z = 0; z < o.gridSize; ++z) {
    for (int y = 0; y < o.gridSize; ++y) {
      for (int x = 0; x < o.gridSize; ++x) {
        const glm::ivec3 c(x, y, z);
        if (scene.occupancyMaterial(objectIndex, c) != 0u) {
          fn(c);
        }
      }
    }
  }
}

}  // namespace

void rebuildShapeClass(VoxelScene& scene, int objectIndex, ShapeClass& out) {
  const VoxelObject& o = scene.cpuObject(objectIndex);
  const int n = finePerAxis(o);
  out.fineN = n;
  out.corners.clear();
  out.edges.clear();
  if (n <= 0 || o.gridSize <= 0) {
    out.occValid = false;
    out.dirty = false;
    return;
  }

  glm::ivec3 mn(o.gridSize);
  glm::ivec3 mx(-1);
  bool anyBrick = false;
  int occupiedCoarse = 0;
  forEachOccupiedCoarse(scene, objectIndex, o, [&](const glm::ivec3& c) {
    ++occupiedCoarse;
    mn = glm::min(mn, c);
    mx = glm::max(mx, c);
    if (scene.coarseBrickPage(objectIndex, c) != VoxelScene::kInvalidBrickPage) {
      anyBrick = true;
    }
  });
  if (occupiedCoarse == 0) {
    out.occValid = false;
    out.dirty = false;
    return;
  }
  const int Fcls = VoxelScene::kFinePerCoarse;
  out.occFineMn = mn * Fcls;
  out.occFineMx = (mx + 1) * Fcls - glm::ivec3(1);
  out.occValid = true;

  // Static world: collidePair probes occupancy, it never iterates these lists.
  // Classifying 64×2×64 ground cubes here made every split O(ground).
  if (objectIndex == 0) {
    out.corners.clear();
    out.edges.clear();
    out.dirty = false;
    return;
  }

  bool solidBox = !anyBrick;
  if (solidBox) {
    for (int z = mn.z; z <= mx.z && solidBox; ++z) {
      for (int y = mn.y; y <= mx.y && solidBox; ++y) {
        for (int x = mn.x; x <= mx.x && solidBox; ++x) {
          if (!uniformSolidCoarse(scene, objectIndex, glm::ivec3(x, y, z))) {
            solidBox = false;
          }
        }
      }
    }
  }

  const int F = VoxelScene::kFinePerCoarse;
  auto uniqueIds = [](std::vector<uint32_t>& v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
  };

  if (solidBox) {
    const glm::ivec3 fmn = mn * F;
    const glm::ivec3 fmx = (mx + 1) * F - 1;
    emitAabbEdges(scene, objectIndex, n, fmn, fmx, out);
    uniqueIds(out.corners);
    uniqueIds(out.edges);
    out.dirty = false;
    return;
  }

  std::vector<glm::ivec3> fines;
  fines.reserve(64);
  forEachOccupiedCoarse(scene, objectIndex, o, [&](const glm::ivec3& c) {
    if (uniformSolidCoarse(scene, objectIndex, c)) {
      const glm::ivec3 f0 = c * F;
      emitAabbEdges(scene, objectIndex, n, f0, f0 + glm::ivec3(F - 1), out);
      return;
    }
    scene.collectOccupiedFines(objectIndex, c, fines);
    for (const glm::ivec3& p : fines) {
      classifyFine(scene, objectIndex, n, p, out);
    }
  });
  uniqueIds(out.corners);
  uniqueIds(out.edges);
  out.dirty = false;
}

void computeMassProperties(VoxelScene& scene, int objectIndex, RigidBody& body) {
  const VoxelObject& o = scene.cpuObject(objectIndex);
  const int F = VoxelScene::kFinePerCoarse;
  const float s = o.voxelSize / static_cast<float>(F);
  const double s3 = static_cast<double>(s) * static_cast<double>(s) * static_cast<double>(s);
  const double miFine = static_cast<double>(kDensityWood) * s3;
  const glm::vec3 gridCenter = 0.5f * static_cast<float>(o.gridSize) * o.voxelSize * glm::vec3(1.0f);

  double mass = 0.0;
  glm::dvec3 moment(0.0);
  uint32_t count = 0;
  std::vector<glm::ivec3> fines;
  fines.reserve(64);

  forEachOccupiedCoarse(scene, objectIndex, o, [&](const glm::ivec3& c) {
    if (scene.coarseBrickPage(objectIndex, c) == VoxelScene::kInvalidBrickPage) {
      const double mC = miFine * static_cast<double>(F * F * F);
      const glm::dvec3 p((static_cast<double>(c.x) + 0.5) * static_cast<double>(o.voxelSize),
                         (static_cast<double>(c.y) + 0.5) * static_cast<double>(o.voxelSize),
                         (static_cast<double>(c.z) + 0.5) * static_cast<double>(o.voxelSize));
      mass += mC;
      moment += mC * p;
      count += static_cast<uint32_t>(F * F * F);
    } else {
      scene.collectOccupiedFines(objectIndex, c, fines);
      for (const glm::ivec3& fp : fines) {
        const glm::dvec3 p((static_cast<double>(fp.x) + 0.5) * static_cast<double>(s),
                           (static_cast<double>(fp.y) + 0.5) * static_cast<double>(s),
                           (static_cast<double>(fp.z) + 0.5) * static_cast<double>(s));
        mass += miFine;
        moment += miFine * p;
        ++count;
      }
    }
  });

  body.occupiedFine = count;
  if (mass <= 1e-8 || !body.dynamic) {
    body.invM = 0.0f;
    body.comLocal = gridCenter;
    body.Iloc = glm::mat3(1.0f);
    body.IinvW = glm::mat3(0.0f);
    return;
  }

  const glm::dvec3 com = moment / mass;
  body.comLocal = glm::vec3(com);
  glm::dmat3 I(0.0);
  const double a = static_cast<double>(o.voxelSize);
  const double a2 = a * a;

  forEachOccupiedCoarse(scene, objectIndex, o, [&](const glm::ivec3& c) {
    if (scene.coarseBrickPage(objectIndex, c) == VoxelScene::kInvalidBrickPage) {
      const double mC = miFine * static_cast<double>(F * F * F);
      const glm::dvec3 p((static_cast<double>(c.x) + 0.5) * static_cast<double>(o.voxelSize),
                         (static_cast<double>(c.y) + 0.5) * static_cast<double>(o.voxelSize),
                         (static_cast<double>(c.z) + 0.5) * static_cast<double>(o.voxelSize));
      const glm::dvec3 r = p - com;
      const double r2 = glm::dot(r, r);
      const double ic = mC * a2 / 6.0;
      I[0][0] += ic + mC * (r2 - r.x * r.x);
      I[1][1] += ic + mC * (r2 - r.y * r.y);
      I[2][2] += ic + mC * (r2 - r.z * r.z);
      I[0][1] -= mC * r.x * r.y;
      I[1][0] -= mC * r.x * r.y;
      I[0][2] -= mC * r.x * r.z;
      I[2][0] -= mC * r.x * r.z;
      I[1][2] -= mC * r.y * r.z;
      I[2][1] -= mC * r.y * r.z;
    } else {
      scene.collectOccupiedFines(objectIndex, c, fines);
      for (const glm::ivec3& fp : fines) {
        const glm::dvec3 p((static_cast<double>(fp.x) + 0.5) * static_cast<double>(s),
                           (static_cast<double>(fp.y) + 0.5) * static_cast<double>(s),
                           (static_cast<double>(fp.z) + 0.5) * static_cast<double>(s));
        const glm::dvec3 r = p - com;
        const double r2 = glm::dot(r, r);
        I[0][0] += miFine * (r2 - r.x * r.x);
        I[1][1] += miFine * (r2 - r.y * r.y);
        I[2][2] += miFine * (r2 - r.z * r.z);
        I[0][1] -= miFine * r.x * r.y;
        I[1][0] -= miFine * r.x * r.y;
        I[0][2] -= miFine * r.x * r.z;
        I[2][0] -= miFine * r.x * r.z;
        I[1][2] -= miFine * r.y * r.z;
        I[2][1] -= miFine * r.y * r.z;
      }
    }
  });

  const glm::vec3 newComW = glm::vec3(o.objectToWorld() * glm::vec4(body.comLocal, 1.0f));
  if (body.invM > 0.0f) {
    body.v += glm::cross(body.w, newComW - body.x);
  }
  body.x = newComW;

  body.invM = static_cast<float>(1.0 / mass);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      body.Iloc[i][j] = static_cast<float>(I[i][j]);
    }
    body.Iloc[i][i] = std::max(body.Iloc[i][i], 1e-4f);
  }
}

}  // namespace physics
