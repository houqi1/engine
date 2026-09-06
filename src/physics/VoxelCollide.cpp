#include "physics/VoxelCollide.h"

#include "scene/VoxelScene.h"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

constexpr glm::ivec3 kFaceN[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0},
                                  {0, 0, 1}, {0, 0, -1}};

int finePerAxis(const VoxelObject& o) { return o.gridSize * VoxelScene::kFinePerCoarse; }

float fineSize(const VoxelObject& o) {
  return o.voxelSize / static_cast<float>(VoxelScene::kFinePerCoarse);
}

glm::vec3 fineCenterLocal(const VoxelObject& o, const glm::ivec3& p) {
  const float s = fineSize(o);
  return glm::vec3((static_cast<float>(p.x) + 0.5f) * s, (static_cast<float>(p.y) + 0.5f) * s,
                   (static_cast<float>(p.z) + 0.5f) * s);
}

bool occupiedAt(const VoxelScene& scene, int objectIndex, int n, const glm::ivec3& p) {
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

bool probeOccupancy(const VoxelScene& scene, int objectIndex, const VoxelObject& o,
                    const glm::vec3& worldP, glm::ivec3& hitFine) {
  const glm::vec3 localP = glm::vec3(o.worldToObject() * glm::vec4(worldP, 1.0f));
  const float s = fineSize(o);
  if (s <= 1e-8f) {
    return false;
  }
  const int n = finePerAxis(o);
  const glm::vec3 gf = localP / s;
  const glm::ivec3 base(static_cast<int>(std::floor(gf.x)), static_cast<int>(std::floor(gf.y)),
                        static_cast<int>(std::floor(gf.z)));
  const glm::ivec3 deltas[7] = {{0, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
  for (const glm::ivec3& d : deltas) {
    const glm::ivec3 p = base + d;
    if (occupiedAt(scene, objectIndex, n, p)) {
      hitFine = p;
      return true;
    }
  }
  return false;
}

glm::vec3 contactNormal(const VoxelScene& scene, int objectIndex, const VoxelObject& o,
                        const glm::ivec3& hitFine, const glm::vec3& fromWorld) {
  const int n = finePerAxis(o);
  const glm::mat4 o2w = o.objectToWorld();
  const glm::vec3 hitW = glm::vec3(o2w * glm::vec4(fineCenterLocal(o, hitFine), 1.0f));
  const glm::vec3 towardA = fromWorld - hitW;
  glm::vec3 best(0.0f, 1.0f, 0.0f);
  float bestDot = -1.0e30f;
  bool anyEmpty = false;
  for (const glm::ivec3& fn : kFaceN) {
    if (occupiedAt(scene, objectIndex, n, hitFine + fn)) {
      continue;
    }
    const glm::vec3 nLocal(static_cast<float>(fn.x), static_cast<float>(fn.y),
                           static_cast<float>(fn.z));
    const glm::vec3 nw = glm::normalize(glm::mat3(o2w) * nLocal);
    const float dn = glm::dot(nw, towardA);
    if (!anyEmpty || dn > bestDot) {
      best = nw;
      bestDot = dn;
      anyEmpty = true;
    }
  }
  // Empty-neighbor n already points out of the solid. Never flip it — after the
  // first hit the corner sits slightly below the cell center and a flip pushes down.
  if (anyEmpty) {
    return best;
  }
  return glm::vec3(0.0f, 1.0f, 0.0f);
}

void considerContact(const VoxelScene& scene, const RigidBody& a, const RigidBody& b,
                     const VoxelObject& oa, const VoxelObject& ob, const glm::ivec3& pA,
                     std::vector<Contact>& out) {
  const glm::vec3 worldP = glm::vec3(oa.objectToWorld() * glm::vec4(fineCenterLocal(oa, pA), 1.0f));
  glm::ivec3 hitFine(0);
  if (!probeOccupancy(scene, b.shapeIndex, ob, worldP, hitFine)) {
    return;
  }
  Contact c;
  c.a = a.shapeIndex;
  c.b = b.shapeIndex;
  c.p = worldP;
  c.n = contactNormal(scene, b.shapeIndex, ob, hitFine, worldP);
  c.rA = worldP - a.x;
  c.rB = worldP - b.x;
  const glm::vec3 cB =
      glm::vec3(ob.objectToWorld() * glm::vec4(fineCenterLocal(ob, hitFine), 1.0f));
  // Two spheres of radius r: rest along n is 2r. Occupancy already means hit;
  // negative d is a gap, not a miss.
  c.d = 2.0f * kSphereRadius - glm::dot(worldP - cB, c.n);
  out.push_back(c);
}

void testList(const VoxelScene& scene, const RigidBody& a, const RigidBody& b,
              const VoxelObject& oa, const VoxelObject& ob, const ShapeClass& ca,
              const std::vector<uint32_t>& ids, std::vector<Contact>& out) {
  for (uint32_t id : ids) {
    considerContact(scene, a, b, oa, ob, unpackFine(id, ca.fineN), out);
  }
}

void transformAabb(const glm::mat4& m, const glm::vec3& mn, const glm::vec3& mx, glm::vec3& wmn,
                   glm::vec3& wmx) {
  wmn = glm::vec3(1e30f);
  wmx = glm::vec3(-1e30f);
  for (int i = 0; i < 8; ++i) {
    const glm::vec3 c((i & 1) ? mx.x : mn.x, (i & 2) ? mx.y : mn.y, (i & 4) ? mx.z : mn.z);
    const glm::vec3 w = glm::vec3(m * glm::vec4(c, 1.0f));
    wmn = glm::min(wmn, w);
    wmx = glm::max(wmx, w);
  }
}

bool worldAabb(const VoxelScene& scene, int objectIndex, const ShapeClass& sc, glm::vec3& wmn,
               glm::vec3& wmx) {
  if (!sc.occValid) {
    return false;
  }
  const VoxelObject& o = scene.cpuObject(objectIndex);
  const float s = fineSize(o);
  const glm::vec3 lmn(static_cast<float>(sc.occFineMn.x) * s, static_cast<float>(sc.occFineMn.y) * s,
                      static_cast<float>(sc.occFineMn.z) * s);
  const glm::vec3 lmx((static_cast<float>(sc.occFineMx.x) + 1.0f) * s,
                      (static_cast<float>(sc.occFineMx.y) + 1.0f) * s,
                      (static_cast<float>(sc.occFineMx.z) + 1.0f) * s);
  transformAabb(o.objectToWorld(), lmn, lmx, wmn, wmx);
  return true;
}

}  // namespace

void collidePair(VoxelScene& scene, const RigidBody& a, const RigidBody& b, const ShapeClass& ca,
                 const ShapeClass& cb, std::vector<Contact>& out) {
  if (a.shapeIndex < 0 || b.shapeIndex < 0 || a.shapeIndex == b.shapeIndex) {
    return;
  }
  glm::vec3 amn, amx, bmn, bmx;
  if (!worldAabb(scene, a.shapeIndex, ca, amn, amx) ||
      !worldAabb(scene, b.shapeIndex, cb, bmn, bmx)) {
    return;
  }
  const VoxelObject& oa = scene.cpuObject(a.shapeIndex);
  const VoxelObject& ob = scene.cpuObject(b.shapeIndex);
  // 6-neighbor probe sees one fine cell outside the occupancy AABB.
  const float pad = std::max(fineSize(oa), fineSize(ob));
  amn -= pad;
  amx += pad;
  bmn -= pad;
  bmx += pad;
  if (amn.x > bmx.x || amx.x < bmn.x || amn.y > bmx.y || amx.y < bmn.y || amn.z > bmx.z ||
      amx.z < bmn.z) {
    return;
  }
  const size_t before = out.size();
  if (a.invM > 0.0f) {
    testList(scene, a, b, oa, ob, ca, ca.corners, out);
  }
  if (b.invM > 0.0f) {
    testList(scene, b, a, ob, oa, cb, cb.corners, out);
  }
  if (out.size() - before > static_cast<size_t>(kMaxContactsPerPair)) {
    std::partial_sort(out.begin() + static_cast<std::ptrdiff_t>(before),
                      out.begin() + static_cast<std::ptrdiff_t>(before + kMaxContactsPerPair),
                      out.end(), [](const Contact& l, const Contact& r) { return l.d > r.d; });
    out.resize(before + static_cast<size_t>(kMaxContactsPerPair));
  }
}

void gatherCornerNormals(const VoxelScene& scene, const ShapeClass& fromClass, int fromObj,
                         int againstObj, std::vector<DebugCornerNormal>& out) {
  out.clear();
  if (fromObj < 0 || againstObj < 0 || fromObj >= scene.cpuObjectCount() ||
      againstObj >= scene.cpuObjectCount()) {
    return;
  }
  const VoxelObject& oa = scene.cpuObject(fromObj);
  const VoxelObject& ob = scene.cpuObject(againstObj);
  out.reserve(fromClass.corners.size());
  for (uint32_t id : fromClass.corners) {
    const glm::ivec3 p = unpackFine(id, fromClass.fineN);
    DebugCornerNormal dn;
    dn.p = glm::vec3(oa.objectToWorld() * glm::vec4(fineCenterLocal(oa, p), 1.0f));
    glm::ivec3 hitFine(0);
    if (!probeOccupancy(scene, againstObj, ob, dn.p, hitFine)) {
      out.push_back(dn);
      continue;
    }
    dn.hit = true;
    dn.n = contactNormal(scene, againstObj, ob, hitFine, dn.p);
    const glm::vec3 cB =
        glm::vec3(ob.objectToWorld() * glm::vec4(fineCenterLocal(ob, hitFine), 1.0f));
    dn.d = 2.0f * kSphereRadius - glm::dot(dn.p - cB, dn.n);
    out.push_back(dn);
  }
}

}  // namespace physics
