#include "physics/VoxelCollide.h"

#include "scene/VoxelScene.h"

#include <algorithm>
#include <cmath>
#include <vector>

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
                        const glm::ivec3& hitFine, const glm::vec3& fromWorld, int& nFace) {
  const int n = finePerAxis(o);
  const glm::mat4 o2w = o.objectToWorld();
  const glm::vec3 hitW = glm::vec3(o2w * glm::vec4(fineCenterLocal(o, hitFine), 1.0f));
  const glm::vec3 towardA = fromWorld - hitW;
  glm::vec3 best(0.0f, 1.0f, 0.0f);
  nFace = 2;
  float bestDot = -1.0e30f;
  bool anyEmpty = false;
  for (int i = 0; i < 6; ++i) {
    const glm::ivec3& fn = kFaceN[i];
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
      nFace = i;
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
  c.n = contactNormal(scene, b.shapeIndex, ob, hitFine, worldP, c.nFace);
  c.rA = worldP - a.x;
  c.rB = worldP - b.x;
  const glm::vec3 cB =
      glm::vec3(ob.objectToWorld() * glm::vec4(fineCenterLocal(ob, hitFine), 1.0f));
  // Two spheres of radius r: faces flush when centers are 2r apart (d == 0).
  // d < 0 is a gap (speculative); d > 0 is overlap. Keep look-ahead contacts so the
  // solver can limit approach speed; drop hits beyond the 6-neighbor fine range.
  c.d = 2.0f * kSphereRadius - glm::dot(worldP - cB, c.n);
  if (c.d < -kContactLookAhead) {
    return;
  }
  c.fineA = packFine(pA.x, pA.y, pA.z, finePerAxis(oa));
  c.fineB = packFine(hitFine.x, hitFine.y, hitFine.z, finePerAxis(ob));
  out.push_back(c);
}

struct FineBox {
  glm::ivec3 mn{0};
  glm::ivec3 mx{-1};

  bool contains(const glm::ivec3& p) const {
    return p.x >= mn.x && p.x <= mx.x && p.y >= mn.y && p.y <= mx.y && p.z >= mn.z && p.z <= mx.z;
  }
};

FineBox makeFineBox(const VoxelObject& o, const glm::vec3& lmn, const glm::vec3& lmx) {
  FineBox fb;
  if (lmn.x > lmx.x || lmn.y > lmx.y || lmn.z > lmx.z) {
    return fb;
  }
  const float s = fineSize(o);
  if (s <= 1e-8f) {
    return fb;
  }
  const int n = finePerAxis(o);
  fb.mn = glm::ivec3(static_cast<int>(std::floor(lmn.x / s)), static_cast<int>(std::floor(lmn.y / s)),
                     static_cast<int>(std::floor(lmn.z / s)));
  fb.mx = glm::ivec3(static_cast<int>(std::floor((lmx.x - 1e-6f) / s)),
                     static_cast<int>(std::floor((lmx.y - 1e-6f) / s)),
                     static_cast<int>(std::floor((lmx.z - 1e-6f) / s)));
  fb.mn = glm::max(fb.mn, glm::ivec3(0));
  fb.mx = glm::min(fb.mx, glm::ivec3(n - 1));
  return fb;
}

void testList(const VoxelScene& scene, const RigidBody& a, const RigidBody& b,
              const VoxelObject& oa, const VoxelObject& ob, const ShapeClass& ca,
              const std::vector<uint32_t>& ids, const FineBox& box, std::vector<Contact>& out) {
  for (uint32_t id : ids) {
    const glm::ivec3 p = unpackFine(id, ca.fineN);
    if (!box.contains(p)) {
      continue;
    }
    considerContact(scene, a, b, oa, ob, p, out);
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

glm::vec3 arbitraryPerp(const glm::vec3& n) {
  glm::vec3 t;
  if (n.x >= 0.57735f) {
    t = glm::vec3(n.y, -n.x, 0.0f);
  } else {
    t = glm::vec3(0.0f, n.z, -n.y);
  }
  const float len = glm::length(t);
  if (len <= 1e-8f) {
    return glm::vec3(1.0f, 0.0f, 0.0f);
  }
  return t / len;
}

// Box3D b3ReduceManifoldPoints (erincatto/box3d src/convex_manifold.c).
void reduceBucket(std::vector<Contact>& points) {
  if (points.size() <= 4) {
    return;
  }
  const glm::vec3 n = points[0].n;
  const float bias = 0.95f;
  const float tolSqr = kSlop * kSlop;
  std::vector<Contact> work = points;
  std::vector<Contact> kept;
  kept.reserve(4);

  const glm::vec3 searchDir = arbitraryPerp(n);
  int bestIndex = -1;
  float bestScore = -1.0e30f;
  for (int i = 0; i < static_cast<int>(work.size()); ++i) {
    if (work[static_cast<size_t>(i)].d < -kSlop) {
      continue;
    }
    const float score =
        work[static_cast<size_t>(i)].d + glm::dot(searchDir, work[static_cast<size_t>(i)].p);
    if (bias * score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }
  if (bestIndex < 0) {
    bestIndex = 0;
    for (int i = 1; i < static_cast<int>(work.size()); ++i) {
      if (work[static_cast<size_t>(i)].d > work[static_cast<size_t>(bestIndex)].d) {
        bestIndex = i;
      }
    }
  }
  kept.push_back(work[static_cast<size_t>(bestIndex)]);
  work[static_cast<size_t>(bestIndex)] = work.back();
  work.pop_back();
  const glm::vec3 a = kept[0].p;

  bestScore = 0.0f;
  bestIndex = -1;
  for (int i = 0; i < static_cast<int>(work.size()); ++i) {
    const glm::vec3 delta = work[static_cast<size_t>(i)].p - a;
    const glm::vec3 v = delta - glm::dot(delta, n) * n;
    const float distSqr = glm::dot(v, v);
    const float sep = std::max(0.0f, work[static_cast<size_t>(i)].d);
    const float score = distSqr + 4.0f * sep * sep;
    if (bias * score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }
  if (bestIndex < 0 || bestScore < tolSqr) {
    points.swap(kept);
    return;
  }
  kept.push_back(work[static_cast<size_t>(bestIndex)]);
  work[static_cast<size_t>(bestIndex)] = work.back();
  work.pop_back();
  const glm::vec3 bpt = kept[1].p;

  bestScore = tolSqr;
  bestIndex = -1;
  float bestSignedArea = 0.0f;
  const glm::vec3 ba = bpt - a;
  for (int i = 0; i < static_cast<int>(work.size()); ++i) {
    const float signedArea =
        glm::dot(n, glm::cross(ba, work[static_cast<size_t>(i)].p - a));
    const float score = std::abs(signedArea);
    if (bias * score >= bestScore) {
      bestScore = score;
      bestIndex = i;
      bestSignedArea = signedArea;
    }
  }
  if (bestIndex < 0) {
    points.swap(kept);
    return;
  }
  kept.push_back(work[static_cast<size_t>(bestIndex)]);
  work[static_cast<size_t>(bestIndex)] = work.back();
  work.pop_back();
  const glm::vec3 cpt = kept[2].p;

  bestScore = tolSqr;
  bestIndex = -1;
  const float sign = bestSignedArea < 0.0f ? -1.0f : 1.0f;
  for (int i = 0; i < static_cast<int>(work.size()); ++i) {
    const glm::vec3 p = work[static_cast<size_t>(i)].p;
    const float u1 = sign * glm::dot(n, glm::cross(p - a, ba));
    const float u2 = sign * glm::dot(n, glm::cross(p - bpt, cpt - bpt));
    const float u3 = sign * glm::dot(n, glm::cross(p - cpt, a - cpt));
    const float score = std::max(u1, std::max(u2, u3));
    if (bias * score > bestScore) {
      bestScore = score;
      bestIndex = i;
    }
  }
  if (bestIndex >= 0) {
    kept.push_back(work[static_cast<size_t>(bestIndex)]);
  }
  points.swap(kept);
}

void reducePairContacts(std::vector<Contact>& out, size_t before) {
  if (out.size() <= before) {
    return;
  }
  std::vector<Contact> src(out.begin() + static_cast<std::ptrdiff_t>(before), out.end());
  out.resize(before);
  std::vector<char> done(src.size(), 0);
  for (size_t i = 0; i < src.size(); ++i) {
    if (done[i]) {
      continue;
    }
    std::vector<Contact> bucket;
    bucket.push_back(src[i]);
    done[i] = 1;
    for (size_t j = i + 1; j < src.size(); ++j) {
      if (done[j]) {
        continue;
      }
      if (src[j].b == src[i].b && src[j].nFace == src[i].nFace) {
        bucket.push_back(src[j]);
        done[j] = 1;
      }
    }
    reduceBucket(bucket);
    out.insert(out.end(), bucket.begin(), bucket.end());
  }
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
  if (amn.x - pad > bmx.x + pad || amx.x + pad < bmn.x - pad || amn.y - pad > bmx.y + pad ||
      amx.y + pad < bmn.y - pad || amn.z - pad > bmx.z + pad || amx.z + pad < bmn.z - pad) {
    return;
  }

  glm::vec3 bInA_mn, bInA_mx, aInB_mn, aInB_mx;
  transformAabb(oa.worldToObject(), bmn, bmx, bInA_mn, bInA_mx);
  transformAabb(ob.worldToObject(), amn, amx, aInB_mn, aInB_mx);
  const FineBox boxA = makeFineBox(oa, bInA_mn - pad, bInA_mx + pad);
  const FineBox boxB = makeFineBox(ob, aInB_mn - pad, aInB_mx + pad);

  const size_t before = out.size();
  testList(scene, a, b, oa, ob, ca, ca.corners, boxA, out);
  testList(scene, a, b, oa, ob, ca, ca.edges, boxA, out);
  testList(scene, b, a, ob, oa, cb, cb.corners, boxB, out);
  testList(scene, b, a, ob, oa, cb, cb.edges, boxB, out);
  reducePairContacts(out, before);
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
    int nFace = 2;
    dn.n = contactNormal(scene, againstObj, ob, hitFine, dn.p, nFace);
    const glm::vec3 cB =
        glm::vec3(ob.objectToWorld() * glm::vec4(fineCenterLocal(ob, hitFine), 1.0f));
    dn.d = 2.0f * kSphereRadius - glm::dot(dn.p - cB, dn.n);
    out.push_back(dn);
  }
}

}  // namespace physics
