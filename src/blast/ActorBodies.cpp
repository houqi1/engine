#include "blast/ActorBodies.h"

#include "NvBlast.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace blast {
namespace {

glm::mat3 boxInertia(float m, const glm::vec3& full) {
  const float xx = full.x * full.x;
  const float yy = full.y * full.y;
  const float zz = full.z * full.z;
  glm::mat3 I(0.0f);
  I[0][0] = m * (yy + zz) / 12.0f;
  I[1][1] = m * (xx + zz) / 12.0f;
  I[2][2] = m * (xx + yy) / 12.0f;
  return I;
}

glm::mat3 rotateInertia(const glm::mat3& Ilocal, const glm::vec3 axis[3]) {
  glm::mat3 R(axis[0], axis[1], axis[2]);
  return R * Ilocal * glm::transpose(R);
}

void massProps(const std::vector<ChunkObb>& shapes, const std::vector<uint32_t>& chunks, float& mass, glm::vec3& com,
               glm::mat3& Icom) {
  mass = 0.0f;
  com = glm::vec3(0.0f);
  for (uint32_t ci : chunks) {
    if (ci >= shapes.size()) {
      continue;
    }
    const ChunkObb& s = shapes[ci];
    mass += s.mass;
    com += s.mass * s.center;
  }
  if (mass > 0.0f) {
    com /= mass;
  }
  Icom = glm::mat3(0.0f);
  for (uint32_t ci : chunks) {
    if (ci >= shapes.size()) {
      continue;
    }
    const ChunkObb& s = shapes[ci];
    const glm::vec3 full(2.0f * s.half.x, 2.0f * s.half.y, 2.0f * s.half.z);
    const glm::mat3 I0 = rotateInertia(boxInertia(s.mass, full), s.axis);
    const glm::vec3 r = s.center - com;
    Icom += I0 + s.mass * (glm::dot(r, r) * glm::mat3(1.0f) - glm::outerProduct(r, r));
  }
}

float projectObb(const ChunkObb& a, const glm::vec3& n) {
  return a.half.x * std::abs(glm::dot(a.axis[0], n)) + a.half.y * std::abs(glm::dot(a.axis[1], n)) +
         a.half.z * std::abs(glm::dot(a.axis[2], n));
}

bool overlapObb(const ChunkObb& a, const ChunkObb& b, float& penetration) {
  glm::vec3 axes[15];
  int nAxes = 0;
  for (int i = 0; i < 3; ++i) {
    axes[nAxes++] = a.axis[i];
    axes[nAxes++] = b.axis[i];
  }
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      const glm::vec3 c = glm::cross(a.axis[i], b.axis[j]);
      const float len2 = glm::dot(c, c);
      if (len2 > 1.0e-10f) {
        axes[nAxes++] = c / std::sqrt(len2);
      }
    }
  }
  const glm::vec3 d = b.center - a.center;
  float minPen = std::numeric_limits<float>::max();
  for (int i = 0; i < nAxes; ++i) {
    const glm::vec3& n = axes[i];
    const float dist = std::abs(glm::dot(d, n));
    const float ra = projectObb(a, n);
    const float rb = projectObb(b, n);
    const float overlap = ra + rb - dist;
    if (overlap < 0.0f) {
      penetration = 0.0f;
      return false;
    }
    minPen = std::min(minPen, overlap);
  }
  penetration = minPen;
  return true;
}

}  // namespace

void buildCylinderChunkShapes(const NvBlastAsset* asset, float density, std::vector<ChunkObb>& shapes) {
  const uint32_t n = NvBlastAssetGetChunkCount(asset, nullptr);
  const NvBlastChunk* chunks = NvBlastAssetGetChunks(asset, nullptr);
  shapes.assign(n, ChunkObb{});
  const glm::vec3 half(0.5f * kCylinderT, 0.5f * cylinderLayerHeight(), 0.5f * cylinderArcLength());
  for (uint32_t i = 0; i < n; ++i) {
    ChunkObb s;
    s.chunkIndex = i;
    s.center = glm::vec3(chunks[i].centroid[0], chunks[i].centroid[1], chunks[i].centroid[2]);
    glm::vec3 radial(s.center.x, 0.0f, s.center.z);
    const float rlen = glm::length(radial);
    if (rlen > 1.0e-8f) {
      radial /= rlen;
    } else {
      radial = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    s.axis[0] = radial;
    s.axis[1] = glm::vec3(0.0f, 1.0f, 0.0f);
    s.axis[2] = glm::normalize(glm::cross(s.axis[1], s.axis[0]));
    s.half = half;
    s.volume = chunks[i].volume;
    s.mass = density * s.volume;
    shapes[i] = s;
  }
}

void inheritVelocity(ActorBody& child, const BodySnapshot& parent) {
  const glm::vec3 r = child.x - parent.x;
  child.v = parent.v + glm::cross(parent.w, r);
  child.w = parent.w;
  child.q = parent.q;
}

void rebuildBodiesFromFamily(NvBlastFamily* family, const NvBlastAsset* asset, SplitWorld& world,
                             const BodySnapshot* inheritFrom, NvBlastLog logFn) {
  const uint32_t actorCount = NvBlastFamilyGetActorCount(family, logFn);
  std::vector<NvBlastActor*> actors(actorCount, nullptr);
  NvBlastFamilyGetActors(actors.data(), actorCount, family, logFn);
  world.bodies.clear();
  world.bodies.reserve(actorCount);
  for (uint32_t i = 0; i < actorCount; ++i) {
    NvBlastActor* a = actors[i];
    if (a == nullptr) {
      continue;
    }
    ActorBody b;
    b.actor = a;
    b.actorIndex = NvBlastActorGetIndex(a, logFn);
    b.anchored = NvBlastActorHasExternalBonds(a, logFn);
    const uint32_t nvis = NvBlastActorGetVisibleChunkCount(a, logFn);
    b.chunks.resize(nvis);
    NvBlastActorGetVisibleChunkIndices(b.chunks.data(), nvis, a, logFn);
    massProps(world.shapes, b.chunks, b.mass, b.x, b.Ibody);
    b.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);
    if (inheritFrom != nullptr && !b.anchored) {
      inheritVelocity(b, *inheritFrom);
    }
    if (b.anchored) {
      b.v = glm::vec3(0.0f);
      b.w = glm::vec3(0.0f);
    }
    world.bodies.push_back(b);
  }
}

void integrateDynamic(SplitWorld& world, float dt, const glm::vec3& gravity) {
  for (ActorBody& b : world.bodies) {
    if (b.anchored || b.mass <= 0.0f) {
      continue;
    }
    b.v += gravity * dt;
    b.x += b.v * dt;
    const glm::quat wq(0.0f, b.w.x, b.w.y, b.w.z);
    b.q = glm::normalize(b.q + (0.5f * dt) * (wq * b.q));
  }
}

float totalMass(const SplitWorld& world) {
  float m = 0.0f;
  for (const ActorBody& b : world.bodies) {
    m += b.mass;
  }
  return m;
}

glm::vec3 totalLinearMomentum(const SplitWorld& world) {
  glm::vec3 p(0.0f);
  for (const ActorBody& b : world.bodies) {
    p += b.mass * b.v;
  }
  return p;
}

glm::vec3 totalAngularMomentumAbout(const SplitWorld& world, const glm::vec3& origin) {
  glm::vec3 L(0.0f);
  for (const ActorBody& b : world.bodies) {
    const glm::mat3 R = glm::mat3_cast(b.q);
    const glm::mat3 Iw = R * b.Ibody * glm::transpose(R);
    const glm::vec3 r = b.x - origin;
    L += glm::cross(r, b.mass * b.v) + Iw * b.w;
  }
  return L;
}

float maxBirthPenetration(const SplitWorld& world) {
  float maxPen = 0.0f;
  for (size_t i = 0; i < world.bodies.size(); ++i) {
    for (size_t j = i + 1; j < world.bodies.size(); ++j) {
      const ActorBody& A = world.bodies[i];
      const ActorBody& B = world.bodies[j];
      for (uint32_t ca : A.chunks) {
        if (ca >= world.shapes.size()) {
          continue;
        }
        for (uint32_t cb : B.chunks) {
          if (cb >= world.shapes.size()) {
            continue;
          }
          float pen = 0.0f;
          if (overlapObb(world.shapes[ca], world.shapes[cb], pen)) {
            maxPen = std::max(maxPen, pen);
          }
        }
      }
    }
  }
  return maxPen;
}

bool hasGhostAnchor(const SplitWorld& world, NvBlastLog logFn) {
  for (const ActorBody& b : world.bodies) {
    if (b.actor == nullptr) {
      continue;
    }
    const bool ext = NvBlastActorHasExternalBonds(b.actor, logFn);
    if (b.anchored != ext) {
      return true;
    }
    if (!ext && b.mass > 0.0f) {
      // Dynamic bodies must be allowed to move; anchored flag already false.
    }
    if (ext && (glm::length(b.v) > 1.0e-8f || glm::length(b.w) > 1.0e-8f)) {
      return true;
    }
  }
  return false;
}

}  // namespace blast
