#pragma once

#include "physics/RigidBody.h"
#include <unordered_map>
#include <cmath>
#include <utility>

namespace physics {

// Generation prevents recycled slots from inheriting contacts; revision covers
// edits even when the resulting occupancy bounds happen to be unchanged.
struct ContactBodyStamp {
  uint32_t generation = 0;
  uint64_t revision = 0;
  float fineSize = kVoxel;
};

class ContactCache {
public:
  struct Stats { int generated = 0, reused = 0, warmPoints = 0; };
  void clear() { pairs_.clear(); serial_ = 0; }
  void invalidate(int slot) {
    for (auto it = pairs_.begin(); it != pairs_.end();) {
      if (static_cast<int>(it->first >> 32) == slot || static_cast<int>(it->first & 0xffffffffu) == slot)
        it = pairs_.erase(it);
      else ++it;
    }
  }
  void beginSubstep() { ++serial_; }
  void endSubstep() {
    // Sleeping/disabled/separated pairs must not retain old impact impulses.
    for (auto it = pairs_.begin(); it != pairs_.end();)
      if (it->second.seen != serial_) it = pairs_.erase(it); else ++it;
  }
  size_t size() const { return pairs_.size(); }

  struct Pair;
  // Two-phase use for parallel narrow phase: call slot() serially for every
  // pair (the only map mutation; node references stay valid across rehash),
  // then collectInto() may run concurrently for distinct slots.
  Pair& slot(int a, int b) { return pairs_[key(a, b)]; }

  template<class Generate>
  Stats collect(const RigidBody& a, const RigidBody& b, ContactBodyStamp sa, ContactBodyStamp sb,
                int substep, std::vector<Contact>& out, Generate&& generate, bool enabled = true) {
    if (!enabled) { generate(out); return {1, 0, 0}; }
    return collectInto(slot(a.shapeIndex, b.shapeIndex), a, b, sa, sb, substep, out,
                       std::forward<Generate>(generate));
  }

  template<class Generate>
  Stats collectInto(Pair& pair, const RigidBody& a, const RigidBody& b, ContactBodyStamp sa,
                    ContactBodyStamp sb, int substep, std::vector<Contact>& out, Generate&& generate) const {
    const bool compatible = pair.seen + 1 == serial_ && pair.seen != 0 &&
        same(pair.a, a, pair.sa, sa) && same(pair.b, b, pair.sb, sb);
    const float limit = 0.2f * std::min(sa.fineSize, sb.fineSize);
    const float moved = motion(pair.a, a) + motion(pair.b, b);
    const float predicted = kSubDt * (speed(a) + speed(b));
    bool reuse = compatible && substep != 0 && !pair.points.empty() &&
                 moved <= limit && predicted <= limit;
    const size_t begin = out.size();
    if (reuse) {
      for (const Point& p : pair.points) {
        const RigidBody& ca = p.solved.a == a.shapeIndex ? a : b;
        const RigidBody& cb = p.solved.b == b.shapeIndex ? b : a;
        Contact c = p.solved;
        c.n = glm::normalize(cb.q * p.normalB);
        const glm::vec3 pa = ca.x + ca.q * p.localA;
        const glm::vec3 pb = cb.x + cb.q * p.localB;
        const glm::vec3 delta = pa - pb;
        c.d = p.depth - glm::dot(delta, c.n);
        // A changing support patch requires a fresh manifold, not just dropping
        // the stale point (which could omit a replacement support point).
        if (c.d < -kContactLookAhead ||
            glm::length(delta - glm::dot(delta, c.n) * c.n) > limit) {
          reuse = false; break;
        }
        c.p = pa; c.rA = pa - ca.x; c.rB = pa - cb.x;
        seed(c, p.solved);
        out.push_back(c);
      }
      if (!reuse) out.resize(begin);
    }
    Stats stats;
    if (reuse) {
      stats.reused = 1;
    } else {
      stats.generated = 1;
      generate(out);
      const bool warm = compatible && moved < 2.5f * limit;
      for (size_t i = begin; i < out.size(); ++i) {
        Contact& c = out[i];
        if (warm) {
          for (const Point& old : pair.points) {
            if (c.a == old.solved.a && c.b == old.solved.b && c.fineA == old.solved.fineA &&
                c.fineB == old.solved.fineB && c.nFace == old.solved.nFace &&
                glm::dot(c.n, old.solved.n) > 0.98f) {
              seed(c, old.solved); break;
            }
          }
        }
      }
      pair.points.clear();
      for (size_t i = begin; i < out.size(); ++i) {
        const Contact& c = out[i];
        const RigidBody& ca = c.a == a.shapeIndex ? a : b;
        const RigidBody& cb = c.b == b.shapeIndex ? b : a;
        pair.points.push_back({c, glm::conjugate(ca.q) * (c.p - ca.x),
                             glm::conjugate(cb.q) * (c.p - cb.x), glm::conjugate(cb.q) * c.n, c.d});
      }
      pair.a = pose(a); pair.b = pose(b);
      pair.sa = sa; pair.sb = sb;
    }
    pair.seen = serial_;
    for (size_t i = begin; i < out.size(); ++i) {
      out[i].cachePoint = static_cast<uint32_t>(i - begin);
      if (out[i].lambdaN > 0.0f) ++stats.warmPoints;
    }
    return stats;
  }
  void store(const std::vector<Contact>& contacts) {
    for (const Contact& c : contacts) {
      auto it = pairs_.find(key(c.a, c.b));
      if (it != pairs_.end() && c.cachePoint < it->second.points.size())
        it->second.points[c.cachePoint].solved = c;
    }
  }

  struct Pose {
    glm::vec3 x{0}, com{0}; glm::quat q{1,0,0,0};
    float invM = 0, extent = 0; bool awake = false, dynamic = false;
  };
  struct Point { Contact solved; glm::vec3 localA, localB, normalB; float depth; };
  struct Pair {
    Pose a, b; ContactBodyStamp sa, sb; std::vector<Point> points; uint64_t seen = 0;
  };

private:
  std::unordered_map<uint64_t, Pair> pairs_;
  uint64_t serial_ = 0;
  static uint64_t key(int a, int b) {
    return (uint64_t(static_cast<uint32_t>(std::min(a,b))) << 32) | static_cast<uint32_t>(std::max(a,b));
  }
  static Pose pose(const RigidBody& b) { return {b.x,b.comLocal,b.q,b.invM,b.extent,b.awake,b.dynamic}; }
  static bool same(const Pose& p, const RigidBody& b, ContactBodyStamp old, ContactBodyStamp now) {
    return old.generation == now.generation && old.revision == now.revision && old.fineSize == now.fineSize &&
           p.invM == b.invM && p.com == b.comLocal && p.awake == b.awake && p.dynamic == b.dynamic;
  }
  static float motion(const Pose& p, const RigidBody& b) {
    const float dot = std::clamp(std::abs(glm::dot(p.q,b.q)),0.0f,1.0f);
    return glm::length(b.x-p.x) + 2.0f * std::acos(dot) * std::max(b.extent,kSphereRadius);
  }
  static float speed(const RigidBody& b) {
    return glm::length(b.v) + glm::length(b.w) * std::max(b.extent,kSphereRadius);
  }
  static void seed(Contact& c, const Contact& old) {
    // Reuse physical support only, never the previous penetration correction.
    c.lambdaN = c.d >= -kSlop ? std::max(0.0f, old.lambdaNVel) : 0.0f;
    c.lambdaNVel = c.lambdaN;
    c.lambdaT = 0;
    c.JtWorld = c.d >= 0.0f ? old.JtWorld - glm::dot(old.JtWorld,c.n)*c.n : glm::vec3(0);
    const float length = glm::length(c.JtWorld), maxF = kFriction*c.lambdaN;
    if (length > maxF && length > 0) c.JtWorld *= maxF/length;
    c.preVelA = c.preVelB = glm::vec3(0);
  }
};
} // namespace physics
