#include "physics/Solver.h"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

void applyImpulse(RigidBody& a, RigidBody& b, const glm::vec3& rA, const glm::vec3& rB,
                  const glm::vec3& J) {
  if (a.invM > 0.0f) {
    a.v += a.invM * J;
    a.w += a.IinvW * glm::cross(rA, J);
    a.awake = true;
    a.sleepTimer = 0.0f;
  }
  if (b.invM > 0.0f) {
    b.v -= b.invM * J;
    b.w -= b.IinvW * glm::cross(rB, J);
    b.awake = true;
    b.sleepTimer = 0.0f;
  }
}

float effectiveMass(const RigidBody& a, const RigidBody& b, const glm::vec3& rA,
                    const glm::vec3& rB, const glm::vec3& n) {
  const glm::vec3 tA = glm::cross(rA, n);
  const glm::vec3 tB = glm::cross(rB, n);
  float k = a.invM + b.invM;
  if (a.invM > 0.0f) {
    k += glm::dot(tA, a.IinvW * tA);
  }
  if (b.invM > 0.0f) {
    k += glm::dot(tB, b.IinvW * tB);
  }
  return k;
}

}  // namespace

void refreshInverseInertiaWorld(RigidBody& b) {
  if (b.invM <= 0.0f) {
    b.IinvW = glm::mat3(0.0f);
    return;
  }
  const glm::mat3 R = glm::mat3_cast(b.q);
  const glm::mat3 IinvL = glm::inverse(b.Iloc);
  b.IinvW = R * IinvL * glm::transpose(R);
}

void solveContacts(std::vector<RigidBody>& bodies, std::vector<Contact>& contacts, float hSub,
                   int iterations) {
  const int iters = std::max(1, iterations);
  for (int it = 0; it < iters; ++it) {
  for (Contact& c : contacts) {
    if (c.a < 0 || c.b < 0 || c.a >= static_cast<int>(bodies.size()) ||
        c.b >= static_cast<int>(bodies.size())) {
      continue;
    }
    RigidBody& A = bodies[static_cast<size_t>(c.a)];
    RigidBody& B = bodies[static_cast<size_t>(c.b)];
    if (A.invM <= 0.0f && B.invM <= 0.0f) {
      continue;
    }

    const glm::vec3 vA = A.v + glm::cross(A.w, c.rA);
    const glm::vec3 vB = B.v + glm::cross(B.w, c.rB);
    const glm::vec3 vRel = vA - vB;
    const float vn = glm::dot(vRel, c.n);
    const float k = effectiveMass(A, B, c.rA, c.rB, c.n);
    if (k <= 1e-8f) {
      continue;
    }

    // Target normal velocity: speculative gap limits approach; penetration gets
    // Baumgarte push-out; flush / within slop wants vn == 0.
    float targetVn = 0.0f;
    if (hSub > 0.0f) {
      if (c.d < 0.0f) {
        targetVn = c.d / hSub;
      } else if (c.d > kSlop) {
        targetVn = kBaumgarte * (c.d - kSlop) / hSub;
      }
    }
    float dLam = (targetVn - vn) / k;
    const float lambdaN = std::max(0.0f, c.lambdaN + dLam);
    dLam = lambdaN - c.lambdaN;
    c.lambdaN = lambdaN;
    applyImpulse(A, B, c.rA, c.rB, dLam * c.n);

    // No friction while still separated — otherwise speculative contacts brake in air.
    if (c.d < 0.0f || c.lambdaN <= 0.0f) {
      continue;
    }

    const glm::vec3 vA2 = A.v + glm::cross(A.w, c.rA);
    const glm::vec3 vB2 = B.v + glm::cross(B.w, c.rB);
    const glm::vec3 vRel2 = vA2 - vB2;
    glm::vec3 vt = vRel2 - glm::dot(vRel2, c.n) * c.n;
    const float vtLen = glm::length(vt);
    if (vtLen <= 1e-6f) {
      continue;
    }
    const glm::vec3 t = vt / vtLen;
    const float kt = effectiveMass(A, B, c.rA, c.rB, t);
    if (kt <= 1e-8f) {
      continue;
    }
    float dLamT = -glm::dot(vt, t) / kt;
    const float maxF = kFriction * c.lambdaN;
    const float lambdaT = std::clamp(c.lambdaT + dLamT, -maxF, maxF);
    dLamT = lambdaT - c.lambdaT;
    c.lambdaT = lambdaT;
    applyImpulse(A, B, c.rA, c.rB, dLamT * t);
  }
  }
}

void integrateBodies(std::vector<RigidBody>& bodies, float hSub) {
  for (RigidBody& b : bodies) {
    if (!b.awake || b.invM <= 0.0f) {
      continue;
    }
    b.x += b.v * hSub;
    const glm::quat wq(0.0f, b.w.x, b.w.y, b.w.z);
    b.q += (0.5f * hSub) * (wq * b.q);
    b.q = glm::normalize(b.q);
    refreshInverseInertiaWorld(b);
  }
}

}  // namespace physics
