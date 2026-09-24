// solveContacts (island-partitioned, parallel) must match the original single
// sequential-impulse pass bit for bit.
#include "physics/Solver.h"

#include <cstring>
#include <iostream>
#include <random>
#include <stdexcept>

using namespace physics;

namespace {

// Verbatim original algorithm operating on RigidBody directly.
void applyRef(RigidBody& a, RigidBody& b, const glm::vec3& rA, const glm::vec3& rB, const glm::vec3& J) {
  if (a.invM > 0.0f) { a.v += a.invM * J; a.w += a.IinvW * glm::cross(rA, J); }
  if (b.invM > 0.0f) { b.v -= b.invM * J; b.w -= b.IinvW * glm::cross(rB, J); }
}
float massRef(const RigidBody& a, const RigidBody& b, const glm::vec3& rA, const glm::vec3& rB,
              const glm::vec3& n) {
  const glm::vec3 tA = glm::cross(rA, n), tB = glm::cross(rB, n);
  float k = a.invM + b.invM;
  if (a.invM > 0.0f) k += glm::dot(tA, a.IinvW * tA);
  if (b.invM > 0.0f) k += glm::dot(tB, b.IinvW * tB);
  return k;
}
void solveRef(std::vector<RigidBody>& bodies, std::vector<Contact>& contacts, float hSub, int iterations) {
  const int nb = static_cast<int>(bodies.size());
  for (Contact& c : contacts) {
    if (c.a < 0 || c.b < 0 || c.a >= nb || c.b >= nb) continue;
    c.lambdaN = std::max(0.0f, c.lambdaN);
    c.lambdaNVel = c.lambdaN;
    if (c.d < 0.0f) c.JtWorld = glm::vec3(0.0f);
    applyRef(bodies[c.a], bodies[c.b], c.rA, c.rB, c.lambdaN * c.n + c.JtWorld);
  }
  for (int it = 0; it < std::max(1, iterations); ++it) {
    for (Contact& c : contacts) {
      if (c.a < 0 || c.b < 0 || c.a >= nb || c.b >= nb) continue;
      RigidBody& A = bodies[c.a];
      RigidBody& B = bodies[c.b];
      if (A.invM <= 0.0f && B.invM <= 0.0f) continue;
      const glm::vec3 vRel = (A.v + glm::cross(A.w, c.rA)) - (B.v + glm::cross(B.w, c.rB));
      const float vn = glm::dot(vRel, c.n);
      const float k = massRef(A, B, c.rA, c.rB, c.n);
      if (k <= 1e-8f) continue;
      float tColl = 0.0f, tBaum = 0.0f;
      if (hSub > 0.0f) {
        if (c.d < 0.0f) tColl = c.d / hSub;
        else if (c.d > kSlop) tBaum = kBaumgarte * (c.d - kSlop) / hSub;
      }
      float dLam = (tColl + tBaum - vn) / k;
      const float lambdaN = std::max(0.0f, c.lambdaN + dLam);
      dLam = lambdaN - c.lambdaN;
      c.lambdaN = lambdaN;
      applyRef(A, B, c.rA, c.rB, dLam * c.n);
      c.lambdaNVel = std::max(0.0f, c.lambdaNVel + (tColl - vn) / k);
      const glm::vec3 vRel2 = (A.v + glm::cross(A.w, c.rA)) - (B.v + glm::cross(B.w, c.rB));
      const glm::vec3 vt = vRel2 - glm::dot(vRel2, c.n) * c.n;
      const float vtLen = glm::length(vt);
      glm::vec3 friction = c.JtWorld;
      if (vtLen > 1e-6f) {
        const glm::vec3 t = vt / vtLen;
        const float kt = massRef(A, B, c.rA, c.rB, t);
        if (kt > 1e-8f) friction -= (vtLen / kt) * t;
      }
      const float maxF = c.d >= 0.0f ? kFriction * c.lambdaN : 0.0f;
      const float len = glm::length(friction);
      if (len > maxF && len > 0.0f) friction *= maxF / len;
      applyRef(A, B, c.rA, c.rB, friction - c.JtWorld);
      c.JtWorld = friction;
      c.lambdaT = glm::length(friction);
    }
  }
}

bool same(const glm::vec3& a, const glm::vec3& b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
bool same(float a, float b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }

}  // namespace

int main() {
  try {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> u(-1.0f, 1.0f);
    int checkedContacts = 0;
    for (int trial = 0; trial < 60; ++trial) {
      // Few statics shared by everyone (ground), many dynamics in varied islands.
      const int nb = 20 + trial * 17;
      std::vector<RigidBody> bodies(static_cast<size_t>(nb));
      for (int i = 0; i < nb; ++i) {
        RigidBody& b = bodies[static_cast<size_t>(i)];
        b.shapeIndex = i;
        b.dynamic = i >= 3;
        b.invM = b.dynamic ? 0.5f + 0.5f * u(rng) + 0.6f : 0.0f;
        b.q = glm::normalize(glm::quat(1.0f + u(rng), u(rng), u(rng), u(rng)));
        b.Iloc = glm::mat3(1.0f + 0.3f * u(rng));
        b.v = glm::vec3(u(rng), u(rng) - 1.0f, u(rng));
        b.w = glm::vec3(u(rng), u(rng), u(rng));
        refreshInverseInertiaWorld(b);
      }
      std::vector<Contact> contacts;
      const int nc = nb * 4 + trial * 11;
      std::uniform_int_distribution<int> pick(0, nb - 1);
      std::uniform_int_distribution<int> local(0, 5);
      for (int k = 0; k < nc; ++k) {
        Contact c;
        const int mode = k % 5;
        c.a = pick(rng);
        // Mostly ground contacts and local neighbours, some long-range links,
        // plus the odd static-static and invalid index.
        if (mode < 2) c.b = pick(rng) % 3;
        else if (mode < 4) c.b = std::min(nb - 1, c.a + local(rng));
        else c.b = pick(rng);
        if (k % 97 == 0) c.b = nb + 5;
        if (c.a == c.b) c.b = (c.b + 1) % nb;
        c.n = glm::normalize(glm::vec3(u(rng), 1.5f + u(rng), u(rng)));
        c.rA = 0.1f * glm::vec3(u(rng), u(rng), u(rng));
        c.rB = 0.1f * glm::vec3(u(rng), u(rng), u(rng));
        c.d = 0.02f * u(rng);
        c.lambdaN = k % 3 == 0 ? 0.05f * (u(rng) + 1.0f) : 0.0f;
        c.JtWorld = k % 4 == 0 ? 0.01f * glm::vec3(u(rng), 0.0f, u(rng)) : glm::vec3(0.0f);
        contacts.push_back(c);
      }
      auto refBodies = bodies;
      auto refContacts = contacts;
      solveRef(refBodies, refContacts, kSubDt, kContactIters);
      solveContacts(bodies, contacts, kSubDt, kContactIters);
      for (int i = 0; i < nb; ++i) {
        if (!same(bodies[i].v, refBodies[i].v) || !same(bodies[i].w, refBodies[i].w))
          throw std::runtime_error("body velocity differs from sequential reference");
      }
      for (size_t k = 0; k < contacts.size(); ++k) {
        const Contact& a = contacts[k];
        const Contact& b = refContacts[k];
        if (!same(a.lambdaN, b.lambdaN) || !same(a.lambdaNVel, b.lambdaNVel) || !same(a.lambdaT, b.lambdaT) ||
            !same(a.JtWorld, b.JtWorld))
          throw std::runtime_error("contact accumulator differs from sequential reference");
      }
      checkedContacts += static_cast<int>(contacts.size());
    }
    std::cout << "PASS island-parallel solver is bit-identical to sequential reference ("
              << checkedContacts << " contacts)\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "FAIL: " << e.what() << '\n';
    return 1;
  }
}
