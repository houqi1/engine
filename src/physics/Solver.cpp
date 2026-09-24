#include "physics/Solver.h"

#include "physics/ParallelFor.h"

#include <algorithm>
#include <cmath>

namespace physics {
namespace {

// Compact hot state for the iteration loop (64 bytes vs a full RigidBody with
// its support vector). Same arithmetic as operating on RigidBody directly.
struct SolverBody {
  glm::vec3 v;
  float invM;
  glm::vec3 w;
  float pad;
  glm::mat3 IinvW;
};

struct Row {
  int a, b;
  float k;
};

// Below this many contacts islands are solved on the calling thread.
constexpr size_t kParallelSolveMinContacts = 256;

template<class Body>
void applyImpulse(Body& a, Body& b, const glm::vec3& rA, const glm::vec3& rB,
                  const glm::vec3& J) {
  if (a.invM > 0.0f) {
    a.v += a.invM * J;
    a.w += a.IinvW * glm::cross(rA, J);
  }
  if (b.invM > 0.0f) {
    b.v -= b.invM * J;
    b.w -= b.IinvW * glm::cross(rB, J);
  }
}

template<class Body>
float effectiveMass(const Body& a, const Body& b, const glm::vec3& rA,
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

int findRoot(std::vector<int>& parent, int i) {
  while (parent[static_cast<size_t>(i)] != i) {
    parent[static_cast<size_t>(i)] = parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
    i = parent[static_cast<size_t>(i)];
  }
  return i;
}

// Sequential impulses over one island's contacts, in their original order.
void solveIsland(std::vector<SolverBody>& sb, std::vector<Contact>& contacts, std::vector<Row>& rows,
                 const int* order, size_t count, float hSub, int iterations) {
  // Velocities are sampled by PhysicsWorld before this warm start. The seed is
  // applied once in this substep; subsequent iterations apply only its delta.
  for (size_t o = 0; o < count; ++o) {
    const size_t i = static_cast<size_t>(order[o]);
    Contact& c = contacts[i];
    c.lambdaN = std::max(0.0f, c.lambdaN);
    c.lambdaNVel = c.lambdaN;
    if (c.d < 0.0f) c.JtWorld = glm::vec3(0.0f);
    applyImpulse(sb[static_cast<size_t>(rows[i].a)], sb[static_cast<size_t>(rows[i].b)], c.rA, c.rB,
                 c.lambdaN * c.n + c.JtWorld);
  }
  // Normal effective mass depends only on rA/rB/n/IinvW, all fixed within the
  // substep: compute once instead of once per iteration.
  for (size_t o = 0; o < count; ++o) {
    const size_t i = static_cast<size_t>(order[o]);
    const Contact& c = contacts[i];
    rows[i].k = effectiveMass(sb[static_cast<size_t>(rows[i].a)], sb[static_cast<size_t>(rows[i].b)], c.rA,
                              c.rB, c.n);
  }
  const int iters = std::max(1, iterations);
  for (int it = 0; it < iters; ++it) {
    for (size_t o = 0; o < count; ++o) {
      const size_t i = static_cast<size_t>(order[o]);
      Contact& c = contacts[i];
      const Row& row = rows[i];
      SolverBody& A = sb[static_cast<size_t>(row.a)];
      SolverBody& B = sb[static_cast<size_t>(row.b)];
      if (A.invM <= 0.0f && B.invM <= 0.0f) {
        continue;
      }

      const glm::vec3 vA = A.v + glm::cross(A.w, c.rA);
      const glm::vec3 vB = B.v + glm::cross(B.w, c.rB);
      const glm::vec3 vRel = vA - vB;
      const float vn = glm::dot(vRel, c.n);
      const float k = row.k;
      if (k <= 1e-8f) {
        continue;
      }

      // Target normal velocity: speculative gap limits approach; penetration gets
      // Baumgarte push-out; flush / within slop wants vn == 0.
      float targetVnColl = 0.0f;
      float targetVnBaum = 0.0f;
      if (hSub > 0.0f) {
        if (c.d < 0.0f) {
          targetVnColl = c.d / hSub;
        } else if (c.d > kSlop) {
          targetVnBaum = kBaumgarte * (c.d - kSlop) / hSub;
        }
      }
      const float targetVn = targetVnColl + targetVnBaum;
      float dLam = (targetVn - vn) / k;
      const float lambdaN = std::max(0.0f, c.lambdaN + dLam);
      dLam = lambdaN - c.lambdaN;
      c.lambdaN = lambdaN;
      applyImpulse(A, B, c.rA, c.rB, dLam * c.n);
      float dLamVel = (targetVnColl - vn) / k;
      const float lambdaNVel = std::max(0.0f, c.lambdaNVel + dLamVel);
      c.lambdaNVel = lambdaNVel;

      const glm::vec3 vA2 = A.v + glm::cross(A.w, c.rA);
      const glm::vec3 vB2 = B.v + glm::cross(B.w, c.rB);
      const glm::vec3 vRel2 = vA2 - vB2;
      glm::vec3 vt = vRel2 - glm::dot(vRel2, c.n) * c.n;
      const float vtLen = glm::length(vt);
      // Accumulate a world-space tangent vector. A scalar accumulated against a
      // changing slip direction cannot be safely warm-started (direction reversal
      // would otherwise reuse friction with the wrong sign).
      glm::vec3 friction = c.JtWorld;
      if (vtLen > 1e-6f) {
        const glm::vec3 t = vt / vtLen;
        const float kt = effectiveMass(A, B, c.rA, c.rB, t);
        if (kt > 1e-8f) friction -= (vtLen / kt) * t;
      }
      const float maxF = c.d >= 0.0f ? kFriction * c.lambdaN : 0.0f;
      const float length = glm::length(friction);
      if (length > maxF && length > 0.0f) friction *= maxF / length;
      applyImpulse(A, B, c.rA, c.rB, friction - c.JtWorld);
      c.JtWorld = friction;
      c.lambdaT = glm::length(friction);
    }
  }
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
  const int bodyCount = static_cast<int>(bodies.size());
  // Gather only bodies referenced by contacts into a dense array; contacts
  // index it through local ids. Scratch persists across calls.
  thread_local std::vector<SolverBody> sb;
  thread_local std::vector<int> localOf;
  thread_local std::vector<int> globalOf;
  thread_local std::vector<Row> rows;
  thread_local std::vector<int> parent;
  thread_local std::vector<int> islandOf;    // per contact, -1 = skipped
  thread_local std::vector<int> islandStart; // prefix offsets into order
  thread_local std::vector<int> order;       // contact indices grouped by island
  thread_local std::vector<int> islandIdOfRoot;
  localOf.assign(static_cast<size_t>(bodyCount), -1);
  globalOf.clear();
  sb.clear();
  auto local = [&](int g) {
    int& l = localOf[static_cast<size_t>(g)];
    if (l < 0) {
      l = static_cast<int>(sb.size());
      const RigidBody& b = bodies[static_cast<size_t>(g)];
      sb.push_back(SolverBody{b.v, b.invM, b.w, 0.0f, b.IinvW});
      globalOf.push_back(g);
    }
    return l;
  };
  rows.resize(contacts.size());
  for (size_t i = 0; i < contacts.size(); ++i) {
    const Contact& c = contacts[i];
    const bool valid = c.a >= 0 && c.b >= 0 && c.a < bodyCount && c.b < bodyCount;
    rows[i] = valid ? Row{local(c.a), local(c.b), 0.0f} : Row{-1, -1, 0.0f};
  }

  // Islands: only bodies with invM > 0 are ever written, so two contacts can
  // interact only through a shared movable body. Static/kinematic bodies are
  // read-only and may be shared freely. Solving each island in its original
  // contact order is therefore bit-identical to one global sequential pass.
  parent.resize(sb.size());
  for (size_t l = 0; l < sb.size(); ++l) parent[l] = static_cast<int>(l);
  for (const Row& r : rows) {
    if (r.a < 0 || sb[static_cast<size_t>(r.a)].invM <= 0.0f || sb[static_cast<size_t>(r.b)].invM <= 0.0f) {
      continue;
    }
    const int ra = findRoot(parent, r.a);
    const int rb = findRoot(parent, r.b);
    if (ra != rb) parent[static_cast<size_t>(std::max(ra, rb))] = std::min(ra, rb);
  }
  islandIdOfRoot.assign(sb.size(), -1);
  islandOf.assign(contacts.size(), -1);
  islandStart.clear();
  int islands = 0;
  for (size_t i = 0; i < rows.size(); ++i) {
    const Row& r = rows[i];
    if (r.a < 0) continue;
    // A contact with no movable body does nothing; skip it as before.
    int anchor = -1;
    if (sb[static_cast<size_t>(r.a)].invM > 0.0f) anchor = r.a;
    else if (sb[static_cast<size_t>(r.b)].invM > 0.0f) anchor = r.b;
    if (anchor < 0) continue;
    int& id = islandIdOfRoot[static_cast<size_t>(findRoot(parent, anchor))];
    if (id < 0) {
      id = islands++;
      islandStart.push_back(0);
    }
    islandOf[i] = id;
    ++islandStart[static_cast<size_t>(id)];
  }
  // Counting sort (stable): contacts keep their relative order inside an island.
  int sum = 0;
  for (int& s : islandStart) {
    const int cnt = s;
    s = sum;
    sum += cnt;
  }
  islandStart.push_back(sum);
  order.resize(static_cast<size_t>(sum));
  {
    thread_local std::vector<int> cursor;
    cursor.assign(islandStart.begin(), islandStart.end() - 1);
    for (size_t i = 0; i < rows.size(); ++i) {
      if (islandOf[i] >= 0) order[static_cast<size_t>(cursor[static_cast<size_t>(islandOf[i])]++)] = static_cast<int>(i);
    }
  }

  // Warm start of a skipped (no movable body) contact still normalises its
  // accumulators, matching the previous behaviour for reporting.
  for (size_t i = 0; i < rows.size(); ++i) {
    if (rows[i].a >= 0 && islandOf[i] < 0) {
      Contact& c = contacts[i];
      c.lambdaN = std::max(0.0f, c.lambdaN);
      c.lambdaNVel = c.lambdaN;
      if (c.d < 0.0f) c.JtWorld = glm::vec3(0.0f);
    }
  }

  std::vector<SolverBody>& sbRef = sb;
  std::vector<Row>& rowsRef = rows;
  const std::vector<int>& orderRef = order;
  const std::vector<int>& startRef = islandStart;
  auto solveOne = [&](size_t island) {
    const int begin = startRef[island];
    const int end = startRef[island + 1];
    solveIsland(sbRef, contacts, rowsRef, orderRef.data() + begin, static_cast<size_t>(end - begin), hSub,
                iterations);
  };
  if (static_cast<size_t>(sum) >= kParallelSolveMinContacts && islands > 1) {
    parallelFor(static_cast<size_t>(islands), 2, solveOne);
  } else {
    for (int island = 0; island < islands; ++island) solveOne(static_cast<size_t>(island));
  }

  for (size_t l = 0; l < sb.size(); ++l) {
    RigidBody& b = bodies[static_cast<size_t>(globalOf[l])];
    b.v = sb[l].v;
    b.w = sb[l].w;
  }
}

glm::vec3 contactImpulseOnA(const Contact& c) { return c.lambdaNVel * c.n + c.JtWorld; }

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
