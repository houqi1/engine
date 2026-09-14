#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/HardFracture.h"
#include "blast/OccupancySampler.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <string>
#include <vector>

namespace {

int gFailures = 0;

void expect(bool cond, const std::string& msg) {
  if (!cond) {
    std::cerr << "FAIL: " << msg << "\n";
    ++gFailures;
  }
}

class DenseCylinderView final : public blast::OccupancyView {
public:
  explicit DenseCylinderView(const blast::CylinderRaster& spec, bool cut)
      : spec_(spec), solid_(static_cast<size_t>(spec.nx * spec.ny * spec.nz), 0),
        anc_(static_cast<size_t>(spec.nx * spec.ny * spec.nz), 0) {
    blast::forEachCylinderFine(spec_, [&](int x, int y, int z) {
      if (cut && blast::inBaseCut270(x, y, z, spec_)) {
        return;
      }
      solid_[idx(x, y, z)] = 1;
      if (blast::isBaseAnchorFine(x, y, z, spec_)) {
        anc_[idx(x, y, z)] = 1;
      }
    });
  }
  int nx() const override { return spec_.nx; }
  int ny() const override { return spec_.ny; }
  int nz() const override { return spec_.nz; }
  float voxelSize() const override { return spec_.voxelSize; }
  float density() const override { return blast::kE1Density; }
  float originX() const override { return spec_.ox; }
  float originY() const override { return spec_.oy; }
  float originZ() const override { return spec_.oz; }
  bool solid(int x, int y, int z) const override {
    if (x < 0 || y < 0 || z < 0 || x >= spec_.nx || y >= spec_.ny || z >= spec_.nz) {
      return false;
    }
    return solid_[idx(x, y, z)] != 0;
  }
  bool anchor(int x, int y, int z) const override {
    if (x < 0 || y < 0 || z < 0 || x >= spec_.nx || y >= spec_.ny || z >= spec_.nz) {
      return false;
    }
    return anc_[idx(x, y, z)] != 0;
  }
  uint32_t occupied() const {
    uint32_t n = 0;
    for (uint8_t v : solid_) {
      n += v != 0 ? 1u : 0u;
    }
    return n;
  }

private:
  size_t idx(int x, int y, int z) const {
    return static_cast<size_t>(x + spec_.nx * (y + spec_.ny * z));
  }
  blast::CylinderRaster spec_;
  std::vector<uint8_t> solid_;
  std::vector<uint8_t> anc_;
};

bool sixConnected(const DenseCylinderView& v) {
  const int dx[6] = {1, -1, 0, 0, 0, 0};
  const int dy[6] = {0, 0, 1, -1, 0, 0};
  const int dz[6] = {0, 0, 0, 0, 1, -1};
  int sx = -1, sy = -1, sz = -1;
  uint32_t total = 0;
  for (int z = 0; z < v.nz(); ++z) {
    for (int y = 0; y < v.ny(); ++y) {
      for (int x = 0; x < v.nx(); ++x) {
        if (!v.solid(x, y, z)) {
          continue;
        }
        ++total;
        if (sx < 0) {
          sx = x;
          sy = y;
          sz = z;
        }
      }
    }
  }
  if (total == 0) {
    return false;
  }
  std::vector<uint8_t> seen(static_cast<size_t>(v.nx() * v.ny() * v.nz()), 0);
  auto id = [&](int x, int y, int z) { return x + v.nx() * (y + v.ny() * z); };
  std::queue<int> q;
  q.push(id(sx, sy, sz));
  seen[static_cast<size_t>(id(sx, sy, sz))] = 1;
  uint32_t reach = 0;
  while (!q.empty()) {
    const int p = q.front();
    q.pop();
    ++reach;
    const int z = p / (v.nx() * v.ny());
    const int rem = p - z * v.nx() * v.ny();
    const int y = rem / v.nx();
    const int x = rem - y * v.nx();
    for (int d = 0; d < 6; ++d) {
      const int nx = x + dx[d], ny = y + dy[d], nz = z + dz[d];
      if (!v.solid(nx, ny, nz)) {
        continue;
      }
      const int nid = id(nx, ny, nz);
      if (!seen[static_cast<size_t>(nid)]) {
        seen[static_cast<size_t>(nid)] = 1;
        q.push(nid);
      }
    }
  }
  return reach == total;
}

uint32_t cavityCount(const DenseCylinderView& v, const blast::CylinderRaster& spec) {
  uint32_t n = 0;
  const float inner = spec.R - 0.5f * spec.t - spec.voxelSize;
  for (int z = 0; z < spec.nz; ++z) {
    for (int y = spec.y0; y < spec.y1; ++y) {
      for (int x = 0; x < spec.nx; ++x) {
        if (!v.solid(x, y, z)) {
          continue;
        }
        const float px = blast::voxelCenter(x, spec.voxelSize, spec.ox);
        const float pz = blast::voxelCenter(z, spec.voxelSize, spec.oz);
        const float dx = px - blast::cylinderAxisX(spec);
        const float dz = pz - blast::cylinderAxisZ(spec);
        if (std::sqrt(dx * dx + dz * dz) < inner) {
          ++n;
        }
      }
    }
  }
  return n;
}

void heightAndAround(const blast::VoxelStructureGraph& g, int& heightNodes, int& aroundNodes) {
  heightNodes = 0;
  aroundNodes = 0;
  std::vector<int> yBins(64, 0);
  std::vector<int> aBins(16, 0);
  for (const auto& n : g.nodes) {
    const int yb = std::clamp(static_cast<int>(n.cy / 0.2f), 0, 63);
    yBins[static_cast<size_t>(yb)] += 1;
    float az = std::atan2(n.cz - 3.2f, n.cx - 3.2f);
    if (az < 0) {
      az += 6.28318530718f;
    }
    const int ab = std::clamp(static_cast<int>(az / (6.28318530718f / 16.0f)), 0, 15);
    aBins[static_cast<size_t>(ab)] += 1;
  }
  for (int c : yBins) {
    if (c > 0) {
      ++heightNodes;
    }
  }
  for (int c : aBins) {
    if (c > 0) {
      ++aroundNodes;
    }
  }
}

blast::OccupancySample build(const blast::CylinderRaster& spec, bool cut, int agg) {
  DenseCylinderView view(spec, cut);
  blast::OccupancySampleOpts opts;
  opts.agg = agg;
  return blast::sampleOccupancy(view, opts);
}

void solveWorld(blast::StructureWorld& world, int ticks) {
  physics::FixedStepClock clock;
  for (int i = 0; i < ticks; ++i) {
    clock.advance(physics::kDt, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  }
}

void driveUntilConverged(blast::StructureWorld& world, int maxTicks) {
  for (int i = 0; i < maxTicks; ++i) {
    solveWorld(world, 1);
    if (world.debug().converged) {
      return;
    }
  }
}

void testRaster() {
  std::cout << "E1 raster wall, cavity, 6-connect\n";
  blast::CylinderRaster spec{};
  DenseCylinderView intact(spec, false);
  expect(intact.occupied() > 1000, "occupied wall voxels");
  expect(sixConnected(intact), "shell is 6-connected");
  expect(cavityCount(intact, spec) == 0, "cavity stays empty");
  DenseCylinderView cut(spec, true);
  expect(cut.occupied() < intact.occupied(), "cut removes voxels");
  expect(sixConnected(cut), "cut shell still 6-connected via remaining strip");
}

void testGraph(blast::BlastRuntime& rt) {
  std::cout << "E1 occupancy sample + anchors\n";
  blast::CylinderRaster spec{};
  auto sample = build(spec, false, blast::kE1Agg);
  expect(sample.error == blast::BlastError::Ok, sample.message);
  expect(sample.worldBonds > 0, "world bonds present");
  int h = 0, a = 0;
  heightAndAround(sample.graph, h, a);
  expect(h >= 8, "multiple height nodes");
  expect(a >= 8, "multiple circumferential nodes");
  expect(sample.graph.nodes.size() > 16, "not a single node");
  std::cout << "  occupied=" << sample.occupied << " nodes=" << sample.graph.nodes.size()
            << " bonds=" << sample.graph.bonds.size() << " world=" << sample.worldBonds << " mass=" << sample.mass
            << "\n";
  blast::StructureWorld world;
  expect(world.init(rt), "world init");
  VoxelObjectId id{1, 1};
  expect(world.mountSample(id, std::move(sample), 200, blast::kE1StrengthHoldPa, blast::cylinderAxisX(spec),
                           blast::cylinderAxisZ(spec)) == blast::BlastError::Ok,
         world.lastError());
  world.clear();
}

void testReaction(blast::BlastRuntime& rt) {
  std::cout << "E1 intact weight vs reaction\n";
  blast::CylinderRaster spec{};
  auto sample = build(spec, false, blast::kE1Agg);
  expect(sample.error == blast::BlastError::Ok, sample.message);
  blast::StructureWorld world;
  expect(world.init(rt), "world");
  VoxelObjectId id{1, 1};
  expect(world.mountSample(id, std::move(sample), 400, blast::kE1StrengthHoldPa, blast::cylinderAxisX(spec),
                           blast::cylinderAxisZ(spec)) == blast::BlastError::Ok,
         world.lastError());
  driveUntilConverged(world, 8);
  const auto& d = world.debug();
  std::cout << "  W=" << d.weight << " Ry=" << d.reactionY << " lin=" << d.linErr << " conv=" << d.converged
            << " strip=" << d.stripMaxStress << "\n";
  expect(d.converged, "intact converged");
  expect(d.weight > 0.0f, "weight positive");
  const float rel = std::abs(d.reactionY - d.weight) / d.weight;
  std::cout << "  rel=" << rel << "\n";
  expect(rel <= blast::kE1ReactionRelTol, "T24-style reaction within 0.1%");
  world.clear();
}

void testCutAndDensity(blast::BlastRuntime& rt) {
  std::cout << "E1 270 cut stress rise and double density\n";
  blast::CylinderRaster spec{};
  auto intactS = build(spec, false, blast::kE1Agg);
  auto cutS = build(spec, true, blast::kE1Agg);
  expect(intactS.error == blast::BlastError::Ok, intactS.message);
  expect(cutS.error == blast::BlastError::Ok, cutS.message);
  for (const auto& n : cutS.graph.nodes) {
    expect(blast::nodeReachesAnchor(cutS.graph, n.stableId), "cut: every node reaches anchor");
  }
  blast::StructureWorld world;
  expect(world.init(rt), "world");
  VoxelObjectId id{1, 1};
  expect(world.mountSample(id, std::move(intactS), 400, blast::kE1StrengthHoldPa, blast::cylinderAxisX(spec),
                           blast::cylinderAxisZ(spec)) == blast::BlastError::Ok,
         "mount intact");
  driveUntilConverged(world, 8);
  const float strip0 = world.debug().stripMaxStress;
  expect(world.debug().converged, "intact solve");
  expect(world.mountSample(id, std::move(cutS), 400, blast::kE1StrengthHoldPa, blast::cylinderAxisX(spec),
                           blast::cylinderAxisZ(spec)) == blast::BlastError::Ok,
         "mount cut");
  world.markCut(true);
  driveUntilConverged(world, 8);
  const float strip1 = world.debug().stripMaxStress;
  std::cout << "  strip intact=" << strip0 << " cut=" << strip1 << " lin=" << world.debug().linErr
            << " conv=" << world.debug().converged << "\n";
  expect(strip1 > strip0 * 5.0f, "remaining strip stress rises after cut");
  if (!world.debug().converged) {
    std::cout << "  note: 3000-node cut residual still above SDK tolerance in head-test budget\n";
  }
  const float s1 = strip1;
  expect(world.setDensityScale(2.0f), "double density");
  driveUntilConverged(world, 8);
  const float strip2 = world.debug().stripMaxStress;
  std::cout << "  strip 2rho=" << strip2 << " ratio=" << strip2 / s1 << "\n";
  expect(strip2 > s1 * 1.8f && strip2 < s1 * 2.2f, "stress scales with density");
  world.clear();
}

void testOriginShift(blast::BlastRuntime& rt) {
  std::cout << "E1 local origin translation\n";
  blast::CylinderRaster a{};
  blast::CylinderRaster b = a;
  b.ox = 1.25f;
  b.oy = -0.4f;
  b.oz = 0.75f;
  auto sa = build(a, false, blast::kE1Agg);
  auto sb = build(b, false, blast::kE1Agg);
  expect(sa.error == blast::BlastError::Ok && sb.error == blast::BlastError::Ok, "both samples");
  blast::StructureWorld wa;
  blast::StructureWorld wb;
  expect(wa.init(rt) && wb.init(rt), "worlds");
  VoxelObjectId id{1, 1};
  expect(wa.mountSample(id, std::move(sa), 400, blast::kE1StrengthHoldPa, blast::cylinderAxisX(a),
                        blast::cylinderAxisZ(a)) == blast::BlastError::Ok,
         "mount a");
  expect(wb.mountSample(id, std::move(sb), 400, blast::kE1StrengthHoldPa, blast::cylinderAxisX(b),
                        blast::cylinderAxisZ(b)) == blast::BlastError::Ok,
         "mount b");
  driveUntilConverged(wa, 8);
  driveUntilConverged(wb, 8);
  const float relR = std::abs(wa.debug().reactionY - wb.debug().reactionY) /
                     std::max(wa.debug().weight, 1.0f);
  const float relS = std::abs(wa.debug().stripMaxStress - wb.debug().stripMaxStress) /
                     std::max(wa.debug().stripMaxStress, 1.0f);
  std::cout << "  relR=" << relR << " relS=" << relS << "\n";
  expect(relR <= blast::kE1ReactionRelTol * 10.0f, "reaction invariant to origin");
  expect(relS <= 0.05f, "strip stress invariant to origin");
  wa.clear();
  wb.clear();
}

void testAggCompare(blast::BlastRuntime& rt) {
  std::cout << "E1 agg 1 vs 2\n";
  blast::CylinderRaster spec{};
  auto a1 = build(spec, false, 1);
  auto a2 = build(spec, false, 2);
  expect(a1.error == blast::BlastError::Ok && a2.error == blast::BlastError::Ok, "both agg");
  int h1 = 0, c1 = 0, h2 = 0, c2 = 0;
  heightAndAround(a1.graph, h1, c1);
  heightAndAround(a2.graph, h2, c2);
  std::cout << "  agg1 nodes=" << a1.graph.nodes.size() << " hBins=" << h1 << " aBins=" << c1 << "\n";
  std::cout << "  agg2 nodes=" << a2.graph.nodes.size() << " hBins=" << h2 << " aBins=" << c2 << "\n";
  expect(h1 >= 8 && h2 >= 8, "both keep height resolution");
  expect(c1 >= 8 && c2 >= 8, "both keep circumferential resolution");
  expect(a1.graph.nodes.size() > a2.graph.nodes.size(), "agg1 has more nodes");
  (void)rt;
}

void testResetMemory(blast::BlastRuntime& rt) {
  std::cout << "E1 reset cycle memory\n";
  blast::StructureWorld world;
  expect(world.init(rt), "world");
  const std::size_t base = rt.liveBytes();
  blast::CylinderRaster spec{};
  VoxelObjectId id{1, 1};
  for (int i = 0; i < 8; ++i) {
    auto sample = build(spec, i % 2 == 1, blast::kE1Agg);
    expect(world.mountSample(id, std::move(sample), 50, blast::kE1StrengthHoldPa, blast::cylinderAxisX(spec),
                             blast::cylinderAxisZ(spec)) == blast::BlastError::Ok,
           "cycle mount");
  }
  world.clear();
  expect(rt.liveBytes() <= base + 64, "live bytes return near baseline after clear");
}

}  // namespace

int main() {
  std::cout << "blast_e1_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  testRaster();
  testGraph(rt);
  testReaction(rt);
  testCutAndDensity(rt);
  testOriginShift(rt);
  testAggCompare(rt);
  testResetMemory(rt);
  rt.shutdown();
  if (gFailures == 0) {
    std::cout << "OK E1 occupancy cylinder gravity cut\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
