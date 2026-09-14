#include "blast/BlastMemory.h"
#include "blast/StructureWorld.h"
#include "physics/PhysicsTypes.h"

#include "NvBlastGlobals.h"

#include <cstdint>
#include <iostream>
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

struct TickLog {
  std::vector<uint64_t> ids;
  std::vector<float> dts;
  void operator()(uint64_t id, float dt) {
    ids.push_back(id);
    dts.push_back(dt);
  }
};

void testRuntimeLifecycle() {
  std::cout << "E0 runtime init / shutdown\n";
  blast::BlastRuntime rt;
  expect(!rt.initialized(), "runtime starts uninitialized");
  expect(rt.init(), "runtime init");
  expect(rt.initialized(), "runtime initialized");
  expect(rt.init(), "init is idempotent");
  const std::size_t baseline = rt.runtimeBaselineBytes();
  expect(rt.liveBytes() == baseline, "no instance bytes at init");
  expect(rt.errorCount() == 0, "no errors at init");
  rt.shutdown();
  expect(!rt.initialized(), "runtime shutdown");
  rt.shutdown();
  expect(!rt.initialized(), "shutdown is idempotent");
}

void testStructureWorldEmpty() {
  std::cout << "E0 empty StructureWorld\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  const std::size_t baseline = rt.liveBytes();
  blast::StructureWorld world;
  expect(!world.initialized(), "world starts uninitialized");
  expect(world.init(rt), "world init");
  expect(world.initialized(), "world initialized");
  expect(world.instanceCount() == 0, "zero instances");
  expect(world.physicsTicksReceived() == 0, "no ticks yet");
  expect(world.blastLiveBytes() == baseline, "init does not allocate instance memory");
  expect(world.blastRuntimeBaselineBytes() == rt.runtimeBaselineBytes(), "baseline visible");
  world.clear();
  world.clear();
  expect(world.initialized(), "clear keeps world initialized");
  expect(world.instanceCount() == 0, "clear leaves zero instances");
  world.shutdown();
  expect(!world.initialized(), "shutdown");
  world.onPhysicsTick(1, physics::kDt);
  expect(world.physicsTicksReceived() == 0, "ticks ignored after shutdown");
  rt.shutdown();
}

void testFixedStepClock() {
  std::cout << "E0 FixedStepClock matches PhysicsWorld::step rules\n";
  physics::FixedStepClock clock;
  TickLog log;
  expect(clock.advance(0.0f, log) == 0, "zero dt produces no ticks");
  expect(clock.advance(physics::kDt * 0.5f, log) == 0, "half step does not fire");
  expect(clock.advance(physics::kDt * 0.5f, log) == 1, "two half frames make one tick");
  expect(log.ids.size() == 1 && log.ids[0] == 1, "first tick id is 1");
  expect(log.dts[0] == physics::kDt, "notified dt is kDt not frameDt");

  log.ids.clear();
  log.dts.clear();
  expect(clock.advance(1.0f, log) == physics::kMaxStepsPerFrame, "long frame is capped");
  expect(log.ids.size() == static_cast<size_t>(physics::kMaxStepsPerFrame), "cap count");
  expect(log.ids[0] == 2 && log.ids[1] == 3, "ids stay monotonic across frames");
  for (float dt : log.dts) {
    expect(dt == physics::kDt, "capped frame still uses kDt");
  }

  clock.resetSession();
  expect(clock.tickId == 0 && clock.accumulator == 0.0f, "session reset");
  log.ids.clear();
  expect(clock.advance(physics::kDt, log) == 1, "after reset first tick is 1 again");
  expect(log.ids[0] == 1, "ids do not continue old session");
}

void testWorldReceivesClockTicks() {
  std::cout << "E0 StructureWorld receives FixedStepClock ticks\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  const std::size_t before = rt.liveBytes();
  blast::StructureWorld world;
  expect(world.init(rt), "world");
  physics::FixedStepClock clock;

  const int n0 = clock.advance(1.0f / 120.0f, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  expect(n0 == 0, "1/120 frame: no physics tick");
  expect(world.physicsTicksReceived() == 0, "structure not called on sub-dt frame");

  const int n1 = clock.advance(1.0f / 120.0f, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  expect(n1 == 1, "second 1/120 frame completes one tick");
  expect(world.physicsTicksReceived() == 1, "one structure call");
  expect(world.lastTickId() == 1, "last tick id 1");
  expect(world.lastDt() == physics::kDt, "last dt is kDt");

  const int n2 = clock.advance(1.0f / 30.0f, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
  expect(n2 == 2, "1/30 frame runs two ticks");
  expect(world.physicsTicksReceived() == 3, "total three calls");
  expect(world.lastTickId() == 3, "monotonic ids 1,2,3");
  expect(world.instanceCount() == 0, "still zero instances");
  expect(rt.liveBytes() == before, "empty ticks do not allocate");

  world.clear();
  expect(world.physicsTicksReceived() == 0 && world.lastTickId() == 0, "clear resets tick session");
  expect(world.initialized(), "runtime still alive after clear");
  rt.shutdown();
}

void testInitRequiresRuntime() {
  std::cout << "E0 StructureWorld init fails without runtime\n";
  blast::BlastRuntime rt;
  blast::StructureWorld world;
  expect(!world.init(rt), "init rejected before runtime.init");
  expect(!world.initialized(), "world not initialized");
}

void testRepeatedClearBaseline() {
  std::cout << "E0 repeated clear does not grow memory\n";
  blast::BlastRuntime rt;
  expect(rt.init(), "runtime");
  blast::StructureWorld world;
  expect(world.init(rt), "world");
  const std::size_t live0 = rt.liveBytes();
  const auto allocs0 = rt.allocator().liveAllocs();
  for (int i = 0; i < 50; ++i) {
    world.clear();
    physics::FixedStepClock clock;
    clock.advance(physics::kDt, [&](uint64_t id, float dt) { world.onPhysicsTick(id, dt); });
    world.clear();
  }
  expect(rt.liveBytes() == live0, "live bytes stable after 50 clear/tick cycles");
  expect(rt.allocator().liveAllocs() == allocs0, "live allocs stable");
  world.shutdown();
  rt.shutdown();
}

}  // namespace

int main() {
  std::cout << "blast_e0_tests NvBlast " << VE_NVBLAST_VERSION << " sha " << VE_NVBLAST_SHA << "\n";
  testRuntimeLifecycle();
  testStructureWorldEmpty();
  testFixedStepClock();
  testWorldReceivesClockTicks();
  testInitRequiresRuntime();
  testRepeatedClearBaseline();
  if (gFailures == 0) {
    std::cout << "OK E0 runtime StructureWorld FixedStepClock\n";
    return 0;
  }
  std::cerr << gFailures << " failure(s)\n";
  return 1;
}
