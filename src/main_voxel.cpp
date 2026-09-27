#include "blast/BlastMemory.h"
#include "blast/CylinderVoxels.h"
#include "blast/FrameVoxels.h"
#include "core/Window.h"
#include "gfx/GfxDevice.h"
#include "physics/PhysicsTypes.h"
#include "render/VoxelRenderer.h"
#include "scene/VoxelScene.h"

#include <GLFW/glfw3.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <windows.h>
#endif

#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <cstdio>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#ifdef _WIN32
extern "C" {
__declspec(dllexport) DWORD NvOptimusEnablement = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace {

struct Options {
  bool benchmark = false;
  bool e2Perf = false;
  bool frameFail = false;
  bool importObject = false;
  bool importTopple = false;
  bool e5Contact = false;
  bool collapsePerf = false;
  bool digConverge = false;
  bool warmRebuild = true;
  bool contactLoads = true;
  float solveBudgetMs = -1.0f;  // < 0: keep the scene default
  int collisionPerf = 0;  // >0: spawn N scatter boxes and time physics stages
  float frameHeightMeters = 4.0f;
  std::string frameImpactMode = "shear";
  int frameRenderHz = 60;
  bool frameGroundAnchors = false;  // --frame-anchors ground (E5.2)
  bool help = false;
  uint32_t frames = 300;
  uint32_t warmup = 60;
  int width = 2560;
  int height = 1440;
  bool color = false;  // Match the scene's default import, not Solid Color output.
  VoxelRenderer::BenchmarkSettings renderer{};
  // stampMeshIntoWorld centers the hut in X/Z, with its base at 2.5 * coarse voxelSize
  // (4.0 m at the default of 1.6 m coarse / 0.1 m fine). Ground top is 3.2 m.
  // Camera is in world meters. Yaw 0 looks along -Z; positive pitch looks down.
  std::array<float, 5> camera{4.0f, 5.0f, 6.0f, 0.67474094f, 0.0f};
  std::string capture;
  std::string csv;
};

Options parseOptions(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    auto value = [&]() -> std::string {
      if (++i >= argc || argv[i][0] == '\0') {
        throw std::runtime_error("Missing value for " + std::string(arg));
      }
      return argv[i];
    };
    auto integer = [&](int minimum, int maximum) {
      const std::string text = value();
      int result = 0;
      const auto parsed = std::from_chars(text.data(), text.data() + text.size(), result);
      if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size() ||
          result < minimum || result > maximum) {
        throw std::runtime_error("Invalid value for " + std::string(arg) + ": " + text);
      }
      return result;
    };
    if (arg == "--benchmark") {
      options.benchmark = true;
    } else if (arg == "--e2-perf") {
      options.e2Perf = true;
    } else if (arg == "--solve-budget") {
      options.solveBudgetMs = static_cast<float>(integer(0, 1000));
    } else if (arg == "--no-contact-loads") {
      options.contactLoads = false;
    } else if (arg == "--dig-converge") {
      options.digConverge = true;
    } else if (arg == "--no-warm-rebuild") {
      options.warmRebuild = false;
    } else if (arg == "--collapse-perf") {
      options.collapsePerf = true;
    } else if (arg == "--e5-contact") {
      options.e5Contact = true;
    } else if (arg == "--import-object") {
      options.importObject = true;
    } else if (arg == "--import-topple") {
      options.importTopple = true;
    } else if (arg == "--frame-fail") {
      options.frameFail = true;
    } else if (arg == "--collision-perf") {
      options.collisionPerf = integer(1, static_cast<int>(VoxelScene::kMaxVoxelObjects));
    } else if (arg == "--frame-height") {
      const std::string text = value();
      size_t parsed = 0;
      options.frameHeightMeters = std::stof(text, &parsed);
      if (parsed != text.size() || !std::isfinite(options.frameHeightMeters) ||
          options.frameHeightMeters < 1.0f || options.frameHeightMeters > 9.2f) {
        throw std::runtime_error("--frame-height must be between 1.0 and 9.2 meters");
      }
    } else if (arg == "--frame-impact") {
      options.frameImpactMode = value();
      if (options.frameImpactMode != "shear" && options.frameImpactMode != "spread" && options.frameImpactMode != "stress")
        throw std::runtime_error("--frame-impact must be shear, spread or stress");
    } else if (arg == "--frame-render-hz") {
      options.frameRenderHz = integer(30, 240);
    } else if (arg == "--frame-anchors") {
      const std::string mode = value();
      if (mode != "explicit" && mode != "ground") {
        throw std::runtime_error("--frame-anchors must be explicit or ground");
      }
      options.frameGroundAnchors = mode == "ground";
    } else if (arg == "--help" || arg == "-h") {
      options.help = true;
    } else if (arg == "--frames") {
      options.frames = static_cast<uint32_t>(integer(1, INT32_MAX));
    } else if (arg == "--warmup") {
      options.warmup = static_cast<uint32_t>(integer(0, INT32_MAX));
    } else if (arg == "--width") {
      options.width = integer(1, INT32_MAX);
    } else if (arg == "--height") {
      options.height = integer(1, INT32_MAX);
    } else if (arg == "--beam") {
      options.renderer.beam = integer(0, 1) != 0;
    } else if (arg == "--brick-skip") {
      options.renderer.brickSkip = integer(0, 1) != 0;
    } else if (arg == "--dir-brick") {
      options.renderer.dirBrick = integer(0, 1) != 0;
    } else if (arg == "--dir-coarse") {
      options.renderer.dirCoarse = integer(0, 1) != 0;
    } else if (arg == "--color") {
      options.color = integer(0, 1) != 0;
    } else if (arg == "--stage") {
      options.renderer.stage = static_cast<uint32_t>(integer(0, 5));
    } else if (arg == "--camera") {
      for (float& component : options.camera) {
        const std::string text = value();
        size_t parsed = 0;
        component = std::stof(text, &parsed);
        if (parsed != text.size() || !std::isfinite(component)) {
          throw std::runtime_error("Invalid finite camera component: " + text);
        }
      }
      if (std::abs(options.camera[4]) > 1.2f) {
        throw std::runtime_error("Camera pitch must be in [-1.2, 1.2] radians (no silent clamp)");
      }
    } else if (arg == "--capture") {
      options.capture = value();
    } else if (arg == "--csv") {
      options.csv = value();
    } else {
      throw std::runtime_error("Unknown option: " + std::string(arg));
    }
  }
  if (options.benchmark && options.e2Perf) {
    throw std::runtime_error("Use either --benchmark or --e2-perf, not both");
  }
  if (argc > 1 && !options.benchmark && !options.e2Perf && !options.frameFail && !options.importObject && !options.importTopple && !options.e5Contact && !options.collapsePerf && !options.digConverge && options.collisionPerf == 0 &&
      !options.help) {
    throw std::runtime_error("Rendering options require --benchmark; use --help for usage");
  }
  auto outputPath = [](const std::string& text) -> std::filesystem::path {
    if (text.empty()) {
      return {};
    }
    const auto path = std::filesystem::absolute(text);
    const auto parent = std::filesystem::weakly_canonical(path.parent_path());
    if (!std::filesystem::is_directory(parent)) {
      throw std::runtime_error("Output parent directory does not exist: " + parent.string());
    }
    return (parent / path.filename()).lexically_normal().make_preferred();
  };
  const auto capturePath = outputPath(options.capture);
  const auto csvPath = outputPath(options.csv);
  if (!capturePath.empty() && !csvPath.empty()) {
    bool samePath = capturePath == csvPath;
#ifdef _WIN32
    // Ordinal case folding matches Windows names without locale-dependent lowercasing.
    const int comparison = CompareStringOrdinal(capturePath.c_str(), -1, csvPath.c_str(), -1, TRUE);
    if (comparison == 0) {
      throw std::runtime_error("Cannot compare benchmark output paths");
    }
    samePath = comparison == CSTR_EQUAL;
#endif
    std::error_code error;
    const bool sameFile = std::filesystem::equivalent(capturePath, csvPath, error);
    if (error && error != std::errc::no_such_file_or_directory) {
      throw std::filesystem::filesystem_error("Cannot compare benchmark output files",
                                              capturePath, csvPath, error);
    }
    if (samePath || sameFile) {
      throw std::runtime_error("Capture and CSV must use different output paths");
    }
  }
  return options;
}

void sizeBenchmarkWindow(Window& window, const Options& options) {
  // A borderless window avoids work-area constraints and the outer/client-size mismatch.
  glfwSetWindowAttrib(window.handle(), GLFW_DECORATED, GLFW_FALSE);
  glfwSetWindowAttrib(window.handle(), GLFW_RESIZABLE, GLFW_FALSE);
#ifdef _WIN32
  const HWND hwnd = glfwGetWin32Window(window.handle());
  if (!hwnd || !SetWindowPos(hwnd, HWND_TOP, 0, 0, options.width, options.height,
                             SWP_SHOWWINDOW | SWP_FRAMECHANGED)) {
    throw std::runtime_error("Cannot size the Windows benchmark client area");
  }
#else
  glfwSetWindowPos(window.handle(), 0, 0);
#endif
  // GLFW uses screen coordinates, which need not equal framebuffer pixels on HiDPI.
  for (int attempt = 0; attempt < 8; ++attempt) {
    window.pollEvents();
    const VkExtent2D extent = window.framebufferExtent();
    if (extent.width == static_cast<uint32_t>(options.width) &&
        extent.height == static_cast<uint32_t>(options.height)) {
      window.clearResizedFlag();
      return;
    }
    if (extent.width == 0 || extent.height == 0 || window.shouldClose()) {
      throw std::runtime_error("Benchmark window is closed or has a zero-sized framebuffer");
    }
    int width = 0;
    int height = 0;
    glfwGetWindowSize(window.handle(), &width, &height);
    glfwSetWindowSize(window.handle(),
                     std::max(1, static_cast<int>(std::lround(
                         double(width) * options.width / extent.width))),
                     std::max(1, static_cast<int>(std::lround(
                         double(height) * options.height / extent.height))));
    glfwWaitEventsTimeout(0.05);
  }
  const VkExtent2D extent = window.framebufferExtent();
  throw std::runtime_error("Cannot obtain requested benchmark framebuffer " +
                           std::to_string(options.width) + "x" + std::to_string(options.height) +
                           "; actual " + std::to_string(extent.width) + "x" +
                           std::to_string(extent.height));
}

void runBenchmark(const Options& options, Window& window, GfxDevice& gfx, VoxelScene& scene) {
  auto checkExtent = [&]() {
    const VkExtent2D fb = window.framebufferExtent();
    const VkExtent2D swap = gfx.swapchainExtent();
    if (fb.width != static_cast<uint32_t>(options.width) ||
        fb.height != static_cast<uint32_t>(options.height) ||
        swap.width != fb.width || swap.height != fb.height) {
      throw std::runtime_error("Benchmark framebuffer/swapchain no longer matches requested size");
    }
  };
  checkExtent();
  std::ofstream csv;
  if (!options.csv.empty()) {
    csv.open(options.csv, std::ios::trunc);
    csv.imbue(std::locale::classic());
    if (!csv) {
      throw std::runtime_error("Cannot open benchmark CSV: " + options.csv);
    }
  }
  if (scene.importPath().empty()) {
    throw std::runtime_error("Benchmark pirate hut is missing: scene.importPath() is empty");
  }
  scene.spinnerEnabled() = false;
  scene.spinSpeed() = 0.0f;
  scene.nestedMicroVoxels() = true;
  scene.nestedFineVoxels() = true;
  scene.solidColorOutput() = false;
  scene.renderMode() = 0;
  MeshVoxelizeConfig config;
  config.gridN = 64;
  config.padding = 1;
  config.sampleColor = options.color;
  const uint32_t initialFines = scene.occupiedFineCount();
  if (!scene.importSurfaceMesh(gfx, scene.importPath(), config)) {
    throw std::runtime_error("Benchmark import failed: " + scene.importStatus());
  }
  if (scene.occupiedFineCount() <= initialFines) {
    throw std::runtime_error("Benchmark import did not add occupied fine voxels to the world");
  }
  const auto& pose = options.camera;
  // setYawPitch also moves the orbit camera, so set the exact position afterward.
  scene.camera().setYawPitch(pose[3], pose[4]);
  scene.camera().setPosition(glm::vec3(pose[0], pose[1], pose[2]));
  scene.camera().setPerspective(60.0f, float(options.width) / float(options.height), 0.1f, 200.0f);

  VoxelRenderer renderer(gfx);
  renderer.configureBenchmark(options.renderer, options.warmup, options.frames);
  renderer.init(scene);
  VkPhysicalDeviceProperties gpu{};
  vkGetPhysicalDeviceProperties(gfx.physicalDevice(), &gpu);
  const char* quality = options.renderer.stage == 0 ? "full" : "diagnostic_not_full_quality";
  std::cout << std::fixed << std::setprecision(9)
            << "Benchmark quality=" << quality << " GPU=" << gfx.deviceName()
            << " vendor=" << gpu.vendorID << " device=" << gpu.deviceID
            << " driver=" << gpu.driverVersion << " api=" << gpu.apiVersion << '\n'
            << "Framebuffer=" << options.width << 'x' << options.height
            << " swapchain=" << gfx.swapchainExtent().width << 'x' << gfx.swapchainExtent().height
            << " format=" << gfx.swapchainFormat() << " present=" << gfx.presentModeName()
            << " frames_in_flight=" << gfx.framesInFlight()
            << " timestamp_period_ns=" << gpu.limits.timestampPeriod << '\n'
            << "Camera=" << pose[0] << ' ' << pose[1] << ' ' << pose[2] << ' ' << pose[3]
            << ' ' << pose[4] << " (xyz yaw pitch; radians), fov_y=60 near=0.1 far=200\n"
            << "Import=" << scene.importPath() << " grid=" << config.gridN
            << " padding=" << config.padding << " color=" << options.color
            << " conservative=" << config.conservative << '\n'
            << "Scene grid=" << scene.gridSize() << " voxel_size=" << scene.voxelSize()
            << " gameplay_voxel=" << scene.gameplayVoxelSize()
            << " fine_voxel=" << scene.fineVoxelSize()
            << " objects=" << scene.objectCount() << " occupied_coarse=" << scene.occupiedCount()
            << '/' << scene.voxelCount() << " occupied_micro=" << scene.occupiedMicroCount()
            << " occupied_fine=" << scene.occupiedFineCount()
            << " brick_pages=" << scene.allocatedBrickPages() << " brick_slabs=" << scene.brickSlabCount()
            << " brick_bytes=" << scene.brickPoolBytes() << " occ_mip_bytes=" << scene.occMipBytes() << '\n'
            << "Settings beam=" << options.renderer.beam << " brick_skip=" << options.renderer.brickSkip
            << " dir_brick=" << options.renderer.dirBrick << " dir_coarse=" << options.renderer.dirCoarse
            << " beam_margin=" << renderer.beamMargin() << " stage=" << options.renderer.stage
            << " nested_micro=" << scene.nestedMicroVoxels() << " nested_fine=" << scene.nestedFineVoxels()
            << " ao_strength=" << scene.aoStrength() << " ao_power=" << scene.aoPower()
            << " max_steps=" << scene.maxSteps() << " solid_color=" << scene.solidColorOutput()
            << " collapse_full_bricks=" << scene.collapseFullBricks() << '\n'
            << "Lighting dir=" << scene.lightDir().x << ' ' << scene.lightDir().y << ' ' << scene.lightDir().z
            << " ambient=" << scene.ambient() << " sky=" << scene.showSky()
            << " sky_texture=" << scene.sky().image.extent.width << 'x' << scene.sky().image.extent.height
            << " sky_yaw=" << scene.skyYaw() << " sky_intensity=" << scene.skyIntensity() << '\n'
            << "Warmup=" << options.warmup << " measured=" << options.frames
            << " UI=normal_locked input=off simulation=off spinner=off\n"
#ifdef VE_ENABLE_VALIDATION
            << "Validation build=enabled\n"
#else
            << "Validation build=disabled\n"
#endif
            << "GPU samples include beam + main + blit + UI, not CPU/presentation latency.\n"
            << "Capture/readback runs only after all measured submissions and timestamp collection."
            << std::endl;

  auto last = std::chrono::steady_clock::now();
  float fps = 60.0f;
  const uint64_t totalFrames = uint64_t{options.warmup} + options.frames;
  for (uint64_t submitted = 0; submitted < totalFrames; ++submitted) {
    window.pollEvents();
    if (window.shouldClose()) {
      throw std::runtime_error("Benchmark interrupted before all frames were submitted");
    }
    checkExtent();
    const auto now = std::chrono::steady_clock::now();
    const float dt = std::chrono::duration<float>(now - last).count();
    last = now;
    fps = fps * 0.9f + (dt > 0.0f ? 1.0f / dt : fps) * 0.1f;
    // No camera input, edit input, or scene.update: geometry and transforms stay fixed.
    if (!renderer.draw(scene, fps) || gfx.swapchainWasRecreated()) {
      throw std::runtime_error("Benchmark frame skipped or swapchain recreated; rerun at a stable size");
    }
    checkExtent();
  }
  const auto timings = renderer.finishBenchmark();
  using Timing = VoxelRenderer::GpuTiming;
  const std::array<double Timing::*, 6> fields{
      &Timing::totalMs, &Timing::beamMs, &Timing::mainMs,
      &Timing::computeMs, &Timing::blitMs, &Timing::uiMs};
  const std::array<const char*, 6> names{"total", "beam", "main", "compute", "blit", "ui"};
  Timing median;
  Timing p95;
  for (size_t stage = 0; stage < fields.size(); ++stage) {
    const auto field = fields[stage];
    std::vector<double> sorted;
    sorted.reserve(timings.size());
    for (const auto& timing : timings) {
      if (!std::isfinite(timing.*field) || timing.*field < 0.0) {
        throw std::runtime_error("Invalid raw GPU timing sample");
      }
      sorted.push_back(timing.*field);
    }
    std::sort(sorted.begin(), sorted.end());
    const size_t n = sorted.size();
    median.*field = (sorted[(n - 1) / 2] + sorted[n / 2]) * 0.5;
    p95.*field = sorted[static_cast<size_t>(std::ceil(0.95 * double(n))) - 1];
    std::cout << "GPU " << names[stage] << " ms: median=" << median.*field
              << " p95=" << p95.*field << '\n';
  }
  std::cout << "Samples=" << timings.size() << " (raw timestamps, no EMA; p95 nearest-rank)\n";
  if (csv.is_open()) {
    auto quoted = [](const std::string& text) {
      std::string result = "\"";
      for (char c : text) {
        if (c == '"') result += '"';
        result += c;
      }
      return result + '"';
    };
    csv << "kind,sample,submission,total_ms,beam_ms,main_ms,compute_ms,blit_ms,ui_ms,"
           "quality,width,height,warmup,frames,beam,brick_skip,dir_brick,dir_coarse,color,stage,"
           "camera_x,camera_y,camera_z,yaw,pitch,objects,occupied_coarse,occupied_micro,occupied_fine,"
           "brick_pages,brick_slabs,gpu,driver_version,present_mode,import_path,import_grid,import_padding,"
           "nested_micro,nested_fine,ao_strength,ao_power,max_steps,beam_margin,"
           "effective_kernel,brick_backend,pipeline_statistics\n";
    csv << std::fixed << std::setprecision(9);
    auto row = [&](const char* kind, const Timing& timing, bool sample) {
      csv << kind << ',';
      if (sample) csv << timing.submission - options.warmup;
      csv << ',';
      if (sample) csv << timing.submission;
      for (auto field : fields) csv << ',' << timing.*field;
      csv << ',' << quality << ',' << options.width << ',' << options.height << ',' << options.warmup
          << ',' << options.frames << ',' << options.renderer.beam << ',' << options.renderer.brickSkip
          << ',' << options.renderer.dirBrick << ',' << options.renderer.dirCoarse << ',' << options.color
          << ',' << options.renderer.stage;
      for (float component : pose) csv << ',' << component;
      csv << ',' << scene.objectCount() << ',' << scene.occupiedCount() << ',' << scene.occupiedMicroCount()
          << ',' << scene.occupiedFineCount() << ',' << scene.allocatedBrickPages() << ',' << scene.brickSlabCount()
          << ',' << quoted(gfx.deviceName()) << ',' << gpu.driverVersion << ',' << gfx.presentModeName()
          << ',' << quoted(scene.importPath()) << ',' << config.gridN << ',' << config.padding
          << ',' << scene.nestedMicroVoxels() << ',' << scene.nestedFineVoxels() << ',' << scene.aoStrength()
          << ',' << scene.aoPower() << ',' << scene.maxSteps() << ',' << renderer.beamMargin()
          << ',' << quoted(renderer.activeKernelName())
          << ',' << quoted(gfx.storageBufferNonUniformIndexing() ? "nonuniform indexed" : "portable")
          << ',' << gfx.pipelineExecutableStatisticsEnabled() << '\n';
    };
    for (const auto& timing : timings) row("frame", timing, true);
    row("median", median, false);
    row("p95", p95, false);
    csv.close();
    if (!csv) throw std::runtime_error("Failed to write benchmark CSV: " + options.csv);
  }
  if (!options.capture.empty()) {
    renderer.capturePpm(options.capture);
    std::cout << "Capture=" << options.capture << " (P6 RGB from RGBA8, pre-UI, top-to-bottom)\n";
  }
  std::cout << "Benchmark complete: " << quality << std::endl;
}

struct E2PerfSample {
  double updateMs = 0;
  double commitMs = 0;
  float solveMs = 0;
  float probeMs = 0;
  float statsMs = 0;
  float candidateMs = 0;
  float collideMs = 0;
  float contactSolveMs = 0;
  float integrateMs = 0;
  float rebuildMs = 0;
  float structureCbMs = 0;
  int awake = 0;
  int occupied = 0;
  int contacts = 0;
  uint32_t candidates = 0;
  uint32_t actors = 1;
  uint32_t probeExports = 0;
};

struct PerfLog {
  FILE* f = nullptr;
  explicit PerfLog(const std::filesystem::path& path) {
    f = std::fopen(path.string().c_str(), "w");
    if (f != nullptr) {
      std::setvbuf(f, nullptr, _IONBF, 0);
    }
  }
  PerfLog(const PerfLog&) = delete;
  PerfLog& operator=(const PerfLog&) = delete;
  ~PerfLog() {
    if (f != nullptr) {
      std::fflush(f);
      std::fclose(f);
      f = nullptr;
    }
  }
  void line(const std::string& s) {
    if (f != nullptr) {
      std::fwrite(s.data(), 1, s.size(), f);
      std::fputc('\n', f);
      std::fflush(f);
    }
#ifdef _WIN32
    OutputDebugStringA(s.c_str());
    OutputDebugStringA("\n");
#endif
  }
};

void printE2PerfPhase(PerfLog& out, const char* name, const std::vector<E2PerfSample>& samples) {
  auto med = [](std::vector<double> v) -> double {
    if (v.empty()) {
      return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return 0.5 * (v[(n - 1) / 2] + v[n / 2]);
  };
  std::vector<double> update, commit, solve, probe, stats, cand, collide, contact, integ, rebuild, structCb, engine;
  update.reserve(samples.size());
  for (const E2PerfSample& s : samples) {
    update.push_back(s.updateMs);
    commit.push_back(s.commitMs);
    solve.push_back(s.solveMs);
    probe.push_back(s.probeMs);
    stats.push_back(s.statsMs);
    cand.push_back(s.candidateMs);
    collide.push_back(s.collideMs);
    contact.push_back(s.contactSolveMs);
    integ.push_back(s.integrateMs);
    rebuild.push_back(s.rebuildMs);
    structCb.push_back(s.structureCbMs);
    engine.push_back(s.updateMs - static_cast<double>(s.structureCbMs) + s.commitMs);
  }
  E2PerfSample last{};
  if (!samples.empty()) {
    last = samples.back();
  }
  std::ostringstream row;
  row << std::fixed << std::setprecision(3);
  row << "## D " << name << "  n=" << samples.size() << "  occupiedBodies=" << last.occupied
      << " awake=" << last.awake << " contacts=" << last.contacts << " cand=" << last.candidates
      << " actors=" << last.actors << " exports=" << last.probeExports;
  out.line(row.str());
  row.str(std::string());
  row.clear();
  row << std::fixed << std::setprecision(3);
  row << "median update=" << med(update) << "  commit=" << med(commit) << "  A solve=" << med(solve)
      << "  B probe=" << med(probe) << "  C stats=" << med(stats) << " cand=" << med(cand);
  out.line(row.str());
  row.str(std::string());
  row.clear();
  row << std::fixed << std::setprecision(3);
  row << "median collide=" << med(collide) << "  contactSolve=" << med(contact)
      << "  integrate=" << med(integ) << "  rebuild=" << med(rebuild) << "  struct-cb=" << med(structCb);
  out.line(row.str());
  row.str(std::string());
  row.clear();
  row << std::fixed << std::setprecision(3);
  row << "median engine-without-structure (D)=" << med(engine) << " ms / tick";
  out.line(row.str());
}

struct E2FpsSample {
  double frameMs = 0;
  double updateMs = 0;
  double commitMs = 0;
  double drawMs = 0;
  float solveMs = 0;
  float probeMs = 0;
  float statsMs = 0;
  float candidateMs = 0;
  float collideMs = 0;
  float structureCbMs = 0;
  uint32_t candidates = 0;
  uint32_t actors = 1;
  uint32_t fractured = 0;
  uint32_t exports = 0;
  int awake = 0;
  int contacts = 0;
};

void printE2FpsPhase(PerfLog& out, const char* name, const std::vector<E2FpsSample>& samples) {
  auto med = [](std::vector<double> v) -> double {
    if (v.empty()) {
      return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return 0.5 * (v[(n - 1) / 2] + v[n / 2]);
  };
  auto p95 = [](std::vector<double> v) -> double {
    if (v.empty()) {
      return 0.0;
    }
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return v[static_cast<size_t>(std::ceil(0.95 * static_cast<double>(n))) - 1];
  };
  std::vector<double> frame, update, commit, draw, solve, probe, collide, structCb;
  frame.reserve(samples.size());
  for (const E2FpsSample& s : samples) {
    frame.push_back(s.frameMs);
    update.push_back(s.updateMs);
    commit.push_back(s.commitMs);
    draw.push_back(s.drawMs);
    solve.push_back(s.solveMs);
    probe.push_back(s.probeMs);
    collide.push_back(s.collideMs);
    structCb.push_back(s.structureCbMs);
  }
  E2FpsSample last{};
  if (!samples.empty()) {
    last = samples.back();
  }
  const double medFrame = med(frame);
  const double fps = medFrame > 1e-6 ? 1000.0 / medFrame : 0.0;
  const double fpsP95 = p95(frame) > 1e-6 ? 1000.0 / p95(frame) : 0.0;
  const double maxFrame = frame.empty() ? 0.0 : *std::max_element(frame.begin(), frame.end());
  std::ostringstream row;
  row << std::fixed << std::setprecision(2);
  row << "## FPS " << name << "  n=" << samples.size() << "  median=" << fps << " fps  p95=" << fpsP95
      << " fps  frame=" << medFrame << " ms (p95 " << p95(frame) << " max " << maxFrame << ")";
  out.line(row.str());
  row.str(std::string());
  row.clear();
  row << std::fixed << std::setprecision(2);
  row << "  update=" << med(update) << "  commit=" << med(commit) << "  draw=" << med(draw)
      << "  solve=" << med(solve) << "  probe=" << med(probe) << "  collide=" << med(collide)
      << "  struct-cb=" << med(structCb);
  out.line(row.str());
  row.str(std::string());
  row.clear();
  row << "  last cand=" << last.candidates << " actors=" << last.actors << " fractured=" << last.fractured
      << " exports=" << last.exports << " awake=" << last.awake << " contacts=" << last.contacts;
  out.line(row.str());
}

void runE2PerfLiveFps(Window& window, GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  out.line("live FPS: 1280x720 DDA + ImGui, wall-clock dt (same loop as the demo)");
  out.line(std::string("present=") + gfx.presentModeName());
  VoxelRenderer renderer(gfx);
  renderer.init(scene);
  const float aspect = (gfx.swapchainExtent().height > 0)
                           ? static_cast<float>(gfx.swapchainExtent().width) /
                                 static_cast<float>(gfx.swapchainExtent().height)
                           : 16.0f / 9.0f;
  scene.camera().update(aspect);

  auto last = std::chrono::steady_clock::now();
  auto step = [&]() -> E2FpsSample {
    window.pollEvents();
    const auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last).count();
    last = now;
    if (!(dt > 0.0f) || dt > 0.25f) {
      dt = physics::kDt;
    }
    E2FpsSample s;
    const auto t0 = std::chrono::steady_clock::now();
    scene.camera().update(aspect);
    scene.update(dt);
    const auto t1 = std::chrono::steady_clock::now();
    scene.commitStructureSplits(gfx);
    const auto t2 = std::chrono::steady_clock::now();
    const float fps = dt > 0.0f ? 1.0f / dt : 0.0f;
    (void)renderer.draw(scene, fps);
    const auto t3 = std::chrono::steady_clock::now();
    s.frameMs = std::chrono::duration<double, std::milli>(t3 - t0).count();
    s.updateMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    s.commitMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    s.drawMs = std::chrono::duration<double, std::milli>(t3 - t2).count();
    const blast::StructureDebugSnapshot& st = scene.structures().debug(scene.stressCylinderId());
    const physics::DebugSolve& ds = scene.physicsDebug();
    s.solveMs = st.solveMs;
    s.probeMs = st.probeMs;
    s.statsMs = st.statsMs;
    s.candidateMs = st.candidateMs;
    s.collideMs = ds.collideMs;
    s.structureCbMs = ds.structureCallbackMs;
    s.candidates = st.candidateCount;
    s.actors = st.splitActors;
    s.fractured = st.fracturedBonds;
    s.exports = st.probeExportCount;
    s.awake = ds.awakeBodies;
    s.contacts = ds.contacts;
    return s;
  };

  auto measure = [&](int warmup, int frames, const char* name) {
    for (int i = 0; i < warmup; ++i) {
      (void)step();
    }
    std::vector<E2FpsSample> samples;
    samples.reserve(static_cast<size_t>(frames));
    for (int i = 0; i < frames; ++i) {
      samples.push_back(step());
    }
    printE2FpsPhase(out, name, samples);
  };

  scene.setStressCylinderSolverIters(200);
  scene.structures().setFractureEnabled(false);
  scene.structures().setStrengthPa(blast::kE2StrengthHoldPa);
  measure(10, 30, "intact, fracture off");

  // Panel order the user hits: Stress fracture, Fail strength, then Cut 270.
  scene.structures().setFractureEnabled(true);
  scene.structures().setStrengthPa(blast::kE2StrengthFailPa);
  measure(10, 30, "intact, fracture + fail, before cut");

  if (!scene.cutStressCylinder270(gfx)) {
    throw std::runtime_error("cutStressCylinder270 failed in FPS run");
  }
  {
    std::vector<E2FpsSample> cascade;
    bool split = false;
    for (int i = 0; i < 45; ++i) {
      cascade.push_back(step());
      if (cascade.back().actors > 1) {
        split = true;
      }
    }
    printE2FpsPhase(out, split ? "cut + colors + fracture + fail cascade" : "cut + colors + fracture + fail (no split)",
                    cascade);
  }
  measure(10, 90, "cut + colors + fracture + fail after cascade");
  out.line("OK e2-perf live FPS");
}

void runE2PerfEngine(Window& window, GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  out.line("blast e2-perf D: same cylinder in VoxelScene + PhysicsWorld (no DDA draw)");
  out.line("spawn...");
  if (!scene.spawnStressCylinder(gfx)) {
    throw std::runtime_error("spawnStressCylinder failed");
  }
  out.line("spawn ok");
  scene.setStressCylinderSolverIters(200);
  scene.structures().setFractureEnabled(false);
  scene.structures().setStrengthPa(blast::kE2StrengthHoldPa);

  auto drive = [&](int frames) -> std::vector<E2PerfSample> {
    std::vector<E2PerfSample> samples;
    samples.reserve(static_cast<size_t>(frames));
    for (int i = 0; i < frames; ++i) {
      window.pollEvents();
      E2PerfSample s;
      const auto t0 = std::chrono::steady_clock::now();
      scene.update(physics::kDt);
      const auto t1 = std::chrono::steady_clock::now();
      scene.commitStructureSplits(gfx);
      const auto t2 = std::chrono::steady_clock::now();
      s.updateMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
      s.commitMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
      const blast::StructureDebugSnapshot& st = scene.structures().debug(scene.stressCylinderId());
      const physics::DebugSolve& ds = scene.physicsDebug();
      s.solveMs = st.solveMs;
      s.probeMs = st.probeMs;
      s.statsMs = st.statsMs;
      s.candidateMs = st.candidateMs;
      s.collideMs = ds.collideMs;
      s.contactSolveMs = ds.contactSolveMs;
      s.integrateMs = ds.integrateMs;
      s.rebuildMs = ds.rebuildMs;
      s.structureCbMs = ds.structureCallbackMs;
      s.awake = ds.awakeBodies;
      s.occupied = ds.occupiedBodies;
      s.contacts = ds.contacts;
      s.candidates = st.candidateCount;
      s.actors = st.splitActors;
      s.probeExports = st.probeExportCount;
      samples.push_back(s);
    }
    return samples;
  };

  out.line("warmup intact...");
  (void)drive(4);
  printE2PerfPhase(out, "intact standing, fracture off", drive(12));

  out.line("cut 270...");
  if (!scene.cutStressCylinder270(gfx)) {
    throw std::runtime_error("cutStressCylinder270 failed");
  }
  out.line("cut ok");
  scene.structures().setFractureEnabled(false);
  scene.structures().setStrengthPa(blast::kE2StrengthHoldPa);
  (void)drive(3);
  printE2PerfPhase(out, "cut, fracture off", drive(8));

  scene.structures().setStrengthPa(blast::kE2StrengthFailPa);
  scene.structures().setFractureEnabled(true);
  out.line("fracture on, wait split...");
  std::vector<E2PerfSample> untilSplit;
  bool split = false;
  for (int i = 0; i < 16; ++i) {
    auto one = drive(1);
    untilSplit.insert(untilSplit.end(), one.begin(), one.end());
    if (!untilSplit.empty() && untilSplit.back().actors > 1) {
      split = true;
      break;
    }
  }
  printE2PerfPhase(out, "cut + fail-S until first split", untilSplit);
  out.line(std::string("split=") + (split ? "yes" : "no"));
  if (split) {
    printE2PerfPhase(out, "fragments after split", drive(12));
  }
  out.line("OK e2-perf D");
  if (!scene.resetStressCylinder(gfx)) {
    throw std::runtime_error("resetStressCylinder failed before FPS run");
  }
  runE2PerfLiveFps(window, gfx, scene, out);
}

void appendCrashLog(const char* message) {
  std::ofstream log("vulkan_engine_voxel_crash.log", std::ios::app);
  if (!log) {
    return;
  }
  log << message << '\n';
}

void showFatal(const char* message, bool dialogs) {
  std::cerr << "Fatal: " << message << '\n';
  appendCrashLog(message);
#ifdef _WIN32
  if (dialogs) {
    MessageBoxA(nullptr, message, "Vulkan Engine Voxel", MB_OK | MB_ICONERROR);
  }
#else
  (void)dialogs;
#endif
}

}  // namespace

// Many-body collision stress: N scatter boxes dropped in a stacked grid onto the
// ground, timed per fixed tick. Covers the fall, the pile-up and settling.
void runCollisionPerf(GfxDevice& gfx, VoxelScene& scene, int count) {
  const std::filesystem::path reportPath =
      std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" / "collision-perf.txt";
  FILE* report = std::fopen(reportPath.string().c_str(), "a");
  if (report == nullptr) {
    throw std::runtime_error("Cannot open " + reportPath.string());
  }
  scene.spawnScatterBoxes(gfx, static_cast<uint32_t>(count), /*pile=*/true);
  scene.setSimulate(gfx, true);
  struct Row { double tick, broad, sync, narrow, solver, record, sleep; int awake, contacts, pairs; };
  std::vector<Row> rows;
  constexpr int kTicks = 360;
  for (int i = 0; i < kTicks; ++i) {
    const auto t0 = std::chrono::steady_clock::now();
    scene.update(physics::kDt);
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const auto& d = scene.physicsDebug();
    if (std::getenv("VE_COLLISION_PERF_TRACE") != nullptr && i % 10 == 0) {
      float minY = 1e30f, maxY = -1e30f;
      for (int s = 0; s < scene.cpuObjectCount(); ++s) {
        const VoxelObject& o = scene.cpuObject(s);
        if (o.slotOccupied && o.isScatter) { minY = std::min(minY, o.position.y); maxY = std::max(maxY, o.position.y); }
      }
      std::fprintf(report, "  tick %d y=[%.2f,%.2f] contacts=%d awake=%d pairs=%d\n", i, minY, maxY, d.contacts,
                   d.awakeBodies, d.narrowPhasePairs);
    }
    rows.push_back({ms, d.broadPhaseMs, d.broadSyncMs, d.narrowPhaseMs, d.solverOnlyMs, d.contactRecordMs, d.sleepMs,
                    d.awakeBodies, d.contacts, d.narrowPhasePairs});
  }
  auto stat = [&](auto field, int from, int to) {
    std::vector<double> v;
    for (int i = from; i < to; ++i)
      if (rows[static_cast<size_t>(i)].awake > 0) v.push_back(field(rows[static_cast<size_t>(i)]));
    if (v.empty()) return std::array<double, 3>{0, 0, 0};
    std::sort(v.begin(), v.end());
    return std::array<double, 3>{v[v.size() / 2], v[static_cast<size_t>(0.95 * (v.size() - 1))], v.back()};
  };
  auto phase = [&](const char* name, int from, int to) {
    auto t = stat([](const Row& r) { return r.tick; }, from, to);
    auto n = stat([](const Row& r) { return r.narrow; }, from, to);
    auto b = stat([](const Row& r) { return r.broad; }, from, to);
    auto sy = stat([](const Row& r) { return r.sync; }, from, to);
    auto s = stat([](const Row& r) { return r.solver; }, from, to);
    auto rc = stat([](const Row& r) { return r.record; }, from, to);
    auto sl = stat([](const Row& r) { return r.sleep; }, from, to);
    auto aw = stat([](const Row& r) { return static_cast<double>(r.awake); }, from, to);
    auto ct = stat([](const Row& r) { return static_cast<double>(r.contacts); }, from, to);
    std::fprintf(report, "N=%d %-8s tick med=%.2f p95=%.2f max=%.2f | narrow %.2f/%.2f broad %.2f/%.2f (sync %.2f) "
                "solver %.2f/%.2f record %.2f/%.2f sleep %.2f/%.2f | awake~%.0f contacts~%.0f\n",
                count, name, t[0], t[1], t[2], n[0], n[1], b[0], b[1], sy[0], s[0], s[1], rc[0], rc[1], sl[0], sl[1],
                aw[0], ct[0]);
  };
  // Only ticks with awake bodies: once everything sleeps a tick costs ~nothing.
  int active = 0;
  double total = 0;
  for (const Row& r : rows) {
    if (r.awake > 0) ++active;
    total += r.tick;
  }
  phase("active", 0, kTicks);
  std::fprintf(report, "N=%d active ticks=%d total=%.1f ms\n", count, active, total);
  std::fclose(report);
}

void runFrameFail(GfxDevice& gfx, VoxelScene& scene, PerfLog& out, const Options& options) {
  auto dump = [&](const char* tag) {
    const blast::StructureDebugSnapshot& st = scene.structures().debug(scene.stressCylinderId());
    int enabled = 0;
    for (int i = 0; i < scene.cpuObjectCount(); ++i) {
      const VoxelObject& o = scene.cpuObject(i);
      if (o.slotOccupied && o.enabled) {
        ++enabled;
      }
    }
    const blast::StructureInstance* inst = scene.stressStructure();
    char line[512];
    std::snprintf(line, sizeof(line),
                  "%s conv=%d status=%s strip=%.4g S=%.4g cand=%u fracBonds=%u actors=%u "
                  "frac=%d keepBox=%d cut=%d enabledObj=%d nodes=%u bonds=%u",
                  tag, st.converged ? 1 : 0, st.status, st.stripMaxStress, st.strengthPa,
                  st.candidateCount, st.fracturedBonds, st.splitActors, st.fractureEnabled ? 1 : 0,
                  (inst && inst->diag.kind == blast::DiagnosticProfile::Kind::ColumnBox) ? 1 : 0, st.cut ? 1 : 0, enabled, st.nodes, st.bonds);
    out.line(line);
    const auto& phys = scene.physicsDebug();
    std::snprintf(line, sizeof(line), "contact-cache narrow=%d reuse=%d warm=%d collideMs=%.4f solveMs=%.4f",
                  phys.narrowPhasePairs, phys.reusedContactPairs, phys.warmStartedPoints,
                  phys.collideMs, phys.contactSolveMs);
    out.line(line);
    std::snprintf(line, sizeof(line), "physics-stages broad=%.4f narrow=%.4f solver=%.4f record=%.4f structure=%.4f rebuild=%.4f awake=%d contacts=%d",
                  phys.broadPhaseMs, phys.narrowPhaseMs, phys.solverOnlyMs, phys.contactRecordMs,
                  phys.structureCallbackMs, phys.rebuildMs, phys.awakeBodies, phys.contacts);
    out.line(line);
  };
  auto ticks = [&](int n) {
    for (int i = 0; i < n; ++i) {
      scene.update(physics::kDt);
    }
  };

  auto impact = scene.structures().impactSettings();
  impact.shearDamage = options.frameImpactMode != "spread";
  scene.structures().setImpactSettings(impact);
  scene.structures().setStressImpactImpulses(options.frameImpactMode == "stress");
  scene.setGroundContactAnchors(options.frameGroundAnchors);
  if (!scene.spawnStressFrame(gfx, options.frameHeightMeters)) {
    throw std::runtime_error("spawnStressFrame failed");
  }
  ticks(24);
  out.line("columnHeightMeters=" + std::to_string(scene.frameColumnHeightMeters()));
  out.line("impactMode=" + options.frameImpactMode + " renderHz=" + std::to_string(options.frameRenderHz));
  if (options.frameGroundAnchors) {
    const blast::GroundAnchorResult& anchors = scene.lastGroundAnchors();
    out.line("anchors=ground fines=" + std::to_string(anchors.anchorFines) + " faces=" +
             std::to_string(anchors.contactFaces) + " blocked=" + std::to_string(anchors.blockedFaces) + " " +
             scene.structureMountStatus());
  }
  dump("after-spawn");

  if (!scene.cutThreeColumns(gfx)) {
    throw std::runtime_error("cutThreeColumns failed");
  }
  ticks(32);
  dump("after-cut");

  scene.structures().setFractureEnabled(true);
  scene.structures().setStrengthPa(blast::kFrameStrengthFailPa);
  uint32_t firstSplitActors = 0;
  bool landingSplit = false;
  // Run through the actual fall and landing, not just the initial column failure.
  for (int i = 0; i < options.frameRenderHz * 4; ++i) {
    scene.update(1.0f / static_cast<float>(options.frameRenderHz));
    scene.commitStructureSplits(gfx);
    const auto& st = scene.structures().debug(scene.stressCylinderId());
    if (firstSplitActors == 0 && st.splitActors > 1) {
      firstSplitActors = st.splitActors;
    } else if (firstSplitActors > 0 && st.splitActors > firstSplitActors && st.impactDamageEvents > 0) {
      landingSplit = true;
    }
    // Blast actor counts alone miss fragments silently left inside their parent.
    std::vector<uint32_t> boundSlots;
    for (const auto& binding : scene.structures().bindings()) {
      const VoxelObject* object = scene.tryGetObject(binding.objectId);
      if (object == nullptr || !object->enabled) {
        throw std::runtime_error("Frame fracture actor has no visible object");
      }
      if (std::find(boundSlots.begin(), boundSlots.end(), binding.objectId.slot) != boundSlots.end()) {
        throw std::runtime_error("Frame fracture actors still share one physical object");
      }
      boundSlots.push_back(binding.objectId.slot);
    }
    dump((std::string("fail-tick-") + std::to_string(i)).c_str());
  }
  uint32_t singleNodePieces = 0;
  uint32_t multiNodePieces = 0;
  uint32_t largestDynamicNodes = 0;
  for (const auto& binding : scene.structures().bindings()) {
    if (binding.anchored) continue;
    if (binding.graphNodeCount == 1) ++singleNodePieces;
    else ++multiNodePieces;
    largestDynamicNodes = std::max(largestDynamicNodes, binding.graphNodeCount);
  }
  out.line("dynamic single-node=" + std::to_string(singleNodePieces) +
           " multi-node=" + std::to_string(multiNodePieces) +
           " largestNodes=" + std::to_string(largestDynamicNodes));
  // Stress routing uses the Viewer's configurable 0.01 factor; that mode is not
  // guaranteed to break this particular asset on contact at its current strength.
  if (!landingSplit && options.frameImpactMode != "stress") {
    throw std::runtime_error("Frame landing did not produce a secondary fracture");
  }
  out.line("OK frame-fail: landingSplit=" + std::to_string(landingSplit) + " and distinct fragment objects");
}

void runE5Contact(GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  uint64_t persistentToLoad = 0;
  uint64_t impactSkipped = 0;
  auto st = [&]() -> const blast::StructureDebugSnapshot& {
    return scene.structures().debug(scene.stressCylinderId());
  };
  auto ticks = [&](int n) {
    for (int i = 0; i < n; ++i) {
      scene.update(physics::kDt);
      scene.commitStructureSplits(gfx);
      persistentToLoad += st().persistentContacts;
      impactSkipped += st().impactSkippedPersistent;
    }
  };
  auto peak = [](const blast::StructureDebugSnapshot& s) {
    return std::max({s.maxTension, s.maxCompression, s.maxShear});
  };
  auto dump = [&](const std::string& tag) {
    const blast::StructureDebugSnapshot& s = st();
    physics::BodyState body;
    const bool hasBody = scene.getBodyState(scene.stressCylinderId(), body);
    char line[512];
    std::snprintf(line, sizeof(line),
                  "%s conv=%d maxT=%.4g maxC=%.4g maxS=%.4g S=%.3g pairs=%u held=%u nodes=%u |F|=%.1f N "
                  "awake=%d actors=%u fractured=%u solveMs=%.2f",
                  tag.c_str(), s.converged ? 1 : 0, s.maxTension, s.maxCompression, s.maxShear, s.strengthPa,
                  s.contactPairs, s.frozenPairs, s.contactLoadNodes, s.contactForceN, hasBody && body.awake ? 1 : 0,
                  s.splitActors, s.fracturedBonds, s.solveMs);
    out.line(line);
  };
  bool ok = true;
  auto check = [&](bool pass, const std::string& what) {
    out.line(std::string(pass ? "  PASS " : "  FAIL ") + what);
    ok = ok && pass;
  };
  scene.structures().setFractureEnabled(false);

  out.line("== T06 beam on two piers (fracture off) ==");
  scene.clearScatterBoxes(gfx);
  if (!scene.spawnBeamOnPiers(gfx, true)) throw std::runtime_error("reference beam mount failed");
  ticks(600);
  dump("reference t=10s");
  const blast::StructureDebugSnapshot ref = st();
  const float beamWeight = st().mass * 9.81f;

  if (!scene.spawnBeamOnPiers(gfx, false)) throw std::runtime_error("free beam mount failed");
  float awakePeak = 0.0f;
  for (int s = 1; s <= 10; ++s) {
    ticks(60);
    dump("free t=" + std::to_string(s) + "s");
    if (s == 3) awakePeak = peak(st());
  }
  const blast::StructureDebugSnapshot rest = st();
  char line[256];
  std::snprintf(line, sizeof(line), "  beam weight=%.1f N  ratio free/ref: T=%.3f C=%.3f S=%.3f", beamWeight,
                rest.maxTension / std::max(ref.maxTension, 1e-9f), rest.maxCompression / std::max(ref.maxCompression, 1e-9f),
                rest.maxShear / std::max(ref.maxShear, 1e-9f));
  out.line(line);
  const float ratio = peak(rest) / std::max(peak(ref), 1e-9f);
  // Resting on the piers is simply supported, the reference is fixed at both ends
  // (1.5x less mid-span moment), and support concentrates on a few contact nodes.
  check(ratio > 0.5f && ratio < 4.0f, "T06 resting beam stress of the same order as the anchored reference");
  check(rest.contactPairs > 0, "T06 contact loads still applied at t=10s (pairs=" + std::to_string(rest.contactPairs) +
                                   ", held=" + std::to_string(rest.frozenPairs) + ")");
  check(peak(rest) > 0.5f * awakePeak, "T06 stress kept after the beam sleeps (t=3s vs t=10s)");

  if (!scene.spawnBeamOnPiers(gfx, false, 3.0f)) throw std::runtime_error("drop beam mount failed");
  float fallPeak = 0.0f;
  for (int i = 0; i < 36; ++i) {  // 0.6 s, lands after ~0.78 s
    ticks(1);
    fallPeak = std::max(fallPeak, peak(st()));
  }
  std::snprintf(line, sizeof(line), "  free fall 0.6 s: peak=%.4g Pa  ratio to reference=%.3g", fallPeak,
                fallPeak / std::max(peak(ref), 1e-9f));
  out.line(line);
  check(fallPeak < 1e-3f * peak(ref), "T06 free fall adds < 0.1% of the reference stress");

  // The UI re-applies these every frame, with Fail strength (1 MPa) checked by default;
  // T06b/c run the same way so the report matches what the app does.
  bool uiFracture = true;
  bool uiImpact = true;
  auto uiTicks = [&](int n) {
    for (int i = 0; i < n; ++i) {
      scene.structures().setFractureEnabled(uiFracture);
      scene.structures().setStrengthPa(blast::kFrameStrengthFailPa);
      scene.structures().setImpactDamageEnabled(uiImpact);
      scene.structures().setStressImpactImpulses(false);
      scene.structures().setStressImpactScale(blast::kExtStressImpactImpulseFactor);
      scene.structures().setImpactSettings(blast::ImpactSettings{});
      ticks(1);
    }
  };
  auto pieces = [&]() {
    std::string s = "  pieces (nodes @ centre x, m):";
    for (const blast::ActorBinding& b : scene.structures().bindings()) {
      char buf[48];
      std::snprintf(buf, sizeof(buf), " %u@%.2f", b.graphNodeCount, b.comAsset.x - 3.2f);
      s += buf;
    }
    out.line(s);
  };
  auto notchRun = [&](const char* name, bool contactLoads, bool impactDamage) -> bool {
    out.line(std::string("== ") + name + " notch the resting beam, UI settings per tick, contact loads " +
             (contactLoads ? "on" : "off") + ", impact damage " + (impactDamage ? "on" : "off") + " ==");
    scene.clearScatterBoxes(gfx);
    scene.structures().setContactLoadsEnabled(contactLoads);
    uiImpact = impactDamage;
    if (!scene.spawnBeamOnPiers(gfx, false)) throw std::runtime_error("free beam mount failed");
    uiFracture = false;
    uiTicks(360);
    uiFracture = true;
    uiTicks(180);
    dump("intact +3s");
    const bool intactHeld = st().splitActors == 1 && st().fracturedBonds == 0;
    if (!scene.notchDemoBeam(gfx)) throw std::runtime_error("notch failed");
    uiTicks(240);
    dump("notched +4s");
    pieces();
    if (contactLoads || !impactDamage) {
      check(intactHeld, std::string(name) + " intact beam holds at its own strength");
    }
    return st().splitActors > 1 || st().fracturedBonds > 0;
  };
  check(notchRun("T06b", true, true), "T06b notched beam breaks under contact loads");
  check(!notchRun("T06c", false, false), "T06c without contact loads the notched beam does not break");
  // Pre-E5.4 defaults, recorded only: resting contact counted as impact every tick.
  notchRun("T06d", false, true);
  scene.structures().setContactLoadsEnabled(true);
  scene.structures().setImpactDamageEnabled(true);
  scene.structures().setFractureEnabled(false);

  out.line("== T07 weight on the four-column roof (fracture off) ==");
  scene.clearScatterBoxes(gfx);
  if (!scene.spawnStressFrame(gfx, 4.0f)) throw std::runtime_error("spawnStressFrame failed");
  ticks(120);
  dump("roof only");
  const float base = st().maxCompression;
  float prev = base;
  bool monotonic = true;
  for (float density : {8000.0f, 32000.0f, 128000.0f}) {
    if (!scene.dropWeightOnRoof(gfx, density)) throw std::runtime_error("dropWeightOnRoof failed");
    ticks(240);
    std::snprintf(line, sizeof(line), "weight %.0f kg (W=%.0f N)", density * 0.064f, density * 0.064f * 9.81f);
    dump(line);
    monotonic = monotonic && st().maxCompression > prev;
    prev = st().maxCompression;
  }
  scene.removeDemoWeight(gfx);
  ticks(240);
  dump("weight removed");
  check(monotonic, "T07 column compression rises with the weight");
  check(std::abs(st().maxCompression - base) < 0.1f * base, "T07 compression returns to the roof-only value");

  std::snprintf(line, sizeof(line), "== T08 persistent contacts to addLoad=%llu, skipped by the impact route=%llu ==",
                static_cast<unsigned long long>(persistentToLoad), static_cast<unsigned long long>(impactSkipped));
  out.line(line);
  check(persistentToLoad > 0 && impactSkipped > 0, "T08 persistent contacts took the load channel only");
  out.line(ok ? "OK e5-contact" : "FAIL e5-contact");
}

// Dig a block out of the pre-solved hut (fracture off) and time how the rebuilt solver
// re-converges under the per-tick budget; run with and without --no-warm-rebuild.
void runDigConverge(GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  MeshVoxelizeConfig config;
  config.gridN = 64;
  config.padding = 1;
  config.sampleColor = false;
  scene.importAsObject() = true;
  scene.importMount() = true;
  scene.importAgg() = 0;
  scene.importStrengthMPa() = 4.0f;
  scene.structures().setFractureEnabled(false);
  if (!scene.importMeshAsObject(gfx, scene.importPath(), config)) {
    throw std::runtime_error("importMeshAsObject failed: " + scene.importStatus());
  }
  out.line(std::string("warmRebuild=") + (scene.structures().warmRebuild() ? "on" : "off") +
           " budget=" + std::to_string(scene.structures().solveBudgetMs()) + " ms");
  out.line(scene.importStatus());
  for (int i = 0; i < 30; ++i) {
    scene.update(physics::kDt);
  }
  const VoxelObjectId id = scene.importedObjectId();
  const blast::StructureInstance* before = scene.structures().find(id);
  if (before == nullptr) {
    throw std::runtime_error("import not mounted");
  }
  char line[400];
  std::snprintf(line, sizeof(line), "before dig: conv=%d maxC=%.4g maxT=%.4g maxS=%.4g", before->debug.converged ? 1 : 0,
                before->debug.maxCompression, before->debug.maxTension, before->debug.maxShear);
  out.line(line);
  // 0.8 x 0.8 m slice through the whole depth at 2.0-2.8 m, x 4.0-4.8 m of the object grid.
  const uint32_t removed =
      scene.carveFines(gfx, id, [](int x, int y, int /*z*/) { return x >= 40 && x < 48 && y >= 20 && y < 28; });
  out.line("dig removed " + std::to_string(removed) + " fines");
  int firstConverged = -1;
  const auto wall0 = std::chrono::steady_clock::now();
  for (int t = 1; t <= 1200; ++t) {
    scene.update(physics::kDt);
    scene.commitStructureSplits(gfx);
    const blast::StructureInstance* inst = scene.structures().find(id);
    if (inst == nullptr) {
      out.line("instance gone at tick " + std::to_string(t));
      break;
    }
    const auto& d = inst->debug;
    if (firstConverged < 0 && d.converged) {
      firstConverged = t;
    }
    if (t == 1 || t == 15 || t == 30 || t == 60 || t == 120 || t == 240 || t == 480 || t == 960 || t == 1200 ||
        t == firstConverged) {
      std::snprintf(line, sizeof(line),
                    "t=%4d conv=%d linErr=%.3g angErr=%.3g maxC=%.4g maxT=%.4g maxS=%.4g iters=%u solveMs=%.2f "
                    "seeded=%u/%u nodes=%u",
                    t, d.converged ? 1 : 0, d.linErr, d.angErr, d.maxCompression, d.maxTension, d.maxShear,
                    d.solverIters, d.solveMs, d.seededBonds, d.seedableBonds, d.nodes);
      out.line(line);
    }
    if (firstConverged > 0 && t > firstConverged + 60 && t >= 120) {
      break;
    }
  }
  std::snprintf(line, sizeof(line), "first converged at tick %d (%.2f s sim), wall %.1f s", firstConverged,
                firstConverged / 60.0f,
                std::chrono::duration<float>(std::chrono::steady_clock::now() - wall0).count());
  out.line(line);
}

// Collapse the hut under its own weight (0.6 MPa, fracture on, UI settings per tick) and
// time every stage while it breaks into many pieces.
void runCollapsePerf(GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  MeshVoxelizeConfig config;
  config.gridN = 64;
  config.padding = 1;
  config.sampleColor = false;
  scene.importAsObject() = true;
  scene.importMount() = true;
  scene.importAgg() = 0;
  scene.importStrengthMPa() = 0.6f;
  if (!scene.importMeshAsObject(gfx, scene.importPath(), config)) {
    throw std::runtime_error("importMeshAsObject failed: " + scene.importStatus());
  }
  out.line(std::string("contactLoads=") + (scene.structures().contactLoadsEnabled() ? "on" : "off"));
  out.line("tick wallMs/tick | broad narrow solver record sleep | structTick commit | awake/bodies contacts | "
           "instances actors | loadMs solveMs candMs (sum over instances)");
  constexpr int kMaxTicks = 300;
  constexpr int kWindow = 10;
  constexpr float kMaxWallSeconds = 240.0f;
  const auto start = std::chrono::steady_clock::now();
  struct Acc {
    double wall = 0, broad = 0, narrow = 0, solver = 0, record = 0, sleep = 0, stTick = 0, commit = 0, load = 0,
           solve = 0, cand = 0;
  } acc;
  for (int t = 1; t <= kMaxTicks; ++t) {
    scene.structures().setFractureEnabled(true);
    scene.structures().setStrengthPa(blast::kFrameStrengthFailPa);
    scene.structures().setImpactDamageEnabled(true);
    scene.structures().setStressImpactImpulses(false);
    scene.structures().setStressImpactScale(blast::kExtStressImpactImpulseFactor);
    scene.structures().setImpactSettings(blast::ImpactSettings{});
    const auto t0 = std::chrono::steady_clock::now();
    scene.update(physics::kDt);
    acc.wall += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const auto& phys = scene.physicsDebug();
    acc.broad += phys.broadPhaseMs;
    acc.narrow += phys.narrowPhaseMs;
    acc.solver += phys.solverOnlyMs;
    acc.record += phys.contactRecordMs;
    acc.sleep += phys.sleepMs;
    acc.stTick += scene.structureTickMs();
    acc.commit += scene.structureCommitMs();
    uint32_t actors = 0;
    for (uint32_t i = 0; i < scene.structures().instanceCount(); ++i) {
      const blast::StructureInstance* inst = scene.structures().instanceAt(i);
      acc.load += inst->debug.contactLoadMs;
      acc.solve += inst->debug.solveMs;
      acc.cand += inst->debug.candidateMs;
      actors += static_cast<uint32_t>(inst->bindings.size());
    }
    const float wallS = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
    if (t % kWindow == 0 || wallS > kMaxWallSeconds) {
      const int n = t % kWindow == 0 ? kWindow : t % kWindow;
      char line[400];
      std::snprintf(line, sizeof(line),
                    "%4d %9.1f | %6.1f %6.1f %6.1f %6.1f %5.1f | %8.1f %7.1f | %4d/%-4d %6d | %3u %5u | %7.1f %7.1f %6.1f",
                    t, acc.wall / n, acc.broad / n, acc.narrow / n, acc.solver / n, acc.record / n, acc.sleep / n,
                    acc.stTick / n, acc.commit / n, phys.awakeBodies, phys.occupiedBodies, phys.contacts,
                    scene.structures().instanceCount(), actors, acc.load / n, acc.solve / n, acc.cand / n);
      out.line(line);
      const VoxelScene::SplitProfile& sp = scene.splitProfile();
      if (sp.commits > 0) {
        std::snprintf(line, sizeof(line),
                      "     split/tick: gather %.1f scan %.1f extract %.1f shape %.1f gpu %.1f bind %.1f trace %.1f"
                      " | commits %u owners %u unchanged %u created %u (window totals)",
                      sp.gatherMs / n, sp.scanMs / n, sp.extractMs / n, sp.shapeMs / n, sp.gpuMs / n, sp.bindMs / n,
                      sp.traceMs / n, sp.commits, sp.owners, sp.unchangedOwners, sp.created);
        out.line(line);
      }
      scene.resetSplitProfile();
      acc = Acc{};
    }
    if (wallS > kMaxWallSeconds) {
      out.line("stopped: wall-clock limit");
      break;
    }
  }
}

// Real imported geometry: a resting free structure must wake and tip after all but
// one off-centre foot are removed, without needing stress fracture first.
void runImportTopple(GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  auto require = [](bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
  };
  MeshVoxelizeConfig config;
  config.gridN = 64;
  config.padding = 1;
  config.sampleColor = false;
  scene.importAsObject() = true;
  scene.importMount() = true;
  scene.importAgg() = 2;
  require(scene.importMeshAsObject(gfx, scene.importPath(), config), "import failed");
  out.line(scene.importStatus());
  scene.structures().setFractureEnabled(false);
  scene.structures().setImpactDamageEnabled(false);
  const VoxelObjectId id = scene.importedObjectId();
  const blast::StructureInstance* inst = scene.structures().find(id);
  require(inst != nullptr && inst->debug.worldBonds == 0, "import must have zero world bonds");
  require(scene.tryGetObject(id)->motionType == MotionType::Dynamic, "import must be dynamic");

  // Flood the first footprint on the lowest occupied layer, retaining its column.
  const blast::VoxelGrid& grid = inst->grid;
  const int nx = grid.nx, nz = grid.nz;
  int bottom = 0;
  std::vector<glm::ivec2> foot;
  for (; bottom < grid.ny && foot.empty(); ++bottom) {
    for (int z = 0; z < nz && foot.empty(); ++z) {
      for (int x = 0; x < nx; ++x) {
        if (grid.isSolid(x, bottom, z)) {
          foot.emplace_back(x, z);
          break;
        }
      }
    }
  }
  require(!foot.empty(), "hut has no footprint");
  --bottom;
  std::vector<uint8_t> seen(static_cast<size_t>(nx * nz), 0);
  seen[foot.front().x + nx * foot.front().y] = 1;
  glm::ivec2 lo = foot.front(), hi = lo;
  const glm::ivec2 dirs[] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
  for (size_t i = 0; i < foot.size(); ++i) {
    const glm::ivec2 p = foot[i];
    lo = glm::min(lo, p);
    hi = glm::max(hi, p);
    for (const glm::ivec2 d : dirs) {
      const glm::ivec2 q = p + d;
      if (q.x < 0 || q.y < 0 || q.x >= nx || q.y >= nz) continue;
      const size_t key = static_cast<size_t>(q.x + nx * q.y);
      if (!seen[key] && grid.isSolid(q.x, bottom, q.y)) {
        seen[key] = 1;
        foot.push_back(q);
      }
    }
  }
  lo -= glm::ivec2(1);
  hi += glm::ivec2(1);
  const float fineSize = grid.voxelSize;
  const int cutTop = bottom + static_cast<int>(std::ceil(0.8f / fineSize));
  auto body = [&]() {
    physics::BodyState state;
    require(scene.getBodyState(id, state), "imported body missing");
    return state;
  };
  auto tilt = [](const physics::BodyState& state) {
    return glm::degrees(std::acos(std::clamp((state.q * glm::vec3(0, 1, 0)).y, -1.0f, 1.0f)));
  };
  auto tick = [&]() {
    glfwPollEvents();
    scene.update(physics::kDt);
    scene.commitStructureSplits(gfx);
  };
  for (int t = 0; t < 180; ++t) {
    tick();
    if (t >= 60 && !body().awake) break;
  }
  const physics::BodyState settled = body();
  char line[384];
  std::snprintf(line, sizeof(line), "settled: tilt=%.3f deg awake=%d comY=%.3f foot=[%d,%d]x[%d,%d]",
                tilt(settled), settled.awake ? 1 : 0, settled.x.y, lo.x, hi.x, lo.y, hi.y);
  out.line(line);
  require(tilt(settled) < 3.0f, "intact hut did not remain upright");
  const uint32_t removed = scene.carveFines(gfx, id, [=](int x, int y, int z) {
    return y < cutTop && (x < lo.x || x > hi.x || z < lo.y || z > hi.y);
  });
  require(removed > 0, "no other supports were removed");
  tick();
  const physics::BodyState cut = body();
  const VoxelObject* o = scene.tryGetObject(id);
  require(o != nullptr && o->motionType == MotionType::Dynamic, "cut made the hut static");
  const float half = 0.5f * o->gridSize * o->voxelSize;
  const glm::vec3 localCom = (glm::inverse(o->rotation) * (cut.x - o->position) + glm::vec3(half)) / fineSize;
  require(localCom.x < lo.x || localCom.x > hi.x + 1 || localCom.z < lo.y || localCom.z > hi.y + 1,
          "remaining foot still contains the centre of mass projection");
  std::snprintf(line, sizeof(line), "cut: removed=%u awake=%d localCOM=(%.2f,%.2f) outside remaining foot",
                removed, cut.awake ? 1 : 0, localCom.x, localCom.z);
  out.line(line);
  require(cut.awake, "cut did not wake the hut");
  float maxTilt = tilt(cut);
  for (int t = 1; t <= 180; ++t) {
    tick();
    const physics::BodyState st = body();
    maxTilt = std::max(maxTilt, tilt(st));
    if (t == 1 || t % 30 == 0 || maxTilt > 20.0f) {
      std::snprintf(line, sizeof(line), "t=%d tilt=%.3f deg angularSpeed=%.4f awake=%d comY=%.3f",
                    t, tilt(st), glm::length(st.w), st.awake ? 1 : 0, st.x.y);
      out.line(line);
    }
    if (maxTilt > 20.0f) break;
  }
  inst = scene.structures().find(id);
  require(inst != nullptr && inst->debug.worldBonds == 0, "cut introduced ground bonds");
  require(maxTilt > 5.0f, "unsupported hut failed to tip by 5 degrees within three seconds");
  out.line("OK import-topple: zero world bonds, cut wakes body, one off-centre foot causes rigid-body tipping");
}

void runImportObject(GfxDevice& gfx, VoxelScene& scene, PerfLog& out) {
  MeshVoxelizeConfig config;
  config.gridN = 64;
  config.padding = 1;
  config.sampleColor = false;
  scene.importAsObject() = true;
  scene.importMount() = true;
  scene.structures().setFractureEnabled(true);
  bool allFreeAndIntact = true;
  for (int agg : {2, 4, 0}) {
    scene.importAgg() = agg;
    const auto t0 = std::chrono::steady_clock::now();
    if (!scene.importMeshAsObject(gfx, scene.importPath(), config)) {
      throw std::runtime_error("importMeshAsObject failed: " + scene.importStatus());
    }
    const float importMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
    const VoxelObject* o = scene.tryGetObject(scene.importedObjectId());
    if (o == nullptr) {
      throw std::runtime_error("imported object missing");
    }
    const blast::GroundAnchorResult& anchors = scene.lastGroundAnchors();
    const float bottomY = o->position.y - 0.5f * static_cast<float>(o->gridSize) * o->voxelSize;
    char line[512];
    std::snprintf(line, sizeof(line),
                  "agg=%s grid=%d bottomY=%.3f groundAnchorFines=%u faces=%u blocked=%u importMs=%.0f",
                  agg == 0 ? "auto" : std::to_string(agg).c_str(), o->gridSize, bottomY, anchors.anchorFines,
                  anchors.contactFaces, anchors.blockedFaces, importMs);
    out.line(line);
    out.line("  status: " + scene.importStatus());
    const blast::StructureInstance* inst = scene.structures().find(scene.importedObjectId());
    if (inst == nullptr) {
      allFreeAndIntact = false;
      out.line("  not mounted");
      continue;
    }
    const int seconds = 10;
    for (int s = 1; s <= seconds; ++s) {
      for (int i = 0; i < 60; ++i) {
        scene.update(physics::kDt);
        scene.commitStructureSplits(gfx);
      }
      const blast::StructureDebugSnapshot& st = scene.structures().debug(scene.importedObjectId());
      std::snprintf(line, sizeof(line),
                    "  t=%2ds nodes=%u bonds=%u worldBonds=%u mass=%.0f conv=%d linErr=%.3g angErr=%.3g "
                    "maxT=%.3g maxC=%.3g maxS=%.3g S=%.3g fractured=%u actors=%u solveMs=%.2f",
                    s, st.nodes, st.bonds, st.worldBonds, st.mass, st.converged ? 1 : 0, st.linErr, st.angErr,
                    st.maxTension, st.maxCompression, st.maxShear, st.strengthPa, st.fracturedBonds,
                    st.splitActors, st.solveMs);
      out.line(line);
      if (s == seconds && (st.worldBonds != 0 || st.fracturedBonds > 0 || st.splitActors > 1)) {
        allFreeAndIntact = false;
      }
    }
  }
  out.line(allFreeAndIntact ? "OK import-object: hut free-mounted with no ground bonds at every agg"
                            : "FAIL import-object: see lines above");
}

int main(int argc, char** argv) {
  // CLI failures must not block automation, including errors before --benchmark is parsed.
  const bool dialogs = argc == 1;
  blast::BlastRuntime blastRt;
  try {
    const Options options = parseOptions(argc, argv);
    if (options.help) {
      std::cout << "Usage: vulkan_engine_voxel [--benchmark options | --e2-perf | --frame-fail]\n"
                   "  --frame-fail     Test four-column failure through landing\n"
                   "  --import-object  E5.5: import the hut as a free stress object with no ground bonds at agg 2/4/auto\n"
                   "  --import-topple  Check that a resting hut tips after all but one off-centre foot are cut\n"
                   "  --e5-contact     E5.4: contact loads (beam on piers, notch, weight on roof), report\n"
                   "  --no-contact-loads   Pre-E5.4 loads (with --frame-fail etc.)\n"
                   "  --solve-budget MS    Stress solve ms per tick for all structures (0 = no limit, default 8)\n"
                   "  --collapse-perf  Hut at 0.6 MPa collapses; per-stage tick timing (<= 300 ticks / 240 s)\n"
                   "  --dig-converge   Dig the pre-solved hut; ticks for the rebuilt solver to re-converge\n"
                   "  --no-warm-rebuild    Rebuilt solvers start from zero (compare with --dig-converge)\n"
                   "  --frame-height M Column height for --frame-fail (1.0-9.2 m, default 4.0)\n"
                   "  --frame-impact MODE  shear (Viewer default), spread, or stress\n"
                   "  --frame-render-hz N  Frame test cadence (30-240, default 60)\n"
                   "  --frame-anchors M    explicit (column-base rule, default) or ground (E5.2 contact)\n"
                   "  --e2-perf        Headless-ish E2 layer D: spawn cylinder, time scene+collision, exit\n"
                   "  --frames N       Measured submitted frames (default 300, >0)\n"
                   "  --warmup N       Excluded submitted frames (default 60, >=0)\n"
                   "  --width N --height N  Exact framebuffer pixels (default 2560 1440)\n"
                   "  --beam 0/1 --brick-skip 0/1 --dir-brick 0/1 --dir-coarse 0/1 (default all 1)\n"
                   "  --color 0/1      Sample imported mesh color (default 0; shading/AO stay on)\n"
                    "  --camera x y z yaw pitch  Fixed pose, radians (default 4 5 6 0.67474094 0)\n"
                   "  --stage N        0=full (default); 1..5=diagnostic, NOT full-quality performance\n"
                   "  --capture PATH   P6 PPM from pre-UI RGBA8 output (RGB only, after timing)\n"
                   "  --csv PATH       Raw frame rows plus median/p95 rows, with run metadata\n"
                   "Imports scene.importPath() (pirate hut), grid 64, padding 1, conservative surface.\n"
                   "Full nested micro/fine traversal, shading, AO and normal locked UI are retained.\n"
                   "No arguments starts the interactive demo.\n";
      return 0;
    }
    if (!blastRt.init()) {
      showFatal("Blast runtime failed to initialize", dialogs);
      return 1;
    }
    Window window(WindowConfig{
        .title = "Vulkan Engine - Voxel Demo",
        .width = options.benchmark ? options.width : 1280,
        .height = options.benchmark ? options.height : 720,
    });

    if (options.importTopple) {
      glfwHideWindow(window.handle());
    }
    if (options.benchmark) {
      sizeBenchmarkWindow(window, options);
    }
#ifdef _WIN32
    if (!options.benchmark && !options.e2Perf && !options.importTopple) {
      if (HWND hwnd = glfwGetWin32Window(window.handle())) {
        SetWindowPos(hwnd, HWND_TOPMOST, 160, 160, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE);
        SetForegroundWindow(hwnd);
        SetWindowPos(hwnd, HWND_NOTOPMOST, 160, 160, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE);
      }
    }
    if (options.e2Perf || options.frameFail || options.importObject || options.e5Contact || options.collapsePerf || options.digConverge || options.collisionPerf > 0) {
      if (AttachConsole(ATTACH_PARENT_PROCESS) || AllocConsole()) {
        FILE* fp = nullptr;
        freopen_s(&fp, "CONOUT$", "w", stdout);
        freopen_s(&fp, "CONOUT$", "w", stderr);
      }
    }
#endif

    GfxDevice gfx(window);
    VoxelScene scene;
    scene.init(gfx, blastRt);
    scene.structures().setContactLoadsEnabled(options.contactLoads);
    scene.structures().setWarmRebuild(options.warmRebuild);
    if (options.solveBudgetMs >= 0.0f) {
      scene.structures().setSolveBudgetMs(options.solveBudgetMs);
    }

    if (options.collisionPerf > 0) {
      runCollisionPerf(gfx, scene, options.collisionPerf);
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.digConverge) {
      const std::filesystem::path reportPath =
          std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" /
          (options.warmRebuild ? "dig-converge.txt" : "dig-converge-cold.txt");
      PerfLog log(reportPath);
      if (log.f == nullptr) {
        throw std::runtime_error("Cannot open " + reportPath.string());
      }
      try {
        runDigConverge(gfx, scene, log);
      } catch (const std::exception& ex) {
        log.line(std::string("ERROR: ") + ex.what());
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.collapsePerf) {
      const std::filesystem::path reportPath =
          std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" /
          (options.contactLoads ? "collapse-perf.txt" : "collapse-perf-nocontact.txt");
      PerfLog log(reportPath);
      if (log.f == nullptr) {
        throw std::runtime_error("Cannot open " + reportPath.string());
      }
      try {
        runCollapsePerf(gfx, scene, log);
      } catch (const std::exception& ex) {
        log.line(std::string("ERROR: ") + ex.what());
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.e5Contact) {
      const std::filesystem::path reportPath =
          std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" / "e5-contact.txt";
      PerfLog log(reportPath);
      if (log.f == nullptr) {
        throw std::runtime_error("Cannot open " + reportPath.string());
      }
      try {
        runE5Contact(gfx, scene, log);
      } catch (const std::exception& ex) {
        log.line(std::string("ERROR: ") + ex.what());
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.importObject || options.importTopple) {
      const std::filesystem::path reportPath =
          std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" /
          (options.importTopple ? "import-topple.txt" : "import-object.txt");
      PerfLog log(reportPath);
      if (log.f == nullptr) {
        throw std::runtime_error("Cannot open " + reportPath.string());
      }
      try {
        if (options.importTopple) {
          runImportTopple(gfx, scene, log);
        } else {
          runImportObject(gfx, scene, log);
        }
      } catch (const std::exception& ex) {
        log.line(std::string("ERROR: ") + ex.what());
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.frameFail) {
      const std::filesystem::path reportPath =
          std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" / "frame-fail.txt";
      PerfLog log(reportPath);
      if (log.f == nullptr) {
        throw std::runtime_error("Cannot open " + reportPath.string());
      }
      try {
        runFrameFail(gfx, scene, log, options);
      } catch (const std::exception& ex) {
        log.line(std::string("ERROR: ") + ex.what());
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.e2Perf) {
      const std::filesystem::path reportPath =
          std::filesystem::path(VE_ASSETS_DIR).parent_path() / "docs" / "blast-e2-perf-engine.txt";
      PerfLog log(reportPath);
      if (log.f == nullptr) {
        throw std::runtime_error("Cannot open " + reportPath.string());
      }
      log.line(std::string("report=") + reportPath.string());
      try {
        runE2PerfEngine(window, gfx, scene, log);
      } catch (const std::exception& ex) {
        log.line(std::string("ERROR: ") + ex.what());
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    if (options.benchmark) {
      try {
        runBenchmark(options, window, gfx, scene);
      } catch (...) {
        gfx.waitIdle();
        scene.cleanup(gfx);
        throw;
      }
      gfx.waitIdle();
      scene.cleanup(gfx);
      blastRt.shutdown();
      return 0;
    }

    VoxelRenderer renderer(gfx);
    renderer.init(scene);

    std::cout << "Voxel DDA demo running. WASD move, Q/E up/down, right-drag look.\n"
              << "LMB remove voxel, F place voxel (against hit face), Esc quit."
              << std::endl;

    auto last = std::chrono::steady_clock::now();
    float fps = 60.0f;

    while (!window.shouldClose()) {
      window.pollEvents();
      if (glfwGetKey(window.handle(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window.handle(), GLFW_TRUE);
      }

      const auto now = std::chrono::steady_clock::now();
      const float dt = std::chrono::duration<float>(now - last).count();
      last = now;
      const float alpha = 0.1f;
      fps = fps * (1.0f - alpha) + (dt > 0.0f ? (1.0f / dt) : fps) * alpha;

      const float aspect = (gfx.swapchainExtent().height > 0)
                               ? static_cast<float>(gfx.swapchainExtent().width) /
                                     static_cast<float>(gfx.swapchainExtent().height)
                               : 1.0f;
      scene.camera().handleInput(window.handle(), dt);
      scene.camera().update(aspect);
      // Edit / fracture prepare+commit before physics so the same frame uses new shapes.
      scene.handleEditInput(window.handle(), gfx);
      scene.advanceFractureWork(dt);
      scene.commitReadyFractures(gfx);
      scene.update(dt);
      scene.commitStructureSplits(gfx);
      renderer.draw(scene, fps);
    }

    gfx.waitIdle();
    scene.cleanup(gfx);
    blastRt.shutdown();
  } catch (const std::exception& ex) {
    showFatal(ex.what(), dialogs);
    blastRt.shutdown();
    return 1;
  }
  return 0;
}
