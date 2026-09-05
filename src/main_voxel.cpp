#include "core/Window.h"
#include "gfx/GfxDevice.h"
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
  bool help = false;
  uint32_t frames = 300;
  uint32_t warmup = 60;
  int width = 2560;
  int height = 1440;
  bool color = false;  // Match the scene's default import, not Solid Color output.
  VoxelRenderer::BenchmarkSettings renderer{};
  // stampMeshIntoWorld centers the hut in X/Z, with its base at 2.5 * 0.35 m.
  // Camera yaw 0 looks along -Z; positive pitch looks down. Angles are radians.
  std::array<float, 5> camera{4.0f, 3.0f, 5.0f, 0.67474094f, 0.0f};
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
  if (argc > 1 && !options.benchmark && !options.help) {
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

int main(int argc, char** argv) {
  // CLI failures must not block automation, including errors before --benchmark is parsed.
  const bool dialogs = argc == 1;
  try {
    const Options options = parseOptions(argc, argv);
    if (options.help) {
      std::cout << "Usage: vulkan_engine_voxel --benchmark [options]\n"
                   "  --frames N       Measured submitted frames (default 300, >0)\n"
                   "  --warmup N       Excluded submitted frames (default 60, >=0)\n"
                   "  --width N --height N  Exact framebuffer pixels (default 2560 1440)\n"
                   "  --beam 0/1 --brick-skip 0/1 --dir-brick 0/1 --dir-coarse 0/1 (default all 1)\n"
                   "  --color 0/1      Sample imported mesh color (default 0; shading/AO stay on)\n"
                    "  --camera x y z yaw pitch  Fixed pose, radians (default 4 3 5 0.67474094 0)\n"
                   "  --stage N        0=full (default); 1..5=diagnostic, NOT full-quality performance\n"
                   "  --capture PATH   P6 PPM from pre-UI RGBA8 output (RGB only, after timing)\n"
                   "  --csv PATH       Raw frame rows plus median/p95 rows, with run metadata\n"
                   "Imports scene.importPath() (pirate hut), grid 64, padding 1, conservative surface.\n"
                   "Full nested micro/fine traversal, shading, AO and normal locked UI are retained.\n"
                   "No arguments starts the interactive demo.\n";
      return 0;
    }
    Window window(WindowConfig{
        .title = "Vulkan Engine - Voxel Demo",
        .width = options.benchmark ? options.width : 1280,
        .height = options.benchmark ? options.height : 720,
    });

    if (options.benchmark) {
      sizeBenchmarkWindow(window, options);
    }
#ifdef _WIN32
    if (!options.benchmark) {
      if (HWND hwnd = glfwGetWin32Window(window.handle())) {
        SetWindowPos(hwnd, HWND_TOPMOST, 160, 160, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE);
        SetForegroundWindow(hwnd);
        SetWindowPos(hwnd, HWND_NOTOPMOST, 160, 160, 0, 0, SWP_SHOWWINDOW | SWP_NOSIZE);
      }
    }
#endif

    GfxDevice gfx(window);
    VoxelScene scene;
    scene.init(gfx);

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
      scene.update(dt);
      scene.handleEditInput(window.handle(), gfx);
      renderer.draw(scene, fps);
    }

    gfx.waitIdle();
    scene.cleanup(gfx);
  } catch (const std::exception& ex) {
    showFatal(ex.what(), dialogs);
    return 1;
  }
  return 0;
}
