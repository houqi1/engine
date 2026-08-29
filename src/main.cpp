#include "core/Window.h"
#include "gfx/GfxDevice.h"
#include "render/Renderer.h"
#include "scene/Scene.h"

#include <GLFW/glfw3.h>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#include <windows.h>
#endif

#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

#ifdef _WIN32
// Ask Windows laptop GPU switching (Optimus) to prefer the discrete NVIDIA GPU.
extern "C" {
__declspec(dllexport) DWORD NvOptimusEnablement = 1;
__declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
}
#endif

namespace {

void appendCrashLog(const char* message) {
  std::ofstream log("vulkan_engine_crash.log", std::ios::app);
  if (!log) {
    return;
  }
  log << message << '\n';
}

void showFatal(const char* message) {
  std::cerr << "Fatal: " << message << '\n';
  appendCrashLog(message);
#ifdef _WIN32
  MessageBoxA(nullptr, message, "Vulkan Engine", MB_OK | MB_ICONERROR);
#endif
}

}  // namespace

int main() {
  try {
    Window window(WindowConfig{
        .title = "Vulkan Engine - Grass Demo",
        .width = 1280,
        .height = 720,
    });

#ifdef _WIN32
    // Make the first launch obvious on multi-monitor setups.
    if (HWND hwnd = glfwGetWin32Window(window.handle())) {
      SetWindowPos(hwnd, HWND_TOPMOST, 120, 120, 1280, 720, SWP_SHOWWINDOW);
      SetForegroundWindow(hwnd);
      SetWindowPos(hwnd, HWND_NOTOPMOST, 120, 120, 1280, 720, SWP_SHOWWINDOW);
    }
#endif

    GfxDevice gfx(window);
    Scene scene;
    scene.init(gfx);

    Renderer renderer(gfx);
    renderer.init(scene);

    std::cout << "Engine running. WASD move, Q/E up/down, right-drag look, Esc quit." << std::endl;

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
      renderer.draw(scene, dt, fps);
    }

    gfx.waitIdle();
    scene.cleanup(gfx);
  } catch (const std::exception& ex) {
    showFatal(ex.what());
    return 1;
  }
  return 0;
}
