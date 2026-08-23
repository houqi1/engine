#include "core/Window.h"
#include "gfx/GfxDevice.h"
#include "render/Renderer.h"
#include "scene/Scene.h"

#include <GLFW/glfw3.h>

#include <chrono>
#include <iostream>
#include <stdexcept>

int main() {
  try {
    Window window(WindowConfig{
        .title = "Vulkan Engine — Forward PBR + Shadows",
        .width = 1280,
        .height = 720,
    });

    GfxDevice gfx(window);
    Scene scene;
    scene.init(gfx);

    Renderer renderer(gfx);
    renderer.init(scene);

    std::cout << "Engine running. WASD orbit, Q/E zoom, right-drag rotate, Esc quit." << std::endl;

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
    std::cerr << "Fatal: " << ex.what() << '\n';
    return 1;
  }
  return 0;
}
