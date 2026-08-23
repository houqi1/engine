#pragma once

#ifndef GLFW_INCLUDE_VULKAN
#define GLFW_INCLUDE_VULKAN
#endif
#include <GLFW/glfw3.h>

#include <cstdint>
#include <string>

struct WindowConfig {
  std::string title = "Vulkan Engine";
  int width = 1280;
  int height = 720;
};

class Window {
public:
  explicit Window(const WindowConfig& config);
  ~Window();

  Window(const Window&) = delete;
  Window& operator=(const Window&) = delete;

  GLFWwindow* handle() const { return window_; }
  bool shouldClose() const;
  void pollEvents() const;

  VkExtent2D framebufferExtent() const;
  bool wasResized() const { return framebufferResized_; }
  void clearResizedFlag() { framebufferResized_ = false; }

  static void framebufferResizeCallback(GLFWwindow* window, int width, int height);

private:
  GLFWwindow* window_ = nullptr;
  bool framebufferResized_ = false;
};
