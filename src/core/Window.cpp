#include "core/Window.h"

#include <stdexcept>

#if defined(VE_PLATFORM_MACOS)
#define GLFW_EXPOSE_NATIVE_COCOA
#include <GLFW/glfw3native.h>
#import <Cocoa/Cocoa.h>
#import <QuartzCore/CAMetalLayer.h>
#endif

Window::Window(const WindowConfig& config) {
  if (!glfwInit()) {
    throw std::runtime_error("Failed to initialize GLFW");
  }

  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  window_ = glfwCreateWindow(config.width, config.height, config.title.c_str(), nullptr, nullptr);
  if (!window_) {
    glfwTerminate();
    throw std::runtime_error("Failed to create GLFW window");
  }

  glfwSetWindowUserPointer(window_, this);
  glfwSetFramebufferSizeCallback(window_, framebufferResizeCallback);

  // Important on macOS: configure the Metal layer before Vulkan surface creation.
  disableMetalDisplaySync();
}

Window::~Window() {
  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
}

bool Window::shouldClose() const {
  return glfwWindowShouldClose(window_) != 0;
}

void Window::pollEvents() const {
  glfwPollEvents();
}

VkExtent2D Window::framebufferExtent() const {
  int width = 0;
  int height = 0;
  glfwGetFramebufferSize(window_, &width, &height);
  return VkExtent2D{
      static_cast<uint32_t>(width),
      static_cast<uint32_t>(height),
  };
}

void Window::framebufferResizeCallback(GLFWwindow* window, int /*width*/, int /*height*/) {
  auto* self = static_cast<Window*>(glfwGetWindowUserPointer(window));
  if (self) {
    self->framebufferResized_ = true;
  }
}

void Window::disableMetalDisplaySync() const {
#if defined(VE_PLATFORM_MACOS)
  if (!window_) {
    return;
  }
  NSWindow* nsWindow = glfwGetCocoaWindow(window_);
  if (!nsWindow) {
    return;
  }
  NSView* view = [nsWindow contentView];
  if (!view) {
    return;
  }
  [view setWantsLayer:YES];
  CAMetalLayer* layer = nullptr;
  if ([view.layer isKindOfClass:[CAMetalLayer class]]) {
    layer = (CAMetalLayer*)view.layer;
  } else {
    layer = [CAMetalLayer layer];
    view.layer = layer;
  }
  if (@available(macOS 10.13, *)) {
    layer.displaySyncEnabled = NO;
  }
#else
  (void)window_;
#endif
}
