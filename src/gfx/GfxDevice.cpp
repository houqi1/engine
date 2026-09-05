#include "gfx/GfxDevice.h"

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace {

[[noreturn]] void fail(const std::string& message) {
  throw std::runtime_error(message);
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT /*types*/,
    const VkDebugUtilsMessengerCallbackDataEXT* callbackData,
    void* /*userData*/) {
  const char* level = "INFO";
  if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
    level = "ERROR";
  } else if (severity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
    level = "WARN";
  }
  std::cerr << "[Vulkan][" << level << "] " << callbackData->pMessage << '\n';
  return VK_FALSE;
}

}  // namespace

GfxDevice::GfxDevice(Window& window) : window_(window) {
  createInstance();
  createSurface();
  selectDevice();
  createAllocator();
  createSwapchain();
  createFrameData();

  VkCommandPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  poolInfo.queueFamilyIndex = graphicsQueueFamily_;
  if (vkCreateCommandPool(device_.device, &poolInfo, nullptr, &uploadPool_) != VK_SUCCESS) {
    fail("Failed to create upload command pool");
  }

  VkFenceCreateInfo fenceInfo{};
  fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  if (vkCreateFence(device_.device, &fenceInfo, nullptr, &uploadFence_) != VK_SUCCESS) {
    fail("Failed to create upload fence");
  }
}

GfxDevice::~GfxDevice() {
  waitIdle();

  if (uploadFence_) {
    vkDestroyFence(device_.device, uploadFence_, nullptr);
  }
  if (uploadPool_) {
    vkDestroyCommandPool(device_.device, uploadPool_, nullptr);
  }

  for (auto& frame : frames_) {
    if (frame.inFlight) {
      vkDestroyFence(device_.device, frame.inFlight, nullptr);
    }
    if (frame.imageAvailable) {
      vkDestroySemaphore(device_.device, frame.imageAvailable, nullptr);
    }
    if (frame.commandPool) {
      vkDestroyCommandPool(device_.device, frame.commandPool, nullptr);
    }
  }
  frames_.clear();

  destroySwapchainSync();
  destroySwapchain();

  if (allocator_) {
    vmaDestroyAllocator(allocator_);
    allocator_ = VK_NULL_HANDLE;
  }

  if (surface_ != VK_NULL_HANDLE) {
    instanceDispatch_.fp_vkDestroySurfaceKHR(instance_.instance, surface_, nullptr);
    surface_ = VK_NULL_HANDLE;
  }

  vkb::destroy_device(device_);
  vkb::destroy_instance(instance_);
}

void GfxDevice::createInstance() {
  vkb::InstanceBuilder builder;
  builder.set_app_name("Vulkan Engine")
      .set_engine_name("vulkan_engine")
      .require_api_version(1, 3, 0)
      .set_debug_callback(debugCallback);

#if defined(VE_ENABLE_VALIDATION)
  builder.request_validation_layers(true).enable_validation_layers(true);
#endif

  uint32_t glfwExtCount = 0;
  const char** glfwExts = glfwGetRequiredInstanceExtensions(&glfwExtCount);
  if (!glfwExts || glfwExtCount == 0) {
    fail("GLFW did not report required Vulkan instance extensions");
  }
  for (uint32_t i = 0; i < glfwExtCount; ++i) {
    builder.enable_extension(glfwExts[i]);
  }

  auto instRet = builder.build();
  if (!instRet) {
    fail(std::string("Failed to create Vulkan instance: ") + instRet.error().message());
  }
  instance_ = instRet.value();
  instanceDispatch_ = instance_.make_table();
}

void GfxDevice::createSurface() {
  // Ensure Metal display sync is off before MoltenVK binds the CAMetalLayer.
  window_.disableMetalDisplaySync();
  if (glfwCreateWindowSurface(instance_.instance, window_.handle(), nullptr, &surface_) !=
      VK_SUCCESS) {
    fail("Failed to create window surface");
  }
}

void GfxDevice::selectDevice() {
  VkPhysicalDeviceVulkan13Features features13{};
  features13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
  features13.dynamicRendering = VK_TRUE;
  features13.synchronization2 = VK_TRUE;

  VkPhysicalDeviceFeatures features{};
  features.samplerAnisotropy = VK_TRUE;
  features.fillModeNonSolid = VK_TRUE;
  features.depthClamp = VK_TRUE;

  vkb::PhysicalDeviceSelector selector{instance_};
  auto devicesRet = selector.set_surface(surface_)
                        .set_minimum_version(1, 3)
                        .set_required_features(features)
                        .set_required_features_13(features13)
                        .prefer_gpu_device_type(vkb::PreferredDeviceType::discrete)
                        .allow_any_gpu_device_type(true)
                        .select_devices();
  if (!devicesRet) {
    fail(std::string("Failed to enumerate suitable GPUs: ") + devicesRet.error().message());
  }

  const auto& devices = devicesRet.value();
  if (devices.empty()) {
    fail("No suitable Vulkan 1.3 GPU found");
  }

  std::cout << "Available GPUs (" << devices.size() << "):\n";
  for (size_t i = 0; i < devices.size(); ++i) {
    const auto& d = devices[i];
    const char* type = "other";
    switch (d.properties.deviceType) {
      case VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU:
        type = "discrete";
        break;
      case VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU:
        type = "integrated";
        break;
      case VK_PHYSICAL_DEVICE_TYPE_VIRTUAL_GPU:
        type = "virtual";
        break;
      case VK_PHYSICAL_DEVICE_TYPE_CPU:
        type = "cpu";
        break;
      default:
        break;
    }
    std::cout << "  [" << i << "] " << d.properties.deviceName << " (" << type << ")\n";
  }

  auto scoreDevice = [](const vkb::PhysicalDevice& d) {
    int score = 0;
    const std::string name = d.properties.deviceName ? d.properties.deviceName : "";
    if (d.properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
      score += 1000;
    }
    // Prefer NVIDIA RTX on this machine when multiple devices are suitable.
    if (name.find("NVIDIA") != std::string::npos || name.find("RTX") != std::string::npos ||
        name.find("GeForce") != std::string::npos) {
      score += 500;
    }
    if (name.find("4060") != std::string::npos) {
      score += 100;
    }
    // Deprioritize Intel iGPU when a discrete option exists.
    if (name.find("Intel") != std::string::npos &&
        d.properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_INTEGRATED_GPU) {
      score -= 200;
    }
    return score;
  };

  size_t bestIndex = 0;
  int bestScore = scoreDevice(devices[0]);
  for (size_t i = 1; i < devices.size(); ++i) {
    const int s = scoreDevice(devices[i]);
    if (s > bestScore) {
      bestScore = s;
      bestIndex = i;
    }
  }

  physicalDevice_ = devices[bestIndex];
  deviceName_ = physicalDevice_.properties.deviceName ? physicalDevice_.properties.deviceName
                                                       : "Unknown GPU";

  const char* portableBricksEnv = std::getenv("VE_VOXEL_PORTABLE_BRICKS");
  const bool forcePortableBricks = portableBricksEnv && std::strcmp(portableBricksEnv, "1") == 0;
  VkPhysicalDeviceVulkan12Features optionalFeatures12{};
  optionalFeatures12.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_2_FEATURES;
  optionalFeatures12.shaderStorageBufferArrayNonUniformIndexing = VK_TRUE;
  // Query the selected GPU and extend vkb's enable chain only if supported.
  // Keep this out of the selector's required features so the portable path still works.
  storageBufferNonUniformIndexing_ = !forcePortableBricks &&
      physicalDevice_.enable_extension_features_if_present(optionalFeatures12);

  const char* pipelineStatsEnv = std::getenv("VE_VOXEL_PIPELINE_STATS");
  const bool requestPipelineStats = pipelineStatsEnv && std::strcmp(pipelineStatsEnv, "1") == 0;
  if (requestPipelineStats) {
    VkPhysicalDevicePipelineExecutablePropertiesFeaturesKHR optionalPipelineStats{};
    optionalPipelineStats.sType =
        VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PIPELINE_EXECUTABLE_PROPERTIES_FEATURES_KHR;
    optionalPipelineStats.pipelineExecutableInfo = VK_TRUE;
    pipelineExecutableStatisticsEnabled_ = physicalDevice_.enable_extension_if_present(
        VK_KHR_PIPELINE_EXECUTABLE_PROPERTIES_EXTENSION_NAME) &&
        physicalDevice_.enable_extension_features_if_present(optionalPipelineStats);
  }

  {
    const VkSampleCountFlags supported =
        physicalDevice_.properties.limits.framebufferColorSampleCounts &
        physicalDevice_.properties.limits.framebufferDepthSampleCounts;
    if (supported & VK_SAMPLE_COUNT_4_BIT) {
      msaaSamples_ = VK_SAMPLE_COUNT_4_BIT;
    } else if (supported & VK_SAMPLE_COUNT_2_BIT) {
      msaaSamples_ = VK_SAMPLE_COUNT_2_BIT;
    } else {
      msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
    }
  }

  auto devRet = vkb::DeviceBuilder{physicalDevice_}.build();
  if (!devRet) {
    fail(std::string("Failed to create logical device: ") + devRet.error().message());
  }
  device_ = devRet.value();

  auto queueRet = device_.get_queue(vkb::QueueType::graphics);
  if (!queueRet) {
    fail(std::string("Failed to get graphics queue: ") + queueRet.error().message());
  }
  graphicsQueue_ = queueRet.value();

  auto familyRet = device_.get_queue_index(vkb::QueueType::graphics);
  if (!familyRet) {
    fail(std::string("Failed to get graphics queue family: ") + familyRet.error().message());
  }
  graphicsQueueFamily_ = familyRet.value();

  std::cout << "Using GPU: " << deviceName_ << std::endl;
  std::cout << "MSAA samples: " << static_cast<uint32_t>(msaaSamples_) << std::endl;
  const char* brickBackend = storageBufferNonUniformIndexing_ ? "nonuniform indexed" : "portable";
  const char* brickOverride = forcePortableBricks ? " (VE_VOXEL_PORTABLE_BRICKS=1)" : "";
  std::cout << "Voxel brick backend: " << brickBackend << brickOverride << std::endl;
  if (requestPipelineStats) {
    std::cout << "[VoxelPipelineStats] "
              << (pipelineExecutableStatisticsEnabled_
                      ? "enabled"
                      : "unavailable: extension or pipelineExecutableInfo unsupported")
              << std::endl;
  }
  {
    std::ofstream log("vulkan_engine.log", std::ios::app);
    if (log) {
      log << "Using GPU: " << deviceName_ << '\n';
      log << "MSAA samples: " << static_cast<uint32_t>(msaaSamples_) << '\n';
      log << "Voxel brick backend: " << brickBackend << brickOverride << '\n';
    }
  }
}

void GfxDevice::createAllocator() {
  VmaAllocatorCreateInfo info{};
  info.physicalDevice = physicalDevice_.physical_device;
  info.device = device_.device;
  info.instance = instance_.instance;
  info.vulkanApiVersion = VK_API_VERSION_1_3;
  if (vmaCreateAllocator(&info, &allocator_) != VK_SUCCESS) {
    fail("Failed to create VMA allocator");
  }
}

const char* GfxDevice::presentModeName() const {
  switch (presentMode_) {
    case VK_PRESENT_MODE_IMMEDIATE_KHR:
      return "IMMEDIATE (uncapped)";
    case VK_PRESENT_MODE_MAILBOX_KHR:
      return "MAILBOX (uncapped)";
    case VK_PRESENT_MODE_FIFO_RELAXED_KHR:
      return "FIFO_RELAXED";
    case VK_PRESENT_MODE_FIFO_KHR:
    default:
      return "FIFO (VSync)";
  }
}

void GfxDevice::createSwapchain() {
  const VkExtent2D extent = window_.framebufferExtent();
  // Prefer uncapped present modes so Display FPS reflects real throughput.
  // Fallback order: MAILBOX -> IMMEDIATE -> FIFO_RELAXED -> FIFO.
  auto swapRet =
      vkb::SwapchainBuilder{device_, surface_}
          .set_desired_format({VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
          .add_fallback_format({VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
          .set_desired_present_mode(VK_PRESENT_MODE_MAILBOX_KHR)
          .add_fallback_present_mode(VK_PRESENT_MODE_IMMEDIATE_KHR)
          .add_fallback_present_mode(VK_PRESENT_MODE_FIFO_RELAXED_KHR)
          .add_fallback_present_mode(VK_PRESENT_MODE_FIFO_KHR)
          .set_desired_extent(extent.width, extent.height)
          .add_image_usage_flags(VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
          .build();
  if (!swapRet) {
    fail(std::string("Failed to create swapchain: ") + swapRet.error().message());
  }

  swapchain_ = swapRet.value();
  swapchainExtent_ = swapchain_.extent;
  swapchainFormat_ = swapchain_.image_format;
  presentMode_ = swapchain_.present_mode;
  swapchainImages_ = swapchain_.get_images().value();
  swapchainImageViews_ = swapchain_.get_image_views().value();
  createSwapchainSync();

  // MoltenVK may recreate/reconfigure the layer during swapchain build; re-assert.
  window_.disableMetalDisplaySync();

  std::cout << "Present mode: " << presentModeName() << std::endl;
}

void GfxDevice::createSwapchainSync() {
  destroySwapchainSync();

  VkSemaphoreCreateInfo semaphoreInfo{};
  semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
  renderFinishedSemaphores_.resize(swapchainImages_.size());
  for (auto& sem : renderFinishedSemaphores_) {
    if (vkCreateSemaphore(device_.device, &semaphoreInfo, nullptr, &sem) != VK_SUCCESS) {
      fail("Failed to create render-finished semaphore");
    }
  }
  imagesInFlight_.assign(swapchainImages_.size(), VK_NULL_HANDLE);
}

void GfxDevice::destroySwapchainSync() {
  for (VkSemaphore sem : renderFinishedSemaphores_) {
    if (sem) {
      vkDestroySemaphore(device_.device, sem, nullptr);
    }
  }
  renderFinishedSemaphores_.clear();
  imagesInFlight_.clear();
}

void GfxDevice::destroySwapchain() {
  destroySwapchainSync();
  for (VkImageView view : swapchainImageViews_) {
    vkDestroyImageView(device_.device, view, nullptr);
  }
  swapchainImageViews_.clear();
  swapchainImages_.clear();
  vkb::destroy_swapchain(swapchain_);
}

void GfxDevice::recreateSwapchain() {
  VkExtent2D extent = window_.framebufferExtent();
  while ((extent.width == 0 || extent.height == 0) && !window_.shouldClose()) {
    extent = window_.framebufferExtent();
    glfwWaitEvents();
  }
  if (window_.shouldClose()) {
    return;
  }

  waitIdle();
  destroySwapchain();
  createSwapchain();
  window_.clearResizedFlag();
  swapchainRecreated_ = true;
}

void GfxDevice::createFrameData() {
  frames_.resize(kFramesInFlight);

  VkCommandPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  poolInfo.queueFamilyIndex = graphicsQueueFamily_;

  VkSemaphoreCreateInfo semaphoreInfo{};
  semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

  VkFenceCreateInfo fenceInfo{};
  fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
  fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

  for (auto& frame : frames_) {
    if (vkCreateCommandPool(device_.device, &poolInfo, nullptr, &frame.commandPool) != VK_SUCCESS) {
      fail("Failed to create command pool");
    }
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = frame.commandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = 1;
    if (vkAllocateCommandBuffers(device_.device, &allocInfo, &frame.commandBuffer) != VK_SUCCESS) {
      fail("Failed to allocate command buffer");
    }
    if (vkCreateSemaphore(device_.device, &semaphoreInfo, nullptr, &frame.imageAvailable) !=
            VK_SUCCESS ||
        vkCreateFence(device_.device, &fenceInfo, nullptr, &frame.inFlight) != VK_SUCCESS) {
      fail("Failed to create frame sync objects");
    }
  }
}

bool GfxDevice::beginFrame(FrameContext& outFrame) {
  FrameData& frame = frames_[frameIndex_];
  vkWaitForFences(device_.device, 1, &frame.inFlight, VK_TRUE, UINT64_MAX);

  uint32_t imageIndex = 0;
  const VkResult acquire = vkAcquireNextImageKHR(device_.device, swapchain_.swapchain, UINT64_MAX,
                                                 frame.imageAvailable, VK_NULL_HANDLE, &imageIndex);
  if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
    recreateSwapchain();
    return false;
  }
  if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
    fail("Failed to acquire swapchain image");
  }

  if (imagesInFlight_[imageIndex] != VK_NULL_HANDLE) {
    vkWaitForFences(device_.device, 1, &imagesInFlight_[imageIndex], VK_TRUE, UINT64_MAX);
  }
  imagesInFlight_[imageIndex] = frame.inFlight;

  vkResetFences(device_.device, 1, &frame.inFlight);
  vkResetCommandBuffer(frame.commandBuffer, 0);

  VkCommandBufferBeginInfo beginInfo{};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(frame.commandBuffer, &beginInfo);

  outFrame.cmd = frame.commandBuffer;
  outFrame.imageIndex = imageIndex;
  outFrame.frameIndex = frameIndex_;
  outFrame.swapchainImage = swapchainImages_[imageIndex];
  outFrame.swapchainView = swapchainImageViews_[imageIndex];
  outFrame.extent = swapchainExtent_;
  outFrame.swapchainFormat = swapchainFormat_;
  return true;
}

void GfxDevice::endFrame(const FrameContext& frameCtx) {
  FrameData& frame = frames_[frameIndex_];
  vkEndCommandBuffer(frameCtx.cmd);

  VkSemaphore renderFinished = renderFinishedSemaphores_[frameCtx.imageIndex];
  // Voxel rendering blits first; raster rendering uses a color attachment.
  VkPipelineStageFlags waitStage =
      VK_PIPELINE_STAGE_TRANSFER_BIT | VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.waitSemaphoreCount = 1;
  submitInfo.pWaitSemaphores = &frame.imageAvailable;
  submitInfo.pWaitDstStageMask = &waitStage;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &frameCtx.cmd;
  submitInfo.signalSemaphoreCount = 1;
  submitInfo.pSignalSemaphores = &renderFinished;
  if (vkQueueSubmit(graphicsQueue_, 1, &submitInfo, frame.inFlight) != VK_SUCCESS) {
    fail("Failed to submit command buffer");
  }

  VkPresentInfoKHR presentInfo{};
  presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
  presentInfo.waitSemaphoreCount = 1;
  presentInfo.pWaitSemaphores = &renderFinished;
  presentInfo.swapchainCount = 1;
  presentInfo.pSwapchains = &swapchain_.swapchain;
  presentInfo.pImageIndices = &frameCtx.imageIndex;

  const VkResult present = vkQueuePresentKHR(graphicsQueue_, &presentInfo);
  if (present == VK_ERROR_OUT_OF_DATE_KHR || present == VK_SUBOPTIMAL_KHR || window_.wasResized()) {
    recreateSwapchain();
  } else if (present != VK_SUCCESS) {
    fail("Failed to present swapchain image");
  }

  frameIndex_ = (frameIndex_ + 1) % kFramesInFlight;
}

void GfxDevice::waitIdle() const {
  if (device_.device) {
    vkDeviceWaitIdle(device_.device);
  }
}

void GfxDevice::immediateSubmit(const std::function<void(VkCommandBuffer)>& record) {
  VkCommandBufferAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocInfo.commandPool = uploadPool_;
  allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocInfo.commandBufferCount = 1;

  VkCommandBuffer cmd = VK_NULL_HANDLE;
  vkAllocateCommandBuffers(device_.device, &allocInfo, &cmd);

  VkCommandBufferBeginInfo beginInfo{};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
  vkBeginCommandBuffer(cmd, &beginInfo);
  record(cmd);
  vkEndCommandBuffer(cmd);

  VkSubmitInfo submitInfo{};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &cmd;
  vkResetFences(device_.device, 1, &uploadFence_);
  vkQueueSubmit(graphicsQueue_, 1, &submitInfo, uploadFence_);
  vkWaitForFences(device_.device, 1, &uploadFence_, VK_TRUE, UINT64_MAX);
  vkFreeCommandBuffers(device_.device, uploadPool_, 1, &cmd);
}

AllocatedBuffer GfxDevice::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                        VmaMemoryUsage memoryUsage) {
  VkBufferCreateInfo bufferInfo{};
  bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bufferInfo.size = size;
  bufferInfo.usage = usage;

  VmaAllocationCreateInfo allocInfo{};
  allocInfo.usage = memoryUsage;
  if (memoryUsage == VMA_MEMORY_USAGE_AUTO_PREFER_HOST ||
      memoryUsage == VMA_MEMORY_USAGE_CPU_TO_GPU || memoryUsage == VMA_MEMORY_USAGE_CPU_ONLY) {
    allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                      VMA_ALLOCATION_CREATE_MAPPED_BIT;
  } else {
    allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  }

  AllocatedBuffer out{};
  out.size = size;
  if (vmaCreateBuffer(allocator_, &bufferInfo, &allocInfo, &out.buffer, &out.allocation, &out.info) !=
      VK_SUCCESS) {
    fail("Failed to create buffer");
  }
  return out;
}

void GfxDevice::destroyBuffer(AllocatedBuffer& buffer) {
  if (buffer.buffer) {
    vmaDestroyBuffer(allocator_, buffer.buffer, buffer.allocation);
  }
  buffer = {};
}

void GfxDevice::uploadToBuffer(AllocatedBuffer& dst, const void* data, VkDeviceSize size) {
  uploadToBuffer(dst, data, size, 0);
}

void GfxDevice::uploadToBuffer(AllocatedBuffer& dst, const void* data, VkDeviceSize size,
                               VkDeviceSize dstOffset) {
  if (size == 0) {
    return;
  }
  if (dstOffset + size > dst.size) {
    fail("uploadToBuffer: write exceeds destination buffer size");
  }

  AllocatedBuffer staging =
      createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  std::memcpy(staging.info.pMappedData, data, static_cast<size_t>(size));
  if (vmaFlushAllocation(allocator_, staging.allocation, 0, size) != VK_SUCCESS) {
    destroyBuffer(staging);
    fail("Failed to flush buffer upload staging memory");
  }

  immediateSubmit([&](VkCommandBuffer cmd) {
    // Order shared-buffer users before the overwrite, including earlier submissions.
    VkBufferMemoryBarrier2 barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.buffer = dst.buffer;
    barrier.offset = dstOffset;
    barrier.size = size;
    VkDependencyInfo dependency{};
    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dependency.bufferMemoryBarrierCount = 1;
    dependency.pBufferMemoryBarriers = &barrier;
    vkCmdPipelineBarrier2(cmd, &dependency);

    VkBufferCopy copy{};
    copy.srcOffset = 0;
    copy.dstOffset = dstOffset;
    copy.size = size;
    vkCmdCopyBuffer(cmd, staging.buffer, dst.buffer, 1, &copy);

    // A host fence wait alone does not make transfer writes visible to GPU consumers.
    barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
    barrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT;
    vkCmdPipelineBarrier2(cmd, &dependency);
  });

  destroyBuffer(staging);
}

uint32_t GfxDevice::calcMipLevels(uint32_t width, uint32_t height) {
  uint32_t levels = 1;
  uint32_t w = width;
  uint32_t h = height;
  while (w > 1 || h > 1) {
    w = w > 1 ? (w / 2) : 1;
    h = h > 1 ? (h / 2) : 1;
    ++levels;
  }
  return levels;
}

AllocatedImage GfxDevice::createImage(VkExtent3D extent, VkFormat format, VkImageUsageFlags usage,
                                      VkImageAspectFlags aspect, bool dedicated,
                                      uint32_t mipLevels, VkSampleCountFlagBits samples) {
  mipLevels = std::max(1u, mipLevels);
  if (samples == VK_SAMPLE_COUNT_FLAG_BITS_MAX_ENUM || samples == 0) {
    samples = VK_SAMPLE_COUNT_1_BIT;
  }
  // Multisample images are single-mip only.
  if (samples != VK_SAMPLE_COUNT_1_BIT) {
    mipLevels = 1;
  }

  VkImageCreateInfo imageInfo{};
  imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageInfo.imageType = VK_IMAGE_TYPE_2D;
  imageInfo.format = format;
  imageInfo.extent = extent;
  imageInfo.mipLevels = mipLevels;
  imageInfo.arrayLayers = 1;
  imageInfo.samples = samples;
  imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
  imageInfo.usage = usage;
  imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

  VmaAllocationCreateInfo allocInfo{};
  allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  if (dedicated) {
    allocInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
  }

  AllocatedImage out{};
  out.extent = extent;
  out.format = format;
  out.mipLevels = mipLevels;
  out.layerCount = 1;
  out.samples = samples;
  out.imageType = VK_IMAGE_TYPE_2D;
  if (vmaCreateImage(allocator_, &imageInfo, &allocInfo, &out.image, &out.allocation, nullptr) !=
      VK_SUCCESS) {
    fail("Failed to create image");
  }

  VkImageViewCreateInfo viewInfo{};
  viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  viewInfo.image = out.image;
  viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
  viewInfo.format = format;
  viewInfo.subresourceRange.aspectMask = aspect;
  viewInfo.subresourceRange.levelCount = mipLevels;
  viewInfo.subresourceRange.layerCount = 1;
  if (vkCreateImageView(device_.device, &viewInfo, nullptr, &out.view) != VK_SUCCESS) {
    fail("Failed to create image view");
  }
  return out;
}

AllocatedImage GfxDevice::createImage3D(VkExtent3D extent, VkFormat format, VkImageUsageFlags usage,
                                        VkImageAspectFlags aspect, bool dedicated) {
  if (extent.width == 0 || extent.height == 0 || extent.depth == 0) {
    fail("createImage3D: extent must be non-zero in all dimensions");
  }

  VkImageCreateInfo imageInfo{};
  imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageInfo.imageType = VK_IMAGE_TYPE_3D;
  imageInfo.format = format;
  imageInfo.extent = extent;
  imageInfo.mipLevels = 1;
  imageInfo.arrayLayers = 1;
  imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
  imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
  imageInfo.usage = usage;
  imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

  VmaAllocationCreateInfo allocInfo{};
  allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
  if (dedicated) {
    allocInfo.flags = VMA_ALLOCATION_CREATE_DEDICATED_MEMORY_BIT;
  }

  AllocatedImage out{};
  out.extent = extent;
  out.format = format;
  out.mipLevels = 1;
  out.layerCount = 1;
  out.samples = VK_SAMPLE_COUNT_1_BIT;
  out.imageType = VK_IMAGE_TYPE_3D;
  if (vmaCreateImage(allocator_, &imageInfo, &allocInfo, &out.image, &out.allocation, nullptr) !=
      VK_SUCCESS) {
    fail("Failed to create 3D image");
  }

  VkImageViewCreateInfo viewInfo{};
  viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  viewInfo.image = out.image;
  viewInfo.viewType = VK_IMAGE_VIEW_TYPE_3D;
  viewInfo.format = format;
  viewInfo.subresourceRange.aspectMask = aspect;
  viewInfo.subresourceRange.levelCount = 1;
  viewInfo.subresourceRange.layerCount = 1;
  if (vkCreateImageView(device_.device, &viewInfo, nullptr, &out.view) != VK_SUCCESS) {
    fail("Failed to create 3D image view");
  }
  return out;
}

AllocatedImage GfxDevice::createCubemap(uint32_t size, VkFormat format, VkImageUsageFlags usage,
                                        uint32_t mipLevels) {
  mipLevels = std::max(1u, mipLevels);
  size = std::max(1u, size);

  VkImageCreateInfo imageInfo{};
  imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageInfo.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
  imageInfo.imageType = VK_IMAGE_TYPE_2D;
  imageInfo.format = format;
  imageInfo.extent = {size, size, 1};
  imageInfo.mipLevels = mipLevels;
  imageInfo.arrayLayers = 6;
  imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
  imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
  imageInfo.usage = usage;
  imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

  VmaAllocationCreateInfo allocInfo{};
  allocInfo.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;

  AllocatedImage out{};
  out.extent = {size, size, 1};
  out.format = format;
  out.mipLevels = mipLevels;
  out.layerCount = 6;
  out.imageType = VK_IMAGE_TYPE_2D;
  if (vmaCreateImage(allocator_, &imageInfo, &allocInfo, &out.image, &out.allocation, nullptr) !=
      VK_SUCCESS) {
    fail("Failed to create cubemap image");
  }

  VkImageViewCreateInfo viewInfo{};
  viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  viewInfo.image = out.image;
  viewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
  viewInfo.format = format;
  viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  viewInfo.subresourceRange.levelCount = mipLevels;
  viewInfo.subresourceRange.layerCount = 6;
  if (vkCreateImageView(device_.device, &viewInfo, nullptr, &out.view) != VK_SUCCESS) {
    fail("Failed to create cubemap image view");
  }
  return out;
}

void GfxDevice::destroyImage(AllocatedImage& image) {
  if (image.view) {
    vkDestroyImageView(device_.device, image.view, nullptr);
  }
  if (image.image) {
    vmaDestroyImage(allocator_, image.image, image.allocation);
  }
  image = {};
}

void GfxDevice::uploadToImage(AllocatedImage& image, const void* data, VkDeviceSize size,
                              VkExtent3D extent, bool generateMips) {
  AllocatedBuffer staging =
      createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  std::memcpy(staging.info.pMappedData, data, static_cast<size_t>(size));
  if (vmaFlushAllocation(allocator_, staging.allocation, 0, size) != VK_SUCCESS) {
    destroyBuffer(staging);
    fail("Failed to flush image upload staging memory");
  }

  const uint32_t mipLevels = std::max(1u, image.mipLevels);
  generateMips = generateMips && mipLevels > 1;

  immediateSubmit([&](VkCommandBuffer cmd) {
    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels);

    VkBufferImageCopy region{};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.layerCount = 1;
    region.imageExtent = extent;
    vkCmdCopyBufferToImage(cmd, staging.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                           &region);

    if (!generateMips) {
      transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                      VK_ACCESS_2_TRANSFER_WRITE_BIT,
                      VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                      VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels);
      return;
    }

    int32_t mipWidth = static_cast<int32_t>(extent.width);
    int32_t mipHeight = static_cast<int32_t>(extent.height);
    for (uint32_t i = 1; i < mipLevels; ++i) {
      transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                      VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                      VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                      VK_ACCESS_2_TRANSFER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 1);

      VkImageBlit blit{};
      blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      blit.srcSubresource.mipLevel = i - 1;
      blit.srcSubresource.layerCount = 1;
      blit.srcOffsets[1] = {mipWidth, mipHeight, 1};
      blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      blit.dstSubresource.mipLevel = i;
      blit.dstSubresource.layerCount = 1;
      blit.dstOffsets[1] = {mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1, 1};

      vkCmdBlitImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image.image,
                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

      transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                      VK_ACCESS_2_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                      VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 1);

      if (mipWidth > 1) {
        mipWidth /= 2;
      }
      if (mipHeight > 1) {
        mipHeight /= 2;
      }
    }

    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, mipLevels - 1, 1);
  });

  image.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  destroyBuffer(staging);
}

void GfxDevice::uploadToImage3D(AllocatedImage& image, const void* data, VkDeviceSize size,
                                VkExtent3D extent) {
  if (!data || image.image == VK_NULL_HANDLE) {
    fail("uploadToImage3D: invalid image or data");
  }
  if (extent.width == 0 || extent.height == 0 || extent.depth == 0) {
    fail("uploadToImage3D: extent must be non-zero in all dimensions");
  }
  if (image.mipLevels != 1) {
    fail("uploadToImage3D: mipmaps are not supported");
  }
  const VkDeviceSize expected = static_cast<VkDeviceSize>(extent.width) *
                                static_cast<VkDeviceSize>(extent.height) *
                                static_cast<VkDeviceSize>(extent.depth) * 8u;
  if (size != expected) {
    fail("uploadToImage3D: size must be tightly packed 8-byte texels");
  }

  AllocatedBuffer staging =
      createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  std::memcpy(staging.info.pMappedData, data, static_cast<size_t>(size));
  if (vmaFlushAllocation(allocator_, staging.allocation, 0, size) != VK_SUCCESS) {
    destroyBuffer(staging);
    fail("Failed to flush 3D image upload staging memory");
  }

  const VkImageLayout oldLayout = image.layout;
  const bool fromUndefined = oldLayout == VK_IMAGE_LAYOUT_UNDEFINED;
  const VkPipelineStageFlags2 srcStage =
      fromUndefined ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
                    : (VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                       VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT);
  const VkAccessFlags2 srcAccess = fromUndefined ? 0 : VK_ACCESS_2_SHADER_READ_BIT;

  immediateSubmit([&](VkCommandBuffer cmd) {
    transitionImage(cmd, image.image, oldLayout, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, srcStage,
                    srcAccess, VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
    VkBufferImageCopy region{};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.layerCount = 1;
    region.imageExtent = extent;
    vkCmdCopyBufferToImage(cmd, staging.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                           &region);
    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_WRITE_BIT,
                    VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_READ_BIT);
  });

  image.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  destroyBuffer(staging);
}

VkSampler GfxDevice::createSampler(VkFilter filter, VkSamplerAddressMode addressMode,
                                   bool anisotropy, uint32_t mipLevels, float mipLodBias,
                                   VkSamplerMipmapMode mipmapMode) {
  VkSamplerCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  info.magFilter = filter;
  info.minFilter = filter;
  info.mipmapMode = mipmapMode;
  info.addressModeU = addressMode;
  info.addressModeV = addressMode;
  info.addressModeW = addressMode;
  info.minLod = 0.0f;
  info.maxLod = static_cast<float>(std::max(1u, mipLevels));
  // Positive bias samples lower-res mips sooner (helps ground moiré / specular sparkle).
  const float maxBias = physicalDevice_.properties.limits.maxSamplerLodBias;
  info.mipLodBias = std::clamp(mipLodBias, -maxBias, maxBias);
  if (anisotropy) {
    info.anisotropyEnable = VK_TRUE;
    info.maxAnisotropy = physicalDevice_.properties.limits.maxSamplerAnisotropy;
  }

  VkSampler sampler = VK_NULL_HANDLE;
  if (vkCreateSampler(device_.device, &info, nullptr, &sampler) != VK_SUCCESS) {
    fail("Failed to create sampler");
  }
  return sampler;
}

void GfxDevice::destroySampler(VkSampler sampler) {
  if (sampler) {
    vkDestroySampler(device_.device, sampler, nullptr);
  }
}

VkShaderModule GfxDevice::loadShaderModule(const std::string& path) const {
  std::ifstream file(path, std::ios::ate | std::ios::binary);
  if (!file) {
    fail("Failed to open shader: " + path);
  }
  const size_t size = static_cast<size_t>(file.tellg());
  std::vector<uint32_t> buffer(size / 4);
  file.seekg(0);
  file.read(reinterpret_cast<char*>(buffer.data()), static_cast<std::streamsize>(size));

  VkShaderModuleCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
  info.codeSize = size;
  info.pCode = buffer.data();

  VkShaderModule module = VK_NULL_HANDLE;
  if (vkCreateShaderModule(device_.device, &info, nullptr, &module) != VK_SUCCESS) {
    fail("Failed to create shader module: " + path);
  }
  return module;
}

void GfxDevice::transitionImage(VkCommandBuffer cmd, VkImage image, VkImageLayout oldLayout,
                                VkImageLayout newLayout, VkPipelineStageFlags2 srcStage,
                                VkAccessFlags2 srcAccess, VkPipelineStageFlags2 dstStage,
                                VkAccessFlags2 dstAccess, VkImageAspectFlags aspect,
                                uint32_t baseMipLevel, uint32_t levelCount, uint32_t baseArrayLayer,
                                uint32_t layerCount) const {
  VkImageMemoryBarrier2 barrier{};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
  barrier.srcStageMask = srcStage;
  barrier.srcAccessMask = srcAccess;
  barrier.dstStageMask = dstStage;
  barrier.dstAccessMask = dstAccess;
  barrier.oldLayout = oldLayout;
  barrier.newLayout = newLayout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = image;
  barrier.subresourceRange.aspectMask = aspect;
  barrier.subresourceRange.baseMipLevel = baseMipLevel;
  barrier.subresourceRange.levelCount = levelCount;
  barrier.subresourceRange.baseArrayLayer = baseArrayLayer;
  barrier.subresourceRange.layerCount = layerCount;

  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.imageMemoryBarrierCount = 1;
  dependency.pImageMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(cmd, &dependency);
}

void GfxDevice::uploadCubemapRGBA32F(AllocatedImage& image, const float* data) {
  if (!data || image.layerCount != 6 || image.format != VK_FORMAT_R32G32B32A32_SFLOAT) {
    fail("uploadCubemapRGBA32F: invalid cubemap or data");
  }

  const uint32_t mipLevels = std::max(1u, image.mipLevels);
  uint32_t size = image.extent.width;
  VkDeviceSize totalFloats = 0;
  for (uint32_t mip = 0; mip < mipLevels; ++mip) {
    const uint32_t s = std::max(1u, size >> mip);
    totalFloats += static_cast<VkDeviceSize>(s) * s * 6u * 4u;
  }

  AllocatedBuffer staging = createBuffer(totalFloats * sizeof(float), VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                         VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  std::memcpy(staging.info.pMappedData, data, static_cast<size_t>(totalFloats * sizeof(float)));
  if (vmaFlushAllocation(allocator_, staging.allocation, 0, totalFloats * sizeof(float)) != VK_SUCCESS) {
    destroyBuffer(staging);
    fail("Failed to flush cubemap upload staging memory");
  }

  std::vector<VkBufferImageCopy> regions;
  regions.reserve(mipLevels * 6u);
  VkDeviceSize offsetBytes = 0;
  for (uint32_t mip = 0; mip < mipLevels; ++mip) {
    const uint32_t s = std::max(1u, size >> mip);
    const VkDeviceSize faceBytes = static_cast<VkDeviceSize>(s) * s * 4u * sizeof(float);
    for (uint32_t face = 0; face < 6; ++face) {
      VkBufferImageCopy region{};
      region.bufferOffset = offsetBytes;
      region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      region.imageSubresource.mipLevel = mip;
      region.imageSubresource.baseArrayLayer = face;
      region.imageSubresource.layerCount = 1;
      region.imageExtent = {s, s, 1};
      regions.push_back(region);
      offsetBytes += faceBytes;
    }
  }

  immediateSubmit([&](VkCommandBuffer cmd) {
    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, 6);
    vkCmdCopyBufferToImage(cmd, staging.buffer, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           static_cast<uint32_t>(regions.size()), regions.data());
    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, 6);
  });

  image.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  destroyBuffer(staging);
}

VkImageView GfxDevice::createImageView(AllocatedImage& image, VkImageViewType type, uint32_t baseMip,
                                       uint32_t mipCount, uint32_t baseLayer,
                                       uint32_t layerCount) const {
  VkImageViewCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  info.image = image.image;
  info.viewType = type;
  info.format = image.format;
  info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  info.subresourceRange.baseMipLevel = baseMip;
  info.subresourceRange.levelCount = mipCount;
  info.subresourceRange.baseArrayLayer = baseLayer;
  info.subresourceRange.layerCount = layerCount;
  VkImageView view = VK_NULL_HANDLE;
  if (vkCreateImageView(device_.device, &info, nullptr, &view) != VK_SUCCESS) {
    fail("Failed to create image view");
  }
  return view;
}

void GfxDevice::destroyImageView(VkImageView view) const {
  if (view) {
    vkDestroyImageView(device_.device, view, nullptr);
  }
}

void GfxDevice::generateCubemapMips(AllocatedImage& image) {
  if (image.layerCount != 6 || image.mipLevels <= 1) {
    return;
  }
  const uint32_t mipLevels = image.mipLevels;
  immediateSubmit([&](VkCommandBuffer cmd) {
    // mip0 is valid; higher mips are still UNDEFINED.
    transitionImage(cmd, image.image, image.layout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                    VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6);
    if (mipLevels > 1) {
      transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                      VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                      VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 1, mipLevels - 1, 0,
                      6);
    }

    int32_t mipWidth = static_cast<int32_t>(image.extent.width);
    int32_t mipHeight = static_cast<int32_t>(image.extent.height);
    for (uint32_t i = 1; i < mipLevels; ++i) {
      VkImageBlit blit{};
      blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      blit.srcSubresource.mipLevel = i - 1;
      blit.srcSubresource.baseArrayLayer = 0;
      blit.srcSubresource.layerCount = 6;
      blit.srcOffsets[1] = {mipWidth, mipHeight, 1};
      blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      blit.dstSubresource.mipLevel = i;
      blit.dstSubresource.baseArrayLayer = 0;
      blit.dstSubresource.layerCount = 6;
      blit.dstOffsets[1] = {mipWidth > 1 ? mipWidth / 2 : 1, mipHeight > 1 ? mipHeight / 2 : 1, 1};

      vkCmdBlitImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image.image,
                     VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);

      transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                      VK_ACCESS_2_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                      VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 1, 0, 6);

      if (i + 1 < mipLevels) {
        transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                        VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                        VK_ACCESS_2_TRANSFER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, i, 1, 0, 6);
      }

      if (mipWidth > 1) {
        mipWidth /= 2;
      }
      if (mipHeight > 1) {
        mipHeight /= 2;
      }
    }

    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, mipLevels - 1, 1, 0, 6);
  });
  image.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
}

void GfxDevice::downloadCubemapRGBA32F(AllocatedImage& image, std::vector<float>& out) {
  if (image.layerCount != 6 || image.format != VK_FORMAT_R32G32B32A32_SFLOAT) {
    fail("downloadCubemapRGBA32F: invalid cubemap");
  }
  const uint32_t mipLevels = std::max(1u, image.mipLevels);
  const uint32_t size = image.extent.width;
  VkDeviceSize totalFloats = 0;
  for (uint32_t mip = 0; mip < mipLevels; ++mip) {
    const uint32_t s = std::max(1u, size >> mip);
    totalFloats += static_cast<VkDeviceSize>(s) * s * 6u * 4u;
  }
  out.resize(static_cast<size_t>(totalFloats));

  AllocatedBuffer staging =
      createBuffer(totalFloats * sizeof(float), VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                   VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

  std::vector<VkBufferImageCopy> regions;
  regions.reserve(mipLevels * 6u);
  VkDeviceSize offsetBytes = 0;
  for (uint32_t mip = 0; mip < mipLevels; ++mip) {
    const uint32_t s = std::max(1u, size >> mip);
    const VkDeviceSize faceBytes = static_cast<VkDeviceSize>(s) * s * 4u * sizeof(float);
    for (uint32_t face = 0; face < 6; ++face) {
      VkBufferImageCopy region{};
      region.bufferOffset = offsetBytes;
      region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      region.imageSubresource.mipLevel = mip;
      region.imageSubresource.baseArrayLayer = face;
      region.imageSubresource.layerCount = 1;
      region.imageExtent = {s, s, 1};
      regions.push_back(region);
      offsetBytes += faceBytes;
    }
  }

  immediateSubmit([&](VkCommandBuffer cmd) {
    transitionImage(cmd, image.image, image.layout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                    VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, 6);
    vkCmdCopyImageToBuffer(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, staging.buffer,
                           static_cast<uint32_t>(regions.size()), regions.data());
    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, mipLevels, 0, 6);
  });
  image.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  std::memcpy(out.data(), staging.info.pMappedData, static_cast<size_t>(totalFloats * sizeof(float)));
  destroyBuffer(staging);
}

void GfxDevice::downloadImageRGBA32F(AllocatedImage& image, std::vector<float>& out) {
  if (image.format != VK_FORMAT_R32G32B32A32_SFLOAT) {
    fail("downloadImageRGBA32F: expected RGBA32F");
  }
  const uint32_t w = image.extent.width;
  const uint32_t h = image.extent.height;
  const VkDeviceSize floats = static_cast<VkDeviceSize>(w) * h * 4u;
  out.resize(static_cast<size_t>(floats));
  AllocatedBuffer staging = createBuffer(floats * sizeof(float), VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                         VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

  VkBufferImageCopy region{};
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.layerCount = 1;
  region.imageExtent = {w, h, 1};

  immediateSubmit([&](VkCommandBuffer cmd) {
    transitionImage(cmd, image.image, image.layout, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_READ_BIT,
                    VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
    vkCmdCopyImageToBuffer(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, staging.buffer, 1,
                           &region);
    transitionImage(cmd, image.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                    VK_ACCESS_2_TRANSFER_READ_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                    VK_ACCESS_2_SHADER_READ_BIT);
  });
  image.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  std::memcpy(out.data(), staging.info.pMappedData, static_cast<size_t>(floats * sizeof(float)));
  destroyBuffer(staging);
}
