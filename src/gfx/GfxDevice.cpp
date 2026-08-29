#include "gfx/GfxDevice.h"

#define VMA_IMPLEMENTATION
#include <vk_mem_alloc.h>

#include <algorithm>
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
  {
    std::ofstream log("vulkan_engine.log", std::ios::app);
    if (log) {
      log << "Using GPU: " << deviceName_ << '\n';
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

void GfxDevice::createSwapchain() {
  const VkExtent2D extent = window_.framebufferExtent();
  auto swapRet =
      vkb::SwapchainBuilder{device_, surface_}
          .set_desired_format({VK_FORMAT_B8G8R8A8_UNORM, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
          .add_fallback_format({VK_FORMAT_B8G8R8A8_SRGB, VK_COLOR_SPACE_SRGB_NONLINEAR_KHR})
          .set_desired_present_mode(VK_PRESENT_MODE_FIFO_KHR)
          .set_desired_extent(extent.width, extent.height)
          .add_image_usage_flags(VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT)
          .build();
  if (!swapRet) {
    fail(std::string("Failed to create swapchain: ") + swapRet.error().message());
  }

  swapchain_ = swapRet.value();
  swapchainExtent_ = swapchain_.extent;
  swapchainFormat_ = swapchain_.image_format;
  swapchainImages_ = swapchain_.get_images().value();
  swapchainImageViews_ = swapchain_.get_image_views().value();
  createSwapchainSync();
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
  VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;

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
    allocInfo.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
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
  AllocatedBuffer staging =
      createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  std::memcpy(staging.info.pMappedData, data, static_cast<size_t>(size));

  immediateSubmit([&](VkCommandBuffer cmd) {
    VkBufferCopy copy{};
    copy.size = size;
    vkCmdCopyBuffer(cmd, staging.buffer, dst.buffer, 1, &copy);
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
                                      uint32_t mipLevels) {
  mipLevels = std::max(1u, mipLevels);

  VkImageCreateInfo imageInfo{};
  imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageInfo.imageType = VK_IMAGE_TYPE_2D;
  imageInfo.format = format;
  imageInfo.extent = extent;
  imageInfo.mipLevels = mipLevels;
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
  out.mipLevels = mipLevels;
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
                      VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
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

VkSampler GfxDevice::createSampler(VkFilter filter, VkSamplerAddressMode addressMode,
                                   bool anisotropy, uint32_t mipLevels, float mipLodBias) {
  VkSamplerCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  info.magFilter = filter;
  info.minFilter = filter;
  info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
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
                                uint32_t baseMipLevel, uint32_t levelCount) const {
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
  barrier.subresourceRange.layerCount = 1;

  VkDependencyInfo dependency{};
  dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
  dependency.imageMemoryBarrierCount = 1;
  dependency.pImageMemoryBarriers = &barrier;
  vkCmdPipelineBarrier2(cmd, &dependency);
}
