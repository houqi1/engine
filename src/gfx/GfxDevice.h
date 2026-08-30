#pragma once

#include "core/Window.h"
#include "gfx/GpuTypes.h"

#include <VkBootstrap.h>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

struct FrameContext {
  VkCommandBuffer cmd = VK_NULL_HANDLE;
  uint32_t imageIndex = 0;
  uint32_t frameIndex = 0;
  VkImage swapchainImage = VK_NULL_HANDLE;
  VkImageView swapchainView = VK_NULL_HANDLE;
  VkExtent2D extent{};
  VkFormat swapchainFormat = VK_FORMAT_UNDEFINED;
};

class GfxDevice {
public:
  static constexpr uint32_t kFramesInFlight = 2;

  explicit GfxDevice(Window& window);
  ~GfxDevice();

  GfxDevice(const GfxDevice&) = delete;
  GfxDevice& operator=(const GfxDevice&) = delete;

  bool beginFrame(FrameContext& outFrame);
  void endFrame(const FrameContext& frame);
  void waitIdle() const;

  void immediateSubmit(const std::function<void(VkCommandBuffer)>& record);

  AllocatedBuffer createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                               VmaMemoryUsage memoryUsage);
  void destroyBuffer(AllocatedBuffer& buffer);
  void uploadToBuffer(AllocatedBuffer& dst, const void* data, VkDeviceSize size);

  AllocatedImage createImage(VkExtent3D extent, VkFormat format, VkImageUsageFlags usage,
                             VkImageAspectFlags aspect, bool dedicated = false,
                             uint32_t mipLevels = 1,
                             VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT);
  AllocatedImage createCubemap(uint32_t size, VkFormat format, VkImageUsageFlags usage,
                               uint32_t mipLevels = 1);
  void destroyImage(AllocatedImage& image);
  void uploadToImage(AllocatedImage& image, const void* data, VkDeviceSize size, VkExtent3D extent,
                     bool generateMips = false);
  // RGBA32F tightly packed: for each mip, 6 faces in +X,-X,+Y,-Y,+Z,-Z order.
  void uploadCubemapRGBA32F(AllocatedImage& image, const float* data);
  void downloadCubemapRGBA32F(AllocatedImage& image, std::vector<float>& out);
  void downloadImageRGBA32F(AllocatedImage& image, std::vector<float>& out);
  void generateCubemapMips(AllocatedImage& image);

  VkImageView createImageView(AllocatedImage& image, VkImageViewType type, uint32_t baseMip,
                              uint32_t mipCount, uint32_t baseLayer, uint32_t layerCount) const;
  void destroyImageView(VkImageView view) const;

  VkSampler createSampler(VkFilter filter, VkSamplerAddressMode addressMode, bool anisotropy,
                          uint32_t mipLevels = 1, float mipLodBias = 0.0f);
  void destroySampler(VkSampler sampler);

  VkShaderModule loadShaderModule(const std::string& path) const;

  void transitionImage(VkCommandBuffer cmd, VkImage image, VkImageLayout oldLayout,
                       VkImageLayout newLayout, VkPipelineStageFlags2 srcStage,
                       VkAccessFlags2 srcAccess, VkPipelineStageFlags2 dstStage,
                       VkAccessFlags2 dstAccess,
                       VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT,
                       uint32_t baseMipLevel = 0, uint32_t levelCount = 1,
                       uint32_t baseArrayLayer = 0, uint32_t layerCount = 1) const;

  static uint32_t calcMipLevels(uint32_t width, uint32_t height);

  Window& window() { return window_; }
  VkInstance instance() const { return instance_.instance; }
  VkDevice device() const { return device_.device; }
  VkPhysicalDevice physicalDevice() const { return physicalDevice_.physical_device; }
  VkQueue graphicsQueue() const { return graphicsQueue_; }
  uint32_t graphicsQueueFamily() const { return graphicsQueueFamily_; }
  VmaAllocator allocator() const { return allocator_; }
  VkExtent2D swapchainExtent() const { return swapchainExtent_; }
  VkFormat swapchainFormat() const { return swapchainFormat_; }
  uint32_t framesInFlight() const { return kFramesInFlight; }
  bool swapchainWasRecreated() const { return swapchainRecreated_; }
  void clearSwapchainRecreatedFlag() { swapchainRecreated_ = false; }
  const std::string& deviceName() const { return deviceName_; }
  // Preferred MSAA for scene color/depth (4x when supported, else 2x/1x).
  VkSampleCountFlagBits msaaSamples() const { return msaaSamples_; }

private:
  struct FrameData {
    VkCommandPool commandPool = VK_NULL_HANDLE;
    VkCommandBuffer commandBuffer = VK_NULL_HANDLE;
    VkSemaphore imageAvailable = VK_NULL_HANDLE;
    VkFence inFlight = VK_NULL_HANDLE;
  };

  void createInstance();
  void createSurface();
  void selectDevice();
  void createAllocator();
  void createSwapchain();
  void createFrameData();
  void createSwapchainSync();
  void destroySwapchainSync();
  void destroySwapchain();
  void recreateSwapchain();

  Window& window_;

  vkb::Instance instance_{};
  vkb::InstanceDispatchTable instanceDispatch_{};
  VkSurfaceKHR surface_ = VK_NULL_HANDLE;

  vkb::PhysicalDevice physicalDevice_{};
  vkb::Device device_{};
  std::string deviceName_;
  VkSampleCountFlagBits msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;
  VkQueue graphicsQueue_ = VK_NULL_HANDLE;
  uint32_t graphicsQueueFamily_ = 0;
  VmaAllocator allocator_ = VK_NULL_HANDLE;

  vkb::Swapchain swapchain_{};
  VkExtent2D swapchainExtent_{};
  VkFormat swapchainFormat_ = VK_FORMAT_UNDEFINED;
  std::vector<VkImage> swapchainImages_;
  std::vector<VkImageView> swapchainImageViews_;
  std::vector<VkSemaphore> renderFinishedSemaphores_;
  std::vector<VkFence> imagesInFlight_;
  bool swapchainRecreated_ = false;

  std::vector<FrameData> frames_;
  uint32_t frameIndex_ = 0;

  VkCommandPool uploadPool_ = VK_NULL_HANDLE;
  VkFence uploadFence_ = VK_NULL_HANDLE;
};
