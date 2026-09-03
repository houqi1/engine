#pragma once

#include "gfx/GfxDevice.h"
#include "gfx/GpuTypes.h"
#include "scene/VoxelScene.h"

#include <array>
#include <cstdint>

class VoxelRenderer {
public:
  explicit VoxelRenderer(GfxDevice& gfx);
  ~VoxelRenderer();

  void init(VoxelScene& scene);
  void resize();
  void draw(VoxelScene& scene, float displayFps);

private:
  // Must match shaders/voxel_dda.comp std140 layout.
  struct VoxelDdaUBO {
    float invView[16];
    float invProj[16];
    float cameraPos[3];
    float pad0;
    float lightDir[3];
    float ambient;
    float projX;
    float projY;
    uint32_t maxSteps;
    uint32_t renderMode;
    float skyColor[3];
    uint32_t skipTrace;
    float aoStrength;
    float aoPower;
    float skyYaw;
    float skyIntensity;
    uint32_t useSky;
    uint32_t objectCount;
    uint32_t debugHullTmin;
    uint32_t cameraInside;
  };

  struct HullPC {
    float viewProj[16];
    float model[16];
    float cameraPos[3];
    float nearZ;
    float cameraFwd[3];
    float pad;
  };

  struct FrameResources {
    AllocatedBuffer frameUBO{};
    VkDescriptorSet frameSet = VK_NULL_HANDLE;
    AllocatedImage tMin{};
    AllocatedImage tMax{};
    AllocatedImage tBack{};
  };

  enum TimestampSlot : uint32_t {
    kTsFrameBegin = 0,
    kTsAfterCompute = 1,
    kTsAfterBlit = 2,
    kTsFrameEnd = 3,
    kTsPerFrame = 4,
  };

  void createDescriptors();
  void createPipelines();
  void createOutputImage();
  void destroyOutputImage();
  void createIntervalImages();
  void destroyIntervalImages();
  void recordHullPass(VkCommandBuffer cmd, VoxelScene& scene, VkExtent2D extent,
                      uint32_t frameIndex);
  void updateDescriptors(VoxelScene& scene);
  void updateFrameUBO(VoxelScene& scene, uint32_t frameIndex);
  void createTimestampPool();
  void destroyTimestampPool();
  void writeTimestamp(VkCommandBuffer cmd, uint32_t queryIndex, VkPipelineStageFlags2 stage) const;
  void collectGpuTiming(uint32_t frameIndex);
  void initImGui();
  void shutdownImGui();
  void recordImGui(VkCommandBuffer cmd, VoxelScene& scene, float displayFps);

  GfxDevice& gfx_;

  VkDescriptorSetLayout frameLayout_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
  VkDescriptorPool imguiPool_ = VK_NULL_HANDLE;

  VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline computePipeline_ = VK_NULL_HANDLE;
  VkPipelineLayout hullPipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline hullPipeline_ = VK_NULL_HANDLE;

  AllocatedImage outImage_{};
  VkSampler intervalSampler_ = VK_NULL_HANDLE;
  VkFormat intervalFormat_ = VK_FORMAT_R32_SFLOAT;
  static constexpr VkFormat kOutFormat = VK_FORMAT_R8G8B8A8_UNORM;

  std::array<FrameResources, GfxDevice::kFramesInFlight> frames_{};
  std::array<VkImageView, VoxelScene::kGridTexCount> boundGridViews_{};
  VkSampler boundGridSampler_ = VK_NULL_HANDLE;
  std::array<VkBuffer, VoxelScene::kMaxBrickSlabs> boundBrickSlabs_{};
  uint32_t boundBrickSlabCount_ = 0;
  VkBuffer boundObjectBuffer_ = VK_NULL_HANDLE;
  VkBuffer boundPaletteBuffer_ = VK_NULL_HANDLE;
  VkBuffer boundOccMipBuffer_ = VK_NULL_HANDLE;
  VkImageView boundSkyView_ = VK_NULL_HANDLE;
  VkImageView boundTminView_ = VK_NULL_HANDLE;
  VkImageView boundTmaxView_ = VK_NULL_HANDLE;
  VkImageView boundTbackView_ = VK_NULL_HANDLE;
  bool imguiReady_ = false;
  float displayFps_ = 0.0f;
  bool skipTrace_ = false;
  bool showHullTmin_ = false;

  VkQueryPool timestampPool_ = VK_NULL_HANDLE;
  float timestampPeriodNs_ = 1.0f;
  std::array<bool, GfxDevice::kFramesInFlight> timestampPending_{};
  float gpuFrameMs_ = 0.0f;
  float gpuComputeMs_ = 0.0f;
  float gpuBlitMs_ = 0.0f;
  float gpuUiMs_ = 0.0f;
};
