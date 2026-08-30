#pragma once

#include "gfx/GfxDevice.h"
#include "gfx/GpuTypes.h"
#include "scene/VoxelScene.h"

#include <array>

struct ImGuiContext;

class VoxelRenderer {
public:
  explicit VoxelRenderer(GfxDevice& gfx);
  ~VoxelRenderer();

  void init();
  void resize();
  void draw(VoxelScene& scene, float displayFps);

private:
  struct VoxelFrameUBO {
    float view[16];
    float proj[16];
    float cameraPos[3];
    float pad0;
    float lightDir[3];
    float pad1;
  };

  struct VoxelPushConstants {
    float model[16];
    float color[4];
  };

  struct FrameResources {
    AllocatedBuffer frameUBO{};
    VkDescriptorSet frameSet = VK_NULL_HANDLE;
  };

  void createDescriptors();
  void createPipelines();
  void createMsaaTargets();
  void destroyMsaaTargets();
  void initImGui();
  void shutdownImGui();
  void updateFrameUBO(VoxelScene& scene, uint32_t frameIndex);
  void recordImGui(VkCommandBuffer cmd, VoxelScene& scene, float displayFps);

  GfxDevice& gfx_;
  VkSampleCountFlagBits msaaSamples_ = VK_SAMPLE_COUNT_1_BIT;

  VkDescriptorSetLayout frameLayout_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
  VkDescriptorPool imguiPool_ = VK_NULL_HANDLE;

  VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;

  AllocatedImage colorMsaa_{};
  AllocatedImage depthMsaa_{};
  static constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

  std::array<FrameResources, GfxDevice::kFramesInFlight> frames_{};
  bool imguiReady_ = false;
  float displayFps_ = 0.0f;
};
