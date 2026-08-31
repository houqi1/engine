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
    float voxelSize;
    float lightDir[3];
    float ambient;
    float gridOrigin[3];
    float projX;
    uint32_t gridSize[3];
    uint32_t maxSteps;
    float skyColor[3];
    uint32_t renderMode;
    float projY;
    uint32_t nestedMicro;
    float pad2;
    float pad3;
  };

  struct FrameResources {
    AllocatedBuffer frameUBO{};
    VkDescriptorSet frameSet = VK_NULL_HANDLE;
  };

  void createDescriptors();
  void createPipelines();
  void createOutputImage();
  void destroyOutputImage();
  void updateDescriptors(VoxelScene& scene);
  void updateFrameUBO(VoxelScene& scene, uint32_t frameIndex);
  void initImGui();
  void shutdownImGui();
  void recordImGui(VkCommandBuffer cmd, VoxelScene& scene, float displayFps);

  GfxDevice& gfx_;

  VkDescriptorSetLayout frameLayout_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
  VkDescriptorPool imguiPool_ = VK_NULL_HANDLE;

  VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline computePipeline_ = VK_NULL_HANDLE;

  AllocatedImage outImage_{};
  static constexpr VkFormat kOutFormat = VK_FORMAT_R8G8B8A8_UNORM;

  std::array<FrameResources, GfxDevice::kFramesInFlight> frames_{};
  VkBuffer boundVoxelBuffer_ = VK_NULL_HANDLE;
  VkBuffer boundMicroBuffer_ = VK_NULL_HANDLE;
  bool imguiReady_ = false;
  float displayFps_ = 0.0f;
};
