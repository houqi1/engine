#pragma once

#include "gfx/GfxDevice.h"
#include "gfx/GpuTypes.h"
#include "scene/Scene.h"

#include <array>
#include <memory>
#include <unordered_map>
#include <vector>

struct ImGuiContext;

class Renderer {
public:
  explicit Renderer(GfxDevice& gfx);
  ~Renderer();

  void init(Scene& scene);
  void resize();
  void draw(Scene& scene, float dt, float displayFps);

  bool showUi() const { return showUi_; }
  const FrameStats& frameStats() const { return stats_; }

private:
  struct FrameResources {
    AllocatedBuffer frameUBO{};
    VkDescriptorSet frameSet = VK_NULL_HANDLE;
  };

  struct MaterialResources {
    AllocatedBuffer materialUBO{};
    VkDescriptorSet set = VK_NULL_HANDLE;
  };

  void createDescriptors();
  void createPipelines();
  void createRenderTargets();
  void destroyRenderTargets();
  void createShadowResources();
  void destroyShadowResources();
  void initImGui();
  void shutdownImGui();
  void ensureMaterialSet(Material* material);
  void syncMaterialUbo(Material* material);
  void updateFrameUBO(Scene& scene, uint32_t frameIndex);
  void recordGrass(VkCommandBuffer cmd, VkPipeline pipeline, VkPipelineLayout layout,
                   VkDescriptorSet frameSet, Scene& scene, bool shadowPass);
  void recordSky(VkCommandBuffer cmd, VkDescriptorSet frameSet, Scene& scene);
  void recordImGui(VkCommandBuffer cmd, const FrameContext& frame, Scene& scene);
  void createTimestampPool();
  void destroyTimestampPool();
  void writeTimestamp(VkCommandBuffer cmd, uint32_t queryIndex, VkPipelineStageFlags2 stage) const;
  void collectGpuTiming(uint32_t frameIndex);

  glm::mat4 computeLightViewProj(const Scene& scene) const;

  GfxDevice& gfx_;
  FrameStats stats_{};

  VkDescriptorSetLayout frameLayout_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout materialLayout_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout skyLayout_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout iblLayout_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
  VkDescriptorPool imguiPool_ = VK_NULL_HANDLE;

  VkPipelineLayout meshPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout shadowPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout tonemapPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout grassPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout grassShadowPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout skyPipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline meshPipeline_ = VK_NULL_HANDLE;
  VkPipeline shadowPipeline_ = VK_NULL_HANDLE;
  VkPipeline tonemapPipeline_ = VK_NULL_HANDLE;
  VkPipeline grassPipeline_ = VK_NULL_HANDLE;
  VkPipeline grassShadowPipeline_ = VK_NULL_HANDLE;
  VkPipeline skyPipeline_ = VK_NULL_HANDLE;
  VkDescriptorSet skySet_ = VK_NULL_HANDLE;
  VkDescriptorSet iblSet_ = VK_NULL_HANDLE;

  // Fallback 1x1 resources so mesh set=2 is always valid.
  AllocatedImage dummyCube_{};
  AllocatedImage dummyLut_{};
  VkSampler dummyCubeSampler_ = VK_NULL_HANDLE;
  VkSampler dummyLutSampler_ = VK_NULL_HANDLE;

  AllocatedImage depthImage_{};
  AllocatedImage hdrImage_{};
  AllocatedImage shadowImage_{};
  VkSampler shadowSampler_ = VK_NULL_HANDLE;
  VkSampler hdrSampler_ = VK_NULL_HANDLE;
  VkDescriptorSet tonemapSet_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout tonemapLayout_ = VK_NULL_HANDLE;

  static constexpr uint32_t kShadowMapSize = 2048;
  static constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;
  static constexpr VkFormat kHdrFormat = VK_FORMAT_R16G16B16A16_SFLOAT;

  std::array<FrameResources, GfxDevice::kFramesInFlight> frames_{};
  std::unordered_map<Material*, MaterialResources> materialSets_;

  bool imguiReady_ = false;
  bool showUi_ = true;
  bool showShadows_ = true;
  bool showSky_ = true;
  float skyIntensity_ = 1.0f;
  float skyYaw_ = 0.0f;
  float ambientScale_ = 0.35f;  // SH irradiance scale (diffuse IBL)
  bool useSkyAmbient_ = true;
  bool enablePrefilteredIbl_ = true;
  bool enableBrdfLutIbl_ = true;
  float specularIblScale_ = 1.0f;
  // Positive bias samples lower-res mips sooner (shader bias; MoltenVK lacks samplerMipLodBias).
  float mipLodBias_ = 0.5f;

  VkQueryPool timestampPool_ = VK_NULL_HANDLE;
  float timestampPeriodNs_ = 1.0f;
  std::array<bool, GfxDevice::kFramesInFlight> timestampPending_{};
};
