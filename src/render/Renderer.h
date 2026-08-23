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
  void draw(Scene& scene, float dt, float fps);

  bool showUi() const { return showUi_; }

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
  void updateFrameUBO(Scene& scene, uint32_t frameIndex);
  void recordImGui(VkCommandBuffer cmd, const FrameContext& frame, Scene& scene, float fps);

  glm::mat4 computeLightViewProj(const Scene& scene) const;

  GfxDevice& gfx_;

  VkDescriptorSetLayout frameLayout_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout materialLayout_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
  VkDescriptorPool imguiPool_ = VK_NULL_HANDLE;

  VkPipelineLayout meshPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout shadowPipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout tonemapPipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline meshPipeline_ = VK_NULL_HANDLE;
  VkPipeline shadowPipeline_ = VK_NULL_HANDLE;
  VkPipeline tonemapPipeline_ = VK_NULL_HANDLE;

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
};
