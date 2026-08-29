#include "render/Renderer.h"

#include "gfx/PipelineBuilder.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <chrono>
#include <cstring>
#include <stdexcept>

namespace {

VkVertexInputBindingDescription vertexBinding() {
  VkVertexInputBindingDescription binding{};
  binding.binding = 0;
  binding.stride = sizeof(Vertex);
  binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
  return binding;
}

std::vector<VkVertexInputAttributeDescription> vertexAttributes() {
  return {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)},
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)},
      {2, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, uv)},
  };
}

void writeMat4(float* dst, const glm::mat4& m) {
  std::memcpy(dst, glm::value_ptr(m), sizeof(float) * 16);
}

void writeVec3(float* dst, const glm::vec3& v) {
  dst[0] = v.x;
  dst[1] = v.y;
  dst[2] = v.z;
}

}  // namespace

Renderer::Renderer(GfxDevice& gfx) : gfx_(gfx) {}

Renderer::~Renderer() {
  gfx_.waitIdle();
  shutdownImGui();
  destroyTimestampPool();

  for (auto& frame : frames_) {
    gfx_.destroyBuffer(frame.frameUBO);
  }
  for (auto& [mat, res] : materialSets_) {
    (void)mat;
    gfx_.destroyBuffer(res.materialUBO);
  }
  materialSets_.clear();

  if (meshPipeline_) {
    vkDestroyPipeline(gfx_.device(), meshPipeline_, nullptr);
  }
  if (shadowPipeline_) {
    vkDestroyPipeline(gfx_.device(), shadowPipeline_, nullptr);
  }
  if (tonemapPipeline_) {
    vkDestroyPipeline(gfx_.device(), tonemapPipeline_, nullptr);
  }
  if (grassPipeline_) {
    vkDestroyPipeline(gfx_.device(), grassPipeline_, nullptr);
  }
  if (grassShadowPipeline_) {
    vkDestroyPipeline(gfx_.device(), grassShadowPipeline_, nullptr);
  }
  if (skyPipeline_) {
    vkDestroyPipeline(gfx_.device(), skyPipeline_, nullptr);
  }
  if (meshPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), meshPipelineLayout_, nullptr);
  }
  if (shadowPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), shadowPipelineLayout_, nullptr);
  }
  if (tonemapPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), tonemapPipelineLayout_, nullptr);
  }
  if (grassPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), grassPipelineLayout_, nullptr);
  }
  if (grassShadowPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), grassShadowPipelineLayout_, nullptr);
  }
  if (skyPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), skyPipelineLayout_, nullptr);
  }
  if (frameLayout_) {
    vkDestroyDescriptorSetLayout(gfx_.device(), frameLayout_, nullptr);
  }
  if (materialLayout_) {
    vkDestroyDescriptorSetLayout(gfx_.device(), materialLayout_, nullptr);
  }
  if (tonemapLayout_) {
    vkDestroyDescriptorSetLayout(gfx_.device(), tonemapLayout_, nullptr);
  }
  if (skyLayout_) {
    vkDestroyDescriptorSetLayout(gfx_.device(), skyLayout_, nullptr);
  }
  if (descriptorPool_) {
    vkDestroyDescriptorPool(gfx_.device(), descriptorPool_, nullptr);
  }

  destroyShadowResources();
  destroyRenderTargets();
  if (hdrSampler_) {
    gfx_.destroySampler(hdrSampler_);
    hdrSampler_ = VK_NULL_HANDLE;
  }
}

void Renderer::createTimestampPool() {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(gfx_.physicalDevice(), &props);
  timestampPeriodNs_ = props.limits.timestampPeriod;

  VkQueryPoolCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
  info.queryType = VK_QUERY_TYPE_TIMESTAMP;
  info.queryCount = GfxDevice::kFramesInFlight * 2;
  if (vkCreateQueryPool(gfx_.device(), &info, nullptr, &timestampPool_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create timestamp query pool");
  }
  timestampPending_.fill(false);
}

void Renderer::destroyTimestampPool() {
  if (timestampPool_) {
    vkDestroyQueryPool(gfx_.device(), timestampPool_, nullptr);
    timestampPool_ = VK_NULL_HANDLE;
  }
}

void Renderer::writeTimestamp(VkCommandBuffer cmd, uint32_t queryIndex,
                              VkPipelineStageFlags2 stage) const {
  if (!timestampPool_) {
    return;
  }
  vkCmdWriteTimestamp2(cmd, stage, timestampPool_, queryIndex);
}

void Renderer::collectGpuTiming(uint32_t frameIndex) {
  if (!timestampPool_ || !timestampPending_[frameIndex]) {
    return;
  }

  const uint32_t firstQuery = frameIndex * 2;
  uint64_t stamps[2] = {0, 0};
  const VkResult result =
      vkGetQueryPoolResults(gfx_.device(), timestampPool_, firstQuery, 2, sizeof(stamps), stamps,
                            sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
  if (result != VK_SUCCESS) {
    return;
  }

  const double deltaTicks = static_cast<double>(stamps[1] - stamps[0]);
  const float gpuMs = static_cast<float>(deltaTicks * static_cast<double>(timestampPeriodNs_) * 1e-6);
  // EMA so the UI is readable.
  constexpr float alpha = 0.1f;
  stats_.gpuFrameMs = stats_.gpuFrameMs * (1.0f - alpha) + gpuMs * alpha;
  timestampPending_[frameIndex] = false;
}

void Renderer::init(Scene& scene) {
  createDescriptors();
  createRenderTargets();
  createShadowResources();
  createTimestampPool();
  createPipelines();

  for (auto& frame : frames_) {
    frame.frameUBO = gfx_.createBuffer(sizeof(FrameUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                       VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &frameLayout_;
    if (vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &frame.frameSet) != VK_SUCCESS) {
      throw std::runtime_error("Failed to allocate frame descriptor set");
    }

    VkDescriptorBufferInfo bufferInfo{};
    bufferInfo.buffer = frame.frameUBO.buffer;
    bufferInfo.range = sizeof(FrameUBO);

    VkDescriptorImageInfo shadowInfo{};
    shadowInfo.sampler = shadowSampler_;
    shadowInfo.imageView = shadowImage_.view;
    shadowInfo.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet writes[2]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = frame.frameSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &bufferInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = frame.frameSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[1].descriptorCount = 1;
    writes[1].pImageInfo = &shadowInfo;

    vkUpdateDescriptorSets(gfx_.device(), 2, writes, 0, nullptr);
  }

  for (const RenderObject& obj : scene.objects()) {
    ensureMaterialSet(obj.material);
  }

  // Tonemap set
  {
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &tonemapLayout_;
    vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &tonemapSet_);

    VkDescriptorImageInfo imageInfo{};
    imageInfo.sampler = hdrSampler_;
    imageInfo.imageView = hdrImage_.view;
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = tonemapSet_;
    write.dstBinding = 0;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.descriptorCount = 1;
    write.pImageInfo = &imageInfo;
    vkUpdateDescriptorSets(gfx_.device(), 1, &write, 0, nullptr);
  }

  // Sky set
  if (scene.hasSky()) {
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &skyLayout_;
    if (vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &skySet_) != VK_SUCCESS) {
      throw std::runtime_error("Failed to allocate sky descriptor set");
    }

    VkDescriptorImageInfo imageInfo{};
    imageInfo.sampler = scene.sky().sampler;
    imageInfo.imageView = scene.sky().image.view;
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = skySet_;
    write.dstBinding = 0;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.descriptorCount = 1;
    write.pImageInfo = &imageInfo;
    vkUpdateDescriptorSets(gfx_.device(), 1, &write, 0, nullptr);
  }

  initImGui();
}

void Renderer::resize() {
  gfx_.waitIdle();
  destroyRenderTargets();
  createRenderTargets();

  VkDescriptorImageInfo imageInfo{};
  imageInfo.sampler = hdrSampler_;
  imageInfo.imageView = hdrImage_.view;
  imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  VkWriteDescriptorSet write{};
  write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  write.dstSet = tonemapSet_;
  write.dstBinding = 0;
  write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  write.descriptorCount = 1;
  write.pImageInfo = &imageInfo;
  vkUpdateDescriptorSets(gfx_.device(), 1, &write, 0, nullptr);
}

void Renderer::createDescriptors() {
  // Frame layout
  {
    VkDescriptorSetLayoutBinding bindings[2]{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = 2;
    info.pBindings = bindings;
    vkCreateDescriptorSetLayout(gfx_.device(), &info, nullptr, &frameLayout_);
  }

  // Material layout
  {
    VkDescriptorSetLayoutBinding bindings[2]{};
    bindings[0].binding = 0;
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    bindings[1].binding = 1;
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = 2;
    info.pBindings = bindings;
    vkCreateDescriptorSetLayout(gfx_.device(), &info, nullptr, &materialLayout_);
  }

  // Tonemap layout
  {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = 1;
    info.pBindings = &binding;
    vkCreateDescriptorSetLayout(gfx_.device(), &info, nullptr, &tonemapLayout_);
  }

  // Sky equirect layout (set 1)
  {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = 1;
    info.pBindings = &binding;
    vkCreateDescriptorSetLayout(gfx_.device(), &info, nullptr, &skyLayout_);
  }

  VkDescriptorPoolSize sizes[] = {
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 64},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 64},
  };
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.maxSets = 128;
  poolInfo.poolSizeCount = 2;
  poolInfo.pPoolSizes = sizes;
  vkCreateDescriptorPool(gfx_.device(), &poolInfo, nullptr, &descriptorPool_);
}

void Renderer::createRenderTargets() {
  const VkExtent3D extent{gfx_.swapchainExtent().width, gfx_.swapchainExtent().height, 1};
  depthImage_ = gfx_.createImage(extent, kDepthFormat,
                                 VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT, VK_IMAGE_ASPECT_DEPTH_BIT,
                                 true);
  hdrImage_ = gfx_.createImage(extent, kHdrFormat,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               VK_IMAGE_ASPECT_COLOR_BIT, true);
  if (!hdrSampler_) {
    hdrSampler_ = gfx_.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false);
  }
}

void Renderer::destroyRenderTargets() {
  gfx_.destroyImage(depthImage_);
  gfx_.destroyImage(hdrImage_);
}

void Renderer::createShadowResources() {
  const VkExtent3D extent{kShadowMapSize, kShadowMapSize, 1};
  shadowImage_ = gfx_.createImage(
      extent, kDepthFormat,
      VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
      VK_IMAGE_ASPECT_DEPTH_BIT, true);

  // MoltenVK/portability: avoid comparison samplers; compare in shader instead.
  VkSamplerCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  info.magFilter = VK_FILTER_LINEAR;
  info.minFilter = VK_FILTER_LINEAR;
  info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
  info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
  info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
  info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
  info.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
  info.compareEnable = VK_FALSE;
  if (vkCreateSampler(gfx_.device(), &info, nullptr, &shadowSampler_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create shadow sampler");
  }
}

void Renderer::destroyShadowResources() {
  if (shadowSampler_) {
    vkDestroySampler(gfx_.device(), shadowSampler_, nullptr);
    shadowSampler_ = VK_NULL_HANDLE;
  }
  gfx_.destroyImage(shadowImage_);
}

void Renderer::createPipelines() {
  const std::string shaderDir = VE_SHADER_DIR;
  VkShaderModule meshVert = gfx_.loadShaderModule(shaderDir + "/mesh.vert.spv");
  VkShaderModule meshFrag = gfx_.loadShaderModule(shaderDir + "/mesh.frag.spv");
  VkShaderModule shadowVert = gfx_.loadShaderModule(shaderDir + "/shadow.vert.spv");
  VkShaderModule shadowFrag = gfx_.loadShaderModule(shaderDir + "/shadow.frag.spv");
  VkShaderModule fsVert = gfx_.loadShaderModule(shaderDir + "/fullscreen.vert.spv");
  VkShaderModule tonemapFrag = gfx_.loadShaderModule(shaderDir + "/tonemap.frag.spv");
  VkShaderModule grassVert = gfx_.loadShaderModule(shaderDir + "/grass.vert.spv");
  VkShaderModule grassFrag = gfx_.loadShaderModule(shaderDir + "/grass.frag.spv");
  VkShaderModule grassShadowVert = gfx_.loadShaderModule(shaderDir + "/grass_shadow.vert.spv");
  VkShaderModule grassShadowFrag = gfx_.loadShaderModule(shaderDir + "/grass_shadow.frag.spv");
  VkShaderModule skyFrag = gfx_.loadShaderModule(shaderDir + "/sky.frag.spv");

  VkPushConstantRange pushRange{};
  pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  pushRange.offset = 0;
  pushRange.size = sizeof(PushConstants);

  VkPushConstantRange grassPush{};
  grassPush.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  grassPush.offset = 0;
  grassPush.size = sizeof(GrassPushConstants);

  VkPushConstantRange skyPush{};
  skyPush.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
  skyPush.offset = 0;
  skyPush.size = sizeof(SkyPushConstants);

  {
    VkDescriptorSetLayout layouts[] = {frameLayout_, materialLayout_};
    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = 2;
    info.pSetLayouts = layouts;
    info.pushConstantRangeCount = 1;
    info.pPushConstantRanges = &pushRange;
    vkCreatePipelineLayout(gfx_.device(), &info, nullptr, &meshPipelineLayout_);
  }
  {
    VkDescriptorSetLayout layouts[] = {frameLayout_};
    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = 1;
    info.pSetLayouts = layouts;
    info.pushConstantRangeCount = 1;
    info.pPushConstantRanges = &pushRange;
    vkCreatePipelineLayout(gfx_.device(), &info, nullptr, &shadowPipelineLayout_);
  }
  {
    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = 1;
    info.pSetLayouts = &tonemapLayout_;
    vkCreatePipelineLayout(gfx_.device(), &info, nullptr, &tonemapPipelineLayout_);
  }
  {
    VkDescriptorSetLayout layouts[] = {frameLayout_};
    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = 1;
    info.pSetLayouts = layouts;
    info.pushConstantRangeCount = 1;
    info.pPushConstantRanges = &grassPush;
    vkCreatePipelineLayout(gfx_.device(), &info, nullptr, &grassPipelineLayout_);
    vkCreatePipelineLayout(gfx_.device(), &info, nullptr, &grassShadowPipelineLayout_);
  }
  {
    VkDescriptorSetLayout layouts[] = {frameLayout_, skyLayout_};
    VkPipelineLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    info.setLayoutCount = 2;
    info.pSetLayouts = layouts;
    info.pushConstantRangeCount = 1;
    info.pPushConstantRanges = &skyPush;
    vkCreatePipelineLayout(gfx_.device(), &info, nullptr, &skyPipelineLayout_);
  }

  const auto binding = vertexBinding();
  const auto attrs = vertexAttributes();

  meshPipeline_ = PipelineBuilder()
                      .setShaders(meshVert, meshFrag)
                      .setVertexInput(binding, attrs)
                      .setCullMode(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                      .setDepthTest(true, true, VK_COMPARE_OP_GREATER_OR_EQUAL)
                      .setColorFormat(kHdrFormat)
                      .setDepthFormat(kDepthFormat)
                      .setLayout(meshPipelineLayout_)
                      .build(gfx_.device());

  const std::vector<VkVertexInputAttributeDescription> shadowAttrs = {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)},
  };
  shadowPipeline_ = PipelineBuilder()
                        .setShaders(shadowVert, shadowFrag)
                        .setVertexInput(binding, shadowAttrs)
                        .setCullMode(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                        .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
                        .disableColorWrite()
                        .setDepthFormat(kDepthFormat)
                        .setLayout(shadowPipelineLayout_)
                        .build(gfx_.device());

  // Fullscreen tonemap has no vertex input
  tonemapPipeline_ = PipelineBuilder()
                         .setShaders(fsVert, tonemapFrag)
                         .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                         .setDepthTest(false, false, VK_COMPARE_OP_ALWAYS)
                         .setColorFormat(gfx_.swapchainFormat())
                         .setLayout(tonemapPipelineLayout_)
                         .build(gfx_.device());

  const std::vector<VkVertexInputBindingDescription> grassBindings = {
      binding,
      {1, static_cast<uint32_t>(sizeof(GrassInstance)), VK_VERTEX_INPUT_RATE_INSTANCE},
  };
  const std::vector<VkVertexInputAttributeDescription> grassAttrs = {
      {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, position)},
      {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)},
      {2, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, uv)},
      {3, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(GrassInstance, position)},
      {4, 1, VK_FORMAT_R32_SFLOAT, offsetof(GrassInstance, yaw)},
      {5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(GrassInstance, color)},
      {6, 1, VK_FORMAT_R32_SFLOAT, offsetof(GrassInstance, scale)},
  };

  grassPipeline_ = PipelineBuilder()
                       .setShaders(grassVert, grassFrag)
                       .setVertexInput(grassBindings, grassAttrs)
                       .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                       .setDepthTest(true, true, VK_COMPARE_OP_GREATER_OR_EQUAL)
                       .setColorFormat(kHdrFormat)
                       .setDepthFormat(kDepthFormat)
                       .setLayout(grassPipelineLayout_)
                       .build(gfx_.device());

  grassShadowPipeline_ = PipelineBuilder()
                             .setShaders(grassShadowVert, grassShadowFrag)
                             .setVertexInput(grassBindings, grassAttrs)
                             .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                             .setDepthTest(true, true, VK_COMPARE_OP_LESS_OR_EQUAL)
                             .disableColorWrite()
                             .setDepthFormat(kDepthFormat)
                             .setLayout(grassShadowPipelineLayout_)
                             .build(gfx_.device());

  // Reverse-Z: far/cleared depth is 0. Draw sky only into empty pixels.
  skyPipeline_ = PipelineBuilder()
                     .setShaders(fsVert, skyFrag)
                     .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                     .setDepthTest(true, false, VK_COMPARE_OP_EQUAL)
                     .setColorFormat(kHdrFormat)
                     .setDepthFormat(kDepthFormat)
                     .setLayout(skyPipelineLayout_)
                     .build(gfx_.device());

  vkDestroyShaderModule(gfx_.device(), meshVert, nullptr);
  vkDestroyShaderModule(gfx_.device(), meshFrag, nullptr);
  vkDestroyShaderModule(gfx_.device(), shadowVert, nullptr);
  vkDestroyShaderModule(gfx_.device(), shadowFrag, nullptr);
  vkDestroyShaderModule(gfx_.device(), fsVert, nullptr);
  vkDestroyShaderModule(gfx_.device(), tonemapFrag, nullptr);
  vkDestroyShaderModule(gfx_.device(), grassVert, nullptr);
  vkDestroyShaderModule(gfx_.device(), grassFrag, nullptr);
  vkDestroyShaderModule(gfx_.device(), grassShadowVert, nullptr);
  vkDestroyShaderModule(gfx_.device(), grassShadowFrag, nullptr);
  vkDestroyShaderModule(gfx_.device(), skyFrag, nullptr);
}

void Renderer::ensureMaterialSet(Material* material) {
  if (!material || materialSets_.count(material)) {
    return;
  }

  MaterialResources res{};
  res.materialUBO = gfx_.createBuffer(sizeof(MaterialUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                      VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

  MaterialUBO ubo{};
  ubo.baseColorFactor[0] = material->baseColor.r;
  ubo.baseColorFactor[1] = material->baseColor.g;
  ubo.baseColorFactor[2] = material->baseColor.b;
  ubo.baseColorFactor[3] = material->baseColor.a;
  ubo.metallic = material->metallic;
  ubo.roughness = material->roughness;
  ubo.shOnly = material->shOnly ? 1.0f : 0.0f;
  std::memcpy(res.materialUBO.info.pMappedData, &ubo, sizeof(ubo));

  VkDescriptorSetAllocateInfo allocInfo{};
  allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  allocInfo.descriptorPool = descriptorPool_;
  allocInfo.descriptorSetCount = 1;
  allocInfo.pSetLayouts = &materialLayout_;
  vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &res.set);

  VkDescriptorImageInfo imageInfo{};
  imageInfo.sampler = material->albedo->sampler;
  imageInfo.imageView = material->albedo->image.view;
  imageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

  VkDescriptorBufferInfo bufferInfo{};
  bufferInfo.buffer = res.materialUBO.buffer;
  bufferInfo.range = sizeof(MaterialUBO);

  VkWriteDescriptorSet writes[2]{};
  writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  writes[0].dstSet = res.set;
  writes[0].dstBinding = 0;
  writes[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  writes[0].descriptorCount = 1;
  writes[0].pImageInfo = &imageInfo;

  writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
  writes[1].dstSet = res.set;
  writes[1].dstBinding = 1;
  writes[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  writes[1].descriptorCount = 1;
  writes[1].pBufferInfo = &bufferInfo;
  vkUpdateDescriptorSets(gfx_.device(), 2, writes, 0, nullptr);

  materialSets_.emplace(material, res);
}

glm::mat4 Renderer::computeLightViewProj(const Scene& scene) const {
  const glm::vec3 dir = glm::normalize(scene.light().direction);
  const glm::vec3 center(0.0f, 1.0f, 0.0f);
  const glm::vec3 eye = center - dir * 20.0f;
  const glm::mat4 view = glm::lookAt(eye, center, glm::vec3(0, 1, 0));
  const glm::mat4 proj = glm::ortho(-18.0f, 18.0f, -18.0f, 18.0f, 1.0f, 40.0f);
  // Vulkan clip space Y flip for ortho
  glm::mat4 vulkanProj = proj;
  vulkanProj[1][1] *= -1.0f;
  return vulkanProj * view;
}

void Renderer::updateFrameUBO(Scene& scene, uint32_t frameIndex) {
  FrameUBO ubo{};
  writeMat4(ubo.view, scene.camera().view());
  writeMat4(ubo.proj, scene.camera().proj());
  writeMat4(ubo.lightViewProj, computeLightViewProj(scene));
  writeVec3(ubo.cameraPos, scene.camera().position());
  writeVec3(ubo.lightDir, glm::normalize(scene.light().direction));
  writeVec3(ubo.lightColor, scene.light().color);
  ubo.lightIntensity = scene.light().intensity;
  writeVec3(ubo.ambientColor, scene.light().ambient);
  ubo.shadowBias = scene.light().shadowBias;
  ubo.mipLodBias = mipLodBias_;
  ubo.skyYaw = skyYaw_;
  ubo.skyIntensity = skyIntensity_;

  const bool useSh = useSkyAmbient_ && scene.hasSkyIrradianceSH();
  ubo.ambientScale = useSh ? ambientScale_ : 0.0f;
  if (useSh) {
    const Sh9& sh = scene.skyIrradianceSH();
    for (int i = 0; i < 9; ++i) {
      ubo.ambientSH[i][0] = sh.c[i].r;
      ubo.ambientSH[i][1] = sh.c[i].g;
      ubo.ambientSH[i][2] = sh.c[i].b;
      ubo.ambientSH[i][3] = 0.0f;
    }
  }

  std::memcpy(frames_[frameIndex].frameUBO.info.pMappedData, &ubo, sizeof(ubo));
}

void Renderer::recordGrass(VkCommandBuffer cmd, VkPipeline pipeline, VkPipelineLayout layout,
                           VkDescriptorSet frameSet, Scene& scene, bool shadowPass) {
  GrassSystem& grass = scene.grass();
  if (!grass.params().enabled || !grass.ready()) {
    return;
  }

  const std::vector<GrassDrawBatch>& batches =
      shadowPass ? grass.shadowBatches() : grass.colorBatches();
  if (batches.empty()) {
    return;
  }

  GrassPushConstants pc{};
  pc.time = scene.time();
  pc.windStrength = grass.params().windStrength;
  pc.windFrequency = grass.params().windFrequency;

  vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);
  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, layout, 0, 1, &frameSet, 0, nullptr);
  vkCmdPushConstants(cmd, layout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);

  const Mesh& blade = grass.bladeMesh();
  VkBuffer vertexBuffers[] = {blade.vertexBuffer.buffer, grass.instanceBuffer()};
  VkDeviceSize offsets[] = {0, 0};
  vkCmdBindVertexBuffers(cmd, 0, 2, vertexBuffers, offsets);
  vkCmdBindIndexBuffer(cmd, blade.indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);

  for (const GrassDrawBatch& batch : batches) {
    if (batch.instanceCount == 0) {
      continue;
    }
    vkCmdDrawIndexed(cmd, blade.indexCount, batch.instanceCount, 0, 0, batch.firstInstance);
  }
}

void Renderer::recordSky(VkCommandBuffer cmd, VkDescriptorSet frameSet, Scene& scene) {
  if (!showSky_ || !scene.hasSky() || skySet_ == VK_NULL_HANDLE || !skyPipeline_) {
    return;
  }

  SkyPushConstants pc{};
  pc.intensity = skyIntensity_;
  pc.yaw = skyYaw_;

  vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline_);
  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipelineLayout_, 0, 1, &frameSet,
                          0, nullptr);
  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipelineLayout_, 1, 1, &skySet_,
                          0, nullptr);
  vkCmdPushConstants(cmd, skyPipelineLayout_, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);
  vkCmdDraw(cmd, 3, 1, 0, 0);
}

void Renderer::draw(Scene& scene, float /*dt*/, float displayFps) {
  if (gfx_.swapchainWasRecreated()) {
    resize();
    gfx_.clearSwapchainRecreatedFlag();
  }

  scene.grass().rebuildIfNeeded(gfx_);
  {
    const glm::mat4 cameraViewProj = scene.camera().proj() * scene.camera().view();
    const glm::mat4 lightViewProj = computeLightViewProj(scene);
    scene.grass().cull(scene.camera().position(), cameraViewProj, lightViewProj);
  }

  FrameContext frame{};
  if (!gfx_.beginFrame(frame)) {
    return;
  }

  // Fence wait in beginFrame means this slot's previous GPU work is done.
  collectGpuTiming(frame.frameIndex);

  stats_.displayFps = displayFps;
  stats_.displayFrameMs = (displayFps > 1e-3f) ? (1000.0f / displayFps) : 0.0f;

  const auto cpuRecordStart = std::chrono::steady_clock::now();

  updateFrameUBO(scene, frame.frameIndex);
  VkDescriptorSet frameSet = frames_[frame.frameIndex].frameSet;

  const uint32_t tsBegin = frame.frameIndex * 2;
  const uint32_t tsEnd = tsBegin + 1;
  if (timestampPool_) {
    vkCmdResetQueryPool(frame.cmd, timestampPool_, tsBegin, 2);
    writeTimestamp(frame.cmd, tsBegin, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
  }

  // ---- Shadow pass ----
  if (showShadows_) {
    gfx_.transitionImage(frame.cmd, shadowImage_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                         VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                         VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                         VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
                         VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);

    VkClearValue clear{};
    clear.depthStencil = {1.0f, 0};
    VkRenderingAttachmentInfo depthAttach{};
    depthAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
    depthAttach.imageView = shadowImage_.view;
    depthAttach.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
    depthAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depthAttach.clearValue = clear;

    VkRenderingInfo rendering{};
    rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
    rendering.renderArea.extent = {kShadowMapSize, kShadowMapSize};
    rendering.layerCount = 1;
    rendering.pDepthAttachment = &depthAttach;
    vkCmdBeginRendering(frame.cmd, &rendering);

    VkViewport vp{0, 0, (float)kShadowMapSize, (float)kShadowMapSize, 0, 1};
    VkRect2D scissor{{0, 0}, {kShadowMapSize, kShadowMapSize}};
    vkCmdSetViewport(frame.cmd, 0, 1, &vp);
    vkCmdSetScissor(frame.cmd, 0, 1, &scissor);
    vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline_);
    vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipelineLayout_, 0, 1,
                            &frameSet, 0, nullptr);

    for (const RenderObject& obj : scene.objects()) {
      if (!obj.castShadow || !obj.mesh) {
        continue;
      }
      PushConstants pc{};
      writeMat4(pc.model, obj.transform);
      vkCmdPushConstants(frame.cmd, shadowPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0,
                         sizeof(pc), &pc);
      VkDeviceSize offset = 0;
      vkCmdBindVertexBuffers(frame.cmd, 0, 1, &obj.mesh->vertexBuffer.buffer, &offset);
      vkCmdBindIndexBuffer(frame.cmd, obj.mesh->indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
      vkCmdDrawIndexed(frame.cmd, obj.mesh->indexCount, 1, 0, 0, 0);
    }

    recordGrass(frame.cmd, grassShadowPipeline_, grassShadowPipelineLayout_, frameSet, scene, true);

    vkCmdEndRendering(frame.cmd);

    gfx_.transitionImage(frame.cmd, shadowImage_.image, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                         VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL,
                         VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                         VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                         VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
                         VK_IMAGE_ASPECT_DEPTH_BIT);
  }

  // ---- Main HDR pass ----
  gfx_.transitionImage(frame.cmd, hdrImage_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                       0, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
  gfx_.transitionImage(frame.cmd, depthImage_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                       0, VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);

  VkClearValue colorClear{};
  colorClear.color = {{0.02f, 0.03f, 0.05f, 1.0f}};
  VkClearValue depthClear{};
  depthClear.depthStencil = {0.0f, 0};  // reverse-Z clear to 0

  VkRenderingAttachmentInfo colorAttach{};
  colorAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  colorAttach.imageView = hdrImage_.view;
  colorAttach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  colorAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  colorAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  colorAttach.clearValue = colorClear;

  VkRenderingAttachmentInfo depthAttach{};
  depthAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  depthAttach.imageView = depthImage_.view;
  depthAttach.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
  depthAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depthAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  depthAttach.clearValue = depthClear;

  VkRenderingInfo mainInfo{};
  mainInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  mainInfo.renderArea.extent = frame.extent;
  mainInfo.layerCount = 1;
  mainInfo.colorAttachmentCount = 1;
  mainInfo.pColorAttachments = &colorAttach;
  mainInfo.pDepthAttachment = &depthAttach;
  vkCmdBeginRendering(frame.cmd, &mainInfo);

  VkViewport vp{0, 0, (float)frame.extent.width, (float)frame.extent.height, 0, 1};
  VkRect2D scissor{{0, 0}, frame.extent};
  vkCmdSetViewport(frame.cmd, 0, 1, &vp);
  vkCmdSetScissor(frame.cmd, 0, 1, &scissor);
  vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipeline_);
  vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipelineLayout_, 0, 1,
                          &frameSet, 0, nullptr);

  for (const RenderObject& obj : scene.objects()) {
    ensureMaterialSet(obj.material);
    auto it = materialSets_.find(obj.material);
    if (it == materialSets_.end()) {
      continue;
    }
    vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipelineLayout_, 1, 1,
                            &it->second.set, 0, nullptr);
    PushConstants pc{};
    writeMat4(pc.model, obj.transform);
    vkCmdPushConstants(frame.cmd, meshPipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc),
                       &pc);
    VkDeviceSize offset = 0;
    vkCmdBindVertexBuffers(frame.cmd, 0, 1, &obj.mesh->vertexBuffer.buffer, &offset);
    vkCmdBindIndexBuffer(frame.cmd, obj.mesh->indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(frame.cmd, obj.mesh->indexCount, 1, 0, 0, 0);
  }

  recordGrass(frame.cmd, grassPipeline_, grassPipelineLayout_, frameSet, scene, false);
  recordSky(frame.cmd, frameSet, scene);

  vkCmdEndRendering(frame.cmd);

  // ---- Tonemap to swapchain ----
  gfx_.transitionImage(frame.cmd, hdrImage_.image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                       VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                       0, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

  VkClearValue swapClear{};
  swapClear.color = {{0, 0, 0, 1}};
  VkRenderingAttachmentInfo swapAttach{};
  swapAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  swapAttach.imageView = frame.swapchainView;
  swapAttach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  swapAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  swapAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  swapAttach.clearValue = swapClear;

  VkRenderingInfo tonemapInfo{};
  tonemapInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  tonemapInfo.renderArea.extent = frame.extent;
  tonemapInfo.layerCount = 1;
  tonemapInfo.colorAttachmentCount = 1;
  tonemapInfo.pColorAttachments = &swapAttach;
  vkCmdBeginRendering(frame.cmd, &tonemapInfo);
  vkCmdSetViewport(frame.cmd, 0, 1, &vp);
  vkCmdSetScissor(frame.cmd, 0, 1, &scissor);
  vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tonemapPipeline_);
  vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, tonemapPipelineLayout_, 0, 1,
                          &tonemapSet_, 0, nullptr);
  vkCmdDraw(frame.cmd, 3, 1, 0, 0);

  if (showUi_ && imguiReady_) {
    recordImGui(frame.cmd, frame, scene);
  }
  vkCmdEndRendering(frame.cmd);

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                       0);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsEnd, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT);
    timestampPending_[frame.frameIndex] = true;
  }

  const float cpuMs = std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() -
                                                              cpuRecordStart)
                          .count();
  constexpr float cpuAlpha = 0.1f;
  stats_.cpuRecordMs = stats_.cpuRecordMs * (1.0f - cpuAlpha) + cpuMs * cpuAlpha;

  gfx_.endFrame(frame);
}

void Renderer::initImGui() {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();

  VkDescriptorPoolSize poolSizes[] = {
      {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000},
  };
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
  poolInfo.maxSets = 1000;
  poolInfo.poolSizeCount = static_cast<uint32_t>(std::size(poolSizes));
  poolInfo.pPoolSizes = poolSizes;
  vkCreateDescriptorPool(gfx_.device(), &poolInfo, nullptr, &imguiPool_);

  ImGui_ImplGlfw_InitForVulkan(gfx_.window().handle(), true);

  static VkFormat sSwapchainFormat = VK_FORMAT_UNDEFINED;
  sSwapchainFormat = gfx_.swapchainFormat();

  ImGui_ImplVulkan_InitInfo initInfo{};
  initInfo.Instance = gfx_.instance();
  initInfo.PhysicalDevice = gfx_.physicalDevice();
  initInfo.Device = gfx_.device();
  initInfo.QueueFamily = gfx_.graphicsQueueFamily();
  initInfo.Queue = gfx_.graphicsQueue();
  initInfo.DescriptorPool = imguiPool_;
  initInfo.MinImageCount = GfxDevice::kFramesInFlight;
  initInfo.ImageCount = GfxDevice::kFramesInFlight;
  initInfo.UseDynamicRendering = true;
#ifdef IMGUI_IMPL_VULKAN_HAS_DYNAMIC_RENDERING
  initInfo.PipelineRenderingCreateInfo = {};
  initInfo.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  initInfo.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
  initInfo.PipelineRenderingCreateInfo.pColorAttachmentFormats = &sSwapchainFormat;
#endif

  ImGui_ImplVulkan_Init(&initInfo);
  imguiReady_ = true;
}

void Renderer::shutdownImGui() {
  if (!imguiReady_) {
    return;
  }
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  if (imguiPool_) {
    vkDestroyDescriptorPool(gfx_.device(), imguiPool_, nullptr);
    imguiPool_ = VK_NULL_HANDLE;
  }
  imguiReady_ = false;
}

void Renderer::recordImGui(VkCommandBuffer cmd, const FrameContext& /*frame*/, Scene& scene) {
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::Begin("Vulkan Engine");
  ImGui::TextWrapped("GPU: %s", gfx_.deviceName().c_str());
  ImGui::Separator();
  ImGui::Text("Timing (VSync ON)");
  ImGui::Text("Display FPS: %.1f  (%.2f ms)", stats_.displayFps, stats_.displayFrameMs);
  ImGui::Text("CPU record:  %.2f ms", stats_.cpuRecordMs);
  ImGui::Text("GPU work:    %.2f ms", stats_.gpuFrameMs);
  if (stats_.gpuFrameMs > 1e-3f) {
    ImGui::Text("GPU estimate: %.0f FPS  (if uncapped)", 1000.0f / stats_.gpuFrameMs);
  } else {
    ImGui::Text("GPU estimate: n/a");
  }
  ImGui::TextDisabled("Display FPS is capped by refresh rate; GPU estimate is actual work cost.");
  ImGui::Separator();
  ImGui::Checkbox("Shadows", &showShadows_);
  ImGui::Checkbox("Skybox", &showSky_);
  ImGui::DragFloat("Sky Intensity", &skyIntensity_, 0.01f, 0.0f, 8.0f);
  ImGui::DragFloat("Sky Yaw", &skyYaw_, 0.01f, -3.14159f, 3.14159f);
  ImGui::Checkbox("Grass Sky Ambient (SH)", &useSkyAmbient_);
  ImGui::DragFloat("Grass Ambient Scale", &ambientScale_, 0.01f, 0.0f, 4.0f);
  ImGui::TextDisabled("SH from sky HDR; yaw/intensity match skybox.");
  ImGui::DragFloat("Mip LOD Bias", &mipLodBias_, 0.01f, -2.0f, 4.0f);
  ImGui::Separator();
  ImGui::Text("Light");
  ImGui::DragFloat3("Direction", &scene.light().direction.x, 0.01f);
  ImGui::DragFloat("Intensity", &scene.light().intensity, 0.05f, 0.0f, 20.0f);
  ImGui::DragFloat("Shadow Bias", &scene.light().shadowBias, 0.0001f, 0.0f, 0.05f);
  ImGui::ColorEdit3("Light Color", &scene.light().color.x);
  ImGui::ColorEdit3("Ambient", &scene.light().ambient.x);
  ImGui::Separator();
  ImGui::Text("Grass (Phase 1.5 cull)");
  GrassParams& gp = scene.grass().params();
  const GrassCullStats& gs = scene.grass().cullStats();
  ImGui::Checkbox("Enable Grass", &gp.enabled);
  ImGui::Text("Stored instances: %u", scene.grass().instanceCount());
  ImGui::Text("Chunks: %u", gs.chunkCount);
  ImGui::Text("Color draw:  %u chunks / %u inst", gs.colorChunks, gs.colorInstances);
  ImGui::Text("Shadow draw: %u chunks / %u inst", gs.shadowChunks, gs.shadowInstances);
  ImGui::Checkbox("Frustum / Distance Cull", &gp.enableCulling);
  if (ImGui::SliderInt("Grid Resolution", &gp.gridResolution, 50, 700)) {
    scene.grass().markDirty();
  }
  if (ImGui::SliderInt("Chunk Size", &gp.chunkSize, 4, 64)) {
    scene.grass().markDirty();
  }
  if (ImGui::DragFloat("Area Half Extent", &gp.areaHalfExtent, 0.1f, 2.0f, 30.0f)) {
    scene.grass().markDirty();
  }
  ImGui::DragFloat("Full Density Dist", &gp.fullDensityDistance, 0.1f, 1.0f, 80.0f);
  ImGui::DragFloat("Half Density Dist", &gp.halfDensityDistance, 0.1f, 1.0f, 100.0f);
  ImGui::DragFloat("Max Draw Dist", &gp.maxDrawDistance, 0.1f, 2.0f, 150.0f);
  if (ImGui::DragFloat("Height Scale", &gp.heightScale, 0.01f, 0.1f, 2.0f)) {
    scene.grass().markDirty();
  }
  if (ImGui::DragFloat("Height Variance", &gp.heightVariance, 0.01f, 0.0f, 1.0f)) {
    scene.grass().markDirty();
  }
  ImGui::DragFloat("Wind Strength", &gp.windStrength, 0.01f, 0.0f, 2.0f);
  ImGui::DragFloat("Wind Frequency", &gp.windFrequency, 0.01f, 0.1f, 5.0f);
  if (ImGui::ColorEdit3("Base Color", &gp.baseColor.x)) {
    scene.grass().markDirty();
  }
  if (ImGui::ColorEdit3("Tip Color", &gp.tipColor.x)) {
    scene.grass().markDirty();
  }
  ImGui::Separator();
  ImGui::TextWrapped("Controls: WASD move, Q/E up/down, Right-drag look, Esc quit");
  ImGui::End();

  ImGui::Render();
  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
}

