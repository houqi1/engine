#include "render/VoxelRenderer.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

void writeMat4(float* dst, const glm::mat4& m) {
  std::memcpy(dst, glm::value_ptr(m), sizeof(float) * 16);
}

void writeVec3(float* dst, const glm::vec3& v) {
  dst[0] = v.x;
  dst[1] = v.y;
  dst[2] = v.z;
}

}  // namespace

VoxelRenderer::VoxelRenderer(GfxDevice& gfx) : gfx_(gfx) {}

VoxelRenderer::~VoxelRenderer() {
  gfx_.waitIdle();
  shutdownImGui();
  destroyOutputImage();
  destroyTimestampPool();

  if (computePipeline_) {
    vkDestroyPipeline(gfx_.device(), computePipeline_, nullptr);
  }
  if (pipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), pipelineLayout_, nullptr);
  }
  for (auto& frame : frames_) {
    gfx_.destroyBuffer(frame.frameUBO);
  }
  if (descriptorPool_) {
    vkDestroyDescriptorPool(gfx_.device(), descriptorPool_, nullptr);
  }
  if (frameLayout_) {
    vkDestroyDescriptorSetLayout(gfx_.device(), frameLayout_, nullptr);
  }
}

void VoxelRenderer::init(VoxelScene& scene) {
  createDescriptors();
  createOutputImage();
  createTimestampPool();
  createPipelines();

  for (auto& frame : frames_) {
    frame.frameUBO = gfx_.createBuffer(sizeof(VoxelDdaUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                       VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &frameLayout_;
    if (vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &frame.frameSet) != VK_SUCCESS) {
      throw std::runtime_error("Failed to allocate voxel DDA descriptor set");
    }
  }

  updateDescriptors(scene);
  initImGui();
}

void VoxelRenderer::createTimestampPool() {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(gfx_.physicalDevice(), &props);
  timestampPeriodNs_ = props.limits.timestampPeriod;

  VkQueryPoolCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
  info.queryType = VK_QUERY_TYPE_TIMESTAMP;
  info.queryCount = GfxDevice::kFramesInFlight * kTsPerFrame;
  if (vkCreateQueryPool(gfx_.device(), &info, nullptr, &timestampPool_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel timestamp query pool");
  }
  timestampPending_.fill(false);
}

void VoxelRenderer::destroyTimestampPool() {
  if (timestampPool_) {
    vkDestroyQueryPool(gfx_.device(), timestampPool_, nullptr);
    timestampPool_ = VK_NULL_HANDLE;
  }
}

void VoxelRenderer::writeTimestamp(VkCommandBuffer cmd, uint32_t queryIndex,
                                   VkPipelineStageFlags2 stage) const {
  if (!timestampPool_) {
    return;
  }
  vkCmdWriteTimestamp2(cmd, stage, timestampPool_, queryIndex);
}

void VoxelRenderer::collectGpuTiming(uint32_t frameIndex) {
  if (!timestampPool_ || !timestampPending_[frameIndex]) {
    return;
  }

  const uint32_t firstQuery = frameIndex * kTsPerFrame;
  uint64_t stamps[kTsPerFrame] = {};
  const VkResult result = vkGetQueryPoolResults(
      gfx_.device(), timestampPool_, firstQuery, kTsPerFrame, sizeof(stamps), stamps,
      sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
  if (result != VK_SUCCESS) {
    return;
  }

  auto toMs = [&](uint64_t a, uint64_t b) {
    const double deltaTicks = static_cast<double>(b - a);
    return static_cast<float>(deltaTicks * static_cast<double>(timestampPeriodNs_) * 1e-6);
  };

  const float totalMs = toMs(stamps[kTsFrameBegin], stamps[kTsFrameEnd]);
  const float computeMs = toMs(stamps[kTsFrameBegin], stamps[kTsAfterCompute]);
  const float blitMs = toMs(stamps[kTsAfterCompute], stamps[kTsAfterBlit]);
  const float uiMs = toMs(stamps[kTsAfterBlit], stamps[kTsFrameEnd]);

  constexpr float alpha = 0.15f;
  gpuFrameMs_ = gpuFrameMs_ * (1.0f - alpha) + totalMs * alpha;
  gpuComputeMs_ = gpuComputeMs_ * (1.0f - alpha) + computeMs * alpha;
  gpuBlitMs_ = gpuBlitMs_ * (1.0f - alpha) + blitMs * alpha;
  gpuUiMs_ = gpuUiMs_ * (1.0f - alpha) + uiMs * alpha;
  timestampPending_[frameIndex] = false;
}

void VoxelRenderer::resize() {
  destroyOutputImage();
  createOutputImage();
  // Force descriptor refresh so storage-image views stay valid.
  boundVoxelBuffer_ = VK_NULL_HANDLE;
  boundBrickSlabs_.fill(VK_NULL_HANDLE);
  boundBrickSlabCount_ = 0;
  boundObjectBuffer_ = VK_NULL_HANDLE;
  boundSkyView_ = VK_NULL_HANDLE;
}

void VoxelRenderer::createDescriptors() {
  VkDescriptorSetLayoutBinding bindings[6]{};
  bindings[0].binding = 0;
  bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  bindings[0].descriptorCount = 1;
  bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[1].binding = 1;
  bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[1].descriptorCount = 1;
  bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[2].binding = 2;
  bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  bindings[2].descriptorCount = 1;
  bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[3].binding = 3;
  bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[3].descriptorCount = VoxelScene::kMaxBrickSlabs;
  bindings[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[4].binding = 4;
  bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  bindings[4].descriptorCount = 1;
  bindings[4].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[5].binding = 5;
  bindings[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[5].descriptorCount = 1;
  bindings[5].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  VkDescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  layoutInfo.bindingCount = 6;
  layoutInfo.pBindings = bindings;
  if (vkCreateDescriptorSetLayout(gfx_.device(), &layoutInfo, nullptr, &frameLayout_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel DDA set layout");
  }

  VkDescriptorPoolSize poolSizes[] = {
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, GfxDevice::kFramesInFlight},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
       GfxDevice::kFramesInFlight * (2u + VoxelScene::kMaxBrickSlabs)},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, GfxDevice::kFramesInFlight},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, GfxDevice::kFramesInFlight},
  };
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.maxSets = GfxDevice::kFramesInFlight;
  poolInfo.poolSizeCount = static_cast<uint32_t>(std::size(poolSizes));
  poolInfo.pPoolSizes = poolSizes;
  if (vkCreateDescriptorPool(gfx_.device(), &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel DDA descriptor pool");
  }
}

void VoxelRenderer::createOutputImage() {
  const VkExtent2D ext = gfx_.swapchainExtent();
  if (ext.width == 0 || ext.height == 0) {
    return;
  }
  outImage_ = gfx_.createImage({ext.width, ext.height, 1}, kOutFormat,
                               VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                               VK_IMAGE_ASPECT_COLOR_BIT, true);
}

void VoxelRenderer::destroyOutputImage() {
  gfx_.destroyImage(outImage_);
}

void VoxelRenderer::createPipelines() {
  const std::string shaderDir = VE_SHADER_DIR;
  VkShaderModule comp = gfx_.loadShaderModule(shaderDir + "/voxel_dda.comp.spv");

  VkPipelineLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layoutInfo.setLayoutCount = 1;
  layoutInfo.pSetLayouts = &frameLayout_;
  if (vkCreatePipelineLayout(gfx_.device(), &layoutInfo, nullptr, &pipelineLayout_) != VK_SUCCESS) {
    vkDestroyShaderModule(gfx_.device(), comp, nullptr);
    throw std::runtime_error("Failed to create voxel DDA pipeline layout");
  }

  VkPipelineShaderStageCreateInfo stage{};
  stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  stage.module = comp;
  stage.pName = "main";

  VkComputePipelineCreateInfo pipeInfo{};
  pipeInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeInfo.stage = stage;
  pipeInfo.layout = pipelineLayout_;
  if (vkCreateComputePipelines(gfx_.device(), VK_NULL_HANDLE, 1, &pipeInfo, nullptr,
                               &computePipeline_) != VK_SUCCESS) {
    vkDestroyShaderModule(gfx_.device(), comp, nullptr);
    throw std::runtime_error("Failed to create voxel DDA compute pipeline");
  }

  vkDestroyShaderModule(gfx_.device(), comp, nullptr);
}

void VoxelRenderer::updateDescriptors(VoxelScene& scene) {
  if (outImage_.view == VK_NULL_HANDLE || scene.voxelBuffer().buffer == VK_NULL_HANDLE ||
      scene.dummyBrickSlabBuffer().buffer == VK_NULL_HANDLE ||
      scene.objectBuffer().buffer == VK_NULL_HANDLE || !scene.hasSky()) {
    return;
  }

  for (auto& frame : frames_) {
    VkDescriptorBufferInfo uboInfo{};
    uboInfo.buffer = frame.frameUBO.buffer;
    uboInfo.range = sizeof(VoxelDdaUBO);

    VkDescriptorBufferInfo voxelInfo{};
    voxelInfo.buffer = scene.voxelBuffer().buffer;
    voxelInfo.range = scene.voxelBuffer().size;

    VkDescriptorImageInfo imageInfo{};
    imageInfo.imageView = outImage_.view;
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorBufferInfo brickInfos[VoxelScene::kMaxBrickSlabs]{};
    for (uint32_t i = 0; i < VoxelScene::kMaxBrickSlabs; ++i) {
      if (i < scene.brickSlabCount() && scene.brickSlabBuffer(i).buffer != VK_NULL_HANDLE) {
        brickInfos[i].buffer = scene.brickSlabBuffer(i).buffer;
        brickInfos[i].range = scene.brickSlabBuffer(i).size;
      } else {
        brickInfos[i].buffer = scene.dummyBrickSlabBuffer().buffer;
        brickInfos[i].range = scene.dummyBrickSlabBuffer().size;
      }
    }

    VkDescriptorImageInfo skyInfo{};
    skyInfo.sampler = scene.sky().sampler;
    skyInfo.imageView = scene.sky().image.view;
    skyInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorBufferInfo objectInfo{};
    objectInfo.buffer = scene.objectBuffer().buffer;
    objectInfo.range = scene.objectBuffer().size;

    VkWriteDescriptorSet writes[6]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = frame.frameSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uboInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = frame.frameSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &voxelInfo;

    writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[2].dstSet = frame.frameSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo = &imageInfo;

    writes[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[3].dstSet = frame.frameSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = VoxelScene::kMaxBrickSlabs;
    writes[3].pBufferInfo = brickInfos;

    writes[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[4].dstSet = frame.frameSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[4].descriptorCount = 1;
    writes[4].pImageInfo = &skyInfo;

    writes[5].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[5].dstSet = frame.frameSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &objectInfo;

    vkUpdateDescriptorSets(gfx_.device(), 6, writes, 0, nullptr);
  }

  boundVoxelBuffer_ = scene.voxelBuffer().buffer;
  boundBrickSlabCount_ = scene.brickSlabCount();
  boundBrickSlabs_.fill(VK_NULL_HANDLE);
  for (uint32_t i = 0; i < boundBrickSlabCount_; ++i) {
    boundBrickSlabs_[i] = scene.brickSlabBuffer(i).buffer;
  }
  boundObjectBuffer_ = scene.objectBuffer().buffer;
  boundSkyView_ = scene.sky().image.view;
}

void VoxelRenderer::updateFrameUBO(VoxelScene& scene, uint32_t frameIndex) {
  const glm::mat4 view = scene.camera().view();
  const glm::mat4 proj = scene.camera().proj();

  VoxelDdaUBO ubo{};
  writeMat4(ubo.invView, glm::inverse(view));
  writeMat4(ubo.invProj, glm::inverse(proj));
  writeVec3(ubo.cameraPos, scene.camera().position());
  ubo.pad0 = 0.0f;
  writeVec3(ubo.lightDir, glm::normalize(scene.lightDir()));
  ubo.ambient = scene.ambient();
  ubo.projX = proj[0][0];
  ubo.projY = proj[1][1];
  ubo.maxSteps = scene.maxSteps();
  ubo.renderMode = static_cast<uint32_t>(std::max(0, scene.renderMode()));
  writeVec3(ubo.skyColor, scene.skyColor());
  ubo.skipTrace = skipTrace_ ? 1u : 0u;
  ubo.aoStrength = scene.aoStrength();
  ubo.aoPower = scene.aoPower();
  ubo.skyYaw = scene.skyYaw();
  ubo.skyIntensity = scene.skyIntensity();
  ubo.useSky = (scene.showSky() && scene.hasSky()) ? 1u : 0u;
  ubo.objectCount = scene.objectCount();
  ubo.pad1[0] = ubo.pad1[1] = 0.0f;

  void* mapped = frames_[frameIndex].frameUBO.info.pMappedData;
  if (!mapped) {
    throw std::runtime_error("Voxel DDA UBO is not host-mapped");
  }
  std::memcpy(mapped, &ubo, sizeof(ubo));
}

void VoxelRenderer::draw(VoxelScene& scene, float displayFps) {
  displayFps_ = displayFps;

  if (gfx_.swapchainWasRecreated()) {
    resize();
    gfx_.clearSwapchainRecreatedFlag();
  }

  if (outImage_.image == VK_NULL_HANDLE) {
    createOutputImage();
  }
  bool brickSlabsChanged = scene.brickSlabCount() != boundBrickSlabCount_;
  for (uint32_t i = 0; i < scene.brickSlabCount() && !brickSlabsChanged; ++i) {
    brickSlabsChanged = scene.brickSlabBuffer(i).buffer != boundBrickSlabs_[i];
  }
  if (scene.voxelBuffer().buffer != boundVoxelBuffer_ || brickSlabsChanged ||
      scene.objectBuffer().buffer != boundObjectBuffer_ ||
      scene.sky().image.view != boundSkyView_ || outImage_.view == VK_NULL_HANDLE) {
    updateDescriptors(scene);
  }

  FrameContext frame{};
  if (!gfx_.beginFrame(frame)) {
    return;
  }

  if (outImage_.image == VK_NULL_HANDLE || scene.voxelBuffer().buffer == VK_NULL_HANDLE ||
      scene.objectBuffer().buffer == VK_NULL_HANDLE) {
    gfx_.endFrame(frame);
    return;
  }

  collectGpuTiming(frame.frameIndex);
  scene.uploadObjectTransforms(gfx_);
  updateFrameUBO(scene, frame.frameIndex);
  VkDescriptorSet frameSet = frames_[frame.frameIndex].frameSet;

  const uint32_t tsBase = frame.frameIndex * kTsPerFrame;
  if (timestampPool_) {
    vkCmdResetQueryPool(frame.cmd, timestampPool_, tsBase, kTsPerFrame);
    writeTimestamp(frame.cmd, tsBase + kTsFrameBegin, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
  }

  // Compute writes the raycast result.
  gfx_.transitionImage(frame.cmd, outImage_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);

  vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline_);
  vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 1,
                          &frameSet, 0, nullptr);

  const uint32_t groupsX = (frame.extent.width + 7u) / 8u;
  const uint32_t groupsY = (frame.extent.height + 7u) / 8u;
  vkCmdDispatch(frame.cmd, groupsX, groupsY, 1);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsAfterCompute, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
  }

  // Blit compute output into the swapchain.
  gfx_.transitionImage(frame.cmd, outImage_.image, VK_IMAGE_LAYOUT_GENERAL,
                       VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                       VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                       VK_ACCESS_2_TRANSFER_READ_BIT);

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                       VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);

  VkImageBlit blit{};
  blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  blit.srcSubresource.layerCount = 1;
  blit.srcOffsets[0] = {0, 0, 0};
  blit.srcOffsets[1] = {static_cast<int32_t>(outImage_.extent.width),
                        static_cast<int32_t>(outImage_.extent.height), 1};
  blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  blit.dstSubresource.layerCount = 1;
  blit.dstOffsets[0] = {0, 0, 0};
  blit.dstOffsets[1] = {static_cast<int32_t>(frame.extent.width),
                        static_cast<int32_t>(frame.extent.height), 1};

  vkCmdBlitImage(frame.cmd, outImage_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                 frame.swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit,
                 VK_FILTER_NEAREST);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsAfterBlit, VK_PIPELINE_STAGE_2_TRANSFER_BIT);
  }

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                       VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                       VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT);

  VkViewport vp{0, 0, static_cast<float>(frame.extent.width),
                static_cast<float>(frame.extent.height), 0, 1};
  VkRect2D scissor{{0, 0}, frame.extent};

  VkRenderingAttachmentInfo uiAttach{};
  uiAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  uiAttach.imageView = frame.swapchainView;
  uiAttach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  uiAttach.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
  uiAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

  VkRenderingInfo uiInfo{};
  uiInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  uiInfo.renderArea.extent = frame.extent;
  uiInfo.layerCount = 1;
  uiInfo.colorAttachmentCount = 1;
  uiInfo.pColorAttachments = &uiAttach;
  vkCmdBeginRendering(frame.cmd, &uiInfo);
  vkCmdSetViewport(frame.cmd, 0, 1, &vp);
  vkCmdSetScissor(frame.cmd, 0, 1, &scissor);
  recordImGui(frame.cmd, scene, displayFps);
  vkCmdEndRendering(frame.cmd);

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                       0);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsFrameEnd, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT);
    timestampPending_[frame.frameIndex] = true;
  }

  gfx_.endFrame(frame);
}

void VoxelRenderer::initImGui() {
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

void VoxelRenderer::shutdownImGui() {
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

void VoxelRenderer::recordImGui(VkCommandBuffer cmd, VoxelScene& scene, float displayFps) {
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGui::Begin("Voxel DDA");
  ImGui::TextWrapped("GPU: %s", gfx_.deviceName().c_str());
  ImGui::Text("Present: %s", gfx_.presentModeName());
  ImGui::Text("Display FPS: %.1f  (%.2f ms)", displayFps,
              displayFps > 1e-3f ? 1000.0f / displayFps : 0.0f);
  ImGui::Text("GPU total:   %.2f ms  (%.0f FPS)", gpuFrameMs_,
              gpuFrameMs_ > 1e-3f ? 1000.0f / gpuFrameMs_ : 0.0f);
  ImGui::Text("  compute:   %.2f ms", gpuComputeMs_);
  ImGui::Text("  blit:      %.2f ms", gpuBlitMs_);
  ImGui::Text("  ui/other:  %.2f ms", gpuUiMs_);
  ImGui::Checkbox("Skip Trace (baseline)", &skipTrace_);
  ImGui::TextDisabled("Baseline keeps dispatch+imageStore but skips DDA.");
  if (gfx_.vsyncEnabled()) {
    ImGui::TextDisabled("VSync present mode; Display FPS is refresh-capped.");
  } else {
    ImGui::TextDisabled("Compare nested on/off and Skip Trace to locate the bottleneck.");
  }
  ImGui::Text("Objects: %u", scene.objectCount());
  ImGui::Text("Occupied coarse: %u / %u", scene.occupiedCount(), scene.voxelCount());
  ImGui::Text("Brick pages: %u  slabs: %u  pool: %.1f KB", scene.allocatedBrickPages(),
              scene.brickSlabCount(), static_cast<float>(scene.brickPoolBytes()) / 1024.0f);
  ImGui::Text("Occupied 8^3 micros: %u   2^3 fines: %u", scene.occupiedMicroCount(),
              scene.occupiedFineCount());
  ImGui::Separator();
  ImGui::TextWrapped("LMB: remove hit object  |  F: place on hit face  |  RMB drag: look");
  ImGui::SliderInt("Brush Material", &scene.brushMaterial(), 1, 2);
  ImGui::Checkbox("Edit/Render Nested 8^3 + 2^3", &scene.nestedMicroVoxels());
  if (scene.nestedMicroVoxels()) {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f fine voxels");
    ImGui::TextDisabled("coarse -> 8^3 brick -> 2x2x2. Spinner is a 2x2x2 checker.");
    ImGui::TextDisabled("r=0 edits one fine cell (1/16 of a coarse voxel).");
  } else {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f coarse voxels");
    ImGui::TextDisabled("Editing whole coarse cells (each owns an 8^3 brick)");
  }
  ImGui::Checkbox("Spinner Enabled", &scene.spinnerEnabled());
  ImGui::SliderFloat("Spin Speed", &scene.spinSpeed(), -3.0f, 3.0f, "%.2f rad/s");
  if (const std::optional<VoxelHit> hit = scene.lastHit()) {
    if (hit->hasFine || hit->hasMicro) {
      ImGui::Text("Hit obj=%d c=(%d,%d,%d) m=(%d,%d,%d) f=(%d,%d,%d) n=(%d,%d,%d) mat=%u",
                  hit->objectIndex, hit->cell.x, hit->cell.y, hit->cell.z, hit->micro.x,
                  hit->micro.y, hit->micro.z, hit->fine.x, hit->fine.y, hit->fine.z, hit->normal.x,
                  hit->normal.y, hit->normal.z, hit->material);
    } else {
      ImGui::Text("Hit obj=%d: (%d, %d, %d)  n=(%d,%d,%d)  mat=%u", hit->objectIndex, hit->cell.x,
                  hit->cell.y, hit->cell.z, hit->normal.x, hit->normal.y, hit->normal.z,
                  hit->material);
    }
  } else {
    ImGui::TextUnformatted("Hit: none");
  }
  ImGui::Separator();

  const char* modes[] = {"Shaded", "Albedo", "Normal", "Steps", "Coord", "AO"};
  ImGui::Combo("Render Mode", &scene.renderMode(), modes, IM_ARRAYSIZE(modes));

  bool rebuild = false;
  rebuild |= ImGui::SliderInt("Grid Size", &scene.gridSize(), 8, 64);
  rebuild |= ImGui::DragFloat("Voxel Size", &scene.voxelSize(), 0.01f, 0.05f, 2.0f);
  ImGui::DragFloat3("Light Dir", &scene.lightDir().x, 0.01f);
  ImGui::SliderFloat("Ambient", &scene.ambient(), 0.0f, 1.0f);
  ImGui::SliderFloat("AO Strength", &scene.aoStrength(), 0.0f, 1.0f);
  ImGui::SliderFloat("AO Power", &scene.aoPower(), 0.05f, 2.0f);
  ImGui::TextDisabled("Neighbor voxel AO (Minecraft-style). Strength 0 disables.");
  ImGui::Checkbox("Skybox", &scene.showSky());
  ImGui::DragFloat("Sky Intensity", &scene.skyIntensity(), 0.01f, 0.0f, 8.0f);
  ImGui::DragFloat("Sky Yaw", &scene.skyYaw(), 0.01f, -3.14159f, 3.14159f);
  ImGui::ColorEdit3("Sky Color (fallback)", &scene.skyColor().x);
  if (!scene.hasSky()) {
    ImGui::TextDisabled("HDR sky missing; using flat sky color.");
  }
  int maxSteps = static_cast<int>(scene.maxSteps());
  if (ImGui::SliderInt("Max Steps", &maxSteps, 16, 512)) {
    scene.maxSteps() = static_cast<uint32_t>(maxSteps);
  }
  if (rebuild) {
    scene.rebuildVoxels(gfx_);
    boundVoxelBuffer_ = VK_NULL_HANDLE;
    boundBrickSlabs_.fill(VK_NULL_HANDLE);
    boundBrickSlabCount_ = 0;
    boundObjectBuffer_ = VK_NULL_HANDLE;
  }
  ImGui::End();

  // Simple screen-center crosshair for aim.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const ImVec2 center(vp->GetCenter().x, vp->GetCenter().y);
  ImDrawList* dl = ImGui::GetForegroundDrawList();
  const float arm = 8.0f;
  const ImU32 col = IM_COL32(255, 255, 255, 220);
  dl->AddLine(ImVec2(center.x - arm, center.y), ImVec2(center.x + arm, center.y), col, 1.5f);
  dl->AddLine(ImVec2(center.x, center.y - arm), ImVec2(center.x, center.y + arm), col, 1.5f);

  ImGui::Render();
  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
}
