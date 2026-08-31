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
  info.queryCount = GfxDevice::kFramesInFlight * 2;
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
  constexpr float alpha = 0.1f;
  gpuFrameMs_ = gpuFrameMs_ * (1.0f - alpha) + gpuMs * alpha;
  timestampPending_[frameIndex] = false;
}

void VoxelRenderer::resize() {
  destroyOutputImage();
  createOutputImage();
  // Force descriptor refresh so storage-image views stay valid.
  boundVoxelBuffer_ = VK_NULL_HANDLE;
  boundMicroBuffer_ = VK_NULL_HANDLE;
}

void VoxelRenderer::createDescriptors() {
  VkDescriptorSetLayoutBinding bindings[4]{};
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
  bindings[3].descriptorCount = 1;
  bindings[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  VkDescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  layoutInfo.bindingCount = 4;
  layoutInfo.pBindings = bindings;
  if (vkCreateDescriptorSetLayout(gfx_.device(), &layoutInfo, nullptr, &frameLayout_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel DDA set layout");
  }

  VkDescriptorPoolSize poolSizes[] = {
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, GfxDevice::kFramesInFlight},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, GfxDevice::kFramesInFlight * 2},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, GfxDevice::kFramesInFlight},
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
      scene.microBuffer().buffer == VK_NULL_HANDLE) {
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

    VkDescriptorBufferInfo microInfo{};
    microInfo.buffer = scene.microBuffer().buffer;
    microInfo.range = scene.microBuffer().size;

    VkWriteDescriptorSet writes[4]{};
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
    writes[3].descriptorCount = 1;
    writes[3].pBufferInfo = &microInfo;

    vkUpdateDescriptorSets(gfx_.device(), 4, writes, 0, nullptr);
  }

  boundVoxelBuffer_ = scene.voxelBuffer().buffer;
  boundMicroBuffer_ = scene.microBuffer().buffer;
}

void VoxelRenderer::updateFrameUBO(VoxelScene& scene, uint32_t frameIndex) {
  const glm::mat4 view = scene.camera().view();
  const glm::mat4 proj = scene.camera().proj();

  VoxelDdaUBO ubo{};
  writeMat4(ubo.invView, glm::inverse(view));
  writeMat4(ubo.invProj, glm::inverse(proj));
  writeVec3(ubo.cameraPos, scene.camera().position());
  ubo.voxelSize = scene.voxelSize();
  writeVec3(ubo.lightDir, glm::normalize(scene.lightDir()));
  ubo.ambient = scene.ambient();
  writeVec3(ubo.gridOrigin, scene.gridOrigin());
  ubo.projX = proj[0][0];
  const glm::uvec3 dims = scene.gridDims();
  ubo.gridSize[0] = dims.x;
  ubo.gridSize[1] = dims.y;
  ubo.gridSize[2] = dims.z;
  ubo.maxSteps = scene.maxSteps();
  writeVec3(ubo.skyColor, scene.skyColor());
  ubo.renderMode = static_cast<uint32_t>(std::max(0, scene.renderMode()));
  ubo.projY = proj[1][1];
  ubo.nestedMicro = scene.nestedMicroVoxels() ? 1u : 0u;

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
  if (scene.voxelBuffer().buffer != boundVoxelBuffer_ ||
      scene.microBuffer().buffer != boundMicroBuffer_ || outImage_.view == VK_NULL_HANDLE) {
    updateDescriptors(scene);
  }

  FrameContext frame{};
  if (!gfx_.beginFrame(frame)) {
    return;
  }

  if (outImage_.image == VK_NULL_HANDLE || scene.voxelBuffer().buffer == VK_NULL_HANDLE) {
    gfx_.endFrame(frame);
    return;
  }

  collectGpuTiming(frame.frameIndex);
  updateFrameUBO(scene, frame.frameIndex);
  VkDescriptorSet frameSet = frames_[frame.frameIndex].frameSet;

  const uint32_t tsBegin = frame.frameIndex * 2;
  if (timestampPool_) {
    vkCmdResetQueryPool(frame.cmd, timestampPool_, tsBegin, 2);
    writeTimestamp(frame.cmd, tsBegin, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
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
    writeTimestamp(frame.cmd, tsBegin + 1, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT);
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
  ImGui::Text("GPU work:    %.2f ms", gpuFrameMs_);
  if (gpuFrameMs_ > 1e-3f) {
    ImGui::Text("GPU work FPS: %.0f", 1000.0f / gpuFrameMs_);
  } else {
    ImGui::Text("GPU work FPS: n/a");
  }
  if (gfx_.vsyncEnabled()) {
    ImGui::TextDisabled("VSync present mode; Display FPS is refresh-capped.");
  } else {
    ImGui::TextDisabled("If Display FPS stays ~60 on macOS, compositor may still sync;");
    ImGui::TextDisabled("trust GPU work FPS for uncapped cost.");
  }
  ImGui::Text("Occupied coarse: %u / %u", scene.occupiedCount(), scene.voxelCount());
  ImGui::Text("Occupied micro: %u", scene.occupiedMicroCount());
  ImGui::Separator();
  ImGui::TextWrapped("LMB: remove  |  F: place on hit face  |  RMB drag: look");
  ImGui::SliderInt("Brush Material", &scene.brushMaterial(), 1, 2);
  ImGui::Checkbox("Edit/Render Nested 8^3 Micro", &scene.nestedMicroVoxels());
  if (scene.nestedMicroVoxels()) {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f micro-voxels");
    ImGui::TextDisabled("Editing individual micro cells inside coarse bricks");
  } else {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f coarse voxels");
    ImGui::TextDisabled("Editing whole coarse cells (each owns an 8^3 brick)");
  }
  if (const std::optional<VoxelHit> hit = scene.lastHit()) {
    if (hit->hasMicro) {
      ImGui::Text("Hit coarse=(%d,%d,%d) micro=(%d,%d,%d) n=(%d,%d,%d) mat=%u", hit->cell.x,
                  hit->cell.y, hit->cell.z, hit->micro.x, hit->micro.y, hit->micro.z, hit->normal.x,
                  hit->normal.y, hit->normal.z, hit->material);
    } else {
      ImGui::Text("Hit: (%d, %d, %d)  n=(%d,%d,%d)  mat=%u", hit->cell.x, hit->cell.y, hit->cell.z,
                  hit->normal.x, hit->normal.y, hit->normal.z, hit->material);
    }
  } else {
    ImGui::TextUnformatted("Hit: none");
  }
  ImGui::Separator();

  const char* modes[] = {"Shaded", "Albedo", "Normal", "Steps", "Coord"};
  ImGui::Combo("Render Mode", &scene.renderMode(), modes, IM_ARRAYSIZE(modes));

  bool rebuild = false;
  rebuild |= ImGui::SliderInt("Grid Size", &scene.gridSize(), 8, 64);
  rebuild |= ImGui::DragFloat("Voxel Size", &scene.voxelSize(), 0.01f, 0.05f, 2.0f);
  ImGui::DragFloat3("Light Dir", &scene.lightDir().x, 0.01f);
  ImGui::SliderFloat("Ambient", &scene.ambient(), 0.0f, 1.0f);
  ImGui::ColorEdit3("Sky Color", &scene.skyColor().x);
  int maxSteps = static_cast<int>(scene.maxSteps());
  if (ImGui::SliderInt("Max Steps", &maxSteps, 16, 512)) {
    scene.maxSteps() = static_cast<uint32_t>(maxSteps);
  }
  if (rebuild) {
    scene.rebuildVoxels(gfx_);
    boundVoxelBuffer_ = VK_NULL_HANDLE;
    boundMicroBuffer_ = VK_NULL_HANDLE;
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
