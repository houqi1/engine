#include "render/VoxelRenderer.h"

#include "gfx/PipelineBuilder.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

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

VoxelRenderer::VoxelRenderer(GfxDevice& gfx) : gfx_(gfx), msaaSamples_(gfx.msaaSamples()) {}

VoxelRenderer::~VoxelRenderer() {
  gfx_.waitIdle();
  shutdownImGui();
  destroyMsaaTargets();

  if (pipeline_) {
    vkDestroyPipeline(gfx_.device(), pipeline_, nullptr);
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

void VoxelRenderer::init() {
  createDescriptors();
  createMsaaTargets();
  createPipelines();

  for (auto& frame : frames_) {
    frame.frameUBO = gfx_.createBuffer(sizeof(VoxelFrameUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                       VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &frameLayout_;
    if (vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &frame.frameSet) != VK_SUCCESS) {
      throw std::runtime_error("Failed to allocate voxel frame descriptor set");
    }

    VkDescriptorBufferInfo bufferInfo{};
    bufferInfo.buffer = frame.frameUBO.buffer;
    bufferInfo.range = sizeof(VoxelFrameUBO);

    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = frame.frameSet;
    write.dstBinding = 0;
    write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    write.descriptorCount = 1;
    write.pBufferInfo = &bufferInfo;
    vkUpdateDescriptorSets(gfx_.device(), 1, &write, 0, nullptr);
  }

  initImGui();
}

void VoxelRenderer::resize() {
  destroyMsaaTargets();
  createMsaaTargets();
}

void VoxelRenderer::createDescriptors() {
  VkDescriptorSetLayoutBinding binding{};
  binding.binding = 0;
  binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  binding.descriptorCount = 1;
  binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

  VkDescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  layoutInfo.bindingCount = 1;
  layoutInfo.pBindings = &binding;
  if (vkCreateDescriptorSetLayout(gfx_.device(), &layoutInfo, nullptr, &frameLayout_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel frame set layout");
  }

  VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, GfxDevice::kFramesInFlight};
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.maxSets = GfxDevice::kFramesInFlight;
  poolInfo.poolSizeCount = 1;
  poolInfo.pPoolSizes = &poolSize;
  if (vkCreateDescriptorPool(gfx_.device(), &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel descriptor pool");
  }
}

void VoxelRenderer::createMsaaTargets() {
  const VkExtent3D extent{gfx_.swapchainExtent().width, gfx_.swapchainExtent().height, 1};
  depthMsaa_ = gfx_.createImage(extent, kDepthFormat, VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                                VK_IMAGE_ASPECT_DEPTH_BIT, true, 1, msaaSamples_);
  if (msaaSamples_ != VK_SAMPLE_COUNT_1_BIT) {
    colorMsaa_ = gfx_.createImage(extent, gfx_.swapchainFormat(),
                                  VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT, VK_IMAGE_ASPECT_COLOR_BIT,
                                  true, 1, msaaSamples_);
  }
}

void VoxelRenderer::destroyMsaaTargets() {
  gfx_.destroyImage(colorMsaa_);
  gfx_.destroyImage(depthMsaa_);
}

void VoxelRenderer::createPipelines() {
  const std::string shaderDir = VE_SHADER_DIR;
  VkShaderModule vert = gfx_.loadShaderModule(shaderDir + "/voxel.vert.spv");
  VkShaderModule frag = gfx_.loadShaderModule(shaderDir + "/voxel.frag.spv");

  VkPushConstantRange pushRange{};
  pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
  pushRange.offset = 0;
  pushRange.size = sizeof(VoxelPushConstants);

  VkPipelineLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layoutInfo.setLayoutCount = 1;
  layoutInfo.pSetLayouts = &frameLayout_;
  layoutInfo.pushConstantRangeCount = 1;
  layoutInfo.pPushConstantRanges = &pushRange;
  if (vkCreatePipelineLayout(gfx_.device(), &layoutInfo, nullptr, &pipelineLayout_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel pipeline layout");
  }

  pipeline_ = PipelineBuilder()
                  .setShaders(vert, frag)
                  .setVertexInput(vertexBinding(), vertexAttributes())
                  .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
                  .setCullMode(VK_CULL_MODE_BACK_BIT, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                  .setMultisampling(msaaSamples_)
                  .setDepthTest(true, true, VK_COMPARE_OP_GREATER_OR_EQUAL)
                  .setColorFormat(gfx_.swapchainFormat())
                  .setDepthFormat(kDepthFormat)
                  .setLayout(pipelineLayout_)
                  .build(gfx_.device());

  vkDestroyShaderModule(gfx_.device(), vert, nullptr);
  vkDestroyShaderModule(gfx_.device(), frag, nullptr);
}

void VoxelRenderer::updateFrameUBO(VoxelScene& scene, uint32_t frameIndex) {
  VoxelFrameUBO ubo{};
  writeMat4(ubo.view, scene.camera().view());
  writeMat4(ubo.proj, scene.camera().proj());
  writeVec3(ubo.cameraPos, scene.camera().position());
  writeVec3(ubo.lightDir, glm::normalize(scene.lightDir()));

  auto& frame = frames_[frameIndex];
  void* mapped = frame.frameUBO.info.pMappedData;
  if (!mapped) {
    throw std::runtime_error("Voxel frame UBO is not host-mapped");
  }
  std::memcpy(mapped, &ubo, sizeof(ubo));
}

void VoxelRenderer::draw(VoxelScene& scene, float displayFps) {
  displayFps_ = displayFps;

  if (gfx_.swapchainWasRecreated()) {
    resize();
    gfx_.clearSwapchainRecreatedFlag();
  }

  FrameContext frame{};
  if (!gfx_.beginFrame(frame)) {
    return;
  }

  updateFrameUBO(scene, frame.frameIndex);
  VkDescriptorSet frameSet = frames_[frame.frameIndex].frameSet;
  const bool useMsaa = msaaSamples_ != VK_SAMPLE_COUNT_1_BIT;

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                       0, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
  if (useMsaa) {
    gfx_.transitionImage(frame.cmd, colorMsaa_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                         VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                         VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, 0,
                         VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                         VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
  }
  gfx_.transitionImage(frame.cmd, depthMsaa_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                       0, VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT,
                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_DEPTH_BIT);

  VkClearValue colorClear{};
  colorClear.color = {{0.08f, 0.09f, 0.12f, 1.0f}};
  VkClearValue depthClear{};
  depthClear.depthStencil = {0.0f, 0};

  VkRenderingAttachmentInfo colorAttach{};
  colorAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  colorAttach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  colorAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  colorAttach.clearValue = colorClear;
  if (useMsaa) {
    colorAttach.imageView = colorMsaa_.view;
    colorAttach.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttach.resolveMode = VK_RESOLVE_MODE_AVERAGE_BIT;
    colorAttach.resolveImageView = frame.swapchainView;
    colorAttach.resolveImageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  } else {
    colorAttach.imageView = frame.swapchainView;
    colorAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  }

  VkRenderingAttachmentInfo depthAttach{};
  depthAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  depthAttach.imageView = depthMsaa_.view;
  depthAttach.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
  depthAttach.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depthAttach.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depthAttach.clearValue = depthClear;

  VkRenderingInfo rendering{};
  rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  rendering.renderArea.extent = frame.extent;
  rendering.layerCount = 1;
  rendering.colorAttachmentCount = 1;
  rendering.pColorAttachments = &colorAttach;
  rendering.pDepthAttachment = &depthAttach;
  vkCmdBeginRendering(frame.cmd, &rendering);

  VkViewport vp{0, 0, (float)frame.extent.width, (float)frame.extent.height, 0, 1};
  VkRect2D scissor{{0, 0}, frame.extent};
  vkCmdSetViewport(frame.cmd, 0, 1, &vp);
  vkCmdSetScissor(frame.cmd, 0, 1, &scissor);

  vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
  vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout_, 0, 1,
                          &frameSet, 0, nullptr);

  Mesh& cube = scene.cubeMesh();
  VkDeviceSize offset = 0;
  vkCmdBindVertexBuffers(frame.cmd, 0, 1, &cube.vertexBuffer.buffer, &offset);
  vkCmdBindIndexBuffer(frame.cmd, cube.indexBuffer.buffer, 0, VK_INDEX_TYPE_UINT32);

  const float scale = scene.voxelSize() * 0.98f;
  for (const VoxelInstance& voxel : scene.voxels()) {
    VoxelPushConstants pc{};
    const glm::mat4 model = glm::translate(glm::mat4(1.0f), voxel.position) *
                            glm::scale(glm::mat4(1.0f), glm::vec3(scale));
    writeMat4(pc.model, model);
    pc.color[0] = voxel.color.r;
    pc.color[1] = voxel.color.g;
    pc.color[2] = voxel.color.b;
    pc.color[3] = 1.0f;
    vkCmdPushConstants(frame.cmd, pipelineLayout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), &pc);
    vkCmdDrawIndexed(frame.cmd, cube.indexCount, 1, 0, 0, 0);
  }

  vkCmdEndRendering(frame.cmd);

  // ImGui is 1x-only: load resolved swapchain and overlay UI.
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

  ImGui::Begin("Voxel Demo");
  ImGui::TextWrapped("GPU: %s", gfx_.deviceName().c_str());
  ImGui::Text("Display FPS: %.1f", displayFps);
  ImGui::Text("MSAA: %ux", static_cast<uint32_t>(msaaSamples_));
  ImGui::Separator();
  ImGui::Text("Placeholder voxel shell (cube-per-voxel).");
  ImGui::Text("Solid voxels: %zu", scene.voxels().size());
  bool rebuild = false;
  rebuild |= ImGui::SliderInt("Grid Size", &scene.gridSize(), 8, 48);
  rebuild |= ImGui::DragFloat("Voxel Size", &scene.voxelSize(), 0.01f, 0.1f, 1.0f);
  ImGui::DragFloat3("Light Dir", &scene.lightDir().x, 0.01f);
  if (rebuild) {
    scene.rebuildVoxels();
  }
  ImGui::End();

  ImGui::Render();
  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
}
