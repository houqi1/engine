#include "gfx/PipelineBuilder.h"

#include <algorithm>
#include <stdexcept>

PipelineBuilder& PipelineBuilder::setShaders(VkShaderModule vert, VkShaderModule frag) {
  stages_.clear();

  VkPipelineShaderStageCreateInfo vs{};
  vs.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  vs.stage = VK_SHADER_STAGE_VERTEX_BIT;
  vs.module = vert;
  vs.pName = "main";
  stages_.push_back(vs);

  VkPipelineShaderStageCreateInfo fs{};
  fs.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  fs.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
  fs.module = frag;
  fs.pName = "main";
  stages_.push_back(fs);
  return *this;
}

PipelineBuilder& PipelineBuilder::setVertexInput(
    const VkVertexInputBindingDescription& binding,
    const std::vector<VkVertexInputAttributeDescription>& attrs) {
  bindings_ = {binding};
  attributes_ = attrs;
  hasVertexInput_ = true;
  return *this;
}

PipelineBuilder& PipelineBuilder::setVertexInput(
    const std::vector<VkVertexInputBindingDescription>& bindings,
    const std::vector<VkVertexInputAttributeDescription>& attrs) {
  bindings_ = bindings;
  attributes_ = attrs;
  hasVertexInput_ = true;
  return *this;
}

PipelineBuilder& PipelineBuilder::setTopology(VkPrimitiveTopology topology) {
  topology_ = topology;
  return *this;
}

PipelineBuilder& PipelineBuilder::setPolygonMode(VkPolygonMode mode) {
  polygonMode_ = mode;
  return *this;
}

PipelineBuilder& PipelineBuilder::setCullMode(VkCullModeFlags cull, VkFrontFace front) {
  cullMode_ = cull;
  frontFace_ = front;
  return *this;
}

PipelineBuilder& PipelineBuilder::setMultisampling(VkSampleCountFlagBits samples) {
  samples_ = samples;
  return *this;
}

PipelineBuilder& PipelineBuilder::setDepthTest(bool enable, bool write, VkCompareOp compare) {
  depthTest_ = enable;
  depthWrite_ = write;
  depthCompare_ = compare;
  return *this;
}

PipelineBuilder& PipelineBuilder::setDepthClamp(bool enable) {
  depthClamp_ = enable;
  return *this;
}

PipelineBuilder& PipelineBuilder::setColorBlend(bool enableAlphaBlend) {
  blend_ = enableAlphaBlend;
  return *this;
}

PipelineBuilder& PipelineBuilder::setColorFormat(VkFormat format) {
  colorFormat_ = format;
  colorFormats_ = {format};
  return *this;
}

PipelineBuilder& PipelineBuilder::setColorFormats(const std::vector<VkFormat>& formats) {
  colorFormats_ = formats;
  colorFormat_ = formats.empty() ? VK_FORMAT_UNDEFINED : formats[0];
  return *this;
}

PipelineBuilder& PipelineBuilder::setMinMaxBlend(uint32_t attachmentCount) {
  minMaxBlendCount_ = std::max(2u, std::min(attachmentCount, 3u));
  blend_ = false;
  return *this;
}

PipelineBuilder& PipelineBuilder::setDepthFormat(VkFormat format) {
  depthFormat_ = format;
  return *this;
}

PipelineBuilder& PipelineBuilder::setLayout(VkPipelineLayout layout) {
  layout_ = layout;
  return *this;
}

PipelineBuilder& PipelineBuilder::disableColorWrite() {
  colorWrite_ = false;
  return *this;
}

VkPipeline PipelineBuilder::build(VkDevice device) const {
  VkPipelineVertexInputStateCreateInfo vertexInput{};
  vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
  if (hasVertexInput_) {
    vertexInput.vertexBindingDescriptionCount = static_cast<uint32_t>(bindings_.size());
    vertexInput.pVertexBindingDescriptions = bindings_.data();
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributes_.size());
    vertexInput.pVertexAttributeDescriptions = attributes_.data();
  }

  VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
  inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
  inputAssembly.topology = topology_;

  VkPipelineViewportStateCreateInfo viewportState{};
  viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
  viewportState.viewportCount = 1;
  viewportState.scissorCount = 1;

  VkPipelineRasterizationStateCreateInfo raster{};
  raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
  raster.polygonMode = polygonMode_;
  raster.cullMode = cullMode_;
  raster.frontFace = frontFace_;
  raster.lineWidth = 1.0f;
  raster.depthClampEnable = depthClamp_ ? VK_TRUE : VK_FALSE;
  raster.depthBiasEnable = VK_FALSE;

  VkPipelineMultisampleStateCreateInfo ms{};
  ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
  ms.rasterizationSamples = samples_;

  VkPipelineDepthStencilStateCreateInfo depth{};
  depth.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
  depth.depthTestEnable = depthTest_ ? VK_TRUE : VK_FALSE;
  depth.depthWriteEnable = depthWrite_ ? VK_TRUE : VK_FALSE;
  depth.depthCompareOp = depthCompare_;
  depth.maxDepthBounds = 1.0f;

  std::vector<VkPipelineColorBlendAttachmentState> colorAttaches;
  const uint32_t colorCount = !colorFormats_.empty()
                                  ? static_cast<uint32_t>(colorFormats_.size())
                                  : (colorFormat_ != VK_FORMAT_UNDEFINED ? 1u : 0u);
  colorAttaches.resize(std::max(colorCount, minMaxBlendCount_));
  for (uint32_t i = 0; i < colorAttaches.size(); ++i) {
    VkPipelineColorBlendAttachmentState& att = colorAttaches[i];
    att.colorWriteMask = colorWrite_ ? (VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT)
                                     : 0;
    if (minMaxBlendCount_ > 0 && i < minMaxBlendCount_) {
      att.blendEnable = VK_TRUE;
      att.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
      att.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
      att.colorBlendOp = (i == 1u) ? VK_BLEND_OP_MAX : VK_BLEND_OP_MIN;
      att.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
      att.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
      att.alphaBlendOp = att.colorBlendOp;
    } else if (blend_) {
      att.blendEnable = VK_TRUE;
      att.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
      att.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
      att.colorBlendOp = VK_BLEND_OP_ADD;
      att.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
      att.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
      att.alphaBlendOp = VK_BLEND_OP_ADD;
    }
  }

  VkPipelineColorBlendStateCreateInfo blend{};
  blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
  blend.attachmentCount = static_cast<uint32_t>(colorAttaches.size());
  blend.pAttachments = colorAttaches.empty() ? nullptr : colorAttaches.data();

  std::vector<VkDynamicState> dynamics = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo dynamicState{};
  dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
  dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamics.size());
  dynamicState.pDynamicStates = dynamics.data();

  VkPipelineRenderingCreateInfo rendering{};
  rendering.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  std::vector<VkFormat> formats = colorFormats_;
  if (formats.empty() && colorFormat_ != VK_FORMAT_UNDEFINED) {
    formats = {colorFormat_};
  }
  if (!formats.empty()) {
    rendering.colorAttachmentCount = static_cast<uint32_t>(formats.size());
    rendering.pColorAttachmentFormats = formats.data();
  }
  rendering.depthAttachmentFormat = depthFormat_;

  VkGraphicsPipelineCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
  info.pNext = &rendering;
  info.stageCount = static_cast<uint32_t>(stages_.size());
  info.pStages = stages_.data();
  info.pVertexInputState = &vertexInput;
  info.pInputAssemblyState = &inputAssembly;
  info.pViewportState = &viewportState;
  info.pRasterizationState = &raster;
  info.pMultisampleState = &ms;
  info.pDepthStencilState = &depth;
  info.pColorBlendState = &blend;
  info.pDynamicState = &dynamicState;
  info.layout = layout_;

  VkPipeline pipeline = VK_NULL_HANDLE;
  if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &pipeline) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create graphics pipeline");
  }
  return pipeline;
}
