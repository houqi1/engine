#pragma once

#include <vulkan/vulkan.h>

#include <vector>

class PipelineBuilder {
public:
  PipelineBuilder& setShaders(VkShaderModule vert, VkShaderModule frag);
  PipelineBuilder& setVertexInput(const VkVertexInputBindingDescription& binding,
                                  const std::vector<VkVertexInputAttributeDescription>& attrs);
  PipelineBuilder& setTopology(VkPrimitiveTopology topology);
  PipelineBuilder& setPolygonMode(VkPolygonMode mode);
  PipelineBuilder& setCullMode(VkCullModeFlags cull, VkFrontFace front);
  PipelineBuilder& setMultisampling(VkSampleCountFlagBits samples);
  PipelineBuilder& setDepthTest(bool enable, bool write, VkCompareOp compare);
  PipelineBuilder& setColorBlend(bool enableAlphaBlend);
  PipelineBuilder& setColorFormat(VkFormat format);
  PipelineBuilder& setDepthFormat(VkFormat format);
  PipelineBuilder& setLayout(VkPipelineLayout layout);
  PipelineBuilder& disableColorWrite();

  VkPipeline build(VkDevice device) const;

private:
  std::vector<VkPipelineShaderStageCreateInfo> stages_;
  VkVertexInputBindingDescription binding_{};
  std::vector<VkVertexInputAttributeDescription> attributes_;
  bool hasVertexInput_ = false;

  VkPrimitiveTopology topology_ = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
  VkPolygonMode polygonMode_ = VK_POLYGON_MODE_FILL;
  VkCullModeFlags cullMode_ = VK_CULL_MODE_BACK_BIT;
  VkFrontFace frontFace_ = VK_FRONT_FACE_COUNTER_CLOCKWISE;
  VkSampleCountFlagBits samples_ = VK_SAMPLE_COUNT_1_BIT;

  bool depthTest_ = true;
  bool depthWrite_ = true;
  VkCompareOp depthCompare_ = VK_COMPARE_OP_GREATER_OR_EQUAL;  // reverse-Z

  bool blend_ = false;
  bool colorWrite_ = true;

  VkFormat colorFormat_ = VK_FORMAT_UNDEFINED;
  VkFormat depthFormat_ = VK_FORMAT_UNDEFINED;
  VkPipelineLayout layout_ = VK_NULL_HANDLE;
};
