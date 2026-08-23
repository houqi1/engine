#pragma once

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <cstdint>

struct AllocatedBuffer {
  VkBuffer buffer = VK_NULL_HANDLE;
  VmaAllocation allocation = VK_NULL_HANDLE;
  VmaAllocationInfo info{};
  VkDeviceSize size = 0;
};

struct AllocatedImage {
  VkImage image = VK_NULL_HANDLE;
  VkImageView view = VK_NULL_HANDLE;
  VmaAllocation allocation = VK_NULL_HANDLE;
  VkExtent3D extent{};
  VkFormat format = VK_FORMAT_UNDEFINED;
  VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
  uint32_t mipLevels = 1;
};

struct Vertex {
  float position[3];
  float normal[3];
  float uv[2];
};

struct FrameUBO {
  float view[16];
  float proj[16];
  float lightViewProj[16];
  float cameraPos[3];
  float pad0;
  float lightDir[3];
  float pad1;
  float lightColor[3];
  float lightIntensity;
  float ambientColor[3];
  float shadowBias;
  float mipLodBias;
  float pad2;
  float pad3;
};

struct MaterialUBO {
  float baseColorFactor[4];
  float metallic;
  float roughness;
  float pad0;
  float pad1;
};

struct PushConstants {
  float model[16];
};
