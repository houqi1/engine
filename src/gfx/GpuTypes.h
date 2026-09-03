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
  uint32_t layerCount = 1;
  VkSampleCountFlagBits samples = VK_SAMPLE_COUNT_1_BIT;
  VkImageType imageType = VK_IMAGE_TYPE_2D;
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
  float skyYaw;
  float skyIntensity;
  float ambientScale;      // scales SH irradiance (diffuse IBL)
  float iblMaxLod;          // prefiltered cubemap max mip
  float specularIblScale;   // specular IBL intensity
  float enablePrefiltered;  // 1 = include/show prefiltered cubemap
  float enableBrdfLut;      // 1 = include/show BRDF LUT term
  float ambientSH[9][4];    // L2 irradiance coeffs (std140 vec4 each)
};

struct MaterialUBO {
  float baseColorFactor[4];
  float metallic;
  float roughness;
  float shOnly;  // 1 = raw sky SH visualization
  float pad1;
};

struct PushConstants {
  float model[16];
};

// Per-instance grass blade (vertex buffer, VK_VERTEX_INPUT_RATE_INSTANCE).
struct GrassInstance {
  float position[3];
  float yaw;
  float color[3];
  float scale;
};

struct GrassPushConstants {
  float time;
  float windStrength;
  float windFrequency;
  float pad;
};

struct SkyPushConstants {
  float intensity;
  float yaw;
  float pad0;
  float pad1;
};

struct FrameStats {
  float displayFps = 0.0f;     // includes VSync wait
  float displayFrameMs = 0.0f; // 1000 / displayFps
  float cpuRecordMs = 0.0f;    // CPU time to record + submit prep (no present wait)
  float gpuFrameMs = 0.0f;     // GPU timestamp delta for the rendered frame
};
