#pragma once

#include "gfx/GpuTypes.h"

#include <cstdint>

class GfxDevice;

struct Texture {
  AllocatedImage image{};
  VkSampler sampler = VK_NULL_HANDLE;
};

namespace TextureFactory {
Texture createSolid(GfxDevice& gfx, float r, float g, float b, float a = 1.0f);
Texture createCheckerboard(GfxDevice& gfx, uint32_t size, uint32_t checkSize);
void destroy(GfxDevice& gfx, Texture& texture);
}  // namespace TextureFactory
