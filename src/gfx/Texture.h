#pragma once

#include "gfx/GpuTypes.h"

#include <cstdint>
#include <string>
#include <vector>

class GfxDevice;

struct Texture {
  AllocatedImage image{};
  VkSampler sampler = VK_NULL_HANDLE;
};

struct EquirectHdrData {
  int width = 0;
  int height = 0;
  std::vector<float> rgba;  // width * height * 4, linear
};

namespace TextureFactory {
Texture createSolid(GfxDevice& gfx, float r, float g, float b, float a = 1.0f);
Texture createCheckerboard(GfxDevice& gfx, uint32_t size, uint32_t checkSize);
EquirectHdrData loadHdrEquirectData(const std::string& path);
Texture createEquirectTexture(GfxDevice& gfx, const EquirectHdrData& data);
// Loads Radiance (.hdr) equirectangular panorama as RGBA32F.
Texture loadHdrEquirect(GfxDevice& gfx, const std::string& path);
void destroy(GfxDevice& gfx, Texture& texture);
}  // namespace TextureFactory
