#pragma once

#include "gfx/GpuTypes.h"
#include "gfx/ShIrradiance.h"
#include "gfx/Texture.h"

#include <cstdint>
#include <string>

class GfxDevice;

struct IblMaps {
  AllocatedImage prefiltered{};  // RGBA32F cubemap with mips
  AllocatedImage brdfLut{};      // RGBA32F 2D (RG used)
  VkSampler cubeSampler = VK_NULL_HANDLE;
  VkSampler lutSampler = VK_NULL_HANDLE;
  uint32_t mipCount = 1;
  bool ready = false;
};

namespace IblBake {

constexpr uint32_t kEnvSize = 1024;
constexpr uint32_t kPrefilterSize = 512;
constexpr uint32_t kBrdfSize = 512;
constexpr uint32_t kCacheVersion = 8;  // GPU bake + explicit face UV→dir mapping

// GPU bake from equirect (equirect→cube, prefilter, BRDF LUT), then optional cache write via caller.
IblMaps build(GfxDevice& gfx, const EquirectHdrData& equirect);

// Load cache if valid; otherwise GPU-bake, save cache, and fill outSh.
IblMaps buildOrLoad(GfxDevice& gfx, const EquirectHdrData& equirect, const std::string& sourceHdrPath,
                    const std::string& cacheDir, Sh9* outSh);

void destroy(GfxDevice& gfx, IblMaps& maps);

}  // namespace IblBake
