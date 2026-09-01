#pragma once

#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"

#include <glm/glm.hpp>

#include <array>
#include <cstdint>
#include <string>
#include <vector>

class GfxDevice;

struct MeshVoxelizeConfig {
  int gridN = 48;
  int padding = 1;
  uint32_t fallbackMaterial = 1;
  bool conservative = true;
  bool sampleColor = true;
};

struct MeshVoxelizeResult {
  bool ok = false;
  int n = 0;
  float voxelSize = 0.35f;
  glm::vec3 bmin{0.0f};
  std::vector<uint32_t> material;
  std::array<glm::vec3, 256> palette{};
  uint32_t paletteUsed = 0;
  uint32_t occupied = 0;
  std::string error;
  std::string warning;
};

class MeshVoxelizerGpu {
public:
  void ensure(GfxDevice& gfx);
  void destroy(GfxDevice& gfx);
  MeshVoxelizeResult voxelizeObjSurface(GfxDevice& gfx, const std::string& path,
                                        const MeshVoxelizeConfig& cfg);

private:
  VkDescriptorSetLayout setLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline pipeline_ = VK_NULL_HANDLE;
  VkDescriptorPool pool_ = VK_NULL_HANDLE;
  bool ready_ = false;
};

MeshVoxelizeResult voxelizeObjSurface(GfxDevice& gfx, MeshVoxelizerGpu& gpu, const std::string& path,
                                      const MeshVoxelizeConfig& cfg);
