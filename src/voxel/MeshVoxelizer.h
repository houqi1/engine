#pragma once

#include "gfx/GpuTypes.h"
#include "gfx/Texture.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <string>
#include <vector>

class GfxDevice;

struct MeshVoxelizeConfig {
  int gridN = 64;
  int padding = 1;
  int fineSubdiv = 16;  // must match VoxelScene::kFinePerCoarse (8^3 micro * 2^3 fine)
  uint32_t fallbackMaterial = 1;
  bool conservative = true;
  bool sampleColor = true;
};

struct MeshVoxelizeResult {
  bool ok = false;
  int n = 0;
  int subdiv = 16;
  int fineN = 0;
  float voxelSize = 0.35f;
  glm::vec3 bmin{0.0f};
  std::vector<uint32_t> material;  // coarse occupancy (1 = solid)
  std::vector<uint32_t> fineBits;
  std::vector<uint32_t> fineId;    // sorted occupied fine indices with a color sample
  std::vector<uint32_t> fineRgb;   // 0x00RRGGBB, parallel to fineId
  std::vector<uint32_t> coarseRgb; // 0x00RRGGBB per coarse cell, fallback if a fine has no sample
  uint32_t occupied = 0;
  uint32_t occupiedFine = 0;
  uint32_t colorSamples = 0;
  uint32_t colorDropped = 0;
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
