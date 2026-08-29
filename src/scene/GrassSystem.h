#pragma once

#include "gfx/GpuTypes.h"
#include "gfx/Mesh.h"

#include <glm/glm.hpp>

#include <cstdint>
#include <vector>

class GfxDevice;

struct GrassParams {
  bool enabled = true;
  int gridResolution = 220;      // ~45k blades; safer default for Intel iGPU
  float areaHalfExtent = 11.0f;  // covers most of the demo ground plane
  float windStrength = 0.35f;
  float windFrequency = 1.6f;
  float heightScale = 0.55f;
  float heightVariance = 0.35f;
  glm::vec3 baseColor{0.18f, 0.42f, 0.12f};
  glm::vec3 tipColor{0.45f, 0.72f, 0.22f};

  // Chunked frustum / distance culling (Phase 1.5).
  bool enableCulling = true;
  int chunkSize = 16;                 // blades along one chunk edge
  float fullDensityDistance = 14.0f;  // 100% instances
  float halfDensityDistance = 26.0f;  // 50% until this, then 25%
  float maxDrawDistance = 42.0f;      // skip beyond
};

struct GrassDrawBatch {
  uint32_t firstInstance = 0;
  uint32_t instanceCount = 0;
};

struct GrassCullStats {
  uint32_t chunkCount = 0;
  uint32_t colorChunks = 0;
  uint32_t shadowChunks = 0;
  uint32_t colorInstances = 0;
  uint32_t shadowInstances = 0;
};

class GrassSystem {
public:
  void init(GfxDevice& gfx);
  void cleanup(GfxDevice& gfx);
  void rebuildIfNeeded(GfxDevice& gfx);
  void markDirty() { dirty_ = true; }

  // Build draw batches for the color pass (camera frustum) and shadow pass (light frustum).
  void cull(const glm::vec3& cameraPos, const glm::mat4& cameraViewProj,
            const glm::mat4& lightViewProj);

  GrassParams& params() { return params_; }
  const GrassParams& params() const { return params_; }

  const Mesh& bladeMesh() const { return blade_; }
  VkBuffer instanceBuffer() const { return instanceBuffer_.buffer; }
  uint32_t instanceCount() const { return instanceCount_; }
  bool ready() const { return instanceCount_ > 0 && instanceBuffer_.buffer != VK_NULL_HANDLE; }

  const std::vector<GrassDrawBatch>& colorBatches() const { return colorBatches_; }
  const std::vector<GrassDrawBatch>& shadowBatches() const { return shadowBatches_; }
  const GrassCullStats& cullStats() const { return cullStats_; }

private:
  struct Chunk {
    glm::vec3 aabbMin{0.0f};
    glm::vec3 aabbMax{0.0f};
    glm::vec3 center{0.0f};
    uint32_t firstInstance = 0;
    uint32_t countQuarter = 0;  // ~25% (LOD far)
    uint32_t countHalf = 0;     // ~50% (LOD mid)
    uint32_t countFull = 0;     // 100% (LOD near)
  };

  void rebuild(GfxDevice& gfx);

  GrassParams params_{};
  GrassParams builtParams_{};
  Mesh blade_{};
  AllocatedBuffer instanceBuffer_{};
  uint32_t instanceCount_ = 0;
  bool dirty_ = true;
  bool initialized_ = false;

  std::vector<Chunk> chunks_;
  std::vector<GrassDrawBatch> colorBatches_;
  std::vector<GrassDrawBatch> shadowBatches_;
  GrassCullStats cullStats_{};
};
