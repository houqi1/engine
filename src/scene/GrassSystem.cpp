#include "scene/GrassSystem.h"

#include "gfx/GfxDevice.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace {

float hash11(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return static_cast<float>(x) / static_cast<float>(0xffffffffu);
}

struct Plane {
  glm::vec3 n{0.0f, 1.0f, 0.0f};
  float d = 0.0f;  // n·p + d >= 0 => inside
};

Plane normalizePlane(glm::vec4 p) {
  const float len = glm::length(glm::vec3(p));
  Plane out{};
  if (len > 1e-6f) {
    out.n = glm::vec3(p) / len;
    out.d = p.w / len;
  }
  return out;
}

glm::vec4 matrixRow(const glm::mat4& m, int row) {
  return glm::vec4(m[0][row], m[1][row], m[2][row], m[3][row]);
}

// Perspective (reverse-Z infinite): 4 side planes are enough; behind-camera is outside.
// Ortho light: include near/far as well.
int extractFrustumPlanes(const glm::mat4& viewProj, Plane* planes, bool includeNearFar) {
  const glm::vec4 r0 = matrixRow(viewProj, 0);
  const glm::vec4 r1 = matrixRow(viewProj, 1);
  const glm::vec4 r2 = matrixRow(viewProj, 2);
  const glm::vec4 r3 = matrixRow(viewProj, 3);

  planes[0] = normalizePlane(r3 + r0);  // left
  planes[1] = normalizePlane(r3 - r0);  // right
  planes[2] = normalizePlane(r3 + r1);  // bottom
  planes[3] = normalizePlane(r3 - r1);  // top
  if (!includeNearFar) {
    return 4;
  }
  planes[4] = normalizePlane(r3 + r2);  // near
  planes[5] = normalizePlane(r3 - r2);  // far
  return 6;
}

bool aabbOutsidePlane(const glm::vec3& bmin, const glm::vec3& bmax, const Plane& plane) {
  // Positive vertex along plane normal.
  glm::vec3 p;
  p.x = plane.n.x >= 0.0f ? bmax.x : bmin.x;
  p.y = plane.n.y >= 0.0f ? bmax.y : bmin.y;
  p.z = plane.n.z >= 0.0f ? bmax.z : bmin.z;
  return glm::dot(plane.n, p) + plane.d < 0.0f;
}

bool aabbInFrustum(const glm::vec3& bmin, const glm::vec3& bmax, const Plane* planes, int count) {
  for (int i = 0; i < count; ++i) {
    if (aabbOutsidePlane(bmin, bmax, planes[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace

void GrassSystem::init(GfxDevice& gfx) {
  blade_ = MeshFactory::createGrassBlade(gfx, 5);
  initialized_ = true;
  dirty_ = true;
  rebuildIfNeeded(gfx);
}

void GrassSystem::cleanup(GfxDevice& gfx) {
  if (instanceBuffer_.buffer) {
    gfx.destroyBuffer(instanceBuffer_);
  }
  MeshFactory::destroy(gfx, blade_);
  instanceCount_ = 0;
  chunks_.clear();
  colorBatches_.clear();
  shadowBatches_.clear();
  cullStats_ = {};
  initialized_ = false;
  dirty_ = true;
}

void GrassSystem::rebuildIfNeeded(GfxDevice& gfx) {
  if (!initialized_) {
    return;
  }
  if (!dirty_ && params_.gridResolution == builtParams_.gridResolution &&
      params_.areaHalfExtent == builtParams_.areaHalfExtent &&
      params_.heightScale == builtParams_.heightScale &&
      params_.heightVariance == builtParams_.heightVariance &&
      params_.baseColor == builtParams_.baseColor && params_.tipColor == builtParams_.tipColor &&
      params_.chunkSize == builtParams_.chunkSize) {
    return;
  }
  rebuild(gfx);
}

void GrassSystem::rebuild(GfxDevice& gfx) {
  const int res = std::clamp(params_.gridResolution, 1, 800);
  params_.gridResolution = res;
  const int chunkSize = std::clamp(params_.chunkSize, 4, 64);
  params_.chunkSize = chunkSize;

  std::vector<GrassInstance> instances;
  instances.reserve(static_cast<size_t>(res) * static_cast<size_t>(res));
  chunks_.clear();

  const float extent = std::max(1.0f, params_.areaHalfExtent);
  const float step = (extent * 2.0f) / static_cast<float>(res);
  const float bladeMaxY =
      params_.heightScale * (1.0f + params_.heightVariance * 0.5f) * 1.15f + 0.05f;
  // Wind / jitter padding so frustum tests stay conservative.
  const float xzPad = step * 0.6f + params_.windStrength * params_.heightScale * 1.25f + 0.05f;

  const int chunksPerSide = (res + chunkSize - 1) / chunkSize;
  chunks_.reserve(static_cast<size_t>(chunksPerSide) * static_cast<size_t>(chunksPerSide));

  std::vector<GrassInstance> tier0;
  std::vector<GrassInstance> tier1;
  std::vector<GrassInstance> tier2;
  tier0.reserve(static_cast<size_t>(chunkSize * chunkSize / 4 + 8));
  tier1.reserve(static_cast<size_t>(chunkSize * chunkSize / 4 + 8));
  tier2.reserve(static_cast<size_t>(chunkSize * chunkSize / 2 + 8));

  for (int cz = 0; cz < chunksPerSide; ++cz) {
    for (int cx = 0; cx < chunksPerSide; ++cx) {
      tier0.clear();
      tier1.clear();
      tier2.clear();

      float minX = 1e9f;
      float maxX = -1e9f;
      float minZ = 1e9f;
      float maxZ = -1e9f;

      const int x0 = cx * chunkSize;
      const int z0 = cz * chunkSize;
      const int x1 = std::min(x0 + chunkSize, res);
      const int z1 = std::min(z0 + chunkSize, res);

      for (int z = z0; z < z1; ++z) {
        for (int x = x0; x < x1; ++x) {
          const uint32_t seed =
              static_cast<uint32_t>(x * 73856093u) ^ static_cast<uint32_t>(z * 19349663u);
          const float jx = (hash11(seed) - 0.5f) * step;
          const float jz = (hash11(seed * 3u + 1u) - 0.5f) * step;

          const float px = -extent + (static_cast<float>(x) + 0.5f) * step + jx;
          const float pz = -extent + (static_cast<float>(z) + 0.5f) * step + jz;

          // Keep a small clearing around demo props near origin.
          if ((px * px + pz * pz) < 2.5f * 2.5f) {
            continue;
          }

          const float t = hash11(seed * 7u + 9u);
          const float scale = params_.heightScale *
                              (1.0f - params_.heightVariance * 0.5f + params_.heightVariance * t);
          const float yaw = hash11(seed * 11u + 5u) * 6.2831853f;

          glm::vec3 color = glm::mix(params_.baseColor, params_.tipColor, hash11(seed * 13u + 2u));
          color.r += (hash11(seed + 17u) - 0.5f) * 0.05f;
          color.g += (hash11(seed + 19u) - 0.5f) * 0.05f;
          color.b += (hash11(seed + 23u) - 0.5f) * 0.03f;

          GrassInstance inst{};
          inst.position[0] = px;
          inst.position[1] = 0.0f;
          inst.position[2] = pz;
          inst.yaw = yaw;
          inst.color[0] = color.r;
          inst.color[1] = color.g;
          inst.color[2] = color.b;
          inst.scale = std::max(0.05f, scale);

          minX = std::min(minX, px);
          maxX = std::max(maxX, px);
          minZ = std::min(minZ, pz);
          maxZ = std::max(maxZ, pz);

          // Progressive density prefixes: 25% -> 50% -> 100% via checker tiers.
          const int lx = x - x0;
          const int lz = z - z0;
          const bool evenX = (lx & 1) == 0;
          const bool evenZ = (lz & 1) == 0;
          if (evenX && evenZ) {
            tier0.push_back(inst);
          } else if (!evenX && !evenZ) {
            tier1.push_back(inst);
          } else {
            tier2.push_back(inst);
          }
        }
      }

      const uint32_t total =
          static_cast<uint32_t>(tier0.size() + tier1.size() + tier2.size());
      if (total == 0) {
        continue;
      }

      Chunk chunk{};
      chunk.firstInstance = static_cast<uint32_t>(instances.size());
      chunk.countQuarter = static_cast<uint32_t>(tier0.size());
      chunk.countHalf = chunk.countQuarter + static_cast<uint32_t>(tier1.size());
      chunk.countFull = total;
      chunk.aabbMin = glm::vec3(minX - xzPad, 0.0f, minZ - xzPad);
      chunk.aabbMax = glm::vec3(maxX + xzPad, bladeMaxY, maxZ + xzPad);
      chunk.center = 0.5f * (chunk.aabbMin + chunk.aabbMax);

      instances.insert(instances.end(), tier0.begin(), tier0.end());
      instances.insert(instances.end(), tier1.begin(), tier1.end());
      instances.insert(instances.end(), tier2.begin(), tier2.end());
      chunks_.push_back(chunk);
    }
  }

  // Rebuild is rare (ImGui); wait so in-flight frames release the old buffer.
  gfx.waitIdle();
  if (instanceBuffer_.buffer) {
    gfx.destroyBuffer(instanceBuffer_);
  }

  instanceCount_ = static_cast<uint32_t>(instances.size());
  colorBatches_.clear();
  shadowBatches_.clear();
  cullStats_ = {};
  cullStats_.chunkCount = static_cast<uint32_t>(chunks_.size());

  if (instanceCount_ == 0) {
    builtParams_ = params_;
    dirty_ = false;
    return;
  }

  const VkDeviceSize bytes = instances.size() * sizeof(GrassInstance);
  instanceBuffer_ = gfx.createBuffer(
      bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
      VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  gfx.uploadToBuffer(instanceBuffer_, instances.data(), bytes);

  builtParams_ = params_;
  dirty_ = false;
}

void GrassSystem::cull(const glm::vec3& cameraPos, const glm::mat4& cameraViewProj,
                       const glm::mat4& lightViewProj) {
  colorBatches_.clear();
  shadowBatches_.clear();
  cullStats_ = {};
  cullStats_.chunkCount = static_cast<uint32_t>(chunks_.size());

  if (!params_.enabled || chunks_.empty()) {
    return;
  }

  const float fullDist = std::max(0.5f, params_.fullDensityDistance);
  const float halfDist = std::max(fullDist + 0.5f, params_.halfDensityDistance);
  const float maxDist = std::max(halfDist + 0.5f, params_.maxDrawDistance);

  if (!params_.enableCulling) {
    colorBatches_.reserve(chunks_.size());
    shadowBatches_.reserve(chunks_.size());
    for (const Chunk& chunk : chunks_) {
      GrassDrawBatch batch{chunk.firstInstance, chunk.countFull};
      colorBatches_.push_back(batch);
      shadowBatches_.push_back(batch);
      cullStats_.colorInstances += chunk.countFull;
      cullStats_.shadowInstances += chunk.countFull;
    }
    cullStats_.colorChunks = static_cast<uint32_t>(colorBatches_.size());
    cullStats_.shadowChunks = static_cast<uint32_t>(shadowBatches_.size());
    return;
  }

  Plane camPlanes[6];
  Plane lightPlanes[6];
  const int camPlaneCount = extractFrustumPlanes(cameraViewProj, camPlanes, false);
  const int lightPlaneCount = extractFrustumPlanes(lightViewProj, lightPlanes, true);

  colorBatches_.reserve(chunks_.size());
  shadowBatches_.reserve(chunks_.size());

  const glm::vec2 camXZ(cameraPos.x, cameraPos.z);

  for (const Chunk& chunk : chunks_) {
    const glm::vec2 centerXZ(chunk.center.x, chunk.center.z);
    const float dist = glm::length(centerXZ - camXZ);

    // Color pass: camera frustum + distance LOD.
    if (dist <= maxDist && aabbInFrustum(chunk.aabbMin, chunk.aabbMax, camPlanes, camPlaneCount)) {
      uint32_t count = 0;
      if (dist > halfDist) {
        count = chunk.countQuarter;
      } else if (dist > fullDist) {
        count = chunk.countHalf;
      } else {
        count = chunk.countFull;
      }
      if (count > 0) {
        colorBatches_.push_back(GrassDrawBatch{chunk.firstInstance, count});
        cullStats_.colorInstances += count;
      }
    }

    // Shadow pass: light frustum; still fade by camera distance (shadows matter near viewer).
    // Use one LOD coarser than color so shadow cost stays lower.
    if (dist <= maxDist &&
        aabbInFrustum(chunk.aabbMin, chunk.aabbMax, lightPlanes, lightPlaneCount)) {
      uint32_t count = 0;
      if (dist > halfDist) {
        count = chunk.countQuarter;
      } else if (dist > fullDist) {
        count = chunk.countQuarter;  // coarser than color's half
      } else {
        count = chunk.countHalf;  // coarser than color's full
      }
      if (count > 0) {
        shadowBatches_.push_back(GrassDrawBatch{chunk.firstInstance, count});
        cullStats_.shadowInstances += count;
      }
    }
  }

  cullStats_.colorChunks = static_cast<uint32_t>(colorBatches_.size());
  cullStats_.shadowChunks = static_cast<uint32_t>(shadowBatches_.size());
}
