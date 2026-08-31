#include "scene/VoxelScene.h"

#include "gfx/GfxDevice.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {

constexpr float kFltMax = std::numeric_limits<float>::max();

float safeInv(float v) {
  if (std::abs(v) < 1e-8f) {
    return (v >= 0.0f) ? kFltMax : -kFltMax;
  }
  return 1.0f / v;
}

}  // namespace

void VoxelScene::init(GfxDevice& gfx) {
  camera_.setOrbitTarget(glm::vec3(0.0f, 2.0f, 0.0f));
  camera_.setOrbitDistance(18.0f);
  camera_.setYawPitch(0.85f, 0.45f);

  rebuildVoxels(gfx);
}

void VoxelScene::cleanup(GfxDevice& gfx) {
  gfx.destroyBuffer(voxelBuffer_);
  voxelsCpu_.clear();
  occupiedCount_ = 0;
  lastHit_.reset();
}

void VoxelScene::update(float dt) {
  time_ += dt;
  lastHit_ = pickCenterRay();
}

bool VoxelScene::inBounds(const glm::ivec3& p) const {
  return p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < gridSize_ && p.y < gridSize_ &&
         p.z < gridSize_;
}

uint32_t VoxelScene::indexOf(const glm::ivec3& p) const {
  const uint32_t n = static_cast<uint32_t>(gridSize_);
  return static_cast<uint32_t>(p.x) + static_cast<uint32_t>(p.y) * n +
         static_cast<uint32_t>(p.z) * n * n;
}

uint32_t VoxelScene::getVoxel(const glm::ivec3& p) const {
  if (!inBounds(p) || voxelsCpu_.empty()) {
    return 0;
  }
  return voxelsCpu_[indexOf(p)];
}

bool VoxelScene::setVoxelCpu(const glm::ivec3& p, uint32_t material) {
  if (!inBounds(p) || voxelsCpu_.empty()) {
    return false;
  }

  const uint32_t idx = indexOf(p);
  const uint32_t old = voxelsCpu_[idx];
  if (old == material) {
    return false;
  }

  if (old == 0 && material != 0) {
    ++occupiedCount_;
  } else if (old != 0 && material == 0) {
    occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
  }

  voxelsCpu_[idx] = material;
  return true;
}

void VoxelScene::flushVoxels(GfxDevice& gfx) {
  if (voxelsCpu_.empty() || voxelBuffer_.buffer == VK_NULL_HANDLE) {
    return;
  }
  gfx.uploadToBuffer(voxelBuffer_, voxelsCpu_.data(),
                     sizeof(uint32_t) * voxelsCpu_.size());
}

int VoxelScene::applySphereBrush(const glm::ivec3& center, float radius, uint32_t material,
                                 bool placeOnlyEmpty) {
  const float r = std::max(0.0f, radius);
  const int extent = static_cast<int>(std::ceil(r));
  const float r2 = r * r;
  int changed = 0;

  for (int dz = -extent; dz <= extent; ++dz) {
    for (int dy = -extent; dy <= extent; ++dy) {
      for (int dx = -extent; dx <= extent; ++dx) {
        const float dist2 = static_cast<float>(dx * dx + dy * dy + dz * dz);
        // radius 0 edits only the exact center cell.
        if (r <= 0.0f) {
          if (dx != 0 || dy != 0 || dz != 0) {
            continue;
          }
        } else if (dist2 > r2) {
          continue;
        }

        const glm::ivec3 p = center + glm::ivec3(dx, dy, dz);
        if (!inBounds(p)) {
          continue;
        }
        if (placeOnlyEmpty && getVoxel(p) != 0) {
          continue;
        }
        if (setVoxelCpu(p, material)) {
          ++changed;
        }
      }
    }
  }
  return changed;
}

std::optional<VoxelHit> VoxelScene::pickCenterRay() const {
  if (voxelsCpu_.empty() || gridSize_ <= 0 || voxelSize_ <= 0.0f) {
    return std::nullopt;
  }

  const glm::vec3 originWorld = camera_.position();
  const glm::vec3 dirWorld = glm::normalize(camera_.forward());
  if (glm::dot(dirWorld, dirWorld) < 1e-12f) {
    return std::nullopt;
  }

  // Same grid-space convention as the GPU DDA shader.
  const glm::vec3 ro = (originWorld - gridOrigin_) / voxelSize_;
  const glm::vec3 rd = dirWorld;

  const glm::vec3 invDir(safeInv(rd.x), safeInv(rd.y), safeInv(rd.z));
  glm::vec3 sgn(rd.x >= 0.0f ? 1.0f : -1.0f, rd.y >= 0.0f ? 1.0f : -1.0f,
                rd.z >= 0.0f ? 1.0f : -1.0f);

  const glm::vec3 boundsMax(static_cast<float>(gridSize_));
  const glm::vec3 t0 = (glm::vec3(0.0f) - ro) * invDir;
  const glm::vec3 t1 = (boundsMax - ro) * invDir;
  const glm::vec3 tSmaller = glm::min(t0, t1);
  const glm::vec3 tLarger = glm::max(t0, t1);
  const float tEnter = std::max(std::max(tSmaller.x, tSmaller.y), std::max(tSmaller.z, 0.0f));
  const float tExit = std::min(std::min(tLarger.x, tLarger.y), tLarger.z);
  if (tEnter > tExit) {
    return std::nullopt;
  }

  glm::vec3 pos = ro + rd * (tEnter + 1e-4f);
  glm::ivec3 mapPos = glm::clamp(glm::ivec3(glm::floor(pos)), glm::ivec3(0),
                                 glm::ivec3(gridSize_ - 1));

  const glm::vec3 deltaDist = glm::abs(invDir);
  glm::vec3 sideDist =
      (sgn * (glm::vec3(mapPos) - pos) + (sgn * 0.5f + 0.5f)) * deltaDist;
  const glm::ivec3 rayStep(static_cast<int>(sgn.x), static_cast<int>(sgn.y),
                           static_cast<int>(sgn.z));

  glm::bvec3 mask(false);
  if (tSmaller.x > tSmaller.y && tSmaller.x > tSmaller.z) {
    mask = glm::bvec3(true, false, false);
  } else if (tSmaller.y > tSmaller.z) {
    mask = glm::bvec3(false, true, false);
  } else {
    mask = glm::bvec3(false, false, true);
  }

  const uint32_t maxSteps = std::max(maxSteps_, 1u);
  for (uint32_t i = 0; i < maxSteps; ++i) {
    if (!inBounds(mapPos)) {
      break;
    }

    const uint32_t mat = getVoxel(mapPos);
    if (mat != 0) {
      VoxelHit hit{};
      hit.cell = mapPos;
      hit.material = mat;
      hit.normal = glm::ivec3(-glm::vec3(mask) * sgn);
      return hit;
    }

    // Match GLSL: lessThanEqual(sideDist.xyz, min(sideDist.yzx, sideDist.zxy))
    const glm::vec3 yzx(sideDist.y, sideDist.z, sideDist.x);
    const glm::vec3 zxy(sideDist.z, sideDist.x, sideDist.y);
    mask = glm::lessThanEqual(sideDist, glm::min(yzx, zxy));

    sideDist += glm::vec3(mask) * deltaDist;
    mapPos += glm::ivec3(glm::vec3(mask)) * rayStep;
  }

  return std::nullopt;
}

void VoxelScene::handleEditInput(GLFWwindow* window, GfxDevice& gfx) {
  const bool lmb = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  const bool fKey = glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS;

  const bool removeEdge = lmb && !prevLmb_;
  const bool placeEdge = fKey && !prevF_;
  prevLmb_ = lmb;
  prevF_ = fKey;

  // Use previous-frame ImGui capture flags (NewFrame happens later in the renderer).
  bool uiBlocks = false;
  if (ImGui::GetCurrentContext() != nullptr) {
    const ImGuiIO& io = ImGui::GetIO();
    uiBlocks = io.WantCaptureMouse || io.WantCaptureKeyboard;
  }
  if (uiBlocks || (!removeEdge && !placeEdge)) {
    return;
  }

  const std::optional<VoxelHit> hit = pickCenterRay();
  lastHit_ = hit;
  if (!hit.has_value()) {
    return;
  }

  const float radius = std::clamp(brushRadius_, 0.0f, 16.0f);
  brushRadius_ = radius;

  if (removeEdge) {
    if (applySphereBrush(hit->cell, radius, 0, false) > 0) {
      flushVoxels(gfx);
    }
    lastHit_ = pickCenterRay();
    return;
  }

  if (placeEdge) {
    const glm::ivec3 place = hit->cell + hit->normal;
    const uint32_t mat = static_cast<uint32_t>(std::clamp(brushMaterial_, 1, 255));
    if (applySphereBrush(place, radius, mat, true) > 0) {
      flushVoxels(gfx);
    }
    lastHit_ = pickCenterRay();
  }
}

void VoxelScene::rebuildVoxels(GfxDevice& gfx) {
  const int n = std::clamp(gridSize_, 8, 128);
  gridSize_ = n;
  maxSteps_ = static_cast<uint32_t>(std::max(16, n * 3));

  const float half = 0.5f * static_cast<float>(n);
  gridOrigin_ = glm::vec3(-half, -half * 0.35f, -half) * voxelSize_;

  const size_t count = static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n);
  voxelsCpu_.assign(count, 0u);
  occupiedCount_ = 0;
  lastHit_.reset();

  const float radius = 0.38f * static_cast<float>(n);
  const float center = half - 0.5f;
  const int cubeMin = n / 2 - std::max(2, n / 10);
  const int cubeMax = n / 2 + std::max(2, n / 10);

  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        const float fx = static_cast<float>(x) + 0.5f - center;
        const float fy = static_cast<float>(y) + 0.5f - center;
        const float fz = static_cast<float>(z) + 0.5f - center;
        const float dist = std::sqrt(fx * fx + fy * fy + fz * fz);
        const bool inShell = (dist <= radius && dist >= radius - 1.6f);
        const bool inCube =
            x >= cubeMin && x < cubeMax && y >= cubeMin && y < cubeMax && z >= cubeMin && z < cubeMax;
        if (!inShell && !inCube) {
          continue;
        }
        const size_t idx = static_cast<size_t>(x) + static_cast<size_t>(y) * static_cast<size_t>(n) +
                           static_cast<size_t>(z) * static_cast<size_t>(n) * static_cast<size_t>(n);
        voxelsCpu_[idx] = inCube ? 2u : 1u;
        ++occupiedCount_;
      }
    }
  }

  const VkDeviceSize bytes = sizeof(uint32_t) * voxelsCpu_.size();
  if (voxelBuffer_.buffer == VK_NULL_HANDLE || voxelBuffer_.size < bytes) {
    gfx.destroyBuffer(voxelBuffer_);
    voxelBuffer_ = gfx.createBuffer(bytes,
                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                    VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }
  gfx.uploadToBuffer(voxelBuffer_, voxelsCpu_.data(), bytes);
}
