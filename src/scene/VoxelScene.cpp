#include "scene/VoxelScene.h"

#include "gfx/GfxDevice.h"
#include "gfx/Texture.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <algorithm>
#include <bit>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

constexpr float kFltMax = std::numeric_limits<float>::max();

float safeInv(float v) {
  if (std::abs(v) < 1e-8f) {
    return (v >= 0.0f) ? kFltMax : -kFltMax;
  }
  return 1.0f / v;
}

glm::bvec3 stepMaskCpu(const glm::vec3& sideDist) {
  const glm::vec3 yzx(sideDist.y, sideDist.z, sideDist.x);
  const glm::vec3 zxy(sideDist.z, sideDist.x, sideDist.y);
  return glm::lessThanEqual(sideDist, glm::min(yzx, zxy));
}

// Exact Shadertoy 8^3 template packed as 16 uints (512 bits), index = y*64 + z*8 + x.
constexpr uint32_t kMicroTemplateWords[16] = {
    0x818181ffu, 0xff818181u, 0x00004281u, 0x81420000u, 0x00240081u, 0x81002400u,
    0x18000081u, 0x81000018u, 0x18000081u, 0x81000018u, 0x00240081u, 0x81002400u,
    0x00004281u, 0x81420000u, 0x818181ffu, 0xff818181u,
};

}  // namespace

void VoxelScene::init(GfxDevice& gfx) {
  camera_.setOrbitTarget(glm::vec3(0.0f, 0.4f, 0.0f));
  camera_.setOrbitDistance(22.0f);
  camera_.setYawPitch(0.75f, 0.55f);

  const std::string skyPath = std::string(VE_ASSETS_DIR) + "/sky/autumn_field_puresky_2k.hdr";
  try {
    sky_ = TextureFactory::loadHdrEquirect(gfx, skyPath);
    std::cout << "Loaded voxel skybox: " << skyPath << std::endl;
  } catch (const std::exception& e) {
    std::cerr << "Voxel skybox load failed: " << e.what() << std::endl;
    sky_ = TextureFactory::createSolid(gfx, skyColor_.r, skyColor_.g, skyColor_.b);
  }

  rebuildVoxels(gfx);
}

void VoxelScene::cleanup(GfxDevice& gfx) {
  gfx.destroyBuffer(voxelBuffer_);
  gfx.destroyBuffer(microBuffer_);
  TextureFactory::destroy(gfx, sky_);
  voxelsCpu_.clear();
  microCpu_.clear();
  occupiedCount_ = 0;
  occupiedMicroCount_ = 0;
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

bool VoxelScene::microInBounds(const glm::ivec3& m) const {
  return m.x >= 0 && m.y >= 0 && m.z >= 0 && m.x < kMicroRes && m.y < kMicroRes && m.z < kMicroRes;
}

uint32_t VoxelScene::indexOf(const glm::ivec3& p) const {
  const uint32_t n = static_cast<uint32_t>(gridSize_);
  return static_cast<uint32_t>(p.x) + static_cast<uint32_t>(p.y) * n +
         static_cast<uint32_t>(p.z) * n * n;
}

uint32_t VoxelScene::microBitIndex(const glm::ivec3& m) const {
  // Match shader/template: y*64 + z*8 + x
  return static_cast<uint32_t>(m.y * 64 + m.z * 8 + m.x);
}

uint32_t VoxelScene::getVoxel(const glm::ivec3& p) const {
  if (!inBounds(p) || voxelsCpu_.empty()) {
    return 0;
  }
  return materialOf(voxelsCpu_[indexOf(p)]);
}

bool VoxelScene::getMicro(const glm::ivec3& coarse, const glm::ivec3& micro) const {
  if (!inBounds(coarse) || !microInBounds(micro) || microCpu_.empty()) {
    return false;
  }
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t mask = 1u << (bit % 32u);
  const uint32_t base = indexOf(coarse) * static_cast<uint32_t>(kMicroWords);
  return (microCpu_[base + word] & mask) != 0u;
}

bool VoxelScene::microBrickEmpty(uint32_t coarseIndex) const {
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    if (microCpu_[base + static_cast<uint32_t>(i)] != 0u) {
      return false;
    }
  }
  return true;
}

void VoxelScene::clearMicroBrick(uint32_t coarseIndex) {
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    const uint32_t w = microCpu_[base + static_cast<uint32_t>(i)];
    if (w != 0u) {
      occupiedMicroCount_ -= static_cast<uint32_t>(std::popcount(w));
      microCpu_[base + static_cast<uint32_t>(i)] = 0u;
    }
  }
  syncHasMicroFlag(coarseIndex);
}

void VoxelScene::fillMicroBrickTemplate(uint32_t coarseIndex) {
  // clearMicroBrick syncs hasMicro=false; restore after filling bits.
  clearMicroBrick(coarseIndex);
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    const uint32_t w = kMicroTemplateWords[i];
    microCpu_[base + static_cast<uint32_t>(i)] = w;
    occupiedMicroCount_ += static_cast<uint32_t>(std::popcount(w));
  }
  syncHasMicroFlag(coarseIndex);
}

void VoxelScene::fillMicroBrickSolid(uint32_t coarseIndex) {
  clearMicroBrick(coarseIndex);
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    microCpu_[base + static_cast<uint32_t>(i)] = 0xFFFFFFFFu;
    occupiedMicroCount_ += 32u;
  }
  syncHasMicroFlag(coarseIndex);
}

void VoxelScene::syncHasMicroFlag(uint32_t coarseIndex) {
  if (coarseIndex >= voxelsCpu_.size()) {
    return;
  }
  const uint32_t mat = materialOf(voxelsCpu_[coarseIndex]);
  if (mat == 0u) {
    voxelsCpu_[coarseIndex] = 0u;
    return;
  }
  voxelsCpu_[coarseIndex] = packVoxel(mat, !microBrickEmpty(coarseIndex));
}

bool VoxelScene::setVoxelCpu(const glm::ivec3& p, uint32_t material) {
  if (!inBounds(p) || voxelsCpu_.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(p);
  const uint32_t oldMat = materialOf(voxelsCpu_[idx]);
  const uint32_t newMat = material & kVoxelMatMask;
  if (oldMat == newMat) {
    return false;
  }
  if (oldMat == 0 && newMat != 0) {
    ++occupiedCount_;
    voxelsCpu_[idx] = packVoxel(newMat, false);
    fillMicroBrickSolid(idx);
  } else if (oldMat != 0 && newMat == 0) {
    occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
    clearMicroBrick(idx);
    voxelsCpu_[idx] = 0u;
  } else {
    voxelsCpu_[idx] = packVoxel(newMat, hasMicroOf(voxelsCpu_[idx]));
  }
  return true;
}

bool VoxelScene::setMicroCpu(const glm::ivec3& coarse, const glm::ivec3& micro, bool solid) {
  if (!inBounds(coarse) || !microInBounds(micro) || microCpu_.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(coarse);
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t mask = 1u << (bit % 32u);
  uint32_t& dst = microCpu_[idx * static_cast<uint32_t>(kMicroWords) + word];
  const bool was = (dst & mask) != 0u;
  if (was == solid) {
    return false;
  }
  if (solid) {
    dst |= mask;
    ++occupiedMicroCount_;
  } else {
    dst &= ~mask;
    occupiedMicroCount_ = occupiedMicroCount_ > 0 ? occupiedMicroCount_ - 1 : 0;
  }
  syncHasMicroFlag(idx);
  return true;
}

void VoxelScene::ensureCoarseBrick(const glm::ivec3& coarse, uint32_t material) {
  if (!inBounds(coarse)) {
    return;
  }
  const uint32_t idx = indexOf(coarse);
  if (materialOf(voxelsCpu_[idx]) == 0u) {
    ++occupiedCount_;
    // Start empty so placement creates the first micros explicitly.
    clearMicroBrick(idx);
    voxelsCpu_[idx] = packVoxel(material, false);
  }
}

void VoxelScene::flushAll(GfxDevice& gfx) {
  if (!voxelsCpu_.empty() && voxelBuffer_.buffer != VK_NULL_HANDLE) {
    gfx.uploadToBuffer(voxelBuffer_, voxelsCpu_.data(), sizeof(uint32_t) * voxelsCpu_.size());
  }
  if (!microCpu_.empty() && microBuffer_.buffer != VK_NULL_HANDLE) {
    gfx.uploadToBuffer(microBuffer_, microCpu_.data(), sizeof(uint32_t) * microCpu_.size());
  }
}

int VoxelScene::applyCoarseSphereBrush(const glm::ivec3& center, float radius, uint32_t material,
                                       bool placeOnlyEmpty) {
  const float r = std::max(0.0f, radius);
  const int extent = static_cast<int>(std::ceil(r));
  const float r2 = r * r;
  int changed = 0;
  for (int dz = -extent; dz <= extent; ++dz) {
    for (int dy = -extent; dy <= extent; ++dy) {
      for (int dx = -extent; dx <= extent; ++dx) {
        const float dist2 = static_cast<float>(dx * dx + dy * dy + dz * dz);
        if (r <= 0.0f) {
          if (dx || dy || dz) {
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

int VoxelScene::applyMicroSphereBrush(const glm::ivec3& coarse, const glm::ivec3& micro,
                                      float radius, bool solid, uint32_t placeMaterial) {
  const float r = std::max(0.0f, radius);
  const int extent = static_cast<int>(std::ceil(r));
  const float r2 = r * r;
  int changed = 0;

  // Work in absolute micro coordinates so the brush can cross coarse boundaries.
  const glm::ivec3 absCenter = coarse * kMicroRes + micro;

  for (int dz = -extent; dz <= extent; ++dz) {
    for (int dy = -extent; dy <= extent; ++dy) {
      for (int dx = -extent; dx <= extent; ++dx) {
        const float dist2 = static_cast<float>(dx * dx + dy * dy + dz * dz);
        if (r <= 0.0f) {
          if (dx || dy || dz) {
            continue;
          }
        } else if (dist2 > r2) {
          continue;
        }

        const glm::ivec3 absMicro = absCenter + glm::ivec3(dx, dy, dz);
        auto divFloor = [](int a, int b) {
          int q = a / b;
          int r = a % b;
          if (r != 0 && ((r < 0) != (b < 0))) {
            --q;
          }
          return q;
        };
        glm::ivec3 c(divFloor(absMicro.x, kMicroRes), divFloor(absMicro.y, kMicroRes),
                     divFloor(absMicro.z, kMicroRes));
        glm::ivec3 m = absMicro - c * kMicroRes;
        if (!inBounds(c) || !microInBounds(m)) {
          continue;
        }

        if (solid) {
          ensureCoarseBrick(c, placeMaterial);
          if (setMicroCpu(c, m, true)) {
            ++changed;
          }
        } else {
          if (getVoxel(c) == 0) {
            continue;
          }
          if (setMicroCpu(c, m, false)) {
            ++changed;
            const uint32_t idx = indexOf(c);
            if (microBrickEmpty(idx)) {
              // Drop empty coarse cell.
              voxelsCpu_[idx] = 0;
              occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
            }
          }
        }
      }
    }
  }
  return changed;
}

std::optional<VoxelHit> VoxelScene::pickCenterRay() const {
  if (nestedMicroVoxels_) {
    return pickCenterRayNested();
  }
  return pickCenterRayCoarse();
}

std::optional<VoxelHit> VoxelScene::pickCenterRayCoarse() const {
  if (voxelsCpu_.empty() || gridSize_ <= 0 || voxelSize_ <= 0.0f) {
    return std::nullopt;
  }

  const glm::vec3 originWorld = camera_.position();
  const glm::vec3 rd = glm::normalize(camera_.forward());
  if (glm::dot(rd, rd) < 1e-12f) {
    return std::nullopt;
  }
  const glm::vec3 ro = (originWorld - gridOrigin_) / voxelSize_;
  const glm::vec3 invDir(safeInv(rd.x), safeInv(rd.y), safeInv(rd.z));
  const glm::vec3 sgn(rd.x >= 0.0f ? 1.0f : -1.0f, rd.y >= 0.0f ? 1.0f : -1.0f,
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
  glm::ivec3 mapPos =
      glm::clamp(glm::ivec3(glm::floor(pos)), glm::ivec3(0), glm::ivec3(gridSize_ - 1));
  const glm::vec3 deltaDist = glm::abs(invDir);
  glm::vec3 sideDist = (sgn * (glm::vec3(mapPos) - pos) + (sgn * 0.5f + 0.5f)) * deltaDist;
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

  for (uint32_t i = 0; i < std::max(maxSteps_, 1u); ++i) {
    if (!inBounds(mapPos)) {
      break;
    }
    const uint32_t mat = getVoxel(mapPos);
    if (mat != 0) {
      VoxelHit hit{};
      hit.cell = mapPos;
      hit.material = mat;
      hit.normal = glm::ivec3(-glm::vec3(mask) * sgn);
      hit.hasMicro = false;
      return hit;
    }
    mask = stepMaskCpu(sideDist);
    sideDist += glm::vec3(mask) * deltaDist;
    mapPos += glm::ivec3(glm::vec3(mask)) * rayStep;
  }
  return std::nullopt;
}

std::optional<VoxelHit> VoxelScene::pickCenterRayNested() const {
  if (voxelsCpu_.empty() || microCpu_.empty() || gridSize_ <= 0 || voxelSize_ <= 0.0f) {
    return std::nullopt;
  }

  const glm::vec3 originWorld = camera_.position();
  const glm::vec3 rd = glm::normalize(camera_.forward());
  if (glm::dot(rd, rd) < 1e-12f) {
    return std::nullopt;
  }
  const glm::vec3 ro = (originWorld - gridOrigin_) / voxelSize_;
  const glm::vec3 invDir(safeInv(rd.x), safeInv(rd.y), safeInv(rd.z));
  const glm::vec3 sgn(rd.x >= 0.0f ? 1.0f : -1.0f, rd.y >= 0.0f ? 1.0f : -1.0f,
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
  glm::ivec3 mapPos =
      glm::clamp(glm::ivec3(glm::floor(pos)), glm::ivec3(0), glm::ivec3(gridSize_ - 1));
  const glm::ivec3 startPos = mapPos;
  const glm::vec3 deltaDist = glm::abs(invDir);
  glm::vec3 sideDist = (sgn * (glm::vec3(mapPos) - pos) + (sgn * 0.5f + 0.5f)) * deltaDist;
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

  for (uint32_t i = 0; i < std::max(maxSteps_, 1u); ++i) {
    if (!inBounds(mapPos)) {
      break;
    }

    const uint32_t packed = voxelsCpu_[indexOf(mapPos)];
    const uint32_t mat = materialOf(packed);
    if (mat != 0) {
      // Empty micro brick: skip nested DDA (matches GPU HAS_MICRO bit).
      if (hasMicroOf(packed)) {
        glm::vec3 local01;
        if (mapPos == startPos) {
          local01 = glm::clamp(pos - glm::vec3(mapPos), glm::vec3(0.0f), glm::vec3(0.9999f));
        } else {
          const glm::vec3 mini = ((glm::vec3(mapPos) - ro) + 0.5f - 0.5f * sgn) * invDir;
          const float d = std::max(mini.x, std::max(mini.y, mini.z));
          const glm::vec3 intersect = ro + rd * d;
          local01 = glm::clamp(intersect - glm::vec3(mapPos), glm::vec3(0.0f), glm::vec3(0.9999f));
        }

        glm::vec3 localPos = glm::clamp(local01 * 8.0f, glm::vec3(0.0001f), glm::vec3(7.9999f));
        glm::ivec3 microPos = glm::ivec3(glm::floor(localPos));
        glm::vec3 microSide =
            (sgn * (glm::vec3(microPos) - localPos) + (sgn * 0.5f + 0.5f)) * deltaDist;
        glm::bvec3 microMask = mask;

        auto finishHit = [&](const glm::ivec3& m, const glm::bvec3& msk) -> VoxelHit {
          VoxelHit hit{};
          hit.cell = mapPos;
          hit.micro = m;
          hit.material = mat;
          hit.normal = glm::ivec3(-glm::vec3(msk) * sgn);
          hit.hasMicro = true;
          return hit;
        };

        if (getMicro(mapPos, microPos)) {
          return finishHit(microPos, microMask);
        }

        for (int s = 0; s < 32; ++s) {
          microMask = stepMaskCpu(microSide);
          microSide += glm::vec3(microMask) * deltaDist;
          microPos += glm::ivec3(glm::vec3(microMask)) * rayStep;
          if (!microInBounds(microPos)) {
            break;
          }
          if (getMicro(mapPos, microPos)) {
            return finishHit(microPos, microMask);
          }
        }
        // Hollow template miss: continue coarse traversal.
      }
    }

    mask = stepMaskCpu(sideDist);
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
  const uint32_t mat = static_cast<uint32_t>(std::clamp(brushMaterial_, 1, 255));

  int changed = 0;
  if (nestedMicroVoxels_ && hit->hasMicro) {
    if (removeEdge) {
      changed = applyMicroSphereBrush(hit->cell, hit->micro, radius, false, mat);
    } else if (placeEdge) {
      glm::ivec3 placeMicro = hit->micro + hit->normal;
      glm::ivec3 placeCoarse = hit->cell;
      // Cross coarse boundaries when placing on an outer micro face.
      for (int a = 0; a < 3; ++a) {
        if (placeMicro[a] < 0) {
          placeMicro[a] = kMicroRes - 1;
          placeCoarse[a] -= 1;
        } else if (placeMicro[a] >= kMicroRes) {
          placeMicro[a] = 0;
          placeCoarse[a] += 1;
        }
      }
      if (inBounds(placeCoarse) && microInBounds(placeMicro)) {
        changed = applyMicroSphereBrush(placeCoarse, placeMicro, radius, true, mat);
      }
    }
  } else {
    if (removeEdge) {
      changed = applyCoarseSphereBrush(hit->cell, radius, 0, false);
    } else if (placeEdge) {
      changed = applyCoarseSphereBrush(hit->cell + hit->normal, radius, mat, true);
    }
  }

  if (changed > 0) {
    flushAll(gfx);
  }
  lastHit_ = pickCenterRay();
}

void VoxelScene::rebuildVoxels(GfxDevice& gfx) {
  const int n = std::clamp(gridSize_, 8, 64);  // micro bits: keep default range practical
  gridSize_ = n;
  maxSteps_ = static_cast<uint32_t>(std::max(16, n * 3));

  const float half = 0.5f * static_cast<float>(n);
  // Flat ground near y=0; keep XZ centered on origin.
  gridOrigin_ = glm::vec3(-half, 0.0f, -half) * voxelSize_;

  const size_t count = static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n);
  voxelsCpu_.assign(count, 0u);
  microCpu_.assign(count * static_cast<size_t>(kMicroWords), 0u);
  occupiedCount_ = 0;
  occupiedMicroCount_ = 0;
  lastHit_.reset();

  // Single-material flat ground slab (2 coarse cells thick).
  constexpr uint32_t kGroundMat = 1u;
  const int groundThickness = 2;
  for (int z = 0; z < n; ++z) {
    for (int x = 0; x < n; ++x) {
      for (int y = 0; y < groundThickness; ++y) {
        const glm::ivec3 p(x, y, z);
        const uint32_t idx = indexOf(p);
        voxelsCpu_[idx] = packVoxel(kGroundMat, false);
        ++occupiedCount_;
        fillMicroBrickSolid(idx);
      }
    }
  }

  const VkDeviceSize voxelBytes = sizeof(uint32_t) * voxelsCpu_.size();
  const VkDeviceSize microBytes = sizeof(uint32_t) * microCpu_.size();

  if (voxelBuffer_.buffer == VK_NULL_HANDLE || voxelBuffer_.size < voxelBytes) {
    gfx.destroyBuffer(voxelBuffer_);
    voxelBuffer_ = gfx.createBuffer(voxelBytes,
                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                    VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }
  if (microBuffer_.buffer == VK_NULL_HANDLE || microBuffer_.size < microBytes) {
    gfx.destroyBuffer(microBuffer_);
    microBuffer_ = gfx.createBuffer(microBytes,
                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                    VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }

  gfx.uploadToBuffer(voxelBuffer_, voxelsCpu_.data(), voxelBytes);
  gfx.uploadToBuffer(microBuffer_, microCpu_.data(), microBytes);
}
