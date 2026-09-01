#include "scene/VoxelScene.h"

#include "gfx/GfxDevice.h"
#include "gfx/Texture.h"

#include <GLFW/glfw3.h>
#include <imgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
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

void writeMat4(float* dst, const glm::mat4& m) {
  std::memcpy(dst, glm::value_ptr(m), sizeof(float) * 16);
}

// Exact Shadertoy 8^3 template packed as 16 uints (512 bits), index = y*64 + z*8 + x.
constexpr uint32_t kMicroTemplateWords[16] = {
    0x818181ffu, 0xff818181u, 0x00004281u, 0x81420000u, 0x00240081u, 0x81002400u,
    0x18000081u, 0x81000018u, 0x18000081u, 0x81000018u, 0x00240081u, 0x81002400u,
    0x00004281u, 0x81420000u, 0x818181ffu, 0xff818181u,
};

}  // namespace

glm::mat4 VoxelObject::objectToWorld() const {
  const float extent = static_cast<float>(gridSize) * voxelSize;
  const glm::vec3 centerLocal(0.5f * extent, 0.5f * extent, 0.5f * extent);
  // Local meters: grid corner at 0. Rotate about grid center, then place center at position.
  return glm::translate(glm::mat4(1.0f), position) * glm::mat4_cast(rotation) *
         glm::translate(glm::mat4(1.0f), -centerLocal);
}

glm::mat4 VoxelObject::worldToObject() const {
  return glm::inverse(objectToWorld());
}

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
  gfx.destroyBuffer(objectBuffer_);
  TextureFactory::destroy(gfx, sky_);
  objects_.clear();
  objectsGpu_.clear();
  voxelsCpu_.clear();
  microCpu_.clear();
  occupiedCount_ = 0;
  occupiedMicroCount_ = 0;
  lastHit_.reset();
}

void VoxelScene::update(float dt) {
  time_ += dt;
  for (VoxelObject& o : objects_) {
    o.nestedMicro = nestedMicroVoxels_;
  }
  if (objects_.size() >= 2) {
    objects_[1].enabled = spinnerEnabled_;
    objects_[1].rotation = glm::angleAxis(time_ * spinSpeed_, glm::vec3(0.0f, 1.0f, 0.0f));
  }
  fillGpuObjectRecords();
  lastHit_ = pickCenterRay();
}

glm::vec3 VoxelScene::gridOrigin() const {
  if (objects_.empty()) {
    const float half = 0.5f * static_cast<float>(gridSize_);
    return glm::vec3(-half, 0.0f, -half) * voxelSize_;
  }
  // Legacy accessor: world position of ground grid corner (local 0).
  return glm::vec3(objects_[0].objectToWorld() * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

VoxelObject* VoxelScene::ground() {
  return objects_.empty() ? nullptr : &objects_[0];
}

const VoxelObject* VoxelScene::ground() const {
  return objects_.empty() ? nullptr : &objects_[0];
}

bool VoxelScene::inBounds(const VoxelObject& o, const glm::ivec3& p) const {
  return p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < o.gridSize && p.y < o.gridSize &&
         p.z < o.gridSize;
}

bool VoxelScene::microInBounds(const glm::ivec3& m) const {
  return m.x >= 0 && m.y >= 0 && m.z >= 0 && m.x < kMicroRes && m.y < kMicroRes && m.z < kMicroRes;
}

uint32_t VoxelScene::indexOf(const VoxelObject& o, const glm::ivec3& p) const {
  const uint32_t n = static_cast<uint32_t>(o.gridSize);
  return static_cast<uint32_t>(p.x) + static_cast<uint32_t>(p.y) * n +
         static_cast<uint32_t>(p.z) * n * n;
}

uint32_t VoxelScene::microBitIndex(const glm::ivec3& m) const {
  return static_cast<uint32_t>(m.y * 64 + m.z * 8 + m.x);
}

uint32_t VoxelScene::getVoxel(const VoxelObject& o, const glm::ivec3& p) const {
  if (!inBounds(o, p) || o.voxelsCpu.empty()) {
    return 0;
  }
  return materialOf(o.voxelsCpu[indexOf(o, p)]);
}

bool VoxelScene::getMicro(const VoxelObject& o, const glm::ivec3& coarse,
                          const glm::ivec3& micro) const {
  if (!inBounds(o, coarse) || !microInBounds(micro) || o.microCpu.empty()) {
    return false;
  }
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t mask = 1u << (bit % 32u);
  const uint32_t base = indexOf(o, coarse) * static_cast<uint32_t>(kMicroWords);
  return (o.microCpu[base + word] & mask) != 0u;
}

bool VoxelScene::microBrickEmpty(const VoxelObject& o, uint32_t coarseIndex) const {
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    if (o.microCpu[base + static_cast<uint32_t>(i)] != 0u) {
      return false;
    }
  }
  return true;
}

void VoxelScene::clearMicroBrick(VoxelObject& o, uint32_t coarseIndex) {
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    const uint32_t w = o.microCpu[base + static_cast<uint32_t>(i)];
    if (w != 0u) {
      occupiedMicroCount_ -= static_cast<uint32_t>(std::popcount(w));
      o.microCpu[base + static_cast<uint32_t>(i)] = 0u;
    }
  }
  syncHasMicroFlag(o, coarseIndex);
}

void VoxelScene::fillMicroBrickTemplate(VoxelObject& o, uint32_t coarseIndex) {
  clearMicroBrick(o, coarseIndex);
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    const uint32_t w = kMicroTemplateWords[i];
    o.microCpu[base + static_cast<uint32_t>(i)] = w;
    occupiedMicroCount_ += static_cast<uint32_t>(std::popcount(w));
  }
  syncHasMicroFlag(o, coarseIndex);
}

void VoxelScene::fillMicroBrickSolid(VoxelObject& o, uint32_t coarseIndex) {
  clearMicroBrick(o, coarseIndex);
  const uint32_t base = coarseIndex * static_cast<uint32_t>(kMicroWords);
  for (int i = 0; i < kMicroWords; ++i) {
    o.microCpu[base + static_cast<uint32_t>(i)] = 0xFFFFFFFFu;
    occupiedMicroCount_ += 32u;
  }
  syncHasMicroFlag(o, coarseIndex);
}

void VoxelScene::syncHasMicroFlag(VoxelObject& o, uint32_t coarseIndex) {
  if (coarseIndex >= o.voxelsCpu.size()) {
    return;
  }
  const uint32_t mat = materialOf(o.voxelsCpu[coarseIndex]);
  if (mat == 0u) {
    o.voxelsCpu[coarseIndex] = 0u;
    return;
  }
  o.voxelsCpu[coarseIndex] = packVoxel(mat, !microBrickEmpty(o, coarseIndex));
}

bool VoxelScene::setVoxelCpu(VoxelObject& o, const glm::ivec3& p, uint32_t material) {
  if (!inBounds(o, p) || o.voxelsCpu.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(o, p);
  const uint32_t oldMat = materialOf(o.voxelsCpu[idx]);
  const uint32_t newMat = material & kVoxelMatMask;
  if (oldMat == newMat) {
    return false;
  }
  if (oldMat == 0 && newMat != 0) {
    ++occupiedCount_;
    o.voxelsCpu[idx] = packVoxel(newMat, false);
    fillMicroBrickSolid(o, idx);
  } else if (oldMat != 0 && newMat == 0) {
    occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
    clearMicroBrick(o, idx);
    o.voxelsCpu[idx] = 0u;
  } else {
    o.voxelsCpu[idx] = packVoxel(newMat, hasMicroOf(o.voxelsCpu[idx]));
  }
  return true;
}

bool VoxelScene::setMicroCpu(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                             bool solid) {
  if (!inBounds(o, coarse) || !microInBounds(micro) || o.microCpu.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(o, coarse);
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t mask = 1u << (bit % 32u);
  uint32_t& dst = o.microCpu[idx * static_cast<uint32_t>(kMicroWords) + word];
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
  syncHasMicroFlag(o, idx);
  return true;
}

void VoxelScene::ensureCoarseBrick(VoxelObject& o, const glm::ivec3& coarse, uint32_t material) {
  if (!inBounds(o, coarse)) {
    return;
  }
  const uint32_t idx = indexOf(o, coarse);
  if (materialOf(o.voxelsCpu[idx]) == 0u) {
    ++occupiedCount_;
    clearMicroBrick(o, idx);
    o.voxelsCpu[idx] = packVoxel(material, false);
  }
}

void VoxelScene::packObjectPool() {
  voxelsCpu_.clear();
  microCpu_.clear();
  occupiedCount_ = 0;
  occupiedMicroCount_ = 0;

  for (VoxelObject& o : objects_) {
    o.voxelOffset = static_cast<uint32_t>(voxelsCpu_.size());
    o.microOffset = static_cast<uint32_t>(microCpu_.size());
    voxelsCpu_.insert(voxelsCpu_.end(), o.voxelsCpu.begin(), o.voxelsCpu.end());
    microCpu_.insert(microCpu_.end(), o.microCpu.begin(), o.microCpu.end());

    for (uint32_t packed : o.voxelsCpu) {
      if (materialOf(packed) != 0u) {
        ++occupiedCount_;
      }
    }
    for (uint32_t w : o.microCpu) {
      occupiedMicroCount_ += static_cast<uint32_t>(std::popcount(w));
    }
  }
}

void VoxelScene::fillGpuObjectRecords() {
  objectsGpu_.resize(objects_.size());
  for (size_t i = 0; i < objects_.size(); ++i) {
    const VoxelObject& o = objects_[i];
    GpuVoxelObject& g = objectsGpu_[i];
    writeMat4(g.worldToObject, o.worldToObject());
    writeMat4(g.objectToWorld, o.objectToWorld());
    g.voxelSize = o.voxelSize;
    g._pad0[0] = g._pad0[1] = g._pad0[2] = 0.0f;
    g.gridSize[0] = static_cast<uint32_t>(o.gridSize);
    g.gridSize[1] = static_cast<uint32_t>(o.gridSize);
    g.gridSize[2] = static_cast<uint32_t>(o.gridSize);
    g.flags = 0;
    if (o.nestedMicro) {
      g.flags |= VoxelObject::kFlagNestedMicro;
    }
    if (o.enabled) {
      g.flags |= VoxelObject::kFlagEnabled;
    }
    g.voxelOffset = o.voxelOffset;
    g.microOffset = o.microOffset;
    g._pad1[0] = g._pad1[1] = 0;
  }
}

void VoxelScene::ensureGpuBuffers(GfxDevice& gfx) {
  const VkDeviceSize voxelBytes = sizeof(uint32_t) * voxelsCpu_.size();
  const VkDeviceSize microBytes = sizeof(uint32_t) * microCpu_.size();
  const VkDeviceSize objectBytes = sizeof(GpuVoxelObject) * std::max<size_t>(objectsGpu_.size(), 1);

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
  if (objectBuffer_.buffer == VK_NULL_HANDLE || objectBuffer_.size < objectBytes) {
    gfx.destroyBuffer(objectBuffer_);
    objectBuffer_ = gfx.createBuffer(objectBytes,
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                     VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }
}

void VoxelScene::flushObject(GfxDevice& gfx, int objectIndex) {
  if (objectIndex < 0 || objectIndex >= static_cast<int>(objects_.size())) {
    return;
  }
  VoxelObject& o = objects_[static_cast<size_t>(objectIndex)];

  if (!o.voxelsCpu.empty() && o.voxelOffset + o.voxelsCpu.size() <= voxelsCpu_.size()) {
    std::copy(o.voxelsCpu.begin(), o.voxelsCpu.end(), voxelsCpu_.begin() + o.voxelOffset);
    if (voxelBuffer_.buffer != VK_NULL_HANDLE) {
      gfx.uploadToBuffer(voxelBuffer_, o.voxelsCpu.data(),
                         sizeof(uint32_t) * o.voxelsCpu.size(),
                         sizeof(uint32_t) * static_cast<VkDeviceSize>(o.voxelOffset));
    }
  }
  if (!o.microCpu.empty() && o.microOffset + o.microCpu.size() <= microCpu_.size()) {
    std::copy(o.microCpu.begin(), o.microCpu.end(), microCpu_.begin() + o.microOffset);
    if (microBuffer_.buffer != VK_NULL_HANDLE) {
      gfx.uploadToBuffer(microBuffer_, o.microCpu.data(),
                         sizeof(uint32_t) * o.microCpu.size(),
                         sizeof(uint32_t) * static_cast<VkDeviceSize>(o.microOffset));
    }
  }
}

void VoxelScene::uploadObjectTransforms(GfxDevice& gfx) {
  fillGpuObjectRecords();
  if (objectsGpu_.empty() || objectBuffer_.buffer == VK_NULL_HANDLE) {
    return;
  }
  gfx.uploadToBuffer(objectBuffer_, objectsGpu_.data(),
                     sizeof(GpuVoxelObject) * objectsGpu_.size());
}

void VoxelScene::buildGroundObject(VoxelObject& o) const {
  const int n = std::clamp(gridSize_, 8, 64);
  o.gridSize = n;
  o.voxelSize = voxelSize_;
  o.nestedMicro = nestedMicroVoxels_;
  o.editable = true;
  o.enabled = true;
  o.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

  const float half = 0.5f * static_cast<float>(n);
  // Center of ground slab in world (matches old gridOrigin corner + half extent in XZ, y mid slab).
  // objectToWorld rotates about grid center then places center at `position`.
  // Old corner was (-half, 0, -half)*vs; center was (0, half*vs? no: y from 0..2 cells).
  // Grid local center = (half, half, half)*vs in meters from corner.
  // We want corner at (-half, 0, -half)*vs => center at (0, half*vs, 0) in world for y?
  // corner_world = position - R*centerLocal. With R=I: position = corner + centerLocal
  // = (-half,0,-half)*vs + (half, half, half)*vs = (0, half*vs, 0).
  o.position = glm::vec3(0.0f, half * voxelSize_, 0.0f);

  const size_t count = static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n);
  o.voxelsCpu.assign(count, 0u);
  o.microCpu.assign(count * static_cast<size_t>(kMicroWords), 0u);

  constexpr uint32_t kGroundMat = 1u;
  const int groundThickness = 2;
  // Temporary occupied counters via local ops — packObjectPool recounts later.
  // Use a mutable copy pattern: fill through non-const helpers on a temp scene? Inline fill:
  for (int z = 0; z < n; ++z) {
    for (int x = 0; x < n; ++x) {
      for (int y = 0; y < groundThickness; ++y) {
        const uint32_t idx =
            static_cast<uint32_t>(x) + static_cast<uint32_t>(y) * static_cast<uint32_t>(n) +
            static_cast<uint32_t>(z) * static_cast<uint32_t>(n) * static_cast<uint32_t>(n);
        o.voxelsCpu[idx] = packVoxel(kGroundMat, false);
        const uint32_t base = idx * static_cast<uint32_t>(kMicroWords);
        for (int i = 0; i < kMicroWords; ++i) {
          o.microCpu[base + static_cast<uint32_t>(i)] = 0xFFFFFFFFu;
        }
        o.voxelsCpu[idx] = packVoxel(kGroundMat, true);
      }
    }
  }
}

void VoxelScene::buildSpinnerObject(VoxelObject& o) const {
  constexpr int kSpinnerN = 8;
  o.gridSize = kSpinnerN;
  o.voxelSize = voxelSize_;
  o.nestedMicro = nestedMicroVoxels_;
  o.editable = true;
  o.enabled = spinnerEnabled_;
  o.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  // Hover above ground: ground top ~ 2*voxelSize, place cube center higher.
  o.position = glm::vec3(0.0f, 4.0f * voxelSize_ + 0.5f * static_cast<float>(kSpinnerN) * voxelSize_,
                         0.0f);

  const size_t count =
      static_cast<size_t>(kSpinnerN) * static_cast<size_t>(kSpinnerN) * static_cast<size_t>(kSpinnerN);
  o.voxelsCpu.assign(count, 0u);
  o.microCpu.assign(count * static_cast<size_t>(kMicroWords), 0u);

  constexpr uint32_t kSpinnerMat = 2u;
  // Hollow shell for a clearer rotating silhouette.
  for (int z = 0; z < kSpinnerN; ++z) {
    for (int y = 0; y < kSpinnerN; ++y) {
      for (int x = 0; x < kSpinnerN; ++x) {
        const bool shell = (x == 0 || y == 0 || z == 0 || x == kSpinnerN - 1 || y == kSpinnerN - 1 ||
                            z == kSpinnerN - 1);
        if (!shell) {
          continue;
        }
        const uint32_t idx =
            static_cast<uint32_t>(x) + static_cast<uint32_t>(y) * static_cast<uint32_t>(kSpinnerN) +
            static_cast<uint32_t>(z) * static_cast<uint32_t>(kSpinnerN) *
                static_cast<uint32_t>(kSpinnerN);
        o.voxelsCpu[idx] = packVoxel(kSpinnerMat, true);
        const uint32_t base = idx * static_cast<uint32_t>(kMicroWords);
        for (int i = 0; i < kMicroWords; ++i) {
          o.microCpu[base + static_cast<uint32_t>(i)] = kMicroTemplateWords[i];
        }
      }
    }
  }
}

void VoxelScene::rebuildVoxels(GfxDevice& gfx) {
  const int n = std::clamp(gridSize_, 8, 64);
  gridSize_ = n;
  maxSteps_ = static_cast<uint32_t>(std::max(16, n * 3));
  lastHit_.reset();

  objects_.clear();
  objects_.resize(2);
  buildGroundObject(objects_[0]);
  buildSpinnerObject(objects_[1]);

  packObjectPool();
  fillGpuObjectRecords();
  ensureGpuBuffers(gfx);

  const VkDeviceSize voxelBytes = sizeof(uint32_t) * voxelsCpu_.size();
  const VkDeviceSize microBytes = sizeof(uint32_t) * microCpu_.size();
  const VkDeviceSize objectBytes = sizeof(GpuVoxelObject) * objectsGpu_.size();
  gfx.uploadToBuffer(voxelBuffer_, voxelsCpu_.data(), voxelBytes);
  gfx.uploadToBuffer(microBuffer_, microCpu_.data(), microBytes);
  gfx.uploadToBuffer(objectBuffer_, objectsGpu_.data(), objectBytes);
}

int VoxelScene::applyCoarseSphereBrush(VoxelObject& o, const glm::ivec3& center, float radius,
                                       uint32_t material, bool placeOnlyEmpty) {
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
        if (!inBounds(o, p)) {
          continue;
        }
        if (placeOnlyEmpty && getVoxel(o, p) != 0) {
          continue;
        }
        if (setVoxelCpu(o, p, material)) {
          ++changed;
        }
      }
    }
  }
  return changed;
}

int VoxelScene::applyMicroSphereBrush(VoxelObject& o, const glm::ivec3& coarse,
                                      const glm::ivec3& micro, float radius, bool solid,
                                      uint32_t placeMaterial) {
  const float r = std::max(0.0f, radius);
  const int extent = static_cast<int>(std::ceil(r));
  const float r2 = r * r;
  int changed = 0;
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
        if (!inBounds(o, c) || !microInBounds(m)) {
          continue;
        }

        if (solid) {
          ensureCoarseBrick(o, c, placeMaterial);
          if (setMicroCpu(o, c, m, true)) {
            ++changed;
          }
        } else {
          if (getVoxel(o, c) == 0) {
            continue;
          }
          if (setMicroCpu(o, c, m, false)) {
            ++changed;
            const uint32_t idx = indexOf(o, c);
            if (microBrickEmpty(o, idx)) {
              o.voxelsCpu[idx] = 0;
              occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
            }
          }
        }
      }
    }
  }
  return changed;
}

std::optional<VoxelScene::PickResult> VoxelScene::pickObject(const VoxelObject& o, int objectIndex,
                                                            const glm::vec3& Ow,
                                                            const glm::vec3& Dw) const {
  if (!o.enabled || o.voxelsCpu.empty() || o.gridSize <= 0 || o.voxelSize <= 0.0f) {
    return std::nullopt;
  }

  const glm::mat4 w2o = o.worldToObject();
  const glm::vec3 Ol = glm::vec3(w2o * glm::vec4(Ow, 1.0f));
  const glm::vec3 Dl = glm::vec3(w2o * glm::vec4(Dw, 0.0f));
  if (glm::dot(Dl, Dl) < 1e-12f) {
    return std::nullopt;
  }

  const glm::vec3 ro = Ol / o.voxelSize;
  const glm::vec3 rd = Dl;  // do not renormalize
  const glm::vec3 invDir(safeInv(rd.x), safeInv(rd.y), safeInv(rd.z));
  const glm::vec3 sgn(rd.x >= 0.0f ? 1.0f : -1.0f, rd.y >= 0.0f ? 1.0f : -1.0f,
                      rd.z >= 0.0f ? 1.0f : -1.0f);

  const glm::vec3 boundsMax(static_cast<float>(o.gridSize));
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
      glm::clamp(glm::ivec3(glm::floor(pos)), glm::ivec3(0), glm::ivec3(o.gridSize - 1));
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

  const bool useNested = o.nestedMicro && nestedMicroVoxels_;

  auto makeHit = [&](const glm::ivec3& cell, const glm::ivec3& micro, uint32_t mat, glm::bvec3 msk,
                     bool hasMicro, float tLocal) -> PickResult {
    // Local hit point in meters, then to world for comparable t.
    const glm::vec3 hitLocalMeters = (ro + rd * tLocal) * o.voxelSize;
    const glm::vec3 hitWorld = glm::vec3(o.objectToWorld() * glm::vec4(hitLocalMeters, 1.0f));
    const float tWorld = glm::dot(hitWorld - Ow, Dw);

    VoxelHit hit{};
    hit.cell = cell;
    hit.micro = micro;
    hit.material = mat;
    hit.normal = glm::ivec3(-glm::vec3(msk) * sgn);
    hit.hasMicro = hasMicro;
    hit.objectIndex = objectIndex;
    return PickResult{hit, tWorld};
  };

  for (uint32_t i = 0; i < std::max(maxSteps_, 1u); ++i) {
    if (!inBounds(o, mapPos)) {
      break;
    }

    const uint32_t packed = o.voxelsCpu[indexOf(o, mapPos)];
    const uint32_t mat = materialOf(packed);
    if (mat != 0u) {
      if (!useNested) {
        float tHit = tEnter;
        if (mapPos != startPos) {
          tHit = glm::dot(sideDist - deltaDist, glm::vec3(mask));
        }
        return makeHit(mapPos, glm::ivec3(0), mat, mask, false, tHit);
      }

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

        auto microT = [&](const glm::ivec3& /*m*/, const glm::bvec3& msk,
                          const glm::vec3& side) -> float {
          // Approximate: coarse entry + micro face t in micro units / 8.
          float tCoarse = tEnter;
          if (mapPos != startPos) {
            const glm::vec3 mini = ((glm::vec3(mapPos) - ro) + 0.5f - 0.5f * sgn) * invDir;
            tCoarse = std::max(mini.x, std::max(mini.y, mini.z));
          }
          const float tMicroLocal = glm::dot(side - deltaDist, glm::vec3(msk));
          return tCoarse + tMicroLocal / 8.0f;
        };

        if (getMicro(o, mapPos, microPos)) {
          return makeHit(mapPos, microPos, mat, microMask, true,
                         microT(microPos, microMask, microSide + deltaDist));
        }

        for (int s = 0; s < 32; ++s) {
          microMask = stepMaskCpu(microSide);
          microSide += glm::vec3(microMask) * deltaDist;
          microPos += glm::ivec3(glm::vec3(microMask)) * rayStep;
          if (!microInBounds(microPos)) {
            break;
          }
          if (getMicro(o, mapPos, microPos)) {
            return makeHit(mapPos, microPos, mat, microMask, true,
                           microT(microPos, microMask, microSide));
          }
        }
      }
    }

    mask = stepMaskCpu(sideDist);
    sideDist += glm::vec3(mask) * deltaDist;
    mapPos += glm::ivec3(glm::vec3(mask)) * rayStep;
  }
  return std::nullopt;
}

std::optional<VoxelHit> VoxelScene::pickCenterRay() const {
  if (objects_.empty()) {
    return std::nullopt;
  }
  const glm::vec3 Ow = camera_.position();
  const glm::vec3 Dw = glm::normalize(camera_.forward());
  if (glm::dot(Dw, Dw) < 1e-12f) {
    return std::nullopt;
  }

  std::optional<PickResult> best;
  for (size_t i = 0; i < objects_.size(); ++i) {
    auto hit = pickObject(objects_[i], static_cast<int>(i), Ow, Dw);
    if (!hit.has_value()) {
      continue;
    }
    if (!best.has_value() || hit->tWorld < best->tWorld) {
      best = hit;
    }
  }
  if (!best.has_value()) {
    return std::nullopt;
  }
  return best->hit;
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
  const int objIndex = hit->objectIndex;
  if (objIndex < 0 || objIndex >= static_cast<int>(objects_.size())) {
    return;
  }
  VoxelObject& o = objects_[static_cast<size_t>(objIndex)];
  if (!o.editable || !o.enabled) {
    return;
  }

  const float radius = std::clamp(brushRadius_, 0.0f, 16.0f);
  brushRadius_ = radius;
  const uint32_t mat = static_cast<uint32_t>(std::clamp(brushMaterial_, 1, 255));

  int changed = 0;
  if (nestedMicroVoxels_ && hit->hasMicro) {
    if (removeEdge) {
      changed = applyMicroSphereBrush(o, hit->cell, hit->micro, radius, false, mat);
    } else if (placeEdge) {
      glm::ivec3 placeMicro = hit->micro + hit->normal;
      glm::ivec3 placeCoarse = hit->cell;
      for (int a = 0; a < 3; ++a) {
        if (placeMicro[a] < 0) {
          placeMicro[a] = kMicroRes - 1;
          placeCoarse[a] -= 1;
        } else if (placeMicro[a] >= kMicroRes) {
          placeMicro[a] = 0;
          placeCoarse[a] += 1;
        }
      }
      if (inBounds(o, placeCoarse) && microInBounds(placeMicro)) {
        changed = applyMicroSphereBrush(o, placeCoarse, placeMicro, radius, true, mat);
      }
    }
  } else {
    if (removeEdge) {
      changed = applyCoarseSphereBrush(o, hit->cell, radius, 0, false);
    } else if (placeEdge) {
      changed = applyCoarseSphereBrush(o, hit->cell + hit->normal, radius, mat, true);
    }
  }

  if (changed > 0) {
    flushObject(gfx, objIndex);
  }
  lastHit_ = pickCenterRay();
}
