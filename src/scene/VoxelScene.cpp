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
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
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

constexpr uint32_t kMicroTemplateWords[16] = {
    0x818181ffu, 0xff818181u, 0x00004281u, 0x81420000u, 0x00240081u, 0x81002400u,
    0x18000081u, 0x81000018u, 0x18000081u, 0x81000018u, 0x00240081u, 0x81002400u,
    0x00004281u, 0x81420000u, 0x818181ffu, 0xff818181u,
};

constexpr uint32_t kSolidBrickWords[16] = {
    0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
    0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
    0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu,
};

constexpr uint32_t kEmptyBrickWords[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint32_t occMipAxis(uint32_t n) {
  return (n + static_cast<uint32_t>(VoxelScene::kOccMipRes) - 1u) /
         static_cast<uint32_t>(VoxelScene::kOccMipRes);
}

uint32_t occMipWordCount(uint32_t n) {
  const uint32_t axis = occMipAxis(n);
  const uint32_t bits = axis * axis * axis;
  return std::max(1u, (bits + 31u) / 32u);
}

}  // namespace

glm::mat4 VoxelObject::objectToWorld() const {
  const float extent = static_cast<float>(gridSize) * voxelSize;
  const glm::vec3 centerLocal(0.5f * extent, 0.5f * extent, 0.5f * extent);
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

  const std::string pirateObj =
      std::string(VE_ASSETS_DIR) + "/meshes/pirate-building/Piratebuilding.obj";
  if (std::filesystem::exists(pirateObj)) {
    MeshVoxelizeConfig cfg;
    cfg.gridN = importGridN_;
    cfg.padding = importPadding_;
    cfg.sampleColor = false;
    cfg.conservative = importConservative_;
    importSurfaceMesh(gfx, pirateObj, cfg);
  }
}

void VoxelScene::cleanup(GfxDevice& gfx) {
  voxelizeGpu_.destroy(gfx);
  gfx.destroyBuffer(voxelBuffer_);
  gfx.destroyBuffer(dummyBrickSlabBuffer_);
  gfx.destroyBuffer(objectBuffer_);
  gfx.destroyBuffer(paletteBuffer_);
  gfx.destroyBuffer(occMipBuffer_);
  for (BrickSlab& s : slabs_) {
    gfx.destroyBuffer(s.gpu);
  }
  TextureFactory::destroy(gfx, sky_);
  objects_.clear();
  objectsGpu_.clear();
  voxelsCpu_.clear();
  occMipCpu_.clear();
  slabs_.clear();
  freePages_.clear();
  dirtyPages_.clear();
  nextPage_ = 0;
  allocatedPageCount_ = 0;
  occupiedCount_ = 0;
  occupiedMicroCount_ = 0;
  occupiedFineCount_ = 0;
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
  return glm::vec3(objects_[0].objectToWorld() * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f));
}

bool VoxelScene::inBounds(const VoxelObject& o, const glm::ivec3& p) const {
  return p.x >= 0 && p.y >= 0 && p.z >= 0 && p.x < o.gridSize && p.y < o.gridSize &&
         p.z < o.gridSize;
}

bool VoxelScene::microInBounds(const glm::ivec3& m) const {
  return m.x >= 0 && m.y >= 0 && m.z >= 0 && m.x < kMicroRes && m.y < kMicroRes && m.z < kMicroRes;
}

bool VoxelScene::fineInBounds(const glm::ivec3& f) const {
  return f.x >= 0 && f.y >= 0 && f.z >= 0 && f.x < kFineRes && f.y < kFineRes && f.z < kFineRes;
}

uint32_t VoxelScene::indexOf(const VoxelObject& o, const glm::ivec3& p) const {
  const uint32_t n = static_cast<uint32_t>(o.gridSize);
  return static_cast<uint32_t>(p.x) + static_cast<uint32_t>(p.y) * n +
         static_cast<uint32_t>(p.z) * n * n;
}

uint32_t VoxelScene::microBitIndex(const glm::ivec3& m) const {
  return static_cast<uint32_t>(m.y * 64 + m.z * 8 + m.x);
}

uint32_t VoxelScene::fineBitIndex(const glm::ivec3& f) const {
  return static_cast<uint32_t>(f.y * 4 + f.z * 2 + f.x);
}

CoarseCell& VoxelScene::cellAt(VoxelObject& o, uint32_t idx) {
  return o.cells[idx];
}

const CoarseCell& VoxelScene::cellAt(const VoxelObject& o, uint32_t idx) const {
  return o.cells[idx];
}

uint32_t VoxelScene::getVoxel(const VoxelObject& o, const glm::ivec3& p) const {
  if (!inBounds(o, p) || o.cells.empty()) {
    return 0;
  }
  return cellAt(o, indexOf(o, p)).material;
}

void VoxelScene::ensureSlabCpu(uint32_t slabIndex) {
  while (slabs_.size() <= slabIndex) {
    if (slabs_.size() >= kMaxBrickSlabs) {
      throw std::runtime_error("Brick slab limit reached");
    }
    BrickSlab s;
    s.words.assign(kWordsPerSlab, 0u);
    slabs_.push_back(std::move(s));
  }
}

uint32_t* VoxelScene::brickPageWords(uint32_t page) {
  const uint32_t si = page / kPagesPerSlab;
  const uint32_t li = page % kPagesPerSlab;
  ensureSlabCpu(si);
  return slabs_[si].words.data() + static_cast<size_t>(li) * static_cast<size_t>(kBrickPageWords);
}

const uint32_t* VoxelScene::brickPageWords(uint32_t page) const {
  const uint32_t si = page / kPagesPerSlab;
  const uint32_t li = page % kPagesPerSlab;
  return slabs_[si].words.data() + static_cast<size_t>(li) * static_cast<size_t>(kBrickPageWords);
}

uint8_t VoxelScene::readFineByte(uint32_t page, uint32_t microBit) const {
  const uint32_t packed = brickPageWords(page)[kMicroWords + microBit / 4u];
  return static_cast<uint8_t>((packed >> ((microBit % 4u) * 8u)) & 255u);
}

void VoxelScene::writeFineByte(uint32_t page, uint32_t microBit, uint8_t value) {
  uint32_t* w = brickPageWords(page);
  const uint32_t idx = static_cast<uint32_t>(kMicroWords) + microBit / 4u;
  const uint32_t shift = (microBit % 4u) * 8u;
  w[idx] = (w[idx] & ~(0xFFu << shift)) | (static_cast<uint32_t>(value) << shift);
}

void VoxelScene::fillFineFromOccupancy(uint32_t page) {
  const uint32_t* occ = brickPageWords(page);
  for (int i = 0; i < kMicroCount; ++i) {
    const uint32_t word = static_cast<uint32_t>(i) / 32u;
    const uint32_t mask = 1u << (static_cast<uint32_t>(i) % 32u);
    writeFineByte(page, static_cast<uint32_t>(i), (occ[word] & mask) ? 0xFFu : 0x00u);
  }
}

bool VoxelScene::brickPageEmpty(uint32_t page) const {
  if (page == kInvalidBrickPage) {
    return true;
  }
  const uint32_t si = page / kPagesPerSlab;
  if (si >= slabs_.size()) {
    return true;
  }
  const uint32_t* w = brickPageWords(page);
  for (int i = 0; i < kMicroWords; ++i) {
    if (w[i] != 0u) {
      return false;
    }
  }
  return true;
}

void VoxelScene::recountOccupiedMicro() {
  occupiedMicroCount_ = 0;
  std::vector<uint8_t> free(nextPage_, 0);
  for (uint32_t p : freePages_) {
    if (p < nextPage_) {
      free[p] = 1;
    }
  }
  for (uint32_t p = 0; p < nextPage_; ++p) {
    if (free[p]) {
      continue;
    }
    const uint32_t* w = brickPageWords(p);
    for (int i = 0; i < kMicroWords; ++i) {
      occupiedMicroCount_ += static_cast<uint32_t>(std::popcount(w[i]));
    }
  }
}

void VoxelScene::recountOccupiedFine() {
  occupiedFineCount_ = 0;
  std::vector<uint8_t> freeP(nextPage_, 0);
  for (uint32_t p : freePages_) {
    if (p < nextPage_) {
      freeP[p] = 1;
    }
  }
  for (uint32_t p = 0; p < nextPage_; ++p) {
    if (freeP[p]) {
      continue;
    }
    for (int i = 0; i < kFineTableBytes; ++i) {
      occupiedFineCount_ +=
          static_cast<uint32_t>(std::popcount(static_cast<unsigned>(readFineByte(p, static_cast<uint32_t>(i)))));
    }
  }
}

uint32_t VoxelScene::allocBrickPage(const uint32_t* words16) {
  uint32_t page = kInvalidBrickPage;
  if (!freePages_.empty()) {
    page = freePages_.back();
    freePages_.pop_back();
  } else {
    page = nextPage_++;
    if (page / kPagesPerSlab >= kMaxBrickSlabs) {
      --nextPage_;
      throw std::runtime_error("Brick slab limit reached");
    }
    ensureSlabCpu(page / kPagesPerSlab);
  }
  uint32_t* dst = brickPageWords(page);
  for (int i = 0; i < kMicroWords; ++i) {
    dst[i] = words16[i];
  }
  fillFineFromOccupancy(page);
  dirtyPages_.insert(page);
  ++allocatedPageCount_;
  return page;
}

void VoxelScene::freeBrickPage(uint32_t page) {
  if (page == kInvalidBrickPage) {
    return;
  }
  const uint32_t si = page / kPagesPerSlab;
  if (si >= slabs_.size()) {
    return;
  }
  uint32_t* w = brickPageWords(page);
  for (int i = 0; i < kBrickPageWords; ++i) {
    w[i] = 0u;
  }
  freePages_.push_back(page);
  dirtyPages_.insert(page);
  if (allocatedPageCount_ > 0) {
    --allocatedPageCount_;
  }
}

uint32_t VoxelScene::ensureBrickPage(VoxelObject& o, uint32_t coarseIndex, bool fillSolid) {
  CoarseCell& c = o.cells[coarseIndex];
  if (c.brickPage != kInvalidBrickPage) {
    return c.brickPage;
  }
  c.brickPage = allocBrickPage(fillSolid ? kSolidBrickWords : kEmptyBrickWords);
  return c.brickPage;
}

bool VoxelScene::getMicro(const VoxelObject& o, const glm::ivec3& coarse,
                          const glm::ivec3& micro) const {
  if (!inBounds(o, coarse) || !microInBounds(micro) || o.cells.empty()) {
    return false;
  }
  const CoarseCell& c = cellAt(o, indexOf(o, coarse));
  if (c.material == 0u) {
    return false;
  }
  if (c.brickPage == kInvalidBrickPage) {
    return true;  // uniform solid coarse
  }
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t mask = 1u << (bit % 32u);
  const uint32_t si = c.brickPage / kPagesPerSlab;
  if (si >= slabs_.size()) {
    return false;
  }
  const uint32_t* w = brickPageWords(c.brickPage);
  return (w[word] & mask) != 0u;
}

bool VoxelScene::getFine(const VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                         const glm::ivec3& fine) const {
  if (!fineInBounds(fine) || !getMicro(o, coarse, micro)) {
    return false;
  }
  const CoarseCell& c = cellAt(o, indexOf(o, coarse));
  if (c.brickPage == kInvalidBrickPage) {
    return true;
  }
  const uint8_t byte = readFineByte(c.brickPage, microBitIndex(micro));
  return (byte & (1u << fineBitIndex(fine))) != 0u;
}

bool VoxelScene::setVoxelCpu(VoxelObject& o, const glm::ivec3& p, uint32_t material) {
  if (!inBounds(o, p) || o.cells.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(o, p);
  CoarseCell& c = o.cells[idx];
  const uint32_t newMat = material;
  if (c.material == newMat && (newMat == 0u || c.brickPage == kInvalidBrickPage)) {
    return false;
  }
  if (c.material == 0 && newMat != 0) {
    ++occupiedCount_;
    freeBrickPage(c.brickPage);
    c.material = newMat;
    c.brickPage = kInvalidBrickPage;
  } else if (c.material != 0 && newMat == 0) {
    occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
    freeBrickPage(c.brickPage);
    c.material = 0;
    c.brickPage = kInvalidBrickPage;
  } else {
    c.material = newMat;
  }
  return true;
}

bool VoxelScene::setMicroCpu(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                             bool solid) {
  if (!inBounds(o, coarse) || !microInBounds(micro) || o.cells.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(o, coarse);
  CoarseCell& c = o.cells[idx];
  if (c.material == 0u && !solid) {
    return false;
  }

  const bool wasUniformSolid = (c.material != 0u && c.brickPage == kInvalidBrickPage);
  if (wasUniformSolid && solid) {
    return false;
  }
  if (wasUniformSolid && !solid) {
    ensureBrickPage(o, idx, true);
  }
  if (c.material != 0u && c.brickPage == kInvalidBrickPage && solid) {
    return false;
  }
  if (c.brickPage == kInvalidBrickPage && solid) {
    return false;
  }

  const uint32_t page = c.brickPage;
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t mask = 1u << (bit % 32u);
  uint32_t& dst = brickPageWords(page)[word];
  const bool was = (dst & mask) != 0u;
  if (was == solid) {
    return false;
  }
  if (solid) {
    dst |= mask;
    writeFineByte(page, bit, 0xFFu);
    ++occupiedMicroCount_;
  } else {
    dst &= ~mask;
    writeFineByte(page, bit, 0u);
    occupiedMicroCount_ = occupiedMicroCount_ > 0 ? occupiedMicroCount_ - 1 : 0;
  }
  dirtyPages_.insert(page);

  if (brickPageEmpty(page)) {
    freeBrickPage(page);
    c.brickPage = kInvalidBrickPage;
    c.material = 0;
    occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
  }
  return true;
}

bool VoxelScene::setFineCpu(VoxelObject& o, const glm::ivec3& coarse, const glm::ivec3& micro,
                            const glm::ivec3& fine, bool solid) {
  if (!inBounds(o, coarse) || !microInBounds(micro) || !fineInBounds(fine) || o.cells.empty()) {
    return false;
  }
  const uint32_t idx = indexOf(o, coarse);
  CoarseCell& c = o.cells[idx];
  if (c.material == 0u && !solid) {
    return false;
  }

  const bool wasUniformSolid = (c.material != 0u && c.brickPage == kInvalidBrickPage);
  if (wasUniformSolid && solid) {
    return false;
  }
  if (wasUniformSolid && !solid) {
    ensureBrickPage(o, idx, true);
  }
  if (c.brickPage == kInvalidBrickPage) {
    return false;
  }

  const uint32_t page = c.brickPage;
  const uint32_t bit = microBitIndex(micro);
  const uint32_t word = bit / 32u;
  const uint32_t omask = 1u << (bit % 32u);
  uint32_t& occ = brickPageWords(page)[word];
  const bool microWas = (occ & omask) != 0u;
  if (!microWas && !solid) {
    return false;
  }
  uint8_t byte = readFineByte(page, bit);
  if (microWas && solid && byte == 0xFFu) {
    return false;
  }

  const uint8_t fmask = static_cast<uint8_t>(1u << fineBitIndex(fine));
  const bool was = (byte & fmask) != 0u;
  if (was == solid) {
    return false;
  }
  if (solid) {
    byte = static_cast<uint8_t>(byte | fmask);
    writeFineByte(page, bit, byte);
    if (!microWas) {
      occ |= omask;
      ++occupiedMicroCount_;
    }
  } else {
    byte = static_cast<uint8_t>(byte & ~fmask);
    writeFineByte(page, bit, byte);
    if (byte == 0u && microWas) {
      occ &= ~omask;
      occupiedMicroCount_ = occupiedMicroCount_ > 0 ? occupiedMicroCount_ - 1 : 0;
    }
  }
  dirtyPages_.insert(page);

  if (brickPageEmpty(page)) {
    freeBrickPage(page);
    c.brickPage = kInvalidBrickPage;
    c.material = 0;
    occupiedCount_ = occupiedCount_ > 0 ? occupiedCount_ - 1 : 0;
  }
  return true;
}

void VoxelScene::ensureCoarseBrick(VoxelObject& o, const glm::ivec3& coarse, uint32_t material) {
  if (!inBounds(o, coarse)) {
    return;
  }
  const uint32_t idx = indexOf(o, coarse);
  CoarseCell& c = o.cells[idx];
  if (c.material == 0u) {
    ++occupiedCount_;
    c.material = material;
    c.brickPage = allocBrickPage(kEmptyBrickWords);
  }
}

void VoxelScene::packObjectPool() {
  voxelsCpu_.clear();
  occMipCpu_.clear();
  occupiedCount_ = 0;
  for (VoxelObject& o : objects_) {
    o.voxelOffset = static_cast<uint32_t>(voxelsCpu_.size());
    voxelsCpu_.insert(voxelsCpu_.end(), o.cells.begin(), o.cells.end());
    for (const CoarseCell& c : o.cells) {
      if (c.material != 0u) {
        ++occupiedCount_;
      }
    }
    o.occMipOffset = static_cast<uint32_t>(occMipCpu_.size());
    o.occMipWords = occMipWordCount(static_cast<uint32_t>(std::max(o.gridSize, 1)));
    occMipCpu_.resize(o.occMipOffset + o.occMipWords, 0u);
    fillOccMip(o, occMipCpu_.data() + o.occMipOffset, o.occMipWords);
  }
  recountOccupiedMicro();
  recountOccupiedFine();
}

void VoxelScene::fillOccMip(const VoxelObject& o, uint32_t* words, uint32_t wordCount) const {
  if (!words || wordCount == 0) {
    return;
  }
  std::fill(words, words + wordCount, 0u);
  const int n = o.gridSize;
  if (n <= 0) {
    return;
  }
  const uint32_t macroN = occMipAxis(static_cast<uint32_t>(n));
  const size_t expected =
      static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n);
  if (o.cells.size() != expected) {
    return;
  }
  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        const size_t idx = static_cast<size_t>(x) + static_cast<size_t>(y) * static_cast<size_t>(n) +
                           static_cast<size_t>(z) * static_cast<size_t>(n) * static_cast<size_t>(n);
        if (o.cells[idx].material == 0u) {
          continue;
        }
        const uint32_t mx = static_cast<uint32_t>(x) >> kOccMipShift;
        const uint32_t my = static_cast<uint32_t>(y) >> kOccMipShift;
        const uint32_t mz = static_cast<uint32_t>(z) >> kOccMipShift;
        const uint32_t bit = mx + my * macroN + mz * macroN * macroN;
        const uint32_t wi = bit >> 5;
        if (wi < wordCount) {
          words[wi] |= 1u << (bit & 31u);
        }
      }
    }
  }
}

void VoxelScene::uploadOccMip(GfxDevice& gfx) {
  if (occMipBuffer_.buffer == VK_NULL_HANDLE) {
    return;
  }
  if (occMipCpu_.empty()) {
    uint32_t zero = 0;
    gfx.uploadToBuffer(occMipBuffer_, &zero, sizeof(uint32_t));
    return;
  }
  gfx.uploadToBuffer(occMipBuffer_, occMipCpu_.data(), sizeof(uint32_t) * occMipCpu_.size());
}

void VoxelScene::fillGpuObjectRecords() {
  objectsGpu_.clear();
  objectsGpu_.reserve(objects_.size());
  for (const VoxelObject& o : objects_) {
    if (!o.enabled) {
      continue;
    }
    GpuVoxelObject g{};
    writeMat4(g.worldToObject, o.worldToObject());
    writeMat4(g.objectToWorld, o.objectToWorld());
    g.voxelSize = o.voxelSize;
    g._pad0[0] = g._pad0[1] = g._pad0[2] = 0.0f;
    g.gridSize[0] = static_cast<uint32_t>(o.gridSize);
    g.gridSize[1] = static_cast<uint32_t>(o.gridSize);
    g.gridSize[2] = static_cast<uint32_t>(o.gridSize);
    g.flags = VoxelObject::kFlagEnabled;
    if (o.nestedMicro) {
      g.flags |= VoxelObject::kFlagNestedMicro;
    }
    if (o.useImportPalette) {
      g.flags |= VoxelObject::kFlagImportPalette;
    }
    g.voxelOffset = o.voxelOffset;
    g.occMipOffset = o.occMipOffset;
    g.occMipWords = o.occMipWords;
    g._pad1 = 0;
    objectsGpu_.push_back(g);
  }
}

void VoxelScene::ensureGpuBuffers(GfxDevice& gfx) {
  const VkDeviceSize voxelBytes = sizeof(CoarseCell) * std::max<size_t>(voxelsCpu_.size(), 1);
  const VkDeviceSize slabBytes = sizeof(uint32_t) * kWordsPerSlab;
  const VkDeviceSize objectBytes =
      sizeof(GpuVoxelObject) * std::max<size_t>(std::max(objectsGpu_.size(), objects_.size()), 1);

  if (voxelBuffer_.buffer == VK_NULL_HANDLE || voxelBuffer_.size < voxelBytes) {
    gfx.destroyBuffer(voxelBuffer_);
    voxelBuffer_ = gfx.createBuffer(voxelBytes,
                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                    VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }
  if (dummyBrickSlabBuffer_.buffer == VK_NULL_HANDLE || dummyBrickSlabBuffer_.size < slabBytes) {
    gfx.destroyBuffer(dummyBrickSlabBuffer_);
    dummyBrickSlabBuffer_ = gfx.createBuffer(slabBytes,
                                             VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                             VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
    std::vector<uint32_t> zeros(kWordsPerSlab, 0u);
    gfx.uploadToBuffer(dummyBrickSlabBuffer_, zeros.data(), slabBytes);
  }
  for (BrickSlab& s : slabs_) {
    if (s.gpu.buffer == VK_NULL_HANDLE) {
      s.gpu = gfx.createBuffer(slabBytes,
                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                               VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
      gfx.uploadToBuffer(s.gpu, s.words.data(), slabBytes);
    }
  }
  if (objectBuffer_.buffer == VK_NULL_HANDLE || objectBuffer_.size < objectBytes) {
    gfx.destroyBuffer(objectBuffer_);
    objectBuffer_ = gfx.createBuffer(objectBytes,
                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                     VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }
  const VkDeviceSize paletteBytes = sizeof(glm::vec4) * 256;
  if (paletteBuffer_.buffer == VK_NULL_HANDLE || paletteBuffer_.size < paletteBytes) {
    gfx.destroyBuffer(paletteBuffer_);
    paletteBuffer_ = gfx.createBuffer(
        paletteBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
    uploadPalette(gfx);
  }
  const VkDeviceSize mipBytes = sizeof(uint32_t) * std::max<size_t>(occMipCpu_.size(), 1);
  if (occMipBuffer_.buffer == VK_NULL_HANDLE || occMipBuffer_.size < mipBytes) {
    gfx.destroyBuffer(occMipBuffer_);
    occMipBuffer_ = gfx.createBuffer(
        mipBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  }
}

void VoxelScene::uploadPalette(GfxDevice& gfx) {
  if (paletteBuffer_.buffer == VK_NULL_HANDLE) {
    return;
  }
  gfx.uploadToBuffer(paletteBuffer_, importPalette_.data(), sizeof(glm::vec4) * 256);
}

void VoxelScene::flushDirtyPages(GfxDevice& gfx) {
  for (uint32_t page : dirtyPages_) {
    const uint32_t si = page / kPagesPerSlab;
    const uint32_t li = page % kPagesPerSlab;
    if (si >= slabs_.size() || slabs_[si].gpu.buffer == VK_NULL_HANDLE) {
      continue;
    }
    const uint32_t* src = brickPageWords(page);
    gfx.uploadToBuffer(slabs_[si].gpu, src, sizeof(uint32_t) * static_cast<size_t>(kBrickPageWords),
                       sizeof(uint32_t) * static_cast<VkDeviceSize>(li) *
                           static_cast<VkDeviceSize>(kBrickPageWords));
  }
  dirtyPages_.clear();
}

void VoxelScene::flushObject(GfxDevice& gfx, int objectIndex) {
  if (objectIndex < 0 || objectIndex >= static_cast<int>(objects_.size())) {
    return;
  }
  VoxelObject& o = objects_[static_cast<size_t>(objectIndex)];
  if (!o.cells.empty() && o.voxelOffset + o.cells.size() <= voxelsCpu_.size()) {
    std::copy(o.cells.begin(), o.cells.end(), voxelsCpu_.begin() + o.voxelOffset);
  }
  if (o.occMipWords > 0 && o.occMipOffset + o.occMipWords <= occMipCpu_.size()) {
    fillOccMip(o, occMipCpu_.data() + o.occMipOffset, o.occMipWords);
  }
  ensureGpuBuffers(gfx);
  if (!o.cells.empty() && voxelBuffer_.buffer != VK_NULL_HANDLE) {
    gfx.uploadToBuffer(voxelBuffer_, o.cells.data(), sizeof(CoarseCell) * o.cells.size(),
                       sizeof(CoarseCell) * static_cast<VkDeviceSize>(o.voxelOffset));
  }
  if (o.occMipWords > 0 && occMipBuffer_.buffer != VK_NULL_HANDLE) {
    gfx.uploadToBuffer(occMipBuffer_, occMipCpu_.data() + o.occMipOffset,
                       sizeof(uint32_t) * o.occMipWords,
                       sizeof(uint32_t) * static_cast<VkDeviceSize>(o.occMipOffset));
  }
  flushDirtyPages(gfx);
}

void VoxelScene::uploadObjectTransforms(GfxDevice& gfx) {
  fillGpuObjectRecords();
  if (objectsGpu_.empty()) {
    return;
  }
  ensureGpuBuffers(gfx);
  gfx.uploadToBuffer(objectBuffer_, objectsGpu_.data(),
                     sizeof(GpuVoxelObject) * objectsGpu_.size());
}

void VoxelScene::buildGroundObject(VoxelObject& o) {
  const int n = std::clamp(gridSize_, 8, 64);
  o.gridSize = n;
  o.voxelSize = voxelSize_;
  o.nestedMicro = nestedMicroVoxels_;
  o.editable = true;
  o.enabled = true;
  o.useImportPalette = false;
  o.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  const float half = 0.5f * static_cast<float>(n);
  o.position = glm::vec3(0.0f, half * voxelSize_, 0.0f);

  const size_t count = static_cast<size_t>(n) * static_cast<size_t>(n) * static_cast<size_t>(n);
  o.cells.assign(count, CoarseCell{});

  constexpr uint32_t kGroundMat = 1u;
  const int groundThickness = 2;
  for (int z = 0; z < n; ++z) {
    for (int x = 0; x < n; ++x) {
      for (int y = 0; y < groundThickness; ++y) {
        const uint32_t idx =
            static_cast<uint32_t>(x) + static_cast<uint32_t>(y) * static_cast<uint32_t>(n) +
            static_cast<uint32_t>(z) * static_cast<uint32_t>(n) * static_cast<uint32_t>(n);
        o.cells[idx].material = kGroundMat;
        o.cells[idx].brickPage = kInvalidBrickPage;
      }
    }
  }
}

void VoxelScene::buildSpinnerObject(VoxelObject& o) {
  constexpr int kSpinnerN = 8;
  o.gridSize = kSpinnerN;
  o.voxelSize = voxelSize_;
  o.nestedMicro = nestedMicroVoxels_;
  o.editable = true;
  o.enabled = spinnerEnabled_;
  o.rotation = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  o.position = glm::vec3(0.0f, 4.0f * voxelSize_ + 0.5f * static_cast<float>(kSpinnerN) * voxelSize_,
                         0.0f);

  const size_t count =
      static_cast<size_t>(kSpinnerN) * static_cast<size_t>(kSpinnerN) * static_cast<size_t>(kSpinnerN);
  o.cells.assign(count, CoarseCell{});

  // Same material as ground so albedo stays unified.
  constexpr uint32_t kSpinnerMat = 1u;
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
        o.cells[idx].material = kSpinnerMat;
        const uint32_t page = allocBrickPage(kMicroTemplateWords);
        o.cells[idx].brickPage = page;
      }
    }
  }
}

void VoxelScene::rebuildVoxels(GfxDevice& gfx) {
  const int n = std::clamp(gridSize_, 8, 64);
  gridSize_ = n;
  maxSteps_ = static_cast<uint32_t>(std::max(16, n * 3));
  lastHit_.reset();

  for (BrickSlab& s : slabs_) {
    gfx.destroyBuffer(s.gpu);
  }
  slabs_.clear();
  freePages_.clear();
  dirtyPages_.clear();
  nextPage_ = 0;
  allocatedPageCount_ = 0;

  objects_.clear();
  objects_.resize(2);
  buildGroundObject(objects_[0]);
  buildSpinnerObject(objects_[1]);

  packObjectPool();
  fillGpuObjectRecords();
  ensureGpuBuffers(gfx);
  uploadWorldAndObjects(gfx);
  dirtyPages_.clear();

  if (!lastImportedPath_.empty()) {
    MeshVoxelizeConfig cfg;
    cfg.gridN = importGridN_;
    cfg.padding = importPadding_;
    cfg.sampleColor = importSampleColor_;
    cfg.conservative = importConservative_;
    importSurfaceMesh(gfx, lastImportedPath_, cfg);
  }
}

void VoxelScene::uploadWorldAndObjects(GfxDevice& gfx) {
  if (!voxelsCpu_.empty() && voxelBuffer_.buffer != VK_NULL_HANDLE) {
    gfx.uploadToBuffer(voxelBuffer_, voxelsCpu_.data(), sizeof(CoarseCell) * voxelsCpu_.size());
  }
  uploadOccMip(gfx);
  if (!objectsGpu_.empty() && objectBuffer_.buffer != VK_NULL_HANDLE) {
    gfx.uploadToBuffer(objectBuffer_, objectsGpu_.data(),
                       sizeof(GpuVoxelObject) * objectsGpu_.size());
  }
}

uint32_t VoxelScene::stampMeshIntoWorld(const MeshVoxelizeResult& r, bool sampleColor) {
  if (objects_.empty() || r.n <= 0 || r.material.empty()) {
    return 0;
  }
  VoxelObject& world = objects_[0];
  const int wn = world.gridSize;
  const float wvs = world.voxelSize;
  if (wn <= 0 || wvs <= 0.0f) {
    return 0;
  }

  const float hutExtent = static_cast<float>(r.n) * r.voxelSize;
  const glm::vec3 hutPos(0.0f, 0.5f * hutExtent + 2.5f * voxelSize_, 0.0f);
  const glm::vec3 hutOrigin = hutPos - glm::vec3(0.5f * hutExtent);
  const glm::vec3 worldOrigin =
      world.position - glm::vec3(0.5f * static_cast<float>(wn) * wvs);

  world.useImportPalette = sampleColor;
  importPalette_.fill(glm::vec4(0.62f, 0.64f, 0.68f, 1.0f));
  importPalette_[0] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
  if (sampleColor) {
    importPalette_[1] = glm::vec4(0.62f, 0.64f, 0.68f, 1.0f);
    for (uint32_t i = 1; i < 255; ++i) {
      importPalette_[i + 1] = glm::vec4(r.palette[i], 1.0f);
    }
  }

  uint32_t stamped = 0;
  const int n = r.n;
  for (int z = 0; z < n; ++z) {
    for (int y = 0; y < n; ++y) {
      for (int x = 0; x < n; ++x) {
        const size_t src = static_cast<size_t>(x) + static_cast<size_t>(y) * static_cast<size_t>(n) +
                           static_cast<size_t>(z) * static_cast<size_t>(n) * static_cast<size_t>(n);
        if (src >= r.material.size() || r.material[src] == 0u) {
          continue;
        }
        uint32_t mat = 1u;
        if (sampleColor) {
          mat = std::min(255u, r.material[src] + 1u);
        }
        const glm::vec3 hutMin = hutOrigin + glm::vec3(x, y, z) * r.voxelSize;
        const glm::vec3 hutMax = hutMin + glm::vec3(r.voxelSize);
        const glm::vec3 gmin = (hutMin - worldOrigin) / wvs;
        const glm::vec3 gmax = (hutMax - worldOrigin) / wvs;
        const int x0 = std::max(0, static_cast<int>(std::floor(gmin.x)));
        const int y0 = std::max(0, static_cast<int>(std::floor(gmin.y)));
        const int z0 = std::max(0, static_cast<int>(std::floor(gmin.z)));
        const int x1 = std::min(wn - 1, static_cast<int>(std::ceil(gmax.x) - 1.0f));
        const int y1 = std::min(wn - 1, static_cast<int>(std::ceil(gmax.y) - 1.0f));
        const int z1 = std::min(wn - 1, static_cast<int>(std::ceil(gmax.z) - 1.0f));
        if (x0 > x1 || y0 > y1 || z0 > z1) {
          continue;
        }
        for (int wz = z0; wz <= z1; ++wz) {
          for (int wy = y0; wy <= y1; ++wy) {
            for (int wx = x0; wx <= x1; ++wx) {
              const uint32_t idx = indexOf(world, glm::ivec3(wx, wy, wz));
              CoarseCell& c = world.cells[idx];
              if (c.brickPage != kInvalidBrickPage) {
                freeBrickPage(c.brickPage);
              }
              c.material = mat;
              c.brickPage = kInvalidBrickPage;
              ++stamped;
            }
          }
        }
      }
    }
  }
  return stamped;
}

bool VoxelScene::importSurfaceMesh(GfxDevice& gfx, const std::string& path,
                                   const MeshVoxelizeConfig& cfg) {
  if (objects_.empty()) {
    importStatus_ = "World grid is not ready";
    return false;
  }

  MeshVoxelizeResult r = voxelizeObjSurface(gfx, voxelizeGpu_, path, cfg);
  if (!r.ok) {
    importStatus_ = r.error;
    return false;
  }

  VoxelObject& world = objects_[0];
  for (CoarseCell& c : world.cells) {
    if (c.brickPage != kInvalidBrickPage) {
      freeBrickPage(c.brickPage);
    }
  }
  buildGroundObject(world);

  const uint32_t stamped = stampMeshIntoWorld(r, cfg.sampleColor);

  importPath_ = path;
  lastImportedPath_ = path;
  importGridN_ = cfg.gridN;
  importPadding_ = cfg.padding;
  importSampleColor_ = cfg.sampleColor;

  packObjectPool();
  fillGpuObjectRecords();
  ensureGpuBuffers(gfx);
  uploadWorldAndObjects(gfx);
  uploadPalette(gfx);

  importStatus_ = "Stamped into world  N=" + std::to_string(r.n) +
                  "  srcOcc=" + std::to_string(r.occupied) +
                  "  worldWrites=" + std::to_string(stamped);
  if (!r.warning.empty()) {
    importStatus_ += "  (" + r.warning + ")";
  }
  return true;
}

void VoxelScene::removeImportedMesh(GfxDevice& gfx) {
  if (lastImportedPath_.empty()) {
    importStatus_ = "No imported mesh";
    return;
  }
  lastImportedPath_.clear();
  rebuildVoxels(gfx);
  importStatus_ = "Removed imported mesh";
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
          }
        }
      }
    }
  }
  return changed;
}

int VoxelScene::applyFineSphereBrush(VoxelObject& o, const glm::ivec3& coarse,
                                     const glm::ivec3& micro, const glm::ivec3& fine, float radius,
                                     bool solid, uint32_t placeMaterial) {
  const float r = std::max(0.0f, radius);
  const int extent = static_cast<int>(std::ceil(r));
  const float r2 = r * r;
  int changed = 0;
  const glm::ivec3 absCenter =
      coarse * (kMicroRes * kFineRes) + micro * kFineRes + fine;

  auto divFloor = [](int a, int b) {
    int q = a / b;
    int rem = a % b;
    if (rem != 0 && ((rem < 0) != (b < 0))) {
      --q;
    }
    return q;
  };

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

        const glm::ivec3 absFine = absCenter + glm::ivec3(dx, dy, dz);
        const int cellStride = kMicroRes * kFineRes;
        glm::ivec3 c(divFloor(absFine.x, cellStride), divFloor(absFine.y, cellStride),
                     divFloor(absFine.z, cellStride));
        glm::ivec3 rem = absFine - c * cellStride;
        glm::ivec3 m(divFloor(rem.x, kFineRes), divFloor(rem.y, kFineRes),
                     divFloor(rem.z, kFineRes));
        glm::ivec3 f = rem - m * kFineRes;
        if (!inBounds(o, c) || !microInBounds(m) || !fineInBounds(f)) {
          continue;
        }

        if (solid) {
          ensureCoarseBrick(o, c, placeMaterial);
          if (setFineCpu(o, c, m, f, true)) {
            ++changed;
          }
        } else {
          if (getVoxel(o, c) == 0) {
            continue;
          }
          if (setFineCpu(o, c, m, f, false)) {
            ++changed;
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
  if (!o.enabled || o.cells.empty() || o.gridSize <= 0 || o.voxelSize <= 0.0f) {
    return std::nullopt;
  }

  const glm::mat4 w2o = o.worldToObject();
  const glm::vec3 Ol = glm::vec3(w2o * glm::vec4(Ow, 1.0f));
  const glm::vec3 Dl = glm::vec3(w2o * glm::vec4(Dw, 0.0f));
  if (glm::dot(Dl, Dl) < 1e-12f) {
    return std::nullopt;
  }

  const glm::vec3 ro = Ol / o.voxelSize;
  const glm::vec3 rd = Dl;
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

  auto makeHit = [&](const glm::ivec3& cell, const glm::ivec3& micro, const glm::ivec3& fine,
                     uint32_t mat, glm::bvec3 msk, bool hasMicro, bool hasFine,
                     float tLocal) -> PickResult {
    const glm::vec3 hitLocalMeters = (ro + rd * tLocal) * o.voxelSize;
    const glm::vec3 hitWorld = glm::vec3(o.objectToWorld() * glm::vec4(hitLocalMeters, 1.0f));
    const float tWorld = glm::dot(hitWorld - Ow, Dw);

    VoxelHit hit{};
    hit.cell = cell;
    hit.micro = micro;
    hit.fine = fine;
    hit.material = mat;
    hit.normal = glm::ivec3(-glm::vec3(msk) * sgn);
    hit.hasMicro = hasMicro;
    hit.hasFine = hasFine;
    hit.objectIndex = objectIndex;
    return PickResult{hit, tWorld};
  };

  for (uint32_t i = 0; i < std::max(maxSteps_, 1u); ++i) {
    if (!inBounds(o, mapPos)) {
      break;
    }

    const CoarseCell& cell = cellAt(o, indexOf(o, mapPos));
    const uint32_t mat = cell.material;
    if (mat != 0u) {
      if (!useNested) {
        float tHit = tEnter;
        if (mapPos != startPos) {
          tHit = glm::dot(sideDist - deltaDist, glm::vec3(mask));
        }
        return makeHit(mapPos, glm::ivec3(0), glm::ivec3(0), mat, mask, false, false, tHit);
      }

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

      auto microT = [&](const glm::bvec3& msk, const glm::vec3& side) -> float {
        float tCoarse = tEnter;
        if (mapPos != startPos) {
          const glm::vec3 mini = ((glm::vec3(mapPos) - ro) + 0.5f - 0.5f * sgn) * invDir;
          tCoarse = std::max(mini.x, std::max(mini.y, mini.z));
        }
        const float tMicroLocal = glm::dot(side - deltaDist, glm::vec3(msk));
        return tCoarse + tMicroLocal / 8.0f;
      };

      auto tryFine = [&](const glm::ivec3& mpos, glm::bvec3 enterMask, const glm::vec3& mLocal,
                         float tMicroEnter) -> std::optional<PickResult> {
        glm::vec3 fineLocal = glm::clamp((mLocal - glm::vec3(mpos)) * 2.0f, glm::vec3(0.0001f),
                                         glm::vec3(1.9999f));
        glm::ivec3 finePos = glm::ivec3(glm::floor(fineLocal));
        glm::vec3 fineSide =
            (sgn * (glm::vec3(finePos) - fineLocal) + (sgn * 0.5f + 0.5f)) * deltaDist;
        glm::bvec3 fineMask = enterMask;
        auto fineT = [&](const glm::bvec3& msk, const glm::vec3& side) -> float {
          const float tFineLocal = glm::dot(side - deltaDist, glm::vec3(msk));
          return tMicroEnter + tFineLocal / 16.0f;
        };
        if (getFine(o, mapPos, mpos, finePos)) {
          return makeHit(mapPos, mpos, finePos, mat, fineMask, true, true,
                         fineT(fineMask, fineSide + deltaDist));
        }
        for (int s = 0; s < 8; ++s) {
          fineMask = stepMaskCpu(fineSide);
          fineSide += glm::vec3(fineMask) * deltaDist;
          finePos += glm::ivec3(glm::vec3(fineMask)) * rayStep;
          if (!fineInBounds(finePos)) {
            break;
          }
          if (getFine(o, mapPos, mpos, finePos)) {
            return makeHit(mapPos, mpos, finePos, mat, fineMask, true, true,
                           fineT(fineMask, fineSide));
          }
        }
        return std::nullopt;
      };

      if (getMicro(o, mapPos, microPos)) {
        const float tMicroEnter = microT(microMask, microSide + deltaDist);
        if (auto h = tryFine(microPos, microMask, localPos, tMicroEnter)) {
          return h;
        }
      }

      for (int s = 0; s < 32; ++s) {
        microMask = stepMaskCpu(microSide);
        microSide += glm::vec3(microMask) * deltaDist;
        microPos += glm::ivec3(glm::vec3(microMask)) * rayStep;
        if (!microInBounds(microPos)) {
          break;
        }
        if (getMicro(o, mapPos, microPos)) {
          const float tMicroEnter = microT(microMask, microSide);
          glm::vec3 hitPos = localPos + rd * glm::dot(microSide - deltaDist, glm::vec3(microMask));
          if (auto h = tryFine(microPos, microMask, hitPos, tMicroEnter)) {
            return h;
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
    if (!objects_[i].enabled) {
      continue;
    }
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
  if (nestedMicroVoxels_ && (hit->hasFine || hit->hasMicro)) {
    glm::ivec3 fine = hit->fine;
    glm::ivec3 micro = hit->micro;
    glm::ivec3 coarse = hit->cell;
    if (removeEdge) {
      changed = applyFineSphereBrush(o, coarse, micro, fine, radius, false, mat);
    } else if (placeEdge) {
      glm::ivec3 placeFine = fine + hit->normal;
      glm::ivec3 placeMicro = micro;
      glm::ivec3 placeCoarse = coarse;
      for (int a = 0; a < 3; ++a) {
        if (placeFine[a] < 0) {
          placeFine[a] = kFineRes - 1;
          placeMicro[a] -= 1;
        } else if (placeFine[a] >= kFineRes) {
          placeFine[a] = 0;
          placeMicro[a] += 1;
        }
        if (placeMicro[a] < 0) {
          placeMicro[a] = kMicroRes - 1;
          placeCoarse[a] -= 1;
        } else if (placeMicro[a] >= kMicroRes) {
          placeMicro[a] = 0;
          placeCoarse[a] += 1;
        }
      }
      if (inBounds(o, placeCoarse) && microInBounds(placeMicro) && fineInBounds(placeFine)) {
        changed = applyFineSphereBrush(o, placeCoarse, placeMicro, placeFine, radius, true, mat);
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
    recountOccupiedMicro();
    recountOccupiedFine();
    flushObject(gfx, objIndex);
  }
  lastHit_ = pickCenterRay();
}
