#include "voxel/MeshVoxelizer.h"

#include "gfx/GfxDevice.h"
#include "gfx/Texture.h"

#include <glm/glm.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <stb_image.h>

#include <algorithm>
#include <bit>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {

struct GpuTri {
  float p0[4];
  float p1[4];
  float p2[4];
  float uv0uv1[4];
  float uv2pad[4];
  float kd[4];
};
static_assert(sizeof(GpuTri) == 96, "GpuTri std430 size");

struct VoxelizePC {
  float bmin[3];
  float voxelSize;
  int32_t N;
  int32_t conservative;
  int32_t sampleColor;
  int32_t triCount;
  int32_t subdiv;
  int32_t maxSamples = 0;
};
static_assert(sizeof(VoxelizePC) == 40, "VoxelizePC push-constant size");

constexpr uint32_t kMaxColorSamples = 8u * 1024u * 1024u;

void fail(const std::string& m) { throw std::runtime_error(m); }

std::string toLower(std::string s) {
  for (char& c : s) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return s;
}

bool loadPngRgba(const std::filesystem::path& texPath, Texture& albedo, GfxDevice& gfx) {
  int w = 0, h = 0, comp = 0;
  stbi_uc* pixels = stbi_load(texPath.string().c_str(), &w, &h, &comp, 4);
  if (!pixels || w <= 0 || h <= 0) {
    if (pixels) {
      stbi_image_free(pixels);
    }
    return false;
  }
  TextureFactory::destroy(gfx, albedo);
  albedo.image = gfx.createImage(
      {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1}, VK_FORMAT_R8G8B8A8_SRGB,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
  gfx.uploadToImage(albedo.image, pixels, static_cast<VkDeviceSize>(w * h * 4),
                    {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1});
  albedo.sampler = gfx.createSampler(VK_FILTER_NEAREST, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false);
  stbi_image_free(pixels);
  return true;
}

std::filesystem::path findSiblingAlbedo(const std::filesystem::path& objPath) {
  const std::filesystem::path dir = objPath.parent_path();
  std::vector<std::filesystem::path> images;
  std::error_code ec;
  for (const auto& ent : std::filesystem::directory_iterator(dir, ec)) {
    if (!ent.is_regular_file(ec)) {
      continue;
    }
    const std::string ext = toLower(ent.path().extension().string());
    if (ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".tga") {
      images.push_back(ent.path());
    }
  }
  for (const auto& p : images) {
    const std::string n = toLower(p.filename().string());
    if (n.find("diffuse") != std::string::npos || n.find("albedo") != std::string::npos) {
      return p;
    }
  }
  return images.empty() ? std::filesystem::path{} : images.front();
}

void fillOccupiedMaterials(const std::vector<uint32_t>& occ, uint32_t fallback,
                           MeshVoxelizeResult& out) {
  out.material.assign(occ.size(), 0u);
  out.occupied = 0;
  for (size_t i = 0; i < occ.size(); ++i) {
    if (occ[i] != 0u) {
      out.material[i] = fallback;
      ++out.occupied;
    }
  }
}

}  // namespace

void MeshVoxelizerGpu::ensure(GfxDevice& gfx) {
  if (ready_) {
    return;
  }
  const std::string shaderDir = VE_SHADER_DIR;
  VkShaderModule comp = gfx.loadShaderModule(shaderDir + "/voxelize_surface.comp.spv");

  VkDescriptorSetLayoutBinding bindings[4]{};
  bindings[0].binding = 0;
  bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[0].descriptorCount = 1;
  bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  bindings[1].binding = 1;
  bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[1].descriptorCount = 1;
  bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  bindings[2].binding = 2;
  bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[2].descriptorCount = 1;
  bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  bindings[3].binding = 3;
  bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  bindings[3].descriptorCount = 1;
  bindings[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  VkDescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  layoutInfo.bindingCount = 4;
  layoutInfo.pBindings = bindings;
  if (vkCreateDescriptorSetLayout(gfx.device(), &layoutInfo, nullptr, &setLayout_) != VK_SUCCESS) {
    vkDestroyShaderModule(gfx.device(), comp, nullptr);
    fail("Failed to create voxelize set layout");
  }

  VkPushConstantRange pc{};
  pc.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
  pc.offset = 0;
  pc.size = sizeof(VoxelizePC);

  VkPipelineLayoutCreateInfo pl{};
  pl.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  pl.setLayoutCount = 1;
  pl.pSetLayouts = &setLayout_;
  pl.pushConstantRangeCount = 1;
  pl.pPushConstantRanges = &pc;
  if (vkCreatePipelineLayout(gfx.device(), &pl, nullptr, &pipelineLayout_) != VK_SUCCESS) {
    vkDestroyShaderModule(gfx.device(), comp, nullptr);
    fail("Failed to create voxelize pipeline layout");
  }

  VkPipelineShaderStageCreateInfo stage{};
  stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  stage.module = comp;
  stage.pName = "main";

  VkComputePipelineCreateInfo pipe{};
  pipe.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipe.stage = stage;
  pipe.layout = pipelineLayout_;
  if (vkCreateComputePipelines(gfx.device(), VK_NULL_HANDLE, 1, &pipe, nullptr, &pipeline_) !=
      VK_SUCCESS) {
    vkDestroyShaderModule(gfx.device(), comp, nullptr);
    fail("Failed to create voxelize_surface pipeline");
  }
  vkDestroyShaderModule(gfx.device(), comp, nullptr);

  VkDescriptorPoolSize sizes[] = {
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 3},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1},
  };
  VkDescriptorPoolCreateInfo pool{};
  pool.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  pool.maxSets = 1;
  pool.poolSizeCount = 2;
  pool.pPoolSizes = sizes;
  if (vkCreateDescriptorPool(gfx.device(), &pool, nullptr, &pool_) != VK_SUCCESS) {
    fail("Failed to create voxelize descriptor pool");
  }
  ready_ = true;
}

void MeshVoxelizerGpu::destroy(GfxDevice& gfx) {
  if (pipeline_) {
    vkDestroyPipeline(gfx.device(), pipeline_, nullptr);
    pipeline_ = VK_NULL_HANDLE;
  }
  if (pipelineLayout_) {
    vkDestroyPipelineLayout(gfx.device(), pipelineLayout_, nullptr);
    pipelineLayout_ = VK_NULL_HANDLE;
  }
  if (pool_) {
    vkDestroyDescriptorPool(gfx.device(), pool_, nullptr);
    pool_ = VK_NULL_HANDLE;
  }
  if (setLayout_) {
    vkDestroyDescriptorSetLayout(gfx.device(), setLayout_, nullptr);
    setLayout_ = VK_NULL_HANDLE;
  }
  ready_ = false;
}

MeshVoxelizeResult MeshVoxelizerGpu::voxelizeObjSurface(GfxDevice& gfx, const std::string& path,
                                                        const MeshVoxelizeConfig& cfg) {
  MeshVoxelizeResult out;
  ensure(gfx);

  const int n = std::clamp(cfg.gridN, 8, 64);
  const int padding = std::max(0, cfg.padding);
  const int subdiv = std::clamp(cfg.fineSubdiv, 1, 16);
  if (n - 2 * padding < 1) {
    out.error = "gridN too small for padding";
    return out;
  }
  const int fineN = n * subdiv;

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string err;
  const std::string mtlDir = std::filesystem::path(path).parent_path().string();
  const char* mtlBase = mtlDir.empty() ? nullptr : mtlDir.c_str();
  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &err, path.c_str(), mtlBase, true)) {
    out.error = err.empty() ? ("Failed to parse OBJ: " + path) : err;
    return out;
  }
  if (!err.empty()) {
    out.warning = err;
  }
  if (attrib.vertices.size() < 9) {
    out.error = "OBJ has no triangles";
    return out;
  }

  glm::vec3 bmin(1e30f);
  glm::vec3 bmax(-1e30f);
  std::vector<GpuTri> gpuTris;
  gpuTris.reserve(1024);

  auto vert = [&](int idx) {
    return glm::vec3(attrib.vertices[static_cast<size_t>(3 * idx) + 0],
                     attrib.vertices[static_cast<size_t>(3 * idx) + 1],
                     attrib.vertices[static_cast<size_t>(3 * idx) + 2]);
  };

  for (const auto& shape : shapes) {
    size_t indexOffset = 0;
    for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
      const int fv = shape.mesh.num_face_vertices[f];
      if (fv != 3) {
        indexOffset += static_cast<size_t>(fv);
        continue;
      }
      const tinyobj::index_t i0 = shape.mesh.indices[indexOffset + 0];
      const tinyobj::index_t i1 = shape.mesh.indices[indexOffset + 1];
      const tinyobj::index_t i2 = shape.mesh.indices[indexOffset + 2];
      indexOffset += 3;

      const glm::vec3 p0 = vert(i0.vertex_index);
      const glm::vec3 p1 = vert(i1.vertex_index);
      const glm::vec3 p2 = vert(i2.vertex_index);
      bmin = glm::min(bmin, glm::min(p0, glm::min(p1, p2)));
      bmax = glm::max(bmax, glm::max(p0, glm::max(p1, p2)));

      glm::vec2 uv0(0.0f), uv1(0.0f), uv2(0.0f);
      if (i0.texcoord_index >= 0 && i1.texcoord_index >= 0 && i2.texcoord_index >= 0 &&
          !attrib.texcoords.empty()) {
        uv0 = glm::vec2(attrib.texcoords[static_cast<size_t>(2 * i0.texcoord_index) + 0],
                        attrib.texcoords[static_cast<size_t>(2 * i0.texcoord_index) + 1]);
        uv1 = glm::vec2(attrib.texcoords[static_cast<size_t>(2 * i1.texcoord_index) + 0],
                        attrib.texcoords[static_cast<size_t>(2 * i1.texcoord_index) + 1]);
        uv2 = glm::vec2(attrib.texcoords[static_cast<size_t>(2 * i2.texcoord_index) + 0],
                        attrib.texcoords[static_cast<size_t>(2 * i2.texcoord_index) + 1]);
      }

      glm::vec3 kd(0.62f, 0.64f, 0.68f);
      int matId = -1;
      if (f < shape.mesh.material_ids.size()) {
        matId = shape.mesh.material_ids[f];
      }
      if (matId >= 0 && matId < static_cast<int>(materials.size())) {
        kd = glm::vec3(materials[static_cast<size_t>(matId)].diffuse[0],
                       materials[static_cast<size_t>(matId)].diffuse[1],
                       materials[static_cast<size_t>(matId)].diffuse[2]);
      }

      GpuTri tri{};
      tri.p0[0] = p0.x;
      tri.p0[1] = p0.y;
      tri.p0[2] = p0.z;
      tri.p1[0] = p1.x;
      tri.p1[1] = p1.y;
      tri.p1[2] = p1.z;
      tri.p2[0] = p2.x;
      tri.p2[1] = p2.y;
      tri.p2[2] = p2.z;
      tri.uv0uv1[0] = uv0.x;
      tri.uv0uv1[1] = uv0.y;
      tri.uv0uv1[2] = uv1.x;
      tri.uv0uv1[3] = uv1.y;
      tri.uv2pad[0] = uv2.x;
      tri.uv2pad[1] = uv2.y;
      tri.uv2pad[2] = 0.0f;
      tri.uv2pad[3] = 0.0f;  // hasTex filled after texture load
      tri.kd[0] = kd.x;
      tri.kd[1] = kd.y;
      tri.kd[2] = kd.z;
      tri.kd[3] = 1.0f;
      gpuTris.push_back(tri);
    }
  }

  if (gpuTris.empty()) {
    out.error = "OBJ contains no triangles";
    return out;
  }

  Texture albedo = TextureFactory::createSolid(gfx, 1.0f, 1.0f, 1.0f);
  bool hasTex = false;
  std::string texUsed;
  if (cfg.sampleColor) {
    for (const auto& m : materials) {
      if (m.diffuse_texname.empty()) {
        continue;
      }
      const std::filesystem::path texPath =
          std::filesystem::path(path).parent_path() / m.diffuse_texname;
      if (!std::filesystem::exists(texPath)) {
        continue;
      }
      if (loadPngRgba(texPath, albedo, gfx)) {
        hasTex = true;
        texUsed = texPath.filename().string();
        break;
      }
    }
    if (!hasTex) {
      const std::filesystem::path fallback = findSiblingAlbedo(path);
      if (!fallback.empty() && loadPngRgba(fallback, albedo, gfx)) {
        hasTex = true;
        texUsed = fallback.filename().string();
        if (out.warning.empty()) {
          out.warning = "No MTL map_Kd; using " + texUsed;
        } else {
          out.warning += "; no MTL map_Kd, using " + texUsed;
        }
      }
    }
  }
  if (hasTex) {
    for (auto& tri : gpuTris) {
      tri.uv2pad[3] = 1.0f;
      // OBJ with no MTL keeps fallback Kd 0.62; don't tint the albedo map.
      if (materials.empty()) {
        tri.kd[0] = 1.0f;
        tri.kd[1] = 1.0f;
        tri.kd[2] = 1.0f;
      }
    }
  }

  const glm::vec3 extent = bmax - bmin;
  const float maxEdge = std::max(extent.x, std::max(extent.y, extent.z));
  const int inner = n - 2 * padding;
  const float voxelSize = (maxEdge > 1e-8f) ? (maxEdge / static_cast<float>(inner)) : 1.0f;
  const glm::vec3 gridMin = bmin - glm::vec3(static_cast<float>(padding) * voxelSize);

  const uint32_t cellCount = static_cast<uint32_t>(n) * static_cast<uint32_t>(n) * static_cast<uint32_t>(n);
  const uint64_t fineCount = static_cast<uint64_t>(fineN) * static_cast<uint64_t>(fineN) *
                             static_cast<uint64_t>(fineN);
  const uint32_t fineWords = static_cast<uint32_t>((fineCount + 31ull) / 32ull);
  const VkDeviceSize triBytes = sizeof(GpuTri) * gpuTris.size();
  const uint32_t maxSamples = cfg.sampleColor ? kMaxColorSamples : 0u;
  const VkDeviceSize sampleBytes =
      sizeof(uint32_t) * (1u + 2u * std::max(maxSamples, 1u));
  const VkDeviceSize fineBytes = sizeof(uint32_t) * fineWords;

  AllocatedBuffer triBuf = gfx.createBuffer(
      triBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
      VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  AllocatedBuffer occBuf = gfx.createBuffer(fineBytes,
                                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  AllocatedBuffer seedBuf = gfx.createBuffer(sampleBytes,
                                             VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                             VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  gfx.uploadToBuffer(triBuf, gpuTris.data(), triBytes);

  vkResetDescriptorPool(gfx.device(), pool_, 0);
  VkDescriptorSetAllocateInfo alloc{};
  alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
  alloc.descriptorPool = pool_;
  alloc.descriptorSetCount = 1;
  alloc.pSetLayouts = &setLayout_;
  VkDescriptorSet set = VK_NULL_HANDLE;
  if (vkAllocateDescriptorSets(gfx.device(), &alloc, &set) != VK_SUCCESS) {
    gfx.destroyBuffer(triBuf);
    gfx.destroyBuffer(occBuf);
    gfx.destroyBuffer(seedBuf);
    TextureFactory::destroy(gfx, albedo);
    out.error = "Failed to allocate voxelize descriptor set";
    return out;
  }

  VkDescriptorBufferInfo triInfo{triBuf.buffer, 0, triBytes};
  VkDescriptorBufferInfo occInfo{occBuf.buffer, 0, fineBytes};
  VkDescriptorBufferInfo seedInfo{seedBuf.buffer, 0, sampleBytes};
  VkDescriptorImageInfo texInfo{};
  texInfo.sampler = albedo.sampler;
  texInfo.imageView = albedo.image.view;
  texInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

  VkWriteDescriptorSet writes[4]{};
  for (int i = 0; i < 4; ++i) {
    writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[i].dstSet = set;
    writes[i].dstBinding = static_cast<uint32_t>(i);
    writes[i].descriptorCount = 1;
  }
  writes[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  writes[0].pBufferInfo = &triInfo;
  writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  writes[1].pBufferInfo = &occInfo;
  writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  writes[2].pBufferInfo = &seedInfo;
  writes[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  writes[3].pImageInfo = &texInfo;
  vkUpdateDescriptorSets(gfx.device(), 4, writes, 0, nullptr);

  VoxelizePC pc{};
  pc.bmin[0] = gridMin.x;
  pc.bmin[1] = gridMin.y;
  pc.bmin[2] = gridMin.z;
  pc.voxelSize = voxelSize;
  pc.N = n;
  pc.conservative = cfg.conservative ? 1 : 0;
  pc.sampleColor = cfg.sampleColor ? 1 : 0;
  pc.triCount = static_cast<int32_t>(gpuTris.size());
  pc.subdiv = subdiv;
  pc.maxSamples = static_cast<int32_t>(maxSamples);

  AllocatedBuffer occRead = gfx.createBuffer(fineBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                             VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  AllocatedBuffer seedRead = gfx.createBuffer(sampleBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                              VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  gfx.immediateSubmit([&](VkCommandBuffer cmd) {
    vkCmdFillBuffer(cmd, occBuf.buffer, 0, fineBytes, 0);
    vkCmdFillBuffer(cmd, seedBuf.buffer, 0, 4, 0);
    VkMemoryBarrier2 fillBarrier{};
    fillBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    fillBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    fillBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    fillBarrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    fillBarrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
    VkDependencyInfo fillDep{};
    fillDep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    fillDep.memoryBarrierCount = 1;
    fillDep.pMemoryBarriers = &fillBarrier;
    vkCmdPipelineBarrier2(cmd, &fillDep);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 1, &set, 0,
                            nullptr);
    vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);
    const uint32_t groups = (static_cast<uint32_t>(gpuTris.size()) + 63u) / 64u;
    vkCmdDispatch(cmd, groups, 1, 1);

    VkMemoryBarrier2 computeBarrier{};
    computeBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    computeBarrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    computeBarrier.srcAccessMask = VK_ACCESS_2_SHADER_WRITE_BIT;
    computeBarrier.dstStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    computeBarrier.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
    VkDependencyInfo copyDep{};
    copyDep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    copyDep.memoryBarrierCount = 1;
    copyDep.pMemoryBarriers = &computeBarrier;
    vkCmdPipelineBarrier2(cmd, &copyDep);
    VkBufferCopy fineCopy{};
    fineCopy.size = fineBytes;
    vkCmdCopyBuffer(cmd, occBuf.buffer, occRead.buffer, 1, &fineCopy);
    VkBufferCopy seedCopy{};
    seedCopy.size = sampleBytes;
    vkCmdCopyBuffer(cmd, seedBuf.buffer, seedRead.buffer, 1, &seedCopy);
  });

  if (vmaInvalidateAllocation(gfx.allocator(), occRead.allocation, 0, fineBytes) != VK_SUCCESS ||
      vmaInvalidateAllocation(gfx.allocator(), seedRead.allocation, 0, sampleBytes) != VK_SUCCESS) {
    gfx.destroyBuffer(triBuf);
    gfx.destroyBuffer(occBuf);
    gfx.destroyBuffer(seedBuf);
    gfx.destroyBuffer(occRead);
    gfx.destroyBuffer(seedRead);
    TextureFactory::destroy(gfx, albedo);
    out.error = "Failed to invalidate voxelize readback memory";
    return out;
  }

  std::vector<uint32_t> fineBits(fineWords);
  std::memcpy(fineBits.data(), occRead.info.pMappedData, static_cast<size_t>(fineBytes));
  const auto* sampleWords = static_cast<const uint32_t*>(seedRead.info.pMappedData);
  const uint32_t rawCount = sampleWords ? sampleWords[0] : 0u;
  const uint32_t nWrite = std::min(rawCount, maxSamples);
  out.colorSamples = rawCount;
  out.colorDropped = rawCount > maxSamples ? rawCount - maxSamples : 0u;
  std::vector<std::pair<uint32_t, uint32_t>> recs;
  recs.reserve(nWrite);
  for (uint32_t i = 0; i < nWrite; ++i) {
    const uint32_t fi = sampleWords[1u + i * 2u];
    const uint32_t packed = sampleWords[2u + i * 2u];
    recs.emplace_back(fi, packed);
  }

  gfx.destroyBuffer(triBuf);
  gfx.destroyBuffer(occBuf);
  gfx.destroyBuffer(seedBuf);
  gfx.destroyBuffer(occRead);
  gfx.destroyBuffer(seedRead);
  TextureFactory::destroy(gfx, albedo);

  std::vector<uint32_t> occ(cellCount, 0u);
  uint32_t occupiedFine = 0;
  const uint32_t uFineN = static_cast<uint32_t>(fineN);
  const uint32_t uSub = static_cast<uint32_t>(subdiv);
  const uint32_t uN = static_cast<uint32_t>(n);
  for (uint32_t wi = 0; wi < fineWords; ++wi) {
    uint32_t word = fineBits[wi];
    while (word != 0u) {
      const uint32_t bit = static_cast<uint32_t>(std::countr_zero(word));
      word &= word - 1u;
      const uint32_t fi = (wi << 5) + bit;
      if (fi >= fineCount) {
        break;
      }
      ++occupiedFine;
      const uint32_t x = fi % uFineN;
      const uint32_t y = (fi / uFineN) % uFineN;
      const uint32_t z = fi / (uFineN * uFineN);
      const uint32_t cidx = (x / uSub) + (y / uSub) * uN + (z / uSub) * uN * uN;
      occ[cidx] = 1u;
    }
  }

  constexpr uint32_t kGray888 = (158u << 16) | (163u << 8) | 173u;  // 0.62, 0.64, 0.68
  out.coarseRgb.assign(cellCount, kGray888);
  if (cfg.sampleColor && !recs.empty()) {
    std::sort(recs.begin(), recs.end(),
              [](const std::pair<uint32_t, uint32_t>& a, const std::pair<uint32_t, uint32_t>& b) {
                return a.first < b.first;
              });
    std::vector<uint64_t> coarseR(cellCount, 0);
    std::vector<uint64_t> coarseG(cellCount, 0);
    std::vector<uint64_t> coarseB(cellCount, 0);
    std::vector<uint64_t> coarseN(cellCount, 0);
    out.fineId.reserve(recs.size());
    out.fineRgb.reserve(recs.size());
    for (size_t i = 0; i < recs.size();) {
      const uint32_t fi = recs[i].first;
      uint64_t rs = 0;
      uint64_t gs = 0;
      uint64_t bs = 0;
      uint64_t ns = 0;
      while (i < recs.size() && recs[i].first == fi) {
        const uint32_t p = recs[i].second;
        rs += (p >> 16) & 255u;
        gs += (p >> 8) & 255u;
        bs += p & 255u;
        ++ns;
        ++i;
      }
      if (ns == 0u || fi >= fineCount) {
        continue;
      }
      const uint32_t r8 = static_cast<uint32_t>((rs + ns / 2u) / ns);
      const uint32_t g8 = static_cast<uint32_t>((gs + ns / 2u) / ns);
      const uint32_t b8 = static_cast<uint32_t>((bs + ns / 2u) / ns);
      out.fineId.push_back(fi);
      out.fineRgb.push_back((r8 << 16) | (g8 << 8) | b8);
      const uint32_t x = fi % uFineN;
      const uint32_t y = (fi / uFineN) % uFineN;
      const uint32_t z = fi / (uFineN * uFineN);
      const uint32_t cidx = (x / uSub) + (y / uSub) * uN + (z / uSub) * uN * uN;
      if (cidx < cellCount) {
        coarseR[cidx] += r8;
        coarseG[cidx] += g8;
        coarseB[cidx] += b8;
        ++coarseN[cidx];
      }
    }
    for (uint32_t i = 0; i < cellCount; ++i) {
      if (coarseN[i] > 0u) {
        const uint32_t nC = static_cast<uint32_t>(coarseN[i]);
        const uint32_t r8 = static_cast<uint32_t>((coarseR[i] + nC / 2u) / nC);
        const uint32_t g8 = static_cast<uint32_t>((coarseG[i] + nC / 2u) / nC);
        const uint32_t b8 = static_cast<uint32_t>((coarseB[i] + nC / 2u) / nC);
        out.coarseRgb[i] = (r8 << 16) | (g8 << 8) | b8;
      }
    }
  }
  fillOccupiedMaterials(occ, cfg.fallbackMaterial, out);
  out.ok = true;
  out.n = n;
  out.subdiv = subdiv;
  out.fineN = fineN;
  out.fineBits = std::move(fineBits);
  out.occupiedFine = occupiedFine;
  out.voxelSize = voxelSize;
  out.bmin = gridMin;
  return out;
}

MeshVoxelizeResult voxelizeObjSurface(GfxDevice& gfx, MeshVoxelizerGpu& gpu, const std::string& path,
                                      const MeshVoxelizeConfig& cfg) {
  return gpu.voxelizeObjSurface(gfx, path, cfg);
}
