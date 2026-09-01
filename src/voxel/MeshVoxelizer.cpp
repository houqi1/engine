#include "voxel/MeshVoxelizer.h"

#include "gfx/GfxDevice.h"
#include "gfx/Texture.h"

#include <glm/glm.hpp>

#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>

#include <stb_image.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>
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
};

void fail(const std::string& m) { throw std::runtime_error(m); }

glm::vec3 unpackRgb(uint32_t p) {
  const float r = static_cast<float>((p >> 16) & 255u) / 255.0f;
  const float g = static_cast<float>((p >> 8) & 255u) / 255.0f;
  const float b = static_cast<float>(p & 255u) / 255.0f;
  return glm::vec3(r, g, b);
}

float colorDist2(const glm::vec3& a, const glm::vec3& b) {
  const glm::vec3 d = a - b;
  return glm::dot(d, d);
}

void quantizePalette(const std::vector<uint32_t>& occ, const std::vector<uint32_t>& seeds,
                     uint32_t fallback, bool sampleColor, MeshVoxelizeResult& out) {
  const size_t count = occ.size();
  out.material.assign(count, 0u);
  out.palette.fill(glm::vec3(0.62f, 0.64f, 0.68f));
  out.palette[0] = glm::vec3(0.0f);
  out.paletteUsed = 1;
  out.occupied = 0;

  if (!sampleColor) {
    for (size_t i = 0; i < count; ++i) {
      if (occ[i] != 0u) {
        out.material[i] = fallback;
        ++out.occupied;
      }
    }
    out.palette[fallback] = glm::vec3(0.62f, 0.64f, 0.68f);
    out.paletteUsed = std::max(out.paletteUsed, fallback);
    return;
  }

  std::vector<glm::vec3> colors;
  colors.reserve(256);
  colors.emplace_back(0.0f);
  std::unordered_map<uint32_t, uint32_t> exact;
  exact.reserve(256);

  auto addOrNearest = [&](const glm::vec3& rgb) -> uint32_t {
    const uint32_t key = (static_cast<uint32_t>(rgb.r * 255.0f + 0.5f) << 16) |
                         (static_cast<uint32_t>(rgb.g * 255.0f + 0.5f) << 8) |
                         static_cast<uint32_t>(rgb.b * 255.0f + 0.5f);
    const auto it = exact.find(key);
    if (it != exact.end()) {
      return it->second;
    }
    if (colors.size() < 256) {
      const uint32_t idx = static_cast<uint32_t>(colors.size());
      colors.push_back(rgb);
      exact[key] = idx;
      return idx;
    }
    uint32_t best = 1;
    float bestD = colorDist2(rgb, colors[1]);
    for (uint32_t i = 2; i < colors.size(); ++i) {
      const float d = colorDist2(rgb, colors[i]);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  };

  for (size_t i = 0; i < count; ++i) {
    if (occ[i] == 0u) {
      continue;
    }
    ++out.occupied;
    glm::vec3 rgb = glm::vec3(0.62f, 0.64f, 0.68f);
    if ((seeds[i] >> 24) != 0u) {
      rgb = unpackRgb(seeds[i]);
    }
    out.material[i] = addOrNearest(rgb);
  }

  out.paletteUsed = static_cast<uint32_t>(colors.size());
  for (size_t i = 0; i < colors.size(); ++i) {
    out.palette[i] = colors[i];
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
  if (n - 2 * padding < 1) {
    out.error = "gridN too small for padding";
    return out;
  }

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
  if (cfg.sampleColor) {
    for (const auto& m : materials) {
      if (m.diffuse_texname.empty()) {
        continue;
      }
      std::filesystem::path texPath = std::filesystem::path(path).parent_path() / m.diffuse_texname;
      if (!std::filesystem::exists(texPath)) {
        continue;
      }
      int w = 0, h = 0, comp = 0;
      stbi_uc* pixels = stbi_load(texPath.string().c_str(), &w, &h, &comp, 4);
      if (!pixels || w <= 0 || h <= 0) {
        if (pixels) {
          stbi_image_free(pixels);
        }
        continue;
      }
      TextureFactory::destroy(gfx, albedo);
      albedo.image = gfx.createImage(
          {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1}, VK_FORMAT_R8G8B8A8_SRGB,
          VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
      gfx.uploadToImage(albedo.image, pixels, static_cast<VkDeviceSize>(w * h * 4),
                        {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1});
      albedo.sampler = gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, false);
      stbi_image_free(pixels);
      hasTex = true;
      break;
    }
  }
  if (hasTex) {
    for (auto& tri : gpuTris) {
      tri.uv2pad[3] = 1.0f;
    }
  }

  const glm::vec3 extent = bmax - bmin;
  const float maxEdge = std::max(extent.x, std::max(extent.y, extent.z));
  const int inner = n - 2 * padding;
  const float voxelSize = (maxEdge > 1e-8f) ? (maxEdge / static_cast<float>(inner)) : 1.0f;
  const glm::vec3 gridMin = bmin - glm::vec3(static_cast<float>(padding) * voxelSize);

  const uint32_t cellCount = static_cast<uint32_t>(n) * static_cast<uint32_t>(n) * static_cast<uint32_t>(n);
  const VkDeviceSize triBytes = sizeof(GpuTri) * gpuTris.size();
  const VkDeviceSize gridBytes = sizeof(uint32_t) * cellCount;

  AllocatedBuffer triBuf = gfx.createBuffer(
      triBytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
      VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  AllocatedBuffer occBuf = gfx.createBuffer(gridBytes,
                                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                            VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  AllocatedBuffer seedBuf = gfx.createBuffer(gridBytes,
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
  VkDescriptorBufferInfo occInfo{occBuf.buffer, 0, gridBytes};
  VkDescriptorBufferInfo seedInfo{seedBuf.buffer, 0, gridBytes};
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

  gfx.immediateSubmit([&](VkCommandBuffer cmd) {
    vkCmdFillBuffer(cmd, occBuf.buffer, 0, gridBytes, 0);
    vkCmdFillBuffer(cmd, seedBuf.buffer, 0, gridBytes, 0);
    VkMemoryBarrier2 fillBarrier{};
    fillBarrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
    fillBarrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
    fillBarrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
    fillBarrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
    fillBarrier.dstAccessMask = VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT;
    VkDependencyInfo dep{};
    dep.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
    dep.memoryBarrierCount = 1;
    dep.pMemoryBarriers = &fillBarrier;
    vkCmdPipelineBarrier2(cmd, &dep);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 1, &set, 0,
                            nullptr);
    vkCmdPushConstants(cmd, pipelineLayout_, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(pc), &pc);
    const uint32_t groups = (static_cast<uint32_t>(gpuTris.size()) + 63u) / 64u;
    vkCmdDispatch(cmd, groups, 1, 1);
  });

  AllocatedBuffer occRead = gfx.createBuffer(gridBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                             VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  AllocatedBuffer seedRead = gfx.createBuffer(gridBytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                              VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  gfx.immediateSubmit([&](VkCommandBuffer cmd) {
    VkBufferCopy copy{};
    copy.size = gridBytes;
    vkCmdCopyBuffer(cmd, occBuf.buffer, occRead.buffer, 1, &copy);
    vkCmdCopyBuffer(cmd, seedBuf.buffer, seedRead.buffer, 1, &copy);
  });

  std::vector<uint32_t> occ(cellCount);
  std::vector<uint32_t> seeds(cellCount);
  std::memcpy(occ.data(), occRead.info.pMappedData, static_cast<size_t>(gridBytes));
  std::memcpy(seeds.data(), seedRead.info.pMappedData, static_cast<size_t>(gridBytes));

  gfx.destroyBuffer(triBuf);
  gfx.destroyBuffer(occBuf);
  gfx.destroyBuffer(seedBuf);
  gfx.destroyBuffer(occRead);
  gfx.destroyBuffer(seedRead);
  TextureFactory::destroy(gfx, albedo);

  quantizePalette(occ, seeds, cfg.fallbackMaterial, cfg.sampleColor, out);
  out.ok = true;
  out.n = n;
  out.voxelSize = voxelSize;
  out.bmin = gridMin;
  return out;
}

MeshVoxelizeResult voxelizeObjSurface(GfxDevice& gfx, MeshVoxelizerGpu& gpu, const std::string& path,
                                      const MeshVoxelizeConfig& cfg) {
  return gpu.voxelizeObjSurface(gfx, path, cfg);
}
