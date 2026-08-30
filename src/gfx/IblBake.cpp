#include "gfx/IblBake.h"

#include "gfx/GfxDevice.h"
#include "gfx/PipelineBuilder.h"
#include "gfx/ShIrradiance.h"

#include <glm/glm.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace IblBake {
namespace {

struct IblFacePC {
  float face;        // 0..5
  float roughness;
  float resolution;  // source env face size for PDF mip
  float pad;
};

struct IblCpuData {
  uint32_t mipCount = 1;
  std::vector<float> prefiltered;
  std::vector<float> brdf;
  Sh9 sh{};
};

uint32_t calcCubeMips(uint32_t size) {
  uint32_t levels = 1;
  while ((size >> (levels - 1)) > 1) {
    ++levels;
  }
  return levels;
}

bool writeBytes(std::ofstream& out, const void* data, size_t bytes) {
  out.write(reinterpret_cast<const char*>(data), static_cast<std::streamsize>(bytes));
  return static_cast<bool>(out);
}

bool readBytes(std::ifstream& in, void* data, size_t bytes) {
  in.read(reinterpret_cast<char*>(data), static_cast<std::streamsize>(bytes));
  return static_cast<bool>(in) && static_cast<size_t>(in.gcount()) == bytes;
}

uint64_t fileSizeAndMtime(const std::filesystem::path& path, int64_t& outMtime) {
  std::error_code ec;
  const auto sz = std::filesystem::file_size(path, ec);
  if (ec) {
    outMtime = 0;
    return 0;
  }
  const auto ftime = std::filesystem::last_write_time(path, ec);
  if (ec) {
    outMtime = 0;
  } else {
    outMtime = std::chrono::duration_cast<std::chrono::seconds>(ftime.time_since_epoch()).count();
  }
  return static_cast<uint64_t>(sz);
}

std::filesystem::path cacheFilePath(const std::string& cacheDir, const std::string& sourceHdrPath) {
  const std::filesystem::path src(sourceHdrPath);
  std::string stem = src.stem().string();
  if (stem.empty()) {
    stem = "sky";
  }
  return std::filesystem::path(cacheDir) / (stem + ".veibl");
}

bool saveCache(const std::filesystem::path& cachePath, const std::string& sourceHdrPath,
               const IblCpuData& data) {
  std::error_code ec;
  std::filesystem::create_directories(cachePath.parent_path(), ec);

  int64_t mtime = 0;
  const uint64_t srcSize = fileSizeAndMtime(sourceHdrPath, mtime);
  const std::string srcName = std::filesystem::path(sourceHdrPath).filename().string();

  std::ofstream out(cachePath, std::ios::binary);
  if (!out) {
    return false;
  }

  const char magic[8] = {'V', 'E', 'I', 'B', 'L', '0', '8', '\0'};
  const uint32_t version = kCacheVersion;
  const uint32_t envSize = kEnvSize;
  const uint32_t preSize = kPrefilterSize;
  const uint32_t brdfSize = kBrdfSize;
  const uint32_t mipCount = data.mipCount;
  const uint32_t nameLen = static_cast<uint32_t>(srcName.size());
  const uint32_t shValid = data.sh.valid ? 1u : 0u;
  const uint32_t prefCount = static_cast<uint32_t>(data.prefiltered.size());
  const uint32_t brdfCount = static_cast<uint32_t>(data.brdf.size());

  if (!writeBytes(out, magic, 8) || !writeBytes(out, &version, sizeof(version)) ||
      !writeBytes(out, &envSize, sizeof(envSize)) || !writeBytes(out, &preSize, sizeof(preSize)) ||
      !writeBytes(out, &brdfSize, sizeof(brdfSize)) || !writeBytes(out, &mipCount, sizeof(mipCount)) ||
      !writeBytes(out, &srcSize, sizeof(srcSize)) || !writeBytes(out, &mtime, sizeof(mtime)) ||
      !writeBytes(out, &nameLen, sizeof(nameLen)) || !writeBytes(out, srcName.data(), srcName.size()) ||
      !writeBytes(out, &shValid, sizeof(shValid))) {
    return false;
  }
  for (int i = 0; i < 9; ++i) {
    const float rgb[3] = {data.sh.c[i].r, data.sh.c[i].g, data.sh.c[i].b};
    if (!writeBytes(out, rgb, sizeof(rgb))) {
      return false;
    }
  }
  return writeBytes(out, &prefCount, sizeof(prefCount)) &&
         writeBytes(out, data.prefiltered.data(), data.prefiltered.size() * sizeof(float)) &&
         writeBytes(out, &brdfCount, sizeof(brdfCount)) &&
         writeBytes(out, data.brdf.data(), data.brdf.size() * sizeof(float));
}

bool loadCache(const std::filesystem::path& cachePath, const std::string& sourceHdrPath,
               IblCpuData& outData) {
  if (!std::filesystem::exists(cachePath)) {
    return false;
  }
  int64_t srcMtime = 0;
  const uint64_t srcSize = fileSizeAndMtime(sourceHdrPath, srcMtime);
  const std::string srcName = std::filesystem::path(sourceHdrPath).filename().string();

  std::ifstream in(cachePath, std::ios::binary);
  if (!in) {
    return false;
  }

  char magic[8] = {};
  uint32_t version = 0, envSize = 0, preSize = 0, brdfSize = 0, mipCount = 0, nameLen = 0;
  uint64_t cachedSrcSize = 0;
  int64_t cachedMtime = 0;
  if (!readBytes(in, magic, 8) || std::memcmp(magic, "VEIBL08", 7) != 0 ||
      !readBytes(in, &version, sizeof(version)) || version != kCacheVersion ||
      !readBytes(in, &envSize, sizeof(envSize)) || !readBytes(in, &preSize, sizeof(preSize)) ||
      !readBytes(in, &brdfSize, sizeof(brdfSize)) || !readBytes(in, &mipCount, sizeof(mipCount)) ||
      !readBytes(in, &cachedSrcSize, sizeof(cachedSrcSize)) ||
      !readBytes(in, &cachedMtime, sizeof(cachedMtime)) || !readBytes(in, &nameLen, sizeof(nameLen)) ||
      envSize != kEnvSize || preSize != kPrefilterSize || brdfSize != kBrdfSize ||
      cachedSrcSize != srcSize || cachedMtime != srcMtime || nameLen != srcName.size() ||
      nameLen > 1024) {
    return false;
  }
  std::string cachedName(nameLen, '\0');
  if (!readBytes(in, cachedName.data(), nameLen) || cachedName != srcName) {
    return false;
  }

  uint32_t shValid = 0;
  if (!readBytes(in, &shValid, sizeof(shValid))) {
    return false;
  }
  outData.sh = {};
  outData.sh.valid = shValid != 0;
  for (int i = 0; i < 9; ++i) {
    float rgb[3] = {};
    if (!readBytes(in, rgb, sizeof(rgb))) {
      return false;
    }
    outData.sh.c[i] = glm::vec3(rgb[0], rgb[1], rgb[2]);
  }

  uint32_t prefCount = 0, brdfCount = 0;
  if (!readBytes(in, &prefCount, sizeof(prefCount)) || prefCount == 0 ||
      prefCount > 64u * 1024u * 1024u) {
    return false;
  }
  outData.prefiltered.resize(prefCount);
  if (!readBytes(in, outData.prefiltered.data(), prefCount * sizeof(float)) ||
      !readBytes(in, &brdfCount, sizeof(brdfCount)) || brdfCount == 0 ||
      brdfCount > 64u * 1024u * 1024u) {
    return false;
  }
  outData.brdf.resize(brdfCount);
  if (!readBytes(in, outData.brdf.data(), brdfCount * sizeof(float))) {
    return false;
  }
  outData.mipCount = mipCount;
  return true;
}

IblMaps uploadFromCpu(GfxDevice& gfx, const IblCpuData& data) {
  IblMaps maps{};
  if (data.prefiltered.empty() || data.brdf.empty()) {
    return maps;
  }
  maps.mipCount = data.mipCount;
  maps.prefiltered =
      gfx.createCubemap(kPrefilterSize, VK_FORMAT_R32G32B32A32_SFLOAT,
                        VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, maps.mipCount);
  gfx.uploadCubemapRGBA32F(maps.prefiltered, data.prefiltered.data());
  maps.brdfLut = gfx.createImage(
      {kBrdfSize, kBrdfSize, 1}, VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
  gfx.uploadToImage(maps.brdfLut, data.brdf.data(), data.brdf.size() * sizeof(float),
                    {kBrdfSize, kBrdfSize, 1});
  maps.cubeSampler =
      gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, maps.mipCount);
  maps.lutSampler =
      gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, 1);
  maps.ready = true;
  return maps;
}

void beginColorPass(VkCommandBuffer cmd, VkImageView colorView, uint32_t width, uint32_t height) {
  VkRenderingAttachmentInfo color{};
  color.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  color.imageView = colorView;
  color.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  color.clearValue.color = {{0, 0, 0, 1}};

  VkRenderingInfo info{};
  info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  info.renderArea.extent = {width, height};
  info.layerCount = 1;
  info.colorAttachmentCount = 1;
  info.pColorAttachments = &color;
  vkCmdBeginRendering(cmd, &info);

  VkViewport vp{0, 0, static_cast<float>(width), static_cast<float>(height), 0, 1};
  VkRect2D scissor{{0, 0}, {width, height}};
  vkCmdSetViewport(cmd, 0, 1, &vp);
  vkCmdSetScissor(cmd, 0, 1, &scissor);
}

IblMaps bakeGpu(GfxDevice& gfx, const EquirectHdrData& equirect, IblCpuData* outCpu) {
  IblMaps maps{};
  if (equirect.rgba.empty()) {
    return maps;
  }

  const std::string shaderDir = VE_SHADER_DIR;
  const uint32_t prefMips = calcCubeMips(kPrefilterSize);
  const uint32_t envMips = calcCubeMips(kEnvSize);

  // --- Upload equirect ---
  Texture equirectTex{};
  equirectTex.image = gfx.createImage(
      {static_cast<uint32_t>(equirect.width), static_cast<uint32_t>(equirect.height), 1},
      VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
  gfx.uploadToImage(equirectTex.image, equirect.rgba.data(),
                    equirect.rgba.size() * sizeof(float),
                    {static_cast<uint32_t>(equirect.width), static_cast<uint32_t>(equirect.height), 1});
  equirectTex.sampler =
      gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, 1);

  // --- Env cubemap (mip0 rendered, then blit mips) ---
  AllocatedImage envCube = gfx.createCubemap(
      kEnvSize, VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
          VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
      envMips);

  VkShaderModule fsVert = gfx.loadShaderModule(shaderDir + "/fullscreen.vert.spv");
  VkShaderModule equirectFrag = gfx.loadShaderModule(shaderDir + "/ibl_equirect_to_cube.frag.spv");
  VkShaderModule prefilterFrag = gfx.loadShaderModule(shaderDir + "/ibl_prefilter.frag.spv");
  VkShaderModule brdfFrag = gfx.loadShaderModule(shaderDir + "/ibl_brdf.frag.spv");

  VkDescriptorSetLayoutBinding bind{};
  bind.binding = 0;
  bind.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  bind.descriptorCount = 1;
  bind.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
  VkDescriptorSetLayoutCreateInfo dslInfo{};
  dslInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  dslInfo.bindingCount = 1;
  dslInfo.pBindings = &bind;
  VkDescriptorSetLayout bakeSetLayout = VK_NULL_HANDLE;
  vkCreateDescriptorSetLayout(gfx.device(), &dslInfo, nullptr, &bakeSetLayout);

  VkPushConstantRange pushRange{};
  pushRange.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
  pushRange.offset = 0;
  pushRange.size = sizeof(IblFacePC);

  VkPipelineLayoutCreateInfo plInfo{};
  plInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  plInfo.setLayoutCount = 1;
  plInfo.pSetLayouts = &bakeSetLayout;
  plInfo.pushConstantRangeCount = 1;
  plInfo.pPushConstantRanges = &pushRange;
  VkPipelineLayout cubeLayout = VK_NULL_HANDLE;
  vkCreatePipelineLayout(gfx.device(), &plInfo, nullptr, &cubeLayout);

  VkPipelineLayoutCreateInfo brdfPlInfo = plInfo;
  brdfPlInfo.setLayoutCount = 0;
  brdfPlInfo.pSetLayouts = nullptr;
  brdfPlInfo.pushConstantRangeCount = 0;
  VkPipelineLayout brdfLayout = VK_NULL_HANDLE;
  vkCreatePipelineLayout(gfx.device(), &brdfPlInfo, nullptr, &brdfLayout);

  VkPipeline equirectPipe =
      PipelineBuilder()
          .setShaders(fsVert, equirectFrag)
          .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
          .setDepthTest(false, false, VK_COMPARE_OP_ALWAYS)
          .setColorFormat(VK_FORMAT_R32G32B32A32_SFLOAT)
          .setLayout(cubeLayout)
          .build(gfx.device());
  VkPipeline prefilterPipe =
      PipelineBuilder()
          .setShaders(fsVert, prefilterFrag)
          .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
          .setDepthTest(false, false, VK_COMPARE_OP_ALWAYS)
          .setColorFormat(VK_FORMAT_R32G32B32A32_SFLOAT)
          .setLayout(cubeLayout)
          .build(gfx.device());
  VkPipeline brdfPipe =
      PipelineBuilder()
          .setShaders(fsVert, brdfFrag)
          .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
          .setDepthTest(false, false, VK_COMPARE_OP_ALWAYS)
          .setColorFormat(VK_FORMAT_R32G32B32A32_SFLOAT)
          .setLayout(brdfLayout)
          .build(gfx.device());

  VkDescriptorPoolSize poolSize{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 8};
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.maxSets = 8;
  poolInfo.poolSizeCount = 1;
  poolInfo.pPoolSizes = &poolSize;
  VkDescriptorPool bakePool = VK_NULL_HANDLE;
  vkCreateDescriptorPool(gfx.device(), &poolInfo, nullptr, &bakePool);

  VkDescriptorSet equirectSet = VK_NULL_HANDLE;
  VkDescriptorSet envSet = VK_NULL_HANDLE;
  {
    VkDescriptorSetAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc.descriptorPool = bakePool;
    alloc.descriptorSetCount = 1;
    alloc.pSetLayouts = &bakeSetLayout;
    vkAllocateDescriptorSets(gfx.device(), &alloc, &equirectSet);
    vkAllocateDescriptorSets(gfx.device(), &alloc, &envSet);

    VkDescriptorImageInfo img{};
    img.sampler = equirectTex.sampler;
    img.imageView = equirectTex.image.view;
    img.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = equirectSet;
    write.dstBinding = 0;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.descriptorCount = 1;
    write.pImageInfo = &img;
    vkUpdateDescriptorSets(gfx.device(), 1, &write, 0, nullptr);
  }

  std::cout << "IBL GPU: equirect -> cubemap " << kEnvSize << "^2 ..." << std::endl;
  std::vector<VkImageView> envFaceViews;
  envFaceViews.reserve(6);
  for (int face = 0; face < 6; ++face) {
    envFaceViews.push_back(
        gfx.createImageView(envCube, VK_IMAGE_VIEW_TYPE_2D, 0, 1, static_cast<uint32_t>(face), 1));
  }
  gfx.immediateSubmit([&](VkCommandBuffer cmd) {
    gfx.transitionImage(cmd, envCube.image, VK_IMAGE_LAYOUT_UNDEFINED,
                        VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                        0, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6);

    for (int face = 0; face < 6; ++face) {
      beginColorPass(cmd, envFaceViews[static_cast<size_t>(face)], kEnvSize, kEnvSize);

      IblFacePC pc{};
      pc.face = static_cast<float>(face);
      pc.roughness = 0.0f;
      pc.resolution = static_cast<float>(kEnvSize);

      vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, equirectPipe);
      vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, cubeLayout, 0, 1, &equirectSet, 0,
                              nullptr);
      vkCmdPushConstants(cmd, cubeLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);
      vkCmdDraw(cmd, 3, 1, 0, 0);
      vkCmdEndRendering(cmd);
    }

    gfx.transitionImage(cmd, envCube.image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                        VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                        VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                        VK_ACCESS_2_SHADER_READ_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 6);
  });
  for (VkImageView v : envFaceViews) {
    gfx.destroyImageView(v);
  }
  envCube.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
  gfx.generateCubemapMips(envCube);

  // Update env descriptor now that cube view exists with mips.
  {
    VkSampler envSampler =
        gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, envMips);
    VkDescriptorImageInfo img{};
    img.sampler = envSampler;
    img.imageView = envCube.view;
    img.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkWriteDescriptorSet write{};
    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    write.dstSet = envSet;
    write.dstBinding = 0;
    write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    write.descriptorCount = 1;
    write.pImageInfo = &img;
    vkUpdateDescriptorSets(gfx.device(), 1, &write, 0, nullptr);

    // --- Prefiltered cubemap ---
    std::cout << "IBL GPU: prefiltering " << kPrefilterSize << "^2, " << prefMips << " mips ..."
              << std::endl;
    maps.mipCount = prefMips;
    maps.prefiltered = gfx.createCubemap(
        kPrefilterSize, VK_FORMAT_R32G32B32A32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
            VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
        prefMips);

    std::vector<VkImageView> prefViews;
    prefViews.reserve(static_cast<size_t>(prefMips) * 6u);
    for (uint32_t mip = 0; mip < prefMips; ++mip) {
      for (int face = 0; face < 6; ++face) {
        prefViews.push_back(gfx.createImageView(maps.prefiltered, VK_IMAGE_VIEW_TYPE_2D, mip, 1,
                                                static_cast<uint32_t>(face), 1));
      }
    }

    gfx.immediateSubmit([&](VkCommandBuffer cmd) {
      gfx.transitionImage(cmd, maps.prefiltered.image, VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                          0, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_IMAGE_ASPECT_COLOR_BIT, 0,
                          prefMips, 0, 6);

      size_t viewIndex = 0;
      for (uint32_t mip = 0; mip < prefMips; ++mip) {
        const uint32_t dim = std::max(1u, kPrefilterSize >> mip);
        const float roughness =
            static_cast<float>(mip) / static_cast<float>(std::max(1u, prefMips - 1));
        for (int face = 0; face < 6; ++face) {
          beginColorPass(cmd, prefViews[viewIndex++], dim, dim);

          IblFacePC pc{};
          pc.face = static_cast<float>(face);
          pc.roughness = roughness;
          pc.resolution = static_cast<float>(kEnvSize);

          vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, prefilterPipe);
          vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, cubeLayout, 0, 1, &envSet, 0,
                                  nullptr);
          vkCmdPushConstants(cmd, cubeLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(pc), &pc);
          vkCmdDraw(cmd, 3, 1, 0, 0);
          vkCmdEndRendering(cmd);
        }
      }

      gfx.transitionImage(cmd, maps.prefiltered.image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT,
                          VK_IMAGE_ASPECT_COLOR_BIT, 0, prefMips, 0, 6);
    });
    for (VkImageView v : prefViews) {
      gfx.destroyImageView(v);
    }
    maps.prefiltered.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    // --- BRDF LUT ---
    std::cout << "IBL GPU: BRDF LUT " << kBrdfSize << "^2 ..." << std::endl;
    maps.brdfLut = gfx.createImage(
        {kBrdfSize, kBrdfSize, 1}, VK_FORMAT_R32G32B32A32_SFLOAT,
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
            VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT);

    gfx.immediateSubmit([&](VkCommandBuffer cmd) {
      gfx.transitionImage(cmd, maps.brdfLut.image, VK_IMAGE_LAYOUT_UNDEFINED,
                          VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                          0, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
      beginColorPass(cmd, maps.brdfLut.view, kBrdfSize, kBrdfSize);
      vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, brdfPipe);
      vkCmdDraw(cmd, 3, 1, 0, 0);
      vkCmdEndRendering(cmd);
      gfx.transitionImage(cmd, maps.brdfLut.image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                          VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                          VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                          VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
    });
    maps.brdfLut.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    maps.cubeSampler =
        gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, prefMips);
    maps.lutSampler =
        gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE, false, 1);
    maps.ready = true;

    if (outCpu) {
      gfx.downloadCubemapRGBA32F(maps.prefiltered, outCpu->prefiltered);
      gfx.downloadImageRGBA32F(maps.brdfLut, outCpu->brdf);
      outCpu->mipCount = prefMips;
    }

    gfx.destroySampler(envSampler);
  }

  // Cleanup bake-only resources.
  vkDestroyPipeline(gfx.device(), equirectPipe, nullptr);
  vkDestroyPipeline(gfx.device(), prefilterPipe, nullptr);
  vkDestroyPipeline(gfx.device(), brdfPipe, nullptr);
  vkDestroyPipelineLayout(gfx.device(), cubeLayout, nullptr);
  vkDestroyPipelineLayout(gfx.device(), brdfLayout, nullptr);
  vkDestroyDescriptorPool(gfx.device(), bakePool, nullptr);
  vkDestroyDescriptorSetLayout(gfx.device(), bakeSetLayout, nullptr);
  vkDestroyShaderModule(gfx.device(), fsVert, nullptr);
  vkDestroyShaderModule(gfx.device(), equirectFrag, nullptr);
  vkDestroyShaderModule(gfx.device(), prefilterFrag, nullptr);
  vkDestroyShaderModule(gfx.device(), brdfFrag, nullptr);
  gfx.destroySampler(equirectTex.sampler);
  gfx.destroyImage(equirectTex.image);
  gfx.destroyImage(envCube);

  return maps;
}

}  // namespace

IblMaps build(GfxDevice& gfx, const EquirectHdrData& equirect) {
  IblCpuData cpu{};
  IblMaps maps = bakeGpu(gfx, equirect, &cpu);
  std::cout << "IBL: ready (GPU, prefiltered mips=" << maps.mipCount << ")" << std::endl;
  return maps;
}

IblMaps buildOrLoad(GfxDevice& gfx, const EquirectHdrData& equirect, const std::string& sourceHdrPath,
                    const std::string& cacheDir, Sh9* outSh) {
  if (outSh) {
    *outSh = {};
  }
  if (equirect.rgba.empty()) {
    return {};
  }

  const std::filesystem::path cachePath = cacheFilePath(cacheDir, sourceHdrPath);
  IblCpuData data{};

  if (loadCache(cachePath, sourceHdrPath, data)) {
    std::cout << "IBL: loaded cache " << cachePath.string() << std::endl;
    if (outSh) {
      *outSh = data.sh;
    }
    IblMaps maps = uploadFromCpu(gfx, data);
    std::cout << "IBL: ready (cache, mips=" << maps.mipCount << ")" << std::endl;
    return maps;
  }

  std::cout << "IBL: cache miss, GPU baking ..." << std::endl;
  data.sh = ShIrradiance::projectEquirect(equirect.rgba.data(), equirect.width, equirect.height);
  IblMaps maps = bakeGpu(gfx, equirect, &data);
  if (outSh) {
    *outSh = data.sh;
  }
  if (!saveCache(cachePath, sourceHdrPath, data)) {
    std::cerr << "IBL: failed to write cache " << cachePath.string() << std::endl;
  } else {
    std::cout << "IBL: wrote cache " << cachePath.string() << std::endl;
  }
  std::cout << "IBL: ready (GPU, mips=" << maps.mipCount << ", sh=" << (data.sh.valid ? "ok" : "fail")
            << ")" << std::endl;
  return maps;
}

void destroy(GfxDevice& gfx, IblMaps& maps) {
  if (maps.cubeSampler) {
    gfx.destroySampler(maps.cubeSampler);
    maps.cubeSampler = VK_NULL_HANDLE;
  }
  if (maps.lutSampler) {
    gfx.destroySampler(maps.lutSampler);
    maps.lutSampler = VK_NULL_HANDLE;
  }
  gfx.destroyImage(maps.prefiltered);
  gfx.destroyImage(maps.brdfLut);
  maps = {};
}

}  // namespace IblBake
