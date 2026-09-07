#include "render/VoxelRenderer.h"

#include "gfx/PipelineBuilder.h"

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_vulkan.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <locale>
#include <optional>
#include <stdexcept>
#include <string>

namespace {

void writeMat4(float* dst, const glm::mat4& m) {
  std::memcpy(dst, glm::value_ptr(m), sizeof(float) * 16);
}

void writeVec3(float* dst, const glm::vec3& v) {
  dst[0] = v.x;
  dst[1] = v.y;
  dst[2] = v.z;
}

void drawCornerNormals(VoxelScene& scene, const glm::mat4& viewProj, ImVec2 display) {
  std::vector<physics::DebugCornerNormal> corners;
  scene.gatherCornerNormals(1, 0, corners);
  ImDrawList* dl = ImGui::GetForegroundDrawList();
  auto project = [&](const glm::vec3& world, ImVec2& out) -> bool {
    const glm::vec4 clip = viewProj * glm::vec4(world, 1.0f);
    if (clip.w <= 1.0e-4f) {
      return false;
    }
    const float iw = 1.0f / clip.w;
    const float nx = clip.x * iw;
    const float ny = clip.y * iw;
    if (nx < -1.2f || nx > 1.2f || ny < -1.2f || ny > 1.2f) {
      return false;
    }
    out.x = (nx * 0.5f + 0.5f) * display.x;
    out.y = (ny * 0.5f + 0.5f) * display.y;
    return true;
  };
  constexpr float kLen = 1.5f;
  const float midY = scene.cpuObject(1).position.y;
  for (int i = 0; i < static_cast<int>(corners.size()); ++i) {
    const physics::DebugCornerNormal& cn = corners[static_cast<size_t>(i)];
    ImVec2 a{};
    if (!project(cn.p, a)) {
      continue;
    }
    const char* tb = cn.p.y < midY ? "bot" : "top";
    dl->AddCircleFilled(a, 5.0f, IM_COL32(255, 255, 255, 255));
    if (!cn.hit) {
      dl->AddCircle(a, 9.0f, IM_COL32(160, 160, 160, 255), 0, 2.0f);
      char miss[40];
      std::snprintf(miss, sizeof(miss), "#%d %s miss", i, tb);
      dl->AddText(ImVec2(a.x + 8.0f, a.y - 8.0f), IM_COL32(180, 180, 180, 255), miss);
      continue;
    }
    ImVec2 b{};
    if (!project(cn.p + cn.n * kLen, b)) {
      continue;
    }
    const ImU32 col = cn.n.y >= 0.5f ? IM_COL32(50, 255, 80, 255)
                    : (cn.n.y <= -0.2f ? IM_COL32(255, 50, 50, 255) : IM_COL32(255, 220, 40, 255));
    dl->AddLine(a, b, col, 3.0f);
    dl->AddCircleFilled(b, 4.0f, col);
    char buf[112];
    std::snprintf(buf, sizeof(buf), "#%d %s n=(%.2f,%.2f,%.2f) d=%.3f", i, tb, cn.n.x, cn.n.y,
                  cn.n.z, cn.d);
    dl->AddText(ImVec2(b.x + 6.0f, b.y - 8.0f), col, buf);
  }
  // Magenta = contacts the solver actually used (may differ from the 8 probes).
  for (const physics::Contact& c : scene.physicsDebug().lastContacts) {
    ImVec2 a{};
    ImVec2 b{};
    if (!project(c.p, a) || !project(c.p + c.n * (kLen * 0.7f), b)) {
      continue;
    }
    dl->AddLine(a, b, IM_COL32(255, 80, 255, 255), 5.0f);
    dl->AddCircleFilled(a, 7.0f, IM_COL32(255, 80, 255, 220));
  }
}

void drawBondStress(VoxelScene& scene, const glm::mat4& viewProj, ImVec2 display) {
  ImDrawList* dl = ImGui::GetForegroundDrawList();
  auto project = [&](const glm::vec3& world, ImVec2& out) -> bool {
    const glm::vec4 clip = viewProj * glm::vec4(world, 1.0f);
    if (clip.w <= 1.0e-4f) {
      return false;
    }
    const float iw = 1.0f / clip.w;
    const float nx = clip.x * iw;
    const float ny = clip.y * iw;
    if (nx < -1.2f || nx > 1.2f || ny < -1.2f || ny > 1.2f) {
      return false;
    }
    out.x = (nx * 0.5f + 0.5f) * display.x;
    out.y = (ny * 0.5f + 0.5f) * display.y;
    return true;
  };
  int drawn = 0;
  for (const physics::DebugBond& b : scene.structureDebugBonds()) {
    if (drawn >= 8000) {
      break;
    }
    ImVec2 a{};
    ImVec2 p1{};
    if (!project(b.a, a) || !project(b.b, p1)) {
      continue;
    }
    const float t = std::clamp(b.phi, 0.0f, 1.0f);
    const int r = static_cast<int>(40.0f + 215.0f * t);
    const int g = static_cast<int>(220.0f * (1.0f - t));
    const int alpha = b.alive ? (b.phi >= 0.35f ? 230 : 160) : 100;
    const ImU32 col = b.alive ? IM_COL32(r, g, 40, alpha) : IM_COL32(80, 80, 80, 100);
    const float thickness = !b.alive ? 1.0f : (b.phi >= 1.0f ? 4.0f : (b.phi >= 0.5f ? 2.5f : 1.5f));
    dl->AddLine(a, p1, col, thickness);
    ++drawn;
  }
}

void printPipelineExecutableStatistics(VkDevice device, VkPipeline pipeline) {
  const auto getProperties = reinterpret_cast<PFN_vkGetPipelineExecutablePropertiesKHR>(
      vkGetDeviceProcAddr(device, "vkGetPipelineExecutablePropertiesKHR"));
  const auto getStatistics = reinterpret_cast<PFN_vkGetPipelineExecutableStatisticsKHR>(
      vkGetDeviceProcAddr(device, "vkGetPipelineExecutableStatisticsKHR"));
  if (!getProperties || !getStatistics) {
    std::cerr << "[VoxelPipelineStats] query entry points unavailable\n";
    return;
  }

  auto enumerate = [](auto query, auto& values, VkStructureType type) -> VkResult {
    for (int attempt = 0; attempt < 4; ++attempt) {
      uint32_t count = 0;
      VkResult status = query(&count, nullptr);
      if (status != VK_SUCCESS) {
        return status;
      }
      values.assign(count, {});
      for (auto& value : values) {
        value.sType = type;
      }
      if (count == 0) {
        return VK_SUCCESS;
      }
      status = query(&count, values.data());
      if (status == VK_INCOMPLETE) {
        continue;
      }
      if (status == VK_SUCCESS) {
        values.resize(count);
      }
      return status;
    }
    return VK_INCOMPLETE;
  };

  VkPipelineInfoKHR pipelineInfo{};
  pipelineInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INFO_KHR;
  pipelineInfo.pipeline = pipeline;
  std::vector<VkPipelineExecutablePropertiesKHR> executables;
  const VkResult propertiesStatus = enumerate(
      [&](uint32_t* count, VkPipelineExecutablePropertiesKHR* values) {
        return getProperties(device, &pipelineInfo, count, values);
      }, executables, VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_PROPERTIES_KHR);
  if (propertiesStatus != VK_SUCCESS) {
    std::cerr << "[VoxelPipelineStats] executable enumeration failed: VkResult="
              << propertiesStatus << '\n';
    return;
  }
  std::cout << "[VoxelPipelineStats] executables=" << executables.size() << '\n';
  for (uint32_t index = 0; index < executables.size(); ++index) {
    const auto& executable = executables[index];
    std::cout << "[VoxelPipelineStats] executable=" << index << " name=" << executable.name
              << " stages=" << executable.stages << " subgroupSize=" << executable.subgroupSize
              << " description=" << executable.description << '\n';

    VkPipelineExecutableInfoKHR executableInfo{};
    executableInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_INFO_KHR;
    executableInfo.pipeline = pipeline;
    executableInfo.executableIndex = index;
    std::vector<VkPipelineExecutableStatisticKHR> statistics;
    const VkResult statisticsStatus = enumerate(
        [&](uint32_t* count, VkPipelineExecutableStatisticKHR* values) {
          return getStatistics(device, &executableInfo, count, values);
        }, statistics, VK_STRUCTURE_TYPE_PIPELINE_EXECUTABLE_STATISTIC_KHR);
    if (statisticsStatus != VK_SUCCESS) {
      std::cerr << "[VoxelPipelineStats] executable=" << index
                << " statistics enumeration failed: VkResult=" << statisticsStatus << '\n';
      continue;
    }
    std::cout << "[VoxelPipelineStats] executable=" << index
              << " statistics=" << statistics.size() << '\n';
    for (const auto& statistic : statistics) {
      std::cout << "[VoxelPipelineStats] executable=" << index << " name=" << statistic.name
                << " value=";
      switch (statistic.format) {
        case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_BOOL32_KHR:
          std::cout << "BOOL32:" << (statistic.value.b32 ? "true" : "false");
          break;
        case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_INT64_KHR:
          std::cout << "INT64:" << statistic.value.i64;
          break;
        case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_UINT64_KHR:
          std::cout << "UINT64:" << statistic.value.u64;
          break;
        case VK_PIPELINE_EXECUTABLE_STATISTIC_FORMAT_FLOAT64_KHR: {
          const auto flags = std::cout.flags();
          const auto precision = std::cout.precision(17);
          std::cout << "FLOAT64:" << std::defaultfloat << statistic.value.f64;
          std::cout.flags(flags);
          std::cout.precision(precision);
          break;
        }
        default:
          std::cout << "unknown-format:" << statistic.format;
          break;
      }
      std::cout << " description=" << statistic.description << '\n';
    }
  }
}

}  // namespace

VoxelRenderer::VoxelRenderer(GfxDevice& gfx) : gfx_(gfx) {
  const char* generic = std::getenv("VE_VOXEL_GENERIC_SHADER");
  forceGenericShader_ = generic && std::strcmp(generic, "1") == 0;
}

void VoxelRenderer::configureBenchmark(const BenchmarkSettings& settings, uint32_t warmup,
                                       uint32_t frames) {
  if (imguiReady_ || timestampPool_ || frames == 0 || settings.stage > kStageSkipDda) {
    throw std::runtime_error("Invalid benchmark configuration (must configure before init)");
  }
  benchmark_ = true;
  benchmarkWarmup_ = warmup;
  benchmarkFrames_ = frames;
  benchmarkTimings_.reserve(frames);
  beamSkip_ = settings.beam;
  brickBitSkip_ = settings.brickSkip;
  dirMaskBrick_ = settings.dirBrick;
  dirMaskCoarse_ = settings.dirCoarse;
  traceStage_ = static_cast<int>(settings.stage);
}

VoxelRenderer::~VoxelRenderer() {
  gfx_.waitIdle();
  shutdownImGui();
  destroyOutputImage();
  destroyTimestampPool();

  if (computePipeline_) {
    vkDestroyPipeline(gfx_.device(), computePipeline_, nullptr);
  }
  if (specializedFullPipeline_) {
    vkDestroyPipeline(gfx_.device(), specializedFullPipeline_, nullptr);
  }
  if (specializedSinglePipeline_) {
    vkDestroyPipeline(gfx_.device(), specializedSinglePipeline_, nullptr);
  }
  if (specializedColorPipeline_) {
    vkDestroyPipeline(gfx_.device(), specializedColorPipeline_, nullptr);
  }
  if (slimPipeline_) {
    vkDestroyPipeline(gfx_.device(), slimPipeline_, nullptr);
  }
  if (coarsePipeline_) {
    vkDestroyPipeline(gfx_.device(), coarsePipeline_, nullptr);
  }
  if (beamPipeline_) {
    vkDestroyPipeline(gfx_.device(), beamPipeline_, nullptr);
  }
  if (visPipeline_) {
    vkDestroyPipeline(gfx_.device(), visPipeline_, nullptr);
  }
  if (visPipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), visPipelineLayout_, nullptr);
  }
  if (pipelineLayout_) {
    vkDestroyPipelineLayout(gfx_.device(), pipelineLayout_, nullptr);
  }
  gfx_.destroyImage(dummyBeamImage_);
  gfx_.destroyImage(dummyVisImage_);
  for (auto& frame : frames_) {
    gfx_.destroyBuffer(frame.frameUBO);
  }
  if (descriptorPool_) {
    vkDestroyDescriptorPool(gfx_.device(), descriptorPool_, nullptr);
  }
  if (frameLayout_) {
    vkDestroyDescriptorSetLayout(gfx_.device(), frameLayout_, nullptr);
  }
}

void VoxelRenderer::init(VoxelScene& scene) {
  dummyBeamImage_ = gfx_.createImage({1, 1, 1}, kBeamFormat, VK_IMAGE_USAGE_STORAGE_BIT,
                                     VK_IMAGE_ASPECT_COLOR_BIT, true);
  dummyVisImage_ = gfx_.createImage({1, 1, 1}, kVisFormat, VK_IMAGE_USAGE_STORAGE_BIT,
                                    VK_IMAGE_ASPECT_COLOR_BIT, true);
  createDescriptors();
  createOutputImage();
  createTimestampPool();
  createPipelines();

  for (auto& frame : frames_) {
    frame.frameUBO = gfx_.createBuffer(sizeof(VoxelDdaUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                       VMA_MEMORY_USAGE_AUTO_PREFER_HOST);

    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = descriptorPool_;
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &frameLayout_;
    if (vkAllocateDescriptorSets(gfx_.device(), &allocInfo, &frame.frameSet) != VK_SUCCESS) {
      throw std::runtime_error("Failed to allocate voxel DDA descriptor set");
    }
  }

  updateDescriptors(scene);
  initImGui();
}

void VoxelRenderer::createTimestampPool() {
  VkPhysicalDeviceProperties props{};
  vkGetPhysicalDeviceProperties(gfx_.physicalDevice(), &props);
  timestampPeriodNs_ = props.limits.timestampPeriod;
  uint32_t familyCount = 0;
  vkGetPhysicalDeviceQueueFamilyProperties(gfx_.physicalDevice(), &familyCount, nullptr);
  std::vector<VkQueueFamilyProperties> families(familyCount);
  vkGetPhysicalDeviceQueueFamilyProperties(gfx_.physicalDevice(), &familyCount, families.data());
  const uint32_t validBits = families.at(gfx_.graphicsQueueFamily()).timestampValidBits;
  if (validBits == 0 || timestampPeriodNs_ <= 0.0f ||
      (benchmark_ && !props.limits.timestampComputeAndGraphics)) {
    throw std::runtime_error("GPU queue does not support the required benchmark timestamps");
  }
  timestampMask_ = validBits >= 64 ? UINT64_MAX : ((uint64_t{1} << validBits) - 1);

  VkQueryPoolCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
  info.queryType = VK_QUERY_TYPE_TIMESTAMP;
  info.queryCount = GfxDevice::kFramesInFlight * kTsPerFrame;
  if (vkCreateQueryPool(gfx_.device(), &info, nullptr, &timestampPool_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel timestamp query pool");
  }
  timestampPending_.fill(false);
}

void VoxelRenderer::destroyTimestampPool() {
  if (timestampPool_) {
    vkDestroyQueryPool(gfx_.device(), timestampPool_, nullptr);
    timestampPool_ = VK_NULL_HANDLE;
  }
}

void VoxelRenderer::writeTimestamp(VkCommandBuffer cmd, uint32_t queryIndex,
                                   VkPipelineStageFlags2 stage) const {
  if (!timestampPool_) {
    return;
  }
  vkCmdWriteTimestamp2(cmd, stage, timestampPool_, queryIndex);
}

void VoxelRenderer::collectGpuTiming(uint32_t frameIndex) {
  if (!timestampPool_ || !timestampPending_[frameIndex]) {
    return;
  }

  const uint32_t firstQuery = frameIndex * kTsPerFrame;
  uint64_t stamps[kTsPerFrame] = {};
  const VkResult result = vkGetQueryPoolResults(
      gfx_.device(), timestampPool_, firstQuery, kTsPerFrame, sizeof(stamps), stamps,
      sizeof(uint64_t), VK_QUERY_RESULT_64_BIT);
  if (result != VK_SUCCESS) {
    // Called only after this slot's fence, or device idle. Never reset an unread query.
    throw std::runtime_error("Failed to collect completed voxel GPU timestamps: " +
                             std::to_string(result));
  }

  auto toMs = [&](uint64_t a, uint64_t b) {
    const double deltaTicks = static_cast<double>((b - a) & timestampMask_);
    return deltaTicks * static_cast<double>(timestampPeriodNs_) * 1e-6;
  };

  const GpuTiming timing{
      timestampSubmissions_[frameIndex],
      toMs(stamps[kTsFrameBegin], stamps[kTsFrameEnd]),
      toMs(stamps[kTsFrameBegin], stamps[kTsAfterBeam]),
      toMs(stamps[kTsAfterBeam], stamps[kTsAfterCompute]),
      toMs(stamps[kTsFrameBegin], stamps[kTsAfterCompute]),
      toMs(stamps[kTsAfterCompute], stamps[kTsAfterBlit]),
      toMs(stamps[kTsAfterBlit], stamps[kTsFrameEnd]),
  };
  if (benchmark_ && timing.submission >= benchmarkWarmup_ &&
      timing.submission < uint64_t{benchmarkWarmup_} + benchmarkFrames_) {
    benchmarkTimings_.push_back(timing);
  }

  constexpr float alpha = 0.15f;
  gpuFrameMs_ = gpuFrameMs_ * (1.0f - alpha) + static_cast<float>(timing.totalMs) * alpha;
  gpuBeamMs_ = gpuBeamMs_ * (1.0f - alpha) + static_cast<float>(timing.beamMs) * alpha;
  gpuMainMs_ = gpuMainMs_ * (1.0f - alpha) + static_cast<float>(timing.mainMs) * alpha;
  gpuComputeMs_ = gpuComputeMs_ * (1.0f - alpha) + static_cast<float>(timing.computeMs) * alpha;
  gpuBlitMs_ = gpuBlitMs_ * (1.0f - alpha) + static_cast<float>(timing.blitMs) * alpha;
  gpuUiMs_ = gpuUiMs_ * (1.0f - alpha) + static_cast<float>(timing.uiMs) * alpha;
  timestampPending_[frameIndex] = false;
}

std::vector<VoxelRenderer::GpuTiming> VoxelRenderer::finishBenchmark() {
  if (!benchmark_ || submittedFrames_ != uint64_t{benchmarkWarmup_} + benchmarkFrames_) {
    throw std::runtime_error("Benchmark did not submit exactly warmup + measured frames");
  }
  if (vkDeviceWaitIdle(gfx_.device()) != VK_SUCCESS) {
    throw std::runtime_error("GPU failed while draining benchmark frames");
  }
  for (uint32_t i = 0; i < GfxDevice::kFramesInFlight; ++i) {
    collectGpuTiming(i);
  }
  std::sort(benchmarkTimings_.begin(), benchmarkTimings_.end(),
            [](const GpuTiming& a, const GpuTiming& b) { return a.submission < b.submission; });
  if (benchmarkTimings_.size() != benchmarkFrames_) {
    throw std::runtime_error("Benchmark GPU timing sample count mismatch");
  }
  for (size_t i = 0; i < benchmarkTimings_.size(); ++i) {
    if (benchmarkTimings_[i].submission != uint64_t{benchmarkWarmup_} + i) {
      throw std::runtime_error("Benchmark GPU timing submissions are not contiguous");
    }
  }
  return benchmarkTimings_;
}

void VoxelRenderer::capturePpm(const std::string& path) {
  if (!outImage_.image || outImage_.layout != VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL) {
    throw std::runtime_error("Capture requires a completed rendered output image");
  }
  if (vkDeviceWaitIdle(gfx_.device()) != VK_SUCCESS) {
    throw std::runtime_error("GPU failed before benchmark capture");
  }
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  file.imbue(std::locale::classic());
  if (!file) {
    throw std::runtime_error("Cannot open benchmark capture: " + path);
  }
  const uint32_t width = outImage_.extent.width;
  const uint32_t height = outImage_.extent.height;
  const VkDeviceSize bytes = VkDeviceSize{width} * height * 4;
  AllocatedBuffer staging = gfx_.createBuffer(bytes, VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                               VMA_MEMORY_USAGE_AUTO_PREFER_HOST);
  try {
    gfx_.immediateSubmit([&](VkCommandBuffer cmd) {
      gfx_.transitionImage(cmd, outImage_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_MEMORY_WRITE_BIT,
                           VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
      VkBufferImageCopy region{};
      region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
      region.imageSubresource.layerCount = 1;
      region.imageExtent = {width, height, 1};
      vkCmdCopyImageToBuffer(cmd, outImage_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                             staging.buffer, 1, &region);
      VkMemoryBarrier2 barrier{VK_STRUCTURE_TYPE_MEMORY_BARRIER_2};
      barrier.srcStageMask = VK_PIPELINE_STAGE_2_TRANSFER_BIT;
      barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
      barrier.dstStageMask = VK_PIPELINE_STAGE_2_HOST_BIT;
      barrier.dstAccessMask = VK_ACCESS_2_HOST_READ_BIT;
      VkDependencyInfo dependency{VK_STRUCTURE_TYPE_DEPENDENCY_INFO};
      dependency.memoryBarrierCount = 1;
      dependency.pMemoryBarriers = &barrier;
      vkCmdPipelineBarrier2(cmd, &dependency);
    });
    if (vkQueueWaitIdle(gfx_.graphicsQueue()) != VK_SUCCESS || !staging.info.pMappedData ||
        vmaInvalidateAllocation(gfx_.allocator(), staging.allocation, 0, bytes) != VK_SUCCESS) {
      throw std::runtime_error("Failed to read benchmark RGBA output");
    }
    file << "P6\n" << width << ' ' << height << "\n255\n";
    const auto* rgba = static_cast<const uint8_t*>(staging.info.pMappedData);
    std::vector<char> row(static_cast<size_t>(width) * 3);
    for (uint32_t y = 0; y < height; ++y) {
      for (uint32_t x = 0; x < width; ++x) {
        const size_t src = (static_cast<size_t>(y) * width + x) * 4;
        std::memcpy(row.data() + static_cast<size_t>(x) * 3, rgba + src, 3);
      }
      file.write(row.data(), static_cast<std::streamsize>(row.size()));
    }
    file.close();
    if (!file) {
      throw std::runtime_error("Failed to write benchmark capture: " + path);
    }
  } catch (...) {
    gfx_.destroyBuffer(staging);
    throw;
  }
  gfx_.destroyBuffer(staging);
}

void VoxelRenderer::resize() {
  destroyOutputImage();
  createOutputImage();
  // Force descriptor refresh so storage-image views stay valid.
  boundBrickSlabs_.fill(VK_NULL_HANDLE);
  boundBrickSlabCount_ = 0;
  boundObjectBuffer_ = VK_NULL_HANDLE;
  boundCoarsePoolBuffer_ = VK_NULL_HANDLE;
  boundPaletteBuffer_ = VK_NULL_HANDLE;
  boundOccMipBuffer_ = VK_NULL_HANDLE;
  boundHeatmapBuffer_ = VK_NULL_HANDLE;
  boundSkyView_ = VK_NULL_HANDLE;
  boundBeamView_ = VK_NULL_HANDLE;
  boundVisViews_.fill(VK_NULL_HANDLE);
}

void VoxelRenderer::createDescriptors() {
  VkDescriptorSetLayoutBinding bindings[11]{};
  bindings[0].binding = 0;
  bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
  bindings[0].descriptorCount = 1;
  bindings[0].stageFlags =
      VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

  bindings[1].binding = 1;
  bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[1].descriptorCount = 1;
  bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[2].binding = 2;
  bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  bindings[2].descriptorCount = 1;
  bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[3].binding = 3;
  bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[3].descriptorCount = VoxelScene::kMaxBrickSlabs;
  bindings[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[4].binding = 4;
  bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
  bindings[4].descriptorCount = 1;
  bindings[4].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[5].binding = 5;
  bindings[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[5].descriptorCount = 1;
  bindings[5].stageFlags =
      VK_SHADER_STAGE_COMPUTE_BIT | VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

  bindings[6].binding = 6;
  bindings[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[6].descriptorCount = 1;
  bindings[6].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[7].binding = 7;
  bindings[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[7].descriptorCount = 1;
  bindings[7].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[8].binding = 8;
  bindings[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  bindings[8].descriptorCount = 1;
  bindings[8].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[9].binding = 9;
  bindings[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
  bindings[9].descriptorCount = 1;
  bindings[9].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  bindings[10].binding = 10;
  bindings[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
  bindings[10].descriptorCount = 1;
  bindings[10].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

  VkDescriptorSetLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
  layoutInfo.bindingCount = 11;
  layoutInfo.pBindings = bindings;
  if (vkCreateDescriptorSetLayout(gfx_.device(), &layoutInfo, nullptr, &frameLayout_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel DDA set layout");
  }

  VkDescriptorPoolSize poolSizes[] = {
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, GfxDevice::kFramesInFlight},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
       GfxDevice::kFramesInFlight * (5u + VoxelScene::kMaxBrickSlabs)},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, GfxDevice::kFramesInFlight * 3u},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, GfxDevice::kFramesInFlight},
  };
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.maxSets = GfxDevice::kFramesInFlight;
  poolInfo.poolSizeCount = static_cast<uint32_t>(std::size(poolSizes));
  poolInfo.pPoolSizes = poolSizes;
  if (vkCreateDescriptorPool(gfx_.device(), &poolInfo, nullptr, &descriptorPool_) != VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel DDA descriptor pool");
  }
}

void VoxelRenderer::createOutputImage() {
  const VkExtent2D ext = gfx_.swapchainExtent();
  if (ext.width == 0 || ext.height == 0) {
    return;
  }
  outImage_ = gfx_.createImage({ext.width, ext.height, 1}, kOutFormat,
                               VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
                               VK_IMAGE_ASPECT_COLOR_BIT, true);
  const uint32_t beamW = std::max(1u, (ext.width + 7u) / 8u);
  const uint32_t beamH = std::max(1u, (ext.height + 7u) / 8u);
  beamImage_ = gfx_.createImage({beamW, beamH, 1}, kBeamFormat, VK_IMAGE_USAGE_STORAGE_BIT,
                                VK_IMAGE_ASPECT_COLOR_BIT, true);
  for (uint32_t i = 0; i < GfxDevice::kFramesInFlight; ++i) {
    visImages_[i] = gfx_.createImage(
        {ext.width, ext.height, 1}, kVisFormat,
        VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT |
            VK_IMAGE_USAGE_TRANSFER_DST_BIT,
        VK_IMAGE_ASPECT_COLOR_BIT, true);
    visDepths_[i] = gfx_.createImage({ext.width, ext.height, 1}, kVisDepthFormat,
                                     VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
                                     VK_IMAGE_ASPECT_DEPTH_BIT, true);
  }
}

void VoxelRenderer::destroyOutputImage() {
  gfx_.destroyImage(outImage_);
  gfx_.destroyImage(beamImage_);
  for (AllocatedImage& img : visImages_) {
    gfx_.destroyImage(img);
  }
  for (AllocatedImage& img : visDepths_) {
    gfx_.destroyImage(img);
  }
}

VkPipeline VoxelRenderer::createComputePipeline(VkShaderModule shader, DdaSpec spec,
                                                bool useSpec) const {
  VkSpecializationMapEntry specEntries[8]{};
  specEntries[0].constantID = 0;
  specEntries[0].offset = offsetof(DdaSpec, beamPass);
  specEntries[0].size = sizeof(uint32_t);
  specEntries[1].constantID = 1;
  specEntries[1].offset = offsetof(DdaSpec, enableNested);
  specEntries[1].size = sizeof(uint32_t);
  specEntries[2].constantID = 2;
  specEntries[2].offset = offsetof(DdaSpec, enableShade);
  specEntries[2].size = sizeof(uint32_t);
  specEntries[3].constantID = 3;
  specEntries[3].offset = offsetof(DdaSpec, shadedOnly);
  specEntries[3].size = sizeof(uint32_t);
  specEntries[4].constantID = 4;
  specEntries[4].offset = offsetof(DdaSpec, singleObject);
  specEntries[4].size = sizeof(uint32_t);
  specEntries[5].constantID = 5;
  specEntries[5].offset = offsetof(DdaSpec, groupHeight);
  specEntries[5].size = sizeof(uint32_t);
  specEntries[6].constantID = 6;
  specEntries[6].offset = offsetof(DdaSpec, fineOnly);
  specEntries[6].size = sizeof(uint32_t);
  specEntries[7].constantID = 7;
  specEntries[7].offset = offsetof(DdaSpec, colorMode);
  specEntries[7].size = sizeof(uint32_t);

  VkSpecializationInfo specInfo{};
  specInfo.mapEntryCount = 8;
  specInfo.pMapEntries = specEntries;
  specInfo.dataSize = sizeof(DdaSpec);
  specInfo.pData = &spec;

  VkPipelineShaderStageCreateInfo stage{};
  stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  stage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
  stage.module = shader;
  stage.pName = "main";
  if (useSpec) {
    stage.pSpecializationInfo = &specInfo;
  }

  VkComputePipelineCreateInfo pipeInfo{};
  pipeInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
  pipeInfo.stage = stage;
  pipeInfo.layout = pipelineLayout_;
  if (gfx_.pipelineExecutableStatisticsEnabled()) {
    pipeInfo.flags |= VK_PIPELINE_CREATE_CAPTURE_STATISTICS_BIT_KHR;
  }

  VkPipeline pipeline = VK_NULL_HANDLE;
  if (vkCreateComputePipelines(gfx_.device(), VK_NULL_HANDLE, 1, &pipeInfo, nullptr, &pipeline) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create voxel DDA compute pipeline");
  }
  if (gfx_.pipelineExecutableStatisticsEnabled()) {
    std::cout << "[VoxelPipelineStats] pipeline=" << pipeline << " shader=" << shader
              << " useSpec=" << useSpec << " beamPass=" << spec.beamPass
              << " shadedOnly=" << spec.shadedOnly << " singleObject=" << spec.singleObject
              << " groupHeight=" << spec.groupHeight << '\n';
    printPipelineExecutableStatistics(gfx_.device(), pipeline);
  }
  return pipeline;
}

void VoxelRenderer::createPipelines() {
  const std::string shaderDir = VE_SHADER_DIR;
  VkShaderModule comp = gfx_.loadShaderModule(
      shaderDir + (gfx_.storageBufferNonUniformIndexing() ? "/voxel_dda_indexed.comp.spv"
                                                       : "/voxel_dda.comp.spv"));

  VkPipelineLayoutCreateInfo layoutInfo{};
  layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  layoutInfo.setLayoutCount = 1;
  layoutInfo.pSetLayouts = &frameLayout_;
  if (vkCreatePipelineLayout(gfx_.device(), &layoutInfo, nullptr, &pipelineLayout_) != VK_SUCCESS) {
    vkDestroyShaderModule(gfx_.device(), comp, nullptr);
    throw std::runtime_error("Failed to create voxel DDA pipeline layout");
  }

  VkShaderModule coarseMod = VK_NULL_HANDLE;
  VkShaderModule slimMod = VK_NULL_HANDLE;
  try {
    computePipeline_ = createComputePipeline(comp, DdaSpec{0u, 1u, 1u, 0u, 0u});
    specializedFullPipeline_ = createComputePipeline(comp, DdaSpec{0u, 1u, 1u, 1u, 0u, 8u});
    specializedSinglePipeline_ = createComputePipeline(comp, DdaSpec{0u, 1u, 1u, 1u, 1u, 8u, 1u, 0u});
    specializedColorPipeline_ = createComputePipeline(comp, DdaSpec{0u, 1u, 1u, 1u, 1u, 8u, 1u, 1u});
    beamPipeline_ = createComputePipeline(comp, DdaSpec{1u, 0u, 0u});
    coarseMod = gfx_.loadShaderModule(shaderDir + "/voxel_dda_coarse.comp.spv");
    slimMod = gfx_.loadShaderModule(shaderDir + "/voxel_dda_slim.comp.spv");
    coarsePipeline_ = createComputePipeline(coarseMod, DdaSpec{}, false);
    slimPipeline_ = createComputePipeline(slimMod, DdaSpec{}, false);
  } catch (...) {
    if (slimMod) {
      vkDestroyShaderModule(gfx_.device(), slimMod, nullptr);
    }
    if (coarseMod) {
      vkDestroyShaderModule(gfx_.device(), coarseMod, nullptr);
    }
    vkDestroyShaderModule(gfx_.device(), comp, nullptr);
    throw;
  }

  if (slimMod) {
    vkDestroyShaderModule(gfx_.device(), slimMod, nullptr);
  }
  if (coarseMod) {
    vkDestroyShaderModule(gfx_.device(), coarseMod, nullptr);
  }
  vkDestroyShaderModule(gfx_.device(), comp, nullptr);

  VkPushConstantRange visPush{};
  visPush.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
  visPush.offset = 0;
  visPush.size = 128;
  VkPipelineLayoutCreateInfo visLayoutInfo{};
  visLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
  visLayoutInfo.setLayoutCount = 1;
  visLayoutInfo.pSetLayouts = &frameLayout_;
  visLayoutInfo.pushConstantRangeCount = 1;
  visLayoutInfo.pPushConstantRanges = &visPush;
  if (vkCreatePipelineLayout(gfx_.device(), &visLayoutInfo, nullptr, &visPipelineLayout_) !=
      VK_SUCCESS) {
    throw std::runtime_error("Failed to create vis pipeline layout");
  }

  VkShaderModule visVert = gfx_.loadShaderModule(shaderDir + "/object_obb.vert.spv");
  VkShaderModule visFrag = gfx_.loadShaderModule(shaderDir + "/object_obb.frag.spv");
  VkVertexInputBindingDescription visBind{};
  visBind.binding = 0;
  visBind.stride = static_cast<uint32_t>(sizeof(VisCoarseInstance));
  visBind.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;
  VkVertexInputAttributeDescription visAttrs[2]{};
  visAttrs[0].location = 0;
  visAttrs[0].binding = 0;
  visAttrs[0].format = VK_FORMAT_R32_UINT;
  visAttrs[0].offset = offsetof(VisCoarseInstance, gpuIndex);
  visAttrs[1].location = 1;
  visAttrs[1].binding = 0;
  visAttrs[1].format = VK_FORMAT_R32_UINT;
  visAttrs[1].offset = offsetof(VisCoarseInstance, packedCoarse);
  visPipeline_ = PipelineBuilder()
                     .setShaders(visVert, visFrag)
                     .setVertexInput(visBind, {visAttrs[0], visAttrs[1]})
                     .setTopology(VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST)
                     .setCullMode(VK_CULL_MODE_NONE, VK_FRONT_FACE_COUNTER_CLOCKWISE)
                     .setMultisampling(VK_SAMPLE_COUNT_1_BIT)
                     .setDepthTest(true, true, VK_COMPARE_OP_GREATER_OR_EQUAL)
                     .setColorFormat(kVisFormat)
                     .setColorBlend(false)
                     .setDepthFormat(kVisDepthFormat)
                     .setLayout(visPipelineLayout_)
                     .build(gfx_.device());
  vkDestroyShaderModule(gfx_.device(), visVert, nullptr);
  vkDestroyShaderModule(gfx_.device(), visFrag, nullptr);
}

void VoxelRenderer::recordVisPass(VkCommandBuffer cmd, VoxelScene& scene, VkExtent2D extent,
                                 VkDescriptorSet frameSet, uint32_t frameIndex) {
  AllocatedImage& vis = visImages_[frameIndex];
  AllocatedImage& depth = visDepths_[frameIndex];
  gfx_.transitionImage(cmd, vis.image, vis.layout, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       vis.layout == VK_IMAGE_LAYOUT_UNDEFINED
                           ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
                           : VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                       vis.layout == VK_IMAGE_LAYOUT_UNDEFINED ? 0 : VK_ACCESS_2_SHADER_READ_BIT,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
  gfx_.transitionImage(cmd, depth.image, depth.layout, VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL,
                       depth.layout == VK_IMAGE_LAYOUT_UNDEFINED
                           ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
                           : VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                       depth.layout == VK_IMAGE_LAYOUT_UNDEFINED
                           ? 0
                           : VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT,
                       VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                           VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT,
                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
                           VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT,
                       VK_IMAGE_ASPECT_DEPTH_BIT);

  VkClearValue colorClear{};
  colorClear.color.float32[0] = 0.0f;
  colorClear.color.float32[1] = 0.0f;
  colorClear.color.float32[2] = 0.0f;
  colorClear.color.float32[3] = 1.0f;
  VkClearValue depthClear{};
  depthClear.depthStencil.depth = 0.0f;

  VkRenderingAttachmentInfo colorAtt{};
  colorAtt.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  colorAtt.imageView = vis.view;
  colorAtt.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  colorAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  colorAtt.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  colorAtt.clearValue = colorClear;

  VkRenderingAttachmentInfo depthAtt{};
  depthAtt.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  depthAtt.imageView = depth.view;
  depthAtt.imageLayout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
  depthAtt.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  depthAtt.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  depthAtt.clearValue = depthClear;

  VkRenderingInfo info{};
  info.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  info.renderArea.extent = extent;
  info.layerCount = 1;
  info.colorAttachmentCount = 1;
  info.pColorAttachments = &colorAtt;
  info.pDepthAttachment = &depthAtt;
  vkCmdBeginRendering(cmd, &info);

  VkViewport vp{0.0f, 0.0f, static_cast<float>(extent.width), static_cast<float>(extent.height),
                0.0f, 1.0f};
  VkRect2D scissor{{0, 0}, extent};
  vkCmdSetViewport(cmd, 0, 1, &vp);
  vkCmdSetScissor(cmd, 0, 1, &scissor);
  vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, visPipeline_);
  vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, visPipelineLayout_, 0, 1,
                          &frameSet, 0, nullptr);
  float push[32];
  writeMat4(push, scene.camera().view());
  writeMat4(push + 16, scene.camera().proj());
  vkCmdPushConstants(cmd, visPipelineLayout_,
                     VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT, 0, 128, push);
  if (scene.visInstanceCount() > 0 && scene.visInstanceBuffer().buffer != VK_NULL_HANDLE) {
    VkBuffer vb = scene.visInstanceBuffer().buffer;
    VkDeviceSize off = 0;
    vkCmdBindVertexBuffers(cmd, 0, 1, &vb, &off);
    vkCmdDraw(cmd, 36, scene.visInstanceCount(), 0, 0);
  }
  vkCmdEndRendering(cmd);

  gfx_.transitionImage(cmd, vis.image, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_READ_BIT);
  vis.layout = VK_IMAGE_LAYOUT_GENERAL;
  depth.layout = VK_IMAGE_LAYOUT_DEPTH_ATTACHMENT_OPTIMAL;
}

void VoxelRenderer::updateDescriptors(VoxelScene& scene) {
  if (outImage_.view == VK_NULL_HANDLE || scene.dummyBrickSlabBuffer().buffer == VK_NULL_HANDLE ||
      scene.objectBuffer().buffer == VK_NULL_HANDLE ||
      scene.coarsePoolBuffer().buffer == VK_NULL_HANDLE ||
      scene.paletteBuffer().buffer == VK_NULL_HANDLE ||
      scene.occMipBuffer().buffer == VK_NULL_HANDLE ||
      scene.heatmapBuffer().buffer == VK_NULL_HANDLE || !scene.hasSky() ||
      dummyBeamImage_.view == VK_NULL_HANDLE || dummyVisImage_.view == VK_NULL_HANDLE) {
    return;
  }

  // All frame sets are rewritten together, only on infrequent resource changes.
  gfx_.waitIdle();
  for (auto& frame : frames_) {
    VkDescriptorBufferInfo uboInfo{};
    uboInfo.buffer = frame.frameUBO.buffer;
    uboInfo.range = sizeof(VoxelDdaUBO);

    VkDescriptorBufferInfo poolInfo{};
    poolInfo.buffer = scene.coarsePoolBuffer().buffer;
    poolInfo.range = scene.coarsePoolBuffer().size;

    VkDescriptorImageInfo imageInfo{};
    imageInfo.imageView = outImage_.view;
    imageInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorBufferInfo brickInfos[VoxelScene::kMaxBrickSlabs]{};
    for (uint32_t i = 0; i < VoxelScene::kMaxBrickSlabs; ++i) {
      if (i < scene.brickSlabCount() && scene.brickSlabBuffer(i).buffer != VK_NULL_HANDLE) {
        brickInfos[i].buffer = scene.brickSlabBuffer(i).buffer;
        brickInfos[i].range = scene.brickSlabBuffer(i).size;
      } else {
        brickInfos[i].buffer = scene.dummyBrickSlabBuffer().buffer;
        brickInfos[i].range = scene.dummyBrickSlabBuffer().size;
      }
    }

    VkDescriptorImageInfo skyInfo{};
    skyInfo.sampler = scene.sky().sampler;
    skyInfo.imageView = scene.sky().image.view;
    skyInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkDescriptorBufferInfo objectInfo{};
    objectInfo.buffer = scene.objectBuffer().buffer;
    objectInfo.range = scene.objectBuffer().size;

    VkDescriptorBufferInfo paletteInfo{};
    paletteInfo.buffer = scene.paletteBuffer().buffer;
    paletteInfo.range = scene.paletteBuffer().size;

    VkDescriptorBufferInfo occMipInfo{};
    occMipInfo.buffer = scene.occMipBuffer().buffer;
    occMipInfo.range = scene.occMipBuffer().size;

    VkDescriptorBufferInfo heatmapInfo{};
    heatmapInfo.buffer = scene.heatmapBuffer().buffer;
    heatmapInfo.range = scene.heatmapBuffer().size;

    VkDescriptorImageInfo beamInfo{};
    beamInfo.imageView =
        beamImage_.view != VK_NULL_HANDLE ? beamImage_.view : dummyBeamImage_.view;
    beamInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkDescriptorImageInfo visInfo{};
    const uint32_t fi = static_cast<uint32_t>(&frame - frames_.data());
    visInfo.imageView =
        visImages_[fi].view != VK_NULL_HANDLE ? visImages_[fi].view : dummyVisImage_.view;
    visInfo.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

    VkWriteDescriptorSet writes[11]{};
    writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[0].dstSet = frame.frameSet;
    writes[0].dstBinding = 0;
    writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    writes[0].descriptorCount = 1;
    writes[0].pBufferInfo = &uboInfo;

    writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[1].dstSet = frame.frameSet;
    writes[1].dstBinding = 1;
    writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[1].descriptorCount = 1;
    writes[1].pBufferInfo = &poolInfo;

    writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[2].dstSet = frame.frameSet;
    writes[2].dstBinding = 2;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo = &imageInfo;

    writes[3].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[3].dstSet = frame.frameSet;
    writes[3].dstBinding = 3;
    writes[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[3].descriptorCount = VoxelScene::kMaxBrickSlabs;
    writes[3].pBufferInfo = brickInfos;

    writes[4].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[4].dstSet = frame.frameSet;
    writes[4].dstBinding = 4;
    writes[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[4].descriptorCount = 1;
    writes[4].pImageInfo = &skyInfo;

    writes[5].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[5].dstSet = frame.frameSet;
    writes[5].dstBinding = 5;
    writes[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[5].descriptorCount = 1;
    writes[5].pBufferInfo = &objectInfo;

    writes[6].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[6].dstSet = frame.frameSet;
    writes[6].dstBinding = 6;
    writes[6].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[6].descriptorCount = 1;
    writes[6].pBufferInfo = &paletteInfo;

    writes[7].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[7].dstSet = frame.frameSet;
    writes[7].dstBinding = 7;
    writes[7].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[7].descriptorCount = 1;
    writes[7].pBufferInfo = &occMipInfo;

    writes[8].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[8].dstSet = frame.frameSet;
    writes[8].dstBinding = 8;
    writes[8].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[8].descriptorCount = 1;
    writes[8].pImageInfo = &beamInfo;

    writes[9].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[9].dstSet = frame.frameSet;
    writes[9].dstBinding = 9;
    writes[9].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
    writes[9].descriptorCount = 1;
    writes[9].pImageInfo = &visInfo;

    writes[10].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[10].dstSet = frame.frameSet;
    writes[10].dstBinding = 10;
    writes[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
    writes[10].descriptorCount = 1;
    writes[10].pBufferInfo = &heatmapInfo;

    vkUpdateDescriptorSets(gfx_.device(), 11, writes, 0, nullptr);
  }

  boundBrickSlabCount_ = scene.brickSlabCount();
  boundBrickSlabs_.fill(VK_NULL_HANDLE);
  for (uint32_t i = 0; i < boundBrickSlabCount_; ++i) {
    boundBrickSlabs_[i] = scene.brickSlabBuffer(i).buffer;
  }
  boundObjectBuffer_ = scene.objectBuffer().buffer;
  boundCoarsePoolBuffer_ = scene.coarsePoolBuffer().buffer;
  boundPaletteBuffer_ = scene.paletteBuffer().buffer;
  boundOccMipBuffer_ = scene.occMipBuffer().buffer;
  boundHeatmapBuffer_ = scene.heatmapBuffer().buffer;
  boundSkyView_ = scene.sky().image.view;
  boundBeamView_ = beamImage_.view != VK_NULL_HANDLE ? beamImage_.view : dummyBeamImage_.view;
  for (uint32_t i = 0; i < GfxDevice::kFramesInFlight; ++i) {
    boundVisViews_[i] =
        visImages_[i].view != VK_NULL_HANDLE ? visImages_[i].view : dummyVisImage_.view;
  }
}

void VoxelRenderer::updateFrameUBO(VoxelScene& scene, uint32_t frameIndex) {
  const glm::mat4 view = scene.camera().view();
  const glm::mat4 proj = scene.camera().proj();

  VoxelDdaUBO ubo{};
  writeMat4(ubo.invView, glm::inverse(view));
  writeMat4(ubo.invProj, glm::inverse(proj));
  writeVec3(ubo.cameraPos, scene.camera().position());
  ubo.pad0 = 0.0f;
  writeVec3(ubo.lightDir, glm::normalize(scene.lightDir()));
  ubo.ambient = scene.ambient();
  ubo.projX = proj[0][0];
  ubo.projY = proj[1][1];
  ubo.maxSteps = scene.maxSteps();
  ubo.renderMode = static_cast<uint32_t>(std::max(0, scene.renderMode()));
  writeVec3(ubo.skyColor, scene.skyColor());
  ubo.traceStage = static_cast<uint32_t>(std::clamp(traceStage_, 0, 5));
  ubo.aoStrength = scene.aoStrength();
  ubo.aoPower = scene.aoPower();
  ubo.skyYaw = scene.skyYaw();
  ubo.skyIntensity = scene.skyIntensity();
  ubo.useSky = (scene.showSky() && scene.hasSky()) ? 1u : 0u;
  ubo.objectCount = scene.objectCount();
  ubo.solidColor = scene.solidColorOutput() ? 1u : 0u;
  ubo.dirMaskCoarse = dirMaskCoarse_ ? 1u : 0u;
  ubo.brickBitSkip = brickBitSkip_ ? 1u : 0u;
  ubo.beamSkip = (beamSkip_ && traceStage_ < kStageCoarse && scene.nestedMicroVoxels()) ? 1u : 0u;
  ubo.beamMargin = std::max(0.0f, beamMargin_);
  ubo.dirMaskBrick = dirMaskBrick_ ? 1u : 0u;
  writeVec3(ubo.solidRgb, scene.solidColor());
  ubo.useVis = (visSelect_ ? 1u : 0u) | (showBondStress_ ? 2u : 0u);

  void* mapped = frames_[frameIndex].frameUBO.info.pMappedData;
  if (!mapped) {
    throw std::runtime_error("Voxel DDA UBO is not host-mapped");
  }
  std::memcpy(mapped, &ubo, sizeof(ubo));
  if (vmaFlushAllocation(gfx_.allocator(), frames_[frameIndex].frameUBO.allocation,
                         0, sizeof(ubo)) != VK_SUCCESS) {
    throw std::runtime_error("Failed to flush voxel frame UBO");
  }
}

bool VoxelRenderer::draw(VoxelScene& scene, float displayFps) {
  displayFps_ = displayFps;

  // UI requests must not destroy resources referenced by an unsubmitted command buffer.
  if (importRequested_ || removeImportRequested_ || rebuildRequested_ || simulateRequested_ ||
      debrisPending_ >= 0) {
    gfx_.waitIdle();
    if (importRequested_) {
      MeshVoxelizeConfig cfg;
      cfg.gridN = scene.importGridN();
      cfg.padding = scene.importPadding();
      cfg.sampleColor = scene.importSampleColor();
      const std::string path = scene.importPath().empty()
                                   ? std::string(VE_ASSETS_DIR) + "/meshes/cube.obj"
                                   : scene.importPath();
      scene.importSurfaceMesh(gfx_, path, cfg);
    }
    if (removeImportRequested_) {
      scene.removeImportedMesh(gfx_);
    }
    if (rebuildRequested_) {
      scene.rebuildVoxels(gfx_);
    }
    if (simulateRequested_) {
      scene.setSimulate(gfx_, simulateValue_);
    }
    if (debrisPending_ >= 0) {
      scene.setDebrisCount(gfx_, debrisPending_);
    }
    importRequested_ = removeImportRequested_ = rebuildRequested_ = false;
    simulateRequested_ = false;
    debrisPending_ = -1;
    boundCoarsePoolBuffer_ = VK_NULL_HANDLE;
  }

  if (gfx_.swapchainWasRecreated()) {
    resize();
    gfx_.clearSwapchainRecreatedFlag();
  }

  if (outImage_.image == VK_NULL_HANDLE) {
    createOutputImage();
  }
  scene.flushOccupancyGpu(gfx_);
  scene.uploadObjectTransforms(gfx_);
  if (showBondStress_) {
    scene.uploadBondHeatmap(gfx_);
  }
  bool brickSlabsChanged = scene.brickSlabCount() != boundBrickSlabCount_;
  for (uint32_t i = 0; i < scene.brickSlabCount() && !brickSlabsChanged; ++i) {
    brickSlabsChanged = scene.brickSlabBuffer(i).buffer != boundBrickSlabs_[i];
  }
  if (brickSlabsChanged || scene.objectBuffer().buffer != boundObjectBuffer_ ||
      scene.coarsePoolBuffer().buffer != boundCoarsePoolBuffer_ ||
      scene.paletteBuffer().buffer != boundPaletteBuffer_ ||
      scene.occMipBuffer().buffer != boundOccMipBuffer_ ||
      scene.heatmapBuffer().buffer != boundHeatmapBuffer_ ||
      scene.sky().image.view != boundSkyView_ || outImage_.view == VK_NULL_HANDLE ||
      (beamImage_.view != VK_NULL_HANDLE ? beamImage_.view : dummyBeamImage_.view) !=
          boundBeamView_ ||
      visImages_[0].view != boundVisViews_[0] || visImages_[1].view != boundVisViews_[1]) {
    updateDescriptors(scene);
  }

  FrameContext frame{};
  if (!gfx_.beginFrame(frame)) {
    return false;
  }

  if (outImage_.image == VK_NULL_HANDLE || scene.objectBuffer().buffer == VK_NULL_HANDLE ||
      scene.coarsePoolBuffer().buffer == VK_NULL_HANDLE) {
    gfx_.endFrame(frame);
    return false;
  }

  collectGpuTiming(frame.frameIndex);
  updateFrameUBO(scene, frame.frameIndex);
  VkDescriptorSet frameSet = frames_[frame.frameIndex].frameSet;

  const uint32_t tsBase = frame.frameIndex * kTsPerFrame;
  if (timestampPool_) {
    vkCmdResetQueryPool(frame.cmd, timestampPool_, tsBase, kTsPerFrame);
  }

  // Images are shared across frame slots. Order the previous blit/main reads before reuse;
  // unchanged scenes no longer implicitly drain the queue through a transform upload.
  gfx_.transitionImage(frame.cmd, outImage_.image, outImage_.layout,
                       VK_IMAGE_LAYOUT_GENERAL,
                       outImage_.layout == VK_IMAGE_LAYOUT_UNDEFINED
                           ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT : VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                       outImage_.layout == VK_IMAGE_LAYOUT_UNDEFINED ? 0 : VK_ACCESS_2_TRANSFER_READ_BIT,
                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_SHADER_WRITE_BIT);
  if (beamImage_.image && (beamSkip_ || beamImage_.layout == VK_IMAGE_LAYOUT_UNDEFINED)) {
    gfx_.transitionImage(frame.cmd, beamImage_.image, beamImage_.layout, VK_IMAGE_LAYOUT_GENERAL,
                         VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                          VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT,
                          VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                          VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT);
  }

  // Start at compute, after reuse dependencies, not at TOP while the previous frame runs.
  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsFrameBegin, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
  }

  vkCmdBindDescriptorSets(frame.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout_, 0, 1,
                          &frameSet, 0, nullptr);

  if (visSelect_ && visPipeline_ && visImages_[frame.frameIndex].image &&
      visDepths_[frame.frameIndex].image && scene.objectCount() > 0) {
    recordVisPass(frame.cmd, scene, frame.extent, frameSet, frame.frameIndex);
  }

  if (beamSkip_ && traceStage_ < kStageCoarse && scene.nestedMicroVoxels() &&
      beamPipeline_ && beamImage_.image != VK_NULL_HANDLE) {
    vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, beamPipeline_);
    const uint32_t beamX = (beamImage_.extent.width + 7u) / 8u;
    const uint32_t beamY = (beamImage_.extent.height + 7u) / 8u;
    vkCmdDispatch(frame.cmd, beamX, beamY, 1);
    gfx_.transitionImage(frame.cmd, beamImage_.image, VK_IMAGE_LAYOUT_GENERAL,
                         VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                         VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                         VK_ACCESS_2_SHADER_READ_BIT);
  }

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsAfterBeam, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
  }

  VkPipeline ddaPipe = computePipeline_;
  activeKernelName_ = "FULL generic";
  if (traceStage_ >= kStageCoarse) {
    ddaPipe = coarsePipeline_;
    activeKernelName_ = "COARSE";
  } else if (!scene.nestedMicroVoxels()) {
    ddaPipe = (traceStage_ >= kStageNoShade) ? coarsePipeline_ : slimPipeline_;
    activeKernelName_ = (traceStage_ >= kStageNoShade) ? "COARSE" : "SLIM shaded";
  } else if (!forceGenericShader_ && traceStage_ == kStageFull && scene.renderMode() == 0 &&
             !scene.solidColorOutput()) {
    if (scene.objectCount() == 1 &&
        (scene.objectFlags(0) & (VoxelObject::kFlagNestedMicro | VoxelObject::kFlagNestedFine)) ==
            (VoxelObject::kFlagNestedMicro | VoxelObject::kFlagNestedFine)) {
      const bool colored = (scene.objectFlags(0) & VoxelObject::kFlagImportPalette) != 0;
      ddaPipe = colored ? specializedColorPipeline_ : specializedSinglePipeline_;
      activeKernelName_ = colored ? "FULL specialized (fine, sampled color)" : "FULL specialized (fine)";
    } else {
      ddaPipe = specializedFullPipeline_;
      activeKernelName_ = "FULL specialized (shaded, dynamic object count)";
    }
  }
  if (benchmark_ && submittedFrames_ == 0) {
    std::cout << "DDA kernel: " << activeKernelName_
              << " generic_override=" << forceGenericShader_ << std::endl;
  }
  vkCmdBindPipeline(frame.cmd, VK_PIPELINE_BIND_POINT_COMPUTE, ddaPipe);
  const uint32_t groupsX = (frame.extent.width + 7u) / 8u;
  const uint32_t groupHeight = 8u;
  const uint32_t groupsY = (frame.extent.height + groupHeight - 1u) / groupHeight;
  vkCmdDispatch(frame.cmd, groupsX, groupsY, 1);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsAfterCompute, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT);
  }

  // Blit compute output into the swapchain.
  gfx_.transitionImage(frame.cmd, outImage_.image, VK_IMAGE_LAYOUT_GENERAL,
                       VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                       VK_ACCESS_2_SHADER_WRITE_BIT, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                       VK_ACCESS_2_TRANSFER_READ_BIT);

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_UNDEFINED,
                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT, 0,
                       VK_PIPELINE_STAGE_2_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);

  VkImageBlit blit{};
  blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  blit.srcSubresource.layerCount = 1;
  blit.srcOffsets[0] = {0, 0, 0};
  blit.srcOffsets[1] = {static_cast<int32_t>(outImage_.extent.width),
                        static_cast<int32_t>(outImage_.extent.height), 1};
  blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  blit.dstSubresource.layerCount = 1;
  blit.dstOffsets[0] = {0, 0, 0};
  blit.dstOffsets[1] = {static_cast<int32_t>(frame.extent.width),
                        static_cast<int32_t>(frame.extent.height), 1};

  vkCmdBlitImage(frame.cmd, outImage_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                 frame.swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit,
                 VK_FILTER_NEAREST);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsAfterBlit, VK_PIPELINE_STAGE_2_TRANSFER_BIT);
  }

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                       VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_TRANSFER_BIT,
                       VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT);

  VkViewport vp{0, 0, static_cast<float>(frame.extent.width),
                static_cast<float>(frame.extent.height), 0, 1};
  VkRect2D scissor{{0, 0}, frame.extent};

  VkRenderingAttachmentInfo uiAttach{};
  uiAttach.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
  uiAttach.imageView = frame.swapchainView;
  uiAttach.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
  uiAttach.loadOp = VK_ATTACHMENT_LOAD_OP_LOAD;
  uiAttach.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

  VkRenderingInfo uiInfo{};
  uiInfo.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
  uiInfo.renderArea.extent = frame.extent;
  uiInfo.layerCount = 1;
  uiInfo.colorAttachmentCount = 1;
  uiInfo.pColorAttachments = &uiAttach;
  vkCmdBeginRendering(frame.cmd, &uiInfo);
  vkCmdSetViewport(frame.cmd, 0, 1, &vp);
  vkCmdSetScissor(frame.cmd, 0, 1, &scissor);
  recordImGui(frame.cmd, scene, displayFps);
  vkCmdEndRendering(frame.cmd);

  gfx_.transitionImage(frame.cmd, frame.swapchainImage, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                       VK_IMAGE_LAYOUT_PRESENT_SRC_KHR,
                       VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                       VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT,
                       0);

  if (timestampPool_) {
    writeTimestamp(frame.cmd, tsBase + kTsFrameEnd, VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT);
  }

  gfx_.endFrame(frame);
  outImage_.layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
  if (beamImage_.image) {
    beamImage_.layout = VK_IMAGE_LAYOUT_GENERAL;
  }
  if (timestampPool_) {
    timestampSubmissions_[frame.frameIndex] = submittedFrames_;
    timestampPending_[frame.frameIndex] = true;
  }
  ++submittedFrames_;
  return true;
}

void VoxelRenderer::initImGui() {
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  if (benchmark_) {
    ImGui::GetIO().IniFilename = nullptr;
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NoMouse;
  }
  ImGui::StyleColorsDark();

  VkDescriptorPoolSize poolSizes[] = {
      {VK_DESCRIPTOR_TYPE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000},
      {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000},
      {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000},
      {VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000},
  };
  VkDescriptorPoolCreateInfo poolInfo{};
  poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
  poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
  poolInfo.maxSets = 1000;
  poolInfo.poolSizeCount = static_cast<uint32_t>(std::size(poolSizes));
  poolInfo.pPoolSizes = poolSizes;
  vkCreateDescriptorPool(gfx_.device(), &poolInfo, nullptr, &imguiPool_);

  ImGui_ImplGlfw_InitForVulkan(gfx_.window().handle(), true);

  static VkFormat sSwapchainFormat = VK_FORMAT_UNDEFINED;
  sSwapchainFormat = gfx_.swapchainFormat();

  ImGui_ImplVulkan_InitInfo initInfo{};
  initInfo.Instance = gfx_.instance();
  initInfo.PhysicalDevice = gfx_.physicalDevice();
  initInfo.Device = gfx_.device();
  initInfo.QueueFamily = gfx_.graphicsQueueFamily();
  initInfo.Queue = gfx_.graphicsQueue();
  initInfo.DescriptorPool = imguiPool_;
  initInfo.MinImageCount = GfxDevice::kFramesInFlight;
  initInfo.ImageCount = GfxDevice::kFramesInFlight;
  initInfo.UseDynamicRendering = true;
#ifdef IMGUI_IMPL_VULKAN_HAS_DYNAMIC_RENDERING
  initInfo.PipelineRenderingCreateInfo = {};
  initInfo.PipelineRenderingCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RENDERING_CREATE_INFO;
  initInfo.PipelineRenderingCreateInfo.colorAttachmentCount = 1;
  initInfo.PipelineRenderingCreateInfo.pColorAttachmentFormats = &sSwapchainFormat;
#endif

  ImGui_ImplVulkan_Init(&initInfo);
  imguiReady_ = true;
}

void VoxelRenderer::shutdownImGui() {
  if (!imguiReady_) {
    return;
  }
  ImGui_ImplVulkan_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  if (imguiPool_) {
    vkDestroyDescriptorPool(gfx_.device(), imguiPool_, nullptr);
    imguiPool_ = VK_NULL_HANDLE;
  }
  imguiReady_ = false;
}

void VoxelRenderer::recordImGui(VkCommandBuffer cmd, VoxelScene& scene, float displayFps) {
  ImGui_ImplVulkan_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  {
    const glm::mat4 viewProj = scene.camera().proj() * scene.camera().view();
    if (scene.simulate() && scene.cpuObjectCount() >= 2) {
      drawCornerNormals(scene, viewProj, ImGui::GetIO().DisplaySize);
    }
  }

  ImGuiWindowFlags flags = 0;
  if (benchmark_) {
    ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        ImVec2(520.0f, std::clamp(static_cast<float>(gfx_.swapchainExtent().height) - 20.0f,
                                 32.0f, 900.0f)),
        ImGuiCond_Always);
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
    flags = ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings;
  }
  ImGui::Begin("Voxel DDA", nullptr, flags);
  ImGui::TextWrapped("GPU: %s", gfx_.deviceName().c_str());
  ImGui::Text("Present: %s", gfx_.presentModeName());
  ImGui::Text("Display FPS: %.1f  (%.2f ms)", displayFps,
              displayFps > 1e-3f ? 1000.0f / displayFps : 0.0f);
  ImGui::Text("GPU total:   %.2f ms  (%.0f FPS)", gpuFrameMs_,
              gpuFrameMs_ > 1e-3f ? 1000.0f / gpuFrameMs_ : 0.0f);
  ImGui::Text("  compute:   %.2f ms", gpuComputeMs_);
  if (beamSkip_) {
    ImGui::Text("    beam 8x8: %.2f ms", gpuBeamMs_);
    ImGui::Text("    main DDA: %.2f ms", gpuMainMs_);
  }
  ImGui::Text("  blit:      %.2f ms", gpuBlitMs_);
  ImGui::Text("  ui/other:  %.2f ms", gpuUiMs_);
  const char* stages[] = {
      "Full (DDA + nest + AO + light)",
      "No shade (DDA + nest, skip AO/light)",
      "No 2x2x2 (coarse + 8^3)",
      "Coarse DDA only",
      "Ray gen only",
      "Skip DDA (sky store)",
  };
  ImGui::Combo("Cost Ladder", &traceStage_, stages, IM_ARRAYSIZE(stages));
  const char* modes[] = {"Shaded", "Albedo", "Normal", "Steps", "Coord", "AO"};
  ImGui::Combo("Render Mode", &scene.renderMode(), modes, IM_ARRAYSIZE(modes));
  ImGui::TextDisabled("Steps is here, not in Cost Ladder: dark = few DDA steps, yellow = hit Max Steps.");
  if (traceStage_ >= kStageInterval) {
    ImGui::TextDisabled("Ray gen / Skip DDA never walk the grid, so Steps stays sky.");
  }
  {
    ImGui::Text("DDA kernel: %s", activeKernelName_);
    if (forceGenericShader_) {
      ImGui::TextDisabled("VE_VOXEL_GENERIC_SHADER=1: full specialization disabled.");
    }
    ImGui::Checkbox("Nested 8^3 (micro bricks)", &scene.nestedMicroVoxels());
    ImGui::TextDisabled("Off -> slim kernel. Cost Ladder \"Coarse DDA only\" -> coarse kernel.");
  }
  ImGui::TextDisabled("Compare GPU compute ms at the same camera. Display FPS can lie.");
  ImGui::TextDisabled("Walk the ladder: the step that jumps compute ms is the bottleneck.");
  if (gfx_.vsyncEnabled()) {
    ImGui::TextDisabled("VSync present mode; Display FPS is refresh-capped.");
  }
  ImGui::Text("Objects: %u  (CPU %d)  vis coarses: %u", scene.objectCount(),
              scene.cpuObjectCount(), scene.visInstanceCount());
  ImGui::Checkbox("OBB vis-buffer", &visSelect_);
  ImGui::TextDisabled("On: rasterize occupied coarses, DDA that id (miss falls back). Off: loop all.");
  {
    int debris = debrisPending_ >= 0 ? debrisPending_ : scene.debrisCount();
    if (ImGui::SliderInt("Debris objects", &debris, 0, 30)) {
      debrisPending_ = debris;
    }
    ImGui::TextDisabled("Extra 8^3 boxes. Simulate on: they fall. Off: they spin in place.");
  }
  ImGui::Text("Occupied coarse: %u / %u", scene.occupiedCount(), scene.voxelCount());
  ImGui::Text("Brick pages: %u  slabs: %u  pool: %.1f KB", scene.allocatedBrickPages(),
              scene.brickSlabCount(), static_cast<float>(scene.brickPoolBytes()) / 1024.0f);
  ImGui::Checkbox("Brick 4^3 / 2^3 bit skip", &brickBitSkip_);
  ImGui::TextDisabled("Off = 1-cell DDA inside 8^3 (GDVoxelPlayground). On = jump empty octants.");
  ImGui::BeginDisabled(!brickBitSkip_);
  ImGui::Checkbox("4^3 direction AND (brick)", &dirMaskBrick_);
  ImGui::EndDisabled();
  ImGui::Checkbox("4^3 direction AND (coarse)", &dirMaskCoarse_);
  ImGui::TextDisabled("Off disables coarse tile skipping. Beam certification is independent.");
  ImGui::TextDisabled("AND skips 4^3 tiles whose solids all lie behind this ray.");
  ImGui::Checkbox("Beam empty-prefix prepass (8x8)", &beamSkip_);
  ImGui::BeginDisabled(!beamSkip_);
  ImGui::SliderFloat("Beam margin (m)", &beamMargin_, 0.0f, 16.0f, "%.3f");
  ImGui::EndDisabled();
  ImGui::TextDisabled("Entire tile frustum: skip only a certified empty prefix.");
  ImGui::TextDisabled("Margin adds a roundoff guard; off = trace from the camera.");
  ImGui::Checkbox("Collapse full bricks to INVALID", &scene.collapseFullBricks());
  ImGui::TextDisabled("Off = keep a page even when 8^3 x 2^3 is solid. Toggle applies on next edit.");
  ImGui::Text("Occupied 8^3 micros: %u   2^3 fines: %u", scene.occupiedMicroCount(),
              scene.occupiedFineCount());
  ImGui::Separator();
  ImGui::Checkbox("Bond stress heatmap", &showBondStress_);
  ImGui::TextDisabled("Off by default. Colors the voxels: green = low Φ, red = Φ≥1.");
  {
    const physics::DebugSolve dsHeat = scene.physicsDebug();
    const int nBonds = static_cast<int>(scene.structureDebugBonds().size());
    ImGui::Text("Heatmap bonds=%d  alive=%d  maxPhi=%.2f", nBonds, dsHeat.bondsAlive, dsHeat.maxPhi);
  }
  ImGui::Separator();
  ImGui::TextWrapped("LMB: remove hit object  |  F: place on hit face  |  RMB drag: look");
  ImGui::SliderInt("Brush Material", &scene.brushMaterial(), 1, 2);
  ImGui::Checkbox("Nested 8^3 (micro bricks)", &scene.nestedMicroVoxels());
  ImGui::BeginDisabled(!scene.nestedMicroVoxels());
  ImGui::Checkbox("Nested 2^3 (fine voxels)", &scene.nestedFineVoxels());
  ImGui::EndDisabled();
  if (scene.nestedMicroVoxels() && scene.nestedFineVoxels()) {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f fine voxels");
    ImGui::TextDisabled("coarse -> 8^3 brick -> 2x2x2. Spinner is a 2x2x2 checker.");
    ImGui::TextDisabled("r=0 edits one fine cell (1/16 of a coarse voxel).");
  } else if (scene.nestedMicroVoxels()) {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f micro voxels");
    ImGui::TextDisabled("Render/edit 8^3 bricks; 2x2x2 fines are ignored (virtual full micro).");
  } else {
    ImGui::SliderFloat("Brush Radius", &scene.brushRadius(), 0.0f, 8.0f, "%.1f coarse voxels");
    ImGui::TextDisabled("Editing whole coarse cells (each owns an 8^3 brick)");
  }
  {
    bool sim = simulateRequested_ ? simulateValue_ : scene.simulate();
    if (ImGui::Checkbox("Simulate", &sim)) {
      simulateRequested_ = true;
      simulateValue_ = sim;
    }
    ImGui::TextDisabled("On = solid test box falls on 3.2 m ground. Off = spinner.");
    if (scene.simulate()) {
      ImGui::Text("Test box corners: %u   edges: %u", scene.physicsCornerCount(1),
                  scene.physicsEdgeCount(1));
      ImGui::TextDisabled("Corner rays: green = n.y up, red = n.y down, grey = no hit.");
      const physics::DebugSolve ds = scene.physicsDebug();
      ImGui::Text("Solve contacts=%d  maxD=%.3f  minNy=%.2f", ds.contacts, ds.maxD, ds.minNy);
      ImGui::Text("Box v=(%.2f,%.2f,%.2f) |w|=%.2f", ds.v.x, ds.v.y, ds.v.z, glm::length(ds.w));
      ImGui::Text("Bonds alive=%d  broken=%d  maxPhi=%.2f", ds.bondsAlive, ds.bondsBrokenThisStep,
                  ds.maxPhi);
    }
  }
  ImGui::Checkbox("Show Rotating Object", &scene.spinnerEnabled());
  ImGui::SliderFloat("Spin Speed", &scene.spinSpeed(), -3.0f, 3.0f, "%.2f rad/s");
  ImGui::TextDisabled("Uncheck to hide the spinning voxel object.");
  if (const std::optional<VoxelHit> hit = scene.lastHit()) {
    if (hit->hasFine || hit->hasMicro) {
      ImGui::Text("Hit obj=%d c=(%d,%d,%d) m=(%d,%d,%d) f=(%d,%d,%d) n=(%d,%d,%d) mat=%u",
                  hit->objectIndex, hit->cell.x, hit->cell.y, hit->cell.z, hit->micro.x,
                  hit->micro.y, hit->micro.z, hit->fine.x, hit->fine.y, hit->fine.z, hit->normal.x,
                  hit->normal.y, hit->normal.z, hit->material);
    } else {
      ImGui::Text("Hit obj=%d: (%d, %d, %d)  n=(%d,%d,%d)  mat=%u", hit->objectIndex, hit->cell.x,
                  hit->cell.y, hit->cell.z, hit->normal.x, hit->normal.y, hit->normal.z,
                  hit->material);
    }
  } else {
    ImGui::TextUnformatted("Hit: none");
  }
  ImGui::Separator();
  ImGui::TextUnformatted("Import Mesh (stamp into world grid, one DDA)");
  char pathBuf[512]{};
  const std::string defaultObj = std::string(VE_ASSETS_DIR) + "/meshes/cube.obj";
  const std::string& srcPath = scene.importPath().empty() ? defaultObj : scene.importPath();
  std::strncpy(pathBuf, srcPath.c_str(), sizeof(pathBuf) - 1);
  if (ImGui::InputText("OBJ Path", pathBuf, sizeof(pathBuf))) {
    scene.importPath() = pathBuf;
  }
  ImGui::SliderInt("Import Grid N", &scene.importGridN(), 8, 64);
  ImGui::SliderInt("Import Padding", &scene.importPadding(), 0, 4);
  ImGui::Checkbox("Sample Mesh Color", &scene.importSampleColor());
  ImGui::TextDisabled(
      "Sample Color: each fine stores RGBA; alpha means this voxel has a sampled color.");
  if (ImGui::Button("Import Surface OBJ")) {
    importRequested_ = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("Remove Imported")) {
    removeImportRequested_ = true;
  }
  ImGui::TextWrapped("%s", scene.importStatus().c_str());
  ImGui::Separator();

  ImGui::Checkbox("Solid Color", &scene.solidColorOutput());
  if (scene.solidColorOutput()) {
    ImGui::ColorEdit3("Voxel Color", &scene.solidColor().x);
    ImGui::TextDisabled("Hits write this color; lighting and AO are skipped.");
  }
  ImGui::Checkbox("Skybox", &scene.showSky());
  if (scene.showSky()) {
    ImGui::DragFloat("Sky Intensity", &scene.skyIntensity(), 0.01f, 0.0f, 8.0f);
    ImGui::DragFloat("Sky Yaw", &scene.skyYaw(), 0.01f, -3.14159f, 3.14159f);
    if (!scene.hasSky()) {
      ImGui::TextDisabled("HDR sky missing; using flat sky color.");
    }
  } else {
    ImGui::TextDisabled("Skybox off: miss pixels use Sky Color, no HDR sample.");
  }
  ImGui::ColorEdit3("Sky Color", &scene.skyColor().x);

  bool rebuild = false;
  rebuild |= ImGui::SliderInt("Grid Size", &scene.gridSize(), 8, 64);
  rebuild |= ImGui::DragFloat("Coarse Voxel (m)", &scene.voxelSize(), 0.01f, 0.05f, 2.0f);
  ImGui::TextDisabled("Fine (gameplay) %.3f m   Micro %.3f m   World %.1f m",
                      scene.fineVoxelSize(), scene.microVoxelSize(),
                      static_cast<float>(scene.gridSize()) * scene.voxelSize());
  ImGui::DragFloat3("Light Dir", &scene.lightDir().x, 0.01f);
  ImGui::SliderFloat("Ambient", &scene.ambient(), 0.0f, 1.0f);
  ImGui::SliderFloat("AO Strength", &scene.aoStrength(), 0.0f, 1.0f);
  ImGui::SliderFloat("AO Power", &scene.aoPower(), 0.05f, 2.0f);
  ImGui::TextDisabled("Neighbor voxel AO (Minecraft-style). Strength 0 disables.");
  int maxSteps = static_cast<int>(scene.maxSteps());
  if (ImGui::SliderInt("Max Steps", &maxSteps, 16, 512)) {
    scene.maxSteps() = static_cast<uint32_t>(maxSteps);
  }
  if (rebuild) {
    rebuildRequested_ = true;
  }
  ImGui::End();

  // Simple screen-center crosshair for aim.
  const ImGuiViewport* vp = ImGui::GetMainViewport();
  const ImVec2 center(vp->GetCenter().x, vp->GetCenter().y);
  ImDrawList* dl = ImGui::GetForegroundDrawList();
  const float arm = 8.0f;
  const ImU32 col = IM_COL32(255, 255, 255, 220);
  dl->AddLine(ImVec2(center.x - arm, center.y), ImVec2(center.x + arm, center.y), col, 1.5f);
  dl->AddLine(ImVec2(center.x, center.y - arm), ImVec2(center.x, center.y + arm), col, 1.5f);

  ImGui::Render();
  ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), cmd);
}
