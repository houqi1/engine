#pragma once

#include "gfx/GfxDevice.h"
#include "gfx/GpuTypes.h"
#include "scene/VoxelScene.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

class VoxelRenderer {
public:
  struct BenchmarkSettings {
    bool beam = true;
    bool brickSkip = true;
    bool dirBrick = true;
    bool dirCoarse = true;
    uint32_t stage = 0;
  };

  struct GpuTiming {
    uint64_t submission = 0;  // Zero-based rendered submission, including warmup.
    double totalMs = 0.0;
    double beamMs = 0.0;
    double mainMs = 0.0;
    double computeMs = 0.0;
    double blitMs = 0.0;
    double uiMs = 0.0;
  };

  explicit VoxelRenderer(GfxDevice& gfx);
  ~VoxelRenderer();

  // Configure before init; the normal UI is rendered but cannot change the scene.
  void configureBenchmark(const BenchmarkSettings& settings, uint32_t warmup, uint32_t frames);
  void init(VoxelScene& scene);
  void resize();
  bool draw(VoxelScene& scene, float displayFps);
  std::vector<GpuTiming> finishBenchmark();
  float beamMargin() const { return beamMargin_; }
  const char* activeKernelName() const { return activeKernelName_; }
  // P6 RGB bytes from the pre-UI RGBA8 output, without gamma conversion or alpha.
  void capturePpm(const std::string& path);

private:
  // Must match shaders/voxel_dda.comp std140 layout.
  struct VoxelDdaUBO {
    float invView[16];
    float invProj[16];
    float cameraPos[3];
    float pad0;
    float lightDir[3];
    float ambient;
    float projX;
    float projY;
    uint32_t maxSteps;
    uint32_t renderMode;
    float skyColor[3];
    uint32_t traceStage;
    float aoStrength;
    float aoPower;
    float skyYaw;
    float skyIntensity;
    uint32_t useSky;
    uint32_t objectCount;
    uint32_t solidColor;
    uint32_t dirMaskCoarse;
    uint32_t brickBitSkip;
    uint32_t beamSkip;
    float beamMargin;
    uint32_t dirMaskBrick;
    float solidRgb[3];
    float padSolidEnd;
  };
  static_assert(sizeof(VoxelDdaUBO) == 256, "VoxelDdaUBO std140 size mismatch");

  // Graphics path UBO: compute fields + viewProj + screen size (std140).
  // solidRgb (vec3) packs with screenWidth (uint) in one vec4.
  struct VoxelGfxUBO {
    float invView[16];
    float invProj[16];
    float viewProj[16];
    float cameraPos[3];
    float pad0;
    float lightDir[3];
    float ambient;
    float projX;
    float projY;
    uint32_t maxSteps;
    uint32_t renderMode;
    float skyColor[3];
    uint32_t traceStage;
    float aoStrength;
    float aoPower;
    float skyYaw;
    float skyIntensity;
    uint32_t useSky;
    uint32_t objectCount;
    uint32_t solidColor;
    uint32_t dirMaskCoarse;
    uint32_t brickBitSkip;
    uint32_t beamSkip;
    float beamMargin;
    uint32_t dirMaskBrick;
    float solidRgb[3];
    uint32_t screenWidth;  // packs with solidRgb in std140
    uint32_t screenHeight;
    uint32_t depthSnapshotEnabled;  // 1 = sample DepthSnapshot for DDA cull/clip
    float depthOcclusionMargin;     // world-space meters (prefer miss cull over holes)
    uint32_t padGfx0;               // std140 block round-up
  };
  static_assert(sizeof(VoxelGfxUBO) == 336, "VoxelGfxUBO std140 size mismatch");

  struct FrameResources {
    AllocatedBuffer frameUBO{};
    AllocatedBuffer gfxUBO{};
    AllocatedBuffer visibleInstances{};
    VkDescriptorSet frameSet = VK_NULL_HANDLE;
    VkDescriptorSet gfxSet = VK_NULL_HANDLE;
  };

  enum TimestampSlot : uint32_t {
    kTsFrameBegin = 0,
    kTsAfterBeam = 1,
    kTsAfterCompute = 2,
    kTsAfterBlit = 3,
    kTsFrameEnd = 4,
    kTsPerFrame = 5,
  };

  enum TraceStage : uint32_t {
    kStageFull = 0,
    kStageNoShade = 1,
    kStageNoFine = 2,
    kStageCoarse = 3,
    kStageInterval = 4,
    kStageSkipDda = 5,
  };

  struct DdaSpec {
    uint32_t beamPass = 0;
    uint32_t enableNested = 1;
    uint32_t enableShade = 1;
    uint32_t shadedOnly = 0;
    uint32_t singleObject = 0;
    uint32_t groupHeight = 8;
    uint32_t fineOnly = 0;
    uint32_t colorMode = 2;
  };

  struct VisibleInstance {
    uint32_t objectGpuIndex = 0;
    uint32_t flags = 0;  // bit0 = fullscreen near-clip fallback
  };

  void createDescriptors();
  void createPipelines();
  VkPipeline createComputePipeline(VkShaderModule shader, DdaSpec spec,
                                  bool useSpec = true) const;
  void createOutputImage();
  void destroyOutputImage();
  void createCubeMesh();
  void destroyCubeMesh();
  void updateDescriptors(VoxelScene& scene);
  void updateFrameUBO(VoxelScene& scene, uint32_t frameIndex);
  void updateGfxUBO(VoxelScene& scene, uint32_t frameIndex, uint32_t width, uint32_t height);
  void buildVisibleInstances(VoxelScene& scene, uint32_t frameIndex);
  void createTimestampPool();
  void destroyTimestampPool();
  void writeTimestamp(VkCommandBuffer cmd, uint32_t queryIndex, VkPipelineStageFlags2 stage) const;
  void collectGpuTiming(uint32_t frameIndex);
  void initImGui();
  void shutdownImGui();
  void recordImGui(VkCommandBuffer cmd, VoxelScene& scene, float displayFps);

  GfxDevice& gfx_;

  VkDescriptorSetLayout frameLayout_ = VK_NULL_HANDLE;
  VkDescriptorSetLayout gfxLayout_ = VK_NULL_HANDLE;
  VkDescriptorPool descriptorPool_ = VK_NULL_HANDLE;
  VkDescriptorPool imguiPool_ = VK_NULL_HANDLE;

  VkPipelineLayout pipelineLayout_ = VK_NULL_HANDLE;
  VkPipelineLayout gfxPipelineLayout_ = VK_NULL_HANDLE;
  VkPipeline computePipeline_ = VK_NULL_HANDLE;
  VkPipeline specializedFullPipeline_ = VK_NULL_HANDLE;
  VkPipeline specializedSinglePipeline_ = VK_NULL_HANDLE;
  VkPipeline specializedColorPipeline_ = VK_NULL_HANDLE;
  VkPipeline slimPipeline_ = VK_NULL_HANDLE;
  VkPipeline coarsePipeline_ = VK_NULL_HANDLE;
  VkPipeline beamPipeline_ = VK_NULL_HANDLE;
  VkPipeline visibilityPipeline_ = VK_NULL_HANDLE;
  VkPipeline shadePipeline_ = VK_NULL_HANDLE;
  bool forceGenericShader_ = false;
  const char* activeKernelName_ = "Not rendered";

  AllocatedImage outImage_{};
  AllocatedImage beamImage_{};
  AllocatedImage dummyBeamImage_{};
  AllocatedImage hitImage_{};
  AllocatedImage visibilityDepth_{};
  AllocatedImage depthSnapshot_{};
  VkSampler hitSampler_ = VK_NULL_HANDLE;
  AllocatedBuffer cubeVbo_{};
  AllocatedBuffer cubeIbo_{};
  AllocatedBuffer dummyVisibleInstances_{};
  uint32_t boxInstanceCount_ = 0;
  uint32_t specialInstanceCount_ = 0;
  static constexpr VkFormat kOutFormat = VK_FORMAT_R8G8B8A8_UNORM;
  static constexpr VkFormat kBeamFormat = VK_FORMAT_R32_SFLOAT;
  // SFLOAT avoids MoltenVK false-positive "blending enabled" on integer color attachments.
  static constexpr VkFormat kHitFormat = VK_FORMAT_R32G32B32A32_SFLOAT;
  static constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

  std::array<FrameResources, GfxDevice::kFramesInFlight> frames_{};
  VkBuffer boundCoarsePoolBuffer_ = VK_NULL_HANDLE;
  uint32_t boundGpuResourceSerial_ = 0;
  std::array<VkBuffer, VoxelScene::kMaxBrickSlabs> boundBrickSlabs_{};
  uint32_t boundBrickSlabCount_ = 0;
  std::array<VkBuffer, GfxDevice::kFramesInFlight> boundObjectBuffers_{};
  VkBuffer boundPaletteBuffer_ = VK_NULL_HANDLE;
  VkBuffer boundOccMipBuffer_ = VK_NULL_HANDLE;
  VkImageView boundSkyView_ = VK_NULL_HANDLE;
  VkImageView boundBeamView_ = VK_NULL_HANDLE;
  VkImageView boundHitView_ = VK_NULL_HANDLE;
  VkImageView boundDepthSnapshotView_ = VK_NULL_HANDLE;
  bool imguiReady_ = false;
  bool importRequested_ = false;
  bool removeImportRequested_ = false;
  bool rebuildRequested_ = false;
  bool scatterSpawnRequested_ = false;
  bool scatterClearRequested_ = false;
  bool simulateRequested_ = false;
  uint32_t scatterSpawnCount_ = 0;
  bool pendingSimulate_ = false;
  float displayFps_ = 0.0f;
  int traceStage_ = 0;
  bool brickBitSkip_ = true;
  bool dirMaskBrick_ = true;
  bool dirMaskCoarse_ = true;
  bool beamSkip_ = true;
  bool boxFragmentPath_ = true;  // Phase 1 main path; OFF = compute reference
  int snapshotBatchCount_ = 0;   // 0/2/4/8; 0 = single-pass baseline
  float depthOcclusionMargin_ = 0.005f;
  float beamMargin_ = 0.001f;  // Additional world-space roundoff guard for certified prefixes.

  VkQueryPool timestampPool_ = VK_NULL_HANDLE;
  float timestampPeriodNs_ = 1.0f;
  uint64_t timestampMask_ = UINT64_MAX;
  std::array<bool, GfxDevice::kFramesInFlight> timestampPending_{};
  std::array<uint64_t, GfxDevice::kFramesInFlight> timestampSubmissions_{};
  uint64_t submittedFrames_ = 0;
  bool benchmark_ = false;
  uint32_t benchmarkWarmup_ = 0;
  uint32_t benchmarkFrames_ = 0;
  std::vector<GpuTiming> benchmarkTimings_;
  float gpuFrameMs_ = 0.0f;
  float gpuComputeMs_ = 0.0f;
  float gpuBeamMs_ = 0.0f;
  float gpuMainMs_ = 0.0f;
  float gpuBlitMs_ = 0.0f;
  float gpuUiMs_ = 0.0f;
};
