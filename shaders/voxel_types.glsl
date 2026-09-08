// Shared voxel types/constants. Must match src/scene/VoxelScene.h Phase 0 layout.
#ifndef VE_VOXEL_TYPES_GLSL
#define VE_VOXEL_TYPES_GLSL

struct CoarseCell {
    uint material;
    uint brickPage;  // 0xFFFFFFFFu = none
};

// Must match GpuVoxelObject (208 bytes, std430).
struct GpuVoxelObject {
    mat4 worldToObject;
    mat4 objectToWorld;
    float voxelSize;
    float _pad0;
    float _pad1;
    float _pad2;
    uvec3 gridSize;
    uint flags;  // bit0 nestedMicro, bit1 enabled, bit2 import color, bit3 nestedFine
    uint voxelOffset;  // coarse cell index into shared CoarsePool
    uint occMipOffset;  // 4^3 coarse tiles only: two uints (64 Morton bits) each
    uint occMipWords;   // tile count * 2
    uint _pad4;
    vec3 occMin;  // coarse inclusive
    float _padOccMin;
    vec3 occMax;  // coarse exclusive
    float _padOccMax;
};

const uint MICRO_WORDS = 16u;
const uint FINE_WORDS = 128u;
const uint FINE_COLOR_OFFSET = MICRO_WORDS + FINE_WORDS;
const uint FINE_COLOR_WORDS = 4096u;
const uint BRICK_PAGE_WORDS = FINE_COLOR_OFFSET + FINE_COLOR_WORDS;
const uint PAGES_PER_SLAB = 2048u;
const uint INVALID_BRICK_PAGE = 0xFFFFFFFFu;
const uint FLAG_NESTED = 1u;
const uint FLAG_ENABLED = 2u;
const uint FLAG_IMPORT_PALETTE = 4u;
const uint FLAG_NESTED_FINE = 8u;
const float FLT_MAX = 3.4028235e+38;
const float kPi = 3.14159265359;

#endif  // VE_VOXEL_TYPES_GLSL
