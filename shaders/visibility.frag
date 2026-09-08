#version 450
#extension GL_GOOGLE_include_directive : require

#include "voxel_types.glsl"

layout(set = 0, binding = 0, std140) uniform VoxelGfxUBO {
    mat4 invView;
    mat4 invProj;
    mat4 viewProj;
    vec3 cameraPos;
    float _pad0;
    vec3 lightDir;
    float ambient;
    float projX;
    float projY;
    uint maxSteps;
    uint renderMode;
    vec3 skyColor;
    uint traceStage;
    float aoStrength;
    float aoPower;
    float skyYaw;
    float skyIntensity;
    uint useSky;
    uint objectCount;
    uint solidColor;
    uint dirMaskCoarse;
    uint brickBitSkip;
    uint beamSkip;
    float beamMargin;
    uint dirMaskBrick;
    vec3 solidRgb;
    uint screenWidth;
    uint screenHeight;
    uint depthSnapshotEnabled;
    float depthOcclusionMargin;
    uint _padGfx0;
} ubo;

layout(set = 0, binding = 1, std430) readonly buffer CoarsePool {
    uvec2 cells[];
} coarsePool;

layout(set = 0, binding = 3, std430) readonly buffer BrickPool {
    uint bricks[];
} brickSlabs[8];

layout(set = 0, binding = 5, std430) readonly buffer ObjectBuffer {
    GpuVoxelObject objects[];
};

layout(set = 0, binding = 7, std430) readonly buffer OccupancyMip {
    uint occTiles[];
};

layout(set = 0, binding = 9) uniform sampler2D depthSnapshot;

#include "voxel_trace.glsl"

layout(location = 0) flat in uint vObjectIndex;
// Packed as float bits (RGBA32F) for Metal/MoltenVK color-attachment compatibility.
layout(location = 0) out vec4 outHit;

void main() {
    GpuVoxelObject o = objects[vObjectIndex];

    vec2 uv = (gl_FragCoord.xy + vec2(0.5)) / vec2(float(ubo.screenWidth), float(ubo.screenHeight));
    vec2 ndc = uv * 2.0 - 1.0;
    vec3 dirView = normalize(vec3(ndc.x / ubo.projX, ndc.y / ubo.projY, -1.0));
    vec3 Dw = normalize((ubo.invView * vec4(dirView, 0.0)).xyz);
    vec3 Ow = ubo.cameraPos;

    float tEnter = 0.0;
    float tExit = 1.0e4;
    float tMax = 1.0e4;
    if (!rayOccBoundsWorld(o, Ow, Dw, tEnter, tExit)) {
        discard;
    }

    if (ubo.depthSnapshotEnabled != 0u) {
        float depth = texelFetch(depthSnapshot, ivec2(gl_FragCoord.xy), 0).r;
        // Reverse-Z clear/far is 0 → no occluder.
        float tOpaque = 1.0e30;
        if (depth > 0.0) {
            vec4 clip = vec4(ndc, depth, 1.0);
            vec4 worldH = (ubo.invView * ubo.invProj) * clip;
            vec3 worldPos = worldH.xyz / max(worldH.w, 1e-6);
            tOpaque = dot(worldPos - Ow, Dw);
        }
        float margin = ubo.depthOcclusionMargin * max(tEnter, 1.0);
        if (tEnter > tOpaque + margin) {
            discard;
        }
        tMax = min(tExit, tOpaque + margin);
    } else {
        tMax = tExit;
    }

    TraceHit h = traceObjectBounded(o, Ow, Dw, 0.0, tMax);
    if (!h.hit) {
        discard;
    }

    vec3 hitWorld = Ow + Dw * h.tWorld;
    vec4 clip = ubo.viewProj * vec4(hitWorld, 1.0);
    gl_FragDepth = clip.z / clip.w;

    uint faceAxis = h.mask.x ? 0u : (h.mask.y ? 1u : 2u);
    uint faceSign = 0u;
    if (faceAxis == 0u) {
        faceSign = h.sgn.x < 0.0 ? 1u : 0u;
    } else if (faceAxis == 1u) {
        faceSign = h.sgn.y < 0.0 ? 1u : 0u;
    } else {
        faceSign = h.sgn.z < 0.0 ? 1u : 0u;
    }

    uint y = uint(h.mapPos.x) | (uint(h.mapPos.y) << 10) | (uint(h.mapPos.z) << 20);
    uint z = uint(h.micro.x) | (uint(h.micro.y) << 8) | (uint(h.micro.z) << 16) |
             (uint(h.fine.x) << 24) | (uint(h.fine.y) << 26) | (uint(h.fine.z) << 28);
    uint w = faceAxis | (faceSign << 2) | (h.usedMicro ? 8u : 0u) | (h.usedFine ? 16u : 0u) |
             (h.material << 8);

    outHit = vec4(uintBitsToFloat(vObjectIndex + 1u), uintBitsToFloat(y), uintBitsToFloat(z),
                  uintBitsToFloat(w));
}
