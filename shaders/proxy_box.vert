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

layout(set = 0, binding = 5, std430) readonly buffer ObjectBuffer {
    GpuVoxelObject objects[];
};

layout(set = 0, binding = 8, std430) readonly buffer VisibleInstanceBuffer {
    uvec2 instances[];  // x = objectGpuIndex, y = flags (bit0 = fullscreen clip fallback)
};

layout(location = 0) in vec3 inPos;

layout(location = 0) flat out uint vObjectIndex;

void main() {
    uvec2 inst = instances[gl_InstanceIndex];
    uint objectIndex = inst.x;
    vObjectIndex = objectIndex;

    if ((inst.y & 1u) != 0u) {
        // Near-clip / in-box fallback: fullscreen triangle (non-indexed draw of 3 verts).
        vec2 uv = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
        gl_Position = vec4(uv * 2.0 - 1.0, 1.0, 1.0);
        return;
    }

    GpuVoxelObject o = objects[objectIndex];
    vec3 localVoxel = mix(o.occMin, o.occMax, inPos);
    vec3 localMeters = localVoxel * o.voxelSize;
    vec4 world = o.objectToWorld * vec4(localMeters, 1.0);
    gl_Position = ubo.viewProj * world;
}
