#version 450

// Unit-cube triangles (36 verts). Instance = one occupied coarse.
layout(push_constant) uniform VisPush {
    mat4 view;
    mat4 proj;
} pc;

struct GpuVoxelObject {
    mat4 worldToObject;
    mat4 objectToWorld;
    float voxelSize;
    float _pad0;
    float _pad1;
    float _pad2;
    uvec3 gridSize;
    uint flags;
    uint voxelOffset;
    uint occMipOffset;
    uint occMipWords;
    uint cpuIndex;
    vec3 occMin;
    float _padOccMin;
    vec3 occMax;
    float _padOccMax;
};

layout(set = 0, binding = 5, std430) readonly buffer ObjectBuffer {
    GpuVoxelObject objects[];
};

layout(location = 0) in uint inGpuIndex;
layout(location = 1) in uint inPackedCoarse;

layout(location = 0) flat out uint vGpuIndex;

vec3 unitCubeVert(uint i) {
    const vec3 c[8] = vec3[](
        vec3(0.0, 0.0, 0.0), vec3(1.0, 0.0, 0.0), vec3(1.0, 1.0, 0.0), vec3(0.0, 1.0, 0.0),
        vec3(0.0, 0.0, 1.0), vec3(1.0, 0.0, 1.0), vec3(1.0, 1.0, 1.0), vec3(0.0, 1.0, 1.0));
    const uint idx[36] = uint[](
        0u, 1u, 2u, 0u, 2u, 3u,
        4u, 6u, 5u, 4u, 7u, 6u,
        0u, 4u, 5u, 0u, 5u, 1u,
        3u, 2u, 6u, 3u, 6u, 7u,
        0u, 3u, 7u, 0u, 7u, 4u,
        1u, 5u, 6u, 1u, 6u, 2u);
    return c[idx[i]];
}

void main() {
    GpuVoxelObject o = objects[inGpuIndex];
    vGpuIndex = inGpuIndex;
    // One cube per occupied coarse. Occupancy AABB covers holes after a cut.
    if (o.voxelSize <= 0.0) {
        gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
        return;
    }
    vec3 coarse = vec3(float(inPackedCoarse & 1023u),
                       float((inPackedCoarse >> 10) & 1023u),
                       float((inPackedCoarse >> 20) & 1023u));
    vec3 local = (coarse + unitCubeVert(uint(gl_VertexIndex))) * o.voxelSize;
    vec4 world = o.objectToWorld * vec4(local, 1.0);
    gl_Position = pc.proj * pc.view * world;
}
