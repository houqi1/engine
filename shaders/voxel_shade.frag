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

layout(set = 0, binding = 2) uniform sampler2D hitBuffer;

layout(set = 0, binding = 3, std430) readonly buffer BrickPool {
    uint bricks[];
} brickSlabs[8];

layout(set = 0, binding = 4) uniform sampler2D skyMap;

layout(set = 0, binding = 5, std430) readonly buffer ObjectBuffer {
    GpuVoxelObject objects[];
};

layout(set = 0, binding = 6, std430) readonly buffer ImportPalette {
    vec4 importPalette[256];
};

layout(set = 0, binding = 7, std430) readonly buffer OccupancyMip {
    uint occTiles[];
};

#include "voxel_trace.glsl"

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

const uint MODE_SHADED = 0u;
const uint MODE_ALBEDO = 1u;
const uint MODE_NORMAL = 2u;
const uint MODE_STEPS = 3u;
const uint MODE_COORD = 4u;
const uint MODE_AO = 5u;

vec2 directionToEquirectUv(vec3 dir) {
    float phi = atan(dir.z, dir.x);
    float theta = asin(clamp(dir.y, -1.0, 1.0));
    return vec2(fract(phi / (2.0 * kPi) + 0.5), 0.5 - theta / kPi);
}

vec3 ACESFilm(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

vec3 sampleSky(vec3 dir) {
    if (ubo.useSky == 0u) {
        return ubo.skyColor;
    }
    float cy = cos(ubo.skyYaw);
    float sy = sin(ubo.skyYaw);
    vec3 rotated = vec3(cy * dir.x + sy * dir.z, dir.y, -sy * dir.x + cy * dir.z);
    vec3 hdr = textureLod(skyMap, directionToEquirectUv(normalize(rotated)), 0.0).rgb;
    vec3 ldr = ACESFilm(max(hdr, vec3(0.0)) * ubo.skyIntensity);
    return pow(ldr, vec3(1.0 / 2.2));
}

vec3 rayDirFromUv(vec2 uv) {
    vec2 ndc = uv * 2.0 - 1.0;
    vec3 dirView = normalize(vec3(ndc.x / ubo.projX, ndc.y / ubo.projY, -1.0));
    return normalize((ubo.invView * vec4(dirView, 0.0)).xyz);
}

void main() {
    ivec2 pixel = ivec2(gl_FragCoord.xy);
    vec4 hitF = texelFetch(hitBuffer, pixel, 0);
    uvec4 hit = uvec4(floatBitsToUint(hitF.x), floatBitsToUint(hitF.y), floatBitsToUint(hitF.z),
                      floatBitsToUint(hitF.w));
    vec3 dir = rayDirFromUv(vUV);

    if (hit.x == 0u) {
        outColor = vec4(sampleSky(dir), 1.0);
        return;
    }

    uint objectIndex = hit.x - 1u;
    GpuVoxelObject o = objects[objectIndex];

    ivec3 mapPos = ivec3(int(hit.y & 1023u), int((hit.y >> 10) & 1023u), int((hit.y >> 20) & 1023u));
    ivec3 micro = ivec3(int(hit.z & 255u), int((hit.z >> 8) & 255u), int((hit.z >> 16) & 255u));
    ivec3 fine = ivec3(int((hit.z >> 24) & 3u), int((hit.z >> 26) & 3u), int((hit.z >> 28) & 3u));
    uint faceAxis = hit.w & 3u;
    uint faceSign = (hit.w >> 2) & 1u;
    bool usedMicro = ((hit.w >> 3) & 1u) != 0u;
    bool usedFine = ((hit.w >> 4) & 1u) != 0u;
    uint matId = (hit.w >> 8) & 255u;

    bvec3 mask = bvec3(faceAxis == 0u, faceAxis == 1u, faceAxis == 2u);
    vec3 sgn = vec3(1.0);
    if (faceAxis == 0u) {
        sgn.x = faceSign != 0u ? -1.0 : 1.0;
    } else if (faceAxis == 1u) {
        sgn.y = faceSign != 0u ? -1.0 : 1.0;
    } else {
        sgn.z = faceSign != 0u ? -1.0 : 1.0;
    }

    CoarseCell cell = readCell(o, mapPos);
    uint page = cell.brickPage;
    vec3 uvw = vec3(0.5);
    {
        vec3 Ol = (o.worldToObject * vec4(ubo.cameraPos, 1.0)).xyz;
        vec3 Dl = mat3(o.worldToObject) * dir;
        vec3 ro = Ol / o.voxelSize;
        vec3 rd = Dl;
        // Nested hits are inside the coarse cube. Intersect the packed face of the
        // actual micro/fine voxel — subdividing the coarse entry UV (via fract)
        // made AO crawl with the camera and clamp to occluded corners.
        float scale = usedFine ? 16.0 : (usedMicro ? 8.0 : 1.0);
        ivec3 vp = usedFine ? (mapPos * 16 + micro * 2 + fine)
                            : (usedMicro ? (mapPos * 8 + micro) : mapPos);
        vec3 vmin = vec3(vp) / scale;
        vec3 faceN = vec3(mask);
        float plane = dot(faceN, vmin) + (faceSign != 0u ? (1.0 / scale) : 0.0);
        float denom = dot(faceN, rd);
        float tHit = abs(denom) > 1e-12 ? (plane - dot(faceN, ro)) / denom : 0.0;
        uvw = clamp((ro + rd * tHit - vmin) * scale, vec3(0.0), vec3(1.0));
    }

    if (ubo.solidColor != 0u) {
        outColor = vec4(ubo.solidRgb, 1.0);
        return;
    }

    vec3 albedo = albedoForVoxel(o, matId, page, micro, fine, usedMicro, usedFine);
    vec3 normalLocal = -vec3(mask) * sgn;
    vec3 normalWorld = normalize(mat3(o.objectToWorld) * normalLocal);

    ivec3 solidPos = usedFine ? (mapPos * 16 + micro * 2 + fine)
                              : (usedMicro ? (mapPos * 8 + micro) : mapPos);
    uint aoSpace = usedFine ? 2u : (usedMicro ? 1u : 0u);
    float ao = evalVoxelAo(o, solidPos, mask, sgn, uvw, aoSpace, page);

    vec3 color;
    if (ubo.renderMode == MODE_ALBEDO) {
        color = albedo;
    } else if (ubo.renderMode == MODE_NORMAL) {
        color = normalWorld * 0.5 + 0.5;
    } else if (ubo.renderMode == MODE_COORD) {
        color = usedFine ? (vec3(fine) * 0.5)
                         : (usedMicro ? (vec3(micro) / 7.0)
                                      : (vec3(mapPos) / max(vec3(o.gridSize - uvec3(1u)), vec3(1.0))));
    } else if (ubo.renderMode == MODE_AO) {
        color = vec3(ao);
    } else if (ubo.renderMode == MODE_STEPS) {
        color = vec3(0.2);
    } else {
        vec3 L = normalize(-ubo.lightDir);
        float ndotl = max(dot(normalWorld, L), 0.0);
        color = albedo * (ubo.ambient + (1.0 - ubo.ambient) * ndotl) * ao;
    }
    outColor = vec4(color, 1.0);
}
