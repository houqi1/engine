#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(location = 3) in vec3 iPosition;
layout(location = 4) in float iYaw;
layout(location = 5) in vec3 iColor;
layout(location = 6) in float iScale;

layout(push_constant, std430) uniform GrassPC {
    float time;
    float windStrength;
    float windFrequency;
    float _pad;
} pc;

layout(set = 0, binding = 0, std140) uniform FrameUBO {
    mat4 view;
    mat4 proj;
    mat4 lightViewProj;
    vec3 cameraPos;
    float _pad0;
    vec3 lightDir;
    float _pad1;
    vec3 lightColor;
    float lightIntensity;
    vec3 ambientColor;
    float shadowBias;
    float mipLodBias;
    float skyYaw;
    float skyIntensity;
    float ambientScale;
    vec4 ambientSH[9];
} frame;

void main() {
    // inNormal / iColor declared to match shared vertex input layout.
    float cy = cos(iYaw);
    float sy = sin(iYaw);
    mat2 rot = mat2(cy, -sy, sy, cy);

    vec3 local = inPosition;
    local.y *= iScale;
    local.xz *= mix(0.85, 1.15, fract(iYaw * 3.7));

    float tip = clamp(inUV.y, 0.0, 1.0);
    float windPhase = iPosition.x * 0.35 + iPosition.z * 0.28 + iYaw * 2.0;
    float wind = sin(pc.time * pc.windFrequency + windPhase) * pc.windStrength;
    float wind2 = cos(pc.time * pc.windFrequency * 0.73 + windPhase * 1.3) * pc.windStrength * 0.45;
    local.x += (wind + wind2) * tip * tip * iScale;
    local.z += wind2 * tip * tip * iScale * 0.6;

    vec2 xz = rot * local.xz;
    vec3 world = vec3(xz.x, local.y, xz.y) + iPosition;

    gl_Position = frame.lightViewProj * vec4(world, 1.0);
}
