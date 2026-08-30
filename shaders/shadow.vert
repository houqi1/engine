#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(push_constant, std430) uniform PushConstants {
    mat4 model;
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
    float iblMaxLod;
    float specularIblScale;
    float enablePrefiltered;
    float enableBrdfLut;
    vec4 ambientSH[9];
} frame;

void main() {
    gl_Position = frame.lightViewProj * pc.model * vec4(inPosition, 1.0);
}
