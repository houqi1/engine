#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

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

layout(set = 1, binding = 0) uniform sampler2D skyMap;

layout(push_constant, std430) uniform SkyPC {
    float intensity;
    float yaw;
    float _pad0;
    float _pad1;
} pc;

const float kPi = 3.14159265359;

vec2 directionToEquirectUv(vec3 dir) {
    // Poly Haven / Blender-style equirectangular mapping.
    float phi = atan(dir.z, dir.x);
    float theta = asin(clamp(dir.y, -1.0, 1.0));
    return vec2(fract(phi / (2.0 * kPi) + 0.5), 0.5 - theta / kPi);
}

void main() {
    vec2 ndc = vUV * 2.0 - 1.0;
    // Stable view-ray from reverse-Z / infinite perspective (camera looks down -Z).
    vec3 viewDir = normalize(vec3(ndc.x / frame.proj[0][0], ndc.y / frame.proj[1][1], -1.0));
    vec3 dir = normalize(mat3(transpose(mat3(frame.view))) * viewDir);

    float cy = cos(pc.yaw);
    float sy = sin(pc.yaw);
    dir = vec3(cy * dir.x + sy * dir.z, dir.y, -sy * dir.x + cy * dir.z);

    vec3 color = textureLod(skyMap, directionToEquirectUv(normalize(dir)), 0.0).rgb;
    outColor = vec4(max(color, vec3(0.0)) * pc.intensity, 1.0);
}
