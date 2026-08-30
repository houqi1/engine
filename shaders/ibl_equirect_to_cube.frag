#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D equirectMap;

layout(push_constant, std430) uniform PC {
    float face;        // 0..5 = +X,-X,+Y,-Y,+Z,-Z
    float roughness;   // unused here
    float resolution;  // unused here
    float _pad;
} pc;

const float PI = 3.14159265359;

// Inverse of Vulkan cubemap face selection (spec table).
// uv in [0,1], origin top-left of the face image.
vec3 faceUvToDir(int face, vec2 uv) {
    float uc = 2.0 * uv.x - 1.0;
    float vc = 2.0 * uv.y - 1.0;
    if (face == 0) return normalize(vec3( 1.0, -vc, -uc)); // +X
    if (face == 1) return normalize(vec3(-1.0, -vc,  uc)); // -X
    if (face == 2) return normalize(vec3( uc,  1.0,  vc)); // +Y
    if (face == 3) return normalize(vec3( uc, -1.0, -vc)); // -Y
    if (face == 4) return normalize(vec3( uc, -vc,  1.0)); // +Z
    return normalize(vec3(-uc, -vc, -1.0));               // -Z
}

void main() {
    vec3 dir = faceUvToDir(int(pc.face + 0.5), vUV);
    // Same equirect convention as sky.frag
    float phi = atan(dir.z, dir.x);
    float theta = asin(clamp(dir.y, -1.0, 1.0));
    vec2 uv = vec2(fract(phi / (2.0 * PI) + 0.5), 0.5 - theta / PI);
    outColor = vec4(texture(equirectMap, uv).rgb, 1.0);
}
