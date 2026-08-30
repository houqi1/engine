#version 450

layout(location = 0) in vec3 vWorldNormal;
layout(location = 1) in vec3 vWorldPos;
layout(location = 2) in vec3 vColor;

layout(set = 0, binding = 0, std140) uniform FrameUBO {
    mat4 view;
    mat4 proj;
    vec3 cameraPos;
    float _pad0;
    vec3 lightDir;
    float _pad1;
} frame;

layout(location = 0) out vec4 outColor;

void main() {
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-frame.lightDir);
    float ndotl = max(dot(N, L), 0.0);
    float ambient = 0.18;
    vec3 lit = vColor * (ambient + (1.0 - ambient) * ndotl);
    outColor = vec4(lit, 1.0);
}
