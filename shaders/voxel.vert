#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec2 inUV;

layout(push_constant, std430) uniform PushConstants {
    mat4 model;
    vec4 color;
} pc;

layout(set = 0, binding = 0, std140) uniform FrameUBO {
    mat4 view;
    mat4 proj;
    vec3 cameraPos;
    float _pad0;
    vec3 lightDir;
    float _pad1;
} frame;

layout(location = 0) out vec3 vWorldNormal;
layout(location = 1) out vec3 vWorldPos;
layout(location = 2) out vec3 vColor;

void main() {
    vec4 worldPos = pc.model * vec4(inPosition, 1.0);
    vWorldPos = worldPos.xyz;
    vWorldNormal = mat3(pc.model) * inNormal;
    vColor = pc.color.rgb;
    gl_Position = frame.proj * frame.view * worldPos;
}
