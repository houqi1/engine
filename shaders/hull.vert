#version 450

layout(location = 0) in vec3 inPos;
layout(location = 0) out vec3 worldPos;

layout(push_constant) uniform HullPC {
    mat4 viewProj;
    mat4 model;
    vec3 cameraPos;
    float nearZ;
    vec3 cameraFwd;
    float _pad;
} pc;

void main() {
    vec4 world = pc.model * vec4(inPos, 1.0);
    worldPos = world.xyz;
    gl_Position = pc.viewProj * world;
}
