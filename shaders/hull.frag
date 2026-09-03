#version 450

layout(location = 0) in vec3 worldPos;

layout(location = 0) out float outMin;
layout(location = 1) out float outMax;
layout(location = 2) out float outBack;

layout(push_constant) uniform HullPC {
    mat4 viewProj;
    mat4 model;
    vec3 cameraPos;
    float nearZ;
    vec3 cameraFwd;
    float _pad;
} pc;

void main() {
    vec3 delta = worldPos - pc.cameraPos;
    // Interpolated hull vertices can pass through the camera; length() then
    // collapses toward 0 and MIN/MAX t flicker (black sparkles).
    float alongFwd = dot(delta, pc.cameraFwd);
    if (alongFwd < pc.nearZ * 0.5) {
        discard;
    }
    vec3 rd = delta / max(length(delta), 1e-6);
    float t = dot(delta, rd);
    outMin = t;
    outMax = t;
    outBack = gl_FrontFacing ? 1.0e10 : t;
}
