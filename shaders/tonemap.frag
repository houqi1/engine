#version 450

layout(location = 0) in vec2 vUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D hdrColor;

// ACES filmic tonemap (Narkowicz approx)
vec3 ACESFilm(vec3 x) {
    const float a = 2.51;
    const float b = 0.03;
    const float c = 2.43;
    const float d = 0.59;
    const float e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main() {
    vec3 hdr = texture(hdrColor, vUV).rgb;
    vec3 ldr = ACESFilm(hdr);
    // Approximate sRGB OETF
    ldr = pow(ldr, vec3(1.0 / 2.2));
    outColor = vec4(ldr, 1.0);
}
