#version 450

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vWorldNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vLightSpacePos;
layout(location = 4) in vec3 vColor;
layout(location = 5) in float vHeight;

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
    float _pad2;
    float _pad3;
} frame;

layout(set = 0, binding = 1) uniform sampler2D shadowMap;

float ShadowPCF(vec4 lightSpacePos) {
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords.xy = projCoords.xy * 0.5 + 0.5;

    if (projCoords.z > 1.0 || projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0) {
        return 1.0;
    }

    float bias = frame.shadowBias * 1.5;
    float currentDepth = projCoords.z - bias;
    vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            float closest = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r;
            shadow += currentDepth > closest ? 0.0 : 1.0;
        }
    }
    return shadow / 9.0;
}

void main() {
    // Blade silhouette comes from tapered geometry — no discard (keeps Early-Z).
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-frame.lightDir);
    // Two-sided lighting for thin blades.
    float wrap = abs(dot(N, L));
    wrap = mix(wrap, wrap * 0.65 + 0.35, 0.35);

    float shadow = ShadowPCF(vLightSpacePos);
    vec3 albedo = mix(vColor * 0.75, vColor * 1.15, vHeight);
    vec3 lit = albedo * (frame.ambientColor + frame.lightColor * frame.lightIntensity * wrap * shadow);

    // Cheap tip translucency.
    float back = pow(1.0 - wrap, 2.0) * vHeight;
    lit += albedo * frame.lightColor * frame.lightIntensity * back * 0.15 * shadow;

    outColor = vec4(lit, 1.0);
}
