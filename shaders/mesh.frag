#version 450

layout(location = 0) in vec3 vWorldPos;
layout(location = 1) in vec3 vWorldNormal;
layout(location = 2) in vec2 vUV;
layout(location = 3) in vec4 vLightSpacePos;

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
} frame;

layout(set = 0, binding = 1) uniform sampler2D shadowMap;

layout(set = 1, binding = 0) uniform sampler2D albedoMap;

layout(set = 1, binding = 1, std140) uniform MaterialUBO {
    vec4 baseColorFactor;
    float metallic;
    float roughness;
    float _pad0;
    float _pad1;
} material;

const float PI = 3.14159265359;

float DistributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float NdotH2 = NdotH * NdotH;
    float denom = NdotH2 * (a2 - 1.0) + 1.0;
    return a2 / (PI * denom * denom);
}

float GeometrySchlickGGX(float NdotV, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return NdotV / (NdotV * (1.0 - k) + k);
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    float NdotV = max(dot(N, V), 0.0);
    float NdotL = max(dot(N, L), 0.0);
    return GeometrySchlickGGX(NdotV, roughness) * GeometrySchlickGGX(NdotL, roughness);
}

vec3 FresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float ShadowPCF(vec4 lightSpacePos) {
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    // Vulkan NDC: xy in [-1,1] -> [0,1], depth already [0,1]
    projCoords.xy = projCoords.xy * 0.5 + 0.5;

    if (projCoords.z > 1.0 || projCoords.x < 0.0 || projCoords.x > 1.0 ||
        projCoords.y < 0.0 || projCoords.y > 1.0) {
        return 1.0;
    }

    float bias = frame.shadowBias;
    float currentDepth = projCoords.z - bias;

    vec2 texelSize = 1.0 / vec2(textureSize(shadowMap, 0));
    float shadow = 0.0;
    for (int x = -1; x <= 1; ++x) {
        for (int y = -1; y <= 1; ++y) {
            vec2 offset = vec2(float(x), float(y)) * texelSize;
            float closest = texture(shadowMap, projCoords.xy + offset).r;
            shadow += currentDepth > closest ? 0.0 : 1.0;
        }
    }
    return shadow / 9.0;
}

void main() {
    // Positive bias picks lower mips sooner — reduces ground moiré on grazing angles.
    // (Sampler mipLodBias is unavailable under MoltenVK portability.)
    const float kMipLodBias = 0.5;
    vec4 albedoSample = texture(albedoMap, vUV, kMipLodBias) * material.baseColorFactor;
    vec3 albedo = albedoSample.rgb;
    float metallic = material.metallic;
    float roughness = max(material.roughness, 0.04);

    vec3 N = normalize(vWorldNormal);
    vec3 V = normalize(frame.cameraPos - vWorldPos);
    vec3 L = normalize(-frame.lightDir);
    vec3 H = normalize(V + L);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    float NDF = DistributionGGX(N, H, roughness);
    float G = GeometrySmith(N, V, L, roughness);
    vec3 F = FresnelSchlick(max(dot(H, V), 0.0), F0);

    vec3 numerator = NDF * G * F;
    float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0) + 0.0001;
    vec3 specular = numerator / denominator;

    vec3 kS = F;
    vec3 kD = (vec3(1.0) - kS) * (1.0 - metallic);

    float NdotL = max(dot(N, L), 0.0);
    vec3 radiance = frame.lightColor * frame.lightIntensity;
    float shadow = ShadowPCF(vLightSpacePos);

    vec3 Lo = (kD * albedo / PI + specular) * radiance * NdotL * shadow;
    vec3 ambient = frame.ambientColor * albedo;
    vec3 color = ambient + Lo;

    outColor = vec4(color, albedoSample.a);
}
