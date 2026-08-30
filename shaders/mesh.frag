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

layout(set = 0, binding = 1) uniform sampler2D shadowMap;

layout(set = 1, binding = 0) uniform sampler2D albedoMap;

layout(set = 1, binding = 1, std140) uniform MaterialUBO {
    vec4 baseColorFactor;
    float metallic;
    float roughness;
    float shOnly;
    float _pad1;
} material;

layout(set = 2, binding = 0) uniform samplerCube prefilteredMap;
layout(set = 2, binding = 1) uniform sampler2D brdfLUT;

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

vec3 FresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

float ShadowPCF(vec4 lightSpacePos) {
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
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

vec3 rotateYaw(vec3 v, float yaw) {
    float cy = cos(yaw);
    float sy = sin(yaw);
    return vec3(cy * v.x + sy * v.z, v.y, -sy * v.x + cy * v.z);
}

vec3 evalSHIrradiance(vec3 n) {
    float x = n.x;
    float y = n.y;
    float z = n.z;

    float Y0 = 0.282095;
    float Y1 = 0.488603 * y;
    float Y2 = 0.488603 * z;
    float Y3 = 0.488603 * x;
    float Y4 = 1.092548 * x * y;
    float Y5 = 1.092548 * y * z;
    float Y6 = 0.315392 * (3.0 * z * z - 1.0);
    float Y7 = 1.092548 * x * z;
    float Y8 = 0.546274 * (x * x - y * y);

    vec3 e = frame.ambientSH[0].rgb * Y0
           + frame.ambientSH[1].rgb * Y1
           + frame.ambientSH[2].rgb * Y2
           + frame.ambientSH[3].rgb * Y3
           + frame.ambientSH[4].rgb * Y4
           + frame.ambientSH[5].rgb * Y5
           + frame.ambientSH[6].rgb * Y6
           + frame.ambientSH[7].rgb * Y7
           + frame.ambientSH[8].rgb * Y8;
    return max(e, vec3(0.0));
}

vec3 skyAmbient(vec3 worldN) {
    if (frame.ambientScale <= 0.0) {
        return frame.ambientColor;
    }
    vec3 nEnv = rotateYaw(normalize(worldN), frame.skyYaw);
    return evalSHIrradiance(nEnv) * frame.skyIntensity * frame.ambientScale;
}

// Independent toggles: prefiltered cubemap and/or BRDF LUT.
vec3 evalSpecularIbl(vec3 N, vec3 V, float NdotV, vec3 F0, float roughness) {
    bool usePref = frame.enablePrefiltered > 0.5;
    bool useBrdf = frame.enableBrdfLut > 0.5;
    if (!usePref && !useBrdf) {
        return vec3(0.0);
    }

    vec3 Frough = FresnelSchlickRoughness(NdotV, F0, roughness);
    vec3 R = rotateYaw(reflect(-V, N), frame.skyYaw);
    float lod = roughness * max(frame.iblMaxLod, 0.0);
    vec3 prefiltered = textureLod(prefilteredMap, R, lod).rgb * frame.skyIntensity;
    vec2 brdf = texture(brdfLUT, vec2(NdotV, roughness)).rg;
    vec3 brdfTerm = Frough * brdf.x + brdf.y;

    if (usePref && useBrdf) {
        return prefiltered * brdfTerm * frame.specularIblScale;
    }
    if (usePref) {
        // Raw prefiltered environment at this roughness lod.
        return prefiltered * frame.specularIblScale;
    }
    // BRDF LUT only: R=scale, G=bias (visualize channels).
    return vec3(brdf.x, brdf.y, 0.0) * frame.specularIblScale;
}

void main() {
    vec3 N = normalize(vWorldNormal);

    // Debug probe: sky SH diffuse + specular IBL only (no direct light / shadows).
    if (material.shOnly > 0.5) {
        vec3 albedo = material.baseColorFactor.rgb;
        float metallic = material.metallic;
        float roughness = max(material.roughness, 0.04);
        vec3 V = normalize(frame.cameraPos - vWorldPos);
        float NdotV = max(dot(N, V), 0.0);
        vec3 F0 = mix(vec3(0.04), albedo, metallic);

        vec3 Frough = FresnelSchlickRoughness(NdotV, F0, roughness);
        vec3 kDibl = (vec3(1.0) - Frough) * (1.0 - metallic);
        vec3 diffuseIBL = kDibl * albedo * skyAmbient(N);
        vec3 specularIBL = evalSpecularIbl(N, V, NdotV, F0, roughness);

        outColor = vec4(diffuseIBL + specularIBL, 1.0);
        return;
    }

    vec4 albedoSample = texture(albedoMap, vUV, frame.mipLodBias) * material.baseColorFactor;
    vec3 albedo = albedoSample.rgb;
    float metallic = material.metallic;
    float roughness = max(material.roughness, 0.04);

    vec3 V = normalize(frame.cameraPos - vWorldPos);
    vec3 L = normalize(-frame.lightDir);
    vec3 H = normalize(V + L);
    float NdotV = max(dot(N, V), 0.0);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    // Direct lighting
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

    // Diffuse IBL from SH
    vec3 diffuseIBL = kD * albedo * skyAmbient(N);

    // Specular IBL (split-sum); each half can be toggled independently.
    vec3 specularIBL = evalSpecularIbl(N, V, NdotV, F0, roughness);
    if (frame.enablePrefiltered > 0.5 || frame.enableBrdfLut > 0.5) {
        vec3 Frough = FresnelSchlickRoughness(NdotV, F0, roughness);
        vec3 kDibl = (vec3(1.0) - Frough) * (1.0 - metallic);
        diffuseIBL = kDibl * albedo * skyAmbient(N);
    }

    vec3 color = Lo + diffuseIBL + specularIBL;
    outColor = vec4(color, albedoSample.a);
}
