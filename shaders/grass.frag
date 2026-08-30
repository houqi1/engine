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

vec3 rotateYaw(vec3 v, float yaw) {
    float cy = cos(yaw);
    float sy = sin(yaw);
    return vec3(cy * v.x + sy * v.z, v.y, -sy * v.x + cy * v.z);
}

// Matches CPU ShIrradiance basis; ambientSH already includes cosine lobe.
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

vec3 grassAmbient(vec3 worldN) {
    // Same yaw as sky.frag: evaluate unrotated SH in environment space.
    if (frame.ambientScale <= 0.0) {
        return frame.ambientColor;
    }
    vec3 nEnv = rotateYaw(normalize(worldN), frame.skyYaw);
    vec3 aPos = evalSHIrradiance(nEnv);
    vec3 aNeg = evalSHIrradiance(-nEnv);
    return max(aPos, aNeg) * frame.skyIntensity * frame.ambientScale;
}

void main() {
    // Blade silhouette comes from tapered geometry â€?no discard (keeps Early-Z).
    vec3 N = normalize(vWorldNormal);
    vec3 L = normalize(-frame.lightDir);
    // Two-sided lighting for thin blades.
    float wrap = abs(dot(N, L));
    wrap = mix(wrap, wrap * 0.65 + 0.35, 0.35);

    float shadow = ShadowPCF(vLightSpacePos);
    vec3 albedo = mix(vColor * 0.75, vColor * 1.15, vHeight);

    // Slight root darkening so blades sit better on the ground.
    vec3 ambient = grassAmbient(N) * mix(0.55, 1.0, vHeight);
    vec3 lit = albedo * (ambient + frame.lightColor * frame.lightIntensity * wrap * shadow);

    // Cheap tip translucency.
    float back = pow(1.0 - wrap, 2.0) * vHeight;
    lit += albedo * frame.lightColor * frame.lightIntensity * back * 0.15 * shadow;

    outColor = vec4(lit, 1.0);
}
