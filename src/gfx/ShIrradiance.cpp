#include "gfx/ShIrradiance.h"

#include <algorithm>
#include <cmath>

namespace ShIrradiance {
namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kTwoPi = 6.28318530717958647692f;

// Real orthonormal SH basis (y-up), same layout as evaluate().
void evalBasis(const glm::vec3& n, float* y) {
  const float x = n.x;
  const float yy = n.y;
  const float z = n.z;

  y[0] = 0.282095f;
  y[1] = 0.488603f * yy;
  y[2] = 0.488603f * z;
  y[3] = 0.488603f * x;
  y[4] = 1.092548f * x * yy;
  y[5] = 1.092548f * yy * z;
  y[6] = 0.315392f * (3.0f * z * z - 1.0f);
  y[7] = 1.092548f * x * z;
  y[8] = 0.546274f * (x * x - yy * yy);
}

// Cosine-hemisphere convolution factors per band (Ramamoorthi).
float bandFactor(int index) {
  if (index == 0) {
    return kPi;
  }
  if (index < 4) {
    return (2.0f * kPi) / 3.0f;
  }
  return kPi / 4.0f;
}

glm::vec3 directionFromEquirectUV(float u, float v) {
  // Inverse of sky.frag directionToEquirectUv:
  //   u = phi/(2pi) + 0.5
  //   v = 0.5 - theta/pi , theta = asin(y)
  const float phi = (u - 0.5f) * kTwoPi;
  const float theta = (0.5f - v) * kPi;
  const float ct = std::cos(theta);
  const float st = std::sin(theta);
  return glm::vec3(ct * std::cos(phi), st, ct * std::sin(phi));
}

}  // namespace

Sh9 projectEquirect(const float* rgba, int width, int height) {
  Sh9 out{};
  if (!rgba || width <= 0 || height <= 0) {
    return out;
  }

  std::array<glm::vec3, 9> L{};
  const float dPhi = kTwoPi / static_cast<float>(width);
  const float dTheta = kPi / static_cast<float>(height);

  for (int y = 0; y < height; ++y) {
    const float v = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
    const float theta = (0.5f - v) * kPi;
    const float cosTheta = std::cos(theta);
    // Equirect texel solid angle on the sphere.
    const float weight = std::max(cosTheta, 0.0f) * dPhi * dTheta;
    if (weight <= 0.0f) {
      continue;
    }

    for (int x = 0; x < width; ++x) {
      const float u = (static_cast<float>(x) + 0.5f) / static_cast<float>(width);
      const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(width) +
                          static_cast<size_t>(x)) *
                         4u;
      const glm::vec3 color = glm::max(glm::vec3(rgba[idx], rgba[idx + 1], rgba[idx + 2]),
                                       glm::vec3(0.0f));

      const glm::vec3 dir = directionFromEquirectUV(u, v);
      float Y[9];
      evalBasis(dir, Y);

      for (int i = 0; i < 9; ++i) {
        L[i] += color * (Y[i] * weight);
      }
    }
  }

  // Bake cosine lobe so the shader reconstructs irradiance directly.
  for (int i = 0; i < 9; ++i) {
    out.c[i] = L[i] * bandFactor(i);
  }
  out.valid = true;
  return out;
}

glm::vec3 evaluate(const Sh9& sh, glm::vec3 normal) {
  if (!sh.valid) {
    return glm::vec3(0.0f);
  }
  const glm::vec3 n = glm::normalize(normal);
  float Y[9];
  evalBasis(n, Y);

  glm::vec3 result(0.0f);
  for (int i = 0; i < 9; ++i) {
    result += sh.c[i] * Y[i];
  }
  return glm::max(result, glm::vec3(0.0f));
}

}  // namespace ShIrradiance
