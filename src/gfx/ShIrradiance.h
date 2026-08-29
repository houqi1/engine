#pragma once

#include <glm/glm.hpp>

#include <array>

// L2 irradiance SH: 9 RGB coefficients with cosine-lobe convolution baked in.
// Evaluate with the matching Y_lm basis (y-up) used during projection.
struct Sh9 {
  std::array<glm::vec3, 9> c{};
  bool valid = false;
};

namespace ShIrradiance {

// Project an equirectangular HDR (RGBA float, no vertical flip) to irradiance SH.
// UV/direction convention matches shaders/sky.frag.
Sh9 projectEquirect(const float* rgba, int width, int height);

// CPU helper for debugging / sanity checks.
glm::vec3 evaluate(const Sh9& sh, glm::vec3 normal);

}  // namespace ShIrradiance
