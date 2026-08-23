#include "gfx/Texture.h"

#include "gfx/GfxDevice.h"

#include <vector>

namespace TextureFactory {

Texture createSolid(GfxDevice& gfx, float r, float g, float b, float a) {
  const uint8_t pixel[4] = {
      static_cast<uint8_t>(r * 255.0f),
      static_cast<uint8_t>(g * 255.0f),
      static_cast<uint8_t>(b * 255.0f),
      static_cast<uint8_t>(a * 255.0f),
  };

  Texture tex;
  tex.image = gfx.createImage({1, 1, 1}, VK_FORMAT_R8G8B8A8_SRGB,
                              VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                              VK_IMAGE_ASPECT_COLOR_BIT);
  gfx.uploadToImage(tex.image, pixel, 4, {1, 1, 1});
  tex.sampler = gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, false);
  return tex;
}

Texture createCheckerboard(GfxDevice& gfx, uint32_t size, uint32_t checkSize) {
  std::vector<uint8_t> pixels(size * size * 4);
  for (uint32_t y = 0; y < size; ++y) {
    for (uint32_t x = 0; x < size; ++x) {
      const bool dark = ((x / checkSize) + (y / checkSize)) % 2 == 0;
      const uint8_t c = dark ? 48 : 210;
      const size_t i = (y * size + x) * 4;
      pixels[i + 0] = c;
      pixels[i + 1] = c;
      pixels[i + 2] = static_cast<uint8_t>(dark ? 56 : 220);
      pixels[i + 3] = 255;
    }
  }

  const uint32_t mipLevels = GfxDevice::calcMipLevels(size, size);
  Texture tex;
  // TRANSFER_SRC needed for mipmap blit chain.
  tex.image = gfx.createImage(
      {size, size, 1}, VK_FORMAT_R8G8B8A8_SRGB,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT,
      VK_IMAGE_ASPECT_COLOR_BIT, false, mipLevels);
  gfx.uploadToImage(tex.image, pixels.data(), pixels.size(), {size, size, 1}, true);
  // Anisotropic + mips. LOD bias is applied in mesh.frag (MoltenVK lacks samplerMipLodBias).
  tex.sampler =
      gfx.createSampler(VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, true, mipLevels);
  return tex;
}

void destroy(GfxDevice& gfx, Texture& texture) {
  if (texture.sampler) {
    gfx.destroySampler(texture.sampler);
    texture.sampler = VK_NULL_HANDLE;
  }
  gfx.destroyImage(texture.image);
  texture = {};
}

}  // namespace TextureFactory
