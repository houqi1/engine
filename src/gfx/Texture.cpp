#include "gfx/Texture.h"

#include "gfx/GfxDevice.h"

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#include <stdexcept>
#include <string>
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

EquirectHdrData loadHdrEquirectData(const std::string& path) {
  stbi_set_flip_vertically_on_load(0);
  int width = 0;
  int height = 0;
  int components = 0;
  float* pixels = stbi_loadf(path.c_str(), &width, &height, &components, 4);
  if (!pixels || width <= 0 || height <= 0) {
    const char* reason = stbi_failure_reason();
    throw std::runtime_error("Failed to load HDR equirect '" + path + "': " +
                             (reason ? reason : "unknown"));
  }

  EquirectHdrData data;
  data.width = width;
  data.height = height;
  const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height) * 4u;
  data.rgba.assign(pixels, pixels + count);
  stbi_image_free(pixels);
  return data;
}

Texture createEquirectTexture(GfxDevice& gfx, const EquirectHdrData& data) {
  if (data.rgba.empty() || data.width <= 0 || data.height <= 0) {
    throw std::runtime_error("createEquirectTexture: empty HDR data");
  }

  const VkDeviceSize bytes = data.rgba.size() * sizeof(float);
  Texture tex;
  tex.image = gfx.createImage(
      {static_cast<uint32_t>(data.width), static_cast<uint32_t>(data.height), 1},
      VK_FORMAT_R32G32B32A32_SFLOAT,
      VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, VK_IMAGE_ASPECT_COLOR_BIT);
  gfx.uploadToImage(tex.image, data.rgba.data(), bytes,
                    {static_cast<uint32_t>(data.width), static_cast<uint32_t>(data.height), 1});

  // Equirect: wrap horizontally, clamp vertically to avoid pole seams.
  VkSamplerCreateInfo info{};
  info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  info.magFilter = VK_FILTER_LINEAR;
  info.minFilter = VK_FILTER_LINEAR;
  info.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
  info.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
  info.minLod = 0.0f;
  info.maxLod = 1.0f;
  if (vkCreateSampler(gfx.device(), &info, nullptr, &tex.sampler) != VK_SUCCESS) {
    gfx.destroyImage(tex.image);
    throw std::runtime_error("Failed to create sky sampler");
  }
  return tex;
}

Texture loadHdrEquirect(GfxDevice& gfx, const std::string& path) {
  return createEquirectTexture(gfx, loadHdrEquirectData(path));
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
