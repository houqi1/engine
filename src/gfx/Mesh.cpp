#include "gfx/Mesh.h"

#include "gfx/GfxDevice.h"

#include <cmath>
#include <vector>

namespace {

void uploadMesh(GfxDevice& gfx, Mesh& mesh, const std::vector<Vertex>& vertices,
                const std::vector<uint32_t>& indices) {
  mesh.vertexBuffer = gfx.createBuffer(
      vertices.size() * sizeof(Vertex),
      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
      VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  mesh.indexBuffer = gfx.createBuffer(
      indices.size() * sizeof(uint32_t),
      VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
      VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE);
  mesh.indexCount = static_cast<uint32_t>(indices.size());

  gfx.uploadToBuffer(mesh.vertexBuffer, vertices.data(), vertices.size() * sizeof(Vertex));
  gfx.uploadToBuffer(mesh.indexBuffer, indices.data(), indices.size() * sizeof(uint32_t));
}

}  // namespace

namespace MeshFactory {

Mesh createCube(GfxDevice& gfx) {
  // 24 unique verts for correct normals
  const Vertex verts[] = {
      {{-0.5f, -0.5f, 0.5f}, {0, 0, 1}, {0, 1}}, {{0.5f, -0.5f, 0.5f}, {0, 0, 1}, {1, 1}},
      {{0.5f, 0.5f, 0.5f}, {0, 0, 1}, {1, 0}},   {{-0.5f, 0.5f, 0.5f}, {0, 0, 1}, {0, 0}},
      {{0.5f, -0.5f, -0.5f}, {0, 0, -1}, {0, 1}}, {{-0.5f, -0.5f, -0.5f}, {0, 0, -1}, {1, 1}},
      {{-0.5f, 0.5f, -0.5f}, {0, 0, -1}, {1, 0}}, {{0.5f, 0.5f, -0.5f}, {0, 0, -1}, {0, 0}},
      {{-0.5f, -0.5f, -0.5f}, {-1, 0, 0}, {0, 1}}, {{-0.5f, -0.5f, 0.5f}, {-1, 0, 0}, {1, 1}},
      {{-0.5f, 0.5f, 0.5f}, {-1, 0, 0}, {1, 0}},  {{-0.5f, 0.5f, -0.5f}, {-1, 0, 0}, {0, 0}},
      {{0.5f, -0.5f, 0.5f}, {1, 0, 0}, {0, 1}},   {{0.5f, -0.5f, -0.5f}, {1, 0, 0}, {1, 1}},
      {{0.5f, 0.5f, -0.5f}, {1, 0, 0}, {1, 0}},   {{0.5f, 0.5f, 0.5f}, {1, 0, 0}, {0, 0}},
      {{-0.5f, 0.5f, 0.5f}, {0, 1, 0}, {0, 1}},   {{0.5f, 0.5f, 0.5f}, {0, 1, 0}, {1, 1}},
      {{0.5f, 0.5f, -0.5f}, {0, 1, 0}, {1, 0}},   {{-0.5f, 0.5f, -0.5f}, {0, 1, 0}, {0, 0}},
      {{-0.5f, -0.5f, -0.5f}, {0, -1, 0}, {0, 1}}, {{0.5f, -0.5f, -0.5f}, {0, -1, 0}, {1, 1}},
      {{0.5f, -0.5f, 0.5f}, {0, -1, 0}, {1, 0}},  {{-0.5f, -0.5f, 0.5f}, {0, -1, 0}, {0, 0}},
  };

  std::vector<uint32_t> indices;
  indices.reserve(36);
  for (uint32_t face = 0; face < 6; ++face) {
    const uint32_t b = face * 4;
    indices.push_back(b + 0);
    indices.push_back(b + 1);
    indices.push_back(b + 2);
    indices.push_back(b + 0);
    indices.push_back(b + 2);
    indices.push_back(b + 3);
  }

  Mesh mesh;
  uploadMesh(gfx, mesh, std::vector<Vertex>(std::begin(verts), std::end(verts)), indices);
  return mesh;
}

Mesh createPlane(GfxDevice& gfx, float size) {
  const float h = size * 0.5f;
  const std::vector<Vertex> verts = {
      {{-h, 0, h}, {0, 1, 0}, {0, 0}},
      {{h, 0, h}, {0, 1, 0}, {size, 0}},
      {{h, 0, -h}, {0, 1, 0}, {size, size}},
      {{-h, 0, -h}, {0, 1, 0}, {0, size}},
  };
  const std::vector<uint32_t> indices = {0, 1, 2, 0, 2, 3};

  Mesh mesh;
  uploadMesh(gfx, mesh, verts, indices);
  return mesh;
}

Mesh createSphere(GfxDevice& gfx, float radius, int segments) {
  segments = std::max(segments, 8);
  std::vector<Vertex> verts;
  std::vector<uint32_t> indices;

  for (int y = 0; y <= segments; ++y) {
    const float v = static_cast<float>(y) / static_cast<float>(segments);
    const float phi = v * 3.14159265f;
    for (int x = 0; x <= segments; ++x) {
      const float u = static_cast<float>(x) / static_cast<float>(segments);
      const float theta = u * 6.2831853f;
      const float sx = std::sin(phi) * std::cos(theta);
      const float sy = std::cos(phi);
      const float sz = std::sin(phi) * std::sin(theta);
      Vertex vert{};
      vert.position[0] = sx * radius;
      vert.position[1] = sy * radius;
      vert.position[2] = sz * radius;
      vert.normal[0] = sx;
      vert.normal[1] = sy;
      vert.normal[2] = sz;
      vert.uv[0] = u;
      vert.uv[1] = v;
      verts.push_back(vert);
    }
  }

  for (int y = 0; y < segments; ++y) {
    for (int x = 0; x < segments; ++x) {
      const uint32_t i0 = y * (segments + 1) + x;
      const uint32_t i1 = i0 + 1;
      const uint32_t i2 = i0 + (segments + 1);
      const uint32_t i3 = i2 + 1;
      // CCW when viewed from outside (matches cube/plane; +theta then +phi).
      indices.push_back(i0);
      indices.push_back(i1);
      indices.push_back(i2);
      indices.push_back(i1);
      indices.push_back(i3);
      indices.push_back(i2);
    }
  }

  Mesh mesh;
  uploadMesh(gfx, mesh, verts, indices);
  return mesh;
}

Mesh createGrassBlade(GfxDevice& gfx, int segments) {
  segments = std::max(segments, 2);
  std::vector<Vertex> verts;
  std::vector<uint32_t> indices;
  verts.reserve(static_cast<size_t>(segments + 1) * 2);
  indices.reserve(static_cast<size_t>(segments) * 6);

  constexpr float height = 1.0f;
  constexpr float halfWidth = 0.045f;

  for (int i = 0; i <= segments; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(segments);
    const float y = t * height;
    const float widthScale = (1.0f - t) * (1.0f - t * 0.35f);
    const float x = halfWidth * widthScale;

    Vertex left{};
    left.position[0] = -x;
    left.position[1] = y;
    left.position[2] = 0.0f;
    left.normal[0] = 0.0f;
    left.normal[1] = 0.0f;
    left.normal[2] = 1.0f;
    left.uv[0] = 0.0f;
    left.uv[1] = t;

    Vertex right{};
    right.position[0] = x;
    right.position[1] = y;
    right.position[2] = 0.0f;
    right.normal[0] = 0.0f;
    right.normal[1] = 0.0f;
    right.normal[2] = 1.0f;
    right.uv[0] = 1.0f;
    right.uv[1] = t;

    verts.push_back(left);
    verts.push_back(right);
  }

  for (int i = 0; i < segments; ++i) {
    const uint32_t i0 = static_cast<uint32_t>(i * 2);
    const uint32_t i1 = i0 + 1;
    const uint32_t i2 = i0 + 2;
    const uint32_t i3 = i0 + 3;
    indices.push_back(i0);
    indices.push_back(i2);
    indices.push_back(i1);
    indices.push_back(i1);
    indices.push_back(i2);
    indices.push_back(i3);
  }

  Mesh mesh;
  uploadMesh(gfx, mesh, verts, indices);
  return mesh;
}

void destroy(GfxDevice& gfx, Mesh& mesh) {
  gfx.destroyBuffer(mesh.vertexBuffer);
  gfx.destroyBuffer(mesh.indexBuffer);
  mesh = {};
}

}  // namespace MeshFactory
