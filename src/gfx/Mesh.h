#pragma once

#include "gfx/GpuTypes.h"

#include <cstdint>
#include <vector>

class GfxDevice;

struct Mesh {
  AllocatedBuffer vertexBuffer{};
  AllocatedBuffer indexBuffer{};
  uint32_t indexCount = 0;
};

namespace MeshFactory {
Mesh createCube(GfxDevice& gfx);
Mesh createPlane(GfxDevice& gfx, float size);
Mesh createSphere(GfxDevice& gfx, float radius, int segments);
Mesh createGrassBlade(GfxDevice& gfx, int segments = 5);
void destroy(GfxDevice& gfx, Mesh& mesh);
}  // namespace MeshFactory
