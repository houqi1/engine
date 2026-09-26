#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include <cstdint>
#include <functional>
#include <vector>

namespace blast {

// A cubic fine grid placed in the world: world = position + rotation * (local - halfExtent),
// local measured in meters from the grid corner (the VoxelObject convention).
struct AnchorGridFrame {
  glm::vec3 position{0.0f};
  glm::quat rotation{1.0f, 0.0f, 0.0f, 0.0f};
  float fineSize = 0.1f;
  int fineN = 0;
};

using FineSolidFn = std::function<bool(int x, int y, int z)>;

struct AnchorSource {
  AnchorGridFrame frame;
  FineSolidFn solid;
  // Another mounted structure. Touching it cannot anchor (it may break and leave a
  // ghost support) and is reported so the caller can refuse the mount.
  bool blocksMount = false;
};

struct GroundAnchorResult {
  int fineN = 0;
  std::vector<uint8_t> anchor;  // fineN^3, index x + n * (y + n * z)
  uint32_t anchorFines = 0;     // fines with at least one face on a source
  uint32_t contactFaces = 0;    // exposed faces whose neighbour cell is solid in a source
  uint32_t blockedFaces = 0;    // exposed faces touching a blocksMount source
  bool isAnchor(int x, int y, int z) const {
    return x >= 0 && y >= 0 && z >= 0 && x < fineN && y < fineN && z < fineN &&
           anchor[static_cast<size_t>(x + fineN * (y + fineN * z))] != 0;
  }
};

// Mount-time anchor detection (E5.2). A solid fine is an anchor when one of its exposed
// faces lies against a solid fine of a static source: the point paddingFines beyond
// the face centre (0.5 = the neighbouring cell centre) must fall inside the source.
// Only the layer actually in contact is marked, so the world-bond area equals the
// real contact area.
GroundAnchorResult findGroundAnchors(const AnchorGridFrame& object, const FineSolidFn& solid,
                                     const std::vector<AnchorSource>& sources, float paddingFines = 0.5f);

}  // namespace blast
