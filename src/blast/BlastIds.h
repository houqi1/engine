#pragma once

#include <cstdint>
#include <limits>

namespace blast {

// Application-stable IDs. SDK chunkIndex / graphNodeIndex / bondIndex are
// not durable across reorder, split, or family rebuild.
constexpr uint32_t kInvalidIndex = (std::numeric_limits<uint32_t>::max)();

struct StableIds {
  uint32_t baseNode = 100;
  uint32_t topNode = 101;
  uint32_t internalBond = 200;
  uint32_t worldBond = 201;
};

struct IdMaps {
  StableIds stable{};

  uint32_t chunkBase = kInvalidIndex;
  uint32_t chunkTop = kInvalidIndex;

  uint32_t graphBase = kInvalidIndex;
  uint32_t graphTop = kInvalidIndex;
  uint32_t graphWorld = kInvalidIndex;

  uint32_t bondInternal = kInvalidIndex;
  uint32_t bondWorld = kInvalidIndex;
};

}  // namespace blast
