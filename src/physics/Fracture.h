#pragma once

#include <cstdint>

namespace physics {

// Occupied fines below this are deleted instead of becoming a new Shape.
constexpr uint32_t kResidualFineLimit = 8u;

}  // namespace physics
