#pragma once

#include "physics/PhysicsTypes.h"
#include "physics/RigidBody.h"

#include <vector>

namespace physics {

void refreshInverseInertiaWorld(RigidBody& b);
void solveContacts(std::vector<RigidBody>& bodies, std::vector<Contact>& contacts, float hSub,
                   int iterations = kContactIters);
void integrateBodies(std::vector<RigidBody>& bodies, float hSub);
// World impulse on body A after SI. Normal part is collision-only (no Baumgarte).
glm::vec3 contactImpulseOnA(const Contact& c);

}  // namespace physics
