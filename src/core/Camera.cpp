#include "core/Camera.h"

#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>

#include <algorithm>
#include <cmath>

void Camera::setPerspective(float fovYDegrees, float aspect, float nearZ, float farZ) {
  fovY_ = fovYDegrees;
  nearZ_ = nearZ;
  farZ_ = farZ;
  update(aspect);
}

void Camera::setOrbitTarget(const glm::vec3& target) {
  target_ = target;
  syncPositionFromOrbit();
}

void Camera::setOrbitDistance(float distance) {
  distance_ = std::max(0.5f, distance);
  syncPositionFromOrbit();
}

void Camera::setYawPitch(float yawRad, float pitchRad) {
  yaw_ = yawRad;
  pitch_ = pitchRad;
  syncPositionFromOrbit();
}

void Camera::setPosition(const glm::vec3& position) {
  position_ = position;
}

void Camera::syncPositionFromOrbit() {
  pitch_ = std::clamp(pitch_, -1.2f, 1.2f);
  position_.x = target_.x + distance_ * std::cos(pitch_) * std::sin(yaw_);
  position_.y = target_.y + distance_ * std::sin(pitch_);
  position_.z = target_.z + distance_ * std::cos(pitch_) * std::cos(yaw_);
}

glm::vec3 Camera::forward() const {
  // Match orbit look direction: from camera toward orbit target.
  return glm::vec3(-std::cos(pitch_) * std::sin(yaw_), -std::sin(pitch_),
                   -std::cos(pitch_) * std::cos(yaw_));
}

glm::vec3 Camera::right() const {
  return glm::normalize(glm::cross(forward(), glm::vec3(0.0f, 1.0f, 0.0f)));
}

void Camera::update(float aspect) {
  pitch_ = std::clamp(pitch_, -1.2f, 1.2f);

  view_ = glm::lookAt(position_, position_ + forward(), glm::vec3(0.0f, 1.0f, 0.0f));

  // Reverse-Z infinite-ish perspective: map near->1, far->0
  const float f = 1.0f / std::tan(glm::radians(fovY_) * 0.5f);
  proj_ = glm::mat4(0.0f);
  proj_[0][0] = f / aspect;
  proj_[1][1] = -f;  // flip Y for Vulkan NDC
  proj_[2][2] = 0.0f;
  proj_[2][3] = -1.0f;
  proj_[3][2] = nearZ_;
  proj_[3][3] = 0.0f;
}

void Camera::handleInput(GLFWwindow* window, float dt) {
  const float moveSpeed = 8.0f;
  const float rotateSpeed = 0.005f;

  const glm::vec3 fwd = forward();
  const glm::vec3 rgt = right();
  const glm::vec3 up(0.0f, 1.0f, 0.0f);

  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
    position_ += fwd * moveSpeed * dt;
  }
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
    position_ -= fwd * moveSpeed * dt;
  }
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
    position_ -= rgt * moveSpeed * dt;
  }
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
    position_ += rgt * moveSpeed * dt;
  }
  if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
    position_ -= up * moveSpeed * dt;
  }
  if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
    position_ += up * moveSpeed * dt;
  }

  if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
    double x = 0.0;
    double y = 0.0;
    glfwGetCursorPos(window, &x, &y);
    if (!rotating_) {
      rotating_ = true;
      lastX_ = x;
      lastY_ = y;
    } else {
      const float dx = static_cast<float>(x - lastX_);
      const float dy = static_cast<float>(y - lastY_);
      yaw_ -= dx * rotateSpeed;
      pitch_ += dy * rotateSpeed;
      lastX_ = x;
      lastY_ = y;
    }
  } else {
    rotating_ = false;
  }
}
