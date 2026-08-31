#pragma once

#include <glm/glm.hpp>

struct GLFWwindow;

class Camera {
public:
  void setPerspective(float fovYDegrees, float aspect, float nearZ, float farZ);
  void setOrbitTarget(const glm::vec3& target);
  void setOrbitDistance(float distance);
  void setYawPitch(float yawRad, float pitchRad);
  void setPosition(const glm::vec3& position);

  void update(float aspect);
  void handleInput(GLFWwindow* window, float dt);

  glm::mat4 view() const { return view_; }
  glm::mat4 proj() const { return proj_; }
  glm::vec3 position() const { return position_; }
  glm::vec3 forward() const;
  glm::vec3 right() const;
  float nearZ() const { return nearZ_; }
  float farZ() const { return farZ_; }

private:
  void syncPositionFromOrbit();

  float fovY_ = 60.0f;
  float nearZ_ = 0.1f;
  float farZ_ = 200.0f;
  float yaw_ = 0.6f;
  float pitch_ = 0.45f;
  float distance_ = 12.0f;
  glm::vec3 target_{0.0f, 1.0f, 0.0f};
  glm::vec3 position_{0.0f};
  glm::mat4 view_{1.0f};
  glm::mat4 proj_{1.0f};

  bool rotating_ = false;
  double lastX_ = 0.0;
  double lastY_ = 0.0;
};
