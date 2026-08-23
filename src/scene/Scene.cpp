#include "scene/Scene.h"

#include "gfx/GfxDevice.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>

void Scene::init(GfxDevice& gfx) {
  cube_ = MeshFactory::createCube(gfx);
  plane_ = MeshFactory::createPlane(gfx, 24.0f);
  sphere_ = MeshFactory::createSphere(gfx, 0.75f, 48);

  checker_ = TextureFactory::createCheckerboard(gfx, 512, 32);
  white_ = TextureFactory::createSolid(gfx, 0.85f, 0.85f, 0.88f);
  rust_ = TextureFactory::createSolid(gfx, 0.72f, 0.28f, 0.18f);
  gold_ = TextureFactory::createSolid(gfx, 1.0f, 0.76f, 0.33f);

  matFloor_ = Material{&checker_, glm::vec4(1.0f), 0.0f, 0.85f};
  matCube_ = Material{&rust_, glm::vec4(1.0f), 0.1f, 0.45f};
  matSphere_ = Material{&white_, glm::vec4(0.2f, 0.45f, 0.95f, 1.0f), 0.05f, 0.25f};
  matGold_ = Material{&gold_, glm::vec4(1.0f), 1.0f, 0.2f};

  objects_.clear();
  objects_.push_back(RenderObject{&plane_, &matFloor_, glm::mat4(1.0f), true});

  objects_.push_back(RenderObject{
      &cube_, &matCube_,
      glm::translate(glm::mat4(1.0f), glm::vec3(-2.0f, 0.5f, 0.0f)), true});
  objects_.push_back(RenderObject{
      &sphere_, &matSphere_,
      glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, 0.75f, 1.0f)), true});
  objects_.push_back(RenderObject{
      &sphere_, &matGold_,
      glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.75f, -2.0f)), true});
  objects_.push_back(RenderObject{
      &cube_, &matCube_,
      glm::translate(glm::mat4(1.0f), glm::vec3(3.5f, 0.5f, -1.5f)) *
          glm::rotate(glm::mat4(1.0f), 0.4f, glm::vec3(0, 1, 0)),
      true});

  camera_.setOrbitTarget(glm::vec3(0.0f, 1.0f, 0.0f));
  camera_.setOrbitDistance(11.0f);
  camera_.setYawPitch(0.7f, 0.4f);
}

void Scene::cleanup(GfxDevice& gfx) {
  objects_.clear();
  TextureFactory::destroy(gfx, checker_);
  TextureFactory::destroy(gfx, white_);
  TextureFactory::destroy(gfx, rust_);
  TextureFactory::destroy(gfx, gold_);
  MeshFactory::destroy(gfx, cube_);
  MeshFactory::destroy(gfx, plane_);
  MeshFactory::destroy(gfx, sphere_);
}

void Scene::update(float dt) {
  time_ += dt;
  if (objects_.size() > 2) {
    objects_[2].transform =
        glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, 0.75f + 0.2f * std::sin(time_ * 1.5f), 1.0f));
  }
  if (objects_.size() > 3) {
    objects_[3].transform =
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.75f, -2.0f)) *
        glm::rotate(glm::mat4(1.0f), time_ * 0.7f, glm::vec3(0, 1, 0));
  }
}
