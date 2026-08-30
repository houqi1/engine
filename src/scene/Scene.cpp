#include "scene/Scene.h"

#include "gfx/GfxDevice.h"

#include <glm/gtc/matrix_transform.hpp>

#include <cmath>
#include <iostream>
#include <string>

void Scene::init(GfxDevice& gfx) {
  cube_ = MeshFactory::createCube(gfx);
  plane_ = MeshFactory::createPlane(gfx, 24.0f);
  sphere_ = MeshFactory::createSphere(gfx, 0.75f, 48);
  probeSphere_ = MeshFactory::createSphere(gfx, 1.1f, 64);

  checker_ = TextureFactory::createCheckerboard(gfx, 512, 32);
  white_ = TextureFactory::createSolid(gfx, 0.85f, 0.85f, 0.88f);
  rust_ = TextureFactory::createSolid(gfx, 0.72f, 0.28f, 0.18f);
  gold_ = TextureFactory::createSolid(gfx, 1.0f, 0.76f, 0.33f);

  const std::string skyPath = std::string(VE_ASSETS_DIR) + "/sky/autumn_field_puresky_2k.hdr";
  const std::string iblCacheDir = std::string(VE_ASSETS_DIR) + "/sky/cache";
  try {
    EquirectHdrData skyHdr = TextureFactory::loadHdrEquirectData(skyPath);
    sky_ = TextureFactory::createEquirectTexture(gfx, skyHdr);
    std::cout << "Loaded skybox: " << skyPath << " (" << skyHdr.width << "x" << skyHdr.height << ")"
              << std::endl;
    ibl_ = IblBake::buildOrLoad(gfx, skyHdr, skyPath, iblCacheDir, &skyIrradianceSH_);
  } catch (const std::exception& e) {
    std::cerr << "Skybox/IBL load failed: " << e.what() << std::endl;
    skyIrradianceSH_ = {};
    IblBake::destroy(gfx, ibl_);
  }

  matFloor_ = Material{&checker_, glm::vec4(1.0f), 0.0f, 0.85f};
  matCube_ = Material{&rust_, glm::vec4(1.0f), 0.1f, 0.45f};
  matSphere_ = Material{&white_, glm::vec4(0.2f, 0.45f, 0.95f, 1.0f), 0.05f, 0.25f};
  matGold_ = Material{&gold_, glm::vec4(1.0f), 1.0f, 0.2f};
  // SH + specular IBL debug probe (no direct light). Mid roughness so reflections read clearly.
  matProbe_ = Material{&white_, glm::vec4(1.0f), 0.0f, 0.25f, true};

  objects_.clear();
  objects_.push_back(RenderObject{&plane_, &matFloor_, glm::mat4(1.0f), true});

  // SH irradiance debug sphere in the grass clearing (static, no shadow cast).
  objects_.push_back(RenderObject{
      &probeSphere_, &matProbe_,
      glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 1.15f, 0.0f)), false});

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

  grass_.init(gfx);
}

void Scene::cleanup(GfxDevice& gfx) {
  objects_.clear();
  grass_.cleanup(gfx);
  IblBake::destroy(gfx, ibl_);
  TextureFactory::destroy(gfx, sky_);
  TextureFactory::destroy(gfx, checker_);
  TextureFactory::destroy(gfx, white_);
  TextureFactory::destroy(gfx, rust_);
  TextureFactory::destroy(gfx, gold_);
  MeshFactory::destroy(gfx, cube_);
  MeshFactory::destroy(gfx, plane_);
  MeshFactory::destroy(gfx, sphere_);
  MeshFactory::destroy(gfx, probeSphere_);
}

void Scene::update(float dt) {
  time_ += dt;
  // objects_[0]=plane, [1]=SH probe (static), [2]=cube, [3]=blue sphere, [4]=gold sphere
  if (objects_.size() > 3) {
    objects_[3].transform =
        glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, 0.75f + 0.2f * std::sin(time_ * 1.5f), 1.0f));
  }
  if (objects_.size() > 4) {
    objects_[4].transform =
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.75f, -2.0f)) *
        glm::rotate(glm::mat4(1.0f), time_ * 0.7f, glm::vec3(0, 1, 0));
  }
}
