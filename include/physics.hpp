#pragma once

#include <memory>

#include <Eigen/Dense>

#include "common.hpp"
#include "particle.hpp"
#include "object.hpp"

namespace physics
{

template <typename T>
using Integrator = T(*)(const T& value, const double timestep, T (*)(const T&, const double));

struct PhysicsManager
{

  PhysicsManager(const double timestep, std::vector<ObjectBase>& objects, Integrator integrator) : timestep_{timestep}, objects_(std::move(objects)), integrator_(integrator);

  void updateObjects(const size_t idx);

  void applyGravity(const size_t idx);

  // void collisionCheckWall(Particle& particle, const double WINDOW_HEIGHT, const double WINDOW_WIDTH);

  void collisionCheckWall(const size_t idx, const double WINDOW_HEIGHT, const double WINDOW_WIDTH);

  void step();

private:
  double timestep_;
  std::vector<std::unique_ptr<ObjectBase>> objects_;
  Integrator integrator;
}

// void collisionCheckOtherParticles(Particle& particle_1, Particle& particle_2);

};  // namespace physics
