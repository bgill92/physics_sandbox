#pragma once

#include <concepts>

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "common.hpp"
#include "particle.hpp"
#include "object.hpp"


namespace physics
{

struct PhysicsManager
{

  PhysicsManager(const double timestep, std::vector<Object>& objects) : timestep_{timestep}, objects_(objects) {}

  void updateObject(const size_t idx);

  void applyGravity(const size_t idx);

  void clearForces(const size_t idx);

  void collisionCheckWall(const size_t idx, const double WINDOW_HEIGHT, const double WINDOW_WIDTH);

  void collisionCheck(Particle& particle_1, Particle& particle_2);

  void step(const size_t idx);

private:
  double timestep_;
  std::vector<Object>& objects_;
};


};  // namespace physics
