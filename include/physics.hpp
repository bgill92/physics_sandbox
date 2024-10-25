#pragma once

#include <variant>

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "common.hpp"
#include "particle.hpp"
#include "object.hpp"


namespace physics
{

struct PendulumConstraint
{

  PendulumConstraint() = delete;

  PendulumConstraint(const physics::State& center_of_contraint, const double length) : center_of_constraint_{center_of_constraint}, length_{length} {}

  physics::State getCenterOfConstraint() {return center_of_constraint_;}

  double getLength() {return length_;}

private: 
  physics::State center_of_constraint_;
  double length_;
}

using Constraint = std::variant<PendulumConstraint>;

struct PhysicsManager
{

  PhysicsManager(const Config& config, std::vector<Object>& objects) : config_{config}, objects_(objects) {}

  void updateObject(const size_t idx);

  void applyGravity(const size_t idx);

  void clearForces(const size_t idx);

  void evaluateConstraint(const size_t idx, const physics::State& previous_state);

  void collisionCheckWall(const size_t idx);

  void collisionCheck(Particle& particle_1, Particle& particle_2);

  void step(const size_t idx);

private:
  Config config_;
  std::vector<Object>& objects_;
  std::vector<Constraint>& constraints_;
};


};  // namespace physics
