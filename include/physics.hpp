#pragma once

#include <variant>

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "common.hpp"
#include "particle.hpp"
#include "object.hpp"


namespace physics
{

struct CircleConstraint
{

  CircleConstraint() = delete;

  CircleConstraint(const physics::State& center_of_constraint, const double radius) : center_of_constraint_{center_of_constraint}, radius_{radius} {}

  physics::State getCenterOfConstraint() {return center_of_constraint_;}

  double getRadius() {return radius_;}

private: 
  physics::State center_of_constraint_;
  double radius_;
};

using Constraint = std::variant<CircleConstraint>;

struct PhysicsManager
{

  PhysicsManager(const Config& config, std::vector<Object>& objects, std::vector<Constraint>& constraints) : config_{config}, objects_{objects}, constraints_{constraints} {}

  void updateObject(const size_t idx);

  void applyGravity(const size_t idx);

  void clearForces(const size_t idx);

  void evaluateConstraint(const size_t idx);

  void collisionCheckWall(const size_t idx);

  void collisionCheck(Particle& particle_1, Particle& particle_2);

  void step(const size_t idx);

  void updateVelocityAfterConstraint(const size_t idx, const physics::State& previous_state);

private:
  Config config_;
  std::vector<Object>& objects_;
  std::vector<Constraint>& constraints_;
  double time_ {0};
  double total_energy_ {0};
};


};  // namespace physics
