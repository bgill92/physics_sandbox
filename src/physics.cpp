#include "dynamics.hpp"
#include "particle.hpp"
#include "physics.hpp"
#include "object.hpp"
#include "integrator.hpp"

#include <functional>
#include <iostream>
#include <type_traits>

namespace physics
{
void PhysicsManager::updateObject(const size_t idx)
{
  if ( Particle* particle = std::get_if<Particle>(&objects_.at(idx)))
  {
    const auto derivative_func = [&particle](const physics::State& derivative_at_state, const double timestep) -> physics::State
    {
      return particle->getDynamics().stateDerivative(derivative_at_state, timestep);
    };

    particle->getDynamics().getState() += integrator::RK4<physics::State>(particle->getDynamics().getState(), this->config_.timestep_physics, derivative_func);
  }  
}

void PhysicsManager::applyGravity(const size_t idx)
{
  if ( Particle* particle = std::get_if<Particle>(&objects_.at(idx)))
  {
    particle->getDynamics().getForce() = physics::Force({0, -9.81 * particle->getDynamics().getMass(), 0});
  }
}

void PhysicsManager::clearForces(const size_t idx)
{
  if ( Particle* particle = std::get_if<Particle>(&objects_.at(idx)))
  {
    particle->getDynamics().clearForce();
  }
}

void PhysicsManager::collisionCheckWall(const size_t idx)
{
  if ( Particle* particle = std::get_if<Particle>(&objects_.at(idx)))
  {
    particle->collisionCheckWall(config_.window_height, config_.window_width, config_.particle_COR);
  }
}

void PhysicsManager::step(const size_t idx)
{

  applyGravity(idx);

  auto physics::State previous_state;
  if (objects_.at(idx).index() == 0)
  {
    previous_state = std::get<Particle>(objects_.at(idx)).getDynamics().getState()
  }

  updateObject(idx);

  evaluateConstraint(idx, previous_state);

  // Resolve collisions with other objects
  for (size_t j = idx + 1; j < objects_.size(); j++)
  {
    // Both are particles
    if ((objects_.at(idx).index() == 0) && (objects_.at(j).index() == 0))
    {
      collisionCheck(std::get<Particle>(objects_.at(idx)), std::get<Particle>(objects_.at(j)));
    }    
  }  

  collisionCheckWall(idx); 

  clearForces(idx);

}

void PhysicsManager::collisionCheck(Particle& particle_1, Particle& particle_2)
{

  auto& dynamics_1 = particle_1.getDynamics();
  auto& dynamics_2 = particle_2.getDynamics();

  auto& state_1 = dynamics_1.getState();
  auto& state_2 = dynamics_2.getState();

  // Get the positions
  Eigen::Vector3d pos_1 = state_1.segment<3>(0);
  Eigen::Vector3d pos_2 = state_2.segment<3>(0);

  // Find the difference in position and normalize the difference
  Eigen::Vector3d delta_pos = (pos_2 - pos_1);

  // If the distance between the particles is greater than the sum of the radius, then they are not in collision
  if (delta_pos.norm() > (particle_1.getRadius() + particle_2.getRadius()))
  {
    return;
  }

  // How much to push the particle along the collision axis
  auto corr = (particle_1.getRadius() + particle_2.getRadius() - delta_pos.norm()) / 2.0;
  // Normalize the vector along the collision axis
  delta_pos.normalize();

  // Calculate the position adjustment and add it to the state of the first particle
  State pos_adjustment_1{ 0, 0, 0, 0, 0, 0 };
  pos_adjustment_1.head<3>() = delta_pos * -corr;
  state_1 += pos_adjustment_1;

  // Calculate the position adjustment and add it to the state of the second particle
  State pos_adjustment_2{ 0, 0, 0, 0, 0, 0 };
  pos_adjustment_2.head<3>() = delta_pos * corr;
  state_2 += pos_adjustment_2;

  // Get the velocity along the collision axis
  auto vel_axis_1 = (state_1.segment(3, 3)).dot(delta_pos);
  auto vel_axis_2 = (state_2.segment(3, 3)).dot(delta_pos);

  const auto m1 = dynamics_1.getMass();
  const auto m2 = dynamics_2.getMass();

  // Calculate the new velocities
  const auto new_vel_axis_1 =
      (m1 * vel_axis_1 + m2 * vel_axis_2 - m2 * (vel_axis_1 - vel_axis_2) * config_.particle_COR) / (m1 + m2);
  const auto new_vel_axis_2 =
      (m1 * vel_axis_1 + m2 * vel_axis_2 - m1 * (vel_axis_2 - vel_axis_1) * config_.particle_COR) / (m1 + m2);

  // Set the new velocities
  State vel_adjustment_1{ 0, 0, 0, 0, 0, 0 };
  vel_adjustment_1.tail<3>() = delta_pos * (new_vel_axis_1 - vel_axis_1);
  state_1 += vel_adjustment_1;

  State vel_adjustment_2{ 0, 0, 0, 0, 0, 0 };
  vel_adjustment_2.tail<3>() = delta_pos * (new_vel_axis_2 - vel_axis_2);
  state_2 += vel_adjustment_2;
}

void PhysicsManager::evaluateConstraint(const size_t idx)
{
  if ( PendulumConstraint* constraint = std::get_if<PendulumConstraint>(&constraints_.at(idx)))
  {

    particle->getDynamics().getState() += integrator::RK4<physics::State>(particle->getDynamics().getState(), this->config_.timestep_physics, derivative_func);
  } 
}

}  // namespace physics