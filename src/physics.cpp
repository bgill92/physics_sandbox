#include "dynamics.hpp"
#include "object.hpp"
#include "particle.hpp"
#include "physics.hpp"

#include <functional>
#include <iostream>

namespace
{
  const double COEFFICIENT_OF_RESTITUTION = 0.5;
};

namespace physics
{
template<typename T, typename U>
T update(const T& state, const U& forces, const double timestep, std::function<T(const T&, const T&, const U&, const double)> derivative_func)
{

  const auto bound_derivative_func = [&state, &forces, &derivative_func, &timestep](const T& delta_state)
  {return derivative_func(state, delta_state, forces, timestep);};

  // k1 is the slope at the beginning of the time step
  // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
  // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the
  // midpoint. Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate
  // of the slope at the endpoint.
  const T k1 = bound_derivative_func(stateVector().setZero());
  const T k2 = bound_derivative_func(k1 * timestep * 0.5);
  const T k3 = bound_derivative_func(k2 * timestep * 0.5);
  const T k4 = bound_derivative_func(k3 * timestep);

  return (1.0 / 6.0) * timestep * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

void updateObject(Object& object, const double timestep)
{
  const auto dynamics_model = dynamics::PointMass(timestep);

  const auto dynamics_update_func = [&dynamics_model](const physics::stateVector& state, const physics::stateVector& derivative, const physics::commandVector& forces, const double timestep)
  {return dynamics_model.derivativeAtState(state, derivative, forces, timestep);};

  object.addState(update<physics::stateVector, physics::commandVector>(object.getState(), object.getForces() / object.getMass(), timestep, dynamics_update_func));
}

void collisionCheckWall(Particle& particle, const double WINDOW_HEIGHT, const double WINDOW_WIDTH)
{
  // If a particle hits the bottom
  if ((particle.getState(physics::STATE_VECTOR_Y_POS_IDX) - particle.getRadius()) < 0)
  {
    particle.setState(physics::STATE_VECTOR_Y_POS_IDX, particle.getRadius());
    particle.setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
                      -1 * particle.getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
  // If a particle hits the top
  if ((particle.getState(physics::STATE_VECTOR_Y_POS_IDX) + particle.getRadius()) > WINDOW_HEIGHT)
  {
    particle.setState(physics::STATE_VECTOR_Y_POS_IDX, WINDOW_HEIGHT - particle.getRadius());
    particle.setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
                      -1 * particle.getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
  // If a particle hits the right side
  if ((particle.getRadius() + particle.getState(physics::STATE_VECTOR_X_POS_IDX)) > WINDOW_WIDTH)
  {
    particle.setState(physics::STATE_VECTOR_X_POS_IDX, WINDOW_WIDTH - particle.getRadius());
    particle.setState(physics::STATE_VECTOR_X_LIN_VEL_IDX,
                      -1 * particle.getState(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
  // If the particle hits the left side
  if (particle.getState(physics::STATE_VECTOR_X_POS_IDX) < particle.getRadius())
  {
    particle.setState(physics::STATE_VECTOR_X_POS_IDX, particle.getRadius());
    particle.setState(physics::STATE_VECTOR_X_LIN_VEL_IDX,
                      -1 * particle.getState(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
}

void collisionCheckOtherParticles(DrawableParticle& drawable_particle_1, DrawableParticle& drawable_particle_2)
{
  auto& particle_1 = drawable_particle_1.getParticle();
  auto& particle_2 = drawable_particle_2.getParticle();

  // Get the positions
  Eigen::Vector3d pos_1 = particle_1.getState().segment<3>(0);
  Eigen::Vector3d pos_2 = particle_2.getState().segment<3>(0);

  // Find the difference in position and normalize the difference
  Eigen::Vector3d delta_pos = (pos_2 - pos_1);

  // If the distance between the particles is greater than the sum of the radius, then they are not in collision
  if (delta_pos.norm() > (particle_1.getRadius() + particle_2.getRadius())) 
  {
    return;
  }

  // How much to push the particle along the collision axis
  auto corr = (particle_1.getRadius() + particle_2.getRadius() - delta_pos.norm())/2.0;
  // Normalize the vector along the collision axis
  delta_pos.normalize();

  // Calculate the position adjustment and add it to the state of the first particle 
  stateVector pos_adjustment_1 {0, 0, 0, 0, 0, 0};
  pos_adjustment_1.head<3>() = delta_pos*-corr;
  particle_1.addState(pos_adjustment_1);

  // Calculate the position adjustment and add it to the state of the second particle 
  stateVector pos_adjustment_2 {0, 0, 0, 0, 0, 0};
  pos_adjustment_2.head<3>() = delta_pos*corr;
  particle_2.addState(pos_adjustment_2);

  // Get the velocity along the collision axis
  auto vel_axis_1 = (particle_1.getState().segment(3,3)).dot(delta_pos);
  auto vel_axis_2 = (particle_2.getState().segment(3,3)).dot(delta_pos);

  const auto m1 = particle_1.getMass();
  const auto m2 = particle_2.getMass();

  // Calculate the new velocities
  const auto new_vel_axis_1 = (m1*vel_axis_1 + m2*vel_axis_2 - m2*(vel_axis_1 - vel_axis_2)*COEFFICIENT_OF_RESTITUTION)/(m1 + m2);
  const auto new_vel_axis_2 = (m1*vel_axis_1 + m2*vel_axis_2 - m1*(vel_axis_2 - vel_axis_1)*COEFFICIENT_OF_RESTITUTION)/(m1 + m2);

  // Set the new velocities
  stateVector vel_adjustment_1 {0, 0, 0, 0, 0, 0};
  vel_adjustment_1.tail<3>() = delta_pos*(new_vel_axis_1 - vel_axis_1);
  particle_1.addState(vel_adjustment_1);

  stateVector vel_adjustment_2 {0, 0, 0, 0, 0, 0};
  vel_adjustment_2.tail<3>() = delta_pos*(new_vel_axis_2 - vel_axis_2);
  particle_2.addState(vel_adjustment_2);
}

}  // namespace physics