#include "physics.hpp"
#include "object.hpp"
#include "particle.hpp"

#include <iostream>

namespace
{
  const double COEFFICIENT_OF_RESTITUTION = 0.5;
};

namespace physics
{
// TODO: Make this a templated function with T as the return type
// TODO: Make it such that the derivative function can be passed into runge-kutta

stateVector update(const AMatrix& A, const BMatrix& B, const stateVector& state, const commandVector& command,
                   const double timestep)
{
  // Update is Runge-Kutta 4
  // Essentially we calculate the derivative at four points, take a weighted average of the derivatives,
  // and find the next state using that weighted derivative average
  const auto evaluateDerivative = [&timestep](const AMatrix& A, const BMatrix& B, const stateVector& state,
                                              const stateVector& derivative, const commandVector& command) {
    // Calculate the next state given the derivative, and then calculate the derivative at the next state
    return A * (state + derivative * timestep) + B * command;
  };

  // k1 is the slope at the beginning of the time step
  // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
  // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the
  // midpoint. Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate
  // of the slope at the endpoint.
  const auto k1 = evaluateDerivative(A, B, state, stateVector(), command);
  const auto k2 = evaluateDerivative(A, B, state, k1 * timestep * 0.5, command);
  const auto k3 = evaluateDerivative(A, B, state, k2 * timestep * 0.5, command);
  const auto k4 = evaluateDerivative(A, B, state, k3 * timestep, command);

  return (1.0 / 6.0) * timestep * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
}

std::pair<AMatrix, BMatrix> generateAandBMatrices(const double timestep)
{
  // A and B matrices for simple linear Newtonian motion
  // The A matrix in this case just gets the velocities of the state
  AMatrix A{ { 0.0, 0.0, 0.0, 1.0, 0.0, 0.0 }, //
  					 { 0.0, 0.0, 0.0, 0.0, 1.0, 0.0 }, //
  					 { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 }, //
             { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, //
             { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, //
             { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };

  // The B matrix gets the acceleration due to the forces
  BMatrix B{ { 0.0, 0.0, 0.0 }, //
             { 0.0, 0.0, 0.0 }, //
             { 0.0, 0.0, 0.0 }, //
             { timestep, 0.0, 0.0 }, //
             { 0.0, timestep, 0.0 }, //
             { 0.0, 0.0, timestep } };

  return { A, B };
}

void updateObject(Object& object, const double timestep)
{
  const auto AandB = generateAandBMatrices(timestep);

  object.addState(update(AandB.first, AandB.second, object.getState(), object.getForces() / object.getMass(), timestep));
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

  // Calculate the position adjustment and add it to the state of the first particle 
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