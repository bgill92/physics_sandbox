#include "dynamics.hpp"
#include "particle.hpp"
#include "physics.hpp"
#include "object.hpp"

#include <functional>
#include <iostream>

namespace
{
const double COEFFICIENT_OF_RESTITUTION = 1.0;
const unsigned int WINDOW_WIDTH = 1000;
const unsigned int WINDOW_HEIGHT = 1000;
};

namespace physics
{
// // TODO: Make std::function argument a function pointer and constrain it via a concept
// template <typename T, typename U>
// T update(const T& state, const U& forces, const double timestep,
//          std::function<T(const T&, const T&, const U&, const double)> derivative_func)
// {
//   const auto bound_derivative_func = [&state, &forces, &derivative_func, &timestep](const T& delta_state) {
//     return derivative_func(state, delta_state, forces, timestep);
//   };

//   // k1 is the slope at the beginning of the time step
//   // If we use the slope k1 to step halfway through the time step, then k2 is an estimate of the slope at the midpoint.
//   // If we use the slope k2 to step halfway through the time step, then k3 is another estimate of the slope at the
//   // midpoint. Finally, we use the slope, k3, to step all the way across the time step (to t₀+h), and k4 is an estimate
//   // of the slope at the endpoint.
//   const T k1 = bound_derivative_func(stateVector().setZero());
//   const T k2 = bound_derivative_func(k1 * timestep * 0.5);
//   const T k3 = bound_derivative_func(k2 * timestep * 0.5);
//   const T k4 = bound_derivative_func(k3 * timestep);

//   return (1.0 / 6.0) * timestep * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
// }

// void updateParticle(Particle& particle, const double timestep)
// {
//   const auto dynamics_update_func = [&particle](const physics::stateVector& state,
//                                                 const physics::stateVector& derivative,
//                                                 const physics::commandVector& forces, const double timestep) {
//     return particle.getDynamics().derivativeAtState(derivative, timestep);
//   };

//   particle.getDynamics().getStateObject().addState(update<physics::stateVector, physics::commandVector>(
//       particle.getDynamics().getStateObject().getState(), particle.getDynamics().getStateObject().getForces() / particle.getDynamics().getStateObject().getMass(), timestep, dynamics_update_func));
// }

void PhysicsManager::updateObject(const size_t idx)
{
  const auto& object = objects_.at(idx);
  const auto derivative_func = [&object](const physics::stateVector& derivative_at_state, const double timestep)
  {
    return object->getDynamics().stateDerivative(derivative_at_state, timestep);
  };
  object->setStateVector(integrator_(object->getStateVector(), this->timestep_, derivative_func));
}

void PhysicsManager::applyGravity(const size_t idx)
{
  const auto& object = objects_.at(idx);
  object->getState().addForceVector({0, -9.81 * object.getMass(), 0});
}

// void updateObject(ObjectBase& object, const double timestep)
// {
//   const auto dynamics_update_func = [&object](const physics::stateVector& state,
//                                                 const physics::stateVector& derivative,
//                                                 const physics::commandVector& forces, const double timestep) {
//     return object.getDynamics().derivativeAtState(derivative, timestep);
//   };

//   const auto update<physics::stateVector, physics::commandVector>(object.getState(), object.getAcceleration(), timestep, dynamics_update_func)

//   object.getDynamics().getStateObject().addState(update<physics::stateVector, physics::commandVector>(
//       object.getState(), object.getAcceleration(), timestep, dynamics_update_func));
// }

void PhysicsManager::collisionCheckWall(const size_t idx, const double WINDOW_HEIGHT, const double WINDOW_WIDTH)
{
  const auto& object = objects_.at(idx);

  // If a particle hits the bottom
  if ((object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_Y_POS_IDX) - object->getRadius()) < 0)
  {
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_Y_POS_IDX, object->getRadius());
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
                      -1 * object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
  // If a particle hits the top
  if ((object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_Y_POS_IDX) + object->getRadius()) > WINDOW_HEIGHT)
  {
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_Y_POS_IDX, WINDOW_HEIGHT - object->getRadius());
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
                      -1 * object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
  // If a particle hits the right side
  if ((object->getRadius() + object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_X_POS_IDX)) > WINDOW_WIDTH)
  {
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_X_POS_IDX, WINDOW_WIDTH - object->getRadius());
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_X_LIN_VEL_IDX,
                      -1 * object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
  // If the particle hits the left side
  if (object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_X_POS_IDX) < object->getRadius())
  {
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_X_POS_IDX, object->getRadius());
    object->getState().setStateVectorAtIdx(physics::STATE_VECTOR_X_LIN_VEL_IDX,
                      -1 * object->getState().getStateVectorAtIdx(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
  }
}

void PhysicsManager::step()
{

  for (size_t idx = 0; idx < objects_.size(), idx++)
  {

    applyGravity(idx);

    updateObjects(idx);

    collisionCheckWall(idx, WINDOW_HEIGHT, WINDOW_WIDTH);    

  }

}

// void collisionCheckWall(Particle& particle, const double WINDOW_HEIGHT, const double WINDOW_WIDTH)
// {
//   // If a particle hits the bottom
//   if ((particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_POS_IDX) - particle.getRadius()) < 0)
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_POS_IDX, particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
//   // If a particle hits the top
//   if ((particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_POS_IDX) + particle.getRadius()) > WINDOW_HEIGHT)
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_POS_IDX, WINDOW_HEIGHT - particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
//   // If a particle hits the right side
//   if ((particle.getRadius() + particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_POS_IDX)) > WINDOW_WIDTH)
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_POS_IDX, WINDOW_WIDTH - particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
//   // If the particle hits the left side
//   if (particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_POS_IDX) < particle.getRadius())
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_POS_IDX, particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
// }

// void collisionCheckWall(Particle& particle, const double WINDOW_HEIGHT, const double WINDOW_WIDTH)
// {
//   // If a particle hits the bottom
//   if ((particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_POS_IDX) - particle.getRadius()) < 0)
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_POS_IDX, particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
//   // If a particle hits the top
//   if ((particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_POS_IDX) + particle.getRadius()) > WINDOW_HEIGHT)
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_POS_IDX, WINDOW_HEIGHT - particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_Y_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
//   // If a particle hits the right side
//   if ((particle.getRadius() + particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_POS_IDX)) > WINDOW_WIDTH)
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_POS_IDX, WINDOW_WIDTH - particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
//   // If the particle hits the left side
//   if (particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_POS_IDX) < particle.getRadius())
//   {
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_POS_IDX, particle.getRadius());
//     particle.getDynamics().getStateObject().setState(physics::STATE_VECTOR_X_LIN_VEL_IDX,
//                       -1 * particle.getDynamics().getStateObject().getState(physics::STATE_VECTOR_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION);
//   }
// }

// void collisionCheckOtherParticles(Particle& particle_1, Particle& particle_2)
// {

//   auto& state_1 = particle_1.getDynamics().getStateObject();
//   auto& state_2 = particle_2.getDynamics().getStateObject();

//   // Get the positions
//   Eigen::Vector3d pos_1 = state_1.getState().segment<3>(0);
//   Eigen::Vector3d pos_2 = state_2.getState().segment<3>(0);

//   // Find the difference in position and normalize the difference
//   Eigen::Vector3d delta_pos = (pos_2 - pos_1);

//   // If the distance between the particles is greater than the sum of the radius, then they are not in collision
//   if (delta_pos.norm() > (particle_1.getRadius() + particle_2.getRadius()))
//   {
//     return;
//   }

//   // How much to push the particle along the collision axis
//   auto corr = (particle_1.getRadius() + particle_2.getRadius() - delta_pos.norm()) / 2.0;
//   // Normalize the vector along the collision axis
//   delta_pos.normalize();

//   // Calculate the position adjustment and add it to the state of the first particle
//   stateVector pos_adjustment_1{ 0, 0, 0, 0, 0, 0 };
//   pos_adjustment_1.head<3>() = delta_pos * -corr;
//   state_1.addState(pos_adjustment_1);

//   // Calculate the position adjustment and add it to the state of the second particle
//   stateVector pos_adjustment_2{ 0, 0, 0, 0, 0, 0 };
//   pos_adjustment_2.head<3>() = delta_pos * corr;
//   state_2.addState(pos_adjustment_2);

//   // Get the velocity along the collision axis
//   auto vel_axis_1 = (state_1.getState().segment(3, 3)).dot(delta_pos);
//   auto vel_axis_2 = (state_2.getState().segment(3, 3)).dot(delta_pos);

//   const auto m1 = state_1.getMass();
//   const auto m2 = state_2.getMass();

//   // Calculate the new velocities
//   const auto new_vel_axis_1 =
//       (m1 * vel_axis_1 + m2 * vel_axis_2 - m2 * (vel_axis_1 - vel_axis_2) * COEFFICIENT_OF_RESTITUTION) / (m1 + m2);
//   const auto new_vel_axis_2 =
//       (m1 * vel_axis_1 + m2 * vel_axis_2 - m1 * (vel_axis_2 - vel_axis_1) * COEFFICIENT_OF_RESTITUTION) / (m1 + m2);

//   // Set the new velocities
//   stateVector vel_adjustment_1{ 0, 0, 0, 0, 0, 0 };
//   vel_adjustment_1.tail<3>() = delta_pos * (new_vel_axis_1 - vel_axis_1);
//   state_1.addState(vel_adjustment_1);

//   stateVector vel_adjustment_2{ 0, 0, 0, 0, 0, 0 };
//   vel_adjustment_2.tail<3>() = delta_pos * (new_vel_axis_2 - vel_axis_2);
//   state_2.addState(vel_adjustment_2);
// }
}  // namespace physics