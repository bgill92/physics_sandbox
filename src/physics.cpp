#include "dynamics.hpp"
#include "particle.hpp"
#include "physics.hpp"

#include <functional>
#include <iostream>
#include <thread>
#include <type_traits>

#include <iostream>
#include <iomanip>

namespace physics
{

void collisionCheckBoundaries::operator()(Particle& particle)
{
  auto& state = particle.getDynamics().getState();

  // If a particle hits the bottom
  if ((state(physics::STATE_Y_POS_IDX) - particle.getRadius()) < 0)
  {
    state(physics::STATE_Y_POS_IDX) = particle.getRadius();
    state(physics::STATE_Y_LIN_VEL_IDX) = -1 * state(physics::STATE_Y_LIN_VEL_IDX) * config_.particle_COR;
  }
  // If a particle hits the top
  if ((state(physics::STATE_Y_POS_IDX) + particle.getRadius()) > config_.window_height / config_.pixels_to_meters_ratio)
  {
    state(physics::STATE_Y_POS_IDX) = config_.window_height / config_.pixels_to_meters_ratio - particle.getRadius();
    state(physics::STATE_Y_LIN_VEL_IDX) = -1 * state(physics::STATE_Y_LIN_VEL_IDX) * config_.particle_COR;
  }
  // If a particle hits the right side
  if ((particle.getRadius() + state(physics::STATE_X_POS_IDX)) > config_.window_width / config_.pixels_to_meters_ratio)
  {
    state(physics::STATE_X_POS_IDX) = config_.window_width / config_.pixels_to_meters_ratio - particle.getRadius();
    state(physics::STATE_X_LIN_VEL_IDX) = -1 * state(physics::STATE_X_LIN_VEL_IDX) * config_.particle_COR;
  }
  // If the particle hits the left side
  if (state(physics::STATE_X_POS_IDX) < particle.getRadius())
  {
    state(physics::STATE_X_POS_IDX) = particle.getRadius();
    state(physics::STATE_X_LIN_VEL_IDX) = -1 * state(physics::STATE_X_LIN_VEL_IDX) * config_.particle_COR;
  }
}

/**
 * @brief      Template specilization of a Particle colliding with another particle
 *
 * @param      second_object  The second object, which is a particle
 */
template <>
void collisionCheck<Particle>::operator()(Particle& second_object)
{
  auto& dynamics_1 = first_object_.getDynamics();
  auto& dynamics_2 = second_object.getDynamics();

  auto& state_1 = dynamics_1.getState();
  auto& state_2 = dynamics_2.getState();

  // Get the positions
  Eigen::Vector3d pos_1 = state_1.segment<3>(0);
  Eigen::Vector3d pos_2 = state_2.segment<3>(0);

  // Find the difference in position and normalize the difference
  Eigen::Vector3d delta_pos = (pos_2 - pos_1);

  // If the distance between the particles is greater than the sum of the radius, then they are not in collision
  if (delta_pos.norm() > (first_object_.getRadius() + second_object.getRadius()))
  {
    return;
  }

  // How much to push the particle along the collision axis
  auto corr = (first_object_.getRadius() + second_object.getRadius() - delta_pos.norm()) / 2.0;
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

template <hasDynamics T>
void stepObject(const size_t idx, const Config& config, constraints::ConstraintsManager& constraints_manager, T& object)
{
  // Should gravity be applied to the object this timestep?
  if (config.gravity_flag)
  {
    applyGravity<T>(object);
  }

  // Store the current state of the object got position based dynamics
  constraints_manager.storePreviousState(idx, object.getDynamics().getState());

  // Update the state of the object
  updateObject<T>(config.timestep_physics, object);

//   // Create a collision checker used to resolve collisions with the object and other objects
//   collisionCheck collision_checker{ config, object };

//   // Resolve collisions with other objects
//   for (size_t j = idx + 1; j < objects.size(); j++)
//   {
//     std::visit(collision_checker, objects.at(j));
//   }

//   // Create a collision checker to resolve the collisions between the object and the boundaries
//   collisionCheckBoundaries boundary_collision_checker{ config };

//   // Resolve any collisions between the object and the boundary
//   boundary_collision_checker(object);

//   // Clear the forces on the object
//   clearForces<T>(object);
}


void PhysicsManager::step(const size_t idx)
{
  // A reference to the current object
  auto& object = objects_.at(idx);

  // A lambda which is used to step the object once the proper type has been dispatched via std::visit (I think thats how it works?)
  const auto step_lambda = [&](auto&& object) { stepObject<decltype(object)>(idx, config_, constraints_manager_, object); };

  std::visit(step_lambda, object);

  time_ += this->config_.timestep_physics;
}

template<hasDynamics T>
void collide(const size_t idx, const Config& config, T& object, std::vector<Object>& objects)
{

  // Create a collision checker used to resolve collisions with the object and other objects
  collisionCheck collision_checker{ config, object };

  // Resolve collisions with other objects
  for (size_t j = idx + 1; j < objects.size(); j++)
  {
    std::visit(collision_checker, objects.at(j));
  }

  // Create a collision checker to resolve the collisions between the object and the boundaries
  collisionCheckBoundaries boundary_collision_checker{ config };

  // Resolve any collisions between the object and the boundary
  boundary_collision_checker(object);
}

void PhysicsManager::run(const std::atomic<bool>& sim_running)
{
  while (sim_running)
  {

    // Calculate the cycle time based on physics timestep
    const int target_cycle_time = static_cast<int>(1'000'000 * config_.timestep_physics);

    // Get the start time of the loop
    const auto start_time = std::chrono::high_resolution_clock::now();

    {
      // Get the mutex and lock
      std::scoped_lock lock{ mtx_ };

      // Step all the objects forward in time
      for (size_t i = 0; i < objects_.size(); i++)
      {
        // Apply forces
        // Update Object
        step(i);
      }

      // Evaluate constraints
      constraints_manager_.evaluateConstraints();

      // Evaluate collisions
      for (size_t idx = 0; idx < objects_.size(); idx++)
      {
        auto& object = objects_.at(idx);

        const auto evaluate_collisions = [&](auto&& object)
        {
          collide<decltype(object)>(idx, config_, object, objects_);
        };

        std::visit(evaluate_collisions, object);
      }
      
      // Clear forces
      const auto clear_forces_lambda = [&](auto&& object)
      {
        clearForces<decltype(object)>(object);
      };

      for (size_t i = 0; i < objects_.size(); i++)
      {
        std::visit(clear_forces_lambda, objects_.at(i));
      }      

      // Update velocity after constraints
      constraints_manager_.updateVelocityAfterConstraint(config_.timestep_physics);

    }

    // Get the end time
    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the sim duration in microseconds
    const auto sim_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    if (sim_duration.count() > target_cycle_time)
    {
      std::cout << "The simulation time took greater than the target physics timestep\n";
    }
    else
    {
      // Sleep until the next time to simulate
      std::this_thread::sleep_for(std::chrono::microseconds(target_cycle_time - sim_duration.count()));
    }
  }
}

}  // namespace physics