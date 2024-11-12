#pragma once

#include <atomic>
#include <concepts>
#include <mutex>
#include <variant>

#include <Eigen/Dense>
#include <SFML/Graphics.hpp>

#include "constraints.hpp"
#include "common.hpp"
#include "dynamics.hpp"
#include "integrator.hpp"
#include "particle.hpp"

namespace physics
{

/**
 * @brief      Applies gravity to the force vector of an object
 *
 * @param      object  The object
 *
 * @tparam     T       A class which satisfies the hasDynamics concept
 */
template <hasDynamics T>
void applyGravity(T& object)
{
  object.getDynamics().getForce() = physics::Force({ 0, -9.81 * object.getDynamics().getMass(), 0 });
}

/**
 * @brief      Updates the state of the function based on current velocity and applied force using an RK4 integrator
 *
 * @param[in]  timestep  The timestep to move the object forward in time
 * @param      object    The object
 *
 * @tparam     T         A class which satisfies the hasDynamics concept
 */
template <hasDynamics T>
void updateObject(const double timestep, T& object)
{
  const auto derivative_func = [&object](const physics::State& derivative_at_state,
                                         const double timestep) -> physics::State {
    return object.getDynamics().stateDerivative(derivative_at_state, timestep);
  };

  object.getDynamics().getState() +=
      integrator::RK4<physics::State>(object.getDynamics().getState(), timestep, derivative_func);
}

/**
 * @brief      A struct which is used to resolve a collision check for an object. This struct is expected to be used
 *             as the first argument of a std::visit function call. The expectation is that there is a specialization
 *             of the template for each type of Object possible, and the Object it will be colliding with.
 *
 * @tparam     T     A class which satisfies the hasDynamics concept
 */
template <hasDynamics T>
struct collisionCheck
{
  collisionCheck() = delete;
  collisionCheck(const Config& config, T& first_object) : config_{ config }, first_object_{ first_object }
  {
  }

  /**
   * @brief      Calculates the physics for a particle hittiing the boundaries of the environment
   *
   * @param      particle  The particle
   */
  void operator()(Particle& second_object);

private:
  const Config& config_;
  T& first_object_;
};

/**
 * @brief      Clears the force vector of an Object
 *
 * @param      object  The object
 *
 * @tparam     T       A class which satisfies the hasDynamics concept
 */
template <hasDynamics T>
void clearForces(T& object)
{
  object.getDynamics().clearForce();
}

/**
 * @brief      Collision checks an object against the boundaries of the environment
 */
struct collisionCheckBoundaries
{
  collisionCheckBoundaries() = delete;
  collisionCheckBoundaries(const Config& config) : config_{ config }
  {
  }

  void operator()(Particle& particle);

private:
  const Config& config_;
};

/**
 * @brief      The overall function which steps the object forward in time
 *
 * @param[in]  idx      The index of the object (used for collision resolution)
 * @param[in]  config   The simulation configuration
 * @param      objects  The vector of Objects to collision check against
 * @param      object   The object to step
 *
 * @tparam     T        A class which satisfies the hasDynamics concept
 */
template <hasDynamics T>
void stepObject(const size_t idx, const Config& config, constraints::ConstraintsManager& constraints_manager, T& object);

/**
 * @brief      The manager for performing the physics calculations
 */
struct PhysicsManager
{
  PhysicsManager(const Config& config, std::vector<Object>& objects, std::vector<constraints::Constraint>& constraints,
                 std::mutex& mtx)
    : config_{ config }, objects_{ objects }, constraints_manager_{ objects_, constraints }, mtx_{ mtx }
  {
  }

  /**
   * @brief      Step an object through time
   *
   * @param[in]  idx   Which object to step
   */
  void step(const size_t idx);

  /**
   * @brief      Run the physics
   */
  void run(const std::atomic<bool>& sim_running);

private:
  Config config_;
  std::vector<Object>& objects_;
  constraints::ConstraintsManager constraints_manager_;
  std::mutex& mtx_;
  double time_{ 0 };
};

};  // namespace physics
