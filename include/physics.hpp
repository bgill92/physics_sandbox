#pragma once

#include <concepts>
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

// This concept constrains the class to have a function getDynamics that returns a dynamics::DynamicsBase object
template <typename T>
concept hasDynamics = requires(T a)
{
  std::derived_from<decltype(a.getDynamics()), dynamics::DynamicsBase>;
};

// This concept constrains the class to have a function getConstraint that returns a reference to a std::optional<physics::Constraint>
template <typename T>
concept hasConstraint = requires(T a)
{
  {
    a.getConstraint()
    } -> std::same_as<std::optional<physics::Constraint>&>;
};

// This template constrains the class to satisfy both hasDynamics and hasConstraint
template <typename T>
concept hasDynamicsAndConstraint = hasDynamics<T> && hasConstraint<T>;

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
 * @brief      Evaluates a constraint that an object might contain. Currently only evaluates a CircleConstraint
 *
 * @param      object  The object
 *
 * @tparam     T       A class which satisfies the hasDynamicsAndConstraint concept
 */
template <hasDynamicsAndConstraint T>
void evaluateConstraint(T& object)
{
  if (!object.getConstraint().has_value())
  {
    return;
  }

  if (CircleConstraint* constraint = std::get_if<CircleConstraint>(&object.getConstraint().value()))
  {
    // Get the state
    physics::State& state = object.getDynamics().getState();

    // Get the difference in position between the current state and the
    Eigen::Vector3d difference_pos = (state - constraint->getCenterOfConstraint()).head<3>();

    // Normalize the distance to the center
    difference_pos.normalize();

    //
    state.head<3>() = (difference_pos * constraint->getRadius()) + constraint->getCenterOfConstraint().head<3>();
  }
  else
  {
    std::cout << "Non-supported constraint, exiting...";
    exit(0);
  }
}

/**
 * @brief      Performs a velocity correction to the object after a constraint has been satisfied. Necessary for
 * particle based dynamics.
 *
 * @param[in]  timestep        The timestep over which the object was subject to the constraint
 * @param      object          The object
 * @param[in]  previous_state  The previous state of the object
 *
 * @tparam     T               A class which satisfies the hasDynamicsAndConstraint concept
 */
template <hasDynamicsAndConstraint T>
void updateVelocityAfterConstraint(const double timestep, T& object, const physics::State& previous_state)
{
  if (!object.getConstraint().has_value())
  {
    return;
  }

  physics::State& state = object.getDynamics().getState();

  Eigen::Vector3d diff_in_pos = (state.head<3>() - previous_state.head<3>());

  state.tail<3>() = diff_in_pos / timestep;
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
void stepObject(const size_t idx, const Config& config, std::vector<Object>& objects, T& object);

/**
 * @brief      The manager for performing the physics calculations
 */
struct PhysicsManager
{
  PhysicsManager(const Config& config, std::vector<Object>& objects) : config_{ config }, objects_{ objects }
  {
  }

  void step(const size_t idx);

  // void resetTotalEnergy() {total_energy_ = 0;}

  // double getTotalEnergy() const {return total_energy_;}

private:
  Config config_;
  std::vector<Object>& objects_;
  double time_{ 0 };
  // double total_energy_ {0};
};

};  // namespace physics
