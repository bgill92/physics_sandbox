#pragma once

#include <array>
#include <atomic>
#include <concepts>
#include <mutex>
#include <optional>
#include <variant>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <SFML/Graphics.hpp>

#include "constraints.hpp"
#include "common.hpp"
#include "dynamics.hpp"
#include "integrator.hpp"
#include "input.hpp"
#include "particle.hpp"
#include "rectangle.hpp"

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
  object.getDynamics().getForce() += physics::Force({ 0, -9.81 * object.getDynamics().getMass(), 0 });
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

  void operator()(Rectangle& second_object);

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
  collisionCheckBoundaries(const Config& config, constraints::ConstraintsManager& constraints_manager,
                           const size_t object_idx)
    : config_{ config }, constraints_manager_{ constraints_manager }, object_idx_{ object_idx }
  {
    // Make boundary lines going from bottom, right, top, left
    boundaries_[0] = { { 0, 0 }, { static_cast<double>(config_.window_width) / config_.pixels_to_meters_ratio, 0 } };
    boundaries_[1] = { { static_cast<double>(config_.window_width) / config_.pixels_to_meters_ratio, 0 },
                       { static_cast<double>(config_.window_width) / config_.pixels_to_meters_ratio,
                         static_cast<double>(config_.window_height) / config_.pixels_to_meters_ratio } };
    boundaries_[2] = { { static_cast<double>(config_.window_width) / config_.pixels_to_meters_ratio,
                         static_cast<double>(config_.window_height) / config_.pixels_to_meters_ratio },
                       { 0, static_cast<double>(config_.window_height) / config_.pixels_to_meters_ratio } };
    boundaries_[3] = { { 0, static_cast<double>(config_.window_height) / config_.pixels_to_meters_ratio }, { 0, 0 } };

    x_bound_ = static_cast<double>(config_.window_width) / config_.pixels_to_meters_ratio;
    y_bound_ = static_cast<double>(config_.window_height) / config_.pixels_to_meters_ratio;

    // Make a 90 deg rotation matrix
    const Eigen::Rotation2Dd rot(std::numbers::pi / 2);

    // Calculate the normal vector at each boundary
    normals_[0] = (rot * (boundaries_[0].second - boundaries_[0].first).normalized()).normalized();
    normals_[1] = (rot * (boundaries_[1].second - boundaries_[1].first).normalized()).normalized();
    normals_[2] = (rot * (boundaries_[2].second - boundaries_[2].first).normalized()).normalized();
    normals_[3] = (rot * (boundaries_[3].second - boundaries_[3].first).normalized()).normalized();
  }

  void operator()(Particle& particle);

  void operator()(Rectangle& rectangle);

private:
  const Config& config_;
  constraints::ConstraintsManager& constraints_manager_;
  size_t object_idx_;
  std::array<Line, 4> boundaries_;
  std::array<Eigen::Vector2d, 4> normals_;
  double x_bound_;
  double y_bound_;
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
void stepObject(const size_t idx, const Config& config, constraints::ConstraintsManager& constraints_manager, T& object,
                const std::optional<std::reference_wrapper<input::InputProcessor>>& input_processor_maybe,
                std::mutex& input_mtx);

/**
 * @brief      The manager for performing the physics calculations
 */
struct PhysicsManager
{
  PhysicsManager(const Config& config, std::vector<Object>& objects, std::vector<constraints::Constraint>& constraints,
                 std::mutex& drawing_mtx, std::mutex& input_mtx)
    : config_{ config }
    , objects_{ objects }
    , constraints_manager_{ objects_, constraints }
    , drawing_mtx_{ drawing_mtx }
    , input_mtx_{ input_mtx }
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

  void attachInputProcessor(input::InputProcessor& input_processor)
  {
    input_processor_ = std::ref(input_processor);
  }

private:
  Config config_;
  std::vector<Object>& objects_;
  constraints::ConstraintsManager constraints_manager_;
  std::mutex& drawing_mtx_;
  std::mutex& input_mtx_;
  std::optional<std::reference_wrapper<input::InputProcessor>> input_processor_;
  double time_{ 0 };
};

};  // namespace physics
