#include "control.hpp"

#include <thread>
#include <iostream>

#include "particle.hpp"
#include "rectangle.hpp"

namespace control
{

physics::Force PID::controlInput(std::vector<sensor::Sensor>& sensors)
{
  // TODO: Need more checks for sensors and sensor checking functionality
  // Get the sensor reading
  const auto read_sensor = [&](auto& sensor) { return sensor.getReading(); };
  double sensor_reading;
  for (const auto& sensor : sensors)
  {
    sensor_reading = std::visit(read_sensor, sensor);
  }

  auto control = physics::Force().setZero();

  static double error_running = 0.0;

  static double error_last = 0.0;

  const auto error = ref_ - sensor_reading;

  error_running += error;

  if (error_last == 0.0)
  {
    error_last = error;
  }

  // TODO: Make the force be calculated along some vector
  // Calculate P control
  const physics::Force P{ Kp_ * error, 0, 0 };

  // Calculate I control
  const physics::Force I{ Ki_ * error_running * getTimestep(), 0, 0 };

  // Calculate D control
  const physics::Force D{ Kd_ * ((error - error_last) / getTimestep()), 0, 0 };

  error_last = error;

  return P + I + D;
}

tl::expected<void, std::string> ControllerManager::precheck()
{
  // We have a controller
  if (!controller_.has_value())
  {
    return tl::unexpected("There is no controller\n");
  }

  if (sensors_.size() > 1)
  {
    return tl::unexpected("Can only handle one sensor atm\n");
  }

  // TODO: Need to check specific sensor object idx/idxs for controller

  return {};
}

void ControllerManager::sense()
{
  {
    std::scoped_lock lock{ control_mtx_ };

    // Get the sensor
    const auto get_sensor_object_idx = [&](auto& sensor) { return sensor.getObjectIdx(); };
    for (auto& sensor : sensors_)
    {
      auto sensor_object_idx = std::visit(get_sensor_object_idx, sensor);
      sensor::Sense sense{ objects_.at(sensor_object_idx) };
      std::visit(sense, sensor);
    }
  }
}

void ControllerManager::control()
{
  std::scoped_lock lock{ control_mtx_ };

  // Get the controller object idx
  const auto get_controller_object_idx = [&](auto& controller) { return controller.getObjectIdx(); };
  auto controller_object_idx = std::visit(get_controller_object_idx, controller_.value());

  // Calculate the control
  const auto calculate_control = [&](auto& controller) { return controller.controlInput(sensors_); };
  const auto force = std::visit(calculate_control, controller_.value());

  // Set the force for the object such that the PhysicsManager can apply that force
  object_forces_[controller_object_idx] = force;
}

std::optional<physics::Force> ControllerManager::getObjectForce(const size_t object_idx)
{
  std::scoped_lock lock{ control_mtx_ };
  if (object_forces_.contains(object_idx))
  {
    return object_forces_.at(object_idx);
  }
  else
  {
    return std::nullopt;
  }
}

void ControllerManager::run(const std::atomic<bool>& sim_running)
{
  const auto precheck_maybe = precheck();

  if (!precheck_maybe.has_value())
  {
    std::cout << precheck_maybe.error();
    exit(0);
  }

  while (sim_running)
  {
    // TODO: Need to change this to get timestamp instead of controller
    const auto resolve_controller = [&](auto& controller) { return controller; };
    auto controller = std::visit(resolve_controller, controller_.value());

    // Calculate the cycle time based on physics timestep
    const int target_cycle_time = static_cast<int>(1'000'000 * controller.getTimestep());

    // Get the start time of the loop
    const auto start_time = std::chrono::high_resolution_clock::now();

    sense();

    control();

    // Get the end time
    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the sim duration in microseconds
    const auto control_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    if (control_duration.count() > target_cycle_time)
    {
      std::cout << "The control time took greater than the target control timestep\n";
    }
    else
    {
      // Sleep until the next time to simulate
      std::this_thread::sleep_for(std::chrono::microseconds(target_cycle_time - control_duration.count()));
    }
  }
}
}  // namespace control