#include <iostream>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "common.hpp"
#include "control.hpp"
#include "constraints.hpp"
#include "physics.hpp"
#include "particle.hpp"
#include "simulator.hpp"
#include "sensor.hpp"
#include "utils.hpp"
#include "utils_control.hpp"

#include <SFML/Graphics.hpp>

int main()
{
  // Get config
  // Parse the config file
  const std::string config_file_path = "/home/bilal/physics_sandbox/config/config.json";

  std::ifstream config_file{ config_file_path };

  const auto config = utils::parse(utils::json::parse(config_file));

  const auto object_and_constraint_generator_func =
      [](const Config& config) -> std::pair<std::vector<Object>, std::vector<constraints::Constraint>> {
    constraints::LinearConstraint constraint_1{ 0, physics::State{ 2, 5, 0, 0, 0, 0 },
                                                physics::State{ 8, 5, 0, 0, 0, 0 } };

    const Eigen::Vector2d v1{ 0.0, 0 };

    const Eigen::Vector2d v2{ -1.4, 0 };

    constraints::DistanceConstraint constraint_2{ 0, v1, 1, v2, 0 };

    Rectangle r1{ 0.5, 0.2, 1, { 3, 5, 0, 0, 0, 0 }, sf::Color::Red };

    Rectangle r2{ 3, 0.1, 1, { 3, 6.4, 80 * deg2rad, 0, 0, 0 }, sf::Color::Green };

    std::vector<Object> objects;

    std::vector<constraints::Constraint> constraints;

    objects.push_back(r1);

    objects.push_back(r2);

    constraints.push_back(constraint_1);

    constraints.push_back(constraint_2);

    return { objects, constraints };
  };

  const auto sensor_generator_func = []() {
    std::vector<sensor::Sensor> sensors;

    sensor::OrientationSensor sensor_1{ 1, 0.0174533 };

    sensors.push_back(sensor_1);

    return sensors;
  };

  const std::string controller_config_file_path = "/home/bilal/physics_sandbox/config/config_pid.json";

  std::ifstream controller_config_file{ controller_config_file_path };

  const auto config_pid = utils::parse_PID(utils::json::parse(controller_config_file));

  control::PID pid{ config_pid };

  // auto simulator = Simulator(config, object_and_constraint_generator_func, 0, sensor_generator_func);
  auto simulator = Simulator(config, object_and_constraint_generator_func, std::nullopt, sensor_generator_func, pid);

  simulator.run();

  return 0;
}