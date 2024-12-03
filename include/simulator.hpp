#pragma once

#include "common.hpp"
#include "constraints.hpp"
#include "control.hpp"
#include "graphics.hpp"
#include "input.hpp"
#include "physics.hpp"
#include "sensor.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <optional>

#include <SFML/Graphics.hpp>

struct Simulator
{
  Simulator() = delete;
  Simulator(
      const Config& config,
      const std::function<std::pair<std::vector<Object>, std::vector<constraints::Constraint>>(const Config&)>&
          object_and_constraints_generator,
      const std::optional<size_t>& movable_object = std::nullopt,
      const std::function<std::vector<sensor::Sensor>()>& sensors_generator = []() -> std::vector<sensor::Sensor> {
        return {};
      },
      const std::optional<control::Controller> controller = std::nullopt)
    : drawing_mtx_{}
    , input_mtx_{}
    , config_{ config }
    , window_{ sf::VideoMode(config.window_width, config.window_height), "Simulator" }
    , objects_{ object_and_constraints_generator(config_).first }
    , constraints_{ object_and_constraints_generator(config_).second }
    , controller_manager_{ std::move(controller), input_mtx_, objects_, sensors_generator }
    , physics_manager_{ config_, objects_, constraints_, drawing_mtx_, input_mtx_, controller_manager_ }
    , drawer_manager_{ config_, objects_, constraints_, window_ }
  {
    if (movable_object.has_value())
    {
      input_processor_.emplace(objects_, movable_object.value());
      physics_manager_.attachInputProcessor(input_processor_.value());
    }
  }

  void run();

private:
  std::mutex drawing_mtx_;
  std::mutex input_mtx_;
  Config config_;
  sf::RenderWindow window_;
  std::vector<Object> objects_;
  std::vector<constraints::Constraint> constraints_;
  control::ControllerManager controller_manager_;
  physics::PhysicsManager physics_manager_;
  graphics::DrawerManager drawer_manager_;
  std::optional<input::InputProcessor> input_processor_;
  std::atomic<bool> sim_running_{ false };
};