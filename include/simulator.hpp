#pragma once

#include "common.hpp"
#include "constraints.hpp"
#include "physics.hpp"
#include "graphics.hpp"

#include <atomic>
#include <functional>
#include <mutex>

#include <SFML/Graphics.hpp>

struct Simulator
{
  Simulator() = delete;
  Simulator(const Config& config,
            const std::function<std::pair<std::vector<Object>, std::vector<constraints::Constraint>>(const Config&)>
                object_and_constraints_generator)
    : drawing_mtx_{}
    , config_{ config }
    , window_{ sf::VideoMode(config.window_width, config.window_height), "Simulator" }
    , objects_{ object_and_constraints_generator(config_).first }
    , constraints_{ object_and_constraints_generator(config_).second }
    , physics_manager_{ config_, objects_, constraints_, drawing_mtx_ }
    , drawer_manager_{ config_, objects_, constraints_, window_ }
  {
  }

  void run();

private:
  std::mutex drawing_mtx_;
  Config config_;
  sf::RenderWindow window_;
  std::vector<Object> objects_;
  std::vector<constraints::Constraint> constraints_;
  physics::PhysicsManager physics_manager_;
  graphics::DrawerManager drawer_manager_;
  std::atomic<bool> sim_running_{ false };
};