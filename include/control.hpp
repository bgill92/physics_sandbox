#pragma once

#include "common.hpp"
#include "sensor.hpp"
#include "utils_control.hpp"

#include "expected.hpp"

#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace control
{
struct ControllerBase
{
  virtual physics::Force controlInput(std::vector<sensor::Sensor>& sensors) = 0;

  virtual size_t getObjectIdx() = 0;

  virtual double getTimestep() = 0;
};

struct PID : public ControllerBase
{
  PID() = delete;
  PID(const Config_PID& config)
    : object_idx_{ config.object_idx }
    , timestep_{ config.timestep }
    , ref_{ config.reference_value }
    , Kp_{ config.Kp }
    , Ki_{ config.Ki }
    , Kd_{ config.Kd }
  {
  }

  physics::Force controlInput(std::vector<sensor::Sensor>& sensors) override;

  size_t getObjectIdx() override
  {
    return object_idx_;
  }

  double getTimestep() override
  {
    return timestep_;
  };

private:
  size_t object_idx_;
  double timestep_;
  double ref_;
  double Kp_ = 0;
  double Ki_ = 0;
  double Kd_ = 0;
};

using Controller = std::variant<PID>;

struct ControllerManager
{
  ControllerManager() = delete;
  ControllerManager(const std::optional<Controller>& controller, std::mutex& control_mtx, std::vector<Object>& objects,
                    const std::function<std::vector<sensor::Sensor>()>& sensors_generator)
    : controller_{ std::move(controller) }
    , control_mtx_{ control_mtx }
    , objects_{ objects }
    , sensors_{ sensors_generator() }
  {
  }

  bool hasController()
  {
    return controller_.has_value();
  }

  tl::expected<void, std::string> precheck();

  void run(const std::atomic<bool>& sim_running);

  void sense();

  void control();

  std::optional<physics::Force> getObjectForce(const size_t object_idx);

private:
  std::optional<Controller> controller_;
  std::mutex& control_mtx_;
  std::vector<Object>& objects_;
  std::vector<sensor::Sensor> sensors_;
  std::unordered_map<size_t, physics::Force> object_forces_;
};

}  // namespace control