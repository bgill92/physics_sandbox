#pragma once

#include "common.hpp"

#include <random>

namespace sensor
{

// Measures the orientation of some Object with noise
struct OrientationSensor
{
  OrientationSensor() = delete;
  OrientationSensor(const size_t object_idx, const double noise_std_dev)
    : object_idx_{ object_idx }, gen_{ std::random_device()() }, noise_{ 0, noise_std_dev }
  {
  }

  size_t getObjectIdx() const
  {
    return object_idx_;
  }

  double generateNoise()
  {
    return noise_(gen_);
  }

  double getReading() const
  {
    return noisy_orientation_;
  };

  void setReading(const double reading)
  {
    noisy_orientation_ = reading;
  }

private:
  size_t object_idx_;
  std::mt19937 gen_;
  std::normal_distribution<double> noise_;
  double noisy_orientation_ = 0;
};

using Sensor = std::variant<OrientationSensor>;

struct Sense
{
  Sense() = delete;
  Sense(Object& object) : object_{ object }
  {
  }

  void operator()(OrientationSensor& sensor);

private:
  Object& object_;
};

}  // namespace sensor