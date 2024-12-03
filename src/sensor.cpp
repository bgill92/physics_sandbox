#include "sensor.hpp"

#include "particle.hpp"
#include "rectangle.hpp"

#include <iostream>

namespace sensor
{
void Sense::operator()(OrientationSensor& sensor)
{
  // Get Object state
  const auto get_current_state_lambda = [&](auto& object) -> physics::State& { return object.getDynamics().getState(); };
  auto& state = std::visit(get_current_state_lambda, object_);

  // Generate some noise for the sensor
  const auto noise = sensor.generateNoise();

  // Set the sensor reading
  sensor.setReading(state(physics::STATE_THETA_IDX) + noise);

  // std::cout << "Setting sensor reading to: " << state(physics::STATE_THETA_IDX) << " + " << noise << "\n";
}

}  // namespace sensor