#include "particle.hpp"

#include <iostream>
#include <random>

std::vector<Object> generateParticles(const Config& config)
{
  std::vector<Object> particles;

  // Random numbers generator
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<float> radius(0.1f, 0.5f);
  std::uniform_real_distribution<float> x_vel(-2.5f, 2.5f);
  std::uniform_real_distribution<float> y_vel(-2.5f, 2.5f);

  std::uniform_int_distribution<> color(0, 255);

  for (size_t i = 0; i < config.num_particles; i++)
  {
    const auto radius_samp = radius(gen);

    std::uniform_real_distribution<float> x_pos(
        radius_samp, static_cast<float>(config.window_width) / config.pixels_to_meters_ratio - radius_samp);
    std::uniform_real_distribution<float> y_pos(
        radius_samp, static_cast<float>(config.window_height) / config.pixels_to_meters_ratio - radius_samp);

    const auto x_vel_samp = x_vel(gen);
    const auto y_vel_samp = y_vel(gen);

    auto particle = Particle(radius_samp, radius_samp, { x_pos(gen), y_pos(gen), 0, x_vel_samp, y_vel_samp, 0 },
                             sf::Color(color(gen), color(gen), color(gen)));

    particles.push_back(particle);
  }

  return particles;
}