#include <iostream>
#include <random>
#include "particle.hpp"

void Particle::collisionCheckWall(const double WINDOW_HEIGHT, const double WINDOW_WIDTH, const double COEFFICIENT_OF_RESTITUTION)
{
  auto& state = point_mass_.getState();

  // If a particle hits the bottom
  if ((state(physics::STATE_Y_POS_IDX) - radius_) < 0)
  {
    state(physics::STATE_Y_POS_IDX) = radius_;
    state(physics::STATE_Y_LIN_VEL_IDX) = -1 * state(physics::STATE_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION;
  }
  // If a particle hits the top
  if ((state(physics::STATE_Y_POS_IDX) + radius_) > WINDOW_HEIGHT)
  {
    state(physics::STATE_Y_POS_IDX) =  WINDOW_HEIGHT - radius_;
    state(physics::STATE_Y_LIN_VEL_IDX) = -1 * state(physics::STATE_Y_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION;
  }
  // If a particle hits the right side
  if ((radius_ + state(physics::STATE_X_POS_IDX)) > WINDOW_WIDTH)
  {
    state(physics::STATE_X_POS_IDX) = WINDOW_WIDTH - radius_;
    state(physics::STATE_X_LIN_VEL_IDX) = -1 * state(physics::STATE_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION;
  }
  // If the particle hits the left side
  if (state(physics::STATE_X_POS_IDX) < radius_)
  {
    state(physics::STATE_X_POS_IDX) = radius_;
    state(physics::STATE_X_LIN_VEL_IDX) = -1 * state(physics::STATE_X_LIN_VEL_IDX) * COEFFICIENT_OF_RESTITUTION;
  }
}

std::vector<Object> generateParticles(const size_t num_particles, const unsigned int WINDOW_HEIGHT,
                                                const unsigned int WINDOW_WIDTH)
{
  std::vector<Object> particles;

  // Random numbers generator
  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<float> radius(10.0f, 50.0f);
  std::uniform_real_distribution<float> x_vel(-50.0f, 50.0f);
  std::uniform_real_distribution<float> y_vel(-50.0f, 50.0f);

  std::uniform_int_distribution<> color(0, 255);

  for (size_t i = 0; i < num_particles; i++)
  {
    const auto radius_samp = radius(gen);

    std::uniform_real_distribution<float> x_pos(radius_samp, static_cast<float>(WINDOW_WIDTH) - radius_samp);
    std::uniform_real_distribution<float> y_pos(radius_samp, static_cast<float>(WINDOW_HEIGHT) - radius_samp);

    const auto x_vel_samp = x_vel(gen);
    const auto y_vel_samp = y_vel(gen);

    auto particle = Particle(radius_samp, radius_samp, { x_pos(gen), y_pos(gen), 0, x_vel_samp, y_vel_samp, 0 }, sf::Color(color(gen), color(gen), color(gen)));

    particles.push_back(particle);
  }

  return particles;
}