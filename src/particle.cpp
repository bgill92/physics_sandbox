#include <random>
#include "particle.hpp"

// std::vector<Particle> generateParticles(const size_t num_particles, const unsigned int WINDOW_HEIGHT,
//                                                 const unsigned int WINDOW_WIDTH)
// {
//   std::vector<Particle> particles;

//   // Random numbers generator
//   std::random_device rd;
//   std::mt19937 gen(rd());

//   std::uniform_real_distribution<float> radius(10.0f, 50.0f);
//   std::uniform_real_distribution<float> x_vel(-50.0f, 50.0f);
//   std::uniform_real_distribution<float> y_vel(-50.0f, 50.0f);

//   std::uniform_int_distribution<> color(0, 255);

//   for (size_t i = 0; i < num_particles; i++)
//   {
//     const auto radius_samp = radius(gen);

//     std::uniform_real_distribution<float> x_pos(radius_samp, static_cast<float>(WINDOW_WIDTH) - radius_samp);
//     std::uniform_real_distribution<float> y_pos(radius_samp, static_cast<float>(WINDOW_HEIGHT) - radius_samp);

//     const auto x_vel_samp = x_vel(gen);
//     const auto y_vel_samp = y_vel(gen);

//     auto particle = Particle(radius_samp, radius_samp, { x_pos(gen), y_pos(gen), 0, x_vel_samp, y_vel_samp, 0 }, sf::Color(color(gen), color(gen), color(gen)));

//     particles.push_back(particle);
//   }

//   return particles;
// }