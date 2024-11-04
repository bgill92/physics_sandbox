#include "simulator.hpp"

#include <chrono>

void Simulator::run()
{

  // run the main loop
  while (window_.isOpen())
  {
    // Get the start time of the loop
    const auto start_time = std::chrono::high_resolution_clock::now();

    // handle events
    sf::Event event;
    while (window_.pollEvent(event))
    {
      if (event.type == sf::Event::Closed)
        window_.close();
    }

    // draw it

    window_.clear();

    for (size_t i = 0; i < objects_.size(); i++)
    {
      physics_manager_.step(i);

      drawer_manager_.drawObject(i);
    }

    window_.display();

    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in microseconds (or any other unit)
    const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    // Output the duration
    std::cout << "Loop time: " << duration.count() << " microseconds" << std::endl;

    // std::cout << "total energy: " << physics_manager.getTotalEnergy() << "\n";
    // physics_manager.resetTotalEnergy();
  }

}