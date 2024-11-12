#include "simulator.hpp"

#include <chrono>
#include <thread>

void Simulator::run()
{
  // Calculate the cycle time based on framerate
  const int target_cycle_time = static_cast<int>(1'000'000 / config_.framerate);

  // The simulation has now started
  this->sim_running_ = true;

  const auto run_sim = [&]() { this->physics_manager_.run(this->sim_running_); };

  // Start simulating the physics
  std::thread physics_thread(run_sim);

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

    // Clear the window
    window_.clear();

    // Get the mutex and lock it
    {
      std::scoped_lock lock{ drawing_mtx_ };

      drawer_manager_.drawConstraints();

      // Draw all the objects
      for (size_t i = 0; i < objects_.size(); i++)
      {
        drawer_manager_.drawObject(i);
      }
    }

    // Display the objects
    window_.display();

    const auto end_time = std::chrono::high_resolution_clock::now();

    // Calculate the duration in microseconds
    const auto draw_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

    if (draw_duration.count() > target_cycle_time)
    {
      std::cout << "The drawing time took greater than the target framerate\n";
    }
    else
    {
      // std::cout << "Sleeping for: " << target_cycle_time - draw_duration.count() << " microseconds\n";
      // Sleep until the next time to draw
      std::this_thread::sleep_for(std::chrono::microseconds(target_cycle_time - draw_duration.count()));
    }

    // // Output the duration
    // std::cout << "Draw Duration: " << draw_duration.count() << " microseconds\n";
  }

  // The sim isn't running anymore
  this->sim_running_ = false;

  // Simulation is done
  physics_thread.join();
}