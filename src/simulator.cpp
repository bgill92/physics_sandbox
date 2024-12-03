#include "simulator.hpp"

#include <chrono>
#include <thread>

#include <SFML/Window/Keyboard.hpp>

void Simulator::run()
{
  // Calculate the cycle time based on framerate
  const int target_cycle_time = static_cast<int>(1'000'000 / config_.framerate);

  // The simulation has now started
  this->sim_running_ = true;

  const auto run_sim = [&]() { this->physics_manager_.run(this->sim_running_); };

  // Start simulating the physics
  std::thread physics_thread(run_sim);

  // Start the control
  std::thread control_thread;
  if (controller_manager_.hasController())
  {
    control_thread = std::thread([&]() { this->controller_manager_.run(this->sim_running_); });
  }
  else
  {
    std::cout << "There is no controller\n";
  }

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

    // Process the input
    if (input_processor_)
    {
      std::scoped_lock lock{ input_mtx_ };
      input_processor_.value().processInput();
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
      // Sleep until the next time to draw
      std::this_thread::sleep_for(std::chrono::microseconds(target_cycle_time - draw_duration.count()));
    }
  }

  // The sim isn't running anymore
  this->sim_running_ = false;

  // Simulation is done
  physics_thread.join();

  // Controller is done
  if (controller_manager_.hasController())
  {
    control_thread.join();
  }
}