#include "manager.hpp"
#include <iostream>
#include <random>

Manager::Manager() {
	graphics_ = std::make_unique<Graphics>();
	world_    = std::make_unique<World>();
}

void Manager::initialize() {
	this->graphics_->initialize();
}

void Manager::spawn_random_particles(size_t num_particles) {
    static std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> x_pos(0, kScreenWidth-1);
    std::uniform_int_distribution<> y_pos(0, kScreenHeight-1);

    std::uniform_real_distribution<> x_vel(-100.0, 100.0);
    std::uniform_real_distribution<> y_vel(-100.0, 100.0);

    std::array<uint, 3> colors {255, 255, 255};

    size_t idx = 3;

    for (size_t i = 0; i < num_particles; i++)
    {
    	if (colors[idx] > 0) {
    		colors[idx] -= 1;
    	}
    	else
    	{
    		idx--;
    	}
    	std::cout << colors[0] << ", " << colors[1] << ", " << colors[2] << "\n";
		world_->spawn_particle(i, {x_pos(gen), y_pos(gen), 0}, {x_vel(gen), y_vel(gen), 0.0}, colors);
    }

}

void Manager::run(){

	bool quit = false;

	SDL_Event e;

	bool lastStatePressed = false;
	bool inSameEvent = false;

	int windowHeight;

	world_->spawn_particle(0, {0, 0, 0}, {100.0, 100.0, 0.0}, {255, 0, 0});

	world_->spawn_particle(1, {0, graphics_->getWindowHeight()-1, 0}, {100.0, -100.0, 0.0}, {0, 255, 0});

	// spawn_random_particles(500);

	while (!quit) {		

		SDL_GetRendererOutputSize(graphics_->getRenderer(), NULL, &windowHeight);

		while (SDL_PollEvent(&e) != 0 ) {

			if (e.type == SDL_QUIT) {
				quit = true;
			}

		}

		Uint64 begTime = SDL_GetTicks();

		graphics_->Draw(world_->get_particles());
	    world_->step();

		Uint64 endTime = SDL_GetTicks();

		if ((endTime-begTime) < kMsPerFrame) {
			SDL_Delay(kMsPerFrame - (endTime-begTime));
		}


	}

}