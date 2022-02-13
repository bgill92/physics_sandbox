#include "manager.h"

#include <iostream>

Manager::Manager() {
	_graphics = std::make_unique<Graphics>();
	_world    = std::make_unique<World>();
}

void Manager::initialize() {
	this->_graphics->initialize();
}

void Manager::Run(){

	int x_init = kScreenWidth/2;
	int y_init = kScreenHeight;

	_world->putBlock(x_init,y_init,4,4,2);	

	_world->putBlock(x_init/2,y_init,4,4,1);	

	bool quit = false;

	SDL_Event e;

	while (!quit) {		

		while (SDL_PollEvent(&e) != 0 ) {

			if (e.type == SDL_QUIT) {
				quit = true;
			}

		}

		Uint64 begTime = SDL_GetTicks();

		_graphics->Draw(_world->getObjectsPtr());
	    _world->Step();

		Uint64 endTime = SDL_GetTicks();

		if ((endTime-begTime) < kMsPerFrame) {
			SDL_Delay(kMsPerFrame - (endTime-begTime));
		}
	

	}

}