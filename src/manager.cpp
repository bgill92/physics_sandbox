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

	bool quit = false;

	SDL_Event e;

	bool lastStatePressed = false;
	bool inSameEvent = false;

	int windowHeight;

	while (!quit) {		

		SDL_GetRendererOutputSize(_graphics->getRenderer(), NULL, &windowHeight);

		while (SDL_PollEvent(&e) != 0 ) {

			if (e.type == SDL_QUIT) {
				quit = true;
			} else if ((e.type == SDL_MOUSEBUTTONDOWN) & (!lastStatePressed)) {
				lastStatePressed = true;
				int x,y;

				SDL_GetMouseState(&x, &y);

				_world->putBlock(x,windowHeight - y,4,4,1);	

				inSameEvent = true;

			} else {
				lastStatePressed = false;
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