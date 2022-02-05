#include "manager.h"

Manager::Manager() {
	_graphics = std::make_unique<Graphics>();
}

void Manager::initialize() {
	this->_graphics->initialize();
}

void Manager::Run(){

	// Try to get something on the screen
	SDL_Rect rect;

	int x_init = kScreenWidth/2;
	int y_init = kScreenHeight/2;

	rect.x = x_init;
	rect.y = y_init;
	rect.w = 32;
	rect.h = 32;

	bool quit = false;

	SDL_Event e;

	while (!quit) {		

		while (SDL_PollEvent(&e) != 0 ) {

			if (e.type == SDL_QUIT) {
				quit = true;
			}

		}

		Uint64 begTime = SDL_GetTicks64();


		// TODO: Do the physics and stuff	

	    // Clear screen
	    SDL_SetRenderDrawColor(this->_graphics->getRenderer(), 0x1E, 0x1E, 0x1E, 0xFF);
	    SDL_RenderClear(this->_graphics->getRenderer());

	    // Rectangle color
	    SDL_SetRenderDrawColor(this->_graphics->getRenderer(), 0xFF, 0xFF, 0xFF, 0xFF);
	    SDL_RenderFillRect(this->_graphics->getRenderer(),&rect);

	    // Render screen
	    SDL_RenderPresent(this->_graphics->getRenderer());

	    rect.y++;

	    if (rect.y+(rect.h) > kScreenHeight){
	    	rect.y = 0;
	    }

		Uint64 endTime = SDL_GetTicks64();

		if ((endTime-begTime) < kMsPerFrame) {
			SDL_Delay(kMsPerFrame - (endTime-begTime));
		}
	

	}

}