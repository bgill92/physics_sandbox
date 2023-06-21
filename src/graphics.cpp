#include "graphics.hpp"

void Graphics::initialize(){
	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		std::cerr << "SDL could not initialize.\n";
		std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
	}	

	// Create Window
	sdl_window = SDL_CreateWindow("Physics Sandbox", SDL_WINDOWPOS_CENTERED,
	                            SDL_WINDOWPOS_CENTERED, windowWidth_,
	                            windowHeight_, SDL_WINDOW_SHOWN);

	if (nullptr == sdl_window) {
		std::cerr << "Window could not be created.\n";
		std::cerr << " SDL_Error: " << SDL_GetError() << "\n";
	}

	// Create renderer
	sdl_renderer = SDL_CreateRenderer(sdl_window, -1, SDL_RENDERER_ACCELERATED);
	if (nullptr == sdl_renderer) {
		std::cerr << "Renderer could not be created.\n";
		std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
	}	

	// Set the background color
	SDL_SetRenderDrawColor(sdl_renderer, 0x1E, 0x1E, 0x1E, 0xFF);	

	// Clear the rendering target with the drawing color
	SDL_RenderClear(sdl_renderer);	

	// Render everything
	SDL_RenderPresent(sdl_renderer);

}

void Graphics::Draw(std::vector<Particle> const& particles) {

    // Clear screen
    SDL_SetRenderDrawColor(getRenderer(), 0x1E, 0x1E, 0x1E, 0xFF);
    SDL_RenderClear(getRenderer());

    for (auto const& obj:particles) {
    	SDL_SetRenderDrawColor(this->getRenderer(), obj.view_color()[0], obj.view_color()[1], obj.view_color()[2], 255);
    	SDL_RenderDrawPoint(this->getRenderer(), obj.view_state().position[0], obj.view_state().position[1]);
    }

    // Render screen
    SDL_RenderPresent(this->getRenderer());    

}

Graphics::~Graphics() {
	SDL_DestroyWindow(sdl_window);	
	SDL_Quit();			
}