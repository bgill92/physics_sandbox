#include "graphics.h"

void Graphics::initialize(){

	if (SDL_Init(SDL_INIT_VIDEO) < 0) {
		std::cerr << "SDL could not initialize.\n";
		std::cerr << "SDL_Error: " << SDL_GetError() << "\n";
	}	


	// Create Window
	sdl_window = SDL_CreateWindow("Physics Sandbox", SDL_WINDOWPOS_CENTERED,
	                            SDL_WINDOWPOS_CENTERED, _windowWidth,
	                            _windowHeight, SDL_WINDOW_SHOWN);

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

Graphics::~Graphics() {
	SDL_DestroyWindow(sdl_window);	
	SDL_Quit();			
}