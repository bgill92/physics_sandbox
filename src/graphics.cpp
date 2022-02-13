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

void Graphics::Draw(std::vector<std::unique_ptr<Object>> *objects) {

    // Clear screen
    SDL_SetRenderDrawColor(getRenderer(), 0x1E, 0x1E, 0x1E, 0xFF);
    SDL_RenderClear(getRenderer());

    for (auto const &obj:(*objects)) {
    	DrawObject(obj);
    }

    // Render screen
    SDL_RenderPresent(this->getRenderer());    

}

void Graphics::DrawObject(const std::unique_ptr<Object>& obj) {
	// Take the object and perform the correct drawing animation on it depending on what it is
	if ( (obj->getObjType()) == ObjType::Box ) { // If the object type is box

		SDL_Rect rect;

	    // Set rectangle width and height
	    rect.w = static_cast<Box*>(obj.get())->getWidth();
	    rect.h = static_cast<Box*>(obj.get())->getHeight();

	    // Set the rectangle x and y (The y needs to be subtracted from the height of the screen to get the correct value)
	    rect.x = (obj->getPos()).x;
	    rect.y = _windowHeight - (obj->getPos()).y;

	    // Set the rectangle color
	    SDL_SetRenderDrawColor(this->getRenderer(), 0xFF, 0xFF, 0xFF, 0xFF);
	    SDL_RenderFillRect(this->getRenderer(),&rect);

	}

}

Graphics::~Graphics() {
	SDL_DestroyWindow(sdl_window);	
	SDL_Quit();			
}