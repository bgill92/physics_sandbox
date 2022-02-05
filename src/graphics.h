#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <iostream>
#include <SDL.h>
#include "ini.h"

class Graphics {
	public:

		// Graphics();

		~Graphics();

		void initialize();

		auto getWindowWidth() {return _windowWidth;};

		auto getWindowHeight() {return _windowHeight;};

		auto getRenderer(){return sdl_renderer;};

	private:

		SDL_Window *sdl_window;
		SDL_Renderer *sdl_renderer;

		const std::size_t _windowWidth{kScreenWidth};
		const std::size_t _windowHeight{kScreenHeight};

};

#endif