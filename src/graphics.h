#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <iostream>
#include <SDL.h>
#include <memory>
#include <vector>

#include "ini.h"
#include "physics.h"

class Graphics {
	public:

		// Graphics();

		~Graphics();

		void initialize();

		auto getWindowWidth() {return _windowWidth;};

		auto getWindowHeight() {return _windowHeight;};

		auto getRenderer(){return sdl_renderer;};

		void Draw(std::vector<std::unique_ptr<Object>> *objects);

		void DrawObject(const std::unique_ptr<Object> &obj);

	private:

		SDL_Window *sdl_window;
		SDL_Renderer *sdl_renderer;

		const std::size_t _windowWidth{kScreenWidth};
		const std::size_t _windowHeight{kScreenHeight};

};

#endif