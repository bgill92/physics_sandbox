#pragma once

#include <iostream>
#include <SDL.h>
#include <memory>
#include <vector>

#include "ini.hpp"
#include "physics.hpp"

class Graphics {
	public:
		~Graphics();

		void initialize();

		auto getWindowWidth() {return windowWidth_;};

		auto getWindowHeight() {return windowHeight_;};

		auto getRenderer(){return sdl_renderer;};

		void Draw(std::vector<Particle> const& particles);

	private:

		SDL_Window *sdl_window;
		SDL_Renderer *sdl_renderer;

		const std::size_t windowWidth_{kScreenWidth};
		const std::size_t windowHeight_{kScreenHeight};

};
