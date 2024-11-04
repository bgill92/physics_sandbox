#pragma once

#include "common.hpp"
#include "physics.hpp"
#include "graphics.hpp"

#include <SFML/Graphics.hpp>

struct Simulator
{
	Simulator() = delete;
	Simulator(const Config& config, const std::function<std::vector<Object>(const Config&)> object_generator) : 
		config_{ config }, window_{sf::VideoMode(config.window_width, config.window_height), "Simulator"}, 
		objects_ {object_generator(config_)}, physics_manager_{config_, objects_}, drawer_manager_ {config_, objects_, window_} 
	{
		window_.setFramerateLimit(60);
	}

	void run();

private:
	Config config_;
	sf::RenderWindow window_;
	std::vector<Object> objects_;
	physics::PhysicsManager physics_manager_;
	graphics::DrawerManager drawer_manager_;
};