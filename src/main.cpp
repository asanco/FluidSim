#include <SFML/Graphics.hpp>
#include "solver.hpp"
#include "display_manager.hpp"
#include <sstream>
#include <stdlib.h>
#include "utils.hpp"
#include <iostream>
#include <string>

int main()
{
	std::cout << "Starting Sim" << std::endl;

	//Window setup SFML
	const uint32_t win_width = 500;
	const uint32_t win_height = 500;

	sf::ContextSettings settings;

	settings.antialiasingLevel = 4;

	sf::RenderWindow window(sf::VideoMode(win_width, win_height), "Fluid Sim", sf::Style::Default, settings);
	window.setVerticalSyncEnabled(true);
	window.setFramerateLimit(60);

	//Size of particle
	const float body_radius(4.0f);

	up::Vec2 world_dimension(512.0f, 512.0f);
	up::Solver solver(world_dimension, body_radius, { 0.0f, 980.0f });

	sf::RenderTexture render_tex;
	render_tex.create(win_width, win_height);

	DisplayManager displayManager(render_tex, window, solver);

	sf::Clock clock, spawn_timer;

	uint32_t rowSize(10);
	uint32_t row(0);
	uint32_t col(0);

	//Particle spacing
	const int xOffset = 50;
	const int yOffset = 50;

	//Created predetermined set of particles (1000) with random positions
	for(int i(0); i<100; ++i){
		int randomPosX = std::rand() % ( win_width + 1 );
		int randomPosY = std::rand() % ( win_height + 1 );

		//auto b = solver.addBody(up::Vec2(randomPosX, randomPosY));
		auto b = solver.addBody(up::Vec2(col + xOffset, row + yOffset));
		//b->setVelocity(up::Vec2(0, 0));
		col += 20;

		if(i >=rowSize - 1){
			col = 0;
			row += 20;
			rowSize += 10;
		}
	}

	//Executes neighbor search
	//Either INDEX_SORT or SPATIAL_HASHING
	//solver.getNeighbors("");
	
	while (window.isOpen())
	{
		clock.restart();

		//Get keyboard inputs
		displayManager.processEvents();

		if (displayManager.update) {
			solver.update(0.016f);
			solver.getNeighbors("");
			displayManager.setUpdate(false);
		}

		render_tex.clear(sf::Color::White);

		displayManager.draw();

		render_tex.display();

		window.draw(sf::Sprite(render_tex.getTexture()));
		window.display();
	}

	return 0;
}