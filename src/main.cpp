#include <SFML/Graphics.hpp>
#include <sstream>
#include <stdlib.h>

#include "utils.hpp"

#include <iostream>
#include <fstream>

#include <string>
#include <algorithm>
#include <vector>
using namespace std;

#include <Eigen/Dense>
using namespace Eigen;

static sf::Texture m_bodyTexture; 
static sf::VertexArray m_va(sf::Quads, 0);

std::ofstream simulationFile;

//Particle config
const static int INIT_PARTICLES = 40;
const static int PARTICLE_RADIUS = 8;
const static int BOUNDARY_PARTICLES = 63;

const static Vector2d G(0.f, -9.8f); // external (gravitational) forces

const static float H = 16.f;				// Kernel radius 16
const static float HSQ = H * H;				// radius^2 for optimization
const static float EPS = H; 				// Boundary epsilon
const static float MASS = 45.f;				// assume all particles have the same mass 65
const static float VISC = 5.f;			// 550

const static float REST_DENS = 0.1f;		 // rest density 0.5f
const static float STIFFNESS = 2.f;		 // const for equation of state 2000

static float DT = 0.001f;			 // integration timestep

const static float BOUND_DAMPING = -0.5f;

// rendering projection parameters
const static double VIEW_WIDTH = 800.f;
const static double VIEW_HEIGHT = 600.f;

bool update = false;
bool logInfo = false;
int counter = 0;

struct Particle
{
	Particle(float _x, float _y, bool _isBoundary) : x(_x, _y), v(0.f, 0.f), f(0.f, 0.f), rho(REST_DENS), p(0.f), isBoundary(_isBoundary) {}
	Vector2d  x, v, f;
	float rho, p;
	vector<Particle> neighbors;
	bool isBoundary;
};

//Particles list
static vector<Particle> particles;

void InitParticles(void)
{
	for (float y = EPS; y < VIEW_HEIGHT - EPS * 2.f; y += H)
		for (float x = VIEW_WIDTH / 4; x <= VIEW_WIDTH / 2; x += H)
			if (particles.size() < INIT_PARTICLES)
			{
				float jitter = static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
				particles.push_back(Particle(x + jitter, y + 300, false));
			}

	
	for(float y = EPS; y < VIEW_HEIGHT - EPS * 2.f; y += H){
		for (float x = VIEW_WIDTH / 4; x <= VIEW_WIDTH / 1.5f ; x += H)
			if (particles.size() < INIT_PARTICLES + BOUNDARY_PARTICLES)
			{
				particles.push_back(Particle(x - 100, y + 500, true));
			}
	}
}

void UpdatePositionVelocity(void)
{
    for(auto &p : particles)
    {
		if(p.isBoundary) continue;

        // explicit Euler integration
        p.v += DT*-p.f;
        p.x += DT*p.v;

        // enforce boundary conditions
        if(p.x(0)-EPS < 0.0f)
        {
            p.v(0) *= BOUND_DAMPING;
            p.x(0) = EPS;
        }
        if(p.x(0)+EPS > VIEW_WIDTH) 
        {
            p.v(0) *= BOUND_DAMPING;
            p.x(0) = VIEW_WIDTH-EPS;
        }
        if(p.x(1)-EPS < 0.0f)
        {
            p.v(1) *= BOUND_DAMPING;
            p.x(1) = EPS;
        }
        if(p.x(1)+EPS > VIEW_HEIGHT)
        {
            p.v(1) *= BOUND_DAMPING;
            p.x(1) = VIEW_HEIGHT-EPS;
        }
    }
}

void Render(sf::RenderTexture& m_target)
{
	sf::RectangleShape ground(sf::Vector2f(VIEW_WIDTH, VIEW_HEIGHT));
    ground.setFillColor(sf::Color::Black);

	sf::RenderStates rs_ground;
    m_target.draw(ground, rs_ground);

	sf::RenderStates rs;
	rs.texture = &m_bodyTexture;
	
	//Update vertex array
	m_va.resize(4 * particles.size());

	for (int i = 0; i < particles.size(); i++)
	{
		Particle p = particles[i];

		m_va[4 * i + 0].position = sf::Vector2f(p.x(0) - PARTICLE_RADIUS, p.x(1) - PARTICLE_RADIUS);
		m_va[4 * i + 1].position = sf::Vector2f(p.x(0) + PARTICLE_RADIUS, p.x(1) - PARTICLE_RADIUS);
		m_va[4 * i + 2].position = sf::Vector2f(p.x(0) + PARTICLE_RADIUS, p.x(1) + PARTICLE_RADIUS);
		m_va[4 * i + 3].position = sf::Vector2f(p.x(0) - PARTICLE_RADIUS, p.x(1) + PARTICLE_RADIUS);

		m_va[4 * i + 0].texCoords = sf::Vector2f(0, 0);
		m_va[4 * i + 1].texCoords = sf::Vector2f(512, 0);
		m_va[4 * i + 2].texCoords = sf::Vector2f(512, 512);
		m_va[4 * i + 3].texCoords = sf::Vector2f(0, 512);

		sf::Color color = sf::Color(0, 100, 255);
		if(p.isBoundary) color = sf::Color(255, 0, 0);

		m_va[4 * i + 0].color = color;
		m_va[4 * i + 1].color = color;
		m_va[4 * i + 2].color = color;
		m_va[4 * i + 3].color = color;
	}

    m_target.draw(m_va, rs);	
}

void NeighborSearch()
{
	for (int i = 0; i < particles.size(); i++)
	{
		Particle p = particles[i];
		p.neighbors.clear();
		//std::cout << "Particle " << i << std::endl;

		for (int j = 0; j < particles.size(); j++)
		{
			Particle n = particles[j];
			Vector2d neighborDistance = p.x - n.x;
			float distance = neighborDistance.norm();

			if(distance < 2*H) {
				p.neighbors.push_back(Particle(n.x(0), n.x(1), n.isBoundary));
				if(i == 14) std::cout << particles[14].neighbors.size() << std::endl;
			}
		}
	}

	std::cout << "Particle 14 neighbors size:" << to_string(particles[14].neighbors.size()) << std::endl;

	for(int i = 0; i<particles[14].neighbors.size(); i++){
		Particle n = particles[14].neighbors[i];
		std::cout << to_string(i) << ":" << to_string(n.x(0)) << std::endl;
	}
}

float KernelFunction(float distance)
{
	float q = distance/H;
	float alpha = 5/(14*M_PI*pow(H, 2));
	float t1 = max(1-q, 0.f);
	float t2 = max(2-q, 0.f);

	if(0 <= q && q < 1) {
		return alpha * (t2*t2*t2 - 4*t1*t1*t1);
	} else if(1 <= q && q < 2){
		return alpha * t2*t2*t2;
	}
	else return 0;
}

Vector2d KernelFirstDerivativeFunction(Vector2d normalizedDistance, float distance)
{
	float q = distance/H;
	Vector2d derivQ = normalizedDistance*H;
	float alpha = 5/(14*M_PI*pow(H, 2));
	float t1 = max(1-q, 0.f);
	float t2 = max(2-q, 0.f);

	if(0 <= q && q < 1) {
		return alpha * derivQ * (-3 * t2 * t2 - 12 * t1 * t1);
	} else if(1 <= q && q < 2){
		return alpha * derivQ * -3 * t2 * t2;
	}
	else return Vector2d(0.f,0.f);
}

void CalculateDensityPressure(void)
{
	for(auto &pi : particles)
    {
		if(pi.isBoundary) continue;
		
		pi.rho = 0.f;
		for(auto &pj : particles)
		{
			Vector2d rij = pj.x - pi.x;
			float dist = rij.norm();

			if(dist < 2*H) pi.rho += MASS * KernelFunction(dist);
			if(pj.isBoundary && dist < 2*H) pj.p = pi.p;
		}
		
		pi.p = max(STIFFNESS*(pi.rho/REST_DENS - 1), 0.0f);
    }
}

void CalculateForces(void)
{
    for(auto &pi : particles)
    {
		if(pi.isBoundary) continue;

        Vector2d fpress(0.f, 0.f);
        Vector2d fvisc(0.f, 0.f);

        for(auto &pj : particles)
        {
        	Vector2d rij = pj.x - pi.x;

			Vector2d xij = pi.x - pj.x;
			Vector2d vij = pi.v - pj.v;
            
			float distance = rij.norm();

            if(distance < 2*H)
            {
                // compute pressure force contribution
                fpress += -MASS * (pi.p/pow(pi.rho,2) + pj.p/pow(pj.rho,2)) * KernelFirstDerivativeFunction(rij.normalized(), distance);

                // compute viscosity force contribution (non-pressure acceleration)
                fvisc += MASS / pj.rho * ( vij.dot(xij) / (xij.dot(xij)+0.01f*H*H) ) * KernelFirstDerivativeFunction(rij.normalized(), distance);
            }
        }

		//Sum non-pressure accelerations and pressure accelerations
        pi.f = fpress + 2*VISC * fvisc + G;
    }
}

void OutputInfo(void)
{
	Particle pi = particles[0];

	simulationFile << counter << "," << pi.x(0) << "," <<  -pi.x(1) << "," << pi.rho << "," << pi.p << "\n";
	counter++;
}

void Update(void)
{
	//NeighborSearch();
	CalculateDensityPressure();
	CalculateForces();
	UpdatePositionVelocity();
	if(logInfo) OutputInfo();
}

//Keyboard inputs
void ProcessEvents(sf::RenderWindow& window)
{
	sf::Event event;

	while (window.pollEvent(event))
	{
		if (event.type == sf::Event::Closed) window.close();

		switch (event.type)
		{
		case sf::Event::KeyPressed:
			if (event.key.code == sf::Keyboard::Escape) window.close();
			else if (event.key.code == sf::Keyboard::T) std::cout << "Number of particles: " << particles.size() << std::endl;
			else if (event.key.code == sf::Keyboard::E) {
				update = !update;
				if(update) std::cout << "Simulation resumed" << std::endl;
				else std::cout << "Simulation stopped" << std::endl;
			}
			else if (event.key.code == sf::Keyboard::U && !update) {
				Update();
				std::cout << "Updated simulation step" << std::endl;
			}
			else if (event.key.code == sf::Keyboard::L) {
				logInfo = !logInfo;
				std::cout << "Logging info:" << logInfo << std::endl;
			}
			else if (event.key.code == sf::Keyboard::M) {
				DT *= 0.1f;
				std::cout << "Time step: " << DT << std::endl;
			}
						else if (event.key.code == sf::Keyboard::N) {
				DT *= 10.f;
				std::cout << "Time step: " << DT << std::endl;
			}
			else if (event.key.code == sf::Keyboard::R){
				std::cout << "Restarting Sim" << std::endl;
				particles.clear();
				InitParticles();
			} 
			break;
		default:
			break;
		}
	}
}

int main()
{
	std::cout << "Starting Sim" << std::endl;

	simulationFile.open ("simOutput.csv");
	simulationFile << "Time, PosX, PosY, Density, Pressure \n";

	sf::ContextSettings settings;

	settings.antialiasingLevel = 0;

	sf::RenderWindow window(sf::VideoMode(VIEW_WIDTH, VIEW_HEIGHT), "Fluid Sim", sf::Style::Default, settings);
	window.setVerticalSyncEnabled(true);
	window.setFramerateLimit(60);

	m_bodyTexture.loadFromFile("../res/circle.png");

	//Size of particle
	const float body_radius(4.0f);

	sf::RenderTexture render_tex;
	render_tex.create(VIEW_WIDTH, VIEW_HEIGHT);

	InitParticles();	

	sf::Clock clock;
	
	while (window.isOpen())
	{
		clock.restart();

		//Get keyboard inputs
		ProcessEvents(window);

		render_tex.clear(sf::Color::White);
		
		//Update functions
		if(update) Update();

		//Render particles
		Render(render_tex);

		render_tex.display();

		window.draw(sf::Sprite(render_tex.getTexture()));
		window.display();
	}

	simulationFile.close();

	return 0;
}