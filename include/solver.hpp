#pragma once

#include <SFML/System/Clock.hpp>
#include <iostream>

#include "collision_solver.hpp"
#include "constraint_solver.hpp"
#include "segment.hpp"

#include <string>
#include <math.h>

using namespace std;

namespace up
{

using BodyPtr = fva::Handle<Body>;

class Solver
{
public:
	Solver() = default;

	Solver(const Vec2& dimension, float body_radius, const Vec2& gravity = Vec2(0.0f, 0.0f)) :
		m_collision_solver(dimension, body_radius, m_bodies.getData(), gravity),
		m_dimension(dimension)
	{}

	void update(float dt)
	{
		m_collision_solver.update(m_bodies, m_segmets, dt);

		for (Body& b : m_bodies) {
			b.update(dt);
		}
	}

	BodyPtr addBody(const Vec2& position)
	{
		return m_bodies.add(position, m_collision_solver.defaultBodyRadius());
	}

	BodyPtr addBody(const Vec2& position, float radius)
	{
		float r(std::min(radius, m_collision_solver.defaultBodyRadius()));
		return m_bodies.add(position, r);
	}

	SolidSegmentPtr addSolidSegment(BodyPtr& body1, BodyPtr& body2)
	{
		return m_segmets.add(body1, body2);
	}

	const Vec2& getDimension() const
	{
		return m_dimension;
	}

	const fva::SwapArray<Body>& getBodies() const
	{
		return m_bodies;
	}

	std::vector<Body>& getBodiesData()
	{
		return m_bodies.getData();
	}

	Body* getBodyAt(const up::Vec2& position)
	{
		return m_collision_solver.getBodyAt(position, m_bodies);
	}

	const Grid<GRID_CELL_SIZE>& getGrid() const
	{
		return m_collision_solver.getGrid();
	}

	void getNeighbors(string option)
	{
		if(option == ""){
			basicNeighborSearch();
		}
		else if(option == "INDEX_SORT"){
			indexSort(10, 10, 10);
		}
		else if(option == "SPATIAL_HASHING"){
			//spatialHashing();
		}
		else {
			cout << "No option for neighbor search";
		}
	}

	//Naive neighbor search algorithm
	void basicNeighborSearch(){
		const float kernel_support = 40;

		for(int i = 0; i < m_bodies.size();i++){
			Body currentParticle = m_bodies[i];
			Vec2 currentParticlePos = currentParticle.position();
			for(int j = 0; j < m_bodies.size();j++){
				Body potentialNeighbor = m_bodies[j];
				Vec2 potentialNeighborPos = potentialNeighbor.position();
				if(getDistance(currentParticlePos, potentialNeighborPos)<=kernel_support){
					currentParticle.addNeighbor();
				}
			}
			//if(i%10 == 0) std:: cout << std::endl;
			//std:: cout << currentParticle.m_neighbors << " ";
			
		}
	}

	//Get euclidean distance between two particles
	float getDistance(Vec2 object1, Vec2 object2){
		float a = object1.x - object2.x;
		float b = object1.y - object2.y;
		float distance = sqrt(a * a + b * b); 
		return distance;
	}

	void indexSort(int edgeSize, int gridColumns, int gridRows){
		//Create indices array
		int gridIndices[(gridColumns * gridRows) + 1] = {};
		for(int i = 0; i <= gridRows * gridColumns; i++){
			gridIndices[i] = i;
		}

		//Assign index to each particle
		int particleIndices[m_bodies.size()] = {};
		for(int i = 0; i < m_bodies.size();i++){
			Body currentParticle = m_bodies[i];
			int posX = currentParticle.position().x;
			int posY = currentParticle.position().y;
			int particleIndex = floor(posX/edgeSize) + floor(posY/edgeSize)*gridColumns;

			currentParticle.setIndex(particleIndex);
			particleIndices[i] = particleIndex;
		}

		//sort(particleIndices, 10);
		for (int i = 0; i < sizeof(particleIndices); ++i)
        std:: cout << particleIndices[i] << std::endl;

	}

private:
	const Vec2 m_dimension;
	CollisionSolver m_collision_solver;

	fva::SwapArray<Body> m_bodies;
	fva::SwapArray<SolidSegment> m_segmets;

	friend CollisionSolver;
};

}