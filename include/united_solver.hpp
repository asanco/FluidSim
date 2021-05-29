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

class UnitedSolver
{
public:
	UnitedSolver() = default;

	UnitedSolver(const Vec2& dimension, float body_radius, const Vec2& gravity = Vec2(0.0f, 0.0f)) :
		m_collision_solver(dimension, body_radius, m_bodies.getData(), gravity),
		m_dimension(dimension)
	{}

	void update(float dt)
	{
		m_collision_solver.update(m_bodies, m_segmets, dt);
		m_constraint_solver.update(m_constraints, m_anchors, dt);

		for (SolidSegment& s : m_segmets) {
			s.update(dt);
		}

		for (Body& b : m_bodies) {
			b.update(dt);
		}
	}

	float getGridTime() const
	{
		return m_collision_solver.grid_time;
	}

	float getCollisionTime() const
	{
		return m_collision_solver.collision_time;
	}

	float getUpdateTime() const
	{
		return m_collision_solver.total_update_time;
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

	ConstraintPtr addConstraint(BodyPtr& body1, BodyPtr& body2, float length = 0.0f, float resistance = 1000.0f)
	{
		return m_constraints.add(body1, body2, length, resistance);
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

	std::vector<Body> getBodiesCopy()
	{
		return m_bodies.getData();
	}

	const fva::SwapArray<Constraint>& getConstraints() const
	{
		return m_constraints;
	}

	const fva::SwapArray<SolidSegment>& getSegments() const
	{
		return m_segmets;
	}

	Body* getBodyAt(const up::Vec2& position)
	{
		return m_collision_solver.getBodyAt(position, m_bodies);
	}

	const Grid<GRID_CELL_SIZE>& getGrid() const
	{
		return m_collision_solver.getGrid();
	}

	void resetDebug()
	{
		m_collision_solver.reset_debug(m_bodies);
	}

	void getNeighbors(string option)
	{
		if(option == "INDEX_SORT"){
			indexSort(10, 10, 10);
		}
		else {
			cout << "No option for neighbor search";
		}
	}

	void indexSort(int edgeSize, int gridColumns, int gridRows){
		std::cout << "Starting indexSort" << std::endl;
		//std::cout << m_bodies.size() << std::endl;

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
	ConstraintSolver m_constraint_solver;

	fva::SwapArray<Body> m_bodies;
	fva::SwapArray<Constraint> m_constraints;
	fva::SwapArray<Anchor> m_anchors;
	fva::SwapArray<SolidSegment> m_segmets;

	friend CollisionSolver;
};

}