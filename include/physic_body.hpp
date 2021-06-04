#pragma once

#include "vec2.hpp"
#include <algorithm>
#include <iostream>

namespace up
{

class Body
{
public:
	Body() = default;
	Body(const Vec2& pos, float radius_)
		: m_index(0)
		, m_neighbors(0)
		, m_position(pos)
		, m_old_position(pos)
		, radius(radius_)
		, m_moving(1)
		, inertia(1.0f)
		, debug(false)
		, debug_collision(false)
		, move_acc(0.0f)
		, force_sum(0.0f, 0.0f)
		, last_force_sum(0.0f, 0.0f)
		, friction(10.0f)
	{}
	
	Body& operator=(const Body& b)
	{
		m_neighbors = b.m_neighbors;
		m_position = b.m_position;
		m_old_position = b.m_old_position;
		radius = b.radius;

		return *this;
	}

	void update(float dt)
	{
		const Vec2 v(velocity());

		m_old_position = m_position;
		m_position += v;
	}

	const int& index() const
	{
		return m_index;
	}

	const Vec2& position() const
	{
		return m_position;
	}

	const Vec2& oldPosition() const
	{
		return m_old_position;
	}

	Vec2 velocity() const
	{
		return m_position - m_old_position;
	}

	void setPosition(const Vec2& position)
	{
		m_position = position;
	}

	void setIndex(const int& index)
	{
		m_index = index;
	}

	void addNeighbor(){
		m_neighbors++;
	}

	/*void clearNeighbors(){
		m_neighbors = 0;
	}*/

	void move(const Vec2& delta)
	{
		const Vec2 d(m_moving * delta);
		m_position += d;
		move_acc += std::abs(d.x) + std::abs(d.y);
	}

	void moveOld(const Vec2& delta)
	{
		m_old_position += delta;
	}

	void stop()
	{
		m_old_position = m_position;
	}

	bool moving() const
	{
		return m_moving;
	}

	void moving(bool b)
	{
		m_moving = b;
	}

	void setVelocity(const Vec2& v)
	{
		moveOld(-1.0f*v);
	}

	Vec2 force_sum;
	Vec2 last_force_sum;

	int m_neighbors;

	float radius;
	float inertia;

	float move_acc;

	bool done;
	bool debug;
	bool debug_collision;
	uint32_t check_count;
	float friction;

private:
	int m_index;
	Vec2 m_position;
	Vec2 m_old_position;

	uint8_t m_moving;
};

}




