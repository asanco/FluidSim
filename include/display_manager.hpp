#pragma once

#include <SFML/Graphics.hpp>
#include "solver.hpp"
#include "event_manager.hpp"
#include "segment.hpp"
#include <swarm.hpp>

class DisplayManager
{
public:
    DisplayManager(sf::RenderTarget& window, sf::RenderWindow& ev_window, up::Solver& collisionManager);

    void addOffset(float x, float y) {m_offsetX-=x/m_zoom; m_offsetY-=y/m_zoom;};
    void addOffset(const up::Vec2& off) {m_offsetX-=off.x/m_zoom; m_offsetY-=off.y/m_zoom;};

    // set the absolute zoom
    void setZoom(float zoom) {m_zoom = zoom;};

    // zoom on the current view (depending on the current zoom)
    void zoom(float zoomFactor) {m_zoom *= zoomFactor;};

    // draw the current gameWorld
    void draw();

	void updateVertexArray(const std::vector<up::Body>& bodies, uint32_t id, uint32_t step);

	void processEvents();

	void setUpdate(bool nUpdate);

    // getters
    float getZoom() const {return m_zoom;};
	up::Vec2 worldCoordToDisplayCoord(const up::Vec2&);
	up::Vec2 displayCoordToWorldCoord(const up::Vec2&);
	up::Body* getBodyAt(float x, float y);

	bool clic;
	bool emit;
	bool update;
	float render_time;
	bool speed_mode;
	bool debug_mode;

	sfev::EventManager& eventManager()
	{
		return m_event_manager;
	}

	const up::Vec2& getClicPosition() const
	{
		return up::Vec2(m_clic_position.x, m_clic_position.y);
	}

private:
	up::Solver& m_solver;
	sf::RenderTarget& m_target;
	sf::RenderWindow& m_window;
    sf::Texture m_bodyTexture;

	sfev::EventManager m_event_manager;
	
	sf::VertexArray m_va;
	swrm::Swarm m_swarm;

	bool m_mouse_button_pressed;
	sf::Vector2i m_drag_clic_position, m_clic_position;

    float m_zoom, m_offsetX, m_offsetY, m_windowOffsetX, m_windowOffsetY;

	bool m_show_pressure;
};
