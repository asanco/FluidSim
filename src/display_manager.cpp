#include "display_manager.hpp"
#include <iostream>

DisplayManager::DisplayManager(sf::RenderTarget& target, sf::RenderWindow& window, up::Solver& solver) :
	m_window(window),
	m_target(target),
	m_solver(solver),
	m_event_manager(window),
	m_zoom(1.0f),
	m_offsetX(0.0f),
	m_offsetY(0.0f),
	m_swarm(16),
	m_va(sf::Quads, 0),
	update(false)
{
	m_windowOffsetX = 0;
    m_windowOffsetY = 0;

    m_bodyTexture.loadFromFile("circle.png");
}

up::Vec2 DisplayManager::worldCoordToDisplayCoord(const up::Vec2& worldCoord)
{
    float worldX = worldCoord.x;
    float worldY = worldCoord.y;

    float viewCoordX = (worldX-m_offsetX)*m_zoom+m_windowOffsetX;
    float viewCoordY = (worldY-m_offsetY)*m_zoom+m_windowOffsetY;

    return up::Vec2(viewCoordX, viewCoordY);
}

up::Vec2 DisplayManager::displayCoordToWorldCoord(const up::Vec2& viewCoord)
{
    float viewCoordX = viewCoord.x;
    float viewCoordY = viewCoord.y;

    float worldCoordX = (viewCoordX-m_windowOffsetX)/m_zoom+m_offsetX;
    float worldCoordY = (viewCoordY-m_windowOffsetY)/m_zoom+m_offsetY;

    return up::Vec2(worldCoordX, worldCoordY);
}

void DisplayManager::draw()
{
	sf::Clock clock;
    // draw the world's ground as a big black square
    sf::RectangleShape ground(sf::Vector2f(m_solver.getDimension().x, m_solver.getDimension().y));
    ground.setFillColor(sf::Color::Black);

	sf::RenderStates rs_ground;
	rs_ground.transform.translate(m_windowOffsetX, m_windowOffsetY);
	rs_ground.transform.scale(m_zoom, m_zoom);
	rs_ground.transform.translate(-m_offsetX, -m_offsetY);
    m_target.draw(ground, rs_ground);

    // draw the guys
	const fva::SwapArray<up::Body>& bodies_data = m_solver.getBodies();
    int bodyCount = bodies_data.size();
	m_va.resize(4 * bodyCount);
	auto group = m_swarm.execute([&](uint32_t id, uint32_t worker_count) {updateVertexArray(bodies_data.getConstData(), id, worker_count); });
	group.waitExecutionDone();

	sf::RenderStates rs;
	rs.texture = &m_bodyTexture;
	rs.transform.translate(m_windowOffsetX, m_windowOffsetY);
	rs.transform.scale(m_zoom, m_zoom);
	rs.transform.translate(-m_offsetX, -m_offsetY);

    m_target.draw(m_va, rs);

	render_time = clock.getElapsedTime().asMicroseconds() * 0.001f;
}

void DisplayManager::updateVertexArray(const std::vector<up::Body>& bodies, uint32_t id, uint32_t step)
{
	const uint32_t size(bodies.size());

	if (!size) {
		return;
	}

	uint32_t step_size(size / step);
	uint32_t start_index(id * step_size);
	uint32_t end_index(start_index + step_size);
	if (id == step-1) {
		end_index = size;
	}

	for (uint32_t i(start_index); i<end_index; ++i) {
		const up::Body& body(bodies[i]);
		float radius = body.radius;
		const up::Vec2& position = body.position();

		m_va[4 * i + 0].position = sf::Vector2f(position.x - radius, position.y - radius);
		m_va[4 * i + 1].position = sf::Vector2f(position.x + radius, position.y - radius);
		m_va[4 * i + 2].position = sf::Vector2f(position.x + radius, position.y + radius);
		m_va[4 * i + 3].position = sf::Vector2f(position.x - radius, position.y + radius);

		m_va[4 * i + 0].texCoords = sf::Vector2f(0, 0);
		m_va[4 * i + 1].texCoords = sf::Vector2f(512, 0);
		m_va[4 * i + 2].texCoords = sf::Vector2f(512, 512);
		m_va[4 * i + 3].texCoords = sf::Vector2f(0, 512);

		const float pi = 3.1415926f;
		float t = i / 1000.0f;
		float r = sin(t);
		float g = sin(t + 0.33f * 2 * pi);
		float b = sin(t + 0.66f * 2 * pi);

		sf::Color color = sf::Color(255*r*r, 255*g*g, 255*b*b);

		m_va[4 * i + 0].color = color;
		m_va[4 * i + 1].color = color;
		m_va[4 * i + 2].color = color;
		m_va[4 * i + 3].color = color;
	}
}

void DisplayManager::processEvents()
{
	sf::Vector2i mousePosition = sf::Mouse::getPosition(m_window);

	clic = false;

	sf::Event event;
	while (m_window.pollEvent(event))
	{
		switch (event.type)
		{
		case sf::Event::KeyPressed:
			if (event.key.code == sf::Keyboard::Escape) m_window.close();
			else if ((event.key.code == sf::Keyboard::Subtract)) zoom(0.8f);
			else if ((event.key.code == sf::Keyboard::Space)) update = !update;
			else if ((event.key.code == sf::Keyboard::Add)) zoom(1.2f);
			else if ((event.key.code == sf::Keyboard::E)) emit = !emit;
			else if ((event.key.code == sf::Keyboard::D)) debug_mode = !debug_mode;
			else if ((event.key.code == sf::Keyboard::R))
			{
				m_offsetX = 0.0f;
				m_offsetY = 0.0f;
				m_zoom = 1.0f;
			}
			break;
		case sf::Event::MouseWheelMoved:
			zoom(1 + event.mouseWheel.delta * 0.2f);
			break;
		case sf::Event::MouseButtonPressed:
			if (event.mouseButton.button == sf::Mouse::Left)
			{
				m_mouse_button_pressed = true;
				m_drag_clic_position = mousePosition;
				m_clic_position = mousePosition;
			}

			break;
		case sf::Event::MouseButtonReleased:
			m_mouse_button_pressed = false;
			if (m_clic_position == mousePosition)
			{
				clic = true;
			}

			break;
		case sf::Event::MouseMoved:
			if (m_mouse_button_pressed) // in this case we are dragging
			{
				// updating displayManager offset
				const float vx = float(mousePosition.x - m_drag_clic_position.x);
				const float vy = float(mousePosition.y - m_drag_clic_position.y);
				addOffset(vx, vy);
				m_drag_clic_position = mousePosition;
			}
			break;
		default:
			break;
		}
	}
}

void DisplayManager::setUpdate(bool nUpdate)
{
	update = nUpdate;
}
