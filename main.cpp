#include <iostream>
#include <SFML/Graphics.hpp>
#include <vector>
#include <fstream>

#define SCREEN_SIZE sf::Vector2f(1600, 900)
#define PI 3.1415f
#define RADIUS 20.0f
#define NB_ROPE 1
#define HEIGHT_GROUND 450

sf::RenderWindow window;
sf::Event sfEvent;
class Camera* mainCamera = nullptr;
sf::Vector2f worldPosition;
sf::Vector2i mousePosition;
float dt = 0.0f;
float dtScale = 5.0f;
const float gravity = 9.81f;
const float ejectPow = 10.0f;
float offsetRatio = 1.5f;

sf::Vector2f startPosition = sf::Vector2f(-100.0f, -100.0f);
sf::Vector2f offset = sf::Vector2f(50.0f, 50.0f);

void DebugLog(sf::Vector2f vector)
{
	std::cout << "X: " << vector.x << " - Y: " << vector.y << std::endl;
}
void DebugLog(float value)
{
	std::cout << "Value: " << value << std::endl;
}

float DegToRad(float angle)
{
	return angle * PI / 180.0f;
}

sf::Vector2f Scale(sf::Vector2f vector, float scale)
{
	vector.x *= scale;
	vector.y *= scale;

	return vector;
}
sf::Vector2f Scale(sf::Vector2f vector, sf::Vector2f scale)
{
	vector.x *= scale.x;
	vector.y *= scale.y;

	return vector;
}

sf::Vector2f RotateVector(sf::Vector2f vector, float angle)
{
	sf::Vector2f result;

	result.x = cosf(DegToRad(angle)) * vector.x - sinf(DegToRad(angle)) * vector.y;
	result.y = sinf(DegToRad(angle)) * vector.x + cosf(DegToRad(angle)) * vector.y;

	return result;
}

float Distance(sf::Vector2f vector)
{
	return sqrtf(vector.x * vector.x + vector.y * vector.y);
}
float DistanceSqr(sf::Vector2f vector)
{
	return vector.x * vector.x + vector.y * vector.y;
}
sf::Vector2f Normalize(sf::Vector2f vector)
{
	float length = sqrtf(vector.x * vector.x + vector.y * vector.y);
	return sf::Vector2f(vector.x / length, vector.y / length);
}

class Camera
{
	sf::View m_view;
	sf::Vector2f m_position;
	sf::Vector2f m_previousPosition;
	sf::Vector2i m_firstMousePosition;
	bool m_isPressed = false;

public:
	Camera()
	{
		m_view.setSize(SCREEN_SIZE.x, SCREEN_SIZE.y);
		m_view.setCenter(sf::Vector2f(0.0f, 0.0f));
	}
	void Update()
	{
		if (sf::Mouse::isButtonPressed(sf::Mouse::Right))
		{
			if (!m_isPressed)
			{
				m_isPressed = true;
				m_previousPosition = m_view.getCenter();
				m_firstMousePosition = mousePosition;
			}

			auto tmp = -(mousePosition - m_firstMousePosition);
			m_view.setCenter(m_previousPosition + static_cast<sf::Vector2f>(tmp));
		}
		else
		{
			m_isPressed = false;
		}
	}
	void SetViewGame()
	{
		window.setView(m_view);
	}
	void SetViewUI()
	{
		window.setView(window.getDefaultView());
	}
	sf::View* GetView()
	{
		return &m_view;
	}
};

struct Transform
{
	sf::Vector2f position = sf::Vector2f(0.0f, 0.0f);
};

struct SpringJoint
{
	struct Rigidbody* anchoredRigidbody = nullptr;
	float stiffness = 10.0f;
	float stretching = 60.0f;
	float lenght = 60.0f;

	SpringJoint() {};
	SpringJoint(struct Rigidbody* rb, float stretching, float stiffness)
	{
		anchoredRigidbody = rb;
		this->stretching = stretching;
		this->stiffness = stiffness;
	}
};

struct Rigidbody
{
	Transform* transform = nullptr;

	sf::Vector2f velocity = sf::Vector2f(0.0f, 0.0f);
	sf::Vector2f acceleration = sf::Vector2f(0.0f, 0.0f);
	sf::Vector2f forces = sf::Vector2f(0.0f, 0.0f);

	bool isKinematic = false;

	float gravityScale = 1.0f;
	float mass = 2.0f;
	float damping = 0.5f;

	sf::Vector2f GetVelocityDirection()
	{
		return Normalize(velocity);
	}
	sf::Vector2f GetGravityForce()
	{
		return mass * sf::Vector2f(0.0f, gravity * gravityScale);
	}

	sf::Vector2f CalculateForce(std::vector<SpringJoint>* springJoints)
	{
		if (transform == nullptr) return sf::Vector2f(0.0f, 0.0f);
		if (springJoints == nullptr) return sf::Vector2f(0.0f, 0.0f);

		sf::Vector2f gravityForce = mass * sf::Vector2f(0.0f, gravity * gravityScale);
		sf::Vector2f dampingForce = damping * -velocity;

		for (auto& springJoint : *springJoints)
		{
			if (springJoint.anchoredRigidbody != nullptr)
			{
				//Get translation vector
				auto tmp = (springJoint.anchoredRigidbody->transform->position - transform->position);
				float lenght = 0.0f;
				if (tmp != sf::Vector2f(0.0f, 0.0f))
				{
					//Get lenght
					lenght = Distance(tmp);

					//Normalize vector
					tmp = Normalize(tmp);
				}
				else
				{
					tmp = sf::Vector2f(0.0f, -1.0f);
				}

				//calculate springForce
				sf::Vector2f springForce = springJoint.stiffness * (lenght - springJoint.stretching) * tmp;
				forces += springForce;

				//apply inverse springForce to other rigidbody if exist
				if (!springJoint.anchoredRigidbody->isKinematic)
					springJoint.anchoredRigidbody->forces += -springForce;
			}
		}

		return gravityForce + dampingForce + forces;
	}
	sf::Vector2f CalculateForce(SpringJoint* springJoint)
	{
		if (transform == nullptr) return sf::Vector2f(0.0f, 0.0f);
		if (springJoint == nullptr) return sf::Vector2f(0.0f, 0.0f);

		sf::Vector2f gravityForce = mass * sf::Vector2f(0.0f, gravity * gravityScale);
		sf::Vector2f dampingForce = damping * -velocity;

		if (springJoint->anchoredRigidbody != nullptr)
		{
			//Get translation vector
			auto tmp = (springJoint->anchoredRigidbody->transform->position - transform->position);
			float lenght = 0.0f;
			if (tmp != sf::Vector2f(0.0f, 0.0f))
			{
				//Get lenght
				lenght = Distance(tmp);

				//Normalize vector
				tmp = Normalize(tmp);
			}
			else
			{
				tmp = sf::Vector2f(0.0f, -1.0f);
			}

			//calculate springForce
			sf::Vector2f springForce = springJoint->stiffness * (lenght - springJoint->stretching) * tmp;
			forces += springForce;

			//apply inverse springForce to other rigidbody if exist
			if (!springJoint->anchoredRigidbody->isKinematic)
				springJoint->anchoredRigidbody->forces += -springForce;
		}

		return gravityForce + dampingForce + forces;
	}

	void AddForce(sf::Vector2f force)
	{
		forces += force;
	}

	void CalculateVelocity(sf::Vector2f force)
	{
		acceleration = force / mass;
		velocity += acceleration * dt;
	}
	void ApplyVelocity(Transform* transform)
	{
		if (transform == nullptr) return;

		transform->position += velocity * dt;
	}
	void CalculateAndApplyTransferForce(Rigidbody* other)
	{
		auto totalMass = this->mass + other->mass;

		//Get velocity difference
		auto tmpVelocity = velocity - other->velocity;
		auto length = Distance(tmpVelocity);

		auto lengthInside = -(length * (1.0f - this->mass / totalMass) * 0.9f);
		auto lengthOutside = (length * (1.0f - other->mass / totalMass) * 0.9f);

		//direction * length / dt * mass
		//division by dt for get a acceleration
		//multiplication by mass for get a force
		auto direction = Normalize(tmpVelocity);
		forces = (direction * lengthInside) / dt * mass;
		other->forces = (direction * lengthOutside) / dt * mass;
	}
	void ResetForces() { forces = sf::Vector2f(0.0f, 0.0f); }
};

class Anchor
{
	Transform m_transform;
	Rigidbody m_rigidbody;

	sf::RectangleShape m_rectangleShape;
	sf::FloatRect m_bound = sf::FloatRect(0.0f, 0.0f, 40.0f, 20.0f);

	bool m_isPress = false;

	public:
		Anchor()
		{
			m_rigidbody.transform = &m_transform;
			m_rigidbody.isKinematic = true;

			m_rectangleShape.setFillColor(sf::Color::Blue);
			m_rectangleShape.setSize(sf::Vector2f(40.0f, 20.0f));
			m_rectangleShape.setOrigin(sf::Vector2f(20.0f, 10.0f));
		}

		Transform* GetTransform() { return &m_transform; }
		Rigidbody* GetRigidbody() { return &m_rigidbody; }
		sf::FloatRect GetWorldBound() { return sf::FloatRect(m_transform.position - sf::Vector2f(m_bound.width / 2.0f, m_bound.height / 2.0f), sf::Vector2f(m_bound.width, m_bound.height)); }

		bool IsPress() { return m_isPress; }
		void SetIsPress(bool value) { m_isPress = value; }

		void Draw()
		{
			m_rectangleShape.setPosition(m_transform.position);
			window.draw(m_rectangleShape);
		}
};

class Node
{
	Transform m_transform;
	Rigidbody m_rigidbody;
	SpringJoint m_springJoint;

	sf::CircleShape m_circleShape;

public:
	Transform* GetTransform() { return &m_transform; }
	Rigidbody* GetRigidbody() { return &m_rigidbody; }
	SpringJoint* GetSpringJoint() { return &m_springJoint; }
	
	Node()
	{
		m_rigidbody.transform = &m_transform;

		m_circleShape.setFillColor(sf::Color::Green);
		m_circleShape.setRadius(RADIUS);
		m_circleShape.setOrigin(sf::Vector2f(RADIUS, RADIUS));
	}
	void Update()
	{
		m_rigidbody.CalculateVelocity(m_rigidbody.CalculateForce(&m_springJoint));
		m_rigidbody.ResetForces();
		m_rigidbody.ApplyVelocity(&m_transform);
	}
	void Draw()
	{
		m_circleShape.setPosition(m_transform.position);
		window.draw(m_circleShape);
	}
};

class MultipleNode
{
	Transform m_transform;
	Rigidbody m_rigidbody;
	std::vector<SpringJoint> m_springJoints;

	sf::VertexArray m_vertexArray;
	sf::CircleShape m_circleShape;

public:
	Transform* GetTransform() { return &m_transform; }
	Rigidbody* GetRigidbody() { return &m_rigidbody; }
	SpringJoint* GetSpringJointAt(int index) { if (index < m_springJoints.size() && index > -1) return &m_springJoints[index]; }

	MultipleNode()
	{
		m_rigidbody.transform = &m_transform;

		m_circleShape.setFillColor(sf::Color::Green);
		m_circleShape.setRadius(RADIUS);
		m_circleShape.setOrigin(sf::Vector2f(RADIUS, RADIUS));

		m_vertexArray.setPrimitiveType(sf::PrimitiveType::LineStrip);
		m_vertexArray.resize(2);
		m_vertexArray[0].color = sf::Color::Yellow;
		m_vertexArray[1].color = sf::Color::Yellow;
	}
	void AddJoint(Rigidbody* anchoredRigidbody, float distance = 60.0f, float stiffness = 10.0f)
	{
		m_springJoints.push_back(SpringJoint(anchoredRigidbody, distance, stiffness));
	}
	void Update()
	{
		auto forces = m_rigidbody.CalculateForce(&m_springJoints);
		m_rigidbody.ResetForces();

		if (!m_rigidbody.isKinematic)
		{
			m_rigidbody.CalculateVelocity(forces);
			m_rigidbody.ApplyVelocity(&m_transform);
		}
	}
	void Draw()
	{
		m_vertexArray[0].position = m_transform.position;

		for (auto& joint : m_springJoints)
		{
			m_vertexArray[1].position = joint.anchoredRigidbody->transform->position;
			window.draw(m_vertexArray);
		}

		m_circleShape.setPosition(m_transform.position);
		window.draw(m_circleShape);
	}
};

class Rope
{
	Anchor m_anchor;
	std::vector<Node> m_nodes;

	sf::VertexArray m_vertexArray;

	bool m_activateCollision = false;

	void CalculateCollision(Node* node0, Node* node1)
	{
		auto tmp = node1->GetTransform()->position - node0->GetTransform()->position;
		auto lenght = DistanceSqr(tmp);
		if (lenght < 4 * RADIUS * RADIUS)
		{
			node0->GetTransform()->position = node0->GetTransform()->position + -(Normalize(tmp) * offsetRatio);
			node0->GetRigidbody()->CalculateAndApplyTransferForce(node1->GetRigidbody());
		}
	}

public:
	Rope()
	{

	}
	Rope(size_t nbNode, sf::Vector2f position)
	{
		m_anchor.GetTransform()->position = position;

		m_nodes.resize(nbNode);
		for (size_t i = 0; i < m_nodes.size(); i++)
		{
			m_nodes[i].GetTransform()->position = sf::Vector2f(position.x, position.y + offset.y * (i + 1));
			if (i == 0) m_nodes[i].GetSpringJoint()->anchoredRigidbody = m_anchor.GetRigidbody();
			if (i != 0) m_nodes[i].GetSpringJoint()->anchoredRigidbody = m_nodes[i - 1].GetRigidbody();
		}

		m_vertexArray.setPrimitiveType(sf::PrimitiveType::LineStrip);
		m_vertexArray.resize(nbNode + 1);
	}

	void Init(size_t nbNode, sf::Vector2f position)
	{
		m_anchor.GetTransform()->position = position;

		m_nodes.resize(nbNode);
		for (size_t i = 0; i < m_nodes.size(); i++)
		{
			m_nodes[i].GetTransform()->position = sf::Vector2f(position.x, position.y + offset.y * (i + 1));
			if (i == 0) m_nodes[i].GetSpringJoint()->anchoredRigidbody = m_anchor.GetRigidbody();
			if (i != 0) m_nodes[i].GetSpringJoint()->anchoredRigidbody = m_nodes[i - 1].GetRigidbody();
		}

		m_vertexArray.setPrimitiveType(sf::PrimitiveType::LineStrip);
		m_vertexArray.resize(nbNode + 1);
	}
	void InputEvent()
	{
		if (sfEvent.type == sf::Event::MouseButtonPressed && sfEvent.mouseButton.button == sf::Mouse::Button::Left)
		{
			if (m_anchor.GetWorldBound().contains(worldPosition))
			{
				m_anchor.SetIsPress(true);
			}
		}
		else if (sfEvent.type == sf::Event::MouseButtonReleased && sfEvent.mouseButton.button == sf::Mouse::Button::Left)
		{
			if (m_anchor.IsPress())
			{
				m_anchor.SetIsPress(false);
			}
		}
	}
	void Update()
	{
		if (m_anchor.IsPress())
		{
			m_anchor.GetTransform()->position = worldPosition;
		}

		for (auto& node : m_nodes)
		{
			node.Update();
		}

		if (m_activateCollision)
		{
			for (int i = 0; i < m_nodes.size() - 1; i++)
			{
				for (int j = 1 + i; j < m_nodes.size(); j++)
				{
					CalculateCollision(&m_nodes[i], &m_nodes[j]);
				}
			}
		}
	}
	void Draw()
	{
		m_vertexArray[0].position = m_anchor.GetTransform()->position;
		for (size_t i = 0; i < m_nodes.size(); i++)
		{
			m_vertexArray[i + 1].color = sf::Color::Yellow;
			m_vertexArray[i + 1].position = m_nodes[i].GetTransform()->position;
		}
		window.draw(m_vertexArray);

		m_anchor.Draw();
		
		for (auto& node : m_nodes)
		{
			node.Draw();
		}
	}
};

class Ball
{
	MultipleNode m_centerNode;
	std::vector<MultipleNode> m_nodes;

	bool m_isPress = false;

	void CalculateCollision(MultipleNode* node)
	{
		auto previousVelocity = node->GetRigidbody()->velocity;

		auto lenghtXMax = node->GetTransform()->position.x - (mainCamera->GetView()->getCenter().x + mainCamera->GetView()->getSize().x / 2.0f);
		auto lenghtXMin = node->GetTransform()->position.x - (mainCamera->GetView()->getCenter().x - mainCamera->GetView()->getSize().x / 2.0f);
		auto lenghtY = node->GetTransform()->position.y - (mainCamera->GetView()->getCenter().y + mainCamera->GetView()->getSize().y / 2.0f);

		if (lenghtXMax * lenghtXMax < RADIUS * RADIUS)
		{
			node->GetTransform()->position.x = mainCamera->GetView()->getCenter().x + mainCamera->GetView()->getSize().x / 2.0f - RADIUS;
			node->GetRigidbody()->velocity.x = -previousVelocity.x;
		}
		else if (lenghtXMin * lenghtXMin < RADIUS * RADIUS)
		{
			node->GetTransform()->position.x = mainCamera->GetView()->getCenter().x - mainCamera->GetView()->getSize().x / 2.0f + RADIUS;
			node->GetRigidbody()->velocity.x = -previousVelocity.x;
		}
		if (lenghtY * lenghtY < RADIUS * RADIUS)
		{
			node->GetTransform()->position.y = mainCamera->GetView()->getCenter().y + mainCamera->GetView()->getSize().y / 2.0f - RADIUS;
			node->GetRigidbody()->velocity.y = -previousVelocity.y;
		}
	}

public:
	Ball(int nbNode = 6, float radius = 50.0f)
	{
		m_centerNode.GetTransform()->position = sf::Vector2f(-200, -50.0f);

		m_nodes.resize(nbNode);
		for (int i = 0; i < nbNode; i++)
		{
			m_nodes[i].GetTransform()->position = m_centerNode.GetTransform()->position + RotateVector(sf::Vector2f(1.0f, 0.0f), i * (360.0f / nbNode)) * radius;
			m_centerNode.AddJoint(m_nodes[i].GetRigidbody());

			if (i != 0) m_nodes[i].AddJoint(m_nodes[i - 1].GetRigidbody());
		}
		m_nodes[0].AddJoint(m_nodes[m_nodes.size() - 1].GetRigidbody());
	}

	void InputEvent()
	{
		if (sfEvent.type == sf::Event::MouseButtonPressed && sfEvent.mouseButton.button == sf::Mouse::Button::Left)
		{
			auto lenght = DistanceSqr(worldPosition - m_centerNode.GetTransform()->position);
			if (lenght < RADIUS * RADIUS)
			{
				m_isPress = true;
				m_centerNode.GetRigidbody()->isKinematic = true;
			}
		}
		else if (sfEvent.type == sf::Event::MouseButtonReleased && sfEvent.mouseButton.button == sf::Mouse::Button::Left)
		{
			if (m_isPress)
			{
				m_isPress = false;
				m_centerNode.GetRigidbody()->isKinematic = false;
			}
		}
	}
	void Update()
	{
		if (m_isPress)
		{
			m_centerNode.GetTransform()->position = worldPosition;
		}

		m_centerNode.Update();
		CalculateCollision(&m_centerNode);
		for (auto& node : m_nodes)
		{
			node.Update();

			CalculateCollision(&node);
		}
	}
	void Draw()
	{
		m_centerNode.Draw();
		for (auto& node : m_nodes) node.Draw();
	}
};

int main(int argc, char* argv[])
{
	window.create(sf::VideoMode(static_cast<unsigned int>(SCREEN_SIZE.x), static_cast<unsigned int>(SCREEN_SIZE.y)), "MasseRessort");
	window.setVerticalSyncEnabled(true);

	sf::Clock clock;
	mainCamera = &Camera();

	//Ropes
	std::vector<Rope> ropes;
	ropes.resize(NB_ROPE);

	int i = 0;
	for (auto& rope : ropes)
	{
		rope.Init(5, startPosition + Scale(offset, sf::Vector2f((float)i, 1)));
		i++;
	}
	//

	//Balls
	Ball ball;
	//

	while (window.isOpen())
	{
		dt = clock.restart().asSeconds();
		dt = dt * dtScale;
		mousePosition = sf::Mouse::getPosition(window);
		worldPosition = window.mapPixelToCoords(mousePosition, *mainCamera->GetView());

		while (window.pollEvent(sfEvent))
		{
			if (sfEvent.type == sf::Event::Closed) window.close();

			for (auto& rope : ropes)
			{
				rope.InputEvent();
			}

			ball.InputEvent();
		}		

		for (auto& rope : ropes)
		{
			rope.Update();
		}

		for (int i = 0; i < 2; i++)
		{
			ball.Update();
		}

		mainCamera->Update();

		window.clear();

		mainCamera->SetViewGame();

		for (auto& rope : ropes)
		{
			rope.Draw();
		}

		ball.Draw();

		mainCamera->SetViewUI();

		window.display();
	}

	return 0;
}