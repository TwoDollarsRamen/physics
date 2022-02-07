#pragma once

#include "GameBase.h"

#include <map>

struct CircleShape {
	float radius;
};

struct AABBShape {
	glm::vec2 size;
};

class Rigidbody {
public:
	enum {
		circle,
		aabb
	} type;

	union {
		CircleShape circle;
	} shape;

	glm::vec2 position, velocity;
	float orientation, mass;

	glm::vec3 color;

	void add_force(glm::vec2 force);
	void update(float ts);
};

struct CollisionData {
	glm::vec2 position, normal;
	float depth;

	Rigidbody* a, * b;
};

typedef CollisionData (*DetectorFunc)(Rigidbody* a, Rigidbody* b);

class RigidbodySim : public GameBase
{
private:
	Rigidbody* rigidbodies;
	size_t rigidbody_count;

	Rigidbody* new_rigidbody();

	float accum;

	std::map<std::pair<int, int>, DetectorFunc> detectors;
public:
	RigidbodySim();
	~RigidbodySim();

	void Update();

	void Render();

	void OnMouseClick(int mouseButton);
	void OnMouseRelease(int mouseButton);

	Rigidbody* new_sphere(float radius);
};
