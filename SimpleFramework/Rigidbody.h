#pragma once

#include "GameBase.h"

#include <functional>
#include <map>

struct CircleShape {
	float radius;
};

class Rigidbody {
public:
	enum {
		circle
	} type;

	union {
		CircleShape circle;
	} shape;

	glm::vec2 position;
	glm::vec2 velocity;
	float orientation;
	float mass;

	glm::vec3 color;

	void add_force(glm::vec2 force);
	void update(float ts);
};

typedef bool (*DetectorFunc)(Rigidbody* a, Rigidbody* b);

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
