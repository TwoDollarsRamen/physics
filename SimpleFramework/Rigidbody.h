#pragma once

#include "GameBase.h"

struct CircleShape {
	float radius;
};

struct AABBShape {
	glm::vec2 size;
};

struct PlaneShape {
	glm::vec2 normal;
};

class Rigidbody {
public:
	enum {
		circle,
		aabb,
		plane
	} type;

	union {
		CircleShape circle;
		AABBShape aabb;
	} shape;

	glm::vec2 position, velocity;
	float mass, restitution;

	float stat_friction, kin_friction;

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

	DetectorFunc detectors[3][3];
public:
	RigidbodySim();
	~RigidbodySim();

	void Update();

	void Render();

	void OnMouseClick(int mouseButton);
	void OnMouseRelease(int mouseButton);

	Rigidbody* new_circle(float radius);
	Rigidbody* new_aabb(glm::vec2 size);
};