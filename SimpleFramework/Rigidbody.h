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

class RigidbodySim;

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

	RigidbodySim* sim;

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
	float gravity;
	size_t collision_iterations;

	DetectorFunc detectors[3][3];

	char* save_path = nullptr;

	friend class Rigidbody;
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