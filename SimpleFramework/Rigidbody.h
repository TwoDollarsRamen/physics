#pragma once

#include "GameBase.h"

struct CircleShape {
	float radius;
};

struct BoxShape {
	glm::vec4 local;
	glm::vec2 extents;
};

struct PlaneShape {
	glm::vec2 normal;
};

class RigidbodySim;

class Rigidbody {
public:
	enum {
		circle,
		box
	} type;

	union {
		CircleShape circle;
		BoxShape box;
	} shape;

	glm::vec2 position, velocity;
	float mass, restitution;

	float rotation, ang_vel, moment;

	float stat_friction, kin_friction;

	bool constrain_rot = false;

	glm::vec3 color;

	RigidbodySim* sim;

	void add_force(glm::vec2 force, glm::vec2 pos = { 0.0f, 0.0f });
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
	Rigidbody* new_box(glm::vec2 size);
};