#pragma once

#include "GameBase.h"

#include <random>

struct Particle {
	glm::vec2 position;
	glm::vec2 velocity;

	float life;

	float mass;
};

class ParticleSim : public GameBase
{
private:
	Particle* particles;
	size_t particle_count;

	glm::vec2* force_points;
	size_t force_point_count;

	bool dragging;
	size_t drag_handle;

	bool pause = false;

	bool cam_dragging;
	glm::vec2 cam_drag_offset;

public:
	ParticleSim();
	~ParticleSim();

	void Update();

	void Render();

	void OnMouseClick(int mouseButton);
	void OnMouseRelease(int mouseButton);

	void save_state(const char* path);
	void load_state(const char* path);
};
