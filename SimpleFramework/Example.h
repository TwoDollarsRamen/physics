#pragma once

#include "GameBase.h"

#include <random>

struct Particle {
	glm::vec2 position;
	glm::vec2 velocity;

	float life;
};

class Example : public GameBase
{
private:
	Particle* particles;
	size_t particle_count;

public:
	Example();
	~Example();

	void Update();

	void Render();

	void OnMouseClick(int mouseButton);
};