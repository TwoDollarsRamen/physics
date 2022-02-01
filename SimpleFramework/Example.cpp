#include <stdlib.h>
#include <time.h>

#include "Example.h"

#define particle_max 1000
#define gravity -10

static float random_float(float min, float max) {
	float scale = rand() / (float)RAND_MAX;
	return min + scale * (max - min);
}

Particle new_particle() {
	Particle particle = { { 0, 0}, { 0, 0 }, random_float(0.5, 1) };

	particle.velocity.x = random_float(-5, 5);
	particle.velocity.y = random_float(-5, 5);

	return particle;
}

Example::Example() : GameBase()
{
	particle_count = particle_max;
	particles = new Particle[particle_count];
	memset(particles, 0, particle_count * sizeof(*particles));

	srand(time(0));

	for (size_t i = 0; i < particle_count; i++) {
		particles[i] = new_particle();
	}
}

Example::~Example() {
	delete[] particles;
}

void Example::Update()
{
	//This call ensures that your mouse position and aspect ratio are maintained as correct.
	GameBase::Update();

	//Your physics (or whatever) code goes here!

	while (particle_count < particle_max) {
		particles[particle_count++] = new_particle();
	}

	for (size_t i = 0; i < particle_count; i++) {
		auto& particle = particles[i];
		
		auto to_cursor = particle.position - cursorPos;
		auto inv_dist_to_cursor = 1.0f / glm::length(to_cursor);
		auto to_cursor_vec = glm::normalize(to_cursor);

		particle.velocity += (inv_dist_to_cursor * to_cursor_vec);
		particle.velocity.y += gravity * deltaTime;

		particle.position += particle.velocity * deltaTime;
		particle.life -= deltaTime;

		if (particle.life <= 0.0) {
			if (particle_count > 1) {
				particles[i] = particles[particle_count - 1];
			}
			particle_count--;
		}
	}
}

void Example::Render()
{	
	lines.DrawCircle(cursorPos, 0.2f, { 1, 1, 1});

	for (size_t i = 0; i < particle_count; i++) {
		auto& particle = particles[i];

		lines.DrawCircle(particle.position, particle.life,
			glm::vec3(glm::normalize(particle.velocity), particle.life));
	}

	//This call puts all the lines you've set up on screen - don't delete it or things won't work.
	GameBase::Render();
}

void Example::OnMouseClick(int mouseButton)
{

}