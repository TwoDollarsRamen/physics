#include <stdlib.h>
#include <time.h>

#include <imgui.h>

#include "Rigidbody.h"

#define max_rigidbodies 64
#define physics_timestep 1.0f / 60.0f /* 60 FPS */
#define gravity -0.0f

static bool circle_vs_circle(Rigidbody* a, Rigidbody* b) {
	assert(a && b);
	
	return glm::length(a->position - b->position)
		<= a->shape.circle.radius + b->shape.circle.radius;
}

void Rigidbody::add_force(glm::vec2 force) {
	auto accel = force / mass;
	velocity += accel;
}

void Rigidbody::update(float ts) {
	position += velocity * ts;
	add_force(glm::vec2(0.0f, gravity * mass * ts));
}

RigidbodySim::RigidbodySim() : GameBase(), accum(0.0f) {
	detectors[{Rigidbody::circle, Rigidbody::circle}] = circle_vs_circle;

	rigidbodies = new Rigidbody[max_rigidbodies];

	auto rb1 = new_sphere(1.0f);
	rb1->position.x = -5.0f;
	rb1->velocity.x =  2.0f;

	auto rb2 = new_sphere(2.0f);
	rb2->position.x =  5.0f;
	rb2->velocity.x = -2.0f;
}

RigidbodySim::~RigidbodySim() {
	delete[] rigidbodies;
}

void RigidbodySim::Update() {
	accum += deltaTime;

	while (accum >= physics_timestep) {
		for (size_t i = 0; i < rigidbody_count; i++) {
			auto rb = rigidbodies + i;
			rb->color = { 1.0f, 1.0f, 1.0f };
			rb->update(physics_timestep);
		}

		for (size_t i = 0; i < rigidbody_count; i++) {
			auto a = rigidbodies + i;
			for (size_t j = i + 1; j < rigidbody_count; j++) {
				auto b = rigidbodies + j;

				if (detectors[{a->type, b->type}](a, b)) {
					a->color = { 1.0f, 0.0f, 0.0f };
					b->color = { 1.0f, 0.0f, 0.0f };
				}
			}
		}

		accum -= physics_timestep;
	}

	GameBase::Update();
}

void RigidbodySim::Render() {
	for (size_t i = 0; i < rigidbody_count; i++) {
		auto rb = rigidbodies + i;

		switch (rb->type) {
			case Rigidbody::circle:
				lines.DrawCircle(rb->position, rb->shape.circle.radius, rb->color);
				break;
		}
	}

	GameBase::Render();
}

void RigidbodySim::OnMouseClick(int mouseButton) {
}

void RigidbodySim::OnMouseRelease(int mouseButton) {
}

Rigidbody* RigidbodySim::new_rigidbody() {
	assert(rigidbody_count < max_rigidbodies && "Too many rigidbodies!");

	auto rb = rigidbodies + rigidbody_count++;

	rb->position = { 0, 0 };
	rb->velocity = { 0, 0 };

	return rb;
}

Rigidbody* RigidbodySim::new_sphere(float radius) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::circle;
	rb->shape.circle.radius = radius;

	return rb;
}
