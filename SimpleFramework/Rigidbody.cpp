#include <stdlib.h>
#include <time.h>

#include <imgui.h>

#include "Rigidbody.h"

#define max_rigidbodies 64
#define physics_timestep 1.0f / 60.0f /* 60 FPS */
#define gravity -10.0f

static CollisionData circle_vs_circle(Rigidbody* a, Rigidbody* b) {
	assert(a && b);

	CollisionData result;

	if (glm::length(a->position - b->position)
		> a->shape.circle.radius + b->shape.circle.radius) {
		result.depth = 0.0f;
		return result;
	}

	result.position = a->position - b->position;
	result.normal = glm::normalize(result.position);
	
	result.depth = fabs(glm::length(a->position - b->position) - (a->shape.circle.radius + b->shape.circle.radius));

	result.a = a;
	result.b = b;

	return result;
}

static CollisionData aabb_vs_aabb(Rigidbody* a, Rigidbody* b) {
	assert(a && b);

	/* TODO */

	return {};
}

static CollisionData circle_vs_aabb(Rigidbody* a, Rigidbody* b) {
	glm::vec2 clamped_pos = a->position;

	auto b_min = b->position - b->shape.aabb.size * 0.5f;
	auto b_max = b->position + b->shape.aabb.size * 0.5f;
	
	if (clamped_pos.x < b_min.x) { clamped_pos.x = b_min.x; }
	if (clamped_pos.y < b_min.y) { clamped_pos.y = b_min.y; }
	if (clamped_pos.x > b_max.x) { clamped_pos.x = b_max.x; }
	if (clamped_pos.y > b_max.y) { clamped_pos.y = b_max.y; }

	CollisionData result = {};

	auto circle_to_clamped = clamped_pos - a->position;
	float dist = glm::length(circle_to_clamped);
	result.depth = a->shape.circle.radius - dist;
	result.normal = circle_to_clamped / dist;
	result.position = circle_to_clamped;

	return result;
}

static CollisionData aabb_vs_circle(Rigidbody* a, Rigidbody* b) {
	return circle_vs_aabb(b, a);
}

void Rigidbody::add_force(glm::vec2 force) {
	if (mass <= 0.0f) { return; }

	auto accel = force / mass;
	velocity += accel;
}

void Rigidbody::update(float ts) {
	position += velocity * ts;
	add_force(glm::vec2(0.0f, gravity * mass * ts));
}

RigidbodySim::RigidbodySim() : GameBase(), accum(0.0f) {
	detectors[{Rigidbody::circle, Rigidbody::circle}] = circle_vs_circle;
	detectors[{Rigidbody::aabb,   Rigidbody::aabb}]   = aabb_vs_aabb;
	detectors[{Rigidbody::circle, Rigidbody::aabb}]   = circle_vs_aabb;
	detectors[{Rigidbody::aabb,   Rigidbody::circle}] = aabb_vs_circle;

	rigidbodies = new Rigidbody[max_rigidbodies];

	auto rb1 = new_circle(1.0f);
	rb1->restitution = 0.5f;

	auto rb2 = new_circle(1.0f);
	rb2->position.y = 5.0f;
	rb2->restitution = 0.5f;

	auto rb3 = new_aabb({ 1, 1 });
	rb3->position.x = -3.0f;

	auto floor = new_aabb({ 10, 1 });
	floor->position.y = -4.0f;
	floor->mass = 0.0f;
	floor->restitution = 0.0f;
}

RigidbodySim::~RigidbodySim() {
	delete[] rigidbodies;
}

void RigidbodySim::Update() {
	accum += deltaTime;

	while (accum >= physics_timestep) {
		for (size_t i = 0; i < rigidbody_count; i++) {
			auto rb = rigidbodies + i;
			rb->update(physics_timestep);
		}

		for (size_t i = 0; i < rigidbody_count; i++) {
			auto a = rigidbodies + i;
			for (size_t j = i + 1; j < rigidbody_count; j++) {
				auto b = rigidbodies + j;

				CollisionData cd = detectors[{a->type, b->type}](a, b);
				if (cd.depth > 0.0f) {
					glm::vec2 r_vel  = b->velocity - a->velocity;

					float a_inv_mass = 1.0f / a->mass <= 0.0f ? 1.0f : a->mass;
					float b_inv_mass = 1.0f / b->mass <= 0.0f ? 1.0f : b->mass;

					/* Positionally resolve the collision, to prevent sinking
					 * when multiple objects are stacking on each other. */
					a->position -= cd.depth * (b_inv_mass / (a_inv_mass + b_inv_mass));
					b->position += cd.depth * (b_inv_mass / (a_inv_mass + b_inv_mass));

					float r = std::max(a->restitution, b->restitution);
					float j = glm::dot(-(1 + r) * (r_vel), cd.normal) / (a_inv_mass + b_inv_mass);

					glm::vec2 force = cd.normal * j;

					b->add_force(force);
					a->add_force(-force);
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
			case Rigidbody::aabb: {
				auto min = rb->position - rb->shape.aabb.size * 0.5f;
				auto max = rb->position + rb->shape.aabb.size * 0.5f;

				lines.DrawLineSegment({ min.x, min.y }, { min.x, max.y }, rb->color);
				lines.DrawLineSegment({ min.x, max.y }, { max.x, max.y }, rb->color);
				lines.DrawLineSegment({ max.x, max.y }, { max.x, min.y }, rb->color);
				lines.DrawLineSegment({ max.x, min.y }, { min.x, min.y }, rb->color);
			} break;
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

	rb->restitution = 0.5f;
	rb->mass = 1.0f;

	rb->color = { 1.0f, 1.0f, 1.0f };

	rb->position = { 0, 0 };
	rb->velocity = { 0, 0 };

	return rb;
}

Rigidbody* RigidbodySim::new_circle(float radius) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::circle;
	rb->shape.circle.radius = radius;

	return rb;
}

Rigidbody* RigidbodySim::new_aabb(glm::vec2 size) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::aabb;
	rb->shape.aabb.size = size;

	return rb;
}