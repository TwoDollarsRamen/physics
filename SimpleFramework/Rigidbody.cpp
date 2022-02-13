#include <stdlib.h>
#include <time.h>

#include <imgui.h>

#include "Rigidbody.h"

#define max_rigidbodies 1024
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

	glm::vec2 normal = a->position - b->position;

	CollisionData result = {};

	auto a_min = a->position - a->shape.aabb.size * 0.5f;
	auto a_max = a->position + a->shape.aabb.size * 0.5f;
	auto b_min = b->position - b->shape.aabb.size * 0.5f;
	auto b_max = b->position + b->shape.aabb.size * 0.5f;

	float a_ext = (a_max.x - a_min.x) / 2.0f;
	float b_ext = (b_max.x - b_min.x) / 2.0f;

	float x_overlap = a_ext + b_ext - fabs(normal.x);

	if (x_overlap > 0.0f) {
		a_ext = (a_max.y - a_min.y) / 2.0f;
		b_ext = (b_max.y - b_min.y) / 2.0f;

		float y_overlap = a_ext + b_ext - fabs(normal.y);

		if (y_overlap > 0.0f) {
			if (x_overlap < y_overlap) {
				if (normal.x < 0.0f) {
					result.normal = glm::vec2(-1.0f, 0.0f);
				} else {	
					result.normal = glm::vec2(1.0f, 0.0f);
				}
				result.depth = x_overlap;
			} else {
				if (normal.y < 0.0f) {
					result.normal = glm::vec2(0.0f, -1.0f);
				} else {
					result.normal = glm::vec2(0.0f, 1.0f);
				}
				result.depth = y_overlap;
			}
		}
	}

	result.a = a;
	result.b = b;

	return result;
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

	auto to_circle_clamped = -glm::abs(clamped_pos - a->position);
	float dist = glm::length(to_circle_clamped);
	result.depth = a->shape.circle.radius - dist;
	result.normal = to_circle_clamped / dist;
	result.position = to_circle_clamped;

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
	detectors[Rigidbody::circle][Rigidbody::circle] = circle_vs_circle;
	detectors[Rigidbody::aabb]  [Rigidbody::aabb]     = aabb_vs_aabb;
	detectors[Rigidbody::circle][Rigidbody::aabb]   = circle_vs_aabb;
	detectors[Rigidbody::aabb]  [Rigidbody::circle]   = aabb_vs_circle;

	rigidbodies = new Rigidbody[max_rigidbodies];

	auto floor = new_aabb({ 20, 1 });
	floor->position.y = -4.0f;
	floor->mass = 0.0f;
	floor->restitution = 0.0f;

	/*auto left_side = new_aabb({1, 20});
	left_side->position.x = -4.0f;
	left_side->mass = 0.0f;
	left_side->restitution = 0.0f;

	auto right_side= new_aabb({ 1, 20 });
	right_side->position.x = 4.0f;
	right_side->mass = 0.0f;
	right_side->restitution = 0.0f;*/
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

				CollisionData cd = detectors[a->type][b->type](a, b);
				if (cd.depth > 0.0f) {
					glm::vec2 r_vel = b->velocity - a->velocity;

					float a_inv_mass = 1.0f / a->mass <= 0.0f ? 1.0f : a->mass;
					float b_inv_mass = 1.0f / b->mass <= 0.0f ? 1.0f : b->mass;

					/* Positionally resolve the collision, to prevent sinking
					 * when multiple objects are stacked on each other. */
					glm::vec2 correction = cd.normal * (cd.depth / (a_inv_mass + b_inv_mass));
					a->position += a_inv_mass * correction;
					b->position -= b_inv_mass * correction;

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

	GameBase::Clear();
	GameBase::Render();
}

void RigidbodySim::OnMouseClick(int mouseButton) {
	if (mouseButton == GLFW_MOUSE_BUTTON_LEFT) {
		auto rb = new_circle(0.5f);
		rb->position = cursorPos;
		rb->restitution = 0.5f;
	} else if (mouseButton == GLFW_MOUSE_BUTTON_RIGHT) {
		auto rb = new_aabb({ 1.0f, 1.0f });
		rb->position = cursorPos;
		rb->restitution = 0.5f;
	}
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