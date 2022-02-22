#include <stdlib.h>
#include <time.h>

#include <cmath>

#include <imgui.h>

#include "Rigidbody.h"

#define max_rigidbodies 1024
#define physics_timestep 1.0f / 60.0f /* 60 FPS */
#define default_gravity -10.0f
#define default_collision_iterations 8

#define TEST_MODE

static CollisionData circle_vs_circle(Rigidbody* b, Rigidbody* a) {
	CollisionData result;

	glm::vec2 b_to_a = a->position - b->position;
	float b_to_a_l = glm::length(b_to_a);

	if (b_to_a_l > a->shape.circle.radius + b->shape.circle.radius) {
		result.depth = 0.0f;
		return result;
	}

	result.normal = b_to_a / b_to_a_l;
	result.depth = fabs(b_to_a_l - (a->shape.circle.radius + b->shape.circle.radius));

	result.position = a->position - result.normal * (a->shape.circle.radius - (result.depth / 2.0f));

	result.a = a;
	result.b = b;

	return result;
}

LineRenderer* g_lines;

static bool check_box_corners(Rigidbody* a, Rigidbody* b, glm::vec2& position, int& contacts, float& depth, glm::vec2& normal) {
	glm::vec2 min(INFINITY, INFINITY), max(-INFINITY, -INFINITY);

	glm::vec2 box_size = b->shape.box.extents * 2.0f;
	int contact_count = 0;
	glm::vec2 contact(0, 0);

	for (float x = -b->shape.box.extents.x; x < box_size.x; x += box_size.x) {
		for (float y = -b->shape.box.extents.y; y < box_size.y; y += box_size.y) {
			glm::vec2 world_pos = b->position + x * glm::vec2(a->shape.box.local.x, a->shape.box.local.y)
				+ y * glm::vec2(a->shape.box.local.z, a->shape.box.local.w);
			glm::vec2 box_pos(glm::dot(world_pos - a->position, glm::vec2(b->shape.box.local.x, b->shape.box.local.y)),
				glm::dot(world_pos - a->position, glm::vec2(b->shape.box.local.z, b->shape.box.local.w)));

			g_lines->DrawCircle(world_pos, 0.1f);

			if (box_pos.x < min.x) { min.x = box_pos.x; }
			if (box_pos.y < min.y) { min.y = box_pos.y; }
			if (box_pos.x > max.x) { max.x = box_pos.x; }
			if (box_pos.y > max.y) { max.y = box_pos.y; }

			if (box_pos.x >= -a->shape.box.extents.x && box_pos.x <= a->shape.box.extents.x &&
				box_pos.y >= -a->shape.box.extents.y && box_pos.y <= a->shape.box.extents.y) {
				contact_count++;
				contact += box_pos;
			}
		}
	}

	if (contact_count == 0 ||
		max.x <= -a->shape.box.extents.x || min.x >= a->shape.box.extents.x ||
		max.y <= -a->shape.box.extents.y || min.y >= a->shape.box.extents.y) {
		return false;
	}

	position += a->position + (contact.x * glm::vec2(a->shape.box.local.x, a->shape.box.local.y) +
		contact.y * glm::vec2(a->shape.box.local.z, a->shape.box.local.w)) / (float)contact_count;
	contacts++;

	bool r = false;

	float pen = a->shape.box.extents.x - min.x;
	if (pen > 0.0f && (pen < depth || depth == 0)) {
		normal = glm::vec2(b->shape.box.local.x, b->shape.box.local.y);
		r = true;
		depth = pen;
	}

	pen = max.x + a->shape.box.extents.x;
	if (pen > 0.0f && (pen < depth || depth == 0)) {
		normal = -glm::vec2(b->shape.box.local.x, b->shape.box.local.y);
		r = true;
		depth = pen;
	}

	pen = a->shape.box.extents.y - min.y;
	if (pen > 0.0f && (pen < depth || depth == 0)) {
		normal = glm::vec2(b->shape.box.local.z, b->shape.box.local.w);
		r = true;
		depth = pen;
	}

	pen = max.y + a->shape.box.extents.y;
	if (pen > 0.0f && (pen < depth || depth == 0)) {
		normal = -glm::vec2(b->shape.box.local.z, b->shape.box.local.w);
		r = true;
		depth = pen;
	}

	return r;
}

static CollisionData box_vs_box(Rigidbody* a, Rigidbody* b) {
	CollisionData r = {};

	int contact_count = 0;

	check_box_corners(a, b, r.position, contact_count, r.depth, r.normal);
	if (check_box_corners(b, a, r.position, contact_count, r.depth, r.normal)) {
		r.normal = -r.normal;
	}

	r.position /= (float)contact_count;

	r.a = a;
	r.b = b;

	return r;
}

static CollisionData box_vs_circle(Rigidbody* b, Rigidbody* c) {
	CollisionData r = {};

	glm::vec2 circle_w = c->position - b->position;
	glm::vec2 circle_to_box = glm::vec2(
		glm::dot(circle_w, glm::vec2(b->shape.box.local.x, b->shape.box.local.y)),
		glm::dot(circle_w, glm::vec2(b->shape.box.local.z, b->shape.box.local.w)));

	glm::vec2 clamped_pos = circle_to_box;
	glm::vec2 extents = b->shape.box.extents;
	if (clamped_pos.x < -extents.x) { clamped_pos.x = -extents.x; }
	if (clamped_pos.x >  extents.x) { clamped_pos.x =  extents.x; }
	if (clamped_pos.y < -extents.y) { clamped_pos.y = -extents.y; }
	if (clamped_pos.y >  extents.y) { clamped_pos.y =  extents.y; }

	glm::vec2 clamped_pos_w = b->position +
		clamped_pos.x * glm::vec2(b->shape.box.local.x, b->shape.box.local.y) +
		clamped_pos.y * glm::vec2(b->shape.box.local.z, b->shape.box.local.w);
	glm::vec2 c_to_b = c->position - clamped_pos_w;

	r.depth = c->shape.circle.radius - glm::length(c_to_b);
	r.normal = glm::normalize(c_to_b);
	r.position = clamped_pos_w;

	return r;
}

static CollisionData circle_vs_box(Rigidbody* a, Rigidbody* b) {
	auto cd = box_vs_circle(b, a);

	cd.normal = -cd.normal;

	return cd;
}

static CollisionData plane_vs_circle(Rigidbody* a, Rigidbody* b) {
	return {};
}

static CollisionData circle_vs_plane(Rigidbody* a, Rigidbody* b) {
	return plane_vs_circle(b, a);
}

static CollisionData plane_vs_box(Rigidbody* a, Rigidbody* b) {
	int contact_count = 0;
	glm::vec2 contact(0.0f, 0.0f);
	float contact_v = 0.0f;

	glm::vec2 plane_origin = a->shape.plane.normal * a->position.x;

	glm::vec2 box_size = b->shape.box.extents * 2.0f;

	for (float x = -b->shape.box.extents.x; x < box_size.x; x += box_size.x) {
		for (float y = -b->shape.box.extents.y; y < box_size.y; y += box_size.y) {
			glm::vec2 p = b->position + x * glm::vec2(b->shape.box.local.x, b->shape.box.local.y) +
				y * glm::vec2(b->shape.box.local.z, b->shape.box.local.w);

			float dist = glm::dot(p - plane_origin, a->shape.plane.normal);

			glm::vec2 disp = x * glm::vec2(b->shape.box.local.x, b->shape.box.local.y) +
				y * glm::vec2(b->shape.box.local.z, b->shape.box.local.w);
			glm::vec2 point_vel = b->velocity + b->ang_vel * glm::vec2(-disp.y, disp.x);
			float vel_to_plane = glm::dot(point_vel, a->shape.plane.normal);

			if (dist < 0.0f && vel_to_plane <= 0.0f) {
				contact_count++;
				contact += p;
				contact_v += vel_to_plane;
			}
		}
	}

	if (contact_count > 0) {
		contact /= (float)contact_count;
		
		glm::vec2 local_contact = contact - b->position;

		glm::vec2 r_vel = b->velocity + b->ang_vel * glm::vec2(-local_contact.y, local_contact.x);
		float vel_to_plane = glm::dot(r_vel, a->shape.plane.normal);

		float e = (a->restitution + b->restitution) / 2.0f;

		float r = glm::dot(local_contact, glm::vec2(a->shape.plane.normal.y, -a->shape.plane.normal.x));

		float mass = 1.0f / (1.0f / b->mass + (r * r) / b->moment);

		float j = -(1.0f + e) * vel_to_plane * mass;

		glm::vec2 force = a->shape.plane.normal * j;

		b->add_force(force, contact - b->position);
	}

	return {};
}

static CollisionData box_vs_plane(Rigidbody* a, Rigidbody* b) {
	return plane_vs_box(b, a);
}

void Rigidbody::add_force(glm::vec2 force, glm::vec2 pos) {
	if (mass <= 0.0f) { return; }

	auto accel = force / mass;
	velocity += accel;

	ang_vel += (force.y * pos.x - force.x * pos.y) / moment;
}

void Rigidbody::update(float ts) {
#ifndef TEST_MODE
	position += velocity * ts;
	add_force(glm::vec2(0.0f, sim->gravity * mass * ts));

	rotation += ang_vel * ts;

	if (constrain_rot) {
		rotation = 0.0f;
		ang_vel = 0.0f;
	}
#endif

	if (type == Rigidbody::box) {
		float cs = cosf(rotation);
		float sn = sinf(rotation);
		shape.box.local = glm::vec4(
			glm::normalize(glm::vec2(cs, sn)),
			glm::normalize(glm::vec2(-sn, cs))
		);
	}
}

RigidbodySim::RigidbodySim() : GameBase(), accum(0.0f), gravity(default_gravity), collision_iterations(default_collision_iterations) {
	detectors[Rigidbody::circle][Rigidbody::circle]  = circle_vs_circle;
	detectors[Rigidbody::box]   [Rigidbody::box]     = box_vs_box;
	detectors[Rigidbody::circle][Rigidbody::box]     = circle_vs_box;
	detectors[Rigidbody::box]   [Rigidbody::circle]  = box_vs_circle;
	detectors[Rigidbody::plane] [Rigidbody::circle]  = plane_vs_circle;
	detectors[Rigidbody::circle][Rigidbody::plane]   = circle_vs_plane;
	detectors[Rigidbody::plane] [Rigidbody::box]     = plane_vs_box;
	detectors[Rigidbody::box]   [Rigidbody::plane]   = box_vs_plane;

	rigidbodies = new Rigidbody[max_rigidbodies];
	rigidbody_count = 0;


	g_lines = &lines;

	auto rb = new_plane({ 0.0f, 1.0f });
	rb->position.y = -10.0f;
	rb->mass = 0.0f;
	rb->restitution = 0.5f;

	GameBase::Zoom(0.5f);
}

RigidbodySim::~RigidbodySim() {
	delete[] rigidbodies;
}

void RigidbodySim::Update() {
#ifndef TEST_MODE
	accum += deltaTime;

	while (accum >= physics_timestep) {
		for (size_t i = 0; i < rigidbody_count; i++) {
			auto rb = rigidbodies + i;
			rb->update(physics_timestep);
		}

		for (size_t c = 0; c < collision_iterations; c++) {
			for (size_t i = 0; i < rigidbody_count; i++) {
				auto a = rigidbodies + i;
				for (size_t j = i + 1; j < rigidbody_count; j++) {
					auto b = rigidbodies + j;

					/* No point checking two static bodies. */
					if (a->mass <= 0.0f && b->mass <= 0.0f) { continue; }
					if (a->type == Rigidbody::plane && b->type == Rigidbody::plane) { continue; }

					CollisionData cd = detectors[a->type][b->type](a, b);

					/* Plane vs box is handled separately */
					if ((a->type == Rigidbody::plane && b->type == Rigidbody::box) || (a->type == Rigidbody::box && b->type == Rigidbody::plane)) { continue; }

					if (cd.depth > 0.0f) {
						glm::vec2 r_vel = b->velocity - a->velocity;

						float a_inv_mass = a->mass <= 0.0f ? 0.0f : 1.0f / a->mass;
						float b_inv_mass = b->mass <= 0.0f ? 0.0f : 1.0f / b->mass;

						/* Positionally resolve the collision, to prevent sinking
						 * when multiple objects are stacked on each other. */
						 //glm::vec2 correction = (b_inv_mass / (a_inv_mass + b_inv_mass)) * cd.normal * cd.depth;
						 //a->position -= a_inv_mass * correction;
						 //b->position += b_inv_mass * correction;

						glm::vec2 tangent = { cd.normal.y, -cd.normal.x };
						float r1 = glm::dot(glm::normalize(cd.position - a->position), tangent);
						float r2 = glm::dot(glm::normalize(cd.position - b->position), tangent);
						float v1 = glm::dot(a->velocity, cd.normal) - r1 * a->ang_vel;
						float v2 = glm::dot(b->velocity, cd.normal) + r2 * b->ang_vel;

						if (v1 > v2) {
							float r = (a->restitution + b->restitution) / 2.0f;

							float mass_a = 1.0f / (a_inv_mass + (r1 * r1) / a->moment);
							float mass_b = 1.0f / (b_inv_mass + (r2 * r2) / b->moment);

							float j = (1.0f + r) * mass_a * mass_b / (mass_a + mass_b) * (v1 - v2);

							glm::vec2 force = j * cd.normal;
							a->add_force(-force, cd.position - a->position);
							b->add_force(force, cd.position - b->position);

							/* Calculate and apply a friction impulse. */
							//auto o_r_vel = r_vel;
							//r_vel = b->velocity - a->velocity;
							//glm::vec2 t = glm::normalize(r_vel - glm::dot(o_r_vel, cd.normal) * cd.normal);
							//float t_j = (1.0f + r) * mass_a * mass_b / (mass_a + mass_b) * (v1 - v2);

							///* Average of both bodies' friction values. */
							//float stat_fric = (a->stat_friction + b->stat_friction) / 2;

							//glm::vec2 fric_imp;
							//if (fabs(t_j) < j * stat_fric) {
							//	fric_imp = t_j * t;
							//}
							//else {
							//	float kin_fric = (a->kin_friction + b->kin_friction) / 2;
							//	fric_imp = -j * t * kin_fric;
							//}

							//a->add_force(-fric_imp * t, cd.position - a->position);
							//b->add_force(fric_imp * t, cd.position - b->position);
						}

					}
				}
			}
		}

		accum -= physics_timestep;
	}

#endif

	GameBase::Update();
}


void RigidbodySim::DrawRb(Rigidbody* rb) {
	switch (rb->type) {
		case Rigidbody::circle:
			lines.DrawCircle(rb->position, rb->shape.circle.radius, rb->color);
			lines.DrawLineSegment(rb->position,
				rb->position + glm::vec2(cosf(rb->rotation), sinf(rb->rotation) * rb->shape.circle.radius));
			break;
		case Rigidbody::box: {
			glm::vec2 p1 = rb->position - glm::vec2(rb->shape.box.local.x, rb->shape.box.local.y) * rb->shape.box.extents.x - glm::vec2(rb->shape.box.local.z, rb->shape.box.local.w) * rb->shape.box.extents.y;
			glm::vec2 p2 = rb->position + glm::vec2(rb->shape.box.local.x, rb->shape.box.local.y) * rb->shape.box.extents.x - glm::vec2(rb->shape.box.local.z, rb->shape.box.local.w) * rb->shape.box.extents.y;
			glm::vec2 p3 = rb->position - glm::vec2(rb->shape.box.local.x, rb->shape.box.local.y) * rb->shape.box.extents.x + glm::vec2(rb->shape.box.local.z, rb->shape.box.local.w) * rb->shape.box.extents.y;
			glm::vec2 p4 = rb->position + glm::vec2(rb->shape.box.local.x, rb->shape.box.local.y) * rb->shape.box.extents.x + glm::vec2(rb->shape.box.local.z, rb->shape.box.local.w) * rb->shape.box.extents.y;
			lines.DrawLineSegment(p1, p2, rb->color);
			lines.DrawLineSegment(p2, p4, rb->color);
			lines.DrawLineSegment(p3, p4, rb->color);
			lines.DrawLineSegment(p3, p1, rb->color);
		} break;
		case Rigidbody::plane: {
			auto& plane = rb->shape.plane;

			float segment = 1000.0f;
			glm::vec2 centre_point = plane.normal * rb->position.x;
			glm::vec2 tangent(plane.normal.y, -plane.normal.x);
			glm::vec2 start = centre_point + (tangent * segment);
			glm::vec2 end = centre_point - (tangent * segment);

			lines.DrawLineSegment(start, end, rb->color);
		} break;
	}
}

void RigidbodySim::Render() {
#ifdef TEST_MODE
	Rigidbody a = {};
	a.type = Rigidbody::box;
	a.position = cursorPos;
	a.shape.box.extents = { 1.0f, 1.0f };

	a.rotation = 1.0f;

	Rigidbody b = {};
	b.type = Rigidbody::box;
	b.shape.box.extents = { 1.0f, 1.0f };

	a.update(deltaTime);
	b.update(deltaTime);

	auto cd = detectors[Rigidbody::box][Rigidbody::box](&a, &b);

	a.color = { 1.0f, 1.0f, 1.0f };
	b.color = { 1.0f, 1.0f, 1.0f };

	if (cd.depth > 0.0f) {
		a.color = { 1.0f, 0.0f, 0.0f };
		b.color = { 1.0f, 0.0f, 0.0f };
	}

	DrawRb(&a);
	DrawRb(&b);

	lines.DrawLineSegment(cd.position, cd.position + cd.normal * -cd.depth);
	lines.DrawCircle(cd.position, 0.1f);
#endif

	for (size_t i = 0; i < rigidbody_count; i++) {
		auto rb = rigidbodies + i;

		DrawRb(rb);
	}

	ImGui::Begin("Config");

	ImGui::Text("Bodies: %d", rigidbody_count);
	ImGui::Text("FPS: %g", 1.0 / et);

	ImGui::InputInt("Collision Iterations", (int*)&collision_iterations);
	if (ImGui::IsItemHovered()) {
		ImGui::BeginTooltip();
			ImGui::Text("Higher numbers make the simulation more stable, but also slower.");
		ImGui::EndTooltip();
	}

	ImGui::DragFloat("Gravity", &gravity, 0.01f);

	ImGui::End();

	if (ImGui::BeginMainMenuBar()) {
		if (ImGui::BeginMenu("File")) {
			if (ImGui::MenuItem("Open")) {
				auto path = open_dialog("Data Files\0*.dat\0All Files\0*.*\0\0");
				
				if (path.size() > 0) {
					cur_save_path = path;
					load_state();
				}
			}

			if (ImGui::MenuItem("Save")) {
				if (cur_save_path.size() > 0) {
					save_state();
				} else {
					auto path = save_dialog("Data Files\0*.dat\0All Files\0*.*\0\0");

					if (path.size() > 0) {
						cur_save_path = path;
						save_state();
					}
				}
			}

			if (ImGui::MenuItem("Save As...")) {
				auto path = save_dialog("Data Files\0*.dat\0All Files\0*.*\0\0");

				if (path.size() > 0) {
					cur_save_path = path;
					save_state();
				}
			}

			ImGui::EndMenu();
		}

		ImGui::EndMainMenuBar();
	}

	GameBase::Clear();
	GameBase::Render();
}

void RigidbodySim::OnMouseClick(int mouseButton) {
	if (ImGui::IsAnyItemHovered() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) { return; }

	if (mouseButton == GLFW_MOUSE_BUTTON_LEFT) {
		auto rb = new_circle(1.0f);
		rb->position = cursorPos;
	} else if (mouseButton == GLFW_MOUSE_BUTTON_RIGHT) {
		auto rb = new_box({ 1.0f, 1.0f });
		rb->position = cursorPos;
	}
}

void RigidbodySim::OnMouseRelease(int mouseButton) {
}

Rigidbody* RigidbodySim::new_rigidbody() {
	assert(rigidbody_count < max_rigidbodies && "Too many rigidbodies!");

	auto rb = rigidbodies + rigidbody_count++;

	rb->sim = this;

	rb->restitution = 0.5f;
	rb->mass = 1.0;
	rb->stat_friction = 0.1f;
	rb->kin_friction = 0.1f;

	rb->rotation = 0.0f;
	rb->ang_vel = 0.0f;

	rb->color = { 1.0f, 1.0f, 1.0f };

	rb->position = { 0, 0 };
	rb->velocity = { 0, 0 };

	return rb;
}

Rigidbody* RigidbodySim::new_circle(float radius) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::circle;
	rb->shape.circle.radius = radius;

	rb->moment = 0.5f * rb->mass * radius * radius;

	return rb;
}

Rigidbody* RigidbodySim::new_box(glm::vec2 extents) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::box;
	rb->shape.box.extents = extents;

	rb->moment = 1.0f / 12.0f * rb->mass * (extents.x * 2) * (extents.y * 2);

	return rb;
}

Rigidbody* RigidbodySim::new_plane(glm::vec2 normal) {
	auto rb = new_rigidbody();

	rb->type = Rigidbody::plane;
	rb->shape.plane.normal = normal;

	rb->mass = 0.0f;

	rb->moment = 0.0f;

	return rb;
}

/* This is a lil trick to stop the compiler from
 * padding the data, to avoid cluttering the save file.
 *
 * A better way would be to fwrite/fread every single property
 * separately, but I'm too lazy for that. */
#pragma pack (push, 1)
struct Vec4SaveData {
	float x, y, z, w;
};

struct Vec2SaveData {
	float x, y;
};

struct CircleShapeSaveData {
	float radius;
};

struct BoxShapeSaveData {
	Vec4SaveData local;
	Vec2SaveData extents;
};

struct RigidbodySaveData {
	int type;

	union {
		CircleShape circle;
		BoxShape box;
	} shape;

	Vec2SaveData position, velocity;
	float mass, restitution;

	float rotation, ang_vel, moment;

	float stat_friction, kin_friction;

	bool constrain_rot;
};
#pragma pack (pop)

void RigidbodySim::save_state() {
	FILE* file = fopen(cur_save_path.c_str(), "wb");

	if (!file) { return; }

	fwrite(&rigidbody_count, sizeof(unsigned long), 1, file);

	for (size_t i = 0; i < rigidbody_count; i++) {
		auto rb = rigidbodies + i;

		RigidbodySaveData data;
		data.type = (int)rb->type;
		data.shape.circle.radius = rb->shape.circle.radius;
		data.shape.box.extents.x = rb->shape.box.extents.x;
		data.shape.box.extents.y = rb->shape.box.extents.y;
		data.shape.box.local.x = rb->shape.box.local.x;
		data.shape.box.local.y = rb->shape.box.local.y;
		data.shape.box.local.z = rb->shape.box.local.z;
		data.shape.box.local.w = rb->shape.box.local.w;

		data.position.x = rb->position.x;
		data.position.y = rb->position.y;

		data.velocity.x = rb->velocity.x;
		data.velocity.y = rb->velocity.y;

		data.mass = rb->mass;
		data.restitution = rb->restitution;

		data.rotation = rb->rotation;
		data.ang_vel = rb->ang_vel;
		data.moment = rb->moment;

		data.stat_friction = rb->stat_friction;
		data.kin_friction = rb->kin_friction;
		
		data.constrain_rot = rb->constrain_rot;

		fwrite(&data, sizeof(data), 1, file);
	}

	fclose(file);
}

void RigidbodySim::load_state() {
	FILE* file = fopen(cur_save_path.c_str(), "rb");

	if (!file) { return; }

	fread(&rigidbody_count, sizeof(unsigned long), 1, file);

	for (size_t i = 0; i < rigidbody_count; i++) {
		auto rb = rigidbodies + i;

		RigidbodySaveData data;
		fread(&data, sizeof(data), 1, file);

		rb->type = (Rigidbody::Type)data.type;
		rb->shape.circle.radius = data.shape.circle.radius;
		rb->shape.box.extents.x = data.shape.box.extents.x;
		rb->shape.box.extents.y = data.shape.box.extents.y;
		rb->shape.box.local.x = data.shape.box.local.x;
		rb->shape.box.local.y = data.shape.box.local.y;
		rb->shape.box.local.z = data.shape.box.local.z;
		rb->shape.box.local.w = data.shape.box.local.w;

		rb->position.x = data.position.x;
		rb->position.y = data.position.y;

		rb->velocity.x = data.velocity.x;
		rb->velocity.y = data.velocity.y;

		rb->mass = data.mass;
		rb->restitution = data.restitution;

		rb->rotation = data.rotation;
		rb->ang_vel = data.ang_vel;
		rb->moment = data.moment;

		rb->stat_friction = data.stat_friction;
		rb->kin_friction = data.kin_friction;

		rb->constrain_rot = data.constrain_rot;

		rb->sim = this;
		rb->color = { 1, 1, 1 };
	}

	fclose(file);
}
