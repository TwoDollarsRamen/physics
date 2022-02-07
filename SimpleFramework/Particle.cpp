#include <stdlib.h>
#include <time.h>

#include <imgui.h>

#include "Particle.h"

#define particle_max 1000
#define force_point_max 64
#define gravity -10
#define force_point_rad 0.2f

static float random_float(float min, float max) {
	float scale = rand() / (float)RAND_MAX;
	return min + scale * (max - min);
}

static bool circle_vs_point(glm::vec2 point, float r, glm::vec2 centre) {
	return powf((point.x - centre.x), 2.0f) + pow((point.y - centre.y), 2.0f) < powf(r, 2.0f);
}

Particle new_particle() {
	Particle particle = { { 0, 0}, { 0, 0 }, random_float(0.5, 1) };

	particle.velocity.x = random_float(-5, 5);
	particle.velocity.y = random_float(-5, 5);

	particle.mass = 1.0f;

	return particle;
}

ParticleSim::ParticleSim() : GameBase()
{
	particle_count = particle_max;
	particles = new Particle[particle_count];
	memset(particles, 0, particle_count * sizeof(*particles));

	force_points = new glm::vec2[force_point_max];

	srand((unsigned int)time(0));

	for (size_t i = 0; i < particle_count; i++) {
		particles[i] = new_particle();
	}
}

ParticleSim::~ParticleSim() {
	delete[] particles;
	delete[] force_points;
}

void ParticleSim::Update()
{
	//This call ensures that your mouse position and aspect ratio are maintained as correct.
	GameBase::Update();

	if (cam_dragging) {
		cameraCentre = cam_drag_offset + cursorPos;
	}

	//Your physics (or whatever) code goes here!

	if (pause) { return; }

	while (particle_count < particle_max) {
		particles[particle_count++] = new_particle();
	}

	for (size_t i = 0; i < particle_count; i++) {
		auto& particle = particles[i];

		/* Force points influence the velocity of particles.
		 *
		 * This can lead to some quite interesting behaviour. */
		glm::vec2 force = { 0, 0 };
		for (size_t ii = 0; ii < force_point_count; ii++) {
			auto& fp = force_points[ii];

			auto to_fp = particle.position - fp;
			auto inv_dist_to_fp = 1.0f / glm::length(to_fp);
			auto to_fp_vec = glm::normalize(to_fp);

			force += (inv_dist_to_fp * to_fp_vec) * 100.0f;
		}

		glm::vec2 accel = force / particle.mass;
		particle.velocity += force * deltaTime;

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

void ParticleSim::Render()
{
	if (dragging) {
		force_points[drag_handle] = cursorPos;
	}

	for (size_t i = 0; i < particle_count; i++) {
		auto& particle = particles[i];

		lines.DrawCircle(particle.position, particle.life,
			glm::vec3(glm::normalize(particle.velocity), particle.life), 10);
	}

	for (size_t i = 0; i < force_point_count; i++) {
		auto& fp = force_points[i];

		lines.DrawCircle(fp, force_point_rad, { 1, 1, 1 });
	}

	ImGui::BeginMainMenuBar();

	/* Basic save feature, all the force points as well as the particles can be
	 * saved and loaded again ¯\_(ツ)_/¯ */
	if (ImGui::BeginMenu("File")) {
		if (ImGui::MenuItem("Save")) {
			save_state("save.dat");
		}

		if (ImGui::MenuItem("Load")) {
			load_state("save.dat");
		}

		ImGui::EndMenu();
	}

	if (ImGui::BeginMenu("Simulation")) {
		ImGui::MenuItem("Paused", "", &pause);

		ImGui::EndMenu();
	}

	ImGui::Text("FPS: %g", 1.0 / et);

	ImGui::EndMainMenuBar();

	//This call puts all the lines you've set up on screen - don't delete it or things won't work.
	GameBase::Render();
}

void ParticleSim::OnMouseClick(int mouseButton)
{
	if (ImGui::IsAnyItemHovered() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) { return; }

	/* Adding/dragging force points */
	if (mouseButton == GLFW_MOUSE_BUTTON_LEFT) {
		for (size_t i = 0; i < force_point_count; i++) {
			auto& fp = force_points[i];

			if (circle_vs_point(cursorPos, force_point_rad, fp)) {
				dragging = true;
				drag_handle = i;
				return;
			}
		}

		/* Nothing is being dragged, we want to add a new point. */
		if (force_point_count < force_point_max) {
			force_points[force_point_count++] = cursorPos;
		}
	} else if (mouseButton == GLFW_MOUSE_BUTTON_MIDDLE) {
		cam_dragging = true;
		cam_drag_offset = cameraCentre - cursorPos;
	}
}

void ParticleSim::OnMouseRelease(int mouseButton) {
	if (ImGui::IsAnyItemHovered() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) { return; }

	if (mouseButton == GLFW_MOUSE_BUTTON_MIDDLE) {
		cam_dragging = false;
	} else if (mouseButton == GLFW_MOUSE_BUTTON_LEFT) {
		dragging = false;
	}

	/* Removing force points*/
	if (mouseButton == GLFW_MOUSE_BUTTON_RIGHT) {
		for (size_t i = 0; i < force_point_count; i++) {
			auto& fp = force_points[i];

			if (circle_vs_point(cursorPos, force_point_rad, fp)) {
				if (particle_count > 1) {
					force_points[i] = force_points[force_point_count - 1];
				}
				force_point_count--;

				return;
			}
		}
	}
}

void ParticleSim::save_state(const char* path) {
	FILE* file = fopen(path, "wb");
	if (file) {
		fwrite(&particle_count, sizeof(particle_count), 1, file);
		fwrite(&force_point_count, sizeof(force_point_count), 1, file);

		for (size_t i = 0; i < particle_count; i++) {
			auto& particle = particles[i];

			fwrite(&particle.position.x, sizeof(particle.position.x), 1, file);
			fwrite(&particle.position.y, sizeof(particle.position.y), 1, file);
			fwrite(&particle.velocity.x, sizeof(particle.velocity.x), 1, file);
			fwrite(&particle.velocity.y, sizeof(particle.velocity.y), 1, file);
			fwrite(&particle.life, sizeof(particle.life), 1, file);
			fwrite(&particle.mass, sizeof(particle.mass), 1, file);
		}

		for (size_t i = 0; i < force_point_count; i++) {
			auto& fp = force_points[i];

			fwrite(&fp.x, sizeof(fp.x), 1, file);
			fwrite(&fp.y, sizeof(fp.y), 1, file);
		}

		fclose(file);
	} else {
		fprintf(stderr, "Failed to fopen file `%s'.\n", path);
	}
}

void ParticleSim::load_state(const char* path) {
	FILE* file = fopen("save.dat", "rb");
	if (file) {
		fread(&particle_count, sizeof(particle_count), 1, file);
		fread(&force_point_count, sizeof(force_point_count), 1, file);

		for (size_t i = 0; i < particle_count; i++) {
			auto& particle = particles[i];

			fread(&particle.position.x, sizeof(particle.position.x), 1, file);
			fread(&particle.position.y, sizeof(particle.position.y), 1, file);
			fread(&particle.velocity.x, sizeof(particle.velocity.x), 1, file);
			fread(&particle.velocity.y, sizeof(particle.velocity.y), 1, file);
			fread(&particle.life, sizeof(particle.life), 1, file);
			fread(&particle.mass, sizeof(particle.mass), 1, file);
		}

		for (size_t i = 0; i < force_point_count; i++) {
			auto& fp = force_points[i];

			fread(&fp.x, sizeof(fp.x), 1, file);
			fread(&fp.y, sizeof(fp.y), 1, file);
		}

		fclose(file);
	} else {
		fprintf(stderr, "Failed to fopen file `%s'.\n", path);
	}
}
