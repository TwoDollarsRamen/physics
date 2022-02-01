#include <stdlib.h>
#include <time.h>

#include <imgui.h>

#include "Example.h"

#define particle_max 1000
#define force_point_max 64
#define gravity -10
#define force_point_rad 0.2f

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

	force_points = new glm::vec2[force_point_max];

	srand(time(0));

	for (size_t i = 0; i < particle_count; i++) {
		particles[i] = new_particle();
	}
}

Example::~Example() {
	delete[] particles;
	delete[] force_points;
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

		/* Force points influence the velocity of particles.
		 *
		 * This can lead to some quite interesting behaviour. */
		for (size_t ii = 0; ii < force_point_count; ii++) {
			auto& fp = force_points[ii];

			auto to_fp = particle.position - fp;
			auto inv_dist_to_fp = 1.0f / glm::length(to_fp);
			auto to_fp_vec = glm::normalize(to_fp);

			particle.velocity += (inv_dist_to_fp * to_fp_vec);
		}

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
	if (dragging) {
		force_points[drag_handle] = cursorPos;
	}

	for (size_t i = 0; i < particle_count; i++) {
		auto& particle = particles[i];

		lines.DrawCircle(particle.position, particle.life,
			glm::vec3(glm::normalize(particle.velocity), particle.life));
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
			FILE* file = fopen("save.dat", "wb");
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
				}

				for (size_t i = 0; i < force_point_count; i++) {
					auto& fp = force_points[i];

					fwrite(&fp.x, sizeof(fp.x), 1, file);
					fwrite(&fp.y, sizeof(fp.y), 1, file);
				}

				fclose(file);
			} else {
				fprintf(stderr, "Failed to fopen file `save.dat'.\n");
			}
		}

		if (ImGui::MenuItem("Load")) {
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
				}

				for (size_t i = 0; i < force_point_count; i++) {
					auto& fp = force_points[i];

					fread(&fp.x, sizeof(fp.x), 1, file);
					fread(&fp.y, sizeof(fp.y), 1, file);
				}

				fclose(file);
			} else {
				fprintf(stderr, "Failed to fopen file `save.dat'.\n");
			}
		}

		ImGui::EndMenu();
	}

	ImGui::EndMainMenuBar();

	//This call puts all the lines you've set up on screen - don't delete it or things won't work.
	GameBase::Render();
}

void Example::OnMouseClick(int mouseButton)
{
	if (ImGui::IsAnyItemHovered() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) { return; }

	/* Adding/dragging force points */
	if (mouseButton == GLFW_MOUSE_BUTTON_LEFT) {
		for (size_t i = 0; i < force_point_count; i++) {
			auto& fp = force_points[i];

			if (powf((cursorPos.x - fp.x), 2.0f) + pow((cursorPos.y - fp.y), 2.0f) < powf(force_point_rad, 2.0f)) {
				dragging = true;
				drag_handle = i;
				return;
			}
		}

		/* Nothing is being dragged, we want to add a new point. */
		if (force_point_count < force_point_max) {
			force_points[force_point_count++] = cursorPos;
		}
	}
}

void Example::OnMouseRelease(int mouseButton) {
	if (ImGui::IsAnyItemHovered() || ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow)) { return; }

	dragging = false;

	/* Removing force points*/
	if (mouseButton == GLFW_MOUSE_BUTTON_RIGHT) {
		for (size_t i = 0; i < force_point_count; i++) {
			auto& fp = force_points[i];

			if (powf((cursorPos.x - fp.x), 2.0f) + pow((cursorPos.y - fp.y), 2.0f) < powf(force_point_rad, 2.0f)) {
				if (particle_count > 1) {
					force_points[i] = force_points[force_point_count - 1];
				}
				force_point_count--;

				return;
			}
		}
	}
}