#include <windows.h>

#include "GameBase.h"
#include "GLFWCallbacks.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#define GLFW_EXPOSE_NATIVE_WIN32
#include <glfw3native.h>

double now, last;

glm::mat4 GameBase::GetCameraTransform() const
{
	return glm::ortho(
		-aspectRatio * cameraHeight / 2.0f + cameraCentre.x, 
		aspectRatio * cameraHeight / 2.0f + cameraCentre.x,
		-cameraHeight / 2.0f + cameraCentre.y,
		cameraHeight / 2.0f + cameraCentre.y,
		-1.0f, 1.0f);
}

GameBase::GameBase()
{
	if (!glfwInit())
	{
		return;
	}
	//Can choose resolution here.
	window = glfwCreateWindow(1280, 720, "Physics Testbed", nullptr, nullptr);
	glfwSetWindowPos(window, 1950, 30);

	//This is the somewhat hacky oldschool way of making callbacks work without everything having to be global. Look
	//at the way the function callbacks work to get an idea of what's going on.
	glfwSetWindowUserPointer(window, (void*)this);	

	//These functions will be able to reference the testbed object via the user pointer.
	glfwSetWindowSizeCallback(window, WindowResizeCallback);
	glfwSetMouseButtonCallback(window, MouseButtonCallback);
	glfwSetScrollCallback(window, MouseWheelCallback);

	if (!window)
	{
		//If the window failed to create for some reason, abandon ship.
		glfwTerminate();
		return;
	}

	//We have to tell glfw to use the OpenGL context from the window.
	glfwMakeContextCurrent(window);

	//This is where GLAD gets set up. After this point we can use openGL functions.
	if (!gladLoadGL())
	{
		return;
	}
	glfwSwapInterval(1);


	ImGui::CreateContext();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init();

	simpleShader = ShaderProgram("Simple.vsd", "Simple.fsd");
	simpleShader.UseShader();

	lines.Initialise();

	glClearColor(0, 0, 0, 1);

	grid.Initialise();
	for (float i = (float)-gridLimits; i <= (float)gridLimits; i++)
	{
		glm::vec3 colour = (i == 0) ? glm::vec3(0.8f, 0.0f, 0.8f) : glm::vec3(0.3f, 0.0f, 0.3f);
		grid.DrawLineSegment({ i, -gridLimits }, { i, gridLimits }, colour);
		grid.DrawLineSegment({ -gridLimits, i }, { gridLimits, i }, colour);
	}
	grid.DrawLineSegment({ 0,0 }, { 1, 0 }, { 1, 0, 0 });
	grid.DrawLineSegment({ 0,0 }, { 0, 1 }, { 0, 1, 0 });
	grid.Compile();
}

GameBase::~GameBase()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
	glfwTerminate();
}

void GameBase::Update()
{
	int width, height;
	glfwGetWindowSize(window, &width, &height);
	aspectRatio = width / (float)height;

//	time += deltaTime;

	glm::mat4 deprojection = glm::inverse(GetCameraTransform());
	double cursorX, cursorY;
	glfwGetCursorPos(window, &cursorX, &cursorY);
	cursorX = (cursorX / width) * 2.0 - 1.0;
	cursorY = -((cursorY / height) * 2.0 - 1.0);

	glm::vec4 mousePosNDC(float(cursorX), float(cursorY), 0, 1);

	glm::vec4 mousePosWorld = deprojection * mousePosNDC;

	cursorPos.x = mousePosWorld.x;
	cursorPos.y = mousePosWorld.y;

	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
	{
		cameraCentre.x -= cameraSpeed * deltaTime * cameraHeight;
	}
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
	{
		cameraCentre.x += cameraSpeed * deltaTime * cameraHeight;
	}
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
	{
		cameraCentre.y += cameraSpeed * deltaTime * cameraHeight;
	}
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
	{
		cameraCentre.y -= cameraSpeed * deltaTime * cameraHeight;
	}

	leftButtonDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
	rightButtonDown = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;


	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();

}

void GameBase::Render()
{
	glm::mat4 orthoMat = GetCameraTransform();
	simpleShader.SetUniform("vpMatrix", orthoMat);
	grid.Draw();	//Grid lines don't change so we just draw them.
	lines.UpdateFrame();	//Other lines potentially change every frame, so we have to compile/draw/clear them.


	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

	glfwSwapBuffers(window);
	glfwPollEvents();

	now = glfwGetTime();
	et = (float)(now - last);
	last = now;
}

void GameBase::Clear() {
	glClear(GL_COLOR_BUFFER_BIT);
}

bool GameBase::IsRunning() const
{
	return !glfwWindowShouldClose(window);
}

void GameBase::OnMouseClick(int mouseButton)
{
	//Override this function if you need to respond to mouse clicks.
}

void GameBase::OnMouseRelease(int mouseButton)
{
	//Override this function if you need to respond to mouse button releases.
}

void GameBase::Zoom(float zoomFactor)
{
	cameraHeight /= zoomFactor;
}

std::string GameBase::save_dialog(const char* filter) {
	OPENFILENAMEA ofn;
	CHAR szFile[260] = { 0 };
	CHAR currentDir[256] = { 0 };
	ZeroMemory(&ofn, sizeof(OPENFILENAME));
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = glfwGetWin32Window(window);
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = sizeof(szFile);
	if (GetCurrentDirectoryA(256, currentDir))
		ofn.lpstrInitialDir = currentDir;
	ofn.lpstrFilter = filter;
	ofn.nFilterIndex = 1;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;

	// Sets the default extension by extracting it from the filter
	ofn.lpstrDefExt = strchr(filter, '\0') + 1;

	if (GetSaveFileNameA(&ofn) == TRUE)
		return ofn.lpstrFile;

	return std::string();
}

std::string GameBase::open_dialog(const char* filter) {
	OPENFILENAMEA ofn;
	CHAR szFile[260] = { 0 };
	CHAR currentDir[256] = { 0 };
	ZeroMemory(&ofn, sizeof(OPENFILENAME));
	ofn.lStructSize = sizeof(OPENFILENAME);
	ofn.hwndOwner = glfwGetWin32Window(window);
	ofn.lpstrFile = szFile;
	ofn.nMaxFile = sizeof(szFile);
	if (GetCurrentDirectoryA(256, currentDir))
		ofn.lpstrInitialDir = currentDir;
	ofn.lpstrFilter = filter;
	ofn.nFilterIndex = 1;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

	if (GetOpenFileNameA(&ofn) == TRUE)
		return ofn.lpstrFile;

	return std::string();
}