workspace "physics"
	configurations { "debug", "release" }

project "physics"
	kind "ConsoleApp"
	language "C++"
	cppdialect "C++20"

	objdir = "obj"
	targetdir = "bin"

	files {
		"SimpleFramework/**.h",
		"SimpleFramework/**.cpp",
		"SimpleFramework/**.c",
		"imgui/**.h",
		"imgui/**.cpp",
	}

	includedirs {
		"GLFW",
		"imgui",
		"glm"
	}

	links {
		"X11",
		"dl",
		"pthread",
		"glfw"
	}

	filter "configurations:debug"
		symbols "on"
		runtime "debug"

	filter "configurations:release"
		optimize "on"
		runtime "release"
