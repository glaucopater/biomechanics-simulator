cmake_minimum_required(VERSION 3.16)
# Use DLL CRT before project() so FetchContent subprojects (Jolt, GLFW, imgui) inherit it
if(MSVC)
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL" CACHE STRING "MSVC runtime")
endif()
project(biomechanics-simulator LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
# Suppress FetchContent_Populate deprecation (CMP0169) until we switch to MakeAvailable for imgui
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

# Public include directory: #include "biomechanics/..."
set(PROJECT_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include CACHE PATH "Include directory")

# Jolt Physics: fetch and build from source (match our /MD runtime)
include(FetchContent)
set(USE_ASSERTS OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
set(DEBUG_RENDERER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(PROFILER_IN_DEBUG_AND_RELEASE OFF CACHE BOOL "" FORCE)
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  JoltPhysics
  GIT_REPOSITORY https://github.com/jrouwe/JoltPhysics.git
  GIT_TAG        v5.0.0
  GIT_SHALLOW    TRUE
)
FetchContent_Populate(JoltPhysics)
add_subdirectory(${joltphysics_SOURCE_DIR}/Build ${joltphysics_BINARY_DIR})

# GLFW: window and OpenGL context
include(FetchContent)
FetchContent_Declare(
  glfw
  GIT_REPOSITORY https://github.com/glfw/glfw.git
  GIT_TAG        3.4
  GIT_SHALLOW    TRUE
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

find_package(OpenGL REQUIRED)

# ImGui: UI for stance buttons
include(FetchContent)
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG        v1.89.4
  GIT_SHALLOW    TRUE
)
FetchContent_Populate(imgui)
add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl2.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw)

add_executable(biomechanics_simulator
  src/main.cpp
  src/Log.cpp
  src/Ragdoll.cpp
  src/Simulator.cpp
  src/SimulatorScene.cpp
  src/PoseController.cpp
  src/OpenGLDebugDrawer.cpp
  src/Visualizer.cpp
)
target_include_directories(biomechanics_simulator PRIVATE
  ${PROJECT_INCLUDE_DIR}
  ${joltphysics_SOURCE_DIR}/Jolt
)
target_link_libraries(biomechanics_simulator PRIVATE
  Jolt
  OpenGL::GL
  glfw
  imgui
)
if(MSVC)
  set_property(TARGET biomechanics_simulator PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
  target_compile_options(biomechanics_simulator PRIVATE
    $<$<CONFIG:Release>:/MD>
    $<$<CONFIG:Debug>:/MDd>)
  target_link_options(biomechanics_simulator PRIVATE
    $<$<CONFIG:Release>:/NODEFAULTLIB:LIBCMT>
    $<$<CONFIG:Debug>:/NODEFAULTLIB:LIBCMTd>)
endif()

# Optional: tests (add_subdirectory(tests) and build with -DBUILD_TESTS=ON)
option(BUILD_TESTS "Build tests" OFF)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
