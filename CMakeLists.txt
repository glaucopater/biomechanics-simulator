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
# Enable ObjectStream so we can load Human.tof (Jolt Physics human model)
target_compile_definitions(Jolt PUBLIC JPH_OBJECT_STREAM)

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

# cpp-httplib: header-only HTTP server for control API (GET /status, PATCH /stance)
FetchContent_Declare(
  cpp-httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG        v0.14.3
  GIT_SHALLOW    TRUE
)
FetchContent_Populate(cpp-httplib)

# Build number: increment on each build (visible in window title)
set(BUILD_NUMBER_FILE "${CMAKE_BINARY_DIR}/build_number.txt")
set(BUILD_NUMBER_HEADER "${CMAKE_BINARY_DIR}/build_number.h")
add_custom_target(bump_build_number ALL
  COMMAND ${CMAKE_COMMAND}
    -DBUILD_NUMBER_FILE="${BUILD_NUMBER_FILE}"
    -DBUILD_NUMBER_HEADER="${BUILD_NUMBER_HEADER}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/BumpBuildNumber.cmake"
  COMMENT "Bumping build number"
  BYPRODUCTS "${BUILD_NUMBER_HEADER}"
)

add_executable(biomechanics_simulator
  src/main.cpp
  src/Log.cpp
  src/HumanRagdoll.cpp
  src/Simulator.cpp
  src/SimulatorScene.cpp
  src/PoseController.cpp
  src/OpenGLDebugDrawer.cpp
  src/Visualizer.cpp
  src/HttpControl.cpp
)
add_dependencies(biomechanics_simulator bump_build_number)
target_include_directories(biomechanics_simulator PRIVATE
  ${CMAKE_BINARY_DIR}
  ${PROJECT_INCLUDE_DIR}
  ${joltphysics_SOURCE_DIR}/Jolt
  ${cpp-httplib_SOURCE_DIR}
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

# Tests: E2E verification and headless exe (enable with -DBUILD_TESTS=ON or set ON by default)
option(BUILD_TESTS "Build and enable E2E/headless tests" ON)
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
