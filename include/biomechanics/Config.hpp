#pragma once

#include <string>
#include <filesystem>

namespace biomechanics
{

struct SimulatorConfig
{
    // Physics
    float dt = 1.0f / 240.0f;
    int substeps = 4;

    // Ragdoll
    float ragdoll_mass = 70.0f;
    float height = 1.75f;

    // Control
    float control_frequency = 60.0f; // Hz

    // Simulation
    float simulation_duration = 10.0f; // seconds

    // Paths
    std::filesystem::path output_dir = "output";
};

} // namespace biomechanics
