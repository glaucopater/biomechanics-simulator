#pragma once

namespace biomechanics::config
{
// Physics simulation settings
struct PhysicsSettings
{
    // Time step
    float timeStep = 1.0f / 60.0f;

    // Substeps for more stable integration (increase for ragdoll stability)
    int numSubSteps = 4;

    // Solver iterations (higher = more stable joints, more CPU)
    int velocityIterations = 12;
    int positionIterations = 8;

    // Sleep thresholds (tune if bodies jitter or fall asleep too early)
    float linearSleepThreshold = 0.05f;
    float angularSleepThreshold = 0.05f;
};

// Window / rendering settings
struct WindowSettings
{
    int width = 1280;
    int height = 720;
    const char* title = "Biomechanics Simulator";
};

inline PhysicsSettings getPhysicsSettings()
{
    PhysicsSettings settings;
    return settings;
}

inline WindowSettings getWindowSettings()
{
    WindowSettings settings;
    return settings;
}

} // namespace biomechanics::config
