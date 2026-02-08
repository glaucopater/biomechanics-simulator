Human model from JoltPhysics
============================

To use the official Jolt Physics human rig (same as in Samples/Tests/Rig and PoweredRigTest) with standing and walking stances:

Required files
--------------
1. Human.tof (ragdoll: skeleton, shapes, constraints) - always required for file-based rig.
2. Human/neutral.tof - standing/neutral pose (SkeletalAnimation). Used when present.
3. Human/walk.tof - walking animation (SkeletalAnimation, looped). Used when present.

Copy from JoltPhysics repository:
  From:  <JoltPhysics repo>/Assets/Human.tof          To:  <this project>/assets/Human.tof
  From:  <JoltPhysics repo>/Assets/Human/neutral.tof  To:  <this project>/assets/Human/neutral.tof
  From:  <JoltPhysics repo>/Assets/Human/walk.tof     To:  <this project>/assets/Human/walk.tof

Example if JoltPhysics is at C:\Users\glauc\github\JoltPhysics:
  copy C:\Users\glauc\github\JoltPhysics\Assets\Human.tof assets\
  mkdir assets\Human 2>nul
  copy C:\Users\glauc\github\JoltPhysics\Assets\Human\neutral.tof assets\Human\
  copy C:\Users\glauc\github\JoltPhysics\Assets\Human\walk.tof assets\Human\

Run the simulator from the project root so that paths like "assets/Human.tof" and "assets/Human/neutral.tof" resolve, e.g.:
  build\Release\biomechanics_simulator.exe
  (with working directory = project root)

If Human.tof is not present, the app falls back to the built-in human ragdoll (same skeleton, created in code).
If Human/neutral.tof or Human/walk.tof are missing, standing and walking use procedural poses instead of the keyframed animations.
