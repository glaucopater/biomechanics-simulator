# Milestones

## Milestone 1: Standing stable (no drift, no spin)

**Status:** Done

**Summary:** In **Standing** mode, the model stays in place without drifting horizontally or spinning.

**What was fixed:**

- **Root position:** Root body XZ is anchored on the first frame in Standing and re-applied after every physics sub-step so the character does not drift.
- **Root orientation:** Root rotation is anchored the same way and re-applied each sub-step with `SetPositionAndRotation`, so the character does not spin.
- **Root velocities:** After each sub-step, root linear velocity XZ and root angular velocity are zeroed so physics cannot accumulate drift or spin.

**Implementation:** In `Visualizer.cpp`, inside the sub-steps loop (after each `physics->Update()`), when `mode == Standing` we:

1. Capture anchor position (XZ) and rotation (quat) on first frame.
2. Set root position to `(anchor_x, current_y, anchor_z)` and rotation to the stored quat.
3. Set root linear velocity to `(0, vy, 0)` and angular velocity to zero.

**Also in this period:** Build number in window title (increments every build), Freeze button to pause simulation, HTTP control API (GET /status, PATCH /stance, POST buttons, GET /log).
