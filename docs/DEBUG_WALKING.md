# Debug: Walking stance (HTTP)

Use the HTTP endpoint to switch to Walking and inspect logs / status to debug "crazy" walking (e.g. falling, spinning, not moving forward, wrong foot heights).

## Prerequisites

- Build the app with HTTP support.
- Start the app with an HTTP port, e.g. `--http-port 8080`.

## Trigger walking via endpoint

1. **Start the app** (with HTTP):
   ```powershell
   .\build\Release\biomechanics_simulator.exe --http-port 8080
   ```
   App starts in **Standing**.

2. **Switch to Walking** (via HTTP):
   ```powershell
   curl -X PATCH http://127.0.0.1:8080/stance -H "Content-Type: application/json" -d "{\"stance\":\"walking\"}"
   ```
   Or use the convenience route:
   ```powershell
   curl -X POST http://127.0.0.1:8080/stance/walking
   ```
   Expect: `{"ok":true,"stance":"walking"}` (or similar). Character should start walking.

3. **Poll state while walking**:
   ```powershell
   curl -s http://127.0.0.1:8080/status
   ```
   Response includes:
   - `stance`: `"walking"`
   - `root_x`, `root_y`, `root_z`: root position (should move forward in +Z or -Z depending on forward direction if `walk_forward_speed > 0`)
   - `root_vx`, `root_vy`, `root_vz`: root velocity (use to see drift or lack of forward motion)
   - `walk_time`: time used for animation-driven walk (seconds)
   - `walk_phase`: procedural walk phase in [0, 2π)
   - `log_tail`: last log lines (includes `[Walk]` and `[HTTP] Stance applied`)

4. **Inspect full log**:
   - Log file: `.cursor/debug.log`
   - Or: `curl -s "http://127.0.0.1:8080/log"`

## Log lines to look for

- **`[HTTP] Stance applied: standing -> walking`**  
  Confirms the stance was applied from the endpoint.

- **`[Walk] Entered: root (x,z)=(...), y=..., vel=(...), walk_forward_speed=..., walk_speed=..., using_anim=0|1`**  
  Logged once when entering Walking. Check:
  - Root position and velocity at switch (e.g. large velocity = may already be falling).
  - `walk_forward_speed`: 0 = in place, >0 = forward motion per second.
  - `using_anim=1`: animation-driven walk (from `.tof`); `using_anim=0`: procedural walk (sin-based hips/knees/arms).

- **`[Walk] phase=... time=... root(x,y,z) v(vx,vy,vz) pelvis_y=... L_foot_y=... R_foot_y=... anim|proc`**  
  Logged every 30 frames while walking. Check:
  - `root(x,y,z)`: forward drift (x,z) and height (y).
  - `v(vx,vy,vz)`: forward velocity and vertical (e.g. falling = large negative vy).
  - `pelvis_y`, `L_foot_y`, `R_foot_y`: foot heights (should stay near ground; if one foot is way above the other or both high, gait may be wrong).
  - `anim` vs `proc`: which walk path is active.

## Config (affects walk behavior)

In `SimulatorConfig` (e.g. `Config.hpp` / defaults):

- `walk_forward_speed`: m/s forward when using walk animation (0 = in place).
- `walk_speed`: gait cycle Hz for procedural walk.
- `walk_hip_amplitude`, `walk_knee_amplitude`, `walk_arm_amplitude`: procedural joint swing.
- `walk_max_torque`, `walk_joint_spring_stiffness`: PD limits during walk.

If walking is "crazy", compare these with the values logged in `[Walk] Entered` and consider tuning or checking that the correct drive (anim vs procedural) is used.
