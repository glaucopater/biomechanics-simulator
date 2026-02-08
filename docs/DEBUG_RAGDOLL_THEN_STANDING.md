# Debug scenario: Ragdoll then Standing (HTTP)

Use this to trace what happens when the app starts in Standing, you switch to Ragdoll via HTTP, then back to Standing.

## Prerequisites

- Build the app with HTTP support.
- Start the app with an HTTP port, e.g. `--http-port 8080`.

## Reproduction steps

1. **Start the app** (with HTTP):
   ```powershell
   .\build\Release\biomechanics_simulator.exe --http-port 8080
   ```
   App starts in **Standing**; log will show HTTP control URL.

2. **Switch to Ragdoll** (via HTTP):
   ```powershell
   curl -X POST http://127.0.0.1:8080/stance/ragdoll
   ```
   Expect: `{"ok":true,"stance":"ragdoll"}`. Character should go ragdoll (no pose control).

3. **Switch back to Standing** (via HTTP):
   ```powershell
   curl -X POST http://127.0.0.1:8080/stance/standing
   ```
   Expect: `{"ok":true,"stance":"standing"}`. Character should stabilize in Standing.

4. **Patch position** (optional, to force root to a known position after going to Standing):
   - First get the desired position from when standing was good: `curl -s http://127.0.0.1:8080/status` and note `root_x`, `root_y`, `root_z`.
   - Then after switching to Standing you can force the root there: `curl -X PATCH http://127.0.0.1:8080/position -H "Content-Type: application/json" -d "{\"x\":0,\"y\":0.95,\"z\":0}"`
   - The main thread will snap the root to that position and zero velocities; the standing anchor is updated so the character holds there.

5. **Inspect state** (optional):
   ```powershell
   curl -s http://127.0.0.1:8080/status
   ```
   Check `stance`, `root_x`, `root_z`, `root_vx`, `root_vz`, `root_wx`, `root_wy`, `root_wz` (should go to ~0 when standing holds).

6. **Inspect logs** (optional):
   - Log file: `.cursor/debug.log`
   - Or: `curl -s "http://127.0.0.1:8080/log"`

## Instrumentation (what to look for)

- **`[HTTP] Stance applied: standing -> ragdoll`** – HTTP applied ragdoll.
- **`[HTTP] Stance applied: ragdoll -> standing`** – HTTP applied standing.
- **`[Standing] Anchor set root_x=... root_z=... root_y=... (entering/re-entering Standing)`** – First frame (or re-entry) in Standing; anchor is set from current root position. After ragdoll, this may be a fallen pose; the anchor then locks root XZ and rotation for subsequent sub-steps.

If the character drifts or spins after going back to Standing, compare the anchor values and root position/velocities in `/status` and in the log.
