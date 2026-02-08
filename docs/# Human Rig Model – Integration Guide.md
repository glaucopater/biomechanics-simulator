# Human Rig Model – Integration Guide

This document describes how to use the Jolt Physics Human rig assets in another project with **two stances: standing and walking**.

---

## 1. Asset overview

The Human rig comes from the [JoltPhysics](https://github.com/jrouwe/JoltPhysics) repository. The assets are derived from the **Aloy** character (Horizon Zero Dawn); see `Assets/LICENSE` for licensing.

### 1.1 Required files

| File | Purpose | Required for |
|------|--------|----------------|
| **`Assets/Human.tof`** | Ragdoll definition: skeleton, collision shapes, constraints. | Always |
| **`Assets/Human/neutral.tof`** | Standing / neutral pose (single pose or short animation). | Standing stance |
| **`Assets/Human/walk.tof`** | Walking animation (keyframed, loopable). | Walking stance |

Copy these three files into your project’s asset layout and keep the same relative path (`Human/` subfolder for the two pose/animation files) so paths like `"Human.tof"` and `"Human/neutral.tof"` resolve correctly.

### 1.2 File formats

- **`.tof`** = Jolt **ObjectStream** text format (human-readable serialization).
- **`Human.tof`** contains a **`RagdollSettings`** object (skeleton + physics parts + constraints).
- **`Human/neutral.tof`** and **`Human/walk.tof`** contain **`SkeletalAnimation`** objects (joint names + keyframes). A “standing” stance is typically a neutral pose stored as an animation (e.g. one keyframe or a short clip); you sample it at a fixed time (e.g. `0.0f`) to get the standing pose.

### 1.3 Optional Human assets (same rig)

For reference, other files under `Assets/Human/` that use the **same skeleton** as `Human.tof`:

- `dead_pose1.tof` … `dead_pose4.tof` – ragdoll-style poses  
- `jog_hd.tof`, `neutral_hd.tof`, `skeleton_hd.tof` – higher-density joint animations  
- `sprint.tof` – sprint animation  

For **standing** and **walking** only, you need **`neutral.tof`** and **`walk.tof`**.

---

## 2. Build / API requirements

Your project must:

1. **Use Jolt Physics** (link the Jolt library).
2. **Enable ObjectStream** so `.tof` files can be loaded:
   - CMake: `ENABLE_OBJECT_STREAM` ON (default in Jolt).
   - This compiles `ObjectStreamIn` / `ObjectStreamOut` and RTTI for serializable types.
3. **Include the relevant headers** (see code below).
4. **Register Jolt types** so `RagdollSettings` and `SkeletalAnimation` can be deserialized (call `RegisterTypes()` as in the Jolt samples).

Asset loading in the samples uses a small **`AssetStream`** wrapper that opens files from an “assets base path”; you can replace that with your own file opening (e.g. `std::ifstream`) and pass the stream to `ObjectStreamIn::sReadObject`.

---

## 3. Loading the rig and animations

### 3.1 Load ragdoll definition (skeleton + physics)

Load **`Human.tof`** once to get the shared ragdoll setup:

```cpp
#include <Jolt/ObjectStream/ObjectStreamIn.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

// Your way to open the file (e.g. base path + "Human.tof")
std::ifstream stream("path/to/Assets/Human.tof", std::ios::in);

Ref<RagdollSettings> ragdollSettings;
if (!ObjectStreamIn::sReadObject(stream.Get(), ragdollSettings))
    // handle error: "Unable to read ragdoll"

// Optional: set motion type and layers for your game
for (BodyCreationSettings &part : ragdollSettings->mParts)
{
    part.mMotionType = EMotionType::Dynamic;  // or Kinematic for scripted movement
    part.mObjectLayer = YourLayer;
}
```

You need **one** `RagdollSettings` per “type” of human (shared across all instances that use the same rig).

### 3.2 Load standing and walking animations

Load **`Human/neutral.tof`** (standing) and **`Human/walk.tof`** (walking) as `SkeletalAnimation`:

```cpp
#include <Jolt/Skeleton/SkeletalAnimation.h>

// Standing: neutral pose
Ref<SkeletalAnimation> standingAnim;
std::ifstream neutralStream("path/to/Assets/Human/neutral.tof", std::ios::in);
if (!ObjectStreamIn::sReadObject(neutralStream.Get(), standingAnim))
    // handle error

// Walking: looped animation
Ref<SkeletalAnimation> walkingAnim;
std::ifstream walkStream("path/to/Assets/Human/walk.tof", std::ios::in);
if (!ObjectStreamIn::sReadObject(walkStream.Get(), walkingAnim))
    // handle error

walkingAnim->SetIsLooping(true);  // optional; walk is typically looped
```

Use **`standingAnim`** for the standing stance and **`walkingAnim`** for the walking stance.

---

## 4. Using the rig with two stances (standing / walking)

### 4.1 Create a ragdoll instance

Use the same `RagdollSettings` for every human instance; create one ragdoll per character:

```cpp
Ref<Ragdoll> ragdoll = ragdollSettings->CreateRagdoll(0, 0, physicsSystem);
ragdoll->AddToPhysicsSystem(EActivation::Activate);
```

### 4.2 Pose from skeleton

You need a **`SkeletonPose`** bound to the same skeleton as the ragdoll (from `ragdollSettings->GetSkeleton()`):

```cpp
#include <Jolt/Skeleton/SkeletonPose.h>

SkeletonPose pose;
pose.SetSkeleton(ragdollSettings->GetSkeleton());
```

### 4.3 Standing stance

- Use the **neutral** animation as the “standing” pose.
- Sample at a fixed time (e.g. start of the clip):

```cpp
standingAnim->Sample(0.0f, pose);
pose.CalculateJointMatrices();
ragdoll->SetPose(pose);
```

If the character is kinematic and you only want to display or drive to this pose every frame:

```cpp
standingAnim->Sample(0.0f, pose);
pose.CalculateJointMatrices();
ragdoll->DriveToPoseUsingKinematics(pose, deltaTime);
```

### 4.4 Walking stance

- Use the **walk** animation and advance time each frame (loop is handled by `Sample` if the animation is looping):

```cpp
float walkTime = 0.0f;  // persist per character

// Each frame:
walkTime += deltaTime;
walkingAnim->Sample(walkTime, pose);
pose.CalculateJointMatrices();
ragdoll->DriveToPoseUsingKinematics(pose, deltaTime);
```

For a **dynamic** ragdoll (physics-driven but influenced by the animation), the same pattern applies: sample the walking animation into `pose`, then call `DriveToPoseUsingKinematics` (or `DriveToPoseUsingMotors` if you use motors). See Jolt samples (e.g. KinematicRigTest, SoftKeyframedRigTest) for full context.

### 4.5 Switching between standing and walking

- **Standing**: sample `standingAnim` at `0.0f` (or a fixed time) and drive the ragdoll to that pose.
- **Walking**: maintain a `walkTime`, sample `walkingAnim->Sample(walkTime, pose)`, then drive the ragdoll to that pose.
- When switching from walk to stand, you can reset `walkTime` or leave it; when switching to standing, stop updating `walkTime` and use `standingAnim->Sample(0.0f, pose)`.

---

## 5. Summary: files and usage

| Stance   | Asset file           | Usage |
|----------|----------------------|--------|
| **Rig**  | `Human.tof`          | Load once → `RagdollSettings`. Create one `Ragdoll` per character from it. |
| **Standing** | `Human/neutral.tof` | Load once → `SkeletalAnimation`. Each frame (or when in “stand” state): `Sample(0.0f, pose)`, then `SetPose` or `DriveToPoseUsingKinematics`. |
| **Walking**  | `Human/walk.tof`  | Load once → `SkeletalAnimation`. Each frame: `walkTime += dt`, `Sample(walkTime, pose)`, then drive ragdoll to `pose`. |

### 5.1 Minimal file set for your project

- `Human.tof`  
- `Human/neutral.tof`  
- `Human/walk.tof`  

Paths in code should match your asset layout (e.g. `"Human.tof"`, `"Human/neutral.tof"`, `"Human/walk.tof"` with your base path).

---

## 6. Reference: Jolt sample code

- **Ragdoll loading**: `Samples/Utils/RagdollLoader.cpp` (`RagdollLoader::sLoad` for `Human.tof`).
- **Standing / neutral**: Used in `Samples/Tests/Rig/SkeletonMapperTest.cpp` (`Human/neutral.tof`).
- **Walking**: `Samples/Tests/Rig/KinematicRigTest.cpp`, `SoftKeyframedRigTest.cpp` (load `Human/walk.tof`, sample with time, drive ragdoll).
- **Animation switching**: `Samples/Tests/Rig/PoweredRigTest.cpp` (switches between `neutral`, `walk`, `sprint`, `dead_pose1`… via `Human/<name>.tof`).

---

## 7. Skeleton and pose data

- **RagdollSettings** holds the **Skeleton** (joint hierarchy and names), plus one **BodyCreationSettings** per bone (shapes, mass, constraints).
- **SkeletalAnimation** holds **AnimatedJoint**s: each has a joint name (matching the skeleton) and a list of **Keyframe**s (time, rotation, translation).  
- **SkeletonPose** holds the current joint state (rotation, translation per joint in local space). You fill it with `SkeletalAnimation::Sample(time, pose)`, then call `pose.CalculateJointMatrices()` before passing it to the ragdoll.

The Human rig’s joint names and hierarchy are defined inside `Human.tof` and must match the joint names in `neutral.tof` and `walk.tof` (they do in the provided assets).
