#include "build_number.h"
#include "biomechanics/Config.hpp"
#include "biomechanics/HttpControl.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/OpenGLDebugDrawer.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/SimulatorScene.hpp"
#include "biomechanics/Visualizer.hpp"
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#endif
#include <GLFW/glfw3.h>
#include <GL/gl.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>
#include <Physics/Collision/RayCast.h>
#include <Physics/Collision/CastResult.h>
#include <Physics/Body/BodyFilter.h>
#include <Physics/Body/BodyID.h>
#include <Math/Vec3.h>
#include <Math/Quat.h>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace biomechanics {

namespace {

struct OrbitCamera {
  float azimuth = -0.5f;    // radians, horizontal angle
  float elevation = 0.35f;  // radians, vertical (up positive)
  float distance = 5.f;
  float look_at[3] = {0.f, 0.8f, 0.f};  // mid-body when standing on ground
  float sensitivity = 0.005f;
  float zoom_sensitivity = 0.15f;
  bool left_down = false;
  double last_x = 0.;
  double last_y = 0.;
};

constexpr float MIN_DISTANCE = 1.f;
constexpr float MAX_DISTANCE = 50.f;
constexpr float MAX_ELEVATION = 1.45f;  // ~83 deg

void setup_projection(int width, int height) {
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  float aspect = (width > 0 && height > 0) ? static_cast<float>(width) / static_cast<float>(height) : 1.f;
  float fov = 50.f * static_cast<float>(3.14159265 / 180.0);
  float near_plane = 0.1f;
  float far_plane = 500.f;
  float y = near_plane * std::tan(fov * 0.5f);
  float x = y * aspect;
  glFrustum(-x, x, -y, y, near_plane, far_plane);
}

void setup_camera(const OrbitCamera& cam) {
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  float az_deg = cam.azimuth * 180.f / 3.14159265f;
  float el_deg = cam.elevation * 180.f / 3.14159265f;
  glTranslatef(0.f, 0.f, -cam.distance);
  glRotatef(-el_deg, 1.f, 0.f, 0.f);
  glRotatef(-az_deg, 0.f, 1.f, 0.f);
  glTranslatef(-cam.look_at[0], -cam.look_at[1], -cam.look_at[2]);
}

struct AppState {
  ControllerState* ctrl = nullptr;
  OrbitCamera* camera = nullptr;
};

// Poll mouse for camera orbit (so we don't replace ImGui's callbacks; ImGui needs them for buttons)
void update_camera_from_mouse(GLFWwindow* window, OrbitCamera& cam) {
  if (ImGui::GetIO().WantCaptureMouse)
    return;
  double xpos, ypos;
  glfwGetCursorPos(window, &xpos, &ypos);
  int left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT);
  if (left == GLFW_PRESS) {
    double dx = xpos - cam.last_x;
    double dy = ypos - cam.last_y;
    cam.azimuth -= static_cast<float>(dx * cam.sensitivity);
    cam.elevation += static_cast<float>(dy * cam.sensitivity);
    if (cam.elevation > MAX_ELEVATION)
      cam.elevation = MAX_ELEVATION;
    if (cam.elevation < -MAX_ELEVATION)
      cam.elevation = -MAX_ELEVATION;
  }
  cam.last_x = xpos;
  cam.last_y = ypos;
}

// Ray from camera through pixel (for R-drag body pick)
void get_ray_from_pixel(const OrbitCamera& cam, double mouse_x, double mouse_y,
                        int fb_width, int fb_height,
                        JPH::Vec3& ray_origin, JPH::Vec3& ray_dir) {
  float az = cam.azimuth;
  float el = cam.elevation;
  float d = cam.distance;
  JPH::Vec3 look_at(cam.look_at[0], cam.look_at[1], cam.look_at[2]);
  JPH::Vec3 forward(std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el));
  ray_origin = look_at - d * forward;
  JPH::Vec3 right(std::cos(az), 0.f, -std::sin(az));
  JPH::Vec3 up = right.Cross(forward).Normalized();
  float aspect = (fb_width > 0 && fb_height > 0) ? static_cast<float>(fb_width) / static_cast<float>(fb_height) : 1.f;
  float tan_half_fov = std::tan(25.f * 3.14159265f / 180.f);
  float ndc_x = (2.f * static_cast<float>(mouse_x) / static_cast<float>(fb_width) - 1.f);
  float ndc_y = (1.f - 2.f * static_cast<float>(mouse_y) / static_cast<float>(fb_height));
  ray_dir = (forward + ndc_x * right * (tan_half_fov * aspect) + ndc_y * up * tan_half_fov).Normalized();
}

// Chain scroll: give ImGui first, then zoom (so both work)
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
  ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
  AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(window));
  if (!app || !app->camera)
    return;
  OrbitCamera& cam = *app->camera;
  float scale = 1.f - static_cast<float>(yoffset) * cam.zoom_sensitivity;
  if (scale < 0.5f)
    scale = 0.5f;
  if (scale > 2.f)
    scale = 2.f;
  cam.distance *= scale;
  if (cam.distance < MIN_DISTANCE)
    cam.distance = MIN_DISTANCE;
  if (cam.distance > MAX_DISTANCE)
    cam.distance = MAX_DISTANCE;
}

}  // namespace

void run_demo_visual(const SimulatorConfig& config, int http_port) {
  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW init failed\n");
    return;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  char window_title[80];
  std::snprintf(window_title, sizeof(window_title), "Biomechanics Simulator (build %d)",
#ifdef BIOMECHANICS_BUILD_NUMBER
                BIOMECHANICS_BUILD_NUMBER
#else
                0
#endif
  );
  GLFWwindow* window = glfwCreateWindow(config.window_width, config.window_height,
                                        window_title, nullptr, nullptr);
  if (!window) {
    std::fprintf(stderr, "GLFW CreateWindow failed\n");
    glfwTerminate();
    return;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();
  int fb_width, fb_height;
  glfwGetFramebufferSize(window, &fb_width, &fb_height);
  glViewport(0, 0, fb_width, fb_height);

  SimulatorScene scene;
  create_simulator_scene(config, scene);
  if (!scene.physics) {
    std::fprintf(stderr, "create_simulator_scene failed (physics is null)\n");
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return;
  }
  zero_ragdoll_velocities(scene.ragdoll, scene.physics->GetBodyInterface());

  OpenGLDebugDrawer debug_drawer;

  glEnable(GL_DEPTH_TEST);
  glClearColor(0.2f, 0.2f, 0.25f, 1.f);

  ControllerState ctrl_state;
  int dm = config.default_motion_mode;
  ctrl_state.mode = (dm >= 0 && dm <= 2) ? static_cast<MotionMode>(dm) : MotionMode::Standing;

  OrbitCamera camera;

  AppState app_state;
  app_state.ctrl = &ctrl_state;
  app_state.camera = &camera;
  glfwSetWindowUserPointer(window, &app_state);
  glfwSetScrollCallback(window, scroll_callback);
  glfwGetCursorPos(window, &camera.last_x, &camera.last_y);

  clear_log(true);
  log("App started. Stance: Standing / Walk / Ragdoll. Log file: .cursor/debug.log");
  if (http_port > 0) {
    http_control_start(http_port);
    log("HTTP control: GET http://127.0.0.1:%d/status  PATCH http://127.0.0.1:%d/stance  PATCH http://127.0.0.1:%d/position", http_port, http_port, http_port);
  }

  int frame_count = 0;
  MotionMode last_logged_mode = ctrl_state.mode;
  double test_float_until = 0.0;
  JPH::BodyID drag_body;
  JPH::Vec3 drag_hit_world(0.f, 0.f, 0.f);
  int middle_was_down = 0;
  float standing_anchor_x = 0.f, standing_anchor_z = 0.f;
  JPH::Quat standing_anchor_rot = JPH::Quat::sIdentity();
  bool standing_anchor_valid = false;
  float walk_anchor_x = 0.f, walk_anchor_z = 0.f;
  bool walk_anchor_valid = false;
  bool simulation_frozen = false;
  MotionMode last_frame_mode = ctrl_state.mode;
  int standing_settle_frames = 0;
  constexpr int STANDING_SETTLE_FRAMES_BEFORE_CAPTURE = 90;  // ~1.5 s at 60 fps
  bool initial_standing_captured = false;

  glfwSetKeyCallback(window, [](GLFWwindow* win, int key, int /*scancode*/, int action, int /*mods*/) {
    if (action != GLFW_PRESS)
      return;
    AppState* app = static_cast<AppState*>(glfwGetWindowUserPointer(win));
    if (!app || !app->ctrl)
      return;
    if (key == GLFW_KEY_1)
      app->ctrl->mode = MotionMode::Standing;
    else if (key == GLFW_KEY_2)
      app->ctrl->mode = MotionMode::Walking;
    else if (key == GLFW_KEY_3)
      app->ctrl->mode = MotionMode::Ragdoll;
    else if (key == GLFW_KEY_4)
      app->ctrl->mode = MotionMode::StandingRaiseLeg;
    else if (key == GLFW_KEY_SPACE)
      app->ctrl->jump_triggered = true;
  });

  while (!glfwWindowShouldClose(window)) {
    if (!scene.physics) {
      log("[Visualizer] scene.physics is null; exiting loop");
      break;
    }
    JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
    glfwPollEvents();

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    update_camera_from_mouse(window, camera);

    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(200, 0), ImGuiCond_FirstUseEver);
    ImGui::Begin("Stance");
    if (ImGui::Button("Standing", ImVec2(160, 0)))
      ctrl_state.mode = MotionMode::Standing;
    if (ImGui::Button("Raise leg", ImVec2(160, 0)))
      ctrl_state.mode = MotionMode::StandingRaiseLeg;
    if (ImGui::Button("Walk", ImVec2(160, 0)))
      ctrl_state.mode = MotionMode::Walking;
    if (ImGui::Button("Ragdoll", ImVec2(160, 0)))
      ctrl_state.mode = MotionMode::Ragdoll;
    if (ImGui::Button("Jump", ImVec2(160, 0)))
      ctrl_state.jump_triggered = true;
    if (ImGui::Button("Test (float 2s)", ImVec2(160, 0))) {
      test_float_until = glfwGetTime() + 2.0;
      ctrl_state.mode = MotionMode::Ragdoll;
      log("Test: applying upward force 2s - character should RISE");
    }
    if (ImGui::Button("Reset", ImVec2(160, 0))) {
      test_float_until = 0.0;
      drag_body = JPH::BodyID();
      if (scene.physics && scene.ragdoll && scene.ragdoll_settings) {
        JPH::BodyInterface& bi = scene.physics->GetBodyInterface();
        reset_ragdoll_to_initial_standing(scene, &bi);
        ctrl_state.mode = MotionMode::Standing;
        ctrl_state.walk_phase = 0.f;
        ctrl_state.walk_time = 0.f;
        ctrl_state.jump_triggered = false;
        ctrl_state.jump_frames_hold = 0;
        standing_anchor_valid = false;
      }
    }
    if (ImGui::Button(simulation_frozen ? "Unfreeze" : "Freeze", ImVec2(160, 0))) {
      simulation_frozen = !simulation_frozen;
      log(simulation_frozen ? "[UI] Simulation frozen" : "[UI] Simulation unfrozen");
    }
    ImGui::Text("Camera: L-drag orbit, scroll zoom");
    ImGui::Text("Ragdoll: R-drag to pull body");
    if (http_port > 0)
      ImGui::TextColored(ImVec4(0.4f, 1.f, 0.4f, 1.f), "HTTP: 127.0.0.1:%d", http_port);
    ImGui::End();

    int right_down = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT);
    if (ctrl_state.mode == MotionMode::Ragdoll && !ImGui::GetIO().WantCaptureMouse) {
      if (right_down == GLFW_PRESS && middle_was_down != GLFW_PRESS) {
        double mx, my;
        glfwGetCursorPos(window, &mx, &my);
        int win_w, win_h;
        glfwGetWindowSize(window, &win_w, &win_h);
        double fb_mx = (win_w > 0) ? (mx * static_cast<double>(fb_width) / static_cast<double>(win_w)) : mx;
        double fb_my = (win_h > 0) ? (my * static_cast<double>(fb_height) / static_cast<double>(win_h)) : my;
        JPH::Vec3 ray_origin, ray_dir;
        get_ray_from_pixel(camera, fb_mx, fb_my, fb_width, fb_height, ray_origin, ray_dir);
        JPH::RRayCast ray(JPH::RVec3(ray_origin), ray_dir * 1000.f);
        JPH::RayCastResult result;
        result.mFraction = 1.f;
        JPH::IgnoreSingleBodyFilter skip_ground(scene.ground_id);
        bool hit = scene.physics->GetNarrowPhaseQuery().CastRay(ray, result, {}, {}, skip_ground);
        if (hit && !result.mBodyID.IsInvalid() && scene.ragdoll) {
          bool is_ragdoll = false;
          for (JPH::BodyID id : scene.ragdoll->GetBodyIDs()) {
            if (id == result.mBodyID) {
              is_ragdoll = true;
              break;
            }
          }
          if (is_ragdoll) {
            drag_body = result.mBodyID;
            drag_hit_world = JPH::Vec3(ray.GetPointOnRay(result.mFraction));
            log("[R-drag] Picked body at %.2f, %.2f, %.2f", drag_hit_world.GetX(), drag_hit_world.GetY(), drag_hit_world.GetZ());
          }
        }
        if (drag_body.IsInvalid() && scene.ragdoll && !scene.ragdoll->GetBodyIDs().empty()) {
          float best_dist = 2.5f;
          for (JPH::BodyID id : scene.ragdoll->GetBodyIDs()) {
            if (id.IsInvalid()) continue;
            JPH::Vec3 c = JPH::Vec3(bi.GetPosition(id));
            float t = (c - ray_origin).Dot(ray_dir);
            if (t < 0.01f) continue;
            JPH::Vec3 on_ray = ray_origin + ray_dir * t;
            float d = (on_ray - c).Length();
            if (d < best_dist) {
              best_dist = d;
              drag_body = id;
              drag_hit_world = c;
            }
          }
          if (!drag_body.IsInvalid())
            log("[R-drag] Picked by proximity (dist=%.2f) at %.2f, %.2f, %.2f", best_dist, drag_hit_world.GetX(), drag_hit_world.GetY(), drag_hit_world.GetZ());
        }
        if (drag_body.IsInvalid())
          log("[R-drag] No ragdoll hit; right-click near character");
      } else if (right_down != GLFW_PRESS) {
        drag_body = JPH::BodyID();
      }
    } else {
      drag_body = JPH::BodyID();
    }
    middle_was_down = right_down;

    if (ctrl_state.mode != last_logged_mode) {
      last_logged_mode = ctrl_state.mode;
      if (ctrl_state.mode == MotionMode::Standing)
        log("[UI] Mode -> Standing");
      else if (ctrl_state.mode == MotionMode::StandingRaiseLeg)
        log("[UI] Mode -> Raise leg");
      else if (ctrl_state.mode == MotionMode::Walking)
        log("[UI] Mode -> Walking");
      else
        log("[UI] Mode -> Ragdoll");
    }
    // Human ragdoll body indices: 0=LowerBody, 1=MidBody, 2=UpperBody, 3=Head, 4-7=arms, 8-11=legs
    const size_t num_bodies = scene.ragdoll ? scene.ragdoll->GetBodyIDs().size() : 0;
    bool using_walk_anim = (scene.walking_anim.GetPtr() != nullptr);
    if (ctrl_state.mode == MotionMode::Walking && last_frame_mode != MotionMode::Walking && num_bodies >= 1) {
      JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
      if (!root_id.IsInvalid()) {
        JPH::RVec3 rp = bi.GetPosition(root_id);
        JPH::Vec3 rv = bi.GetLinearVelocity(root_id);
        log("[Walk] Entered: root (x,z)=(%.3f,%.3f) y=%.3f vel=(%.3f,%.3f,%.3f) walk_forward_speed=%.2f walk_speed=%.2f using_anim=%d",
            static_cast<double>(rp.GetX()), static_cast<double>(rp.GetZ()), static_cast<double>(rp.GetY()),
            static_cast<double>(rv.GetX()), static_cast<double>(rv.GetY()), static_cast<double>(rv.GetZ()),
            config.walk_forward_speed, config.walk_speed, using_walk_anim ? 1 : 0);
      }
    }
    if (ctrl_state.mode == MotionMode::Walking && frame_count % 30 == 0 && num_bodies >= 12) {
      JPH::BodyID pid = scene.ragdoll->GetBodyID(0);
      JPH::BodyID lid = scene.ragdoll->GetBodyID(10);
      JPH::BodyID rid = scene.ragdoll->GetBodyID(11);
      if (!pid.IsInvalid() && !lid.IsInvalid() && !rid.IsInvalid()) {
        float px = float(bi.GetPosition(pid).GetX());
        float py = float(bi.GetPosition(pid).GetY());
        float pz = float(bi.GetPosition(pid).GetZ());
        JPH::Vec3 rv = bi.GetLinearVelocity(pid);
        float ly = float(bi.GetPosition(lid).GetY());
        float ry = float(bi.GetPosition(rid).GetY());
        log("[Walk] phase=%.2f time=%.2f root(%.2f,%.2f,%.2f) v(%.2f,%.2f,%.2f) pelvis_y=%.2f L_foot_y=%.2f R_foot_y=%.2f %s",
            ctrl_state.walk_phase, ctrl_state.walk_time, px, py, pz,
            static_cast<double>(rv.GetX()), static_cast<double>(rv.GetY()), static_cast<double>(rv.GetZ()),
            py, ly, ry, using_walk_anim ? "anim" : "proc");
      }
    }
    if (frame_count % 60 == 0 && num_bodies >= 12) {
      auto pos = [&](int idx) { return bi.GetPosition(scene.ragdoll->GetBodyID(idx)); };
      JPH::RVec3 p = pos(0), s = pos(1), h = pos(3);
      JPH::RVec3 lf = pos(10), rf = pos(11);
      JPH::RVec3 lhand = pos(6), rhand = pos(7);
      log("[Limbs] pelvis %.2f,%.2f,%.2f spine %.2f,%.2f,%.2f head %.2f,%.2f,%.2f L_foot %.2f,%.2f,%.2f R_foot %.2f,%.2f,%.2f L_hand %.2f,%.2f,%.2f R_hand %.2f,%.2f,%.2f",
          float(p.GetX()), float(p.GetY()), float(p.GetZ()), float(s.GetX()), float(s.GetY()), float(s.GetZ()),
          float(h.GetX()), float(h.GetY()), float(h.GetZ()), float(lf.GetX()), float(lf.GetY()), float(lf.GetZ()),
          float(rf.GetX()), float(rf.GetY()), float(rf.GetZ()), float(lhand.GetX()), float(lhand.GetY()), float(lhand.GetZ()),
          float(rhand.GetX()), float(rhand.GetY()), float(rhand.GetZ()));
    }
    if (ctrl_state.mode == MotionMode::Standing && num_bodies >= 2 && frame_count % 30 == 0) {
      JPH::BodyID mid_id = scene.ragdoll->GetBodyID(1);
      if (!mid_id.IsInvalid()) {
        JPH::RVec3 mid_pos = bi.GetPosition(mid_id);
        JPH::Vec3 mid_vel = bi.GetLinearVelocity(mid_id);
        log("[Standing] MidBody pos %.4f,%.4f,%.4f vel %.4f,%.4f,%.4f (only root is pinned; MidBody can wobble)",
            static_cast<double>(mid_pos.GetX()), static_cast<double>(mid_pos.GetY()), static_cast<double>(mid_pos.GetZ()),
            static_cast<double>(mid_vel.GetX()), static_cast<double>(mid_vel.GetY()), static_cast<double>(mid_vel.GetZ()));
      }
    }

    ImGui::SetNextWindowPos(ImVec2(220, 10), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(400, 220), ImGuiCond_FirstUseEver);
    ImGui::Begin("Log");
    if (ImGui::Button("Clear"))
      clear_log(false);
    ImGui::SameLine();
    ImGui::Text("(saved: %s)", get_log_path());
    ImGui::BeginChild("LogScroll", ImVec2(0, -4), true);
    for (const std::string& line : get_log_lines())
      ImGui::TextUnformatted(line.c_str());
    ImGui::SetScrollHereY(1.f);
    ImGui::EndChild();
    ImGui::End();

    ImGui::SetNextWindowPos(ImVec2(220, 240), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(340, 320), ImGuiCond_FirstUseEver);
    ImGui::Begin("Limb positions");
    if (scene.ragdoll && scene.ragdoll->GetBodyIDs().size() >= 12) {
      const char* labels[] = {"LowerBody", "MidBody", "Head",
                              "L upper leg", "L lower leg", "R upper leg", "R lower leg",
                              "L upper arm", "L lower arm", "R upper arm", "R lower arm"};
      int indices[] = {0, 1, 3, 8, 10, 9, 11, 4, 6, 5, 7};
      std::ostringstream copy_buf;
      for (int i = 0; i < 11; ++i) {
        JPH::BodyID id = scene.ragdoll->GetBodyID(indices[i]);
        if (!id.IsInvalid()) {
          JPH::RVec3 pos = bi.GetPosition(id);
          double px = static_cast<double>(pos.GetX()), py = static_cast<double>(pos.GetY()), pz = static_cast<double>(pos.GetZ());
          ImGui::Text("%s: %.2f, %.2f, %.2f", labels[i], px, py, pz);
          copy_buf << labels[i] << ": " << px << ", " << py << ", " << pz << "\n";
        }
      }
      if (ImGui::Button("Copy")) {
        std::string text = copy_buf.str();
        if (!text.empty())
          glfwSetClipboardString(window, text.c_str());
      }
      if (!drag_body.IsInvalid()) ImGui::TextColored(ImVec4(0, 1, 0, 1), "R-drag active");
    }
    ImGui::End();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glfwGetFramebufferSize(window, &fb_width, &fb_height);
    glViewport(0, 0, fb_width, fb_height);
    setup_projection(fb_width, fb_height);
    setup_camera(camera);

    const int sub_steps = 4;  // more steps per frame = more PD corrections, stabler standing
    const float sub_dt = static_cast<float>(config.time_step) / static_cast<float>(sub_steps);
    if (!simulation_frozen) {
      if (http_port > 0) {
        http_control_apply_pending_stance(ctrl_state);
        // Ragdoll -> Standing (UI or HTTP): reset to initial standing pose and set anchor from stored pose
        if (last_frame_mode == MotionMode::Ragdoll && ctrl_state.mode == MotionMode::Standing &&
            scene.physics && scene.ragdoll && scene.ragdoll_settings) {
          reset_ragdoll_to_initial_standing(scene, &bi);
          ctrl_state.walk_phase = 0.f;
          ctrl_state.walk_time = 0.f;
          standing_anchor_x = static_cast<float>(scene.initial_standing_root_offset.GetX());
          standing_anchor_z = static_cast<float>(scene.initial_standing_root_offset.GetZ());
          standing_anchor_rot = scene.initial_standing_root_rotation;
          standing_anchor_valid = true;
          log("Ragdoll -> Standing: reset to initial pose (same limb positions as start)");
        }
        double patch_x, patch_y, patch_z;
        if (http_control_consume_pending_position(patch_x, patch_y, patch_z) && scene.ragdoll) {
          JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
          if (!root_id.IsInvalid()) {
            JPH::Quat rot = bi.GetRotation(root_id);
            bi.SetPositionAndRotation(root_id, JPH::RVec3(patch_x, patch_y, patch_z), rot, JPH::EActivation::Activate);
            bi.SetLinearVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
            bi.SetAngularVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
            standing_anchor_x = static_cast<float>(patch_x);
            standing_anchor_z = static_cast<float>(patch_z);
            standing_anchor_rot = rot;
            standing_anchor_valid = true;
            log("Position patched: root %.3f, %.3f, %.3f", patch_x, patch_y, patch_z);
          }
        }
        if (http_control_consume_pending_jump())
          ctrl_state.jump_triggered = true;
        if (http_control_consume_pending_test_float()) {
          test_float_until = glfwGetTime() + 2.0;
          ctrl_state.mode = MotionMode::Ragdoll;
          log("Test: applying upward force 2s - character should RISE");
        }
        if (http_control_consume_pending_reset()) {
          test_float_until = 0.0;
          drag_body = JPH::BodyID();
          if (scene.physics && scene.ragdoll && scene.ragdoll_settings) {
            reset_ragdoll_to_initial_standing(scene, &bi);
            ctrl_state.mode = MotionMode::Standing;
            ctrl_state.walk_phase = 0.f;
            ctrl_state.walk_time = 0.f;
            ctrl_state.jump_triggered = false;
            ctrl_state.jump_frames_hold = 0;
            standing_anchor_valid = false;
          }
        }
      }
      for (int s = 0; s < sub_steps; ++s) {
        apply_pose_control(scene, ctrl_state, config, sub_dt);
        // #region agent log
        if (ctrl_state.mode == MotionMode::Walking && scene.ragdoll && s == 0 && frame_count % 15 == 0) {
          JPH::BodyID rid = scene.ragdoll->GetBodyID(0);
          if (!rid.IsInvalid()) {
            JPH::RVec3 rp = bi.GetPosition(rid);
            JPH::Vec3 rv = bi.GetLinearVelocity(rid);
            debug_instrument_walk("Visualizer.cpp:after_drive", "A,E", "after_drive",
              float(rv.GetX()), float(rv.GetY()), float(rv.GetZ()),
              float(rp.GetX()), float(rp.GetY()), float(rp.GetZ()), sub_dt);
          }
        }
        // #endregion
        // Walking: set root velocity BEFORE physics so the step integrates intended motion (not MoveKinematic’s delta/sub_dt)
        if (ctrl_state.mode == MotionMode::Walking && scene.ragdoll) {
          JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
          if (!root_id.IsInvalid()) {
            JPH::RVec3 pos = bi.GetPosition(root_id);
            JPH::Quat rot = bi.GetRotation(root_id);
            if (!walk_anchor_valid) {
              walk_anchor_x = static_cast<float>(pos.GetX());
              walk_anchor_z = static_cast<float>(pos.GetZ());
              walk_anchor_valid = true;
            }
            JPH::Vec3 forward = rot * JPH::Vec3(0.f, 0.f, -1.f);
            float speed = config.walk_forward_speed;
            walk_anchor_x += forward.GetX() * speed * sub_dt;
            walk_anchor_z += forward.GetZ() * speed * sub_dt;
            bi.SetAngularVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
            bi.SetLinearVelocity(root_id, JPH::Vec3(forward.GetX() * speed, 0.f, forward.GetZ() * speed));
          }
        }
        if (test_float_until > 0 && glfwGetTime() < test_float_until && scene.ragdoll) {
          JPH::BodyID pid = scene.ragdoll->GetBodyID(0);
          if (!pid.IsInvalid())
            bi.AddForce(pid, JPH::Vec3(0, 450, 0));  // upward force so character rises
        }
        if (!drag_body.IsInvalid()) {
          double mx, my;
          glfwGetCursorPos(window, &mx, &my);
          int win_w, win_h;
          glfwGetWindowSize(window, &win_w, &win_h);
          double fb_mx = (win_w > 0) ? (mx * static_cast<double>(fb_width) / static_cast<double>(win_w)) : mx;
          double fb_my = (win_h > 0) ? (my * static_cast<double>(fb_height) / static_cast<double>(win_h)) : my;
          JPH::Vec3 ray_origin, ray_dir;
          get_ray_from_pixel(camera, fb_mx, fb_my, fb_width, fb_height, ray_origin, ray_dir);
          float t = (drag_hit_world - ray_origin).Dot(ray_dir);
          if (t > 0.01f) {
            JPH::Vec3 target = ray_origin + ray_dir * t;
            JPH::Vec3 body_pos = JPH::Vec3(bi.GetPosition(drag_body));
            JPH::Vec3 to_target = target - body_pos;
            JPH::Vec3 vel = bi.GetLinearVelocity(drag_body);
            JPH::Vec3 force = to_target * 200.f - vel * 20.f;
            float mag = force.Length();
            if (mag > 500.f && mag > 1e-5f) force *= 500.f / mag;
            bi.AddForce(drag_body, force);
          }
        }
        scene.physics->Update(sub_dt, 1, scene.temp_allocator, scene.job_system);
        // #region agent log
        if (ctrl_state.mode == MotionMode::Walking && scene.ragdoll && s == 0 && frame_count % 15 == 0) {
          JPH::BodyID rid = scene.ragdoll->GetBodyID(0);
          if (!rid.IsInvalid()) {
            JPH::RVec3 rp = bi.GetPosition(rid);
            JPH::Vec3 rv = bi.GetLinearVelocity(rid);
            debug_instrument_walk("Visualizer.cpp:after_physics", "A,E", "after_physics",
              float(rv.GetX()), float(rv.GetY()), float(rv.GetZ()),
              float(rp.GetX()), float(rp.GetY()), float(rp.GetZ()), sub_dt);
          }
        }
        // #endregion
        // Anchor root XZ and rotation, zero root linear/angular velocity (Standing and Raise leg: same pinning)
        if ((ctrl_state.mode == MotionMode::Standing || ctrl_state.mode == MotionMode::StandingRaiseLeg) && scene.ragdoll) {
          JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
          if (!root_id.IsInvalid()) {
            bi.SetMotionType(root_id, JPH::EMotionType::Kinematic, JPH::EActivation::DontActivate);
            JPH::RVec3 pos = bi.GetPosition(root_id);
            JPH::Quat rot = bi.GetRotation(root_id);
            if (!standing_anchor_valid) {
              standing_anchor_x = static_cast<float>(pos.GetX());
              standing_anchor_z = static_cast<float>(pos.GetZ());
              standing_anchor_rot = rot;
              standing_anchor_valid = true;
              log("[Standing] Anchor set root_x=%.3f root_z=%.3f root_y=%.3f (entering/re-entering Standing)", standing_anchor_x, standing_anchor_z, static_cast<double>(pos.GetY()));
            }
            JPH::RVec3 anchored_pos(standing_anchor_x, pos.GetY(), standing_anchor_z);
            bi.SetPositionAndRotation(root_id, anchored_pos, standing_anchor_rot, JPH::EActivation::DontActivate);
            bi.SetLinearVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
            bi.SetAngularVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
          }
          standing_anchor_valid = true;
          walk_anchor_valid = false;
        } else if (ctrl_state.mode == MotionMode::Walking && scene.ragdoll) {
          // Walking: after physics, pin root XZ to walk anchor and zero velocities (solver cannot drift root)
          JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
          if (!root_id.IsInvalid()) {
            bi.SetMotionType(root_id, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
            JPH::RVec3 pos = bi.GetPosition(root_id);
            JPH::Quat rot = bi.GetRotation(root_id);
            JPH::RVec3 pinned_pos(walk_anchor_x, pos.GetY(), walk_anchor_z);
            bi.SetPositionAndRotation(root_id, pinned_pos, rot, JPH::EActivation::Activate);
            bi.SetLinearVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
            bi.SetAngularVelocity(root_id, JPH::Vec3(0.f, 0.f, 0.f));
          }
        } else {
          if (scene.ragdoll) {
            JPH::BodyID root_id = scene.ragdoll->GetBodyID(0);
            if (!root_id.IsInvalid())
              bi.SetMotionType(root_id, JPH::EMotionType::Dynamic, JPH::EActivation::Activate);
          }
          standing_anchor_valid = false;
          walk_anchor_valid = false;
        }
      }
      clamp_ragdoll_velocities(scene.ragdoll, bi);
      if (ctrl_state.mode == MotionMode::Standing && !simulation_frozen) {
        standing_settle_frames++;
        if (!initial_standing_captured && standing_settle_frames >= STANDING_SETTLE_FRAMES_BEFORE_CAPTURE) {
          capture_standing_pose_as_initial(scene);
          initial_standing_captured = true;
          log("Initial standing pose captured (settled) for reset");
        }
      } else {
        standing_settle_frames = 0;
      }
      if (http_port > 0)
        http_control_update_snapshot(scene, ctrl_state);
      last_frame_mode = ctrl_state.mode;
    }

    frame_count++;

    glBegin(GL_LINES);
    debug_drawer.draw_bodies(scene.physics, scene.ragdoll, scene.ground_id);
    glEnd();

    glDisable(GL_DEPTH_TEST);
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glEnable(GL_DEPTH_TEST);

    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  if (http_port > 0)
    http_control_stop();
  destroy_simulator_scene(scene);
  glfwDestroyWindow(window);
  glfwTerminate();
}

}  // namespace biomechanics
