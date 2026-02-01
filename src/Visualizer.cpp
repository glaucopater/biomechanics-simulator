#include "biomechanics/Config.hpp"
#include "biomechanics/Log.hpp"
#include "biomechanics/OpenGLDebugDrawer.hpp"
#include "biomechanics/PoseController.hpp"
#include "biomechanics/Ragdoll.hpp"
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
#include <cmath>
#include <cstdio>
#include <fstream>
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

// Ray from camera through pixel (for middle-mouse drag)
void get_ray_from_pixel(const OrbitCamera& cam, double mouse_x, double mouse_y,
                        int fb_width, int fb_height,
                        btVector3& ray_origin, btVector3& ray_dir) {
  float az = cam.azimuth;
  float el = cam.elevation;
  float d = cam.distance;
  btVector3 look_at(cam.look_at[0], cam.look_at[1], cam.look_at[2]);
  btVector3 forward(std::sin(az) * std::cos(el), std::sin(el), std::cos(az) * std::cos(el));
  ray_origin = look_at - d * forward;
  btVector3 right(std::cos(az), 0.f, -std::sin(az));
  btVector3 up = right.cross(forward).normalized();
  float aspect = (fb_width > 0 && fb_height > 0) ? static_cast<float>(fb_width) / static_cast<float>(fb_height) : 1.f;
  float tan_half_fov = std::tan(25.f * 3.14159265f / 180.f);
  float ndc_x = (2.f * static_cast<float>(mouse_x) / static_cast<float>(fb_width) - 1.f);
  float ndc_y = (1.f - 2.f * static_cast<float>(mouse_y) / static_cast<float>(fb_height));
  ray_dir = (forward + ndc_x * right * (tan_half_fov * aspect) + ndc_y * up * tan_half_fov).normalized();
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

void run_demo_visual(const SimulatorConfig& config) {
  if (!glfwInit()) {
    std::fprintf(stderr, "GLFW init failed\n");
    return;
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

  GLFWwindow* window = glfwCreateWindow(config.window_width, config.window_height,
                                        "Biomechanics Simulator", nullptr, nullptr);
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
  zero_ragdoll_velocities(scene.ragdoll);

  OpenGLDebugDrawer debug_drawer;
  debug_drawer.setDebugMode(btIDebugDraw::DBG_DrawWireframe);
  scene.world->setDebugDrawer(&debug_drawer);

  glEnable(GL_DEPTH_TEST);
  glClearColor(0.2f, 0.2f, 0.25f, 1.f);

  ControllerState ctrl_state;
  int dm = config.default_motion_mode;
  ctrl_state.mode = (dm >= 0 && dm <= 2) ? static_cast<MotionMode>(dm) : MotionMode::Standing;
  capture_rest_pose(scene.ragdoll, ctrl_state);

  OrbitCamera camera;

  AppState app_state;
  app_state.ctrl = &ctrl_state;
  app_state.camera = &camera;
  glfwSetWindowUserPointer(window, &app_state);
  glfwSetScrollCallback(window, scroll_callback);
  glfwGetCursorPos(window, &camera.last_x, &camera.last_y);

  clear_log(true);
  log("App started. Stance: Standing / Walk / Ragdoll. Log file: .cursor/debug.log");

  int frame_count = 0;
  MotionMode last_logged_mode = ctrl_state.mode;
  double test_float_until = 0.0;
  btRigidBody* drag_body = nullptr;
  btVector3 drag_hit_world(0.f, 0.f, 0.f);
  int middle_was_down = 0;

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
    else if (key == GLFW_KEY_SPACE)
      app->ctrl->jump_triggered = true;
  });

  while (!glfwWindowShouldClose(window)) {
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
      drag_body = nullptr;
      scene.world->setDebugDrawer(nullptr);
      destroy_simulator_scene(scene);
      create_simulator_scene(config, scene);
      zero_ragdoll_velocities(scene.ragdoll);
      scene.world->setDebugDrawer(&debug_drawer);
      ctrl_state.mode = MotionMode::Standing;
      ctrl_state.walk_phase = 0.f;
      ctrl_state.jump_triggered = false;
      ctrl_state.jump_frames_hold = 0;
      capture_rest_pose(scene.ragdoll, ctrl_state);
    }
    ImGui::Text("Camera: L-drag orbit, scroll zoom");
    ImGui::Text("Ragdoll: R-drag to pull body");
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
        btVector3 ray_origin, ray_dir;
        get_ray_from_pixel(camera, fb_mx, fb_my, fb_width, fb_height, ray_origin, ray_dir);
        btVector3 ray_to = ray_origin + ray_dir * 1000.f;
        btCollisionWorld::AllHitsRayResultCallback cb(ray_origin, ray_to);
        scene.world->rayTest(ray_origin, ray_to, cb);
        float best_fraction = 2.f;
        for (int i = 0; i < cb.m_collisionObjects.size(); ++i) {
          btRigidBody* hit = const_cast<btRigidBody*>(btRigidBody::upcast(cb.m_collisionObjects[i]));
          if (!hit || cb.m_hitFractions[i] >= best_fraction) continue;
          if (hit == scene.ground_body) continue;
          for (btRigidBody* b : scene.ragdoll.bodies) {
            if (b == hit) {
              best_fraction = cb.m_hitFractions[i];
              drag_body = hit;
              drag_hit_world = cb.m_hitPointWorld[i];
              break;
            }
          }
        }
        if (!drag_body && !scene.ragdoll.bodies.empty()) {
          float best_dist = 2.5f;
          for (btRigidBody* b : scene.ragdoll.bodies) {
            btVector3 c = b->getWorldTransform().getOrigin();
            float t = (c - ray_origin).dot(ray_dir);
            if (t < 0.01f) continue;
            btVector3 on_ray = ray_origin + ray_dir * t;
            float d = (on_ray - c).length();
            if (d < best_dist) {
              best_dist = d;
              drag_body = b;
              drag_hit_world = c;
            }
          }
          if (drag_body)
            log("[R-drag] Picked by proximity (dist=%.2f) at %.2f, %.2f, %.2f", best_dist, drag_hit_world.x(), drag_hit_world.y(), drag_hit_world.z());
        }
        if (drag_body && best_fraction < 2.f)
          log("[R-drag] Picked body at %.2f, %.2f, %.2f", drag_hit_world.x(), drag_hit_world.y(), drag_hit_world.z());
        else if (!drag_body)
          log("[R-drag] No ragdoll hit (%u hits); right-click near character", static_cast<unsigned>(cb.m_collisionObjects.size()));
      } else if (right_down != GLFW_PRESS) {
        drag_body = nullptr;
      }
    } else {
      drag_body = nullptr;
    }
    middle_was_down = right_down;

    if (ctrl_state.mode != last_logged_mode) {
      last_logged_mode = ctrl_state.mode;
      if (ctrl_state.mode == MotionMode::Standing)
        log("[UI] Mode -> Standing");
      else if (ctrl_state.mode == MotionMode::Walking)
        log("[UI] Mode -> Walking");
      else
        log("[UI] Mode -> Ragdoll");
    }
    if (ctrl_state.mode == MotionMode::Walking && frame_count % 60 == 0 && scene.ragdoll.bodies.size() >= static_cast<size_t>(BodyPart::RightLowerLeg) + 1u) {
      float py = scene.ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]->getWorldTransform().getOrigin().y();
      float ly = scene.ragdoll.bodies[static_cast<int>(BodyPart::LeftLowerLeg)]->getWorldTransform().getOrigin().y();
      float ry = scene.ragdoll.bodies[static_cast<int>(BodyPart::RightLowerLeg)]->getWorldTransform().getOrigin().y();
      log("[Walk] phase=%.2f pelvis_y=%.2f L_foot_y=%.2f R_foot_y=%.2f", ctrl_state.walk_phase, py, ly, ry);
    }
    if (frame_count % 60 == 0 && scene.ragdoll.bodies.size() >= static_cast<size_t>(BodyPart::RightLowerArm) + 1u) {
      btVector3 p = scene.ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]->getWorldTransform().getOrigin();
      btVector3 s = scene.ragdoll.bodies[static_cast<int>(BodyPart::Spine)]->getWorldTransform().getOrigin();
      btVector3 h = scene.ragdoll.bodies[static_cast<int>(BodyPart::Head)]->getWorldTransform().getOrigin();
      btVector3 lf = scene.ragdoll.bodies[static_cast<int>(BodyPart::LeftLowerLeg)]->getWorldTransform().getOrigin();
      btVector3 rf = scene.ragdoll.bodies[static_cast<int>(BodyPart::RightLowerLeg)]->getWorldTransform().getOrigin();
      btVector3 lhand = scene.ragdoll.bodies[static_cast<int>(BodyPart::LeftLowerArm)]->getWorldTransform().getOrigin();
      btVector3 rhand = scene.ragdoll.bodies[static_cast<int>(BodyPart::RightLowerArm)]->getWorldTransform().getOrigin();
      log("[Limbs] pelvis %.2f,%.2f,%.2f spine %.2f,%.2f,%.2f head %.2f,%.2f,%.2f L_foot %.2f,%.2f,%.2f R_foot %.2f,%.2f,%.2f L_hand %.2f,%.2f,%.2f R_hand %.2f,%.2f,%.2f",
          p.x(), p.y(), p.z(), s.x(), s.y(), s.z(), h.x(), h.y(), h.z(),
          lf.x(), lf.y(), lf.z(), rf.x(), rf.y(), rf.z(), lhand.x(), lhand.y(), lhand.z(), rhand.x(), rhand.y(), rhand.z());
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
    if (!scene.ragdoll.bodies.empty()) {
      const char* labels[] = {"Pelvis", "Spine", "Head",
                              "L upper leg", "L lower leg", "R upper leg", "R lower leg",
                              "L upper arm", "L lower arm", "R upper arm", "R lower arm"};
      int parts[] = {static_cast<int>(BodyPart::Pelvis), static_cast<int>(BodyPart::Spine),
                     static_cast<int>(BodyPart::Head),
                     static_cast<int>(BodyPart::LeftUpperLeg), static_cast<int>(BodyPart::LeftLowerLeg),
                     static_cast<int>(BodyPart::RightUpperLeg), static_cast<int>(BodyPart::RightLowerLeg),
                     static_cast<int>(BodyPart::LeftUpperArm), static_cast<int>(BodyPart::LeftLowerArm),
                     static_cast<int>(BodyPart::RightUpperArm), static_cast<int>(BodyPart::RightLowerArm)};
      for (int i = 0; i < 11; ++i) {
        if (parts[i] < static_cast<int>(scene.ragdoll.bodies.size())) {
          btVector3 p = scene.ragdoll.bodies[parts[i]]->getWorldTransform().getOrigin();
          ImGui::Text("%s: %.2f, %.2f, %.2f", labels[i], p.x(), p.y(), p.z());
        }
      }
      if (drag_body) ImGui::TextColored(ImVec4(0, 1, 0, 1), "R-drag active");
    }
    ImGui::End();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glfwGetFramebufferSize(window, &fb_width, &fb_height);
    glViewport(0, 0, fb_width, fb_height);
    setup_projection(fb_width, fb_height);
    setup_camera(camera);

    apply_pose_control(scene.world, scene.ragdoll, ctrl_state, config);
    if (test_float_until > 0 && glfwGetTime() < test_float_until && !scene.ragdoll.bodies.empty()) {
      scene.ragdoll.bodies[static_cast<int>(BodyPart::Pelvis)]->applyCentralForce(btVector3(0, 150, 0));
    }
    if (drag_body) {
      double mx, my;
      glfwGetCursorPos(window, &mx, &my);
      int win_w, win_h;
      glfwGetWindowSize(window, &win_w, &win_h);
      double fb_mx = (win_w > 0) ? (mx * static_cast<double>(fb_width) / static_cast<double>(win_w)) : mx;
      double fb_my = (win_h > 0) ? (my * static_cast<double>(fb_height) / static_cast<double>(win_h)) : my;
      btVector3 ray_origin, ray_dir;
      get_ray_from_pixel(camera, fb_mx, fb_my, fb_width, fb_height, ray_origin, ray_dir);
      float t = (drag_hit_world - ray_origin).dot(ray_dir);
      if (t > 0.01f) {
        btVector3 target = ray_origin + ray_dir * t;
        btVector3 body_pos = drag_body->getWorldTransform().getOrigin();
        btVector3 to_target = target - body_pos;
        btVector3 vel = drag_body->getLinearVelocity();
        btVector3 force = to_target * 200.f - vel * 20.f;
        float mag = force.length();
        if (mag > 500.f && mag > 1e-5f) force *= 500.f / mag;
        drag_body->applyCentralForce(force);
      }
    }
    scene.world->stepSimulation(static_cast<btScalar>(config.time_step), 1);
    clamp_ragdoll_velocities(scene.ragdoll);

    frame_count++;

    glBegin(GL_LINES);
    scene.world->debugDrawWorld();
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

  scene.world->setDebugDrawer(nullptr);
  destroy_simulator_scene(scene);
  glfwDestroyWindow(window);
  glfwTerminate();
}

}  // namespace biomechanics
