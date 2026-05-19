#include "driver_mqtt.hpp"
#include "driver_urdf_renderer.hpp"
#include "trajectory_planner.hpp"
#include "logger.hpp"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <mutex>

using json = nlohmann::json;

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }
float current_arm_jpos[9] = {};
struct OrbitCamera {
    double distance = 2.74;
    double yaw = -0.32;
    double pitch = -0.18;
    double look_at[3] = {0.0, 0.0, 0.5};
    double fov = 50.0f;
    bool dirty = false;
    std::mutex mtx;

    void apply(drivers::URDFRendererPlugin& renderer) {
        std::lock_guard<std::mutex> lock(mtx);
        if (!dirty) return;
        drivers::UrdfCameraConfig cam;
        cam.position[0] = look_at[0] + distance * cos(pitch) * sin(yaw);
        cam.position[1] = look_at[1] + distance * sin(pitch);
        cam.position[2] = look_at[2] + distance * cos(pitch) * cos(yaw);
        cam.look_at[0] = look_at[0];
        cam.look_at[1] = look_at[1];
        cam.look_at[2] = look_at[2];
        cam.up[0] = 0.0f; cam.up[1] = 1.0f; cam.up[2] = 0.0f;
        cam.fov_degrees = fov;
        cam.near_clip = 0.02f;
        cam.far_clip = 20.0f;
        renderer.setCamera(cam);
        dirty = false;
    }
    void orbit(double dx, double dy) {
        std::lock_guard<std::mutex> lock(mtx);
        yaw += dx * 0.005;
        pitch += dy * 0.005;
        pitch = std::max(-1.5, std::min(1.5, pitch));
        dirty = true;
    }
    void zoom(double delta) {
        std::lock_guard<std::mutex> lock(mtx);
        distance *= (1.0 + delta * 0.02);
        distance = std::max(0.5, std::min(20.0, distance));
        dirty = true;
    }
};

static void on_mouse(int event, int x, int y, int flags, void* userdata) {
    auto* cam = static_cast<OrbitCamera*>(userdata);
    static int prev_x = -1, prev_y = -1;
    if (event == cv::EVENT_LBUTTONDOWN) { prev_x = x; prev_y = y; }
    else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) {
        if (prev_x >= 0 && prev_y >= 0) {
            cam->orbit(x - prev_x, y - prev_y);
            prev_x = x; prev_y = y;
        }
    } else if (event == cv::EVENT_LBUTTONUP) { prev_x = prev_y = -1; }
    else if (event == cv::EVENT_MOUSEWHEEL) {
        cam->zoom(((flags >> 16) & 0xFFFF) > 0 ? 1.0 : -1.0);
    }
}

static void print_usage(const char* prog) {
    fprintf(stderr,
        "Usage: %s [options]\n"
        "Options:\n"
        "  --config PATH       Path to JSON config file (default: config/client_setting.json)\n"
        "  --mqtt-host HOST    MQTT broker host (overrides config)\n"
        "  --mqtt-port PORT    MQTT broker port (overrides config)\n"
        "  --urdf-path PATH    Path to URDF file (overrides config)\n"
        "  --help              Show this help\n",
        prog);
}

int main(int argc, char** argv) {
    logger::Logger::GetInstance().Init();
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto src = std::filesystem::path(PROJECT_SOURCE_DIR);
    std::string config_path = (src / "config" / "client_setting.json").string();
    std::string mqtt_host = "127.0.0.1";
    int mqtt_port = 3333;
    std::string mqtt_client_id = "3";
    std::string urdf_path;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (arg == "--mqtt-host" && i + 1 < argc) mqtt_host = argv[++i];
        else if (arg == "--mqtt-port" && i + 1 < argc) mqtt_port = std::stoi(argv[++i]);
        else if (arg == "--urdf-path" && i + 1 < argc) urdf_path = argv[++i];
        else if (arg == "--help") { print_usage(argv[0]); return 0; }
    }

    if (std::filesystem::exists(config_path)) {
        try {
            std::ifstream f(config_path);
            json j; f >> j;
            mqtt_host  = j.value("mqtt_host",  mqtt_host);
            mqtt_port  = j.value("mqtt_port",  mqtt_port);
            mqtt_client_id = j.value("mqtt_client_id", j.value("default_mqtt_client_id", mqtt_client_id));
            if (urdf_path.empty()) urdf_path = j.value("urdf_path", urdf_path);
            LOG_INFO("Loaded config from {}", config_path);
        } catch (const std::exception& e) {
            LOG_WARN("Failed to parse config: {}", e.what());
        }
    }

    if (urdf_path.empty()) urdf_path = "arm_description/urdf/Engineer2.urdf";
    if (!std::filesystem::path(urdf_path).is_absolute())
        urdf_path = (src / urdf_path).string();

    if (!std::filesystem::exists(urdf_path)) {
        LOG_ERROR("URDF file not found: {}", urdf_path);
        return 1;
    }

    drivers::URDFRendererPlugin renderer;
    drivers::UrdfRenderConfig render_cfg;
    render_cfg.width = 800;
    render_cfg.height = 600;
    render_cfg.transparent_background = true;
    render_cfg.anti_aliasing = 0;

    if (!renderer.initialize(&render_cfg)) {
        LOG_ERROR("Renderer init failed: {}", renderer.getLastError());
        return 1;
    }
    if (renderer.loadURDF(urdf_path) != drivers::URDF_SUCCESS) {
        LOG_ERROR("URDF load failed: {}", renderer.getLastError());
        return 1;
    }
    LOG_INFO("URDF loaded: {} ({} joints)", urdf_path, renderer.getJointCount());

    OrbitCamera orbit_cam;
    orbit_cam.dirty = true;
    orbit_cam.apply(renderer);

    URDFMotionPlanner planner;
    if (!planner.LoadURDF(urdf_path)) {
        LOG_ERROR("Planner init failed");
        return 1;
    }
    LOG_INFO("URDFMotionPlanner: {} joints", planner.num_joints());

    auto joint_names = planner.getJointNames();
    size_t num_joints = joint_names.size();

    TrajectoryExecutor executor;
    std::mutex traj_mutex;
    bool trajectory_active = false;
    std::vector<double> traj_current_positions(num_joints, 0.0);

    drivers::MqttClient mqtt_client(mqtt_host, mqtt_port, mqtt_client_id);
    if (!mqtt_client.Connect()) {
        LOG_ERROR("MQTT connection failed, exiting");
        return 1;
    }

    mqtt_client.SetCustomByteBlockHandler(
        [&](const std::vector<uint8_t>& data) {
            if (data.size() < 3) return;

            std::string_view header(reinterpret_cast<const char*>(data.data()), 3);
            const uint8_t* payload = data.data() + 3;
            size_t payload_size = data.size() - 3;

            if (header == "aaa") {
                size_t num_floats = payload_size / sizeof(float);
                if (num_floats < 1) return;
                std::vector<double> target_joints(num_floats);
                const float* fdata = reinterpret_cast<const float*>(payload);
                for (size_t i = 0; i < num_floats; ++i) target_joints[i] = fdata[i];

                LOG_INFO("CustomByteBlock[aaa]: planning to {} joint targets", num_floats);

                std::vector<double> current_pos(num_joints, 0.0);
                {
                    std::lock_guard<std::mutex> lock(traj_mutex);
                    current_pos = traj_current_positions;
                }

                std::vector<double> plan_target(num_joints, 0.0);
                for (size_t i = 0; i < std::min(num_floats, num_joints); ++i) {
                    plan_target[i] = target_joints[i];
                }
                JointTrajectory traj = planner.PlanToTarget(current_pos, plan_target, 3.0);
                if (!traj.points.empty()) {
                    std::lock_guard<std::mutex> lock(traj_mutex);
                    executor.Start(traj);
                    trajectory_active = true;
                    LOG_INFO("Trajectory started: {} points", traj.points.size());
                }
            } else if (header == "bbb") {
                if (payload_size < 6 * sizeof(float)) return;
                const float* fdata = reinterpret_cast<const float*>(payload);
                double target_xyzrpy[6] = {
                    fdata[0], fdata[1], fdata[2], fdata[3], fdata[4], fdata[5]
                };

                LOG_INFO("CustomByteBlock[bbb]: planning to pose ({:.3f} {:.3f} {:.3f}, {:.3f} {:.3f} {:.3f})",
                         target_xyzrpy[0], target_xyzrpy[1], target_xyzrpy[2],
                         target_xyzrpy[3], target_xyzrpy[4], target_xyzrpy[5]);

                std::vector<double> current_pos(num_joints, 0.0);
                {
                    std::lock_guard<std::mutex> lock(traj_mutex);
                    current_pos = traj_current_positions;
                }

                JointTrajectory traj = planner.PlanToPose(current_pos, target_xyzrpy, 3.0);
                if (!traj.points.empty()) {
                    std::lock_guard<std::mutex> lock(traj_mutex);
                    executor.Start(traj);
                    trajectory_active = true;
                    LOG_INFO("Trajectory started: {} points", traj.points.size());
                }
            } else {
                size_t copy_bytes = std::min(data.size(), sizeof(current_arm_jpos));
                memcpy(current_arm_jpos, data.data(), copy_bytes);
            }
        }
    );

    int frame_count = 0;
    auto last_fps_log = std::chrono::steady_clock::now();

    cv::namedWindow("URDF Robot Viewer (MQTT+URDF Renderer)", cv::WINDOW_NORMAL);
    cv::setMouseCallback("URDF Robot Viewer (MQTT+URDF Renderer)", on_mouse, &orbit_cam);

    LOG_INFO("Starting render loop.");
    LOG_INFO("Mouse: left-drag to orbit, scroll to zoom. Press 'q' to quit.");

    while (g_running) {
        std::vector<double> traj_positions;
        bool on_trajectory = false;
        {
            std::lock_guard<std::mutex> lock(traj_mutex);
            on_trajectory = executor.GetCurrentPosition(traj_positions);
            if (!on_trajectory && trajectory_active) {
                trajectory_active = false;
                LOG_INFO("Trajectory completed");
            }
        }

        if (on_trajectory) {
            for (size_t i = 0; i < std::min(num_joints, traj_positions.size()); ++i) {
                renderer.setJointAngle(joint_names[i], traj_positions[i]);
            }
            {
                std::lock_guard<std::mutex> lock(traj_mutex);
                traj_current_positions = traj_positions;
            }
        } else {
            size_t render_joints = std::min(num_joints, sizeof(current_arm_jpos) / sizeof(current_arm_jpos[0]));
            for (size_t i = 0; i < render_joints; ++i) {
                renderer.setJointAngle(joint_names[i], current_arm_jpos[i]);
            }
            {
                std::lock_guard<std::mutex> lock(traj_mutex);
                for (size_t i = 0; i < render_joints; ++i)
                    traj_current_positions[i] = current_arm_jpos[i];
            }
        }

        orbit_cam.apply(renderer);

        if (renderer.renderFrame() != drivers::URDF_SUCCESS) {
            LOG_ERROR("Render failed: {}", renderer.getLastError());
            break;
        }

        cv::Mat rgba = renderer.getImageAsMat();
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);

        cv::imshow("URDF Robot Viewer (MQTT+URDF Renderer)", bgr);
        ++frame_count;

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_log);
        if (elapsed.count() >= 1) {
            LOG_INFO("Render FPS: {}", frame_count);
            frame_count = 0;
            last_fps_log = now;
        }

        int key = cv::pollKey();
        if (key == 'q' || key == 'Q' || key == 27) {
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    cv::destroyAllWindows();
    mqtt_client.Disconnect();
    renderer.shutdown();
    LOG_INFO("Shutdown complete");
    return 0;
}
