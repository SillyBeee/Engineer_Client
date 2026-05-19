#include "driver_urdf_renderer.hpp"
#include "trajectory_planner.hpp"
#include "logger.hpp"
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <cstring>
#include <csignal>
#include <atomic>
#include <mutex>
#include <thread>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

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

    if (event == cv::EVENT_LBUTTONDOWN) {
        prev_x = x;
        prev_y = y;
    } else if (event == cv::EVENT_MOUSEMOVE && (flags & cv::EVENT_FLAG_LBUTTON)) {
        if (prev_x >= 0 && prev_y >= 0) {
            cam->orbit(x - prev_x, y - prev_y);
            prev_x = x;
            prev_y = y;
        }
    } else if (event == cv::EVENT_LBUTTONUP) {
        prev_x = -1;
        prev_y = -1;
    } else if (event == cv::EVENT_MOUSEWHEEL) {
        int delta = (flags >> 16) & 0xFFFF;
        cam->zoom(delta > 0 ? 1.0 : -1.0);
    }
}

static void print_usage(const char* prog) {
    fprintf(stderr,
        "Usage: %s [options] --joint <j1> ... <jN>\n"
        "   or: %s [options] --pose <x> <y> <z> <roll> <pitch> <yaw>\n"
        "Options:\n"
        "  --urdf PATH       URDF file path\n"
        "  --plan-time SEC   OMPL planning time (default: 5.0)\n"
        "  --repeat          Loop the trajectory\n"
        "  --joint           Plan to target joint angles (default if no --pose)\n"
        "  --pose            Plan to target end-effector pose (xyzrpy)\n"
        "  --degrees         Input angles in degrees (only for --joint, default: rad)\n"
        "  --help            Show this help\n"
        "\n"
        "Examples:\n"
        "  %s --joint 0.5 -0.3 0.8 -0.2 0.6 -0.4 0.3 -0.1 0.2\n"
        "  %s --degrees --joint 30 -20 45 -15 35 -25 20 -10 15\n"
        "  %s --pose 0.30 0.0 0.45 0.0 0.0 0.0\n"
        "\n"
        "Mouse controls:\n"
        "  Left drag: orbit\n"
        "  Scroll: zoom\n",
        prog, prog, prog, prog, prog);
}

int main(int argc, char** argv) {
    logger::Logger::GetInstance().Init();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto src = std::filesystem::path(PROJECT_SOURCE_DIR);
    std::string urdf_path = (src / "arm_description/urdf/Engineer2.urdf").string();
    double planning_time = 5.0;
    bool degrees = false;
    bool repeat = false;
    bool pose_mode = false;

    std::vector<double> values;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--urdf") == 0 && i + 1 < argc) {
            urdf_path = argv[++i];
        } else if (strcmp(argv[i], "--plan-time") == 0 && i + 1 < argc) {
            planning_time = std::stod(argv[++i]);
        } else if (strcmp(argv[i], "--degrees") == 0) {
            degrees = true;
        } else if (strcmp(argv[i], "--repeat") == 0) {
            repeat = true;
        } else if (strcmp(argv[i], "--pose") == 0) {
            pose_mode = true;
        } else if (strcmp(argv[i], "--joint") == 0) {
            pose_mode = false;
        } else if (strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        } else {
            values.push_back(std::stod(argv[i]));
        }
    }

    if (values.empty()) {
        fprintf(stderr, "No target values provided.\n\n");
        print_usage(argv[0]);
        return 1;
    }

    if (!std::filesystem::exists(urdf_path)) {
        LOG_ERROR("URDF not found: {}", urdf_path);
        return 1;
    }

    drivers::URDFRendererPlugin renderer;
    drivers::UrdfRenderConfig render_cfg;
    render_cfg.width = 800;
    render_cfg.height = 600;
    render_cfg.transparent_background = false;
    render_cfg.background_color[0] = 0.2f;
    render_cfg.background_color[1] = 0.2f;
    render_cfg.background_color[2] = 0.25f;
    render_cfg.background_color[3] = 1.0f;
    render_cfg.anti_aliasing = 0;

    if (!renderer.initialize(&render_cfg)) {
        LOG_ERROR("Renderer init failed: {}", renderer.getLastError());
        return 1;
    }
    if (renderer.loadURDF(urdf_path) != drivers::URDF_SUCCESS) {
        LOG_ERROR("URDF load failed: {}", renderer.getLastError());
        return 1;
    }

    OrbitCamera orbit_cam;
    orbit_cam.dirty = true;
    orbit_cam.apply(renderer);

    URDFMotionPlanner planner;
    if (!planner.LoadURDF(urdf_path)) {
        LOG_ERROR("Planner init failed");
        return 1;
    }

    size_t num_joints = planner.num_joints();
    auto joint_names = planner.getJointNames();
    std::vector<double> zero_angles(num_joints, 0.0);
    TrajectoryExecutor executor;

    JointTrajectory traj;

    if (pose_mode) {
        if (values.size() != 6) {
            LOG_ERROR("Pose mode expects 6 values (x y z roll pitch yaw), got {}", values.size());
            return 1;
        }

        if (degrees) {
            LOG_WARN("--degrees ignored in pose mode");
        }

        double target_xyzrpy[6] = {
            values[0], values[1], values[2],
            values[3], values[4], values[5]
        };

        LOG_INFO("Planning from zero to end-effector pose:");
        LOG_INFO("  pos:  {:.4f}  {:.4f}  {:.4f}", target_xyzrpy[0], target_xyzrpy[1], target_xyzrpy[2]);
        LOG_INFO("  rpy:  {:.4f}  {:.4f}  {:.4f}", target_xyzrpy[3], target_xyzrpy[4], target_xyzrpy[5]);

        traj = planner.PlanToPose(zero_angles, target_xyzrpy, planning_time);

    } else {
        if (values.size() != num_joints) {
            LOG_ERROR("Joint mode expects {} values, got {}", num_joints, values.size());
            return 1;
        }

        if (degrees) {
            for (auto& a : values) a = a * M_PI / 180.0;
        }

        LOG_INFO("Planning from all-zero to target:");
        for (size_t i = 0; i < num_joints; ++i) {
            LOG_INFO("  {}: 0.0 -> {:.4f}", joint_names[i], values[i]);
        }

        traj = planner.PlanToTarget(zero_angles, values, planning_time);
    }

    if (traj.points.empty()) {
        LOG_ERROR("Planning returned empty trajectory");
        return 1;
    }

    LOG_INFO("Trajectory: {} points, {:.2f}s duration",
             traj.points.size(), traj.points.back().time_from_start);

    std::vector<double> current_pos = zero_angles;
    int frame_count = 0;
    auto last_fps_log = std::chrono::steady_clock::now();

    cv::namedWindow("OMPL+Ruckig Trajectory Demo", cv::WINDOW_NORMAL);
    cv::setMouseCallback("OMPL+Ruckig Trajectory Demo", on_mouse, &orbit_cam);

    LOG_INFO("Press 'q' or ESC to quit. Press 'r' to replay.");
    LOG_INFO("Mouse: left-drag to orbit, scroll to zoom.");

    do {
        for (size_t i = 0; i < num_joints; ++i) {
            renderer.setJointAngle(joint_names[i], zero_angles[i]);
        }
        orbit_cam.apply(renderer);
        renderer.renderFrame();
        cv::Mat rgba = renderer.getImageAsMat();
        cv::Mat bgr;
        cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
        cv::imshow("OMPL+Ruckig Trajectory Demo", bgr);
        cv::waitKey(1);

        executor.Start(traj);

        while (g_running) {
            std::vector<double> pos;
            bool active = executor.GetCurrentPosition(pos);
            if (!active) break;

            current_pos = pos;
            for (size_t i = 0; i < num_joints; ++i) {
                renderer.setJointAngle(joint_names[i], pos[i]);
            }

            orbit_cam.apply(renderer);
            if (renderer.renderFrame() != drivers::URDF_SUCCESS) break;

            cv::Mat frame = renderer.getImageAsMat();
            cv::Mat bgr_frame;
            cv::cvtColor(frame, bgr_frame, cv::COLOR_RGBA2BGR);

            int key = cv::pollKey();
            if (key == 'q' || key == 'Q' || key == 27) {
                g_running = false;
                break;
            }
            if (key == 'r' || key == 'R') {
                executor.Start(traj);
                continue;
            }

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_fps_log);
            if (elapsed.count() >= 1) {
                LOG_INFO("Render FPS: {}", frame_count);
                frame_count = 0;
                last_fps_log = now;
            }

            cv::imshow("OMPL+Ruckig Trajectory Demo", bgr_frame);
            ++frame_count;

            std::this_thread::sleep_for(std::chrono::milliseconds(8));
        }

        LOG_INFO("Trajectory complete");

    } while (repeat && g_running);

    cv::destroyAllWindows();
    renderer.shutdown();
    LOG_INFO("Done");
    return 0;
}
