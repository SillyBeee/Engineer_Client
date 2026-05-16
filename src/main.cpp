#include "driver_mqtt.hpp"
#include "driver_urdf_renderer.hpp"
#include "logger.hpp"
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>

using json = nlohmann::json;

struct AppConfig {
    std::string mqtt_host = "127.0.0.1";
    int mqtt_port = 3333;
    std::string mqtt_client_id = "urdf_viewer";
    std::string urdf_path = "arm_description/urdf/Engineer2.urdf";

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(AppConfig, mqtt_host, mqtt_port, mqtt_client_id, urdf_path)
};

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
    g_running = false;
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
    AppConfig cfg;
    std::string config_path;

    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg == "--config" && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--mqtt-host" && i + 1 < argc) {
            cfg.mqtt_host = argv[++i];
        } else if (arg == "--mqtt-port" && i + 1 < argc) {
            cfg.mqtt_port = std::stoi(argv[++i]);
        } else if (arg == "--urdf-path" && i + 1 < argc) {
            cfg.urdf_path = argv[++i];
        } else if (arg == "--help") {
            print_usage(argv[0]);
            return 0;
        }
    }

    logger::Logger::GetInstance().Init();

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    auto src = std::filesystem::path(PROJECT_SOURCE_DIR);

    if (config_path.empty()) {
        config_path = (src / "config" / "client_setting.json").string();
    }

    if (std::filesystem::exists(config_path)) {
        try {
            std::ifstream file(config_path);
            json j;
            file >> j;
            AppConfig file_cfg = j.get<AppConfig>();
            if (cfg.mqtt_host == "127.0.0.1") cfg.mqtt_host = file_cfg.mqtt_host;
            if (cfg.mqtt_port == 3333) cfg.mqtt_port = file_cfg.mqtt_port;
            if (cfg.mqtt_client_id == "urdf_viewer") cfg.mqtt_client_id = file_cfg.mqtt_client_id;
            if (cfg.urdf_path == "arm_description/urdf/Engineer2.urdf") cfg.urdf_path = file_cfg.urdf_path;
            LOG_INFO("Loaded config from {}", config_path);
        } catch (const std::exception& e) {
            LOG_WARN("Failed to parse config file {}: {}", config_path, e.what());
        }
    } else {
        LOG_INFO("No config file found at {}, using defaults", config_path);
    }

    std::string urdf_path = cfg.urdf_path;
    if (!std::filesystem::path(urdf_path).is_absolute()) {
        urdf_path = (src / urdf_path).string();
    }

    if (!std::filesystem::exists(urdf_path)) {
        LOG_ERROR("URDF file not found: {}", urdf_path);
        return 1;
    }

    drivers::MqttClient mqtt_client(cfg.mqtt_host, cfg.mqtt_port, cfg.mqtt_client_id);
    LOG_INFO("MQTT config: {}:{} client_id={}", cfg.mqtt_host, cfg.mqtt_port, cfg.mqtt_client_id);
    mqtt_client.SetMessageCallback([](const std::string& topic, const std::string& payload) {
        LOG_INFO("MQTT topic={} size={}", topic, payload.size());
    });

    bool mqtt_ok = mqtt_client.Connect();
    if (!mqtt_ok) {
        LOG_WARN("MQTT connection failed, continuing without MQTT");
    } else {
        mqtt_client.Subscribe("RobotDynamicStatus");
        mqtt_client.Subscribe("RobotPosition");
        mqtt_client.Subscribe("CustomByteBlock", 0);
        LOG_INFO("MQTT connected and subscribed");
    }

    drivers::URDFRendererPlugin renderer;
    drivers::UrdfRenderConfig render_cfg;
    render_cfg.width = 800;
    render_cfg.height = 600;
    render_cfg.transparent_background = true;
    render_cfg.anti_aliasing = 0;

    if (!renderer.initialize(&render_cfg)) {
        LOG_ERROR("URDF renderer init failed: {}", renderer.getLastError());
        return 1;
    }

    if (renderer.loadURDF(urdf_path) != drivers::URDF_SUCCESS) {
        LOG_ERROR("URDF load failed: {}", renderer.getLastError());
        return 1;
    }
    LOG_INFO("URDF loaded: {} ({} joints)", urdf_path, renderer.getJointCount());

    drivers::UrdfCameraConfig cam;
    cam.position[0] = -1.0f; cam.position[1] = -0.5f; cam.position[2] = 3.0f;
    cam.look_at[0] = 0.0f; cam.look_at[1] = 0.0f; cam.look_at[2] = 0.5f;
    cam.up[0] = 0.0f; cam.up[1] = 0.0f; cam.up[2] = 1.0f;
    cam.fov_degrees = 50.0f;
    cam.near_clip = 0.02f;
    cam.far_clip = 20.0f;
    renderer.setCamera(cam);

    auto joint_names = renderer.getJointNames();
    double time = 0.0;
    int frame_count = 0;
    auto last_fps_log = std::chrono::steady_clock::now();

    LOG_INFO("Starting render loop. Press 'q' in the window or Ctrl+C to quit.");

    while (g_running) {
        for (size_t i = 0; i < joint_names.size(); ++i) {
            double angle = 0.3 * std::sin(time + i * 0.5);
            renderer.setJointAngle(joint_names[i], angle);
        }

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

        time += 0.05;

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
