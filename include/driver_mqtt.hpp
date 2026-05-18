#ifndef DRIVER_MQTT_HPP
#define DRIVER_MQTT_HPP
#include <array>
#include <functional>
#include <mqtt/async_client.h>
#include "logger.hpp"
#include <unordered_map>
#include <vector>

namespace drivers
{

class MqttClient
{
public:
    enum class InputTopic : uint8_t
    {
        GAME_STATUS = 0,
        GLOBAL_UNIT_STATUS,
        GLOBAL_LOGISTICS_STATUS,
        GLOBAL_SPECIAL_MECHANISM,
        EVENT,
        ROBOT_INJURY_STAT,
        ROBOT_RESPAWN_STATUS,
        ROBOT_STATIC_STATUS,
        ROBOT_DYNAMIC_STATUS,
        ROBOT_MODULE_STATUS,
        ROBOT_POSITION,
        BUFF,
        PENALTY_INFO,
        ROBOT_PATH_PLAN_INFO,
        RADAR_INFO_TO_CLIENT,
        CUSTOM_BYTE_BLOCK,
        MAP_CLICK_INFO_NOTIFY,
        TECH_CORE_MOTION_STATE_SYNC,
        ROBOT_PERFORMANCE_SELECTION_SYNC,
        DEPLOY_MODE_STATUS_SYNC,
        RUNE_STATUS_SYNC,
        SENTRY_STATUS_SYNC,
        DART_SELECT_TARGET_STATUS_SYNC,
        SENTRY_CTRL_RESULT,
        AIR_SUPPORT_STATUS_SYNC,
        COUNT_INPUT_TOPICS,
    };

    enum class OutputTopic : uint8_t
    {
        KEYBOARD_MOUSE_CONTROL = 0,
        CUSTOM_CONTROL,
        MAP_CLICK_INFO_NOTIFY,
        ASSEMBLY_COMMAND,
        ROBOT_PERFORMANCE_SELECTION_COMMAND,
        COMMON_COMMAND,
        HERO_DEPLOY_MODE_EVENT_COMMAND,
        RUNE_ACTIVATE_COMMAND,
        DART_COMMAND,
        SENTRY_CTRL_COMMAND,
        AIR_SUPPORT_COMMAND,
        COUNT_OUTPUT_TOPICS,
    };

    struct TopicMeta
    {
        std::string name;
        int qos;
    };

    explicit MqttClient(const std::string& ip = "192.168.12.1", int port = 3333, const std::string& client_id = "RM_Client");

    bool Connect();
    bool Disconnect();
    void SetConfig(const std::string& ip, int port, const std::string& client_id);

    bool Publish(const std::string& topic, const std::string& payload, int qos = 1);
    bool Publish(OutputTopic topic, const std::string& payload, int qos = -1);
    bool Subscribe(const std::string& topic, int qos = 1);
    bool Subscribe(InputTopic topic, int qos = -1);
    void SetCustomByteBlockHandler(std::function<void(const std::vector<uint8_t>&)> handler);
    void SetUnhandledTopicCallback(std::function<void(const std::string& topic, const std::string& payload)> cb);

    static const TopicMeta& GetInputTopic(InputTopic t);
    static const TopicMeta& GetOutputTopic(OutputTopic t);

    ~MqttClient();

private:
    void InitSubscriber();
    void MessageCallback(mqtt::const_message_ptr msg);

    std::unique_ptr<mqtt::async_client> client_;
    std::unique_ptr<mqtt::callback> client_cb_;

    std::string ip_;
    int port_;
    std::string client_id_;

    std::mutex handler_mutex_;
    std::function<void(const std::vector<uint8_t>&)> custom_byte_block_handler_;
    std::function<void(const std::string&, const std::string&)> unhandled_topic_callback_;

    static const std::array<TopicMeta, static_cast<size_t>(InputTopic::COUNT_INPUT_TOPICS)> INPUT_TOPIC_META_DICT;
    static const std::array<TopicMeta, static_cast<size_t>(OutputTopic::COUNT_OUTPUT_TOPICS)> OUTPUT_TOPIC_META_DICT;
};


class ClientCallback : public virtual mqtt::callback
{
public:
    using MsgFn = std::function<void(mqtt::const_message_ptr)>;
    using ConnFn = std::function<void()>;
    explicit ClientCallback(MsgFn msg_fn, ConnFn conn_fn = nullptr)
        : msg_fn_(std::move(msg_fn)), conn_fn_(std::move(conn_fn)) {}
    void connected(const std::string&) override
    {
        LOG_INFO("MQTT connected");
        if (conn_fn_) conn_fn_();
    }
    void connection_lost(const std::string& cause) override
    {
        LOG_WARN("MQTT connection lost: {}", cause);
    }
    void message_arrived(mqtt::const_message_ptr msg) override
    {
        if (msg_fn_) msg_fn_(msg);
    }
    void delivery_complete(mqtt::delivery_token_ptr token) override
    {
        (void)token;
    }

private:
    MsgFn msg_fn_;
    ConnFn conn_fn_;
};

} // namespace drivers
#endif
