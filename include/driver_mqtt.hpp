#ifndef DRIVER_MQTT_HPP
#define DRIVER_MQTT_HPP
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
    explicit MqttClient(const std::string& ip = "127.0.0.1", int port = 3333, const std::string& client_id = "RM_Client");

    bool Connect();
    bool Disconnect();
    void SetConfig(const std::string& ip, int port, const std::string& client_id);

    bool Publish(const std::string& topic, const std::string& payload, int qos = 1);
    bool Subscribe(const std::string& topic, int qos = 1);

    void SetMessageCallback(std::function<void(const std::string& topic, const std::string& payload)> cb);

    ~MqttClient();

private:
    void MessageCallback(mqtt::const_message_ptr msg);

    std::unique_ptr<mqtt::async_client> client_;
    std::unique_ptr<mqtt::callback> client_cb_;

    std::string ip_;
    int port_;
    std::string client_id_;

    std::function<void(const std::string& topic, const std::string& payload)> msg_callback_;
};


class ClientCallback : public virtual mqtt::callback
{
public:
    using MsgFn = std::function<void(mqtt::const_message_ptr)>;
    explicit ClientCallback(MsgFn msg_fn)
        : msg_fn_(std::move(msg_fn)) {}
    void connected(const std::string&) override
    {
        LOG_INFO("MQTT connected");
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
};

} // namespace drivers
#endif
