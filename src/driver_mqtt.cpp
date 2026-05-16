#include "driver_mqtt.hpp"
#include <chrono>

namespace drivers
{

MqttClient::MqttClient(const std::string& ip, int port, const std::string& client_id)
{
    this->ip_ = ip;
    this->port_ = port;
    this->client_id_ = client_id;
    std::string mqtt_addr = "tcp://" + ip_ + ":" + std::to_string(port_);
    this->client_ = std::make_unique<mqtt::async_client>(mqtt_addr, client_id_);
}

void MqttClient::SetConfig(const std::string& ip, int port, const std::string& client_id)
{
    this->ip_ = ip;
    this->port_ = port;
    this->client_id_ = client_id;
    std::string mqtt_addr = "tcp://" + ip_ + ":" + std::to_string(port_);
    this->client_ = std::make_unique<mqtt::async_client>(mqtt_addr, client_id_);
}

MqttClient::~MqttClient() {
    try {
        if (client_) {
            client_->disconnect()->wait_for(std::chrono::seconds(3));
            client_->stop_consuming();
        }
    } catch (...) {}
}

bool MqttClient::Connect()
{
    try
    {
        auto connect_options = mqtt::connect_options_builder::v3()
                                   .clean_session(false)
                                   .connect_timeout(std::chrono::seconds(5))
                                   .finalize();
        auto msg_fn = std::bind(&MqttClient::MessageCallback, this, std::placeholders::_1);
        this->client_cb_ = std::make_unique<ClientCallback>(msg_fn);
        client_->set_callback(*client_cb_);

        client_->start_consuming();
        LOG_INFO("Connecting to MQTT server...");
        client_->connect(connect_options);
        LOG_INFO("MQTT connect initiated (non-blocking) to {}:{}", ip_, port_);
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_WARN("MQTT connect initiation failed to {}:{} - {}", ip_, port_, e.what());
        try { client_->stop_consuming(); } catch (...) {}
        return false;
    }
}

bool MqttClient::Disconnect()
{
    try
    {
        LOG_INFO("Disconnecting from MQTT server...");
        client_->disconnect()->wait_for(std::chrono::seconds(3));
        client_->stop_consuming();
        LOG_INFO("Disconnected from MQTT server.");
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_WARN("MQTT disconnect: {}", e.what());
    }
    catch (...)
    {
        LOG_WARN("MQTT disconnect: unknown error");
    }
    return false;
}

bool MqttClient::Publish(const std::string& topic, const std::string& payload, int qos)
{
    try
    {
        if (!client_)
        {
            LOG_ERROR("MQTT client not initialized, cannot publish to {}", topic);
            return false;
        }
        auto msg = mqtt::make_message(topic, payload);
        msg->set_qos(qos);
        auto tok = client_->publish(msg);
        if (tok)
        {
            tok->wait();
        }
        LOG_INFO("Published to MQTT topic: {} (qos={})", topic, qos);
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_ERROR("Failed to publish to {}: what(): {}", topic, e.what());
    }
    catch (...)
    {
        LOG_ERROR("Unknown error publishing to {}", topic);
    }
    return false;
}

bool MqttClient::Subscribe(const std::string& topic, int qos)
{
    try
    {
        if (!client_)
        {
            LOG_ERROR("MQTT client not initialized, cannot subscribe to {}", topic);
            return false;
        }
        auto tok = client_->subscribe(topic, qos);
        if (tok)
        {
            tok->wait();
        }
        LOG_INFO("Subscribed to MQTT topic: {} (qos={})", topic, qos);
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_ERROR("Failed to subscribe to {}: what(): {}", topic, e.what());
    }
    catch (...)
    {
        LOG_ERROR("Unknown error subscribing to {}", topic);
    }
    return false;
}

void MqttClient::SetMessageCallback(std::function<void(const std::string&, const std::string&)> cb)
{
    msg_callback_ = std::move(cb);
}

void MqttClient::MessageCallback(mqtt::const_message_ptr msg)
{
    if (!msg) return;

    std::string topic = msg->get_topic();
    std::string payload = msg->to_string();

    if (msg_callback_) {
        msg_callback_(topic, payload);
    }
}

} // namespace drivers
