#include "driver_mqtt.hpp"
#include "protocol.pb.h"
#include <google/protobuf/util/json_util.h>
#include "logger.hpp"
#include <chrono>

namespace drivers
{
using InputTopic = MqttClient::InputTopic;
using OutputTopic = MqttClient::OutputTopic;
using TopicMeta = MqttClient::TopicMeta;

const std::array<TopicMeta, static_cast<size_t>(InputTopic::COUNT_INPUT_TOPICS)> MqttClient::INPUT_TOPIC_META_DICT = {
    TopicMeta { "GameStatus", 1 },
    TopicMeta { "GlobalUnitStatus", 1 },
    TopicMeta { "GlobalLogisticsStatus", 1 },
    TopicMeta { "GlobalSpecialMechanism", 1 },
    TopicMeta { "Event", 1 },
    TopicMeta { "RobotInjuryStat", 1 },
    TopicMeta { "RobotRespawnStatus", 1 },
    TopicMeta { "RobotStaticStatus", 1 },
    TopicMeta { "RobotDynamicStatus", 1 },
    TopicMeta { "RobotModuleStatus", 1 },
    TopicMeta { "RobotPosition", 1 },
    TopicMeta { "Buff", 1 },
    TopicMeta { "PenaltyInfo", 1 },
    TopicMeta { "RobotPathPlanInfo", 1 },
    TopicMeta { "RadarInfoToClient", 1 },
    TopicMeta { "CustomByteBlock", 0 },
    TopicMeta { "MapClickInfoNotify", 1 },
    TopicMeta { "TechCoreMotionStateSync", 1 },
    TopicMeta { "RobotPerformanceSelectionSync", 1 },
    TopicMeta { "DeployModeStatusSync", 1 },
    TopicMeta { "RuneStatusSync", 1 },
    TopicMeta { "SentryStatusSync", 1 },
    TopicMeta { "DartSelectTargetStatusSync", 1 },
    TopicMeta { "SentryCtrlResult", 1 },
    TopicMeta { "AirSupportStatusSync", 1 },
};

const std::array<TopicMeta, static_cast<size_t>(OutputTopic::COUNT_OUTPUT_TOPICS)> MqttClient::OUTPUT_TOPIC_META_DICT = {
    TopicMeta { "KeyboardMouseControl", 1 },
    TopicMeta { "CustomControl", 1 },
    TopicMeta { "MapClickInfoNotify", 1 },
    TopicMeta { "AssemblyCommand", 1 },
    TopicMeta { "RobotPerformanceSelectionCommand", 1 },
    TopicMeta { "CommonCommand", 1 },
    TopicMeta { "HeroDeployModeEventCommand", 1 },
    TopicMeta { "RuneActivateCommand", 1 },
    TopicMeta { "DartCommand", 1 },
    TopicMeta { "SentryCtrlCommand", 1 },
    TopicMeta { "AirSupportCommand", 1 },
};

MqttClient::MqttClient(const std::string& ip, int port, const std::string& client_id)
{
    this->ip_ = ip;
    this->port_ = port;
    this->client_id_ = client_id;
    std::string mqtt_addr = "mqtt://" + ip_ + ":" + std::to_string(port_);
    this->client_ = std::make_unique<mqtt::async_client>(mqtt_addr, client_id_);
}

bool MqttClient::Connect()
{
    try
    {
        auto connect_options = mqtt::connect_options_builder::v3()
                                   .clean_session(false)
                                   .automatic_reconnect()
                                   .finalize();
        this->client_cb_ = std::make_unique<ClientCallback>(
            std::bind(&MqttClient::MessageCallback, this, std::placeholders::_1));
        client_->set_callback(*client_cb_);

        client_->start_consuming();
        LOG_INFO("Connecting to MQTT server {}:{} ...", ip_, port_);
        auto token = client_->connect(connect_options);
        auto rsp = token->get_connect_response();
        if (!rsp.is_session_present())
        {
            LOG_INFO("No session present on server. Subscribing...");
            InitSubscriber();
        }
        connected_ = true;
        LOG_INFO("MQTT connected successfully to {}:{}", ip_, port_);
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_ERROR("Failed to connect to MQTT broker at {}:{} - {}", ip_, port_, e.what());
    }
    catch (...)
    {
        LOG_ERROR("Unknown error connecting to MQTT broker at {}:{}", ip_, port_);
    }
    return false;
}

bool MqttClient::Disconnect()
{
    try
    {
        if (!connected_) return true;
        LOG_INFO("Disconnecting from MQTT server...");
        auto token = client_->disconnect();
        token->wait();
        connected_ = false;
        LOG_INFO("Disconnected from MQTT server.");
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_ERROR("Failed to disconnect - {}", e.what());
    }
    catch (...)
    {
        LOG_ERROR("Unknown error disconnecting");
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
        if (tok) tok->wait();
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_ERROR("Failed to publish to {} - {}", topic, e.what());
    }
    catch (...)
    {
        LOG_ERROR("Unknown error publishing to {}", topic);
    }
    return false;
}

bool MqttClient::Publish(OutputTopic topic, const std::string& payload, int qos)
{
    const auto& info = GetOutputTopic(topic);
    int effective_qos = (qos >= 0 ? qos : info.qos);
    return Publish(info.name, payload, effective_qos);
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
        if (tok) tok->wait();
        LOG_INFO("Subscribed to MQTT topic: {} (qos={})", topic, qos);
        return true;
    }
    catch (const mqtt::exception& e)
    {
        LOG_ERROR("Failed to subscribe to {} - {}", topic, e.what());
    }
    catch (...)
    {
        LOG_ERROR("Unknown error subscribing to {}", topic);
    }
    return false;
}

bool MqttClient::Subscribe(InputTopic topic, int qos)
{
    const auto& info = GetInputTopic(topic);
    int effective_qos = (qos >= 0 ? qos : info.qos);
    return Subscribe(info.name, effective_qos);
}

void MqttClient::SetCustomByteBlockHandler(std::function<void(const std::vector<uint8_t>&)> handler)
{
    std::lock_guard<std::mutex> lock(handler_mutex_);
    custom_byte_block_handler_ = std::move(handler);
}

void MqttClient::SetUnhandledTopicCallback(std::function<void(const std::string& topic, const std::string& payload)> cb)
{
    unhandled_topic_callback_ = std::move(cb);
}

void MqttClient::InitSubscriber()
{
    for (size_t i = 0; i < static_cast<size_t>(InputTopic::COUNT_INPUT_TOPICS); ++i)
    {
        const auto& topic_meta = INPUT_TOPIC_META_DICT[i];
        Subscribe(topic_meta.name, topic_meta.qos);
    }
}

const MqttClient::TopicMeta& MqttClient::GetInputTopic(InputTopic t)
{
    return INPUT_TOPIC_META_DICT[static_cast<size_t>(t)];
}

const MqttClient::TopicMeta& MqttClient::GetOutputTopic(OutputTopic t)
{
    return OUTPUT_TOPIC_META_DICT[static_cast<size_t>(t)];
}

void MqttClient::MessageCallback(mqtt::const_message_ptr msg)
{
    if (!msg) return;

    std::string topic = msg->get_topic();
    std::string payload = msg->to_string();

    LOG_INFO("Received MQTT message on topic: {}", topic);

    if (topic == GetInputTopic(InputTopic::GAME_STATUS).name) {
        GameStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("GameStatus: stage={} round={}/{} countdown={}",
                     status.current_stage(), status.current_round(),
                     status.total_rounds(), status.stage_countdown_sec());
        } else { LOG_ERROR("Failed to parse GameStatus"); }
    }

    else if (GetInputTopic(InputTopic::GLOBAL_UNIT_STATUS).name == topic) {
        GlobalUnitStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("GlobalUnitStatus: base_hp={} robots={}",
                     status.base_health(), status.robot_health_size());
        } else { LOG_ERROR("Failed to parse GlobalUnitStatus"); }
    }

    else if (GetInputTopic(InputTopic::GLOBAL_LOGISTICS_STATUS).name == topic) {
        GlobalLogisticsStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("GlobalLogisticsStatus: economy={} tech={}",
                     status.remaining_economy(), status.tech_level());
        } else { LOG_ERROR("Failed to parse GlobalLogisticsStatus"); }
    }

    else if (GetInputTopic(InputTopic::GLOBAL_SPECIAL_MECHANISM).name == topic) {
        GlobalSpecialMechanism status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("GlobalSpecialMechanism: {}", json);
        } else { LOG_ERROR("Failed to parse GlobalSpecialMechanism"); }
    }

    else if (GetInputTopic(InputTopic::EVENT).name == topic) {
        Event status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("Event: {}", json);
        } else { LOG_ERROR("Failed to parse Event"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_INJURY_STAT).name == topic) {
        RobotInjuryStat status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("RobotInjuryStat: {}", json);
        } else { LOG_ERROR("Failed to parse RobotInjuryStat"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_RESPAWN_STATUS).name == topic) {
        RobotRespawnStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("RobotRespawnStatus: pending={}", (int)status.is_pending_respawn());
        } else { LOG_ERROR("Failed to parse RobotRespawnStatus"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_STATIC_STATUS).name == topic) {
        RobotStaticStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("RobotStaticStatus: robot_id={}", status.robot_id());
        } else { LOG_ERROR("Failed to parse RobotStaticStatus"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_DYNAMIC_STATUS).name == topic) {
        RobotDynamicStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("RobotDynamicStatus: hp={} heat={:.1f} ammo={}",
                     status.current_health(), status.current_heat(), status.remaining_ammo());
        } else { LOG_ERROR("Failed to parse RobotDynamicStatus"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_MODULE_STATUS).name == topic) {
        RobotModuleStatus status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("RobotModuleStatus: power={} rfid={} shooter17={}",
                     status.power_manager(), status.rfid(), status.small_shooter());
        } else { LOG_ERROR("Failed to parse RobotModuleStatus"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_POSITION).name == topic) {
        RobotPosition status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("RobotPosition: robot_id={} x={} y={} yaw={}",
                     status.robot_id(), status.x(), status.y(), status.yaw());
        } else { LOG_ERROR("Failed to parse RobotPosition"); }
    }

    else if (GetInputTopic(InputTopic::BUFF).name == topic) {
        Buff status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("Buff: robot_id={} type={} level={}",
                     status.robot_id(), status.buff_type(), status.buff_level());
        } else { LOG_ERROR("Failed to parse Buff"); }
    }

    else if (GetInputTopic(InputTopic::PENALTY_INFO).name == topic) {
        PenaltyInfo status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("PenaltyInfo: type={} sec={} num={}",
                     status.penalty_type(), status.penalty_effect_sec(), status.total_penalty_num());
        } else { LOG_ERROR("Failed to parse PenaltyInfo"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_PATH_PLAN_INFO).name == topic) {
        RobotPathPlanInfo status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("RobotPathPlanInfo: {}", json);
        } else { LOG_ERROR("Failed to parse RobotPathPlanInfo"); }
    }

    else if (GetInputTopic(InputTopic::RADAR_INFO_TO_CLIENT).name == topic) {
        RadarInfoToClient status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("RadarInfoToClient: {}", json);
        } else { LOG_ERROR("Failed to parse RadarInfoToClient"); }
    }

    else if (GetInputTopic(InputTopic::CUSTOM_BYTE_BLOCK).name == topic) {
        CustomByteBlock status;
        if (status.ParseFromString(payload)) {
            std::vector<uint8_t> packet_data(status.data().begin(), status.data().end());
            static uint64_t total_packets = 0;
            ++total_packets;
            if (total_packets % 100 == 0) {
                LOG_INFO("MQTT CustomByteBlock: {} packets received", total_packets);
            }
            std::function<void(const std::vector<uint8_t>&)> handler;
            { std::lock_guard<std::mutex> lock(handler_mutex_); handler = custom_byte_block_handler_; }
            if (handler) handler(packet_data);
        } else { LOG_ERROR("Failed to parse CustomByteBlock"); }
    }

    else if (GetInputTopic(InputTopic::MAP_CLICK_INFO_NOTIFY).name == topic) {
        MapClickInfoNotify status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("MapClickInfoNotify: {}", json);
        } else { LOG_ERROR("Failed to parse MapClickInfoNotify"); }
    }

    else if (GetInputTopic(InputTopic::TECH_CORE_MOTION_STATE_SYNC).name == topic) {
        TechCoreMotionStateSync status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("TechCoreMotionStateSync: {}", json);
        } else { LOG_ERROR("Failed to parse TechCoreMotionStateSync"); }
    }

    else if (GetInputTopic(InputTopic::ROBOT_PERFORMANCE_SELECTION_SYNC).name == topic) {
        RobotPerformanceSelectionSync status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("RobotPerformanceSelectionSync: {}", json);
        } else { LOG_ERROR("Failed to parse RobotPerformanceSelectionSync"); }
    }

    else if (GetInputTopic(InputTopic::DEPLOY_MODE_STATUS_SYNC).name == topic) {
        DeployModeStatusSync status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("DeployModeStatusSync: {}", json);
        } else { LOG_ERROR("Failed to parse DeployModeStatusSync"); }
    }

    else if (GetInputTopic(InputTopic::RUNE_STATUS_SYNC).name == topic) {
        RuneStatusSync status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("RuneStatusSync: status={} arms={}",
                     status.rune_status(), status.activated_arms());
        } else { LOG_ERROR("Failed to parse RuneStatusSync"); }
    }

    else if (GetInputTopic(InputTopic::SENTRY_STATUS_SYNC).name == topic) {
        SentryStatusSync status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("SentryStatusSync: posture={} weakened={}",
                     status.posture_id(), (int)status.is_weakened());
        } else { LOG_ERROR("Failed to parse SentryStatusSync"); }
    }

    else if (GetInputTopic(InputTopic::DART_SELECT_TARGET_STATUS_SYNC).name == topic) {
        DartSelectTargetStatusSync status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("DartSelectTargetStatusSync: target={}", status.target_id());
        } else { LOG_ERROR("Failed to parse DartSelectTargetStatusSync"); }
    }

    else if (GetInputTopic(InputTopic::SENTRY_CTRL_RESULT).name == topic) {
        SentryCtrlResult status;
        if (status.ParseFromString(payload)) {
            LOG_INFO("SentryCtrlResult: cmd={} result={}", status.command_id(), status.result_code());
        } else { LOG_ERROR("Failed to parse SentryCtrlResult"); }
    }

    else if (GetInputTopic(InputTopic::AIR_SUPPORT_STATUS_SYNC).name == topic) {
        AirSupportStatusSync status;
        if (status.ParseFromString(payload)) {
            std::string json;
            google::protobuf::util::MessageToJsonString(status, &json);
            LOG_INFO("AirSupportStatusSync: {}", json);
        } else { LOG_ERROR("Failed to parse AirSupportStatusSync"); }
    }

    else if (topic.empty()) {
        LOG_WARN("Received MQTT message with empty topic");
    }
    else {
        LOG_WARN("Received MQTT message on unknown topic: {}", topic);
        if (unhandled_topic_callback_) {
            unhandled_topic_callback_(topic, payload);
        }
    }
}

} // namespace drivers
