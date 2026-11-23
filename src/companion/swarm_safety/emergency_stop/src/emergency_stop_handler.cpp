#include "swarm_safety_emergency_stop/emergency_stop_handler.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <iostream>

namespace swarm::safety {

using namespace std::chrono_literals;

EmergencyStopHandlerNode::EmergencyStopHandlerNode() : Node("emergency_stop_handler") {
    declareParameters();
    loadParameters();
    
    setupSubscriptions();
    setupPublishers();
    setupServices();
    
    // Timers
    broadcast_timer_ = create_wall_timer(
        100ms,  // Check broadcast retries
        [this]() { checkBroadcastRetries(); });
    
    ack_check_timer_ = create_wall_timer(
        100ms,  // Check ACK timeouts
        [this]() { checkAckTimeouts(); });
    
    status_timer_ = create_wall_timer(
        100ms,  // 10 Hz status
        [this]() { publishStatus(); });
    
    RCLCPP_INFO(get_logger(), "Emergency Stop Handler Node initialized");
    RCLCPP_INFO(get_logger(), "Mesh broadcast: %s, Parachute: %s, Local kill: %s",
                config_.enable_mesh_broadcast ? "on" : "off",
                config_.deploy_parachute ? "on" : "off",
                config_.local_motor_kill ? "on" : "off");
}

void EmergencyStopHandlerNode::declareParameters() {
    declare_parameter("drone_id", 1);
    declare_parameter("enable_mesh_broadcast", true);
    declare_parameter("enable_5g_broadcast", false);
    declare_parameter("broadcast_retry_count", 3);
    declare_parameter("broadcast_retry_interval_ms", 100);
    declare_parameter("broadcast_timeout_ms", 1000);
    declare_parameter("ack_timeout_ms", 2000);
    declare_parameter("max_missing_acks", 2);
    declare_parameter("require_all_acks", true);
    declare_parameter("deploy_parachute", false);
    declare_parameter("parachute_delay_s", 1.0);
    declare_parameter("parachute_servo_channel", 0);
    declare_parameter("parachute_deploy_pwm", 2000);
    declare_parameter("local_motor_kill", true);
    declare_parameter("motor_kill_pwm", 1000.0);
    declare_parameter("motor_count", 8);
    declare_parameter("require_ground_station_reset", true);
    declare_parameter("reset_cooldown_ms", 5000);
    declare_parameter("status_publish_rate_hz", 10);
    declare_parameter("swarm_members", std::vector<int64_t>{1, 2, 3, 4, 5});
}

void EmergencyStopHandlerNode::loadParameters() {
    drone_id_ = get_parameter("drone_id").as_int();
    config_.enable_mesh_broadcast = get_parameter("enable_mesh_broadcast").as_bool();
    config_.enable_5g_broadcast = get_parameter("enable_5g_broadcast").as_bool();
    config_.broadcast_retry_count = get_parameter("broadcast_retry_count").as_int();
    config_.broadcast_retry_interval_ms = get_parameter("broadcast_retry_interval_ms").as_int();
    config_.broadcast_timeout_ms = get_parameter("broadcast_timeout_ms").as_int();
    config_.ack_timeout_ms = get_parameter("ack_timeout_ms").as_int();
    config_.max_missing_acks_before_escalation = get_parameter("max_missing_acks").as_int();
    config_.require_all_acks = get_parameter("require_all_acks").as_bool();
    config_.deploy_parachute = get_parameter("deploy_parachute").as_bool();
    config_.parachute_delay_s = get_parameter("parachute_delay_s").as_double();
    config_.parachute_servo_channel = get_parameter("parachute_servo_channel").as_int();
    config_.parachute_deploy_pwm = get_parameter("parachute_deploy_pwm").as_int();
    config_.local_motor_kill = get_parameter("local_motor_kill").as_bool();
    config_.motor_kill_pwm = get_parameter("motor_kill_pwm").as_double();
    config_.motor_count = get_parameter("motor_count").as_int();
    config_.require_ground_station_reset = get_parameter("require_ground_station_reset").as_bool();
    config_.reset_cooldown_ms = get_parameter("reset_cooldown_ms").as_int();
    config_.status_publish_rate_hz = get_parameter("status_publish_rate_hz").as_int();
    
    auto members = get_parameter("swarm_members").as_integer_array();
    {
        std::lock_guard<std::mutex> lock(members_mutex_);
        swarm_members_.clear();
        for (auto m : members) {
            swarm_members_.push_back(static_cast<uint16_t>(m));
        }
    }
    
    // Initialize state
    state_.active = false;
    state_.parachute_deployed = false;
    state_.mesh_broadcast_sent = false;
    state_.broadcast_retries = 0;
    state_.escalated = false;
    state_.trigger_time_us = 0;
    state_.last_broadcast_us = 0;
    
    RCLCPP_INFO(get_logger(), "Emergency Stop config loaded: %zu swarm members", swarm_members_.size());
}

void EmergencyStopHandlerNode::setupSubscriptions() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
        "/swarm/agent_states", qos,
        [this](const swarm_msgs::msg::AgentState::SharedPtr msg) {
            if (msg->sysid == drone_id_) {
                agentStateCallback(msg);
            }
        });
    
    safety_event_sub_ = create_subscription<swarm_msgs::msg::SafetyEvent>(
        "/swarm/safety_events", qos,
        [this](const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
            safetyEventCallback(msg);
        });
    
    ack_sub_ = create_subscription<swarm_msgs::msg::EmergencyStopAck>(
        "/swarm/emergency_stop_ack", qos,
        [this](const swarm_msgs::msg::EmergencyStopAck::SharedPtr msg) {
            emergencyStopAckCallback(msg);
        });
    
    mesh_sub_ = create_subscription<swarm_msgs::msg::MeshMessage>(
        "/swarm/mesh_messages", qos,
        [this](const swarm_msgs::msg::MeshMessage::SharedPtr msg) {
            meshMessageCallback(msg);
        });
}

void EmergencyStopHandlerNode::setupPublishers() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    emergency_stop_pub_ = create_publisher<swarm_msgs::msg::EmergencyStop>(
        "/drone" + std::to_string(drone_id_) + "/emergency_stop", qos);
    
    ack_pub_ = create_publisher<swarm_msgs::msg::EmergencyStopAck>(
        "/drone" + std::to_string(drone_id_) + "/emergency_stop_ack", qos);
    
    mesh_pub_ = create_publisher<swarm_msgs::msg::MeshMessage>(
        "/swarm/mesh_messages", qos);
    
    mavlink_cmd_pub_ = create_publisher<mavros_msgs::msg::CommandLong>(
        "/drone" + std::to_string(drone_id_) + "/mavlink_command", qos);
    
    safety_event_pub_ = create_publisher<swarm_msgs::msg::SafetyEvent>(
        "/swarm/safety_events", qos);
}

void EmergencyStopHandlerNode::setupServices() {
    trigger_srv_ = create_service<swarm_msgs::srv::TriggerEmergencyStop>(
        "/drone" + std::to_string(drone_id_) + "/trigger_emergency_stop",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<swarm_msgs::srv::TriggerEmergencyStop::Request> request,
               std::shared_ptr<swarm_msgs::srv::TriggerEmergencyStop::Response> response) {
            triggerServiceCallback(nullptr, request, response);
        });
    
    reset_srv_ = create_service<swarm_msgs::srv::ResetEmergencyStop>(
        "/drone" + std::to_string(drone_id_) + "/reset_emergency_stop",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<swarm_msgs::srv::ResetEmergencyStop::Request> request,
               std::shared_ptr<swarm_msgs::srv::ResetEmergencyStop::Response> response) {
            resetServiceCallback(nullptr, request, response);
        });
}

void EmergencyStopHandlerNode::agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
    current_position_enu_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    current_velocity_enu_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    position_received_ = true;
}

void EmergencyStopHandlerNode::safetyEventCallback(const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
    // Auto-trigger on critical safety events
    if (msg->severity >= 3) {  // EMERGENCY
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!state_.active) {
            triggerEmergencyStop("Safety event: " + msg->details, msg->source_id);
        }
    }
}

void EmergencyStopHandlerNode::emergencyStopAckCallback(const swarm_msgs::msg::EmergencyStopAck::SharedPtr msg) {
    handleAck(*msg);
}

void EmergencyStopHandlerNode::meshMessageCallback(const swarm_msgs::msg::MeshMessage::SharedPtr msg) {
    if (msg->type == 10) {  // EMERGENCY_STOP
        handleIncomingEmergencyStop(*msg);
    }
}

void EmergencyStopHandlerNode::triggerServiceCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<swarm_msgs::srv::TriggerEmergencyStop::Request> request,
    std::shared_ptr<swarm_msgs::srv::TriggerEmergencyStop::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_.active) {
        response->success = false;
        response->message = "Emergency stop already active";
        return;
    }
    
    triggerEmergencyStop(request->reason, request->source_id);
    response->success = true;
    response->message = "Emergency stop triggered";
}

void EmergencyStopHandlerNode::resetServiceCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<swarm_msgs::srv::ResetEmergencyStop::Request> request,
    std::shared_ptr<swarm_msgs::srv::ResetEmergencyStop::Response> response) {
    
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!state_.active) {
        response->success = false;
        response->message = "No active emergency stop";
        return;
    }
    
    if (config_.require_ground_station_reset && request->requester_id != 0) {
        // Check if requester is ground station (ID 0)
        response->success = false;
        response->message = "Only ground station can reset";
        return;
    }
    
    resetEmergencyStop(request->requester_id);
    response->success = true;
    response->message = "Emergency stop reset";
}

void EmergencyStopHandlerNode::triggerEmergencyStop(const std::string& reason, uint16_t source_id) {
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    state_.active = true;
    state_.source_id = source_id;
    state_.reason = reason;
    state_.trigger_time_us = now_us;
    state_.parachute_deployed = false;
    state_.mesh_broadcast_sent = false;
    state_.broadcast_retries = 0;
    state_.escalated = false;
    state_.last_broadcast_us = now_us;
    state_.acks_received.clear();
    
    // Initialize pending ACKs for all swarm members
    {
        std::lock_guard<std::mutex> lock(acks_mutex_);
        pending_acks_.clear();
        for (uint16_t member : swarm_members_) {
            if (member != drone_id_) {
                pending_acks_[member] = {member, 0, false, "pending"};
            }
        }
    }
    
    RCLCPP_WARN(get_logger(), "EMERGENCY STOP TRIGGERED: %s (source: %d)", reason.c_str(), source_id);
    
    // Immediate local actions
    if (config_.local_motor_kill) {
        killMotors();
    }
    
    if (config_.deploy_parachute) {
        // Deploy parachute after delay
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::duration<double>(config_.parachute_delay_s));
            deployParachute();
        }).detach();
    }
    
    // Immediate broadcast
    broadcastEmergencyStop();
    
    // Publish safety event
    auto event = swarm_msgs::msg::SafetyEvent();
    event.timestamp_us = now_us;
    event.source_id = drone_id_;
    event.type = 1;  // EMERGENCY_STOP
    event.severity = 3;  // EMERGENCY
    event.position_enu = {static_cast<float>(current_position_enu_.x()),
                          static_cast<float>(current_position_enu_.y()),
                          static_cast<float>(current_position_enu_.z())};
    event.details = "Emergency stop: " + reason;
    event.sequence = 0;
    safety_event_pub_->publish(event);
}

void EmergencyStopHandlerNode::resetEmergencyStop(uint16_t requester_id) {
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    // Check cooldown
    if (now_us - state_.trigger_time_us < static_cast<uint64_t>(config_.reset_cooldown_ms) * 1000) {
        RCLCPP_WARN(get_logger(), "Reset rejected: cooldown period not elapsed");
        return;
    }
    
    state_.active = false;
    state_.parachute_deployed = false;
    state_.mesh_broadcast_sent = false;
    state_.broadcast_retries = 0;
    state_.escalated = false;
    state_.acks_received.clear();
    
    {
        std::lock_guard<std::mutex> lock(acks_mutex_);
        pending_acks_.clear();
    }
    
    RCLCPP_INFO(get_logger(), "Emergency stop reset by %d", requester_id);
    
    // Publish safety event
    auto event = swarm_msgs::msg::SafetyEvent();
    event.timestamp_us = now_us;
    event.source_id = drone_id_;
    event.type = 1;  // EMERGENCY_STOP
    event.severity = 0;  // INFO
    event.details = "Emergency stop reset by " + std::to_string(requester_id);
    safety_event_pub_->publish(event);
}

void EmergencyStopHandlerNode::broadcastEmergencyStop() {
    if (!config_.enable_mesh_broadcast) return;
    
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    swarm_msgs::msg::MeshMessage mesh_msg;
    mesh_msg.timestamp_us = now_us;
    mesh_msg.source_id = drone_id_;
    mesh_msg.target_id = 0;  // Broadcast
    mesh_msg.type = 10;  // EMERGENCY_STOP
    mesh_msg.priority = 3;  // Critical
    mesh_msg.ttl = 5;
    
    // Payload: emergency stop data
    std::string payload = "ESTOP:" + std::to_string(drone_id_) + ":" + state_.reason;
    mesh_msg.payload = std::vector<uint8_t>(payload.begin(), payload.end());
    
    // Send via mesh publisher
    mesh_pub_->publish(mesh_msg);
    
    state_.mesh_broadcast_sent = true;
    state_.last_broadcast_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    state_.broadcast_retries++;
    
    RCLCPP_INFO(get_logger(), "Emergency stop broadcast sent (retry %d/%d)", 
                state_.broadcast_retries, config_.broadcast_retry_count);
}

void EmergencyStopHandlerNode::deployParachute() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (state_.parachute_deployed) return;
    
    state_.parachute_deployed = true;
    
    // Send MAVLink parachute deploy command
    sendMAVLinkCommand(207, {static_cast<float>(config_.parachute_servo_channel), 
                             static_cast<float>(config_.parachute_deploy_pwm), 0, 0, 0, 0, 0});
    // MAV_CMD_DO_SET_SERVO
    
    RCLCPP_WARN(get_logger(), "PARACHUTE DEPLOYED!");
    
    // Publish safety event
    auto event = swarm_msgs::msg::SafetyEvent();
    event.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    event.source_id = drone_id_;
    event.type = 1;
    event.severity = 3;
    event.details = "Parachute deployed";
    safety_event_pub_->publish(event);
}

void EmergencyStopHandlerNode::killMotors() {
    // Set all motors to minimum PWM (disarmed)
    for (int i = 0; i < config_.motor_count; ++i) {
        sendMAVLinkCommand(207, {static_cast<float>(i), config_.motor_kill_pwm, 0, 0, 0, 0, 0});
    }
    
    // Also send disarm command
    sendMAVLinkLong(400, {0, 21196, 0, 0, 0, 0, 0});  // MAV_CMD_COMPONENT_ARM_DISARM
    
    RCLCPP_WARN(get_logger(), "MOTORS KILLED - PWM set to %.0f", config_.motor_kill_pwm);
}

void EmergencyStopHandlerNode::checkAckTimeouts() {
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::lock_guard<std::mutex> lock(acks_mutex_);
    
    int missing_acks = 0;
    for (auto& [id, ack] : pending_acks_) {
        if (!ack.ack_time_us) {  // Not received yet
            missing_acks++;
        }
    }
    
    // Check if we've exceeded timeout
    if (state_.active && now_us - state_.last_broadcast_us > static_cast<uint64_t>(config_.ack_timeout_ms) * 1000) {
        if (missing_acks >= config_.max_missing_acks_before_escalation && !state_.escalated) {
            escalateIfNeeded();
        }
    }
}

void EmergencyStopHandlerNode::checkBroadcastRetries() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    if (!state_.active || state_.mesh_broadcast_sent) return;
    
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    if (now_us - state_.last_broadcast_us > static_cast<uint64_t>(config_.broadcast_retry_interval_ms) * 1000) {
        if (state_.broadcast_retries < config_.broadcast_retry_count) {
            broadcastEmergencyStop();
        } else if (!state_.escalated) {
            RCLCPP_ERROR(get_logger(), "Broadcast retries exhausted, escalating");
            escalateIfNeeded();
        }
    }
}

void EmergencyStopHandlerNode::escalateIfNeeded() {
    if (state_.escalated) return;
    
    state_.escalated = true;
    
    RCLCPP_ERROR(get_logger(), "ESCALATION: Emergency stop not acknowledged by all members");
    
    // Could trigger additional actions:
    // - Increase broadcast power
    // - Switch to 5G/LoRa backup
    // - Trigger ground station alert
    
    // Publish escalation event
    auto event = swarm_msgs::msg::SafetyEvent();
    event.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    event.source_id = drone_id_;
    event.type = 1;
    event.severity = 3;
    event.details = "Emergency stop escalation: ACK timeout";
    safety_event_pub_->publish(event);
}

void EmergencyStopHandlerNode::handleIncomingEmergencyStop(const swarm_msgs::msg::MeshMessage& msg) {
    // Parse payload
    std::string payload(msg.payload.begin(), msg.payload.end());
    
    // Send ACK back to source
    swarm_msgs::msg::EmergencyStopAck ack;
    ack.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    ack.source_id = drone_id_;
    ack.target_id = msg.source_id;
    ack.emergency_stop_id = 0;  // Would parse from payload
    ack.acknowledged = true;
    ack.parachute_deployed = state_.parachute_deployed;
    ack.status = "received";
    
    ack_pub_->publish(ack);
    
    // Also trigger local emergency stop if not already active
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!state_.active) {
        triggerEmergencyStop("Received from drone " + std::to_string(msg.source_id), msg.source_id);
    }
}

void EmergencyStopHandlerNode::handleAck(const swarm_msgs::msg::EmergencyStopAck& ack) {
    std::lock_guard<std::mutex> lock(acks_mutex_);
    
    auto it = pending_acks_.find(ack.source_id);
    if (it != pending_acks_.end()) {
        it->second.ack_time_us = ack.timestamp_us;
        it->second.parachute_deployed = ack.parachute_deployed;
        it->second.status = "acknowledged";
        
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.acks_received[ack.source_id] = true;
        }
        
        RCLCPP_INFO(get_logger(), "ACK received from drone %d", ack.source_id);
    }
}

void EmergencyStopHandlerNode::sendMeshMessage(uint16_t target_id, const swarm_msgs::msg::MeshMessage& msg) {
    auto msg_copy = msg;
    msg_copy.target_id = target_id;
    mesh_pub_->publish(msg_copy);
}

void EmergencyStopHandlerNode::sendMAVLinkCommand(uint16_t command, const std::vector<float>& params) {
    mavros_msgs::msg::CommandLong msg;
    msg.command = command;
    msg.target_system = drone_id_;
    msg.target_component = 1;
    msg.confirmation = 0;
    
    for (size_t i = 0; i < params.size() && i < 7; ++i) {
        switch (i) {
            case 0: msg.param1 = params[i]; break;
            case 1: msg.param2 = params[i]; break;
            case 2: msg.param3 = params[i]; break;
            case 3: msg.param4 = params[i]; break;
            case 4: msg.param5 = params[i]; break;
            case 5: msg.param6 = params[i]; break;
            case 6: msg.param7 = params[i]; break;
        }
    }
    
    mavlink_cmd_pub_->publish(msg);
}

void EmergencyStopHandlerNode::sendMAVLinkLong(uint16_t command, const std::vector<float>& params) {
    sendMAVLinkCommand(command, params);
}

void EmergencyStopHandlerNode::publishStatus() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    
    auto msg = swarm_msgs::msg::EmergencyStop();
    msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    msg.drone_id = drone_id_;
    msg.active = state_.active;
    msg.source_id = state_.source_id;
    msg.reason = state_.reason;
    msg.parachute_deployed = state_.parachute_deployed;
    msg.mesh_broadcast_sent = state_.mesh_broadcast_sent;
    msg.broadcast_retries = state_.broadcast_retries;
    msg.escalated = state_.escalated;
    msg.trigger_time_us = state_.trigger_time_us;
    
    // Pending ACKs count
    {
        std::lock_guard<std::mutex> lock(acks_mutex_);
        msg.pending_acks = pending_acks_.size();
        for (const auto& [id, ack] : pending_acks_) {
            msg.ack_received_ids.push_back(id);
        }
    }
    
    emergency_stop_pub_->publish(msg);
}

}  // namespace swarm::safety

#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarm::safety::EmergencyStopHandlerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}