#ifndef EMERGENCY_STOP_HANDLER_HPP
#define EMERGENCY_STOP_HANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/agent_state.hpp>
#include <swarm_msgs/msg/safety_event.hpp>
#include <swarm_msgs/msg/emergency_stop.hpp>
#include <swarm_msgs/msg/emergency_stop_ack.hpp>
#include <swarm_msgs/srv/trigger_emergency_stop.hpp>
#include <swarm_msgs/srv/reset_emergency_stop.hpp>
#include <mavros_msgs/msg/command_long.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <chrono>

namespace swarm::safety {

struct EmergencyStopConfig {
    // Mesh broadcast
    bool enable_mesh_broadcast = true;
    bool enable_5g_broadcast = false;
    int broadcast_retry_count = 3;
    int broadcast_retry_interval_ms = 100;
    int broadcast_timeout_ms = 1000;
    
    // ACK tracking
    int ack_timeout_ms = 2000;
    int max_missing_acks_before_escalation = 2;
    bool require_all_acks = true;
    
    // Parachute
    bool deploy_parachute = false;
    float parachute_delay_s = 1.0;
    int parachute_servo_channel = 0;
    int parachute_deploy_pwm = 2000;
    
    // Local actions
    bool local_motor_kill = true;
    float motor_kill_pwm = 1000.0f;  // Disarmed PWM
    int motor_count = 8;
    
    // Reset
    bool require_ground_station_reset = true;
    int reset_cooldown_ms = 5000;
    
    // Rate
    int status_publish_rate_hz = 10;
};

struct EmergencyStopState {
    uint64_t timestamp_us;
    uint16_t source_id;
    std::string reason;
    bool active = false;
    bool parachute_deployed = false;
    bool mesh_broadcast_sent = false;
    std::unordered_map<uint16_t, bool> acks_received;
    uint64_t trigger_time_us = 0;
    uint64_t last_broadcast_us = 0;
    int broadcast_retries = 0;
    bool escalated = false;
};

struct AckInfo {
    uint16_t drone_id;
    uint64_t ack_time_us;
    bool parachute_deployed;
    std::string status;
};

class EmergencyStopHandlerNode : public rclcpp::Node {
public:
    EmergencyStopHandlerNode();
    ~EmergencyStopHandlerNode() = default;

private:
    // Configuration
    void declareParameters();
    void loadParameters();
    
    // ROS interfaces
    void setupSubscriptions();
    void setupPublishers();
    void setupServices();
    
    // Callbacks
    void agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg);
    void safetyEventCallback(const swarm_msgs::msg::SafetyEvent::SharedPtr msg);
    void emergencyStopAckCallback(const swarm_msgs::msg::EmergencyStopAck::SharedPtr msg);
    void meshMessageCallback(const swarm_msgs::msg::MeshMessage::SharedPtr msg);
    
    void triggerServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                                const std::shared_ptr<swarm_msgs::srv::TriggerEmergencyStop::Request> request,
                                std::shared_ptr<swarm_msgs::srv::TriggerEmergencyStop::Response> response);
    void resetServiceCallback(const std::shared_ptr<rmw_request_id_t>,
                              const std::shared_ptr<swarm_msgs::srv::ResetEmergencyStop::Request> request,
                              std::shared_ptr<swarm_msgs::srv::ResetEmergencyStop::Response> response);
    
    // Core logic
    void triggerEmergencyStop(const std::string& reason, uint16_t source_id);
    void resetEmergencyStop(uint16_t requester_id);
    void broadcastEmergencyStop();
    void deployParachute();
    void killMotors();
    void checkAckTimeouts();
    void checkBroadcastRetries();
    void escalateIfNeeded();
    void publishStatus();
    
    // Mesh communication
    void sendMeshMessage(uint16_t target_id, const swarm_msgs::msg::MeshMessage& msg);
    void handleIncomingEmergencyStop(const swarm_msgs::msg::MeshMessage& msg);
    void handleAck(const swarm_msgs::msg::EmergencyStopAck& ack);
    
    // MAVLink commands
    void sendMAVLinkCommand(uint16_t command, const std::vector<float>& params);
    void sendMAVLinkLong(uint16_t command, const std::vector<float>& params);
    
    // State
    EmergencyStopConfig config_;
    EmergencyStopState state_;
    mutable std::mutex state_mutex_;
    
    std::unordered_map<uint16_t, AckInfo> pending_acks_;
    mutable std::mutex acks_mutex_;
    
    // ROS interfaces
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    rclcpp::Subscription<swarm_msgs::msg::SafetyEvent>::SharedPtr safety_event_sub_;
    rclcpp::Subscription<swarm_msgs::msg::EmergencyStopAck>::SharedPtr ack_sub_;
    rclcpp::Subscription<swarm_msgs::msg::MeshMessage>::SharedPtr mesh_sub_;
    
    rclcpp::Publisher<swarm_msgs::msg::EmergencyStop>::SharedPtr emergency_stop_pub_;
    rclcpp::Publisher<swarm_msgs::msg::EmergencyStopAck>::SharedPtr ack_pub_;
    rclcpp::Publisher<swarm_msgs::msg::MeshMessage>::SharedPtr mesh_pub_;
    rclcpp::Publisher<mavros_msgs::msg::CommandLong>::SharedPtr mavlink_cmd_pub_;
    rclcpp::Publisher<swarm_msgs::msg::SafetyEvent>::SharedPtr safety_event_pub_;
    
    rclcpp::Service<swarm_msgs::srv::TriggerEmergencyStop>::SharedPtr trigger_srv_;
    rclcpp::Service<swarm_msgs::srv::ResetEmergencyStop>::SharedPtr reset_srv_;
    
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    rclcpp::TimerBase::SharedPtr ack_check_timer_;
    rclcpp::TimerBase::SharedPtr retry_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    // Own state
    uint16_t drone_id_ = 1;
    Eigen::Vector3d current_position_enu_;
    Eigen::Vector3d current_velocity_enu_;
    bool position_received_ = false;
    
    // Known swarm members
    std::vector<uint16_t> swarm_members_;
    mutable std::mutex members_mutex_;
};

}  // namespace swarm::safety

#endif  // EMERGENCY_STOP_HANDLER_HPP