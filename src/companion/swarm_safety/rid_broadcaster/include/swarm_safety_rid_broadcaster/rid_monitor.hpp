#ifndef RID_MONITOR_HPP
#define RID_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/agent_state.hpp>
#include <swarm_msgs/msg/rid_frame.hpp>
#include <swarm_msgs/msg/rid_status.hpp>
#include <swarm_msgs/msg/safety_event.hpp>
#include <mavros_msgs/msg/rid_frame.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <unordered_map>
#include <chrono>

namespace swarm::safety {

// ASTM F3411 / EU DRI compliant Remote ID frame
struct RIDFrame {
    uint64_t timestamp_us;
    uint16_t source_id;  // Drone ID that transmitted
    
    // Basic ID (Required)
    std::string uas_id;           // 20 char max (Serial or Session ID)
    uint8_t id_type;              // 1=Serial, 2=CAA, 3=Session, 4=Operator
    std::string operator_id;      // 20 char max
    uint8_t uas_type;             // 1=Airplane, 2=Rotorcraft, 3=Gyroplane, 4=Hybrid, 5=Other
    
    // Location (Required)
    double latitude_deg;
    double longitude_deg;
    float altitude_m;       // AMSL (barometric)
    float height_m;         // AGL (GPS/Baro fusion)
    float velocity_north_mps;
    float velocity_east_mps;
    float velocity_down_mps;
    float horizontal_accuracy_m;
    float vertical_accuracy_m;
    float speed_accuracy_mps;
    uint8_t status;         // 0=Normal, 1=Emergency, 2=LostLink
    uint8_t direction;      // Track angle (degrees * 100)
    float ground_speed_mps;
    
    // System (Optional)
    uint16_t sequence;
    uint8_t auth_type;      // 0=none, 1=public key, 2=signature
    std::vector<uint8_t> auth_data;  // 64 bytes max
    
    // Validation
    bool valid = false;
    std::string validation_error;
};

struct RIDMonitorConfig {
    // Validation
    bool enable_frame_validation = true;
    bool enable_utm_interface = true;
    bool enable_spoofing_detection = true;
    double max_position_jump_m = 50.0;      // Max position change between frames
    double max_speed_mps = 100.0;           // Max speed
    double max_acceleration_mps2 = 20.0;    // Max acceleration
    int max_frame_age_ms = 5000;            // Frame timeout
    double min_horizontal_accuracy_m = 100.0;
    double min_vertical_accuracy_m = 50.0;
    
    // UTM
    std::string utm_endpoint;
    std::string utm_auth_token;
    int utm_report_interval_ms = 1000;
    
    // Spoofing detection
    double spoof_distance_threshold_m = 1000.0;  // Distance from known position
    int max_missing_frames = 10;
    
    // Broadcast
    int broadcast_rate_hz = 1;
    std::string own_uas_id;
    std::string own_operator_id;
    uint8_t own_uas_type = 2;  // Rotorcraft
};

struct RIDTrack {
    uint16_t source_id;
    RIDFrame last_frame;
    uint64_t last_update_us;
    int missing_frames = 0;
    bool is_spoofed = false;
    std::vector<RIDFrame> history;  // Last 10 frames for validation
    Eigen::Vector3d last_velocity_enu;
    Eigen::Vector3d last_acceleration_enu;
};

struct RIDMonitorStats {
    uint64_t frames_received = 0;
    uint64_t frames_valid = 0;
    uint64_t frames_invalid = 0;
    uint64_t spoofing_detected = 0;
    uint64_t utm_reports_sent = 0;
    uint64_t utm_reports_failed = 0;
    uint64_t own_frames_broadcast = 0;
};

class RIDMonitorNode : public rclcpp::Node {
public:
    RIDMonitorNode();
    ~RIDMonitorNode() = default;

private:
    // Configuration
    void declareParameters();
    void loadParameters();
    
    // ROS interfaces
    void setupSubscriptions();
    void setupPublishers();
    void setupServices();
    
    // Callbacks
    void mavlinkRIDCallback(const mavros_msgs::msg::RIDFrame::SharedPtr msg);
    void agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg);
    void safetyEventCallback(const swarm_msgs::msg::SafetyEvent::SharedPtr msg);
    void configCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                        const std::shared_ptr<swarm_msgs::srv::UpdateRIDConfig::Request> request,
                        std::shared_ptr<swarm_msgs::srv::UpdateRIDConfig::Response> response);
    
    // Core logic
    void validateFrame(const mavros_msgs::msg::RIDFrame::SharedPtr& mavlink_frame, RIDFrame& out_frame);
    bool validateBasicID(const RIDFrame& frame);
    bool validateLocation(const RIDFrame& frame);
    bool validateKinematics(const RIDFrame& frame);
    bool validateAuthentication(const RIDFrame& frame);
    bool checkSpoofing(const RIDFrame& frame);
    void updateTrack(const RIDFrame& frame);
    void checkTrackTimeouts();
    void broadcastOwnFrame();
    void reportToUTM(const RIDFrame& frame);
    void publishStatus();
    void publishSafetyEvent(const RIDFrame& frame, const std::string& event_type);
    
    // MAVLink conversion
    RIDFrame convertMAVLinkFrame(const mavros_msgs::msg::RIDFrame::SharedPtr& msg);
    mavros_msgs::msg::RIDFrame convertToMAVLink(const RIDFrame& frame);
    
    // UTM interface
    bool sendUTMReport(const RIDFrame& frame);
    
    // State
    RIDMonitorConfig config_;
    mutable std::mutex config_mutex_;
    std::unordered_map<uint16_t, RIDTrack> tracks_;
    mutable std::mutex tracks_mutex_;
    RIDMonitorStats stats_;
    mutable std::mutex stats_mutex_;
    
    // ROS interfaces
    rclcpp::Subscription<mavros_msgs::msg::RIDFrame>::SharedPtr mavlink_rid_sub_;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    rclcpp::Subscription<swarm_msgs::msg::SafetyEvent>::SharedPtr safety_event_sub_;
    rclcpp::Publisher<swarm_msgs::msg::RIDFrame>::SharedPtr rid_frame_pub_;
    rclcpp::Publisher<swarm_msgs::msg::RIDStatus>::SharedPtr rid_status_pub_;
    rclcpp::Publisher<swarm_msgs::msg::SafetyEvent>::SharedPtr safety_event_pub_;
    rclcpp::Publisher<mavros_msgs::msg::RIDFrame>::SharedPtr mavlink_rid_pub_;
    rclcpp::Service<swarm_msgs::srv::UpdateRIDConfig>::SharedPtr config_srv_;
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    rclcpp::TimerBase::SharedPtr validation_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr utm_timer_;
    
    // Own state
    uint16_t drone_id_ = 1;
    Eigen::Vector3d current_position_enu_;
    Eigen::Vector3d current_velocity_enu_;
    bool position_received_ = false;
    uint16_t own_sequence_ = 0;
};

}  // namespace swarm::safety

#endif  // RID_MONITOR_HPP