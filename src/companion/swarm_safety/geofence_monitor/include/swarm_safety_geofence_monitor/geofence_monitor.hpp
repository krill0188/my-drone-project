#ifndef GEOFENCE_MONITOR_HPP
#define GEOFENCE_MONITOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/agent_state.hpp>
#include <swarm_msgs/msg/geofence_status.hpp>
#include <swarm_msgs/msg/geofence_breach.hpp>
#include <swarm_msgs/msg/ai_inference.hpp>
#include <mavros_msgs/msg/command_long.hpp>
#include <vector>
#include <string>
#include <mutex>
#include <optional>

namespace swarm::safety {

struct GeofencePolygon {
    uint32_t id;
    std::vector<std::array<double, 3>> vertices;  // ENU frame [x, y, z]
    double min_altitude_m;
    double max_altitude_m;
    bool is_exclusion;  // true = no-fly zone, false = inclusion zone
};

struct GeofenceConfig {
    std::vector<GeofencePolygon> polygons;
    double buffer_distance_m = 10.0;
    bool enabled = true;
    bool trigger_rtl = true;
    double rtl_altitude_m = 50.0;
    int update_rate_hz = 50;
};

enum class GeofenceAction : uint8_t {
    NONE = 0,
    WARNING = 1,
    RTL = 2,
    LAND = 3,
    TERMINATE = 4
};

enum class GeofenceSeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3
};

struct GeofenceStatus {
    uint64_t timestamp_us;
    uint16_t drone_id;
    bool inside_geofence;
    GeofenceAction current_action;
    GeofenceSeverity severity;
    double min_distance_to_boundary_m;
    uint32_t nearest_polygon_id;
    std::vector<double> distances_to_polygons;
    bool rtl_triggered;
    uint64_t rtl_trigger_time_us;
};

class GeofenceMonitorNode : public rclcpp::Node {
public:
    GeofenceMonitorNode();
    ~GeofenceMonitorNode() = default;

private:
    // Configuration
    void declareParameters();
    void loadParameters();
    bool loadGeofenceFromFile(const std::string& path);

    // ROS interfaces
    void setupSubscriptions();
    void setupPublishers();
    void setupServices();

    // Callbacks
    void agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg);
    void geofenceConfigCallback(const std::shared_ptr<rmw_request_id_t> request_header,
                                 const std::shared_ptr<swarm_msgs::srv::UpdateGeofence::Request> request,
                                 std::shared_ptr<swarm_msgs::srv::UpdateGeofence::Response> response);

    // Core logic
    void checkGeofence(const Eigen::Vector3d& position_enu);
    GeofenceAction evaluatePosition(const Eigen::Vector3d& position_enu) const;
    double pointToPolygonDistance(const Eigen::Vector3d& position, const GeofencePolygon& poly) const;
    bool pointInPolygon(const Eigen::Vector3d& position, const GeofencePolygon& poly) const;
    void triggerRTL(const std::string& reason);
    void publishStatus();

    // MAVLink command
    void sendMAVLinkCommand(uint16_t command, const std::vector<float>& params);

    // State
    GeofenceConfig config_;
    mutable std::mutex config_mutex_;
    GeofenceStatus current_status_;
    Eigen::Vector3d current_position_;
    bool position_received_ = false;
    uint16_t drone_id_ = 1;

    // ROS interfaces
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    rclcpp::Publisher<swarm_msgs::msg::GeofenceStatus>::SharedPtr geofence_status_pub_;
    rclcpp::Publisher<swarm_msgs::msg::GeofenceBreach>::SharedPtr geofence_breach_pub_;
    rclcpp::Publisher<mavros_msgs::msg::CommandLong>::SharedPtr mavlink_cmd_pub_;
    rclcpp::Service<swarm_msgs::srv::UpdateGeofence>::SharedPtr update_geofence_srv_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr check_timer_;

    // Stats
    uint64_t check_count_ = 0;
    uint64_t breach_count_ = 0;
    uint64_t rtl_trigger_count_ = 0;
};

}  // namespace swarm::safety

#endif  // GEOFENCE_MONITOR_HPP