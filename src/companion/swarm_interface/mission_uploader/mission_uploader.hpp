#ifndef MISSION_UPLOADER_HPP
#define MISSION_UPLOADER_HPP

#include <vector>
#include <array>
#include <string>
#include <optional>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/srv/upload_mission.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/msg/formation_state.hpp"
#include "trajectory_planner.hpp"

namespace swarm::interface {

struct Waypoint {
    uint32_t sequence;
    double latitude;
    double longitude;
    float altitude;       // AMSL meters
    float speed;          // m/s
    float loiter_time;    // seconds
    float acceptance_radius; // meters
    uint8_t frame;        // 0=global, 1=local ENU
    uint8_t command;      // MAVLink command (16=WAYPOINT, 21=LOITER, etc.)
};

struct Mission {
    std::string mission_id;
    std::string name;
    uint8_t type;         // 0=WAYPOINT, 1=SEARCH, 2=INSPECT, 3=PATROL, 4=ESCORT
    std::vector<Waypoint> waypoints;
    std::vector<planner::Obstacle> known_obstacles;
    planner::MissionConstraints constraints;
    uint8_t priority = 128;
    uint32_t created_at = 0;
    std::string created_by;
};

struct MissionConstraints {
    planner::Polygon geofence;
    float max_altitude_m = 120.0f;
    float min_altitude_m = 10.0f;
    float max_speed_mps = 15.0f;
    uint32_t max_duration_s = 3600;
    float min_battery_pct = 0.20f;
    std::vector<planner::NoFlyZone> no_fly_zones;
    planner::WeatherConstraints weather;
    bool require_rid = true;
    planner::Jurisdiction jurisdiction;
};

struct FormationConfig {
    uint8_t geometry_type = 1;  // 0=LINE, 1=VEE, 2=DIAMOND, 3=COLUMN, 4=CUSTOM
    float spacing_m = 5.0f;
    float altitude_offset_m = 0.0f;
    float heading_offset_deg = 0.0f;
    std::vector<planner::Vector3> custom_offsets;  // For CUSTOM geometry
};

class MissionUploader {
public:
    using MissionCallback = std::function<void(const Mission&, bool success, const std::string& message)>;
    using TrajectoryCallback = std::function<void(const planner::Trajectory&)>;
    
    MissionUploader(rclcpp::Node::SharedPtr node, 
                    std::shared_ptr<planner::TrajectoryPlanner> trajectory_planner,
                    std::shared_ptr<planner::FormationTrajectoryPlanner> formation_planner);
    
    // Upload mission to swarm
    bool uploadMission(const Mission& mission, const FormationConfig& formation);
    
    // Upload mission with callback for async result
    void uploadMissionAsync(const Mission& mission, const FormationConfig& formation,
                           MissionCallback callback);
    
    // Cancel current mission
    bool cancelMission();
    
    // Pause/resume mission
    bool pauseMission();
    bool resumeMission();
    
    // Get current mission status
    enum class MissionStatus { IDLE, UPLOADING, ACTIVE, PAUSED, COMPLETED, FAILED, ABORTED };
    MissionStatus getMissionStatus() const { return _status; }
    const Mission& getCurrentMission() const { return _current_mission; }
    
    // Set callbacks
    void setMissionCallback(MissionCallback cb) { _mission_cb = cb; }
    void setTrajectoryCallback(TrajectoryCallback cb) { _trajectory_cb = cb; }
    
    // Request RTL for specific drone or entire swarm
    bool requestRTL(uint16_t target_id = 0);  // 0 = all
    
    // Emergency stop
    bool emergencyStop(uint16_t target_id = 0, bool deploy_parachute = false);

private:
    rclcpp::Node::SharedPtr _node;
    std::shared_ptr<planner::TrajectoryPlanner> _trajectory_planner;
    std::shared_ptr<planner::FormationTrajectoryPlanner> _formation_planner;
    
    MissionStatus _status = MissionStatus::IDLE;
    Mission _current_mission;
    FormationConfig _current_formation;
    FormationConfig::geometry_type _geometry_type;
    
    MissionCallback _mission_cb;
    TrajectoryCallback _trajectory_cb;
    
    // ROS 2 service clients
    rclcpp::Client<swarm_msgs::srv::UploadMission>::SharedPtr _mission_upload_client;
    rclcpp::Client<swarm_msgs::srv::RequestRTL>::SharedPtr _rtl_client;
    rclcpp::Client<swarm_msgs::srv::EmergencyStop>::SharedPtr _estop_client;
    
    // ROS 2 subscribers for status
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr _agent_states_sub;
    rclcpp::Subscription<swarm_msgs::msg::FormationState>::SharedPtr _formation_state_sub;
    
    // Helper functions
    bool validateMission(const Mission& mission);
    bool validateFormation(const FormationConfig& formation);
    swarm_msgs::srv::UploadMission::Request::SharedPtr buildUploadRequest(const Mission& mission, 
                                                                            const FormationConfig& formation);
    void agentStatesCallback(const swarm_msgs::msg::AgentState::SharedPtr msg);
    void formationStateCallback(const swarm_msgs::msg::FormationState::SharedPtr msg);
    void handleUploadResponse(rclcpp::Client<swarm_msgs::srv::UploadMission>::SharedFuture future);
};

} // namespace swarm::interface

#endif // MISSION_UPLOADER_HPP