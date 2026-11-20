#include "mission_uploader.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace swarm::interface {

MissionUploader::MissionUploader(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<planner::TrajectoryPlanner> trajectory_planner,
    std::shared_ptr<planner::FormationTrajectoryPlanner> formation_planner)
    : _node(node)
    , _trajectory_planner(trajectory_planner)
    , _formation_planner(formation_planner) {
    
    // Initialize service clients
    _mission_upload_client = _node->create_client<swarm_msgs::srv::UploadMission>("/swarm/upload_mission");
    _rtl_client = _node->create_client<swarm_msgs::srv::RequestRTL>("/swarm/request_rtl");
    _estop_client = _node->create_client<swarm_msgs::srv::EmergencyStop>("/swarm/emergency_stop");
    
    // Initialize subscribers
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    
    _agent_states_sub = _node->create_subscription<swarm_msgs::msg::AgentState>(
        "/swarm/agent_states", qos,
        std::bind(&MissionUploader::agentStatesCallback, this, std::placeholders::_1));
    
    _formation_state_sub = _node->create_subscription<swarm_msgs::msg::FormationState>(
        "/swarm/formation_state", qos,
        std::bind(&MissionUploader::formationStateCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(_node->get_logger(), "Mission Uploader initialized");
}

bool MissionUploader::uploadMission(const Mission& mission, const FormationConfig& formation) {
    if (!validateMission(mission) || !validateFormation(formation)) {
        RCLCPP_ERROR(_node->get_logger(), "Mission or formation validation failed");
        return false;
    }
    
    _current_mission = mission;
    _current_formation = formation;
    _status = MissionStatus::UPLOADING;
    
    // Plan trajectory
    std::vector<planner::Waypoint> waypoints;
    for (const auto& wp : mission.waypoints) {
        planner::Waypoint wp_config;
        wp_config.position = {wp.latitude, wp.longitude, wp.altitude};  // Simplified
        wp_config.speed = wp.speed;
        wp_config.loiter_time = wp.loiter_time;
        wp_config.acceptance_radius = wp.acceptance_radius;
        waypoints.push_back(wp_config);
    }
    
    planner::Trajectory trajectory;
    if (formation.geometry_type != 0 && _formation_planner) {
        // Leader with formation
        std::vector<planner::FormationSlot> slots;  // Would be populated from formation
        trajectory = _formation_planner->planFormationTrajectory(waypoints, mission.known_obstacles, slots);
    } else {
        // Single drone or leaderless
        trajectory = _trajectory_planner->planMission(waypoints, mission.known_obstacles);
    }
    
    if (!trajectory.valid) {
        RCLCPP_ERROR(_node->get_logger(), "Trajectory planning failed");
        _status = MissionStatus::FAILED;
        if (_mission_cb) _mission_cb(mission, false, "Trajectory planning failed");
        return false;
    }
    
    // Build and send upload request
    auto request = buildUploadRequest(mission, formation);
    
    if (!_mission_upload_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(_node->get_logger(), "Mission upload service not available");
        _status = MissionStatus::FAILED;
        if (_mission_cb) _mission_cb(mission, false, "Service unavailable");
        return false;
    }
    
    auto future = _mission_upload_client->async_send_request(request);
    future.wait_for(std::chrono::seconds(10));
    
    if (future.valid()) {
        auto response = future.get();
        if (response->success) {
            _status = MissionStatus::ACTIVE;
            RCLCPP_INFO(_node->get_logger(), "Mission uploaded successfully");
            if (_mission_cb) _mission_cb(mission, true, response->message);
            return true;
        } else {
            _status = MissionStatus::FAILED;
            RCLCPP_ERROR(_node->get_logger(), "Mission upload failed: %s", response->message.c_str());
            if (_mission_cb) _mission_cb(mission, false, response->message);
            return false;
        }
    }
    
    _status = MissionStatus::FAILED;
    return false;
}

void MissionUploader::uploadMissionAsync(const Mission& mission, const FormationConfig& formation,
                                        MissionCallback callback) {
    _mission_cb = callback;
    // In real implementation, would use async with callback
    // For now, synchronous
    bool result = uploadMission(mission, formation);
    if (callback) callback(mission, result, result ? "Success" : "Failed");
}

bool MissionUploader::cancelMission() {
    if (_status == MissionStatus::ACTIVE || _status == MissionStatus::PAUSED) {
        // Send abort command to all drones
        // In real implementation, would use mission abort service
        _status = MissionStatus::ABORTED;
        RCLCPP_INFO(_node->get_logger(), "Mission cancelled");
        return true;
    }
    return false;
}

bool MissionUploader::pauseMission() {
    if (_status == MissionStatus::ACTIVE) {
        _status = MissionStatus::PAUSED;
        RCLCPP_INFO(_node->get_logger(), "Mission paused");
        return true;
    }
    return false;
}

bool MissionUploader::resumeMission() {
    if (_status == MissionStatus::PAUSED) {
        _status = MissionStatus::ACTIVE;
        RCLCPP_INFO(_node->get_logger(), "Mission resumed");
        return true;
    }
    return false;
}

bool MissionUploader::requestRTL(uint16_t target_id) {
    if (!_rtl_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(_node->get_logger(), "RTL service not available");
        return false;
    }
    
    auto request = std::make_shared<swarm_msgs::srv::RequestRTL::Request>();
    request->target_id = target_id;
    request->immediate = false;
    
    auto future = _rtl_client->async_send_request(request);
    future.wait_for(std::chrono::seconds(5));
    
    if (future.valid()) {
        auto response = future.get();
        return response->success;
    }
    return false;
}

bool MissionUploader::emergencyStop(uint16_t target_id, bool deploy_parachute) {
    if (!_estop_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(_node->get_logger(), "Emergency stop service not available");
        return false;
    }
    
    auto request = std::make_shared<swarm_msgs::srv::EmergencyStop::Request>();
    request->target_id = target_id;
    request->deploy_parachute = deploy_parachute;
    request->reason = "Mission uploader emergency stop";
    
    auto future = _estop_client->async_send_request(request);
    future.wait_for(std::chrono::seconds(3));
    
    if (future.valid()) {
        auto response = future.get();
        if (response->success) {
            _status = MissionStatus::ABORTED;
            RCLCPP_WARN(_node->get_logger(), "Emergency stop executed");
        }
        return response->success;
    }
    return false;
}

bool MissionUploader::validateMission(const Mission& mission) {
    if (mission.waypoints.empty()) {
        RCLCPP_ERROR(_node->get_logger(), "Mission has no waypoints");
        return false;
    }
    
    // Check altitude constraints
    for (const auto& wp : mission.waypoints) {
        if (wp.altitude > 120.0f) {  // Regulatory limit
            RCLCPP_ERROR(_node->get_logger(), "Waypoint altitude exceeds limit: %.1fm", wp.altitude);
            return false;
        }
        if (wp.speed > 15.0f) {
            RCLCPP_WARN(_node->get_logger(), "Waypoint speed high: %.1fm/s", wp.speed);
        }
    }
    
    // Check geofence
    if (!mission.constraints.geofence.isValid()) {
        RCLCPP_WARN(_node->get_logger(), "Mission geofence not defined");
    }
    
    return true;
}

bool MissionUploader::validateFormation(const FormationConfig& formation) {
    if (formation.geometry_type > 4) {
        RCLCPP_ERROR(_node->get_logger(), "Invalid geometry type: %d", formation.geometry_type);
        return false;
    }
    
    if (formation.spacing_m < 1.0f || formation.spacing_m > 50.0f) {
        RCLCPP_ERROR(_node->get_logger(), "Invalid spacing: %.1fm", formation.spacing_m);
        return false;
    }
    
    if (formation.geometry_type == 4 && formation.custom_offsets.empty()) {
        RCLCPP_ERROR(_node->get_logger(), "Custom geometry requires custom_offsets");
        return false;
    }
    
    return true;
}

swarm_msgs::srv::UploadMission::Request::SharedPtr 
MissionUploader::buildUploadRequest(const Mission& mission, const FormationConfig& formation) {
    auto request = std::make_shared<swarm_msgs::srv::UploadMission::Request>();
    
    // Convert waypoints
    for (const auto& wp : mission.waypoints) {
        swarm_msgs::msg::MissionWaypoint msg_wp;
        msg_wp.sequence = wp.sequence;
        msg_wp.latitude = wp.latitude;
        msg_wp.longitude = wp.longitude;
        msg_wp.altitude = wp.altitude;
        msg_wp.speed = wp.speed;
        msg_wp.loiter_time = wp.loiter_time;
        msg_wp.acceptance_radius = wp.acceptance_radius;
        msg_wp.frame = wp.frame;
        msg_wp.command = wp.command;
        request->waypoints.push_back(msg_wp);
    }
    
    // Convert obstacles
    for (const auto& obs : mission.known_obstacles) {
        swarm_msgs::msg::Obstacle msg_obs;
        msg_obs.position = {obs.center.x, obs.center.y, obs.center.z};
        msg_obs.size = {obs.size.x, obs.size.y, obs.size.z};
        msg_obs.type = static_cast<uint8_t>(obs.type);
        request->obstacles.push_back(msg_obs);
    }
    
    // Formation config
    request->formation_type = formation.geometry_type;
    request->formation_spacing = formation.spacing_m;
    request->formation_altitude_offset = formation.altitude_offset_m;
    request->formation_heading_offset = formation.heading_offset_deg;
    
    for (const auto& offset : formation.custom_offsets) {
        geometry_msgs::msg::Vector3 v;
        v.x = offset.x;
        v.y = offset.y;
        v.z = offset.z;
        request->custom_offsets.push_back(v);
    }
    
    // Constraints
    request->geofence = mission.constraints.geofence;
    request->max_altitude = mission.constraints.max_altitude_m;
    request->min_altitude = mission.constraints.min_altitude_m;
    request->max_speed = mission.constraints.max_speed_mps;
    request->max_duration = mission.constraints.max_duration_s;
    request->min_battery_pct = mission.constraints.min_battery_pct;
    request->require_rid = mission.constraints.require_rid;
    
    return request;
}

void MissionUploader::agentStatesCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
    // Monitor agent states for mission progress
    // Could trigger replan, RTL, etc.
}

void MissionUploader::formationStateCallback(const swarm_msgs::msg::FormationState::SharedPtr msg) {
    // Monitor formation health
    // Could detect formation loss, trigger reform
}

void MissionUploader::handleUploadResponse(rclcpp::Client<swarm_msgs::srv::UploadMission>::SharedFuture future) {
    if (future.valid()) {
        auto response = future.get();
        if (_mission_cb) {
            _mission_cb(_current_mission, response->success, response->message);
        }
    }
}

// Mission loading from YAML file
namespace swarm::interface {

std::optional<Mission> loadMissionFromYaml(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        
        Mission mission;
        mission.mission_id = config["mission_id"].as<std::string>();
        mission.name = config["name"].as<std::string>();
        mission.type = config["type"].as<uint8_t>();
        mission.priority = config["priority"].as<uint8_t>();
        mission.created_by = config["created_by"].as<std::string>();
        mission.created_at = Timestamp::nowUs() / 1000000;
        
        // Parse waypoints
        if (config["waypoints"]) {
            for (const auto& wp_node : config["waypoints"]) {
                Waypoint wp;
                wp.sequence = wp_node["sequence"].as<uint32_t>();
                wp.latitude = wp_node["latitude"].as<double>();
                wp.longitude = wp_node["longitude"].as<double>();
                wp.altitude = wp_node["altitude"].as<float>();
                wp.speed = wp_node["speed"].as<float>();
                wp.loiter_time = wp_node["loiter_time"].as<float>();
                wp.acceptance_radius = wp_node["acceptance_radius"].as<float>();
                wp.frame = wp_node["frame"].as<uint8_t>();
                wp.command = wp_node["command"].as<uint8_t>();
                mission.waypoints.push_back(wp);
            }
        }
        
        // Parse constraints
        if (config["constraints"]) {
            auto c = config["constraints"];
            mission.constraints.max_altitude_m = c["max_altitude"].as<float>();
            mission.constraints.min_altitude_m = c["min_altitude"].as<float>();
            mission.constraints.max_speed_mps = c["max_speed"].as<float>();
            mission.constraints.max_duration_s = c["max_duration"].as<uint32_t>();
            mission.constraints.min_battery_pct = c["min_battery_pct"].as<float>();
            mission.constraints.require_rid = c["require_rid"].as<bool>();
        }
        
        return mission;
    } catch (const std::exception& e) {
        return std::nullopt;
    }
}

std::optional<FormationConfig> loadFormationFromYaml(const std::string& filepath) {
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        
        FormationConfig formation;
        formation.geometry_type = config["geometry_type"].as<uint8_t>();
        formation.spacing_m = config["spacing"].as<float>();
        formation.altitude_offset_m = config["altitude_offset"].as<float>();
        formation.heading_offset_deg = config["heading_offset"].as<float>();
        
        if (config["custom_offsets"]) {
            for (const auto& off : config["custom_offsets"]) {
                planner::Vector3 v;
                v.x = off[0].as<float>();
                v.y = off[1].as<float>();
                v.z = off[2].as<float>();
                formation.custom_offsets.push_back(v);
            }
        }
        
        return formation;
    } catch (const std::exception& e) {
        return std::nullopt;
    }
}

} // namespace swarm::interface