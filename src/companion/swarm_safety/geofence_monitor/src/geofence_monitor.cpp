#include "swarm_safety_geofence_monitor/geofence_monitor.hpp"
#include <fstream>
#include <nlohmann/json.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace swarm::safety {

using json = nlohmann::json;

GeofenceMonitorNode::GeofenceMonitorNode() : Node("geofence_monitor") {
    declareParameters();
    loadParameters();
    
    setupSubscriptions();
    setupPublishers();
    setupServices();
    
    // Timers
    check_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / config_.update_rate_hz),
        [this]() { 
            if (position_received_) {
                checkGeofence(current_position_);
            }
        });
    
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz status
        [this]() { publishStatus(); });
    
    RCLCPP_INFO(get_logger(), "Geofence Monitor Node initialized");
    RCLCPP_INFO(get_logger(), "Polygons: %zu, Buffer: %.1fm, RTL: %s", 
                config_.polygons.size(), config_.buffer_distance_m,
                config_.trigger_rtl ? "enabled" : "disabled");
}

void GeofenceMonitorNode::declareParameters() {
    declare_parameter("drone_id", 1);
    declare_parameter("update_rate_hz", 50);
    declare_parameter("geofence_config_file", "");
    declare_parameter("buffer_distance_m", 10.0);
    declare_parameter("enabled", true);
    declare_parameter("trigger_rtl", true);
    declare_parameter("rtl_altitude_m", 50.0);
    declare_parameter("polygons", std::vector<double>{});  // Flattened: id, is_exclusion, min_alt, max_alt, v1x, v1y, v1z, v2x...
}

void GeofenceMonitorNode::loadParameters() {
    drone_id_ = get_parameter("drone_id").as_int();
    config_.update_rate_hz = get_parameter("update_rate_hz").as_int();
    config_.buffer_distance_m = get_parameter("buffer_distance_m").as_double();
    config_.enabled = get_parameter("enabled").as_bool();
    config_.trigger_rtl = get_parameter("trigger_rtl").as_bool();
    config_.rtl_altitude_m = get_parameter("rtl_altitude_m").as_double();
    
    // Load geofence from file if specified
    std::string config_file = get_parameter("geofence_config_file").as_string();
    if (!config_file.empty()) {
        loadGeofenceFromFile(config_file);
    } else {
        // Load from parameters (flattened array)
        auto flat_polygons = get_parameter("polygons").as_double_array();
        if (!flat_polygons.empty()) {
            parsePolygonsFromFlatArray(flat_polygons);
        }
    }
    
    // Initialize status
    current_status_.drone_id = drone_id_;
    current_status_.inside_geofence = true;
    current_status_.current_action = GeofenceAction::NONE;
    current_status_.severity = GeofenceSeverity::INFO;
    current_status_.min_distance_to_boundary_m = 1e9;
    current_status_.nearest_polygon_id = 0;
    current_status_.rtl_triggered = false;
    current_status_.rtl_trigger_time_us = 0;
}

bool GeofenceMonitorNode::loadGeofenceFromFile(const std::string& path) {
    try {
        std::ifstream file(path);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open geofence config: %s", path.c_str());
            return false;
        }
        
        json j;
        file >> j;
        
        config_.polygons.clear();
        for (const auto& poly_json : j["polygons"]) {
            GeofencePolygon poly;
            poly.id = poly_json["id"];
            poly.is_exclusion = poly_json["is_exclusion"];
            poly.min_altitude_m = poly_json["min_altitude_m"];
            poly.max_altitude_m = poly_json["max_altitude_m"];
            
            for (const auto& v : poly_json["vertices"]) {
                poly.vertices.push_back({v[0], v[1], v[2]});
            }
            
            if (poly.vertices.size() >= 3) {
                config_.polygons.push_back(poly);
            }
        }
        
        RCLCPP_INFO(get_logger(), "Loaded %zu polygons from %s", config_.polygons.size(), path.c_str());
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Failed to parse geofence config: %s", e.what());
        return false;
    }
}

void GeofenceMonitorNode::parsePolygonsFromFlatArray(const std::vector<double>& flat) {
    config_.polygons.clear();
    size_t idx = 0;
    
    while (idx < flat.size()) {
        if (idx + 6 >= flat.size()) break;  // Need at least id, is_excl, min_alt, max_alt, 3 vertices * 3 coords
        
        GeofencePolygon poly;
        poly.id = static_cast<uint32_t>(flat[idx++]);
        poly.is_exclusion = static_cast<bool>(flat[idx++]);
        poly.min_altitude_m = flat[idx++];
        poly.max_altitude_m = flat[idx++];
        
        // Remaining are vertices (groups of 3)
        while (idx + 2 < flat.size()) {
            poly.vertices.push_back({flat[idx], flat[idx+1], flat[idx+2]});
            idx += 3;
        }
        
        if (poly.vertices.size() >= 3) {
            config_.polygons.push_back(poly);
        }
    }
    
    RCLCPP_INFO(get_logger(), "Parsed %zu polygons from flat array", config_.polygons.size());
}

void GeofenceMonitorNode::setupSubscriptions() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
        "/swarm/agent_states", qos,
        [this](const swarm_msgs::msg::AgentState::SharedPtr msg) {
            if (msg->sysid == drone_id_) {
                agentStateCallback(msg);
            }
        });
}

void GeofenceMonitorNode::setupPublishers() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    geofence_status_pub_ = create_publisher<swarm_msgs::msg::GeofenceStatus>(
        "/drone" + std::to_string(drone_id_) + "/geofence_status", qos);
    
    geofence_breach_pub_ = create_publisher<swarm_msgs::msg::GeofenceBreach>(
        "/drone" + std::to_string(drone_id_) + "/geofence_breach", qos);
    
    mavlink_cmd_pub_ = create_publisher<mavros_msgs::msg::CommandLong>(
        "/drone" + std::to_string(drone_id_) + "/mavlink_command", qos);
}

void GeofenceMonitorNode::setupServices() {
    update_geofence_srv_ = create_service<swarm_msgs::srv::UpdateGeofence>(
        "/drone" + std::to_string(drone_id_) + "/update_geofence",
        [this](const std::shared_ptr<rmw_request_id_t> request_header,
               const std::shared_ptr<swarm_msgs::srv::UpdateGeofence::Request> request,
               std::shared_ptr<swarm_msgs::srv::UpdateGeofence::Response> response) {
            geofenceConfigCallback(request_header, request, response);
        });
}

void GeofenceMonitorNode::agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
    current_position_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    position_received_ = true;
}

void GeofenceMonitorNode::geofenceConfigCallback(
    const std::shared_ptr<rmw_request_id_t>,
    const std::shared_ptr<swarm_msgs::srv::UpdateGeofence::Request> request,
    std::shared_ptr<swarm_msgs::srv::UpdateGeofence::Response> response) {
    
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    config_.polygons.clear();
    for (const auto& poly_msg : request->polygons) {
        GeofencePolygon poly;
        poly.id = poly_msg.id;
        poly.is_exclusion = poly_msg.is_exclusion;
        poly.min_altitude_m = poly_msg.min_altitude_m;
        poly.max_altitude_m = poly_msg.max_altitude_m;
        
        for (const auto& v : poly_msg.vertices) {
            poly.vertices.push_back({v.x, v.y, v.z});
        }
        
        if (poly.vertices.size() >= 3) {
            config_.polygons.push_back(poly);
        }
    }
    
    config_.buffer_distance_m = request->buffer_distance_m;
    config_.enabled = request->enabled;
    config_.trigger_rtl = request->trigger_rtl;
    config_.rtl_altitude_m = request->rtl_altitude_m;
    
    response->success = true;
    response->message = "Geofence updated: " + std::to_string(config_.polygons.size()) + " polygons";
    RCLCPP_INFO(get_logger(), "Geofence config updated via service");
}

void GeofenceMonitorNode::checkGeofence(const Eigen::Vector3d& position_enu) {
    check_count_++;
    
    if (!config_.enabled || config_.polygons.empty()) {
        current_status_.inside_geofence = true;
        current_status_.current_action = GeofenceAction::NONE;
        current_status_.severity = GeofenceSeverity::INFO;
        return;
    }
    
    GeofenceAction action = evaluatePosition(position_enu);
    
    // Update status
    current_status_.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    current_status_.drone_id = drone_id_;
    current_status_.current_action = action;
    
    // Determine severity
    switch (action) {
        case GeofenceAction::NONE:
            current_status_.severity = GeofenceSeverity::INFO;
            current_status_.inside_geofence = true;
            break;
        case GeofenceAction::WARNING:
            current_status_.severity = GeofenceSeverity::WARNING;
            current_status_.inside_geofence = true;
            break;
        case GeofenceAction::RTL:
        case GeofenceAction::LAND:
            current_status_.severity = GeofenceSeverity::CRITICAL;
            current_status_.inside_geofence = false;
            break;
        case GeofenceAction::TERMINATE:
            current_status_.severity = GeofenceSeverity::EMERGENCY;
            current_status_.inside_geofence = false;
            break;
    }
    
    // Handle breach
    if (action != GeofenceAction::NONE && action != current_status_.current_action) {
        breach_count_++;
        publishBreach(action, position_enu);
    }
    
    // Trigger RTL if needed
    if (action == GeofenceAction::RTL && config_.trigger_rtl && !current_status_.rtl_triggered) {
        triggerRTL("Geofence breach");
    }
}

GeofenceAction GeofenceMonitorNode::evaluatePosition(const Eigen::Vector3d& position) const {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    GeofenceAction worst_action = GeofenceAction::NONE;
    double min_distance = 1e9;
    uint32_t nearest_poly_id = 0;
    
    current_status_.distances_to_polygons.clear();
    
    for (const auto& poly : config_.polygons) {
        // Check altitude
        bool altitude_ok = (position.z() >= poly.min_altitude_m) && 
                          (position.z() <= poly.max_altitude_m);
        if (!altitude_ok) continue;
        
        bool inside = pointInPolygon(position, poly);
        double distance = pointToPolygonDistance(position, poly);
        
        current_status_.distances_to_polygons.push_back(distance);
        
        if (distance < min_distance) {
            min_distance = distance;
            nearest_poly_id = poly.id;
        }
        
        if (poly.is_exclusion) {
            // Exclusion zone (no-fly)
            if (inside) {
                return GeofenceAction::TERMINATE;
            } else if (distance < config_.buffer_distance_m) {
                worst_action = std::max(worst_action, GeofenceAction::WARNING);
            }
        } else {
            // Inclusion zone (must stay inside)
            if (!inside) {
                if (distance > config_.buffer_distance_m) {
                    return GeofenceAction::RTL;
                }
                worst_action = std::max(worst_action, GeofenceAction::WARNING);
            }
        }
    }
    
    current_status_.min_distance_to_boundary_m = min_distance;
    current_status_.nearest_polygon_id = nearest_poly_id;
    
    return worst_action;
}

double GeofenceMonitorNode::pointToPolygonDistance(const Eigen::Vector3d& position, 
                                                    const GeofencePolygon& poly) const {
    if (poly.vertices.size() < 2) return 1e9;
    
    double min_dist = 1e9;
    Eigen::Vector2d pos_2d(position.x(), position.y());
    
    for (size_t i = 0, j = poly.vertices.size() - 1; i < poly.vertices.size(); j = i++) {
        Eigen::Vector2d a(poly.vertices[j][0], poly.vertices[j][1]);
        Eigen::Vector2d b(poly.vertices[i][0], poly.vertices[i][1]);
        
        Eigen::Vector2d ab = b - a;
        Eigen::Vector2d ap = pos_2d - a;
        double ab_len2 = ab.squaredNorm();
        
        double t = 0.0;
        if (ab_len2 > 1e-9) {
            t = std::clamp(ap.dot(ab) / ab_len2, 0.0, 1.0);
        }
        
        Eigen::Vector2d closest = a + t * ab;
        double dist = (pos_2d - closest).norm();
        
        if (dist < min_dist) min_dist = dist;
    }
    
    return min_dist;
}

bool GeofenceMonitorNode::pointInPolygon(const Eigen::Vector3d& position, 
                                          const GeofencePolygon& poly) const {
    if (poly.vertices.size() < 3) return false;
    
    bool inside = false;
    double x = position.x();
    double y = position.y();
    
    for (size_t i = 0, j = poly.vertices.size() - 1; i < poly.vertices.size(); j = i++) {
        double xi = poly.vertices[i][0], yi = poly.vertices[i][1];
        double xj = poly.vertices[j][0], yj = poly.vertices[j][1];
        
        bool intersect = ((yi > y) != (yj > y)) &&
                        (x < (xj - xi) * (y - yi) / (yj - yi) + xi);
        
        if (intersect) inside = !inside;
    }
    
    return inside;
}

void GeofenceMonitorNode::triggerRTL(const std::string& reason) {
    current_status_.rtl_triggered = true;
    current_status_.rtl_trigger_time_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    rtl_trigger_count_++;
    
    RCLCPP_WARN(get_logger(), "RTL TRIGGERED: %s (count: %lu)", reason.c_str(), rtl_trigger_count_);
    
    // Send MAVLink RTL command
    sendMAVLinkCommand(20, {0, 0, 0, 0, 0, 0, 0});  // MAV_CMD_NAV_RETURN_TO_LAUNCH
}

void GeofenceMonitorNode::sendMAVLinkCommand(uint16_t command, const std::vector<float>& params) {
    auto msg = mavros_msgs::msg::CommandLong();
    msg.command = command;
    msg.target_system = drone_id_;
    msg.target_component = 1;
    msg.confirmation = 0;
    
    for (size_t i = 0; i < params.size() && i < 7; ++i) {
        msg.param1 = params[0];
        msg.param2 = params.size() > 1 ? params[1] : 0;
        msg.param3 = params.size() > 2 ? params[2] : 0;
        msg.param4 = params.size() > 3 ? params[3] : 0;
        msg.param5 = params.size() > 4 ? params[4] : 0;
        msg.param6 = params.size() > 5 ? params[5] : 0;
        msg.param7 = params.size() > 6 ? params[6] : 0;
        break;
    }
    
    mavlink_cmd_pub_->publish(msg);
}

void GeofenceMonitorNode::publishStatus() {
    if (!position_received_) return;
    
    auto msg = swarm_msgs::msg::GeofenceStatus();
    msg.timestamp_us = current_status_.timestamp_us;
    msg.drone_id = current_status_.drone_id;
    msg.inside_geofence = current_status_.inside_geofence;
    msg.current_action = static_cast<uint8_t>(current_status_.current_action);
    msg.severity = static_cast<uint8_t>(current_status_.severity);
    msg.min_distance_to_boundary_m = current_status_.min_distance_to_boundary_m;
    msg.nearest_polygon_id = current_status_.nearest_polygon_id;
    msg.distances_to_polygons = current_status_.distances_to_polygons;
    msg.rtl_triggered = current_status_.rtl_triggered;
    msg.rtl_trigger_time_us = current_status_.rtl_trigger_time_us;
    
    geofence_status_pub_->publish(msg);
}

void GeofenceMonitorNode::publishBreach(GeofenceAction action, const Eigen::Vector3d& position) {
    auto msg = swarm_msgs::msg::GeofenceBreach();
    msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    msg.drone_id = drone_id_;
    msg.action = static_cast<uint8_t>(action);
    msg.position_enu = {static_cast<float>(position.x()), 
                        static_cast<float>(position.y()), 
                        static_cast<float>(position.z())};
    msg.nearest_polygon_id = current_status_.nearest_polygon_id;
    msg.distance_to_boundary_m = current_status_.min_distance_to_boundary_m;
    msg.buffer_distance_m = config_.buffer_distance_m;
    
    geofence_breach_pub_->publish(msg);
    
    RCLCPP_WARN(get_logger(), "GEOFENCE BREACH: action=%d, pos=(%.1f, %.1f, %.1f), dist=%.1fm", 
                static_cast<int>(action), position.x(), position.y(), position.z(), 
                current_status_.min_distance_to_boundary_m);
}

}  // namespace swarm::safety

#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarm::safety::GeofenceMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}