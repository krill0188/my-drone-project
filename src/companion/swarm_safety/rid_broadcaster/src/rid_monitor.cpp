#include "swarm_safety_rid_broadcaster/rid_monitor.hpp"
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <curl/curl.h>
#include <nlohmann/json.hpp>

namespace swarm::safety {

using json = nlohmann::json;

RIDMonitorNode::RIDMonitorNode() : Node("rid_monitor") {
    declareParameters();
    loadParameters();
    
    setupSubscriptions();
    setupPublishers();
    setupServices();
    
    // Timers
    broadcast_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000 / config_.broadcast_rate_hz),
        [this]() { broadcastOwnFrame(); });
    
    validation_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),  // 10 Hz validation
        [this]() { checkTrackTimeouts(); });
    
    status_timer_ = create_wall_timer(
        std::chrono::milliseconds(1000),  // 1 Hz status
        [this]() { publishStatus(); });
    
    utm_timer_ = create_wall_timer(
        std::chrono::milliseconds(config_.utm_report_interval_ms),
        [this]() { /* UTM reporting handled per frame */ });
    
    RCLCPP_INFO(get_logger(), "RID Monitor Node initialized");
    RCLCPP_INFO(get_logger(), "Own UAS ID: %s, Type: %d, Broadcast: %d Hz",
                config_.own_uas_id.c_str(), config_.own_uas_type, config_.broadcast_rate_hz);
}

void RIDMonitorNode::declareParameters() {
    declare_parameter("drone_id", 1);
    declare_parameter("broadcast_rate_hz", 1);
    declare_parameter("own_uas_id", "SWARM-001");
    declare_parameter("own_operator_id", "OP-001");
    declare_parameter("own_uas_type", 2);
    declare_parameter("enable_validation", true);
    declare_parameter("enable_utm", false);
    declare_parameter("utm_endpoint", "");
    declare_parameter("utm_auth_token", "");
    declare_parameter("max_position_jump_m", 50.0);
    declare_parameter("max_speed_mps", 100.0);
    declare_parameter("max_acceleration_mps2", 20.0);
    declare_parameter("max_frame_age_ms", 5000);
    declare_parameter("spoof_distance_threshold_m", 1000.0);
}

void RIDMonitorNode::loadParameters() {
    drone_id_ = get_parameter("drone_id").as_int();
    config_.broadcast_rate_hz = get_parameter("broadcast_rate_hz").as_int();
    config_.own_uas_id = get_parameter("own_uas_id").as_string();
    config_.own_operator_id = get_parameter("own_operator_id").as_string();
    config_.own_uas_type = get_parameter("own_uas_type").as_int();
    config_.enable_frame_validation = get_parameter("enable_validation").as_bool();
    config_.enable_utm_interface = get_parameter("enable_utm").as_bool();
    config_.max_position_jump_m = get_parameter("max_position_jump_m").as_double();
    config_.max_speed_mps = get_parameter("max_speed_mps").as_double();
    config_.max_acceleration_mps2 = get_parameter("max_acceleration_mps2").as_double();
    config_.max_frame_age_ms = get_parameter("max_frame_age_ms").as_int();
    config_.spoof_distance_threshold_m = get_parameter("spoof_distance_threshold_m").as_double();
    config_.utm_endpoint = get_parameter("utm_endpoint").as_string();
    config_.utm_auth_token = get_parameter("utm_auth_token").as_string();
    config_.enable_utm_interface = !config_.utm_endpoint.empty();
    
    own_sequence_ = 0;
    
    RCLCPP_INFO(get_logger(), "RID Monitor config loaded: validation=%s, UTM=%s", 
                config_.enable_frame_validation ? "on" : "off",
                config_.enable_utm_interface ? "on" : "off");
}

void RIDMonitorNode::setupSubscriptions() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // MAVLink RID frames from other drones
    mavlink_rid_sub_ = create_subscription<mavros_msgs::msg::RIDFrame>(
        "/swarm/rid_frames", qos,
        [this](const mavros_msgs::msg::RIDFrame::SharedPtr msg) {
            mavlinkRIDCallback(msg);
        });
    
    // Agent state for own position/velocity
    agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
        "/swarm/agent_states", qos,
        [this](const swarm_msgs::msg::AgentState::SharedPtr msg) {
            if (msg->sysid == drone_id_) {
                agentStateCallback(msg);
            }
        });
    
    // Safety events
    safety_event_sub_ = create_subscription<swarm_msgs::msg::SafetyEvent>(
        "/swarm/safety_events", qos,
        [this](const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
            safetyEventCallback(msg);
        });
}

void RIDMonitorNode::setupPublishers() {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    
    rid_frame_pub_ = create_publisher<swarm_msgs::msg::RIDFrame>(
        "/drone" + std::to_string(drone_id_) + "/rid_frame", qos);
    
    rid_status_pub_ = create_publisher<swarm_msgs::msg::RIDStatus>(
        "/drone" + std::to_string(drone_id_) + "/rid_status", qos);
    
    safety_event_pub_ = create_publisher<swarm_msgs::msg::SafetyEvent>(
        "/swarm/safety_events", qos);
    
    mavlink_rid_pub_ = create_publisher<mavros_msgs::msg::RIDFrame>(
        "/drone" + std::to_string(drone_id_) + "/mavlink_rid_out", qos);
}

void RIDMonitorNode::setupServices() {
    config_srv_ = create_service<swarm_msgs::srv::UpdateRIDConfig>(
        "/drone" + std::to_string(drone_id_) + "/update_rid_config",
        [this](const std::shared_ptr<rmw_request_id_t>,
               const std::shared_ptr<swarm_msgs::srv::UpdateRIDConfig::Request> request,
               std::shared_ptr<swarm_msgs::srv::UpdateRIDConfig::Response> response) {
            configCallback(nullptr, request, response);
        });
}

void RIDMonitorNode::mavlinkRIDCallback(const mavros_msgs::msg::RIDFrame::SharedPtr msg) {
    if (!config_.enable_frame_validation) return;
    
    RIDFrame frame;
    try {
        frame = convertMAVLinkFrame(msg);
        validateFrame(msg, frame);
    } catch (const std::exception& e) {
        frame.valid = false;
        frame.validation_error = e.what();
    }
    
    // Update stats
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.frames_received++;
        if (frame.valid) stats_.frames_valid++;
        else stats_.frames_invalid++;
    }
    
    if (frame.valid) {
        updateTrack(frame);
        
        // Publish validated frame
        auto out_msg = swarm_msgs::msg::RIDFrame();
        out_msg.timestamp_us = frame.timestamp_us;
        out_msg.source_id = frame.source_id;
        out_msg.uas_id = frame.uas_id;
        out_msg.id_type = frame.id_type;
        out_msg.operator_id = frame.operator_id;
        out_msg.uas_type = frame.uas_type;
        out_msg.latitude_deg = frame.latitude_deg;
        out_msg.longitude_deg = frame.longitude_deg;
        out_msg.altitude_m = frame.altitude_m;
        out_msg.height_m = frame.height_m;
        out_msg.velocity_north_mps = frame.velocity_north_mps;
        out_msg.velocity_east_mps = frame.velocity_east_mps;
        out_msg.velocity_down_mps = frame.velocity_down_mps;
        out_msg.horizontal_accuracy_m = frame.horizontal_accuracy_m;
        out_msg.vertical_accuracy_m = frame.vertical_accuracy_m;
        out_msg.speed_accuracy_mps = frame.speed_accuracy_mps;
        out_msg.status = frame.status;
        out_msg.direction = static_cast<uint16_t>(frame.direction * 100);
        out_msg.ground_speed_mps = frame.ground_speed_mps;
        out_msg.sequence = frame.sequence;
        out_msg.auth_type = frame.auth_type;
        out_msg.auth_data = frame.auth_data;
        
        rid_frame_pub_->publish(out_msg);
        
        // UTM reporting
        if (config_.enable_utm_interface) {
            reportToUTM(frame);
        }
    } else {
        // Invalid frame - publish safety event
        publishSafetyEvent(frame, "RID_FRAME_INVALID: " + frame.validation_error);
    }
}

RIDFrame RIDMonitorNode::convertMAVLinkFrame(const mavros_msgs::msg::RIDFrame::SharedPtr& msg) {
    RIDFrame frame;
    frame.timestamp_us = msg->timestamp_us;
    frame.source_id = msg->source_id;
    frame.uas_id = msg->uas_id;
    frame.id_type = msg->id_type;
    frame.operator_id = msg->operator_id;
    frame.uas_type = msg->uas_type;
    frame.latitude_deg = msg->latitude_deg;
    frame.longitude_deg = msg->longitude_deg;
    frame.altitude_m = msg->altitude_m;
    frame.height_m = msg->height_m;
    frame.velocity_north_mps = msg->velocity_north_mps;
    frame.velocity_east_mps = msg->velocity_east_mps;
    frame.velocity_down_mps = msg->velocity_down_mps;
    frame.horizontal_accuracy_m = msg->horizontal_accuracy_m;
    frame.vertical_accuracy_m = msg->vertical_accuracy_m;
    frame.speed_accuracy_mps = msg->speed_accuracy_mps;
    frame.status = msg->status;
    frame.direction = msg->direction / 100.0f;
    frame.ground_speed_mps = msg->ground_speed_mps;
    frame.sequence = msg->sequence;
    frame.auth_type = msg->auth_type;
    frame.auth_data = msg->auth_data;
    return frame;
}

void RIDMonitorNode::validateFrame(const mavros_msgs::msg::RIDFrame::SharedPtr& msg, RIDFrame& frame) {
    std::vector<std::string> errors;
    
    // Basic ID validation
    if (!validateBasicID(frame)) {
        errors.push_back("Invalid Basic ID");
    }
    
    // Location validation
    if (!validateLocation(frame)) {
        errors.push_back("Invalid Location");
    }
    
    // Kinematics validation
    if (!validateKinematics(frame)) {
        errors.push_back("Invalid Kinematics");
    }
    
    // Authentication validation
    if (!validateAuthentication(frame)) {
        errors.push_back("Authentication Failed");
    }
    
    // Spoofing check
    if (config_.enable_spoofing_detection && !checkSpoofing(frame)) {
        errors.push_back("Possible Spoofing");
    }
    
    if (!errors.empty()) {
        frame.valid = false;
        frame.validation_error = "";
        for (size_t i = 0; i < errors.size(); ++i) {
            if (i > 0) frame.validation_error += "; ";
            frame.validation_error += errors[i];
        }
    } else {
        frame.valid = true;
        frame.validation_error.clear();
    }
}

bool RIDMonitorNode::validateBasicID(const RIDFrame& frame) {
    // UAS ID required, max 20 chars
    if (frame.uas_id.empty() || frame.uas_id.size() > 20) return false;
    
    // ID type valid (1=Serial, 2=CAA, 3=Session, 4=Operator)
    if (frame.id_type < 1 || frame.id_type > 4) return false;
    
    // Operator ID max 20 chars
    if (frame.operator_id.size() > 20) return false;
    
    // UAS type valid (1=Airplane, 2=Rotorcraft, 3=Gyroplane, 4=Hybrid, 5=Other)
    if (frame.uas_type < 1 || frame.uas_type > 5) return false;
    
    return true;
}

bool RIDMonitorNode::validateLocation(const RIDFrame& frame) {
    // Latitude/Longitude bounds
    if (frame.latitude_deg < -90.0 || frame.latitude_deg > 90.0) return false;
    if (frame.longitude_deg < -180.0 || frame.longitude_deg > 180.0) return false;
    
    // Altitude bounds (reasonable)
    if (frame.altitude_m < -1000.0 || frame.altitude_m > 20000.0) return false;
    if (frame.height_m < 0.0 || frame.height_m > 5000.0) return false;
    
    // Accuracy bounds
    if (frame.horizontal_accuracy_m < 0.0 || frame.horizontal_accuracy_m > config_.min_horizontal_accuracy_m) return false;
    if (frame.vertical_accuracy_m < 0.0 || frame.vertical_accuracy_m > config_.min_vertical_accuracy_m) return false;
    if (frame.speed_accuracy_mps < 0.0) return false;
    
    // Status valid
    if (frame.status > 2) return false;
    
    return true;
}

bool RIDMonitorNode::validateKinematics(const RIDFrame& frame) {
    // Speed bounds
    if (frame.ground_speed_mps < 0.0 || frame.ground_speed_mps > config_.max_speed_mps) return false;
    if (std::abs(frame.velocity_north_mps) > config_.max_speed_mps) return false;
    if (std::abs(frame.velocity_east_mps) > config_.max_speed_mps) return false;
    if (std::abs(frame.velocity_down_mps) > config_.max_speed_mps) return false;
    
    // Direction valid (0-360 degrees)
    if (frame.direction < 0.0 || frame.direction >= 360.0) return false;
    
    // Check against track history for acceleration
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    auto it = tracks_.find(frame.source_id);
    if (it != tracks_.end() && !it->second.history.empty()) {
        const auto& last_frame = it->second.history.back();
        double dt = (frame.timestamp_us - last_frame.timestamp_us) / 1e6;
        if (dt > 0.0 && dt < 10.0) {
            Eigen::Vector3d last_vel(last_frame.velocity_north_mps, 
                                     last_frame.velocity_east_mps, 
                                     last_frame.velocity_down_mps);
            Eigen::Vector3d curr_vel(frame.velocity_north_mps, 
                                     frame.velocity_east_mps, 
                                     frame.velocity_down_mps);
            Eigen::Vector3d accel = (curr_vel - last_vel) / dt;
            double accel_mag = accel.norm();
            
            if (accel_mag > config_.max_acceleration_mps2) {
                return false;
            }
        }
    }
    
    return true;
}

bool RIDMonitorNode::validateAuthentication(const RIDFrame& frame) {
    // For now, just check auth type is valid
    if (frame.auth_type > 2) return false;
    
    // Auth data size check
    if (frame.auth_data.size() > 64) return false;
    
    return true;
}

bool RIDMonitorNode::checkSpoofing(const RIDFrame& frame) {
    if (!position_received_) return true;  // Can't check without own position
    
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    auto it = tracks_.find(frame.source_id);
    
    if (it != tracks_.end() && !it->second.history.empty()) {
        const auto& last_frame = it->second.history.back();
        
        // Check position jump
        double lat_diff = frame.latitude_deg - last_frame.latitude_deg;
        double lon_diff = frame.longitude_deg - last_frame.longitude_deg;
        
        // Approximate distance in meters
        double lat_m = lat_diff * 111320.0;
        double lon_m = lon_diff * 111320.0 * cos(last_frame.latitude_deg * M_PI / 180.0);
        double dist = sqrt(lat_m * lat_m + lon_m * lon_m);
        
        if (dist > config_.max_position_jump_m) {
            return false;  // Spoofing detected
        }
        
        // Check against own position
        double own_lat_diff = frame.latitude_deg - (current_position_enu_.y() / 111320.0);
        double own_lon_diff = frame.longitude_deg - (current_position_enu_.x() / (111320.0 * cos(frame.latitude_deg * M_PI / 180.0)));
        double own_dist = sqrt(own_lat_diff * own_lat_diff + own_lon_diff * own_lon_diff) * 111320.0;
        
        if (own_dist < config_.spoof_distance_threshold_m) {
            // Close to us - could be spoofing our own ID
            if (frame.uas_id == config_.own_uas_id && frame.source_id != drone_id_) {
                return false;  // Someone else using our ID
            }
        }
    }
    
    return true;
}

void RIDMonitorNode::updateTrack(const RIDFrame& frame) {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    
    auto& track = tracks_[frame.source_id];
    track.source_id = frame.source_id;
    track.last_frame = frame;
    track.last_update_us = frame.timestamp_us;
    track.missing_frames = 0;
    
    // Update velocity/acceleration
    if (!track.history.empty()) {
        const auto& last_frame = track.history.back();
        double dt = (frame.timestamp_us - last_frame.timestamp_us) / 1e6;
        if (dt > 0.0 && dt < 10.0) {
            Eigen::Vector3d last_vel(last_frame.velocity_north_mps, 
                                     last_frame.velocity_east_mps, 
                                     last_frame.velocity_down_mps);
            Eigen::Vector3d curr_vel(frame.velocity_north_mps, 
                                     frame.velocity_east_mps, 
                                     frame.velocity_down_mps);
            track.last_velocity_enu = curr_vel;
            track.last_acceleration_enu = (curr_vel - last_vel) / dt;
        }
    }
    
    // Add to history (keep last 10)
    track.history.push_back(frame);
    if (track.history.size() > 10) {
        track.history.erase(track.history.begin());
    }
    
    // Update spoofing flag
    track.is_spoofed = !checkSpoofing(frame);
    if (track.is_spoofed) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.spoofing_detected++;
        RCLCPP_WARN(get_logger(), "Spoofing detected for source %d: %s", 
                    frame.source_id, frame.validation_error.c_str());
    }
}

void RIDMonitorNode::checkTrackTimeouts() {
    uint64_t now_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    
    for (auto it = tracks_.begin(); it != tracks_.end();) {
        if (now_us - it->second.last_update_us > static_cast<uint64_t>(config_.max_frame_age_ms) * 1000) {
            it->second.missing_frames++;
            if (it->second.missing_frames > config_.max_missing_frames) {
                RCLCPP_INFO(get_logger(), "Track %d timed out, removing", it->first);
                it = tracks_.erase(it);
            } else {
                ++it;
            }
        } else {
            ++it;
        }
    }
}

void RIDMonitorNode::agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
    current_position_enu_ = Eigen::Vector3d(msg->position.x, msg->position.y, msg->position.z);
    current_velocity_enu_ = Eigen::Vector3d(msg->velocity.x, msg->velocity.y, msg->velocity.z);
    position_received_ = true;
}

void RIDMonitorNode::safetyEventCallback(const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
    // Handle safety events if needed
}

void RIDMonitorNode::configCallback(const std::shared_ptr<rmw_request_id_t>,
                                    const std::shared_ptr<swarm_msgs::srv::UpdateRIDConfig::Request> request,
                                    std::shared_ptr<swarm_msgs::srv::UpdateRIDConfig::Response> response) {
    std::lock_guard<std::mutex> lock(config_mutex_);
    
    config_.max_position_jump_m = request->max_position_jump_m;
    config_.max_speed_mps = request->max_speed_mps;
    config_.max_acceleration_mps2 = request->max_acceleration_mps2;
    config_.max_frame_age_ms = request->max_frame_age_ms;
    config_.spoof_distance_threshold_m = request->spoof_distance_threshold_m;
    config_.enable_frame_validation = request->enable_validation;
    config_.enable_utm_interface = request->enable_utm;
    config_.utm_endpoint = request->utm_endpoint;
    config_.utm_auth_token = request->utm_auth_token;
    config_.utm_report_interval_ms = request->utm_report_interval_ms;
    
    response->success = true;
    response->message = "RID config updated";
    RCLCPP_INFO(get_logger(), "RID config updated via service");
}

void RIDMonitorNode::broadcastOwnFrame() {
    if (!position_received_) return;
    
    RIDFrame frame;
    frame.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    frame.source_id = drone_id_;
    frame.uas_id = config_.own_uas_id;
    frame.id_type = 3;  // Session ID
    frame.operator_id = config_.own_operator_id;
    frame.uas_type = config_.own_uas_type;
    
    // Convert ENU to LLA (simplified)
    frame.latitude_deg = current_position_enu_.y() / 111320.0;
    frame.longitude_deg = current_position_enu_.x() / (111320.0 * cos(frame.latitude_deg * M_PI / 180.0));
    frame.altitude_m = current_position_enu_.z();
    frame.height_m = current_position_enu_.z();  // Simplified
    
    frame.velocity_north_mps = current_velocity_enu_.y();
    frame.velocity_east_mps = current_velocity_enu_.x();
    frame.velocity_down_mps = -current_velocity_enu_.z();
    
    frame.horizontal_accuracy_m = 5.0;
    frame.vertical_accuracy_m = 3.0;
    frame.speed_accuracy_mps = 1.0;
    frame.status = 0;  // Normal
    frame.direction = atan2(frame.velocity_east_mps, frame.velocity_north_mps) * 180.0 / M_PI;
    if (frame.direction < 0) frame.direction += 360.0;
    frame.ground_speed_mps = sqrt(frame.velocity_north_mps * frame.velocity_north_mps + 
                                  frame.velocity_east_mps * frame.velocity_east_mps);
    
    frame.sequence = own_sequence_++;
    frame.auth_type = 0;  // No auth
    
    // Convert to MAVLink and publish
    auto mavlink_frame = convertToMAVLink(frame);
    mavlink_rid_pub_->publish(mavlink_frame);
    
    // Also publish to ROS topic
    auto ros_frame = swarm_msgs::msg::RIDFrame();
    ros_frame.timestamp_us = frame.timestamp_us;
    ros_frame.source_id = frame.source_id;
    ros_frame.uas_id = frame.uas_id;
    ros_frame.id_type = frame.id_type;
    ros_frame.operator_id = frame.operator_id;
    ros_frame.uas_type = frame.uas_type;
    ros_frame.latitude_deg = frame.latitude_deg;
    ros_frame.longitude_deg = frame.longitude_deg;
    ros_frame.altitude_m = frame.altitude_m;
    ros_frame.height_m = frame.height_m;
    ros_frame.velocity_north_mps = frame.velocity_north_mps;
    ros_frame.velocity_east_mps = frame.velocity_east_mps;
    ros_frame.velocity_down_mps = frame.velocity_down_mps;
    ros_frame.horizontal_accuracy_m = frame.horizontal_accuracy_m;
    ros_frame.vertical_accuracy_m = frame.vertical_accuracy_m;
    ros_frame.speed_accuracy_mps = frame.speed_accuracy_mps;
    ros_frame.status = frame.status;
    ros_frame.direction = static_cast<uint16_t>(frame.direction * 100);
    ros_frame.ground_speed_mps = frame.ground_speed_mps;
    ros_frame.sequence = frame.sequence;
    ros_frame.auth_type = frame.auth_type;
    ros_frame.auth_data = frame.auth_data;
    
    rid_frame_pub_->publish(ros_frame);
    
    {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.own_frames_broadcast++;
    }
}

mavros_msgs::msg::RIDFrame RIDMonitorNode::convertToMAVLink(const RIDFrame& frame) {
    mavros_msgs::msg::RIDFrame msg;
    msg.timestamp_us = frame.timestamp_us;
    msg.source_id = frame.source_id;
    msg.uas_id = frame.uas_id;
    msg.id_type = frame.id_type;
    msg.operator_id = frame.operator_id;
    msg.uas_type = frame.uas_type;
    msg.latitude_deg = frame.latitude_deg;
    msg.longitude_deg = frame.longitude_deg;
    msg.altitude_m = frame.altitude_m;
    msg.height_m = frame.height_m;
    msg.velocity_north_mps = frame.velocity_north_mps;
    msg.velocity_east_mps = frame.velocity_east_mps;
    msg.velocity_down_mps = frame.velocity_down_mps;
    msg.horizontal_accuracy_m = frame.horizontal_accuracy_m;
    msg.vertical_accuracy_m = frame.vertical_accuracy_m;
    msg.speed_accuracy_mps = frame.speed_accuracy_mps;
    msg.status = frame.status;
    msg.direction = static_cast<uint16_t>(frame.direction * 100);
    msg.ground_speed_mps = frame.ground_speed_mps;
    msg.sequence = frame.sequence;
    msg.auth_type = frame.auth_type;
    msg.auth_data = frame.auth_data;
    return msg;
}

void RIDMonitorNode::reportToUTM(const RIDFrame& frame) {
    if (!config_.enable_utm_interface || config_.utm_endpoint.empty()) return;
    
    if (!sendUTMReport(frame)) {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.utm_reports_failed++;
    } else {
        std::lock_guard<std::mutex> lock(stats_mutex_);
        stats_.utm_reports_sent++;
    }
}

bool RIDMonitorNode::sendUTMReport(const RIDFrame& frame) {
    // Simplified UTM reporting via HTTP POST
    // In production, use proper UTM client library
    
    json report;
    report["timestamp"] = frame.timestamp_us;
    report["uas_id"] = frame.uas_id;
    report["latitude"] = frame.latitude_deg;
    report["longitude"] = frame.longitude_deg;
    report["altitude"] = frame.altitude_m;
    report["height"] = frame.height_m;
    report["velocity_north"] = frame.velocity_north_mps;
    report["velocity_east"] = frame.velocity_east_mps;
    report["velocity_down"] = frame.velocity_down_mps;
    report["status"] = frame.status;
    
    // TODO: Implement actual HTTP POST to UTM endpoint
    // For now, just log
    RCLCPP_DEBUG(get_logger(), "UTM report for %s: lat=%.6f, lon=%.6f", 
                 frame.uas_id.c_str(), frame.latitude_deg, frame.longitude_deg);
    
    return true;
}

void RIDMonitorNode::publishStatus() {
    auto msg = swarm_msgs::msg::RIDStatus();
    msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    msg.drone_id = drone_id_;
    msg.status = 0;  // Broadcasting
    msg.error_code = 0;
    msg.frames_sent = stats_.own_frames_broadcast;
    msg.frames_failed = stats_.frames_invalid;
    msg.tracks_active = tracks_.size();
    msg.spoofing_detected = stats_.spoofing_detected;
    
    rid_status_pub_->publish(msg);
}

void RIDMonitorNode::publishSafetyEvent(const RIDFrame& frame, const std::string& event_type) {
    auto msg = swarm_msgs::msg::SafetyEvent();
    msg.timestamp_us = frame.timestamp_us;
    msg.source_id = frame.source_id;
    msg.type = 12;  // RID_EVENT
    msg.severity = 1;  // Warning
    msg.position_enu = {0, 0, 0};  // Would convert from lat/lon
    msg.details = "RID: " + event_type + " - " + frame.validation_error;
    msg.sequence = 0;
    
    safety_event_pub_->publish(msg);
}

void RIDMonitorNode::agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
    // Already handled above
}

void RIDMonitorNode::safetyEventCallback(const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
    // Handle safety events
}

void RIDMonitorNode::publishSafetyEvent(const RIDFrame& frame, const std::string& event_type) {
    auto msg = swarm_msgs::msg::SafetyEvent();
    msg.timestamp_us = frame.timestamp_us;
    msg.source_id = frame.source_id;
    msg.type = 12;  // RID_EVENT
    msg.severity = 1;  // Warning
    msg.position_enu = {0, 0, 0};
    msg.details = "RID: " + event_type + " - " + frame.validation_error;
    msg.sequence = 0;
    
    safety_event_pub_->publish(msg);
}

}  // namespace swarm::safety

#include "rclcpp/rclcpp.hpp"
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<swarm::safety::RIDMonitorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}