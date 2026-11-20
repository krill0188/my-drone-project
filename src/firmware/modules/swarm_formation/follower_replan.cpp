#include "follower_replan.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <algorithm>
#include <cmath>

namespace swarm::formation {

void FollowerReplanHandler::onLeaderReplan(const planner::Trajectory& new_trajectory) {
    _replan_active = true;
    _leader_new_trajectory = new_trajectory;
    _replan_start_us = Timestamp::nowUs();
    
    // Initialize follower errors
    _follower_errors.clear();
    auto slots = _controller.getFormationSlots();
    for (const auto& slot : slots) {
        FollowerError err;
        err.follower_id = slot.follower_id;
        err.position_error_flu = {0, 0, 0};
        err.velocity_error_flu = {0, 0, 0};
        err.max_lateral_error = 0.0f;
        err.max_vertical_error = 0.0f;
        err.last_update_us = Timestamp::nowUs();
        _follower_errors.push_back(err);
    }
}

void FollowerReplanHandler::onLeaderEmergencyManeuver(const Vector3& new_leader_vel, 
                                                       const Vector3& new_leader_accel) {
    // Emergency: immediate velocity command to followers
    _replan_active = true;
    _replan_start_us = Timestamp::nowUs();
    
    // Broadcast emergency velocity adjustment
    // In real implementation, would send high-priority MAVLink message
}

bool FollowerReplanHandler::adjustFormationPosition(float dt) {
    if (!_replan_active) return false;
    
    uint64_t now = Timestamp::nowUs();
    float elapsed = static_cast<float>(now - _replan_start_us) / 1e6f;
    
    if (elapsed > _max_adjustment_time_s) {
        // Timeout - formation adjustment taking too long
        _replan_active = false;
        return false;
    }
    
    // Get current formation state
    auto formation_msg = _controller.getFormationStateMsg();
    
    // Update each follower's position error
    for (auto& err : _follower_errors) {
        // Find this follower in formation state
        auto it = std::find(formation_msg.follower_ids.begin(), 
                           formation_msg.follower_ids.end(), err.follower_id);
        
        if (it != formation_msg.follower_ids.end()) {
            size_t idx = std::distance(formation_msg.follower_ids.begin(), it);
            
            if (idx < formation_msg.position_errors_flu.size()) {
                err.position_error_flu = formation_msg.position_errors_flu[idx];
                err.velocity_error_flu = formation_msg.velocity_errors_flu[idx];
                
                float lateral = std::sqrt(err.position_error_flu.x * err.position_error_flu.x + 
                                         err.position_error_flu.y * err.position_error_flu.y);
                float vertical = std::abs(err.position_error_flu.z);
                
                err.max_lateral_error = std::max(err.max_lateral_error, lateral);
                err.max_vertical_error = std::max(err.max_vertical_error, vertical);
                err.last_update_us = Timestamp::nowUs();
            }
        }
    }
    
    // Check if all followers within tolerance
    bool all_within_tolerance = true;
    for (const auto& err : _follower_errors) {
        float lateral = std::sqrt(err.position_error_flu.x * err.position_error_flu.x + 
                                 err.position_error_flu.y * err.position_error_flu.y);
        float vertical = std::abs(err.position_error_flu.z);
        
        if (lateral > 0.5f || vertical > 0.3f) {  // Tolerances from FR-002
            all_within_tolerance = false;
            break;
        }
    }
    
    if (all_within_tolerance) {
        _replan_active = false;
        return true;  // Formation adjustment complete
    }
    
    return false;  // Still adjusting
}

bool FollowerReplanHandler::isFormationStable() const {
    return !_replan_active;
}

float FollowerReplanHandler::getMaxPositionError() const {
    float max_err = 0.0f;
    for (const auto& err : _follower_errors) {
        float lateral = std::sqrt(err.position_error_flu.x * err.position_error_flu.x + 
                                 err.position_error_flu.y * err.position_error_flu.y);
        float vertical = std::abs(err.position_error_flu.z);
        max_err = std::max(max_err, std::max(lateral, vertical));
    }
    return max_err;
}

float FollowerReplanHandler::getAvgPositionError() const {
    if (_follower_errors.empty()) return 0.0f;
    
    float sum = 0.0f;
    for (const auto& err : _follower_errors) {
        float lateral = std::sqrt(err.position_error_flu.x * err.position_error_flu.x + 
                                 err.position_error_flu.y * err.position_error_flu.y);
        float vertical = std::abs(err.position_error_flu.z);
        sum += std::max(lateral, vertical);
    }
    return sum / _follower_errors.size();
}

Vector3 FollowerReplanHandler::computeFollowerVelocityAdjustment(const FormationSlot& slot,
                                                                  const Vector3& leader_pos,
                                                                  const Vector3& leader_vel,
                                                                  const Vector3& target_pos) {
    // Compute required velocity to reach target offset
    Vector3 current_target_flu = slot.offset_flu;
    Vector3 target_flu = target_pos - leader_pos;  // Simplified
    
    // Convert target to FLU frame (would need leader orientation)
    // Simplified: assume leader heading = 0
    Vector3 target_flu_est = target_flu;
    
    Vector3 offset_error = target_flu_est - current_target_flu;
    
    // PID-like velocity adjustment
    float kp = slot.gain_xy;
    float kd = 0.1f;
    
    Vector3 vel_adj;
    vel_adj.x = kp * offset_error.x;
    vel_adj.y = kp * offset_error.y;
    vel_adj.z = slot.gain_z * offset_error.z;
    
    // Limit
    float max_adj = 2.0f;
    float mag = std::sqrt(vel_adj.x*vel_adj.x + vel_adj.y*vel_adj.y);
    if (mag > max_adj) {
        vel_adj.x *= max_adj / mag;
        vel_adj.y *= max_adj / mag;
    }
    vel_adj.z = std::clamp(vel_adj.z, -1.0f, 1.0f);
    
    return vel_adj;
}

Vector3 FollowerReplanHandler::computeTransitionVelocity(const FormationSlot& slot,
                                                          const Vector3& current_offset_flu,
                                                          const Vector3& target_offset_flu,
                                                          float dt) {
    // Smooth transition from current to target offset
    Vector3 error = target_offset_flu - current_offset_flu;
    
    // Exponential approach
    float alpha = 1.0f - std::exp(-dt / 0.5f);  // Time constant 0.5s
    
    Vector3 transition_vel;
    transition_vel.x = error.x * alpha / dt;
    transition_vel.y = error.y * alpha / dt;
    transition_vel.z = error.z * alpha / dt;
    
    // Limit
    float max_vel = 3.0f;
    float mag = std::sqrt(transition_vel.x*transition_vel.x + transition_vel.y*transition_vel.y);
    if (mag > max_vel) {
        transition_vel.x *= max_vel / mag;
        transition_vel.y *= max_vel / mag;
    }
    transition_vel.z = std::clamp(transition_vel.z, -1.5f, 1.5f);
    
    return transition_vel;
}

Vector3 FollowerReplanHandler::applyCollisionAvoidance(uint16_t follower_id, const Vector3& vel_cmd) {
    // Get nearby drones from mesh
    // In real implementation, would query mesh manager for neighbor positions
    
    // Simplified: no collision avoidance adjustment
    return vel_cmd;
}

} // namespace swarm::formation