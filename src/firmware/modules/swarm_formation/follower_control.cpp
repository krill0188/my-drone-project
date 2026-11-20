#include "swarm_formation.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <cmath>
#include <algorithm>

namespace swarm::formation {

FollowerPositionController::FollowerPositionController() 
    : _gains{}, _pos_integral{}, _prev_pos_error{} {}

void FollowerPositionController::setGains(const ControlGains& gains) {
    _gains = gains;
}

Vector3 FollowerPositionController::computeVelocity(const Vector3& own_position,
                                                     const Vector3& own_velocity,
                                                     const Vector3& target_offset_flu,
                                                     const Quaternion& leader_orientation,
                                                     const Vector3& leader_velocity,
                                                     float dt) {
    // Convert target offset from FLU to ENU
    Vector3 target_offset_enu = rotateToEnu(target_offset_flu, leader_orientation);
    
    // Target position in ENU = leader position + offset
    // Leader position would be passed separately; here we use leader_velocity to estimate
    // In real implementation, leader position would come from formation state message
    
    // Position error in ENU frame
    Vector3 pos_error_enu = target_offset_enu;  // Simplified: assuming leader position known
    
    // PID control
    Vector3 vel_cmd_enu;
    
    // XY control
    _pos_integral.x += pos_error_enu.x * dt;
    _pos_integral.y += pos_error_enu.y * dt;
    
    // Anti-windup
    float max_integral = 2.0f;
    _pos_integral.x = std::clamp(_pos_integral.x, -max_integral, max_integral);
    _pos_integral.y = std::clamp(_pos_integral.y, -max_integral, max_integral);
    
    float deriv_x = (pos_error_enu.x - _prev_pos_error.x) / dt;
    float deriv_y = (pos_error_enu.y - _prev_pos_error.y) / dt;
    
    vel_cmd_enu.x = _gains.kp_xy * pos_error_enu.x + _gains.ki_xy * _pos_integral.x + _gains.kd_xy * deriv_x;
    vel_cmd_enu.y = _gains.kp_xy * pos_error_enu.y + _gains.ki_xy * _pos_integral.y + _gains.kd_xy * deriv_y;
    
    // Z control
    _pos_integral.z += pos_error_enu.z * dt;
    _pos_integral.z = std::clamp(_pos_integral.z, -1.0f, 1.0f);
    float deriv_z = (pos_error_enu.z - _prev_pos_error.z) / dt;
    vel_cmd_enu.z = _gains.kp_z * pos_error_enu.z + _gains.ki_z * _pos_integral.z + _gains.kd_z * deriv_z;
    
    // Feedforward: add leader velocity
    vel_cmd_enu.x += leader_velocity.x;
    vel_cmd_enu.y += leader_velocity.y;
    vel_cmd_enu.z += leader_velocity.z;
    
    // Velocity limiting
    float vel_xy = std::sqrt(vel_cmd_enu.x * vel_cmd_enu.x + vel_cmd_enu.y * vel_cmd_enu.y);
    if (vel_xy > _gains.max_vel_xy) {
        float scale = _gains.max_vel_xy / vel_xy;
        vel_cmd_enu.x *= scale;
        vel_cmd_enu.y *= scale;
    }
    vel_cmd_enu.z = std::clamp(vel_cmd_enu.z, -_gains.max_vel_z, _gains.max_vel_z);
    
    // Store for derivative term
    _prev_pos_error = pos_error_enu;
    
    return vel_cmd_enu;
}

Vector3 FollowerPositionController::computeVelocityEnu(const Vector3& pos_error_enu,
                                                        const Vector3& leader_velocity,
                                                        float dt) {
    // PID control in ENU frame
    Vector3 vel_cmd_enu;
    
    _pos_integral.x += pos_error_enu.x * dt;
    _pos_integral.y += pos_error_enu.y * dt;
    _pos_integral.z += pos_error_enu.z * dt;
    
    // Anti-windup
    float max_integral = 2.0f;
    _pos_integral.x = std::clamp(_pos_integral.x, -max_integral, max_integral);
    _pos_integral.y = std::clamp(_pos_integral.y, -max_integral, max_integral);
    _pos_integral.z = std::clamp(_pos_integral.z, -1.0f, 1.0f);
    
    float deriv_x = (pos_error_enu.x - _prev_pos_error.x) / dt;
    float deriv_y = (pos_error_enu.y - _prev_pos_error.y) / dt;
    float deriv_z = (pos_error_enu.z - _prev_pos_error.z) / dt;
    
    vel_cmd_enu.x = _gains.kp_xy * pos_error_enu.x + _gains.ki_xy * _pos_integral.x + _gains.kd_xy * deriv_x;
    vel_cmd_enu.y = _gains.kp_xy * pos_error_enu.y + _gains.ki_xy * _pos_integral.y + _gains.kd_xy * deriv_y;
    vel_cmd_enu.z = _gains.kp_z * pos_error_enu.z + _gains.ki_z * _pos_integral.z + _gains.kd_z * deriv_z;
    
    // Feedforward
    vel_cmd_enu.x += leader_velocity.x;
    vel_cmd_enu.y += leader_velocity.y;
    vel_cmd_enu.z += leader_velocity.z;
    
    // Velocity limiting
    float vel_xy = std::sqrt(vel_cmd_enu.x * vel_cmd_enu.x + vel_cmd_enu.y * vel_cmd_enu.y);
    if (vel_xy > _gains.max_vel_xy) {
        float scale = _gains.max_vel_xy / vel_xy;
        vel_cmd_enu.x *= scale;
        vel_cmd_enu.y *= scale;
    }
    vel_cmd_enu.z = std::clamp(vel_cmd_enu.z, -_gains.max_vel_z, _gains.max_vel_z);
    
    _prev_pos_error = pos_error_enu;
    
    return vel_cmd_enu;
}

void FollowerPositionController::resetIntegrals() {
    _pos_integral = Vector3{0, 0, 0};
    _prev_pos_error = Vector3{0, 0, 0};
}

Vector3 FollowerPositionController::rotateToEnu(const Vector3& vec_flu, const Quaternion& leader_orientation) {
    // FLU to ENU: rotate by +yaw around Z
    float yaw = std::atan2(2.0f * (leader_orientation.w * leader_orientation.z + leader_orientation.x * leader_orientation.y),
                          1.0f - 2.0f * (leader_orientation.y * leader_orientation.y + leader_orientation.z * leader_orientation.z));
    
    float cos_yaw = std::cos(yaw);
    float sin_yaw = std::sin(yaw);
    
    Vector3 result;
    result.x = cos_yaw * vec_flu.x - sin_yaw * vec_flu.y;
    result.y = sin_yaw * vec_flu.x + cos_yaw * vec_flu.y;
    result.z = vec_flu.z;
    
    return result;
}

} // namespace swarm::formation