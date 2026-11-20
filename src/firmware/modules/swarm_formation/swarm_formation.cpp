#include "swarm_formation.hpp"
#include "swarm_comm/mavlink_router.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include "swarm_comm/common/crypto.hpp"
#include <cmath>
#include <algorithm>

namespace swarm::formation {

using namespace swarm::comm;

FormationController::FormationController(MeshManager& mesh) : _mesh(mesh) {}

bool FormationController::initAsLeader(uint32_t formation_id, const FormationGeometry& geometry,
                                       const FormationTolerances& tolerances) {
    _formation_id = formation_id;
    _geometry = geometry;
    _tolerances = tolerances;
    _is_leader = true;
    _state = FormationState::FORMING;
    _sequence = 0;
    
    // Compute follower offsets based on geometry
    computeFollowerOffsets();
    
    // Register MAVLink callbacks
    _mesh.registerCallback(SwarmMsgId::FORMATION_STATE,
        [this](uint16_t src_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len) {
            handleFormationStateMessage(src_id, data, len);
        });
    
    return true;
}

bool FormationController::addFollower(uint16_t follower_id, const FormationSlot& slot) {
    if (!_is_leader) return false;
    
    // Check if already exists
    auto it = std::find_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; });
    
    if (it != _slots.end()) {
        *it = slot;  // Update existing
    } else {
        _slots.push_back(slot);
    }
    
    _follower_ids.push_back(follower_id);
    return true;
}

bool FormationController::removeFollower(uint16_t follower_id) {
    if (!_is_leader) return false;
    
    _slots.erase(std::remove_if(_slots.begin(), _slots.end(),
        [follower_id](const FormationSlot& s) { return s.follower_id == follower_id; }),
        _slots.end());
    
    _follower_ids.erase(std::remove(_follower_ids.begin(), _follower_ids.end(), follower_id),
                        _follower_ids.end());
    
    return true;
}

bool FormationController::updateGeometry(const FormationGeometry& geometry) {
    if (!_is_leader) return false;
    
    _geometry = geometry;
    computeFollowerOffsets();
    _state = FormationState::RECONFIGURING;
    return true;
}

Vector3 FormationController::computeLeaderVelocity(const Vector3& current_pos, 
                                                    const Vector3& target_pos, float dt) {
    Vector3 to_target = target_pos - current_pos;
    float distance = std::sqrt(to_target.x * to_target.x + to_target.y * to_target.y + to_target.z * to_target.z);
    
    if (distance < 0.1f) {
        return Vector3{0, 0, 0};
    }
    
    // Simple proportional control for leader velocity
    float max_speed = 5.0f;  // m/s
    float speed = std::min(distance / dt, max_speed);
    
    Vector3 velocity;
    velocity.x = (to_target.x / distance) * speed;
    velocity.y = (to_target.y / distance) * speed;
    velocity.z = (to_target.z / distance) * speed;
    
    return velocity;
}

bool FormationController::broadcastFormationSetpoints() {
    if (!_is_leader) return false;
    
    // Pack formation set message
    SwarmFormationSetData data;
    data.timestamp_us = Timestamp::nowUs();
    data.leader_id = _mesh.getOwnId();  // Would need to be passed in
    data.formation_id = _formation_id;
    data.geometry_type = static_cast<uint8_t>(_geometry.type);
    data.spacing_m = _geometry.spacing_m;
    data.altitude_offset_m = _geometry.altitude_offset_m;
    data.heading_offset_deg = _geometry.heading_offset_deg;
    data.max_lateral_error_m = _tolerances.max_lateral_error_m;
    data.max_vertical_error_m = _tolerances.max_vertical_error_m;
    data.max_velocity_error_mps = _tolerances.max_velocity_error_mps;
    data.max_yaw_error_deg = _tolerances.max_yaw_error_deg;
    data.rejoin_timeout_s = _tolerances.rejoin_timeout_s;
    data.replan_propagation_ms = _tolerances.replan_propagation_ms;
    data.num_slots = static_cast<uint8_t>(_slots.size());
    
    for (size_t i = 0; i < _slots.size() && i < 50; ++i) {
        data.follower_ids[i] = _slots[i].follower_id;
        data.offsets_flu[i][0] = _slots[i].offset_flu.x;
        data.offsets_flu[i][1] = _slots[i].offset_flu.y;
        data.offsets_flu[i][2] = _slots[i].offset_flu.z;
        data.gains_xy[i] = _slots[i].gain_xy;
        data.gains_z[i] = _slots[i].gain_z;
        data.gains_yaw[i] = _slots[i].gain_yaw;
    }
    data.sequence = _sequence++;
    
    // Pack and send via MAVLink router
    mavlink_message_t msg;
    if (SwarmMessagePacker::packFormationSet(_mesh.getOwnId(), 1, data, msg)) {
        return _mesh.broadcastMessage(msg);
    }
    return false;
}

bool FormationController::initAsFollower(uint32_t formation_id, uint16_t leader_id) {
    _formation_id = formation_id;
    _leader_id = leader_id;
    _is_leader = false;
    _state = FormationState::FORMING;
    _follower_status = FollowerStatus::FORMING;
    _last_leader_msg_us = Timestamp::nowUs();
    
    // Register MAVLink callbacks
    _mesh.registerCallback(SwarmMsgId::FORMATION_SET,
        [this](uint16_t src_id, SwarmMsgId msg_id, const uint8_t* data, uint16_t len) {
            handleFormationSetMessage(src_id, data, len);
        });
    
    return true;
}

bool FormationController::updateFollowerState(const AgentState& leader_state, 
                                               const FormationStateMsg& formation_msg) {
    if (_is_leader) return false;
    
    _last_leader_msg_us = Timestamp::nowUs();
    
    // Update target offset from formation message
    if (formation_msg.formation_id == _formation_id) {
        // Find our slot
        for (size_t i = 0; i < formation_msg.follower_ids.size(); ++i) {
            if (formation_msg.follower_ids[i] == _mesh.getOwnId()) {
                _my_slot_index = i;
                _target_offset_flu.x = formation_msg.relative_positions[i].x;
                _target_offset_flu.y = formation_msg.relative_positions[i].y;
                _target_offset_flu.z = formation_msg.relative_positions[i].z;
                break;
            }
        }
    }
    
    // Compute position error in leader FLU frame
    Vector3 leader_to_follower_enu = _leader_state.position - leader_state.position;
    _position_error_flu = rotateToFlu(leader_to_follower_enu, leader_state.orientation);
    _position_error_flu = _position_error_flu - _target_offset_flu;
    
    // Compute velocity error
    _velocity_error_flu = _leader_state.velocity - leader_state.velocity;
    _yaw_error_deg = 0.0f;  // Would compute from orientation difference
    
    // Update follower status based on error
    float lateral_error = std::sqrt(_position_error_flu.x * _position_error_flu.x + 
                                     _position_error_flu.y * _position_error_flu.y);
    float vertical_error = std::abs(_position_error_flu.z);
    
    if (lateral_error > _tolerances.max_lateral_error_m || 
        vertical_error > _tolerances.max_vertical_error_m) {
        _follower_status = FollowerStatus::REJOINING;
    } else {
        _follower_status = FollowerStatus::HOLDING;
    }
    
    // Check for localization loss
    uint64_t now = Timestamp::nowUs();
    if (now - _last_leader_msg_us > static_cast<uint64_t>(_tolerances.rejoin_timeout_s * 1e6)) {
        _follower_status = FollowerStatus::LOST;
        _localization_lost = true;
    } else if (_localization_lost) {
        _localization_lost = false;
        _follower_status = FollowerStatus::REJOINING;
    }
    
    return true;
}

Vector3 FormationController::computeFollowerVelocity(const AgentState& own_state, 
                                                      const Vector3& target_offset_flu) {
    // Convert target offset to ENU frame
    Vector3 target_offset_enu = rotateToEnu(target_offset_flu, _leader_state.orientation);
    Vector3 target_pos_enu = _leader_state.position + target_offset_enu;
    
    // Position error in ENU
    Vector3 pos_error_enu = target_pos_enu - own_state.position;
    
    // PID control in ENU frame
    Vector3 vel_cmd_enu;
    
    // XY control
    float pos_error_xy = std::sqrt(pos_error_enu.x * pos_error_enu.x + pos_error_enu.y * pos_error_enu.y);
    if (pos_error_xy > 0.01f) {
        _pos_integral_xy.x += pos_error_enu.x * 0.01f;  // dt = 0.01s (100 Hz)
        _pos_integral_xy.y += pos_error_enu.y * 0.01f;
        
        // Anti-windup
        float max_integral = 2.0f;
        _pos_integral_xy.x = std::clamp(_pos_integral_xy.x, -max_integral, max_integral);
        _pos_integral_xy.y = std::clamp(_pos_integral_xy.y, -max_integral, max_integral);
        
        float deriv_x = (pos_error_enu.x - _prev_pos_error_flu.x) / 0.01f;
        float deriv_y = (pos_error_enu.y - _prev_pos_error_flu.y) / 0.01f;
        
        vel_cmd_enu.x = _gains.kp_xy * pos_error_enu.x + _gains.ki_xy * _pos_integral_xy.x + _gains.kd_xy * deriv_x;
        vel_cmd_enu.y = _gains.kp_xy * pos_error_enu.y + _gains.ki_xy * _pos_integral_xy.y + _gains.kd_xy * deriv_y;
    }
    
    // Z control
    float pos_error_z = pos_error_enu.z;
    _pos_integral_z.z += pos_error_z * 0.01f;
    _pos_integral_z.z = std::clamp(_pos_integral_z.z, -1.0f, 1.0f);
    float deriv_z = (pos_error_z - _prev_pos_error_flu.z) / 0.01f;
    vel_cmd_enu.z = _gains.kp_z * pos_error_z + _gains.ki_z * _pos_integral_z.z + _gains.kd_z * deriv_z;
    
    // Velocity limiting
    float vel_xy = std::sqrt(vel_cmd_enu.x * vel_cmd_enu.x + vel_cmd_enu.y * vel_cmd_enu.y);
    if (vel_xy > _gains.max_vel_xy) {
        float scale = _gains.max_vel_xy / vel_xy;
        vel_cmd_enu.x *= scale;
        vel_cmd_enu.y *= scale;
    }
    vel_cmd_enu.z = std::clamp(vel_cmd_enu.z, -_gains.max_vel_z, _gains.max_vel_z);
    
    // Store for derivative term
    _prev_pos_error_flu = pos_error_enu;
    
    return vel_cmd_enu;
}

FormationStateMsg FormationController::getFormationStateMsg() const {
    FormationStateMsg msg;
    msg.timestamp_us = Timestamp::nowUs();
    msg.formation_id = _formation_id;
    msg.leader_id = _leader_id;
    msg.follower_ids = _follower_ids;
    msg.geometry = _geometry;
    msg.tolerances = _tolerances;
    msg.state = _state;
    msg.sequence = _sequence;
    return msg;
}

void FormationController::onLeaderReplan(const Vector3& new_target, const Vector3& new_velocity) {
    if (!_is_leader) return;
    
    _leader_target_pos = new_target;
    _leader_current_vel = new_velocity;
    
    // Recompute follower targets
    computeFollowerOffsets();
    
    // Broadcast updated setpoints immediately
    broadcastFormationSetpoints();
}

void FormationController::onLocalizationLost() {
    _localization_lost = true;
    _follower_status = FollowerStatus::LOST;
}

void FormationController::onLocalizationRestored() {
    _localization_lost = false;
    _follower_status = FollowerStatus::REJOINING;
}

void FormationController::computeFollowerOffsets() {
    if (!_is_leader) return;
    
    _slots.clear();
    _follower_ids.clear();
    _follower_targets_flu.clear();
    
    float spacing = _geometry.spacing_m;
    size_t num_followers = _slots.size();  // Would be set before calling
    
    switch (_geometry.type) {
        case GeometryType::LINE: {
            // Line formation behind leader
            for (size_t i = 0; i < num_followers; ++i) {
                FormationSlot slot;
                slot.follower_id = static_cast<uint16_t>(i + 1);  // Temporary
                slot.offset_flu = Vector3{-static_cast<float>(i + 1) * spacing, 0, _geometry.altitude_offset_m};
                slot.gain_xy = 1.0f;
                slot.gain_z = 1.0f;
                slot.gain_yaw = 1.0f;
                _slots.push_back(slot);
                _follower_targets_flu.push_back(slot.offset_flu);
            }
            break;
        }
        case GeometryType::VEE: {
            // V formation
            for (size_t i = 0; i < num_followers; ++i) {
                FormationSlot slot;
                slot.follower_id = static_cast<uint16_t>(i + 1);
                float side = (i % 2 == 0) ? -1.0f : 1.0f;
                float row = static_cast<float>(i / 2 + 1);
                slot.offset_flu = Vector3{-row * spacing, side * row * spacing * 0.7f, _geometry.altitude_offset_m};
                slot.gain_xy = 1.0f;
                slot.gain_z = 1.0f;
                slot.gain_yaw = 1.0f;
                _slots.push_back(slot);
                _follower_targets_flu.push_back(slot.offset_flu);
            }
            break;
        }
        case GeometryType::DIAMOND: {
            // Diamond formation
            for (size_t i = 0; i < num_followers && i < 4; ++i) {
                FormationSlot slot;
                slot.follower_id = static_cast<uint16_t>(i + 1);
                static const float dx[4] = {-1, -1, 1, 1};
                static const float dy[4] = {-1, 1, -1, 1};
                slot.offset_flu = Vector3{dx[i] * spacing, dy[i] * spacing, _geometry.altitude_offset_m};
                slot.gain_xy = 1.0f;
                slot.gain_z = 1.0f;
                slot.gain_yaw = 1.0f;
                _slots.push_back(slot);
                _follower_targets_flu.push_back(slot.offset_flu);
            }
            break;
        }
        case GeometryType::COLUMN: {
            // Column formation (vertical stack)
            for (size_t i = 0; i < num_followers; ++i) {
                FormationSlot slot;
                slot.follower_id = static_cast<uint16_t>(i + 1);
                slot.offset_flu = Vector3{-static_cast<float>(i + 1) * spacing, 0, 
                                         _geometry.altitude_offset_m - static_cast<float>(i) * spacing * 0.5f};
                slot.gain_xy = 1.0f;
                slot.gain_z = 1.0f;
                slot.gain_yaw = 1.0f;
                _slots.push_back(slot);
                _follower_targets_flu.push_back(slot.offset_flu);
            }
            break;
        }
        case GeometryType::CUSTOM: {
            // Custom offsets from geometry.relative_positions
            for (size_t i = 0; i < _geometry.relative_positions.size() && i < 50; ++i) {
                FormationSlot slot;
                slot.follower_id = static_cast<uint16_t>(i + 1);
                slot.offset_flu = _geometry.relative_positions[i];
                slot.gain_xy = 1.0f;
                slot.gain_z = 1.0f;
                slot.gain_yaw = 1.0f;
                _slots.push_back(slot);
                _follower_targets_flu.push_back(slot.offset_flu);
            }
            break;
        }
    }
    
    _follower_ids.clear();
    for (const auto& slot : _slots) {
        _follower_ids.push_back(slot.follower_id);
    }
}

Vector3 FormationController::rotateToFlu(const Vector3& vec_enu, const Quaternion& leader_orientation) {
    // ENU to FLU: rotate by -yaw around Z
    // Quaternion to yaw
    float yaw = std::atan2(2.0f * (leader_orientation.w * leader_orientation.z + leader_orientation.x * leader_orientation.y),
                          1.0f - 2.0f * (leader_orientation.y * leader_orientation.y + leader_orientation.z * leader_orientation.z));
    
    float cos_yaw = std::cos(-yaw);
    float sin_yaw = std::sin(-yaw);
    
    Vector3 result;
    result.x = cos_yaw * vec_enu.x - sin_yaw * vec_enu.y;
    result.y = sin_yaw * vec_enu.x + cos_yaw * vec_enu.y;
    result.z = vec_enu.z;
    
    return result;
}

Vector3 FormationController::rotateToEnu(const Vector3& vec_flu, const Quaternion& leader_orientation) {
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

bool FormationController::isFormationStable() const {
    if (!_is_leader) return false;
    
    // Check all followers within tolerances
    for (size_t i = 0; i < _follower_targets_flu.size(); ++i) {
        // Would check actual vs target for each follower
        // Simplified: return true if state is HOLDING
    }
    return _state == FormationState::HOLDING;
}

// MAVLink message handlers
void FormationController::handleFormationSetMessage(uint16_t src_id, const uint8_t* data, uint16_t len) {
    if (!_is_leader && src_id == _leader_id) {
        // Parse SWARM_FORMATION_SET message
        // Update local formation state
        // Implementation would unpack the MAVLink message
    }
}

void FormationController::handleFormationStateMessage(uint16_t src_id, const uint8_t* data, uint16_t len) {
    if (_is_leader) {
        // Parse SWARM_FORMATION_STATE message from follower
        // Update follower status tracking
    }
}

} // namespace swarm::formation