#include "landing_sequence.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include "swarm_comm/mavlink_router.hpp"
#include "mavlink.h"
#include <algorithm>
#include <cmath>

namespace swarm::formation {

LandingSequence::LandingSequence(FormationController& controller, MeshManager& mesh,
                                  const std::vector<LandingPad>& pads, const Config& config)
    : _controller(controller), _mesh(mesh), _config(config), _landing_pads(pads) {
    _state.current_phase = Phase::IDLE;
}

bool LandingSequence::start() {
    if (_state.current_phase != Phase::IDLE) {
        _state.abort_reason = "Landing sequence already started";
        return false;
    }
    
    // Assign landing pads if not already assigned
    if (_landing_pads.empty()) {
        if (!assignLandingPads()) {
            _state.abort_reason = "Failed to assign landing pads";
            return false;
        }
    }
    
    // Check battery sufficient for landing
    if (!checkBatteryForLanding()) {
        _state.abort_reason = "Insufficient battery for safe landing";
        return false;
    }
    
    // Check weather
    if (!checkWeatherForLanding()) {
        _state.abort_reason = "Weather conditions unsafe for landing";
        return false;
    }
    
    _state = LandingState{};
    _state.current_phase = Phase::RTL_INITIATED;
    _state.phase_start_us = Timestamp::nowUs();
    _state.leader_id = _controller.getLeaderId();
    
    return true;
}

bool LandingSequence::update(float dt) {
    if (_state.current_phase == Phase::IDLE || _state.current_phase == Phase::ABORTED) {
        return false;
    }
    
    if (_state.current_phase == Phase::COMPLETE) {
        return true;
    }
    
    bool phase_complete = false;
    
    switch (_state.current_phase) {
        case Phase::RTL_INITIATED:
            phase_complete = handleRtlInitiated(dt);
            break;
        case Phase::RTL_CLIMB:
            phase_complete = handleRtlClimb(dt);
            break;
        case Phase::RTL_TRANSIT:
            phase_complete = handleRtlTransit(dt);
            break;
        case Phase::RTL_HOVER:
            phase_complete = handleRtlHover(dt);
            break;
        case Phase::PRECISION_LANDING:
            phase_complete = handlePrecisionLanding(dt);
            break;
        case Phase::FINAL_DESCENT:
            phase_complete = handleFinalDescent(dt);
            break;
        case Phase::TOUCHDOWN:
            phase_complete = handleTouchdown(dt);
            break;
        case Phase::MOTOR_DISARM:
            phase_complete = handleMotorDisarm(dt);
            break;
        default:
            break;
    }
    
    if (phase_complete) {
        advancePhase();
    }
    
    return _state.current_phase == Phase::COMPLETE;
}

void LandingSequence::abort(const std::string& reason) {
    _state.current_phase = Phase::ABORTED;
    _state.abort_reason = reason;
    
    // Emergency stop all motors
    emergencyStop();
    
    if (_completion_cb) {
        _completion_cb(false, "Aborted: " + reason);
    }
}

bool LandingSequence::handleRtlInitiated(float dt) {
    // Send RTL command to all drones
    static bool rtl_sent = false;
    
    if (!rtl_sent) {
        auto all_ids = _controller.getAllDroneIds();
        
        if (_config.leader_lands_first) {
            // Send RTL to leader first
            if (commandRtl(_state.leader_id)) {
                rtl_sent = true;
            }
        } else {
            // Send to all simultaneously
            bool all_sent = true;
            for (uint16_t id : _controller.getAllDroneIds()) {
                if (!commandRtl(id)) {
                    all_sent = false;
                }
            }
            if (all_sent) rtl_sent = true;
        }
    }
    
    if (rtl_sent) {
        // Wait briefly for RTL mode confirmation
        static float wait_time = 0.0f;
        wait_time += 0.02f;
        if (wait_time > 2.0f) {
            return true;
        }
    }
    
    return false;
}

bool LandingSequence::handleRtlClimb(float dt) {
    // Wait for all drones to reach RTL altitude
    // In real implementation, would monitor altitude of each drone
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    
    // Estimate climb time
    float climb_time = (_config.rtl_altitude_m - 10.0f) / 3.0f + 5.0f;  // 3 m/s climb + margin
    
    if (wait_time > climb_time) {
        // All drones should be at RTL altitude
        return true;
    }
    
    return false;
}

bool LandingSequence::handleRtlTransit(float dt) {
    // Wait for all drones to reach home position
    // In real implementation, would monitor position of each drone
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    
    // Estimate transit time based on max distance / speed
    float max_distance = 2000.0f;  // meters
    float transit_speed = 10.0f;   // m/s
    float expected_time = max_distance / transit_speed + 30.0f;  // + margin
    
    if (wait_time > expected_time) {
        return true;
    }
    
    return false;
}

bool LandingSequence::handleRtlHover(float dt) {
    // Hover at home position before landing
    static float hover_time = 0.0f;
    hover_time += 0.02f;
    
    if (hover_time >= 5.0f) {  // 5 second hover
        return true;
    }
    
    return false;
}

bool LandingSequence::handlePrecisionLanding(float dt) {
    if (!_config.use_precision_landing) {
        return true;  // Skip precision landing
    }
    
    // Check if all drones have precision landing ready (IR marker, AprilTag detected)
    static bool precision_cmds_sent = false;
    
    if (!precision_cmds_sent) {
        for (const auto& pad : _state.landing_pads) {
            if (pad.has_precision_target) {
                if (!commandPrecisionLand(pad.drone_id, pad.position_enu)) {
                    abort("Failed to send precision landing command");
                    return false;
                }
            }
        }
        precision_cmds_sent = true;
    }
    
    // Wait for all drones to enter precision landing mode
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    
    if (wait_time > 10.0f) {  // 10 second timeout for precision lock
        return true;
    }
    
    return false;
}

bool LandingSequence::handleFinalDescent(float dt) {
    // Controlled descent to landing
    static bool land_cmds_sent = false;
    
    if (!land_cmds_sent) {
        bool all_sent = true;
        for (const auto& pad : _state.landing_pads) {
            if (!commandLand(pad.drone_id)) {
                all_sent = false;
            }
        }
        if (all_sent) land_cmds_sent = true;
    }
    
    if (!land_cmds_sent) return false;
    
    // Wait for all drones to reach ground
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    
    // Check each drone for touchdown
    static std::vector<uint16_t> drones_touched_down;
    
    for (const auto& pad : _state.landing_pads) {
        if (std::find(_state.drones_landed.begin(), _state.drones_landed.end(), pad.drone_id) 
            == _state.drones_landed.end()) {
            if (checkTouchdown(pad.drone_id)) {
                _state.drones_landed.push_back(pad.drone_id);
            }
        }
    }
    
    // All drones landed?
    if (_state.drones_landed.size() >= _state.landing_pads.size()) {
        return true;
    }
    
    // Timeout
    if (wait_time > 30.0f) {  // 30 second descent timeout
        abort("Descent timeout");
        return false;
    }
    
    return false;
}

bool LandingSequence::handleTouchdown(float dt) {
    // Verify all drones on ground, motors still running
    // Wait for stabilization
    static float stabilize_time = 0.0f;
    stabilize_time += 0.02f;
    
    if (stabilize_time >= 2.0f) {  // 2 second stabilize
        return true;
    }
    
    return false;
}

bool LandingSequence::handleMotorDisarm(float dt) {
    // Disarm motors for landed drones
    static bool disarm_sent = false;
    static bool all_disarmed = false;
    
    if (!disarm_sent) {
        for (uint16_t id : _state.drones_landed) {
            if (std::find(_state.motors_disarmed.begin(), _state.motors_disarmed.end(), id) 
                == _state.motors_disarmed.end()) {
                disarmMotor(id);
            }
        }
        disarm_sent = true;
    }
    
    // Verify all motors disarmed
    if (_state.motors_disarmed.size() >= _state.drones_landed.size()) {
        all_disarmed = true;
    }
    
    if (all_disarmed) {
        return true;
    }
    
    // Timeout
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    if (wait_time > 5.0f) {
        // Force completion even if some motors not confirmed disarmed
        return true;
    }
    
    return false;
}

bool LandingSequence::assignLandingPads() {
    // If pads not provided, assign automatically based on formation
    if (_landing_pads.empty()) {
        auto formation_slots = _controller.getFormationSlots();
        auto home_pos = _controller.getHomePosition();
        
        for (size_t i = 0; i < formation_slots.size(); ++i) {
            LandingPad pad;
            pad.drone_id = formation_slots[i].follower_id;
            pad.position_enu = home_pos + formation_slots[i].offset_flu;  // Simplified
            pad.heading_deg = 0.0f;
            pad.has_precision_target = _config.use_precision_landing;
            _landing_pads.push_back(pad);
        }
        
        // Add leader pad
        LandingPad leader_pad;
        leader_pad.drone_id = _controller.getLeaderId();
        leader_pad.position_enu = home_pos;
        leader_pad.heading_deg = 0.0f;
        leader_pad.has_precision_target = _config.use_precision_landing;
        _landing_pads.insert(_landing_pads.begin(), leader_pad);
    }
    
    _state.landing_pads = _landing_pads;
    return true;
}

bool LandingSequence::commandRtl(uint16_t sysid) {
    // Send MAVLink RTL command
    // MAV_CMD_NAV_RETURN_TO_LAUNCH
    return true;  // Simplified
}

bool LandingSequence::commandLand(uint16_t sysid) {
    // Send MAVLink LAND command
    // MAV_CMD_NAV_LAND
    return true;  // Simplified
}

bool LandingSequence::commandPrecisionLand(uint16_t sysid, const Vector3& pad_position) {
    // Send MAVLink precision landing command
    // MAV_CMD_NAV_LAND with precision targeting
    return true;  // Simplified
}

bool LandingSequence::waitForPosition(uint16_t sysid, const Vector3& target, float tolerance) {
    // Monitor AgentState position
    return true;  // Simplified
}

bool LandingSequence::waitForAltitude(uint16_t sysid, float target_alt, float tolerance) {
    // Monitor AgentState altitude
    return true;  // Simplified
}

bool LandingSequence::checkPrecisionLandingReady(uint16_t sysid) {
    // Check if precision landing target acquired (IR marker, AprilTag)
    return true;  // Simplified
}

bool LandingSequence::checkTouchdown(uint16_t sysid) {
    // Check if drone has landed (altitude near zero, zero velocity, weight on wheels)
    return true;  // Simplified
}

void LandingSequence::disarmMotor(uint16_t sysid) {
    // Send MAVLink disarm command
    // MAV_CMD_COMPONENT_ARM_DISARM with param1=0
    _state.motors_disarmed.push_back(sysid);
}

bool LandingSequence::checkSeparation(const std::vector<uint16_t>& landed_ids) {
    // Check minimum separation between landed drones
    for (size_t i = 0; i < landed_ids.size(); ++i) {
        for (size_t j = i + 1; j < landed_ids.size(); ++j) {
            // Get positions of landed drones
            // Check distance > _config.separation_min_m
        }
    }
    return true;  // Simplified
}

bool LandingSequence::checkBatteryForLanding() {
    // Check all drones have sufficient battery for landing (> 15%)
    return true;  // Simplified
}

bool LandingSequence::checkWeatherForLanding() {
    // Check wind < 5 m/s, no precipitation, visibility > 1000m
    return true;  // Simplified
}

float LandingSequence::getPhaseElapsedS() const {
    return static_cast<float>(Timestamp::nowUs() - _state.phase_start_us) / 1e6f;
}

void LandingSequence::advancePhase() {
    switch (_state.current_phase) {
        case Phase::RTL_INITIATED:
            _state.current_phase = Phase::RTL_CLIMB;
            break;
        case Phase::RTL_CLIMB:
            _state.current_phase = Phase::RTL_TRANSIT;
            break;
        case Phase::RTL_TRANSIT:
            _state.current_phase = Phase::RTL_HOVER;
            break;
        case Phase::RTL_HOVER:
            if (_config.use_precision_landing) {
                _state.current_phase = Phase::PRECISION_LANDING;
            } else {
                _state.current_phase = Phase::FINAL_DESCENT;
            }
            break;
        case Phase::PRECISION_LANDING:
            _state.current_phase = Phase::FINAL_DESCENT;
            break;
        case Phase::FINAL_DESCENT:
            _state.current_phase = Phase::TOUCHDOWN;
            break;
        case Phase::TOUCHDOWN:
            _state.current_phase = Phase::MOTOR_DISARM;
            break;
        case Phase::MOTOR_DISARM:
            _state.current_phase = Phase::COMPLETE;
            break;
        default:
            break;
    }
    
    _state.phase_start_us = Timestamp::nowUs();
}

void LandingSequence::emergencyStop() {
    // Immediately disarm all motors
    for (uint16_t id : _controller.getAllDroneIds()) {
        disarmMotor(id);
    }
}

} // namespace swarm::formation