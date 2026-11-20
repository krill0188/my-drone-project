#include "takeoff_sequence.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include "swarm_comm/mavlink_router.hpp"
#include "mavlink.h"
#include <algorithm>
#include <cmath>

namespace swarm::formation {

TakeoffSequence::TakeoffSequence(FormationController& controller, MeshManager& mesh, const Config& config)
    : _controller(controller), _mesh(mesh), _config(config) {
    _state.current_phase = Phase::IDLE;
}

bool TakeoffSequence::start() {
    if (_state.current_phase != Phase::IDLE) {
        _state.abort_reason = "Takeoff sequence already started";
        return false;
    }
    
    // Reset state
    _state = TakeoffState{};
    _state.current_phase = Phase::PREARM_CHECKS;
    _state.phase_start_us = Timestamp::nowUs();
    
    return true;
}

bool TakeoffSequence::update(float dt) {
    if (_state.current_phase == Phase::IDLE || _state.current_phase == Phase::ABORTED) {
        return false;
    }
    
    if (_state.current_phase == Phase::COMPLETE) {
        return true;
    }
    
    bool phase_complete = false;
    
    switch (_state.current_phase) {
        case Phase::PREARM_CHECKS:
            phase_complete = handlePrearmChecks(dt);
            break;
        case Phase::ARMING:
            phase_complete = handleArming(dt);
            break;
        case Phase::LEADER_TAKEOFF:
            phase_complete = handleLeaderTakeoff(dt);
            break;
        case Phase::FOLLOWER_TAKEOFF:
            phase_complete = handleFollowerTakeoff(dt);
            break;
        case Phase::ALTITUDE_HOLD:
            phase_complete = handleAltitudeHold(dt);
            break;
        case Phase::FORMATION_ACQUISITION:
            phase_complete = handleFormationAcquisition(dt);
            break;
        case Phase::FORMATION_HOLD:
            phase_complete = handleFormationHold(dt);
            break;
        default:
            break;
    }
    
    if (phase_complete) {
        // Advance to next phase
        advancePhase();
    }
    
    return _state.current_phase == Phase::COMPLETE;
}

void TakeoffSequence::abort(const std::string& reason) {
    _state.current_phase = Phase::ABORTED;
    _state.abort_reason = reason;
    
    // Emergency disarm all
    disarmAllDrones();
    
    if (_completion_cb) {
        _completion_cb(false, "Aborted: " + reason);
    }
}

bool TakeoffSequence::handlePrearmChecks(float dt) {
    static bool checks_done = false;
    
    if (!checks_done) {
        bool all_passed = performPrearmChecks();
        
        if (!all_passed) {
            abort("Pre-arm checks failed");
            return false;
        }
        
        checks_done = true;
    }
    
    // Move to arming phase
    return true;
}

bool TakeoffSequence::handleArming(float dt) {
    static bool arming_sent = false;
    static bool arming_confirmed = false;
    
    if (!arming_sent) {
        bool success = armAllDrones();
        if (!success) {
            abort("Failed to arm all drones");
            return false;
        }
        arming_sent = true;
    }
    
    // Wait for arming confirmation from all drones
    // In real implementation, would check AgentState.armed for all drones
    static float wait_time = 0.0f;
    wait_time += 0.02f;  // 50 Hz update
    
    if (wait_time > 3.0f) {  // 3 second timeout
        // In real implementation, verify all drones armed
        arming_confirmed = true;
        wait_time = 0.0f;
    }
    
    if (arming_confirmed) {
        // Next phase depends on config
        if (_config.leader_takes_off_first) {
            // Next will be LEADER_TAKEOFF
        } else {
            // Next will be FOLLOWER_TAKEOFF
        }
        return true;
    }
    
    return false;
}

bool TakeoffSequence::handleLeaderTakeoff(float dt) {
    static bool takeoff_cmd_sent = false;
    static bool leader_airborne_confirmed = false;
    
    if (!takeoff_cmd_sent) {
        if (!_controller.isLeader()) {
            abort("Controller is not leader");
            return false;
        }
        
        uint16_t leader_id = _controller.getLeaderId();
        bool success = commandTakeoff(leader_id, _config.takeoff_altitude_m);
        
        if (!success) {
            abort("Failed to send takeoff command to leader");
            return false;
        }
        
        takeoff_cmd_sent = true;
        _state.leader_takeoff_pos = {0, 0, _config.takeoff_altitude_m};  // Simplified
    }
    
    // Wait for leader to reach target altitude
    // In real implementation, would monitor leader altitude via AgentState
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    
    float expected_time = _config.takeoff_altitude_m / _config.takeoff_speed_mps + 2.0f;  // 2s margin
    
    if (wait_time > expected_time) {
        // Assume leader reached altitude
        _state.leader_airborne = true;
        _state.leader_airborne_confirmed = true;
        
        if (_config.leader_follower_delay_s > 0) {
            // Wait for delay before follower takeoff
            return false;  // Stay in this phase, delay handled in next phase
        }
        
        return true;  // Move to follower takeoff
    }
    
    return false;
}

bool TakeoffSequence::handleFollowerTakeoff(float dt) {
    static bool takeoff_cmds_sent = false;
    static std::vector<uint16_t> followers_airborne;
    static float delay_wait = 0.0f;
    
    // Handle leader-follower delay
    if (_config.leader_follower_delay_s > 0) {
        delay_wait += 0.02f;
        if (delay_wait < _config.leader_follower_delay_s) {
            return false;
        }
    }
    
    if (!takeoff_cmds_sent) {
        auto follower_ids = _controller.getFollowerIds();
        
        for (uint16_t follower_id : follower_ids) {
            bool success = commandTakeoff(follower_id, _config.takeoff_altitude_m);
            if (!success) {
                abort("Failed to send takeoff command to follower " + std::to_string(follower_id));
                return false;
            }
        }
        
        takeoff_cmds_sent = true;
    }
    
    // Wait for all followers to reach altitude
    // In real implementation, would monitor each follower's altitude
    static float wait_time = 0.0f;
    wait_time += 0.02f;
    
    float expected_time = _config.takeoff_altitude_m / _config.takeoff_speed_mps + 2.0f;
    
    if (wait_time > expected_time) {
        // Assume all followers airborne
        _state.followers_airborne.clear();
        auto follower_ids = _controller.getFollowerIds();
        for (uint16_t id : follower_ids) {
            _state.followers_airborne.push_back(id);
        }
        
        return true;
    }
    
    return false;
}

bool TakeoffSequence::handleAltitudeHold(float dt) {
    // Hold at target altitude for configured time before formation acquisition
    static float hold_time = 0.0f;
    hold_time += 0.02f;
    
    if (hold_time >= _config.hold_altitude_before_formation_s) {
        return true;
    }
    
    return false;
}

bool TakeoffSequence::handleFormationAcquisition(float dt) {
    // Move followers to their formation slots
    static bool slots_assigned = false;
    
    if (!_state.formation_acquired) {
        auto slots = _controller.getFormationSlots();
        
        for (const auto& slot : slots) {
            if (!_controller.moveToFormationSlot(slot.follower_id, slot.offset_flu)) {
                abort("Failed to command follower to formation slot");
                return false;
            }
        }
        
        _state.formation_acquired = checkFormationAcquired();
    } else {
        // Check if formation is stable
        if (checkFormationAcquired()) {
            return true;
        }
    }
    
    // Timeout check
    if (isPhaseTimeout(_config.formation_acquisition_timeout_s)) {
        abort("Formation acquisition timeout");
        return false;
    }
    
    return false;
}

bool TakeoffSequence::handleFormationHold(float dt) {
    // Formation acquired and holding - takeoff sequence complete
    if (_completion_cb) {
        _completion_cb(true, "Takeoff sequence completed successfully");
    }
    return true;
}

void TakeoffSequence::advancePhase() {
    switch (_state.current_phase) {
        case Phase::PREARM_CHECKS:
            _state.current_phase = Phase::ARMING;
            break;
        case Phase::ARMING:
            if (_config.leader_takes_off_first) {
                _state.current_phase = Phase::LEADER_TAKEOFF;
            } else {
                _state.current_phase = Phase::FOLLOWER_TAKEOFF;
            }
            break;
        case Phase::LEADER_TAKEOFF:
            _state.current_phase = Phase::FOLLOWER_TAKEOFF;
            break;
        case Phase::FOLLOWER_TAKEOFF:
            _state.current_phase = Phase::ALTITUDE_HOLD;
            break;
        case Phase::ALTITUDE_HOLD:
            _state.current_phase = Phase::FORMATION_ACQUISITION;
            break;
        case Phase::FORMATION_ACQUISITION:
            _state.current_phase = Phase::FORMATION_HOLD;
            break;
        case Phase::FORMATION_HOLD:
            _state.current_phase = Phase::COMPLETE;
            break;
        default:
            break;
    }
    
    _state.phase_start_us = Timestamp::nowUs();
}

bool TakeoffSequence::performPrearmChecks() {
    // Battery check
    if (!checkBatterySufficient()) return false;
    
    // GPS fix check
    if (!checkGPSFix()) return false;
    
    // Weather check
    if (!checkWeatherConditions()) return false;
    
    // Geofence clearance
    if (!checkGeofenceClearance()) return false;
    
    // All drones present
    auto agent_states = _controller.getAllAgentStates();
    if (agent_states.empty()) return false;
    
    // Formation configuration valid
    auto formation_state = _controller.getFormationStateMsg();
    if (formation_state.follower_ids.empty()) return false;
    
    return true;
}

bool TakeoffSequence::armAllDrones() {
    // Send ARM command to all drones via MAVLink
    // In real implementation, would send VEHICLE_CMD_COMPONENT_ARM_DISARM
    return true;  // Simplified
}

bool TakeoffSequence::disarmAllDrones() {
    // Send DISARM command to all drones
    return true;  // Simplified
}

bool TakeoffSequence::commandTakeoff(uint16_t sysid, float altitude) {
    // Send MAVLink TAKEOFF command
    // MAV_CMD_NAV_TAKEOFF with param7 = altitude
    return true;  // Simplified
}

bool TakeoffSequence::waitForAltitude(uint16_t sysid, float target_alt, float tolerance) {
    // Monitor altitude from AgentState
    return true;  // Simplified
}

bool TakeoffSequence::moveToFormationSlot(uint16_t follower_id, const Vector3& target_offset_flu) {
    // Command follower to move to formation slot
    // Uses FormationController's internal logic
    return true;  // Simplified
}

bool TakeoffSequence::checkFormationAcquired() {
    // Check if all followers within tolerance of their slots
    auto formation_msg = _controller.getFormationStateMsg();
    
    for (size_t i = 0; i < formation_msg.follower_ids.size(); ++i) {
        if (i < formation_msg.position_errors_flu.size()) {
            const auto& err = formation_msg.position_errors_flu[i];
            float lateral = std::sqrt(err.x * err.x + err.y * err.y);
            float vertical = std::abs(err.z);
            
            if (lateral > _config.position_tolerance_m || vertical > _config.position_tolerance_m) {
                return false;
            }
        }
    }
    
    return true;
}

bool TakeoffSequence::checkBatterySufficient() {
    // Check all drones have sufficient battery (> 50% for takeoff)
    return true;  // Simplified
}

bool TakeoffSequence::checkGPSFix() {
    // Check all drones have RTK fix
    return true;  // Simplified
}

bool TakeoffSequence::checkWeatherConditions() {
    // Check wind speed < 10 m/s, no precipitation
    return true;  // Simplified
}

bool TakeoffSequence::checkGeofenceClearance() {
    // Check takeoff position and climb path clear of geofence
    return true;  // Simplified
}

float TakeoffSequence::getPhaseElapsedS() const {
    return static_cast<float>(Timestamp::nowUs() - _state.phase_start_us) / 1e6f;
}

bool TakeoffSequence::isPhaseTimeout(float timeout_s) const {
    return getPhaseElapsedS() > timeout_s;
}

} // namespace swarm::formation