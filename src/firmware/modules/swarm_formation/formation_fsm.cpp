#include "swarm_formation.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <algorithm>

namespace swarm::formation {

FormationStateMachine::FormationStateMachine() : _state(FormationState::DISSOLVING), _state_start_time_us(0) {}

FormationState FormationStateMachine::getState() const {
    return _state;
}

bool FormationStateMachine::transitionTo(FormationState new_state) {
    // Validate transition
    bool valid = false;
    switch (_state) {
        case FormationState::DISSOLVING:
            valid = (new_state == FormationState::FORMING);
            break;
        case FormationState::FORMING:
            valid = (new_state == FormationState::HOLDING || new_state == FormationState::DISSOLVING || 
                     new_state == FormationState::RECONFIGURING);
            break;
        case FormationState::HOLDING:
            valid = (new_state == FormationState::RECONFIGURING || new_state == FormationState::DISSOLVING);
            break;
        case FormationState::RECONFIGURING:
            valid = (new_state == FormationState::HOLDING || new_state == FormationState::DISSOLVING);
            break;
    }
    
    if (!valid) return false;
    
    _state = new_state;
    _state_start_time_us = Timestamp::nowUs();
    return true;
}

bool FormationStateMachine::update(const FormationStateMsg& leader_formation_msg, 
                                    const std::vector<FollowerStatus>& follower_statuses) {
    uint64_t now = Timestamp::nowUs();
    float time_in_state_s = static_cast<float>(now - _state_start_time_us) / 1e6f;
    
    switch (_state) {
        case FormationState::FORMING: {
            // Check if all followers have joined
            bool all_holding = true;
            for (auto status : follower_statuses) {
                if (status != FollowerStatus::HOLDING) {
                    all_holding = false;
                    break;
                }
            }
            
            if (all_holding && !follower_statuses.empty()) {
                transitionTo(FormationState::HOLDING);
            } else if (time_in_state_s > 30.0f) {  // Formation timeout
                transitionTo(FormationState::DISSOLVING);
            }
            break;
        }
        
        case FormationState::HOLDING: {
            // Check for reconfiguration request
            // In real implementation, would check for geometry change command
            
            // Check for follower loss
            for (auto status : follower_statuses) {
                if (status == FollowerStatus::LOST) {
                    // Trigger reconfiguration or dissolve
                    // For now, stay in HOLDING but could transition to RECONFIGURING
                }
            }
            break;
        }
        
        case FormationState::RECONFIGURING: {
            // Check if reconfiguration complete
            bool all_holding = true;
            for (auto status : follower_statuses) {
                if (status != FollowerStatus::HOLDING) {
                    all_holding = false;
                    break;
                }
            }
            
            if (all_holding) {
                transitionTo(FormationState::HOLDING);
            } else if (time_in_state_s > 10.0f) {  // Reconfiguration timeout
                transitionTo(FormationState::DISSOLVING);
            }
            break;
        }
        
        case FormationState::DISSOLVING: {
            // Formation dissolved
            break;
        }
    }
    
    return true;
}

float FormationStateMachine::getTimeInState() const {
    uint64_t now = Timestamp::nowUs();
    return static_cast<float>(now - _state_start_time_us) / 1e6f;
}

bool FormationStateMachine::isStable() const {
    return _state == FormationState::HOLDING;
}

bool FormationStateMachine::canAcceptFollower() const {
    return _state == FormationState::FORMING || _state == FormationState::HOLDING;
}

bool FormationStateMachine::canReconfigure() const {
    return _state == FormationState::HOLDING;
}

} // namespace swarm::formation