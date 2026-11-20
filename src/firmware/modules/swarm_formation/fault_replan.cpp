#include "fault_replan.hpp"
#include <algorithm>
#include <iostream>
#include <sstream>

namespace swarm::formation {

FaultReplan::FaultReplanResult FaultReplan::processFault(const AIInferenceResult& fault_inference,
                                                          const std::vector<FollowerState>& current_followers,
                                                          uint16_t leader_id,
                                                          const Eigen::Vector3f& leader_vel_flu,
                                                          uint64_t timestamp_us) {
    FaultReplanResult result;
    result.timestamp_us = timestamp_us;
    result.success = true;
    result.followers = current_followers;
    result.leader_velocity_cmd_flu = leader_vel_flu;
    
    uint8_t fault_type = static_cast<uint8_t>(fault_inference.metadata.fault.fault_type);
    float degradation = fault_inference.metadata.fault.degradation_pct;
    std::string component = fault_inference.metadata.fault.affected_component;
    
    // Check if leader or follower fault
    bool is_leader_fault = component.empty() || component.find("leader") != std::string::npos;
    
    if (is_leader_fault || fault_type == static_cast<uint8_t>(FaultType::GPS_SPOOF) ||
        fault_type == static_cast<uint8_t>(FaultType::IMU_DRIFT)) {
        // Leader fault - affects entire formation
        result = handleLeaderFault(fault_type, degradation, current_followers, timestamp_us);
        result.replan_reason = "Leader fault: " + fault_inference.class_name;
        return result;
    }
    
    // Follower fault - find affected follower
    uint16_t affected_id = 0;
    if (component.find("follower") != std::string::npos) {
        // Extract follower ID from component name
        // e.g., "follower_2_motor" -> 2
    }
    
    // Update follower states with fault
    for (auto& follower : result.followers) {
        if (affected_id == 0 || follower.sysid == affected_id) {
            // Check if this follower has the fault
            bool has_this_fault = (affected_id == 0 || follower.sysid == affected_id);
            
            if (has_this_fault) {
                follower.has_fault = true;
                follower.fault_type = fault_type;
                follower.fault_degradation = degradation;
                follower.fault_start_us = timestamp_us;
                
                // Compute fault offset
                Eigen::Vector3f offset = computeFaultOffset(follower);
                follower.current_offset_flu = follower.desired_offset_flu + offset;
                
                result.formation_changed = true;
                
                std::stringstream ss;
                ss << "Follower " << follower.sysid << " fault: " << fault_inference.class_name
                   << " degradation=" << degradation;
                result.warnings.push_back(ss.str());
            }
        }
    }
    
    // If multiple followers have faults, compact formation
    int fault_count = 0;
    for (const auto& f : result.followers) {
        if (f.has_fault) fault_count++;
    }
    
    if (fault_count > 1 && config_.enable_formation_compact) {
        result.followers = compactFormation(result.followers);
        result.formation_changed = true;
        result.replan_reason = "Multi-follower fault: compacting formation";
    }
    
    return result;
}

FaultReplan::FaultReplanResult FaultReplan::handleLeaderFault(uint8_t fault_type, float degradation,
                                                               const std::vector<FollowerState>& followers,
                                                               uint64_t timestamp_us) {
    FaultReplanResult result;
    result.timestamp_us = timestamp_us;
    result.success = true;
    result.followers = followers;
    result.leader_hold = true;
    result.leader_velocity_cmd_flu = Eigen::Vector3f::Zero();
    
    std::string fault_name;
    switch (static_cast<FaultType>(fault_type)) {
        case FaultType::MOTOR_DEGRADE: fault_name = "Motor Degradation"; break;
        case FaultType::BATTERY_IMBALANCE: fault_name = "Battery Imbalance"; break;
        case FaultType::GPS_SPOOF: fault_name = "GPS Spoofing"; break;
        case FaultType::IMU_DRIFT: fault_name = "IMU Drift"; break;
        case FaultType::SENSOR_FAULT: fault_name = "Sensor Fault"; break;
        case FaultType::COMM_DEGRADE: fault_name = "Comm Degradation"; break;
        default: fault_name = "Unknown";
    }
    
    result.replan_reason = "Leader fault: " + fault_name + " (" + std::to_string(degradation) + ")";
    
    // Leader holds position
    result.warnings.push_back("Leader holding position due to " + fault_name);
    
    // Trigger leader election if critical
    if (config_.leader_fault_triggers_election && 
        degradation > 0.5f &&  // > 50% degradation
        (fault_type == static_cast<uint8_t>(FaultType::MOTOR_DEGRADE) ||
         fault_type == static_cast<uint8_t>(FaultType::BATTERY_IMBALANCE) ||
         fault_type == static_cast<uint8_t>(FaultType::SENSOR_FAULT))) {
        
        uint16_t new_leader = electNewLeader(followers);
        if (new_leader != 0) {
            result.warnings.push_back("Leader election triggered: new leader " + std::to_string(new_leader));
            // Would need to coordinate with CBBA consensus
        }
    }
    
    // Adjust follower offsets for leader fault
    for (auto& follower : result.followers) {
        // Increase spacing for safety
        Eigen::Vector3f offset = follower.desired_offset_flu;
        float scale = 1.0f + degradation * 0.5f;  // Up to 50% more spacing
        follower.current_offset_flu = offset * scale;
        follower.current_offset_flu.z() = offset.z();  // Keep altitude
    }
    
    result.formation_changed = true;
    return result;
}

FaultReplan::FaultReplanResult FaultReplan::checkRecovery(const std::vector<FollowerState>& current_followers,
                                                           uint64_t timestamp_us) {
    FaultReplanResult result;
    result.timestamp_us = timestamp_us;
    result.success = true;
    result.followers = current_followers;
    
    bool any_recovered = false;
    
    for (auto& follower : result.followers) {
        if (follower.has_fault) {
            float time_elapsed = (timestamp_us - follower.fault_start_us) / 1e6f;
            
            if (time_elapsed > config_.recovery_check_interval_s && 
                follower.fault_degradation < config_.recovery_confidence_threshold) {
                
                // Check if fault recovered (would need actual health data)
                // For now, assume recovery if degradation decreased
                if (follower.fault_degradation < 0.1f) {
                    follower.has_fault = false;
                    follower.fault_type = 0;
                    follower.fault_degradation = 0.0f;
                    follower.current_offset_flu = follower.desired_offset_flu;
                    any_recovered = true;
                    
                    result.warnings.push_back("Follower " + std::to_string(follower.sysid) + " recovered from fault");
                }
            }
        }
    }
    
    if (any_recovered) {
        result.formation_changed = true;
        result.replan_reason = "Fault recovery detected";
    }
    
    return result;
}

Eigen::Vector3f FaultReplan::computeFaultOffset(const FollowerState& follower) const {
    auto it = config_.fault_offsets.find(follower.fault_type);
    if (it != config_.fault_offsets.end()) {
        Eigen::Vector3f offset = it->second.position_offset_flu;
        offset *= follower.fault_degradation;  // Scale by degradation
        return offset;
    }
    
    // Default offsets based on fault type
    switch (static_cast<FaultType>(follower.fault_type)) {
        case FaultType::MOTOR_DEGRADE: {
            // Move outward and up for motor issues
            Eigen::Vector3f dir = follower.desired_offset_flu;
            dir.z() = 0;
            if (dir.norm() > 0) dir.normalize();
            return dir * 2.0f * follower.fault_degradation + Eigen::Vector3f(0, 0, 1.0f * follower.fault_degradation);
        }
        case FaultType::BATTERY_IMBALANCE: {
            // Move to safer position, reduce altitude
            return Eigen::Vector3f(0, 0, -1.0f * follower.fault_degradation);
        }
        case FaultType::GPS_SPOOF:
        case FaultType::IMU_DRIFT:
        case FaultType::SENSOR_FAULT: {
            // Hold relative position more carefully
            return Eigen::Vector3f::Zero();
        }
        case FaultType::COMM_DEGRADE: {
            // Move closer to leader for better comm
            Eigen::Vector3f dir = -follower.desired_offset_flu;
            dir.z() = 0;
            if (dir.norm() > 0) dir.normalize();
            return dir * 1.0f * follower.fault_degradation;
        }
        default: {
            return Eigen::Vector3f::Zero();
        }
    }
}

std::vector<FaultReplan::FollowerState> FaultReplan::compactFormation(const std::vector<FollowerState>& followers) const {
    std::vector<FollowerState> compacted = followers;
    
    for (auto& follower : compacted) {
        if (!follower.has_fault) {
            // Reduce spacing for healthy followers
            Eigen::Vector3f offset = follower.desired_offset_flu;
            float scale = config_.compact_distance_factor;
            
            // Don't go below minimum spacing
            float min_spacing = config_.max_compact_distance_m;
            float current_spacing = offset.norm();
            
            if (current_spacing * scale < min_spacing && current_spacing > min_spacing) {
                scale = min_spacing / current_spacing;
            }
            
            follower.current_offset_flu = offset * scale;
            follower.current_offset_flu.z() = offset.z();  // Keep altitude
        }
    }
    
    return compacted;
}

FaultReplan::FollowerState FaultReplan::isolateFollower(const FollowerState& follower) const {
    FollowerState isolated = follower;
    isolated.has_fault = true;
    isolated.fault_type = static_cast<uint8_t>(FaultType::SENSOR_FAULT);
    isolated.fault_degradation = 1.0f;
    
    // Move away from formation
    Eigen::Vector3f dir = follower.desired_offset_flu;
    dir.z() = 0;
    if (dir.norm() > 0) dir.normalize();
    isolated.current_offset_flu = dir * config_.isolation_distance_m;
    isolated.current_offset_flu.z() = follower.desired_offset_flu.z();
    
    return isolated;
}

uint16_t FaultReplan::electNewLeader(const std::vector<FollowerState>& followers) const {
    // Simple election: highest priority (lowest sysid among healthy)
    uint16_t best = 0;
    float best_priority = -1.0f;
    
    for (const auto& f : followers) {
        if (!f.has_fault) {
            float priority = 1.0f / f.sysid;  // Lower ID = higher priority
            if (priority > best_priority) {
                best_priority = priority;
                best = f.sysid;
            }
        }
    }
    
    return best;
}

void FaultReplan::updateFollowerState(uint16_t sysid, const Eigen::Vector3f& pos_flu,
                                      const Eigen::Vector3f& vel_flu,
                                      const Eigen::Vector3f& desired_offset_flu) {
    auto it = follower_states_.find(sysid);
    if (it != follower_states_.end()) {
        it->second.position_flu = pos_flu;
        it->second.velocity_flu = vel_flu;
        it->second.desired_offset_flu = desired_offset_flu;
    } else {
        FollowerState fs;
        fs.sysid = sysid;
        fs.position_flu = pos_flu;
        fs.velocity_flu = vel_flu;
        fs.desired_offset_flu = desired_offset_flu;
        fs.current_offset_flu = desired_offset_flu;
        follower_states_[sysid] = fs;
    }
}

}  // namespace swarm::formation