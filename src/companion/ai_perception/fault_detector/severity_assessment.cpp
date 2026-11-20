#include "severity_assessment.hpp"
#include <algorithm>
#include <sstream>
#include <iostream>

namespace swarm::ai {

FaultSeverity SeverityAssessment::assessSingle(
    const FaultClassifier::FaultDetection& detection,
    const HealthMonitor::SystemHealth& health,
    bool is_leader) const {
    
    auto it = config_.fault_thresholds.find(detection.fault_type);
    if (it == config_.fault_thresholds.end()) {
        // Default thresholds
        if (detection.confidence >= 0.9f) return FaultSeverity::EMERGENCY;
        if (detection.confidence >= 0.75f) return FaultSeverity::CRITICAL;
        if (detection.confidence >= 0.5f) return FaultSeverity::WARNING;
        return FaultSeverity::INFO;
    }
    
    const auto& thresholds = it->second;
    FaultSeverity severity = FaultSeverity::INFO;
    
    // Base severity from confidence
    if (detection.confidence >= thresholds.emergency_confidence) {
        severity = FaultSeverity::EMERGENCY;
    } else if (detection.confidence >= thresholds.critical_confidence) {
        severity = FaultSeverity::CRITICAL;
    } else if (detection.confidence >= thresholds.warning_confidence) {
        severity = FaultSeverity::WARNING;
    }
    
    // Adjust based on degradation
    if (detection.degradation_pct >= thresholds.emergency_degradation) {
        severity = std::max(severity, FaultSeverity::EMERGENCY);
    } else if (detection.degradation_pct >= thresholds.critical_degradation) {
        severity = std::max(severity, FaultSeverity::CRITICAL);
    } else if (detection.degradation_pct >= thresholds.warning_degradation) {
        severity = std::max(severity, FaultSeverity::WARNING);
    }
    
    // Adjust based on system health
    if (health.overall_score < config_.system_critical_score) {
        severity = std::max(severity, FaultSeverity::CRITICAL);
    } else if (health.overall_score < config_.system_warning_score) {
        severity = std::max(severity, FaultSeverity::WARNING);
    }
    
    // Formation role adjustment
    if (config_.consider_formation_role) {
        float multiplier = is_leader ? config_.leader_severity_multiplier : config_.follower_severity_multiplier;
        
        // Convert to numeric for multiplication
        int sev_num = static_cast<int>(severity);
        sev_num = std::min(3, static_cast<int>(sev_num * multiplier));
        severity = static_cast<FaultSeverity>(sev_num);
    }
    
    // Time-based escalation
    if (config_.enable_time_escalation && detection.time_since_onset_s > 0) {
        severity = escalateTime(severity, detection.time_since_onset_s);
    }
    
    return severity;
}

FaultSeverity SeverityAssessment::escalateTime(FaultSeverity base, float time_since_onset) const {
    if (base == FaultSeverity::WARNING && time_since_onset > config_.warning_to_critical_time_s) {
        return FaultSeverity::CRITICAL;
    }
    if (base == FaultSeverity::CRITICAL && time_since_onset > config_.critical_to_emergency_time_s) {
        return FaultSeverity::EMERGENCY;
    }
    return base;
}

FaultSeverity SeverityAssessment::escalateMultiFault(
    const std::map<FaultType, FaultSeverity>& severities) const {
    
    if (!config_.enable_multi_fault_escalation) {
        // Return maximum
        FaultSeverity max_sev = FaultSeverity::INFO;
        for (const auto& [type, sev] : severities) {
            max_sev = std::max(max_sev, sev);
        }
        return max_sev;
    }
    
    int warning_count = 0;
    int critical_count = 0;
    int emergency_count = 0;
    
    for (const auto& [type, sev] : severities) {
        switch (sev) {
            case FaultSeverity::WARNING: warning_count++; break;
            case FaultSeverity::CRITICAL: critical_count++; break;
            case FaultSeverity::EMERGENCY: emergency_count++; break;
            default: break;
        }
    }
    
    // Escalation rules
    if (emergency_count > 0) return FaultSeverity::EMERGENCY;
    if (critical_count > config_.max_simultaneous_critical) return FaultSeverity::EMERGENCY;
    if (critical_count > 0 && warning_count > 0) return FaultSeverity::EMERGENCY;
    if (warning_count > config_.max_simultaneous_warnings) return FaultSeverity::CRITICAL;
    if (critical_count > 0) return FaultSeverity::CRITICAL;
    if (warning_count > 0) return FaultSeverity::WARNING;
    
    return FaultSeverity::INFO;
}

FaultSeverity SeverityAssessment::computeSystemSeverity(
    const std::map<FaultType, FaultSeverity>& fault_severities,
    const HealthMonitor::SystemHealth& health) const {
    
    // Start with multi-fault escalation
    FaultSeverity system_sev = escalateMultiFault(fault_severities);
    
    // Consider overall health
    if (health.overall_score < config_.system_critical_score) {
        system_sev = std::max(system_sev, FaultSeverity::CRITICAL);
    } else if (health.overall_score < config_.system_warning_score) {
        system_sev = std::max(system_sev, FaultSeverity::WARNING);
    }
    
    // Consider failed sensors
    if (!health.failed_sensors.empty()) {
        system_sev = std::max(system_sev, FaultSeverity::CRITICAL);
    } else if (!health.degraded_sensors.empty()) {
        system_sev = std::max(system_sev, FaultSeverity::WARNING);
    }
    
    // Consider critical subsystems
    for (const auto& [sub, score] : health.subsystem_scores) {
        if (sub == "propulsion" && score < 0.5f) {
            system_sev = std::max(system_sev, FaultSeverity::CRITICAL);
        } else if (sub == "navigation" && score < 0.5f) {
            system_sev = std::max(system_sev, FaultSeverity::CRITICAL);
        } else if (sub == "power" && score < 0.3f) {
            system_sev = std::max(system_sev, FaultSeverity::EMERGENCY);
        } else if (sub == "communication" && score < 0.4f) {
            system_sev = std::max(system_sev, FaultSeverity::WARNING);
        }
    }
    
    return system_sev;
}

SeverityAssessment::SeverityResult SeverityAssessment::assess(
    const std::vector<FaultClassifier::FaultDetection>& detections,
    const HealthMonitor::SystemHealth& health,
    bool is_leader,
    const Eigen::Vector3f& position_enu,
    const std::vector<uint16_t>& formation_members) const {
    
    SeverityResult result;
    result.timestamp_us = detections.empty() ? 0 : detections[0].timestamp_us;
    result.drone_id = detections.empty() ? 0 : detections[0].drone_id;
    result.is_leader = is_leader;
    
    // Assess each fault
    for (const auto& det : detections) {
        FaultSeverity sev = assessSingle(det, health, is_leader);
        result.fault_severities[det.fault_type] = sev;
        result.fault_actions[det.fault_type] = getActionForSeverity(sev);
        result.time_since_onset[det.fault_type] = det.time_since_onset_s;
    }
    
    // System severity
    result.system_severity = computeSystemSeverity(result.fault_severities, health);
    result.system_action = getActionForSeverity(result.system_severity);
    
    // Replan check
    result.requires_replan = needsReplan(result);
    if (result.requires_replan) {
        result.replan_reason = generateReplanReason(result);
    }
    
    // Formation risk
    result.formation_risk_score = getFormationRisk(result, formation_members);
    
    // Escalation warnings
    for (const auto& [type, sev] : result.fault_severities) {
        auto it = config_.fault_thresholds.find(type);
        if (it != config_.fault_thresholds.end()) {
            float time_onset = result.time_since_onset.count(type) ? result.time_since_onset.at(type) : 0;
            
            if (sev == FaultSeverity::WARNING && time_onset > config_.warning_to_critical_time_s) {
                result.escalation_warnings.push_back(
                    FAULT_CLASS_NAMES[static_cast<int>(type)] + " approaching CRITICAL");
            }
            if (sev == FaultSeverity::CRITICAL && time_onset > config_.critical_to_emergency_time_s) {
                result.escalation_warnings.push_back(
                    FAULT_CLASS_NAMES[static_cast<int>(type)] + " approaching EMERGENCY");
            }
        }
    }
    
    // Summary
    result.summary = generateSummary(result);
    
    return result;
}

RecommendedAction SeverityAssessment::getActionForSeverity(FaultSeverity severity) const {
    auto it = config_.severity_to_action.find(severity);
    if (it != config_.severity_to_action.end()) {
        return it->second;
    }
    return RecommendedAction::CONTINUE;
}

bool SeverityAssessment::needsReplan(const SeverityResult& result) const {
    // Replan if system action is RTL or emergency land
    if (result.system_action >= RecommendedAction::RTL) return true;
    
    // Replan if any fault requires replan (from detection)
    // This would be set in the detection itself
    
    // Replan if leader has critical fault
    if (result.is_leader && result.system_severity >= FaultSeverity::CRITICAL) return true;
    
    // Replan if multiple critical faults
    int critical_count = 0;
    for (const auto& [type, sev] : result.fault_severities) {
        if (sev >= FaultSeverity::CRITICAL) critical_count++;
    }
    if (critical_count > 1) return true;
    
    return false;
}

float SeverityAssessment::getFormationRisk(const SeverityResult& result,
                                           const std::vector<uint16_t>& formation_members) const {
    if (formation_members.empty()) return 0.0f;
    
    float risk = 0.0f;
    
    // Base risk from system severity
    switch (result.system_severity) {
        case FaultSeverity::EMERGENCY: risk = 1.0f; break;
        case FaultSeverity::CRITICAL: risk = 0.7f; break;
        case FaultSeverity::WARNING: risk = 0.4f; break;
        default: risk = 0.1f; break;
    }
    
    // Leader failure is higher risk
    if (result.is_leader) {
        risk = std::min(1.0f, risk * 1.5f);
    }
    
    // Propulsion faults are highest risk for formation
    for (const auto& [type, sev] : result.fault_severities) {
        if (type == FaultType::MOTOR_DEGRADE || type == FaultType::BATTERY_IMBALANCE) {
            risk = std::max(risk, 0.8f);
        }
        if (type == FaultType::GPS_SPOOF || type == FaultType::IMU_DRIFT) {
            risk = std::max(risk, 0.6f);
        }
        if (type == FaultType::COMM_DEGRADE) {
            risk = std::max(risk, 0.5f);
        }
    }
    
    return std::clamp(risk, 0.0f, 1.0f);
}

std::string SeverityAssessment::generateSummary(const SeverityResult& result) const {
    std::stringstream ss;
    
    ss << "System: " << FAULT_SEVERITY_NAMES[static_cast<int>(result.system_severity)];
    ss << " | Action: " << RECOMMENDED_ACTION_NAMES[static_cast<int>(result.system_action)];
    
    if (result.requires_replan) {
        ss << " | REPLAN REQUIRED";
    }
    
    if (result.is_leader) {
        ss << " | LEADER";
    }
    
    ss << " | Formation Risk: " << std::to_string(result.formation_risk_score).substr(0, 4);
    
    if (!result.fault_severities.empty()) {
        ss << " | Faults: ";
        bool first = true;
        for (const auto& [type, sev] : result.fault_severities) {
            if (!first) ss << ", ";
            ss << FAULT_CLASS_NAMES[static_cast<int>(type)] << "=" 
               << FAULT_SEVERITY_NAMES[static_cast<int>(sev)];
            first = false;
        }
    }
    
    if (!result.escalation_warnings.empty()) {
        ss << " | Warnings: ";
        for (size_t i = 0; i < result.escalation_warnings.size(); ++i) {
            if (i > 0) ss << "; ";
            ss << result.escalation_warnings[i];
        }
    }
    
    return ss.str();
}

std::string SeverityAssessment::generateReplanReason(const SeverityResult& result) const {
    std::stringstream ss;
    
    if (result.system_action >= RecommendedAction::RTL) {
        ss << "System action requires RTL/emergency landing";
    }
    
    if (result.is_leader && result.system_severity >= FaultSeverity::CRITICAL) {
        ss << "Leader critical fault";
    }
    
    int critical_count = 0;
    for (const auto& [type, sev] : result.fault_severities) {
        if (sev >= FaultSeverity::CRITICAL) critical_count++;
    }
    if (critical_count > 1) {
        ss << "Multiple critical faults (" << critical_count << ")";
    }
    
    return ss.str();
}

// Static arrays for string conversion
static const std::vector<std::string> FAULT_CLASS_NAMES = {
    "imu_drift", "gps_spoof", "motor_degrade", "battery_imbalance", "sensor_fault", "comm_degrade"
};

static const std::vector<std::string> FAULT_SEVERITY_NAMES = {
    "INFO", "WARNING", "CRITICAL", "EMERGENCY"
};

static const std::vector<std::string> RECOMMENDED_ACTION_NAMES = {
    "CONTINUE", "DEGRADED", "RTL", "EMERGENCY_LAND"
};

}  // namespace swarm::ai