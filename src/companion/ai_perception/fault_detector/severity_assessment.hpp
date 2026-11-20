#ifndef SEVERITY_ASSESSMENT_HPP
#define SEVERITY_ASSESSMENT_HPP

#include "fault_classifier.hpp"
#include "health_monitor.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <functional>

namespace swarm::ai {

struct SeverityAssessmentConfig {
    // Severity thresholds per fault type
    struct FaultThresholds {
        float warning_confidence = 0.5f;
        float critical_confidence = 0.75f;
        float emergency_confidence = 0.9f;
        float warning_degradation = 0.1f;
        float critical_degradation = 0.3f;
        float emergency_degradation = 0.5f;
    };
    
    std::map<FaultType, FaultThresholds> fault_thresholds = {
        {FaultType::IMU_DRIFT, {0.5f, 0.7f, 0.85f, 0.05f, 0.15f, 0.3f}},
        {FaultType::GPS_SPOOF, {0.6f, 0.8f, 0.9f, 0.1f, 0.3f, 0.5f}},
        {FaultType::MOTOR_DEGRADE, {0.5f, 0.7f, 0.85f, 0.1f, 0.25f, 0.4f}},
        {FaultType::BATTERY_IMBALANCE, {0.6f, 0.8f, 0.9f, 0.05f, 0.15f, 0.3f}},
        {FaultType::SENSOR_FAULT, {0.5f, 0.7f, 0.85f, 0.1f, 0.3f, 0.5f}},
        {FaultType::COMM_DEGRADE, {0.5f, 0.7f, 0.85f, 0.2f, 0.4f, 0.6f}}
    };
    
    // System-level thresholds
    float system_critical_score = 0.4f;
    float system_warning_score = 0.7f;
    
    // Action mapping
    std::map<FaultSeverity, RecommendedAction> severity_to_action = {
        {FaultSeverity::INFO, RecommendedAction::CONTINUE},
        {FaultSeverity::WARNING, RecommendedAction::DEGRADED},
        {FaultSeverity::CRITICAL, RecommendedAction::RTL},
        {FaultSeverity::EMERGENCY, RecommendedAction::EMERGENCY_LAND}
    };
    
    // Multi-fault escalation
    bool enable_multi_fault_escalation = true;
    int max_simultaneous_warnings = 3;
    int max_simultaneous_critical = 1;
    
    // Time-based escalation
    bool enable_time_escalation = true;
    float warning_to_critical_time_s = 30.0f;
    float critical_to_emergency_time_s = 60.0f;
    
    // Formation-aware
    bool consider_formation_role = true;  // Leader vs follower
    float leader_severity_multiplier = 1.2f;
    float follower_severity_multiplier = 0.8f;
};

struct SeverityResult {
    uint64_t timestamp_us;
    uint16_t drone_id;
    
    // Per-fault severity
    std::map<FaultType, FaultSeverity> fault_severities;
    std::map<FaultType, RecommendedAction> fault_actions;
    
    // Overall system severity
    FaultSeverity system_severity = FaultSeverity::INFO;
    RecommendedAction system_action = RecommendedAction::CONTINUE;
    
    // Requires replan
    bool requires_replan = false;
    std::string replan_reason;
    
    // Formation-aware
    bool is_leader = false;
    float formation_risk_score = 0.0f;  // 0-1
    
    // Escalation info
    std::map<FaultType, float> time_since_onset;
    std::vector<std::string> escalation_warnings;
    
    // Summary
    std::string summary;
};

class SeverityAssessment {
public:
    SeverityAssessment() = default;
    ~SeverityAssessment() = default;
    
    void configure(const SeverityAssessmentConfig& config) { config_ = config; }
    
    // Assess severity for multiple faults
    SeverityResult assess(const std::vector<FaultClassifier::FaultDetection>& detections,
                          const HealthMonitor::SystemHealth& health,
                          bool is_leader,
                          const Eigen::Vector3f& position_enu = Eigen::Vector3f::Zero(),
                          const std::vector<uint16_t>& formation_members = {}) const;
    
    // Assess single fault
    FaultSeverity assessSingle(const FaultClassifier::FaultDetection& detection,
                               const HealthMonitor::SystemHealth& health,
                               bool is_leader) const;
    
    // Get recommended action for severity
    RecommendedAction getActionForSeverity(FaultSeverity severity) const;
    
    // Check if replan is needed
    bool needsReplan(const SeverityResult& result) const;
    
    // Get formation risk
    float getFormationRisk(const SeverityResult& result, 
                           const std::vector<uint16_t>& formation_members) const;
    
    // Get config
    const SeverityAssessmentConfig& getConfig() const { return config_; }

private:
    SeverityAssessmentConfig config_;
    
    // Compute fault severity based on confidence, degradation, health
    FaultSeverity computeFaultSeverity(const FaultClassifier::FaultDetection& detection,
                                       const HealthMonitor::SystemHealth& health,
                                       bool is_leader) const;
    
    // Escalate based on multiple faults
    FaultSeverity escalateMultiFault(const std::map<FaultType, FaultSeverity>& severities) const;
    
    // Escalate based on time
    FaultSeverity escalateTime(FaultSeverity base, float time_since_onset) const;
    
    // Compute system severity from fault severities
    FaultSeverity computeSystemSeverity(const std::map<FaultType, FaultSeverity>& fault_severities,
                                        const HealthMonitor::SystemHealth& health) const;
    
    // Generate summary string
    std::string generateSummary(const SeverityResult& result) const;
};

}  // namespace swarm::ai

#endif  // SEVERITY_ASSESSMENT_HPP