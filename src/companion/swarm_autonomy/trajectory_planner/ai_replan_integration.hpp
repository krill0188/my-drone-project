#ifndef AI_REPLAN_INTEGRATION_HPP
#define AI_REPLAN_INTEGRATION_HPP

#include "../../common/ai_inference.hpp"
#include "../../trajectory_planner/hybrid_planner.hpp"
#include "../../formation/formation_geometry.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <functional>
#include <mutex>
#include <deque>

namespace swarm::autonomy {

struct AIReplanConfig {
    // Replan timing
    float max_replan_latency_ms = 100.0f;  // FR-010: < 100ms
    int replan_rate_hz = 20;
    
    // Trigger thresholds
    float obstacle_threat_threshold = 0.7f;      // Threat level HIGH/CRITICAL
    float landing_suitability_threshold = 0.7f;  // SUITABLE/IDEAL
    float fault_severity_threshold = 2;          // CRITICAL (2) or EMERGENCY (3)
    
    // Replan modes
    enum class ReplanMode {
        IMMEDIATE,      // Emergency - stop and replan now
        NEXT_CYCLE,     // Next planning cycle
        FORMATION_WIDE  // Coordinate with formation
    };
    
    // Formation coordination
    bool coordinate_formation = true;
    float formation_replan_delay_ms = 50.0f;  // Delay for follower sync
    
    // Safety
    float min_obstacle_distance_m = 10.0f;   // Trigger distance
    float emergency_stop_distance_m = 5.0f;  // Immediate stop
    bool enable_emergency_stop = true;
    
    // Fallback
    bool use_conservative_fallback = true;   // Geometric avoidance if AI fails
    float conservative_margin_m = 5.0f;
};

struct ReplanTrigger {
    uint64_t timestamp_us;
    ReplanTriggerType type;
    ReplanPriority priority;
    
    // Source inference
    AIInferenceResult inference;
    
    // Context
    Eigen::Vector3f current_position_enu;
    Eigen::Vector3f current_velocity_enu;
    Eigen::Vector3f target_position_enu;
    std::string formation_role;  // "leader" or "follower"
    uint16_t leader_id = 0;
    
    // Requested changes
    Eigen::Vector3f avoidance_vector;     // For obstacles
    Eigen::Vector3f landing_offset_enu;   // For landing
    Eigen::Vector3f formation_offset_enu; // For formation adjustment
    
    bool requires_formation_sync = false;
    std::vector<uint16_t> affected_drones;
};

enum class ReplanTriggerType {
    OBSTACLE_AVOIDANCE = 0,
    LANDING_ADJUSTMENT = 1,
    FAULT_RESPONSE = 2,
    ANOMALY_RESPONSE = 3,
    FORMATION_REJOIN = 4,
    MISSION_UPDATE = 5
};

enum class ReplanPriority {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    EMERGENCY = 3
};

struct ReplanResult {
    uint64_t timestamp_us;
    bool success = false;
    float total_latency_ms = 0.0f;
    
    // New trajectory
    std::vector<Eigen::Vector3f> new_waypoints_enu;
    std::vector<float> waypoint_timestamps_s;
    float trajectory_duration_s = 0.0f;
    
    // Formation
    bool formation_sync_required = false;
    Eigen::Vector3f new_formation_offset_enu;
    std::vector<uint16_t> affected_followers;
    
    // Emergency
    bool emergency_stop = false;
    Eigen::Vector3f emergency_hover_position_enu;
    
    // Diagnostics
    std::string trigger_reason;
    ReplanTriggerType trigger_type;
    int replan_attempts = 0;
    std::string fallback_reason;
};

class AIReplanIntegration {
public:
    AIReplanIntegration();
    ~AIReplanIntegration();
    
    AIReplanIntegration(const AIReplanIntegration&) = delete;
    AIReplanIntegration& operator=(const AIReplanIntegration&) = delete;
    AIReplanIntegration(AIReplanIntegration&&) noexcept;
    AIReplanIntegration& operator=(AIReplanIntegration&&) noexcept;
    
    // Initialize
    bool initialize(const AIReplanConfig& config, 
                    HybridPlanner* planner,
                    FormationGeometryManager* formation_mgr);
    
    // Process AI inference and trigger replan if needed
    void processInference(const AIInferenceResult& inference,
                          const Eigen::Vector3f& position_enu,
                          const Eigen::Vector3f& velocity_enu,
                          const Eigen::Vector3f& target_enu,
                          const std::string& formation_role,
                          uint16_t leader_id = 0);
    
    // Process formation state for follower replan
    void processFormationState(const FormationState& formation_state,
                               uint16_t drone_id);
    
    // Get pending replan result
    std::optional<ReplanResult> getReplanResult();
    
    // Check if replan is in progress
    bool isReplanning() const;
    
    // Emergency stop
    void triggerEmergencyStop(const Eigen::Vector3f& position_enu,
                              const std::string& reason);
    
    // Clear triggers
    void clearTriggers();
    
    // Get config
    const AIReplanConfig& getConfig() const { return config_; }
    
    // Get stats
    struct Stats {
        uint64_t total_triggers = 0;
        uint64_t successful_replans = 0;
        uint64_t failed_replans = 0;
        uint64_t emergency_stops = 0;
        float avg_replan_latency_ms = 0.0f;
        float max_replan_latency_ms = 0.0f;
        std::map<ReplanTriggerType, uint64_t> triggers_by_type;
    };
    const Stats& getStats() const { return stats_; }
    
    // Set callbacks
    using ReplanCallback = std::function<void(const ReplanResult&)>;
    void setReplanCallback(ReplanCallback cb) { replan_callback_ = cb; }
    
    using EmergencyCallback = std::function<void(const Eigen::Vector3f&, const std::string&)>;
    void setEmergencyCallback(EmergencyCallback cb) { emergency_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    AIReplanConfig config_;
    HybridPlanner* planner_ = nullptr;
    FormationGeometryManager* formation_mgr_ = nullptr;
    bool initialized_ = false;
    
    Stats stats_;
    std::mutex stats_mutex_;
    
    std::function<void(const ReplanResult&)> replan_callback_;
    std::function<void(const Eigen::Vector3f&, const std::string&)> emergency_callback_;
};

}  // namespace swarm::autonomy

#endif  // AI_REPLAN_INTEGRATION_HPP