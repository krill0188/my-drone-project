#ifndef FAULT_REPLAN_HPP
#define FAULT_REPLAN_HPP

#include "../../../common/ai_inference.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <cstdint>

namespace swarm::formation {

struct FaultReplanConfig {
    // Follower offset adjustments per fault type
    struct FaultOffset {
        Eigen::Vector3f position_offset_flu;  // FLU frame offset
        Eigen::Vector3f velocity_limit;       // Max velocity during fault
        float altitude_adjustment_m = 0.0f;   // Altitude change
        bool hold_position = false;
    };
    
    std::map<uint8_t, FaultOffset> fault_offsets;  // Key = FaultType
    
    // Formation reconfiguration
    bool enable_formation_compact = true;
    float compact_distance_factor = 0.8f;  // Reduce spacing by 20%
    float max_compact_distance_m = 2.0f;   // Minimum spacing
    
    // Leader fault handling
    bool leader_fault_triggers_election = true;
    float leader_fault_timeout_s = 2.0f;
    
    // Follower fault handling
    bool follower_fault_isolates = true;
    float isolation_distance_m = 10.0f;
    
    // Recovery
    float recovery_check_interval_s = 5.0f;
    float recovery_confidence_threshold = 0.9f;
};

struct FollowerState {
    uint16_t sysid;
    Eigen::Vector3f position_flu;       // Relative to leader
    Eigen::Vector3f velocity_flu;
    Eigen::Vector3f desired_offset_flu; // Desired formation offset
    Eigen::Vector3f current_offset_flu; // Actual offset (with fault adjustment)
    bool has_fault = false;
    uint8_t fault_type = 0;
    float fault_degradation = 0.0f;
    uint64_t fault_start_us = 0;
};

struct FaultReplanResult {
    uint64_t timestamp_us;
    bool success = false;
    bool formation_changed = false;
    
    // Updated follower states
    std::vector<FollowerState> followers;
    
    // Leader commands
    Eigen::Vector3f leader_velocity_cmd_flu;
    bool leader_hold = false;
    
    // New formation geometry
    uint8_t new_geometry_type = 0;  // FormationGeometryType
    float new_spacing_m = 0.0f;
    
    // Diagnostics
    std::string replan_reason;
    std::vector<std::string> warnings;
};

class FaultReplan {
public:
    FaultReplan() = default;
    ~FaultReplan() = default;
    
    void configure(const FaultReplanConfig& config) { config_ = config; }
    
    // Process fault detection and update formation
    FaultReplanResult processFault(const AIInferenceResult& fault_inference,
                                   const std::vector<FollowerState>& current_followers,
                                   uint16_t leader_id,
                                   const Eigen::Vector3f& leader_vel_flu,
                                   uint64_t timestamp_us);
    
    // Check for fault recovery
    FaultReplanResult checkRecovery(const std::vector<FollowerState>& current_followers,
                                     uint64_t timestamp_us);
    
    // Handle leader fault
    FaultReplanResult handleLeaderFault(uint8_t fault_type, float degradation,
                                         const std::vector<FollowerState>& followers,
                                         uint64_t timestamp_us);
    
    // Get current config
    const FaultReplanConfig& getConfig() const { return config_; }
    
    // Update follower state
    void updateFollowerState(uint16_t sysid, const Eigen::Vector3f& pos_flu,
                             const Eigen::Vector3f& vel_flu,
                             const Eigen::Vector3f& desired_offset_flu);

private:
    FaultReplanConfig config_;
    std::map<uint16_t, FollowerState> follower_states_;
    
    // Compute fault offset for follower
    Eigen::Vector3f computeFaultOffset(const FollowerState& follower) const;
    
    // Compact formation
    std::vector<FollowerState> compactFormation(const std::vector<FollowerState>& followers) const;
    
    // Isolate faulty follower
    FollowerState isolateFollower(const FollowerState& follower) const;
    
    // Elect new leader
    uint16_t electNewLeader(const std::vector<FollowerState>& followers) const;
};

}  // namespace swarm::formation

#endif  // FAULT_REPLAN_HPP