#ifndef FOLLOWER_REPLAN_HPP
#define FOLLOWER_REPLAN_HPP

#include "swarm_formation.hpp"
#include <vector>
#include <array>

namespace swarm::formation {

// Follower replan handler: adjusts formation position when leader replans
class FollowerReplanHandler {
public:
    FollowerReplanHandler(FormationController& controller) : _controller(controller) {}
    
    // Called when leader broadcasts new trajectory
    void onLeaderReplan(const planner::Trajectory& new_trajectory);
    
    // Called when leader position changes abruptly (obstacle avoidance)
    void onLeaderEmergencyManeuver(const Vector3& new_leader_vel, const Vector3& new_leader_accel);
    
    // Adjust formation position to follow leader's new path
    bool adjustFormationPosition(float dt);
    
    // Check if formation is stable after replan
    bool isFormationStable() const;
    
    // Get max position error across all followers
    float getMaxPositionError() const;
    
    // Get average position error
    float getAvgPositionError() const;

private:
    FormationController& _controller;
    
    // Replan state
    bool _replan_active = false;
    planner::Trajectory _leader_new_trajectory;
    uint64_t _replan_start_us = 0;
    float _max_adjustment_time_s = 2.0f;  // 2 second bound per FR-003
    
    // Position error tracking
    struct FollowerError {
        uint16_t follower_id;
        Vector3 position_error_flu;
        Vector3 velocity_error_flu;
        float max_lateral_error = 0.0f;
        float max_vertical_error = 0.0f;
        uint64_t last_update_us = 0;
    };
    std::vector<FollowerError> _follower_errors;
    
    // Compute required velocity adjustment for each follower
    Vector3 computeFollowerVelocityAdjustment(const FormationSlot& slot,
                                               const Vector3& leader_pos,
                                               const Vector3& leader_vel,
                                               const Vector3& target_pos);
    
    // Formation keeping during transition
    Vector3 computeTransitionVelocity(const FormationSlot& slot,
                                       const Vector3& current_offset_flu,
                                       const Vector3& target_offset_flu,
                                       float dt);
    
    // Collision avoidance during formation adjustment
    Vector3 applyCollisionAvoidance(uint16_t follower_id, const Vector3& vel_cmd);
};

} // namespace swarm::formation

#endif // FOLLOWER_REPLAN_HPP