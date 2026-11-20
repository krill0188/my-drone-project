#ifndef REJOIN_LOGIC_HPP
#define REJOIN_LOGIC_HPP

#include "swarm_formation.hpp"
#include <vector>
#include <array>
#include <functional>

namespace swarm::formation {

// Follower rejoin logic after temporary localization loss
class RejoinLogic {
public:
    struct Config {
        float max_rejoin_time_s = 3.0f;          // Max time to rejoin (FR-003)
        float max_overshoot_m = 1.0f;            // Max overshoot during rejoin (spec edge case 3)
        float approach_speed_mps = 2.0f;         // Rejoin approach speed
        float capture_radius_m = 0.5f;           // Consider rejoined when within this distance
        float smooth_transition_time_s = 1.0f;   // Smooth transition time
        bool use_trajectory_prediction = true;   // Predict leader trajectory during loss
        float max_prediction_time_s = 5.0f;      // Max time to predict
    };
    
    enum class RejoinState {
        IDLE,
        LOCALIZATION_LOST,
        PREDICTING,
        REJOINING,
        CAPTURED,
        STABILIZING,
        COMPLETE,
        FAILED
    };
    
    struct RejoinContext {
        uint16_t follower_id;
        RejoinState state = RejoinState::IDLE;
        uint64_t loss_start_us = 0;
        uint64_t rejoin_start_us = 0;
        Vector3 last_known_leader_pos;
        Vector3 last_known_leader_vel;
        Vector3 predicted_leader_pos;
        Vector3 target_offset_flu;
        Vector3 current_position_enu;
        Vector3 current_velocity_enu;
        float max_overshoot_observed = 0.0f;
        bool trajectory_predicted = false;
        uint64_t state_start_us = 0;
    };
    
    RejoinLogic(FormationController& controller, MeshManager& mesh, const Config& config = Config());
    
    // Call when localization is lost
    void onLocalizationLost(uint16_t follower_id);
    
    // Call when localization is restored
    void onLocalizationRestored(uint16_t follower_id);
    
    // Update function (call at 50 Hz)
    bool update(uint16_t follower_id, float dt);
    
    // Get current rejoin state
    RejoinState getState(uint16_t follower_id) const;
    
    // Get max overshoot observed
    float getMaxOvershoot(uint16_t follower_id) const;
    
    // Check if rejoin complete
    bool isRejoinComplete(uint16_t follower_id) const;
    
    // Force abort rejoin
    void abortRejoin(uint16_t follower_id, const std::string& reason);

private:
    FormationController& _controller;
    MeshManager& _mesh;
    Config _config;
    
    std::unordered_map<uint16_t, RejoinContext> _contexts;
    
    // State machine functions
    bool handleIdle(RejoinContext& ctx, float dt);
    bool handleLocalizationLost(RejoinContext& ctx, float dt);
    bool handlePredicting(RejoinContext& ctx, float dt);
    bool handleRejoining(RejoinContext& ctx, float dt);
    bool handleCaptured(RejoinContext& ctx, float dt);
    bool handleStabilizing(RejoinContext& ctx, float dt);
    
    // Helper functions
    Vector3 predictLeaderPosition(const Vector3& pos, const Vector3& vel, float dt);
    Vector3 computeRejoinVelocity(const RejoinContext& ctx, const Vector3& leader_pos, const Vector3& leader_vel);
    bool checkCapture(const RejoinContext& ctx, const Vector3& leader_pos);
    void advanceState(RejoinContext& ctx, RejoinState new_state);
    float getStateElapsedS(const RejoinContext& ctx) const;
    
    // Safety
    bool checkOvershootLimit(RejoinContext& ctx);
    bool checkRejoinTimeout(RejoinContext& ctx);
};

} // namespace swarm::formation

#endif // REJOIN_LOGIC_HPP