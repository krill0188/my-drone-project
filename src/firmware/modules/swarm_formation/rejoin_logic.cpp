#include "rejoin_logic.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <algorithm>
#include <cmath>

namespace swarm::formation {

RejoinLogic::RejoinLogic(FormationController& controller, MeshManager& mesh, const Config& config)
    : _controller(controller), _mesh(mesh), _config(config) {}

void RejoinLogic::onLocalizationLost(uint16_t follower_id) {
    auto& ctx = _contexts[follower_id];
    ctx.follower_id = follower_id;
    ctx.state = RejoinState::LOCALIZATION_LOST;
    ctx.loss_start_us = Timestamp::nowUs();
    ctx.state_start_us = ctx.loss_start_us;
    
    // Save last known leader state
    auto leader_state = _controller.getLeaderState();
    ctx.last_known_leader_pos = leader_state.position;
    ctx.last_known_leader_vel = leader_state.velocity;
    
    // Get target offset from formation
    auto slot = _controller.getFormationSlot(follower_id);
    if (slot) {
        ctx.target_offset_flu = slot->offset_flu;
    }
}

void RejoinLogic::onLocalizationRestored(uint16_t follower_id) {
    auto it = _contexts.find(follower_id);
    if (it == _contexts.end()) return;
    
    auto& ctx = it->second;
    
    if (ctx.state == RejoinState::LOCALIZATION_LOST || ctx.state == RejoinState::PREDICTING) {
        // Transition to rejoin
        ctx.state = RejoinState::REJOINING;
        ctx.rejoin_start_us = Timestamp::nowUs();
        ctx.state_start_us = ctx.rejoin_start_us;
    } else if (ctx.state == RejoinState::REJOINING) {
        // Already rejoining, just continue
    }
}

bool RejoinLogic::update(uint16_t follower_id, float dt) {
    auto it = _contexts.find(follower_id);
    if (it == _contexts.end()) return false;
    
    auto& ctx = it->second;
    
    switch (ctx.state) {
        case RejoinState::LOCALIZATION_LOST:
            return handleLocalizationLost(ctx, dt);
        case RejoinState::PREDICTING:
            return handlePredicting(ctx, dt);
        case RejoinState::REJOINING:
            return handleRejoining(ctx, dt);
        case RejoinState::CAPTURED:
            return handleCaptured(ctx, dt);
        case RejoinState::STABILIZING:
            return handleStabilizing(ctx, dt);
        default:
            return false;
    }
}

bool RejoinLogic::handleLocalizationLost(RejoinContext& ctx, float dt) {
    // Check if prediction should start
    if (_config.use_trajectory_prediction) {
        float elapsed = getStateElapsedS(ctx);
        
        if (elapsed > 0.5f) {  // Start predicting after 0.5s
            ctx.state = RejoinState::PREDICTING;
            ctx.state_start_us = Timestamp::nowUs();
            ctx.trajectory_predicted = false;
        }
    }
    
    // Check timeout
    if (checkRejoinTimeout(ctx)) {
        ctx.state = RejoinState::FAILED;
        return false;
    }
    
    return false;
}

bool RejoinLogic::handlePredicting(RejoinContext& ctx, float dt) {
    // Predict leader trajectory
    if (!ctx.trajectory_predicted) {
        float prediction_time = std::min(getStateElapsedS(ctx), _config.max_prediction_time_s);
        ctx.predicted_leader_pos = predictLeaderPosition(ctx.last_known_leader_pos, 
                                                         ctx.last_known_leader_vel, 
                                                         prediction_time);
        ctx.trajectory_predicted = true;
    }
    
    // Check if localization restored (external trigger)
    // If restored, will transition to REJOINING via onLocalizationRestored
    
    // Check prediction timeout
    if (getStateElapsedS(ctx) > _config.max_prediction_time_s) {
        // Prediction timeout, transition to rejoin with last prediction
        ctx.state = RejoinState::REJOINING;
        ctx.rejoin_start_us = Timestamp::nowUs();
        ctx.state_start_us = ctx.rejoin_start_us;
    }
    
    return false;
}

bool RejoinLogic::handleRejoining(RejoinContext& ctx, float dt) {
    // Get current leader position (restored or predicted)
    Vector3 leader_pos;
    Vector3 leader_vel = ctx.last_known_leader_vel;
    
    // Get current follower position (from formation controller)
    // In real implementation, would get from follower's state estimator
    ctx.current_position_enu = ctx.last_known_leader_pos;  // Simplified
    
    // Compute rejoin velocity
    Vector3 vel_cmd = computeRejoinVelocity(ctx, ctx.predicted_leader_pos, leader_vel);
    
    // Send velocity command to follower
    // In real implementation, would send to follower's position controller
    
    // Check capture
    if (checkCapture(ctx, ctx.predicted_leader_pos)) {
        ctx.state = RejoinState::CAPTURED;
        ctx.state_start_us = Timestamp::nowUs();
    }
    
    // Check overshoot
    if (!checkOvershootLimit(ctx)) {
        // Overshoot limit exceeded
        return false;
    }
    
    // Check timeout
    if (checkRejoinTimeout(ctx)) {
        return false;
    }
    
    return false;
}

bool RejoinLogic::handleCaptured(RejoinContext& ctx, float dt) {
    // Within capture radius, now stabilize
    ctx.state = RejoinState::STABILIZING;
    ctx.state_start_us = Timestamp::nowUs();
    return false;
}

bool RejoinLogic::handleStabilizing(RejoinContext& ctx, float dt) {
    // Wait for position error to settle
    float elapsed = getStateElapsedS(ctx);
    
    if (elapsed > _config.smooth_transition_time_s) {
        // Check if position error is within tolerance
        // In real implementation, would check actual position error
        return true;  // Rejoin complete
    }
    
    return false;
}

Vector3 RejoinLogic::predictLeaderPosition(const Vector3& pos, const Vector3& vel, float dt) {
    // Simple constant velocity prediction
    // In real implementation, would use leader's trajectory planner
    Vector3 predicted = pos;
    predicted.x += vel.x * dt;
    predicted.y += vel.y * dt;
    predicted.z += vel.z * dt;
    
    // Could add acceleration if available
    return predicted;
}

Vector3 RejoinLogic::computeRejoinVelocity(const RejoinContext& ctx, 
                                            const Vector3& leader_pos, 
                                            const Vector3& leader_vel) {
    // Target position = leader position + target offset (in ENU)
    // For simplicity, assume target offset is in ENU frame
    // In real implementation, would rotate target_offset_flu by leader yaw
    
    Vector3 target_pos = leader_pos + ctx.target_offset_flu;  // Simplified
    
    // Position error
    Vector3 pos_error = target_pos - ctx.current_position_enu;
    
    // Velocity command: feedforward + proportional
    Vector3 vel_cmd;
    vel_cmd.x = leader_vel.x + ctx.target_offset_flu.x * 0.5f + pos_error.x * 1.0f;
    vel_cmd.y = leader_vel.y + ctx.target_offset_flu.y * 0.5f + pos_error.y * 1.0f;
    vel_cmd.z = leader_vel.z + ctx.target_offset_flu.z * 0.5f + pos_error.z * 1.0f;
    
    // Limit speed
    float max_speed = _config.approach_speed_mps;
    float speed = std::sqrt(vel_cmd.x*vel_cmd.x + vel_cmd.y*vel_cmd.y + vel_cmd.z*vel_cmd.z);
    if (speed > max_speed) {
        float scale = max_speed / speed;
        vel_cmd.x *= scale;
        vel_cmd.y *= scale;
        vel_cmd.z *= scale;
    }
    
    return vel_cmd;
}

bool RejoinLogic::checkCapture(const RejoinContext& ctx, const Vector3& leader_pos) {
    Vector3 target_pos = leader_pos + ctx.target_offset_flu;
    Vector3 error = target_pos - ctx.current_position_enu;
    
    float distance = std::sqrt(error.x*error.x + error.y*error.y + error.z*error.z);
    return distance < _config.capture_radius_m;
}

void RejoinLogic::advanceState(RejoinContext& ctx, RejoinState new_state) {
    ctx.state = new_state;
    ctx.state_start_us = Timestamp::nowUs();
}

float RejoinLogic::getStateElapsedS(const RejoinContext& ctx) const {
    return static_cast<float>(Timestamp::nowUs() - ctx.state_start_us) / 1e6f;
}

bool RejoinLogic::checkOvershootLimit(RejoinContext& ctx) {
    // Track maximum overshoot
    // In real implementation, would track max distance beyond target during rejoin
    // For now, simplified check
    
    // Get current position error
    Vector3 target_pos = ctx.predicted_leader_pos + ctx.target_offset_flu;
    Vector3 error = target_pos - ctx.current_position_enu;
    float distance = std::sqrt(error.x*error.x + error.y*error.y + error.z*error.z);
    
    ctx.max_overshoot_observed = std::max(ctx.max_overshoot_observed, distance);
    
    if (ctx.max_overshoot_observed > _config.max_overshoot_m) {
        return false;  // Overshoot limit exceeded
    }
    
    return true;
}

bool RejoinLogic::checkRejoinTimeout(RejoinContext& ctx) {
    float total_elapsed = static_cast<float>(Timestamp::nowUs() - ctx.loss_start_us) / 1e6f;
    
    if (total_elapsed > _config.max_rejoin_time_s + 2.0f) {  // +2s margin
        return true;
    }
    return false;
}

RejoinLogic::RejoinState RejoinLogic::getState(uint16_t follower_id) const {
    auto it = _contexts.find(follower_id);
    if (it != _contexts.end()) {
        return it->second.state;
    }
    return RejoinState::IDLE;
}

float RejoinLogic::getMaxOvershoot(uint16_t follower_id) const {
    auto it = _contexts.find(follower_id);
    if (it != _contexts.end()) {
        return it->second.max_overshoot_observed;
    }
    return 0.0f;
}

bool RejoinLogic::isRejoinComplete(uint16_t follower_id) const {
    auto it = _contexts.find(follower_id);
    if (it != _contexts.end()) {
        return it->second.state == RejoinState::STABILIZING || 
               it->second.state == RejoinState::COMPLETE;
    }
    return false;
}

void RejoinLogic::abortRejoin(uint16_t follower_id, const std::string& reason) {
    auto it = _contexts.find(follower_id);
    if (it != _contexts.end()) {
        it->second.state = RejoinState::FAILED;
    }
}

} // namespace swarm::formation