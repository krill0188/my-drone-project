#include "trajectory_planner.hpp"
#include <algorithm>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>

namespace swarm::planner {

// ============================================================================
// MPCPlanner Implementation
// ============================================================================

MPCPlanner::MPCPlanner(const PlannerConfig& config) : _config(config) {
    _reference_path.clear();
    _control_inputs.resize(config.mpc_horizon);
    _predicted_states.resize(config.mpc_horizon + 1);
}

void MPCPlanner::setReferencePath(const std::vector<Vector3>& path) {
    _reference_path = path;
}

Trajectory MPCPlanner::optimize(const Vector3& current_pos, const Vector3& current_vel,
                                const std::vector<Vector3>& reference_path,
                                const std::vector<Obstacle>& obstacles) {
    _reference_path = reference_path;
    
    Trajectory trajectory;
    trajectory.planner_used = "MPC";
    
    if (_reference_path.empty()) {
        trajectory.valid = false;
        return trajectory;
    }
    
    // Solve MPC optimization
    solveMPC(current_pos, current_vel, obstacles);
    
    // Build trajectory from predicted states
    trajectory.points.resize(_config.mpc_horizon + 1);
    trajectory.points[0] = {current_pos, current_vel, {0,0,0}, 0.0f, 0};
    
    for (int i = 1; i <= _config.mpc_horizon; ++i) {
        trajectory.points[i].position = _predicted_states[i];
        trajectory.points[i].velocity = _control_inputs[i-1];
        trajectory.points[i].acceleration = {0,0,0};
        trajectory.points[i].timestamp_us = static_cast<uint64_t>(i * _config.mpc_dt * 1e6);
    }
    
    trajectory.total_time = _config.mpc_horizon * _config.mpc_dt;
    trajectory.valid = true;
    
    return trajectory;
}

void MPCPlanner::solveMPC(const Vector3& current_pos, const Vector3& current_vel,
                          const std::vector<Obstacle>& obstacles) {
    // Simplified MPC implementation
    // In real implementation, would use OSQP, qpOASES, or similar QP solver
    // Here we use a simple gradient descent approach for demonstration
    
    // Initialize control inputs to zero
    std::fill(_control_inputs.begin(), _control_inputs.end(), Vector3{0,0,0});
    _predicted_states[0] = current_pos;
    
    // Simple gradient descent iterations
    const int max_iter = 50;
    const float learning_rate = 0.1f;
    
    for (int iter = 0; iter < max_iter; ++iter) {
        // Forward simulation
        _predicted_states[0] = current_pos;
        Vector3 vel = current_vel;
        
        for (int i = 0; i < _config.mpc_horizon; ++i) {
            vel = vel + _control_inputs[i] * _config.mpc_dt;
            _predicted_states[i+1] = _predicted_states[i] + vel * _config.mpc_dt;
        }
        
        // Compute gradients
        std::vector<Vector3> control_gradients(_config.mpc_horizon, {0,0,0});
        
        // Path following gradient
        for (int i = 0; i < _config.mpc_horizon; ++i) {
            Vector3 ref_point = getReferencePoint(i);
            Vector3 error = _predicted_states[i+1] - ref_point;
            
            // Gradient w.r.t control inputs (simplified chain rule)
            for (int j = 0; j <= i; ++j) {
                float dt_pow = std::pow(_config.mpc_dt, i - j + 1);
                control_gradients[j] = control_gradients[j] + error * dt_pow;
            }
        }
        
        // Obstacle avoidance gradient
        for (const auto& obs : obstacles) {
            for (int i = 0; i < _config.mpc_horizon; ++i) {
                Vector3 to_obs = _predicted_states[i+1] - obs.center;
                float dist = std::sqrt(to_obs.x*to_obs.x + to_obs.y*to_obs.y + to_obs.z*to_obs.z);
                
                float safety_dist = 0.0f;
                if (obs.type == Obstacle::SPHERE) safety_dist = obs.size.y;
                else if (obs.type == Obstacle::BOX) safety_dist = std::max({obs.size.x, obs.size.y, obs.size.z});
                else safety_dist = obs.size.y;
                
                safety_dist += _config.safety_margin;
                
                if (dist < safety_dist) {
                    // Repulsive gradient
                    Vector3 repulsive = to_obs * (safety_dist - dist) / (dist + 1e-6);
                    for (int j = 0; j <= i; ++j) {
                        float dt_pow = std::pow(_config.mpc_dt, i - j + 1);
                        control_gradients[j] = control_gradients[j] - repulsive * dt_pow * 0.1f;
                    }
                }
            }
        }
        
        // Control effort regularization
        for (int i = 0; i < _config.mpc_horizon; ++i) {
            control_gradients[i] = control_gradients[i] + _control_inputs[i] * 0.01f;
        }
        
        // Update controls
        for (int i = 0; i < _config.mpc_horizon; ++i) {
            _control_inputs[i] = _control_inputs[i] - control_gradients[i] * learning_rate;
            
            // Clamp to max acceleration
            float max_acc = _config.max_acceleration * _config.mpc_dt;
            float acc_mag = std::sqrt(_control_inputs[i].x*_control_inputs[i].x + 
                                     _control_inputs[i].y*_control_inputs[i].y + 
                                     _control_inputs[i].z*_control_inputs[i].z);
            if (acc_mag > max_acc) {
                float scale = max_acc / (acc_mag + 1e-6);
                _control_inputs[i] = _control_inputs[i] * scale;
            }
        }
    }
}

Vector3 MPCPlanner::getReferencePoint(int step) {
    if (_reference_path.empty()) return {0,0,0};
    
    // Map MPC step to reference path index
    int path_idx = std::min(step * 2, static_cast<int>(_reference_path.size()) - 1);
    return _reference_path[path_idx];
}

std::vector<Vector3> MPCPlanner::getControlInputs() const {
    return _control_inputs;
}

} // namespace swarm::planner