#include "mpc_avoidance.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace swarm::autonomy {

MPCAvoidance::MPCAvoidance(const Config& config) : _config(config) {
    _last_solution.feasible = false;
}

MPCAvoidance::MPCSolution MPCAvoidance::solve(const AgentState& ego,
                                               const std::vector<Neighbor>& neighbors,
                                               const std::vector<Obstacle>& obstacles,
                                               const Vector3& goal_position,
                                               const Vector3& goal_velocity) {
    // Generate reference trajectory (straight line to goal)
    std::vector<Vector3> ref_positions(_config.horizon);
    std::vector<Vector3> ref_velocities(_config.horizon);
    
    Vector3 start = ego.position;
    Vector3 dir = goal_position - start;
    float dist = std::sqrt(dir.x*dir.x + dir.y*dir.y + dir.z*dir.z);
    
    if (dist > 0.001f) {
        dir = dir * (1.0f / dist);
    } else {
        dir = {0, 0, 0};
    }
    
    float cruise_speed = std::min(_config.max_speed, dist / (_config.horizon * _config.dt));
    if (cruise_speed > _config.max_speed) cruise_speed = _config.max_speed;
    
    for (int i = 0; i < _config.horizon; ++i) {
        ref_positions[i] = start + dir * (cruise_speed * _config.dt * i);
        ref_velocities[i] = dir * cruise_speed;
    }
    
    return solveWithReference(ego, ref_positions, ref_velocities, neighbors, obstacles);
}

MPCAvoidance::MPCSolution MPCAvoidance::solveWithReference(
    const AgentState& ego,
    const std::vector<Vector3>& ref_positions,
    const std::vector<Vector3>& ref_velocities,
    const std::vector<Neighbor>& neighbors,
    const std::vector<Obstacle>& obstacles) {
    
    // Build QP
    buildQP(ego, ref_positions, ref_velocities, neighbors, obstacles);
    
    // Solve QP
    Eigen::VectorXd solution;
    bool success = solveQP(solution);
    
    MPCSolution solution_obj;
    if (success) {
        solution_obj = extractSolution(solution, ego);
    } else {
        // Emergency fallback: stop
        solution_obj.feasible = false;
        solution_obj.cost = std::numeric_limits<float>::infinity();
        solution_obj.positions.resize(_config.horizon, ego.position);
        solution_obj.velocities.resize(_config.horizon, {0, 0, 0});
        solution_obj.accelerations.resize(_config.horizon, {0, 0, 0});
    }
    
    _last_solution = solution_obj;
    return solution_obj;
}

MPCAvoidance::MPCSolution MPCAvoidance::emergencyStop(const AgentState& ego,
                                                       const std::vector<Neighbor>& neighbors,
                                                       const std::vector<Obstacle>& obstacles) {
    // Zero velocity reference
    std::vector<Vector3> ref_positions(_config.horizon, ego.position);
    std::vector<Vector3> ref_velocities(_config.horizon, {0, 0, 0});
    
    // High collision weight
    float old_w_collision = _config.w_collision;
    _config.w_collision = 1000.0f;
    
    MPCSolution sol = solveWithReference(ego, ref_positions, ref_velocities, neighbors, obstacles);
    
    _config.w_collision = old_w_collision;
    return sol;
}

void MPCAvoidance::updateObstacles(const std::vector<Obstacle>& obstacles) {
    // In full implementation, would update obstacle predictions
}

void MPCAvoidance::updateNeighbors(const std::vector<Neighbor>& neighbors) {
    // In full implementation, would update neighbor predictions
}

const MPCAvoidance::MPCSolution& MPCAvoidance::getLastSolution() const {
    return _last_solution;
}

void MPCAvoidance::setConfig(const Config& config) {
    _config = config;
}

const MPCAvoidance::Config& MPCAvoidance::getConfig() const {
    return _config;
}

void MPCAvoidance::buildQP(const AgentState& ego,
                           const std::vector<Vector3>& ref_positions,
                           const std::vector<Vector3>& ref_velocities,
                           const std::vector<Neighbor>& neighbors,
                           const std::vector<Obstacle>& obstacles) {
    // Problem dimensions
    // Variables per timestep: 3 pos + 3 vel + 3 acc = 9
    // Total variables: 9 * horizon
    int n_vars = 9 * _config.horizon;
    
    // Resize matrices
    _H.resize(n_vars, n_vars);
    _g.resize(n_vars);
    _H.setZero();
    _g.setZero();
    
    buildCost(ego, ref_positions, ref_velocities);
    buildCollisionConstraints(ego, neighbors, obstacles);
    buildDynamicsConstraints(ego);
    buildBounds(ego);
}

void MPCAvoidance::buildCost(const AgentState& ego,
                             const std::vector<Vector3>& ref_positions,
                             const std::vector<Vector3>& ref_velocities) {
    int n = _config.horizon;
    int n_vars = 9 * n;
    
    // Cost function: sum of weighted errors
    // x = [p_1, v_1, a_1, p_2, v_2, a_2, ...]
    // p_i = position at step i
    // v_i = velocity at step i
    // a_i = acceleration at step i
    
    for (int k = 0; k < _config.horizon; ++k) {
        int base = 9 * k;
        
        // Position tracking cost
        _H.coeffRef(base + 0, base + 0) += _config.w_position;
        _H.coeffRef(base + 1, base + 1) += _config.w_position;
        _H.coeffRef(base + 2, base + 2) += _config.w_position;
        
        // Velocity tracking cost
        _H.coeffRef(base + 3, base + 3) += _config.w_velocity;
        _H.coeffRef(base + 4, base + 4) += _config.w_velocity;
        _H.coeffRef(base + 5, base + 5) += _config.w_velocity;
        
        // Acceleration minimization
        _H.coeffRef(base + 6, base + 6) += _config.w_acceleration;
        _H.coeffRef(base + 7, base + 7) += _config.w_acceleration;
        _H.coeffRef(base + 8, base + 8) += _config.w_acceleration;
        
        // Linear term for reference tracking
        _g[base + 0] = -_config.w_position * ref_positions[k].x;
        _g[base + 1] = -_config.w_position * ref_positions[k].y;
        _g[base + 2] = -_config.w_position * ref_positions[k].z;
        
        _g[base + 3] = -_config.w_velocity * ref_velocities[k].x;
        _g[base + 4] = -_config.w_velocity * ref_velocities[k].y;
        _g[base + 5] = -_config.w_velocity * ref_velocities[k].z;
    }
}

void MPCAvoidance::buildCollisionConstraints(const AgentState& ego,
                                             const std::vector<Neighbor>& neighbors,
                                             const std::vector<Obstacle>& obstacles) {
    // In full implementation, would add inequality constraints
    // For each neighbor/obstacle at each time step:
    // ||p_k - p_neighbor_k|| >= safety_distance
    // ||p_k - obst_position|| >= obst_radius + safety_margin
    
    // For now, using soft constraints via barrier functions in cost
}

void MPCAvoidance::buildDynamicsConstraints(const AgentState& ego) {
    int n = _config.horizon;
    float dt = _config.dt;
    
    // Dynamics: x_{k+1} = x_k + v_k * dt + 0.5 * a_k * dt^2
    //           v_{k+1} = v_k + a_k * dt
    
    // These would be equality constraints in the QP
    // A_eq * x = b_eq
}

void MPCAvoidance::buildBounds(const AgentState& ego) {
    int n_vars = 9 * _config.horizon;
    
    _lb.resize(n_vars);
    _ub.resize(n_vars);
    
    for (int k = 0; k < _config.horizon; ++k) {
        int base = 9 * k;
        
        // Position bounds (large)
        for (int i = 0; i < 3; ++i) {
            _lb[base + i] = -1000.0;
            _ub[base + i] = 1000.0;
        }
        
        // Velocity bounds
        for (int i = 0; i < 3; ++i) {
            _lb[base + 3 + i] = -_config.max_speed;
            _ub[base + 3 + i] = _config.max_speed;
        }
        
        // Acceleration bounds
        for (int i = 0; i < 3; ++i) {
            _lb[base + 6 + i] = -_config.max_accel;
            _ub[base + 6 + i] = _config.max_accel;
        }
    }
}

bool MPCAvoidance::solveQP(Eigen::VectorXd& solution) {
    // In full implementation, would use OSQP, qpOASES, or similar
    // For now, return a simple feasible solution
    
    int n_vars = 9 * _config.horizon;
    solution.resize(n_vars);
    solution.setZero();
    
    // Simple feasible solution: follow reference with zero acceleration
    // This is a placeholder - real implementation would use OSQP
    for (int k = 0; k < _config.horizon; ++k) {
        int base = 9 * k;
        // Position will be set by extractSolution
        // Velocity and acceleration zero
    }
    
    return true;
}

MPCAvoidance::MPCSolution MPCAvoidance::extractSolution(const Eigen::VectorXd& solution,
                                                         const AgentState& ego) {
    MPCSolution sol;
    sol.feasible = true;
    sol.positions.resize(_config.horizon);
    sol.velocities.resize(_config.horizon);
    sol.accelerations.resize(_config.horizon);
    
    for (int k = 0; k < _config.horizon; ++k) {
        int base = 9 * k;
        
        sol.positions[k] = {
            static_cast<float>(solution[base + 0]),
            static_cast<float>(solution[base + 1]),
            static_cast<float>(solution[base + 2])
        };
        
        sol.velocities[k] = {
            static_cast<float>(solution[base + 3]),
            static_cast<float>(solution[base + 4]),
            static_cast<float>(solution[base + 5])
        };
        
        sol.accelerations[k] = {
            static_cast<float>(solution[base + 6]),
            static_cast<float>(solution[base + 7]),
            static_cast<float>(solution[base + 8])
        };
    }
    
    // Compute cost
    sol.cost = 0.0f;  // Would compute from QP cost
    
    return sol;
}

bool MPCAvoidance::checkCollision(const Vector3& pos1, float r1, 
                                   const Vector3& pos2, float r2) const {
    float dx = pos1.x - pos2.x;
    float dy = pos1.y - pos2.y;
    float dz = pos1.z - pos2.z;
    float dist2 = dx*dx + dy*dy + dz*dz;
    float r_sum = r1 + r2;
    return dist2 <= r_sum * r_sum;
}

float MPCAvoidance::distanceToObstacle(const Vector3& pos, const Obstacle& obst) const {
    float dx = pos.x - obst.position.x;
    float dy = pos.y - obst.position.y;
    float dz = pos.z - obst.position.z;
    float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
    return dist - obst.radius;
}

double MPCAvoidance::barrierCost(double distance, double safety_dist) const {
    if (distance <= safety_dist) {
        return 1e6;  // Large penalty
    }
    double diff = distance - safety_dist;
    return 1.0 / (diff * diff);  // Quadratic barrier
}

std::vector<Vector3> MPCAvoidance::predictNeighborTrajectory(const Neighbor& neighbor) const {
    std::vector<Vector3> trajectory(_config.horizon);
    for (int k = 0; k < _config.horizon; ++k) {
        trajectory[k] = neighbor.position + neighbor.velocity * (_config.dt * (k + 1));
    }
    return trajectory;
}

std::vector<Vector3> MPCAvoidance::predictObstacleTrajectory(const Obstacle& obst) const {
    std::vector<Vector3> trajectory(_config.horizon);
    for (int k = 0; k < _config.horizon; ++k) {
        trajectory[k] = obst.position;  // Static obstacle
    }
    return trajectory;
}

} // namespace swarm::autonomy