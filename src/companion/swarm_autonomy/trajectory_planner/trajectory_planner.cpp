#include "trajectory_planner.hpp"
#include <algorithm>
#include <cmath>

namespace swarm::planner {

// ============================================================================
// TrajectoryPlanner Implementation
// ============================================================================

TrajectoryPlanner::TrajectoryPlanner(const PlannerConfig& config) 
    : _config(config), _rrt_planner(config), _mpc_planner(config) {}

Trajectory TrajectoryPlanner::planMission(const std::vector<Waypoint>& waypoints,
                                          const std::vector<Obstacle>& known_obstacles) {
    Trajectory full_trajectory;
    full_trajectory.planner_used = "RRT*+MPC";
    full_trajectory.valid = false;
    
    if (waypoints.size() < 2) {
        return full_trajectory;
    }
    
    _known_obstacles = known_obstacles;
    _current_waypoint_index = 0;
    
    // Plan global path through all waypoints
    std::vector<Vector3> global_path;
    
    for (size_t i = 0; i < waypoints.size() - 1; ++i) {
        Vector3 start = waypoints[i].position;
        Vector3 goal = waypoints[i+1].position;
        
        auto segment = _rrt_planner.plan(start, goal, known_obstacles);
        if (!segment) {
            // Planning failed
            return Trajectory{};
        }
        
        // Append segment (avoid duplicate waypoints)
        if (global_path.empty()) {
            global_path = *segment;
        } else {
            global_path.insert(global_path.end(), segment->begin() + 1, segment->end());
        }
    }
    
    // Smooth and interpolate the global path
    Trajectory smoothed = smoothTrajectory(interpolatePath(global_path, 0.1f));
    
    // Run MPC optimization for initial trajectory
    Vector3 start_pos = waypoints[0].position;
    Vector3 start_vel = {0, 0, 0};
    Trajectory optimized = _mpc_planner.optimize(start_pos, {0,0,0}, global_path, known_obstacles);
    
    // Merge RRT* path with MPC optimization
    Trajectory final_traj = mergeTrajectories(smoothed, optimized);
    
    final_traj.planner_used = "RRT*+MPC";
    final_traj.valid = true;
    _current_trajectory = final_traj;
    
    return final_traj;
}

Trajectory TrajectoryPlanner::replan(const Vector3& current_pos, const Vector3& current_vel,
                                     const std::vector<Obstacle>& current_obstacles) {
    // Combine known and current obstacles
    std::vector<Obstacle> all_obstacles = _known_obstacles;
    all_obstacles.insert(all_obstacles.end(), current_obstacles.begin(), current_obstacles.end());
    
    // RRT* replan from current position to current goal
    if (_current_waypoint_index < _current_trajectory.points.size()) {
        // Find target on current trajectory
        Vector3 goal = _current_trajectory.points.back().position;
        
        auto new_path = _rrt_planner.replan(current_pos, goal, all_obstacles);
        if (new_path) {
            // Update reference path for MPC
            _mpc_planner.setReferencePath(*new_path);
        }
    }
    
    // Run MPC optimization for short horizon
    Trajectory local_traj = _mpc_planner.optimize(current_pos, current_vel, 
                                                   _mpc_planner.getReferencePath(), 
                                                   all_obstacles);
    
    // Merge with remaining global trajectory
    return mergeTrajectories(_current_trajectory, local_traj);
}

Trajectory TrajectoryPlanner::localReplan(const Vector3& current_pos, const Vector3& current_vel,
                                          const std::vector<Obstacle>& dynamic_obstacles,
                                          const Trajectory& current_trajectory) {
    // Combine obstacles
    std::vector<Obstacle> all_obstacles = _known_obstacles;
    all_obstacles.insert(all_obstacles.end(), dynamic_obstacles.begin(), dynamic_obstacles.end());
    
    // Extract current reference path from current trajectory
    std::vector<Vector3> ref_path;
    for (const auto& pt : current_trajectory.points) {
        ref_path.push_back(pt.position);
    }
    
    if (ref_path.empty()) {
        return current_trajectory;
    }
    
    // Run MPC with updated obstacles
    return _mpc_planner.optimize(current_pos, current_vel, ref_path, all_obstacles);
}

Trajectory TrajectoryPlanner::emergencyReplan(const Vector3& current_pos, const Vector3& current_vel,
                                               const std::vector<Obstacle>& obstacles) {
    // Fast emergency replan: direct path to safe point
    std::vector<Obstacle> all_obstacles = _known_obstacles;
    all_obstacles.insert(all_obstacles.end(), obstacles.begin(), obstacles.end());
    
    // Find safe direction (away from nearest obstacle)
    Vector3 safe_dir = {0, 0, 1};  // Default: climb
    float min_dist = std::numeric_limits<float>::max();
    
    for (const auto& obs : obstacles) {
        Vector3 to_obs = obs.center - current_pos;
        float dist = std::sqrt(to_obs.x*to_obs.x + to_obs.y*to_obs.y + to_obs.z*to_obs.z);
        if (dist < min_dist) {
            min_dist = dist;
            // Direction away from obstacle
            safe_dir = -to_obs;
            float mag = std::sqrt(safe_dir.x*safe_dir.x + safe_dir.y*safe_dir.y + safe_dir.z*safe_dir.z);
            if (mag > 0) {
                safe_dir = safe_dir / mag;
            }
        }
    }
    
    // Emergency goal: move 50m in safe direction
    Vector3 emergency_goal = current_pos + safe_dir * 50.0f;
    emergency_goal.z = std::max(emergency_goal.z, 10.0f);  // Minimum altitude
    
    auto path = _rrt_planner.plan(current_pos, emergency_goal, _known_obstacles);
    if (!path) {
        // Fallback: simple straight line up
        Trajectory traj;
        traj.points = {
            {current_pos, current_vel, {0,0,0}, 0, 0},
            {emergency_goal, {0,0,0}, {0,0,0}, 0, 5000000}
        };
        traj.valid = true;
        traj.planner_used = "EMERGENCY";
        return traj;
    }
    
    return interpolatePath(*path, 0.1f);
}

void TrajectoryPlanner::updateObstacles(const std::vector<Obstacle>& obstacles) {
    _known_obstacles = obstacles;
    _rrt_planner.setDynamicObstacles(obstacles);
}

void TrajectoryPlanner::setConfig(const PlannerConfig& config) {
    _config = config;
    _rrt_planner = RRTStarPlanner(config);
    _mpc_planner = MPCPlanner(config);
}

Trajectory TrajectoryPlanner::interpolatePath(const std::vector<Vector3>& path, float dt) {
    Trajectory traj;
    traj.planner_used = "interpolated";
    
    if (path.size() < 2) {
        traj.valid = false;
        return traj;
    }
    
    float total_dist = 0.0f;
    for (size_t i = 1; i < path.size(); ++i) {
        Vector3 diff = path[i] - path[i-1];
        total_dist += std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
    }
    
    float total_time = total_dist / 5.0f;  // Assume 5 m/s average speed
    int num_points = static_cast<int>(total_time / dt) + 1;
    
    traj.points.resize(num_points);
    traj.total_time = total_time;
    traj.total_distance = total_dist;
    
    // Simple linear interpolation with constant velocity segments
    int point_idx = 0;
    for (int i = 0; i < num_points; ++i) {
        float t = static_cast<float>(i) / (num_points - 1);
        float target_dist = t * total_dist;
        
        // Find segment
        float accum_dist = 0.0f;
        size_t seg = 0;
        for (size_t j = 1; j < path.size(); ++j) {
            Vector3 diff = path[j] - path[j-1];
            float seg_dist = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
            if (accum_dist + seg_dist >= target_dist) {
                seg = j - 1;
                break;
            }
            accum_dist += seg_dist;
        }
        
        if (seg >= path.size() - 1) seg = path.size() - 2;
        
        Vector3 diff = path[seg+1] - path[seg];
        float seg_dist = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
        float seg_t = (target_dist - accum_dist) / (seg_dist + 1e-6);
        
        traj.points[i].position = {
            path[seg].x + diff.x * seg_t,
            path[seg].y + diff.y * seg_t,
            path[seg].z + diff.z * seg_t
        };
        
        if (seg_dist > 1e-6) {
            traj.points[i].velocity = {diff.x / (seg_dist/5.0f), diff.y / (seg_dist/5.0f), diff.z / (seg_dist/5.0f)};
        }
        
        traj.points[i].timestamp_us = static_cast<uint64_t>(t * total_time * 1e6);
    }
    
    traj.total_time = total_time;
    traj.total_distance = total_dist;
    traj.valid = true;
    traj.planner_used = "interpolated";
    
    return traj;
}

Trajectory TrajectoryPlanner::smoothTrajectory(const Trajectory& traj) {
    // Simple moving average smoothing
    if (traj.points.size() < 3) return traj;
    
    Trajectory smoothed = traj;
    int window = 3;
    
    for (size_t i = 1; i + 1 < traj.points.size(); ++i) {
        Vector3 sum = {0,0,0};
        int count = 0;
        for (int j = -window/2; j <= window/2; ++j) {
            size_t idx = i + j;
            if (idx < traj.points.size()) {
                sum.x += traj.points[idx].position.x;
                sum.y += traj.points[idx].position.y;
                sum.z += traj.points[idx].position.z;
                count++;
            }
        }
        if (count > 0) {
            smoothed.points[i].position = {sum.x/count, sum.y/count, sum.z/count};
        }
    }
    
    return smoothed;
}

bool TrajectoryPlanner::validateTrajectory(const Trajectory& traj, 
                                           const std::vector<Obstacle>& obstacles) {
    for (const auto& pt : traj.points) {
        for (const auto& obs : obstacles) {
            Vector3 diff = pt.position - obs.center;
            float dist2 = diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
            
            float safety_dist = 0.0f;
            if (obs.type == Obstacle::SPHERE) safety_dist = obs.size.y;
            else if (obs.type == Obstacle::BOX) safety_dist = std::max({obs.size.x, obs.size.y, obs.size.z});
            else safety_dist = obs.size.y;
            
            safety_dist += _config.safety_margin;
            
            if (dist2 < safety_dist * safety_dist) {
                return false;
            }
        }
    }
    return true;
}

Trajectory TrajectoryPlanner::mergeTrajectories(const Trajectory& global, const Trajectory& local) {
    // Simple concatenation: use local for first part, global for rest
    Trajectory merged = local;
    
    // Find transition point
    if (!global.points.empty() && !local.points.empty()) {
        // Find where local trajectory meets global
        Vector3 local_end = local.points.back().position;
        
        // Find closest point on global trajectory
        size_t transition_idx = 0;
        float min_dist = std::numeric_limits<float>::max();
        
        for (size_t i = 0; i < global.points.size(); ++i) {
            Vector3 diff = global.points[i].position - local_end;
            float dist = diff.x*diff.x + diff.y*diff.y + diff.z*diff.z;
            if (dist < min_dist) {
                min_dist = dist;
                transition_idx = i;
            }
        }
        
        // Append global trajectory from transition point
        for (size_t i = transition_idx + 1; i < global.points.size(); ++i) {
            merged.points.push_back(global.points[i]);
        }
        
        merged.total_time = local.total_time + 
                           (global.total_time * (global.points.size() - transition_idx) / global.points.size());
        merged.total_distance = local.total_distance + 
                               (global.total_distance * (global.points.size() - transition_idx) / global.points.size());
    }
    
    return merged;
}

// ============================================================================
// FormationTrajectoryPlanner Implementation
// ============================================================================

FormationTrajectoryPlanner::FormationTrajectoryPlanner(const PlannerConfig& config)
    : _config(config), _leader_planner(config) {}

Trajectory FormationTrajectoryPlanner::planFormationTrajectory(
    const std::vector<Waypoint>& waypoints,
    const std::vector<Obstacle>& obstacles,
    const std::vector<formation::FormationSlot>& follower_slots) {
    
    _follower_slots = follower_slots;
    
    // Plan leader trajectory
    Trajectory leader_traj = _leader_planner.planMission(waypoints, obstacles);
    
    if (!leader_traj.valid) {
        return Trajectory{};
    }
    
    // Generate follower trajectories
    applyFormationConstraints(leader_traj, _follower_trajectories);
    
    return leader_traj;
}

Trajectory FormationTrajectoryPlanner::replanFormation(
    const Vector3& leader_pos, const Vector3& leader_vel,
    const std::vector<Obstacle>& obstacles,
    const std::vector<formation::FormationSlot>& follower_slots) {
    
    _follower_slots = follower_slots;
    
    Trajectory leader_traj = _leader_planner.replan(leader_pos, leader_vel, obstacles);
    
    if (!leader_traj.valid) {
        return Trajectory{};
    }
    
    applyFormationConstraints(leader_traj, _follower_trajectories);
    
    return leader_traj;
}

std::vector<Trajectory> FormationTrajectoryPlanner::getFollowerTrajectories() const {
    return _follower_trajectories;
}

void FormationTrajectoryPlanner::applyFormationConstraints(
    Trajectory& leader_traj, std::vector<Trajectory>& follower_trajs) {
    
    follower_trajs.clear();
    follower_trajs.resize(_follower_slots.size());
    
    for (size_t f = 0; f < _follower_slots.size(); ++f) {
        const auto& slot = _follower_slots[f];
        follower_trajs[f].points.resize(leader_traj.points.size());
        follower_trajs[f].planner_used = "formation_follower_" + std::to_string(f);
        follower_trajs[f].valid = leader_traj.valid;
        
        for (size_t i = 0; i < leader_traj.points.size(); ++i) {
            const auto& leader_pt = leader_traj.points[i];
            
            // Compute follower target position in ENU
            formation::Vector3 offset_enu = slot.offset_flu;  // Would need rotation by leader yaw
            
            // For simplicity, assume leader orientation aligned with velocity
            // In real implementation, would use leader_pt orientation
            follower_trajs[f].points[i].position = {
                leader_pt.position.x + offset_enu.x,
                leader_pt.position.y + offset_enu.y,
                leader_pt.position.z + offset_enu.z
            };
            
            // Follower velocity = leader velocity (maintain formation)
            follower_trajs[f].points[i].velocity = leader_pt.velocity;
            follower_trajs[f].points[i].acceleration = leader_pt.acceleration;
            follower_trajs[f].points[i].timestamp_us = leader_pt.timestamp_us;
        }
        
        follower_trajs[f].total_time = leader_traj.total_time;
        follower_trajs[f].total_distance = leader_traj.total_distance;
        follower_trajs[f].valid = leader_traj.valid;
    }
}

} // namespace swarm::planner