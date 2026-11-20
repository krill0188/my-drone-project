#include "swarm_formation.hpp"
#include "swarm_comm/mavlink_router.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <cmath>

namespace swarm::formation {

LeaderTrajectoryGenerator::LeaderTrajectoryGenerator() 
    : _current_wp_index(0), _waypoint_reached(false) {}

void LeaderTrajectoryGenerator::setWaypoints(const std::vector<Vector3>& waypoints) {
    _waypoints = waypoints;
    _current_wp_index = 0;
    _waypoint_reached = false;
}

void LeaderTrajectoryGenerator::setWaypoints(const std::vector<Vector3>& waypoints, 
                                              const std::vector<float>& speeds,
                                              const std::vector<float>& loiter_times) {
    _waypoints = waypoints;
    _wp_speeds = speeds;
    _wp_loiter_times = loiter_times;
    _current_wp_index = 0;
    _waypoint_reached = false;
    _wp_arrival_time_us = 0;
}

Vector3 LeaderTrajectoryGenerator::computeVelocity(const Vector3& current_pos, 
                                                    const Vector3& current_vel,
                                                    float dt) {
    if (_waypoints.empty() || _current_wp_index >= _waypoints.size()) {
        return Vector3{0, 0, 0};  // Mission complete
    }
    
    Vector3 target = _waypoints[_current_wp_index];
    Vector3 to_target = target - current_pos;
    float distance = std::sqrt(to_target.x * to_target.x + to_target.y * to_target.y + to_target.z * to_target.z);
    
    // Check waypoint arrival
    float acceptance_radius = 2.0f;  // meters
    if (distance < acceptance_radius) {
        if (!_waypoint_reached) {
            _waypoint_reached = true;
            _wp_arrival_time_us = Timestamp::nowUs();
        }
        
        // Check loiter time
        float loiter_time = (_current_wp_index < _wp_loiter_times.size()) ? _wp_loiter_times[_current_wp_index] : 0.0f;
        if (loiter_time > 0.0f) {
            uint64_t elapsed_us = Timestamp::nowUs() - _wp_arrival_time_us;
            if (elapsed_us < static_cast<uint64_t>(loiter_time * 1e6)) {
                return Vector3{0, 0, 0};  // Loitering
            }
        }
        
        // Advance to next waypoint
        _current_wp_index++;
        _waypoint_reached = false;
        if (_current_wp_index >= _waypoints.size()) {
            return Vector3{0, 0, 0};  // Mission complete
        }
        target = _waypoints[_current_wp_index];
        to_target = target - current_pos;
        distance = std::sqrt(to_target.x * to_target.x + to_target.y * to_target.y + to_target.z * to_target.z);
    }
    
    // Get target speed for current waypoint
    float target_speed = (_current_wp_index < _wp_speeds.size()) ? _wp_speeds[_current_wp_index] : 5.0f;
    
    // Compute desired velocity vector
    Vector3 desired_vel;
    if (distance > 0.1f) {
        desired_vel.x = (to_target.x / distance) * target_speed;
        desired_vel.y = (to_target.y / distance) * target_speed;
        desired_vel.z = (to_target.z / distance) * target_speed;
    } else {
        desired_vel = Vector3{0, 0, 0};
    }
    
    // Energy-aware speed adjustment (simplified)
    // In real implementation, would consider battery, wind, etc.
    
    return desired_vel;
}

bool LeaderTrajectoryGenerator::isMissionComplete() const {
    return _current_wp_index >= _waypoints.size();
}

size_t LeaderTrajectoryGenerator::getCurrentWaypointIndex() const {
    return _current_wp_index;
}

Vector3 LeaderTrajectoryGenerator::getCurrentTarget() const {
    if (_current_wp_index < _waypoints.size()) {
        return _waypoints[_current_wp_index];
    }
    return Vector3{0, 0, 0};
}

float LeaderTrajectoryGenerator::getDistanceToTarget(const Vector3& current_pos) const {
    if (_current_wp_index >= _waypoints.size()) return 0.0f;
    Vector3 diff = _waypoints[_current_wp_index] - current_pos;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}

void LeaderTrajectoryGenerator::advanceWaypoint() {
    if (_current_wp_index < _waypoints.size()) {
        _current_wp_index++;
        _waypoint_reached = false;
    }
}

void LeaderTrajectoryGenerator::setCurrentWaypointIndex(size_t index) {
    if (index < _waypoints.size()) {
        _current_wp_index = index;
        _waypoint_reached = false;
    }
}

} // namespace swarm::formation