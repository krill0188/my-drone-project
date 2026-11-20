#include "threat_assessment.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iostream>

namespace swarm::ai {

ThreatAssessment::ThreatResult ThreatAssessment::assess(
    const ObstacleDetector::TrackedObstacle& obstacle,
    const Eigen::Vector3f& ego_velocity,
    const Eigen::Vector3f& ego_position) const {
    
    ThreatResult result;
    
    // Compute threat score
    float score = computeThreatScore(obstacle, ego_velocity, ego_position);
    result.threat_score = std::clamp(score, 0.0f, 1.0f);
    
    // Compute TTC
    result.time_to_collision_s = computeTTC(obstacle, ego_velocity);
    
    // Determine threat level
    if (result.threat_score >= 0.8f || 
        (result.time_to_collision_s > 0 && result.time_to_collision_s < config_.critical_ttc_s)) {
        result.threat_level = ThreatLevel::CRITICAL;
    } else if (result.threat_score >= 0.6f ||
               (result.time_to_collision_s > 0 && result.time_to_collision_s < config_.high_ttc_s)) {
        result.threat_level = ThreatLevel::HIGH;
    } else if (result.threat_score >= 0.3f ||
               (result.time_to_collision_s > 0 && result.time_to_collision_s < config_.medium_ttc_s)) {
        result.threat_level = ThreatLevel::MEDIUM;
    } else {
        result.threat_level = ThreatLevel::LOW;
    }
    
    // Check immediate avoidance
    result.requires_immediate_avoidance = (result.threat_score >= config_.immediate_avoidance_score) ||
                                          (result.time_to_collision_s > 0 && result.time_to_collision_s < config_.critical_ttc_s);
    
    // Compute avoidance vector
    result.avoidance_vector = computeObstacleAvoidance(obstacle, ego_velocity);
    result.avoidance_urgency = result.threat_score;
    
    // Generate reasoning
    std::stringstream ss;
    ss << "Class: " << obstacle.class_name << " (priority: " << config_.class_weights.at(obstacle.class_id) << ")";
    ss << ", Dist: " << obstacle.position_enu.norm() << "m";
    ss << ", TTC: " << (result.time_to_collision_s > 0 ? std::to_string(result.time_to_collision_s) : "inf") << "s";
    ss << ", RelVel: " << obstacle.velocity_enu.norm() << "m/s";
    ss << ", Score: " << result.threat_score;
    result.reasoning = ss.str();
    
    return result;
}

ThreatAssessment::ThreatResult ThreatAssessment::assessMultiple(
    const std::vector<ObstacleDetector::TrackedObstacle>& obstacles,
    const Eigen::Vector3f& ego_velocity,
    const Eigen::Vector3f& ego_position) const {
    
    ThreatResult max_threat;
    max_threat.threat_level = ThreatLevel::LOW;
    max_threat.threat_score = 0.0f;
    max_threat.time_to_collision_s = -1.0f;
    max_threat.requires_immediate_avoidance = false;
    max_threat.avoidance_vector = Eigen::Vector3f::Zero();
    max_threat.avoidance_urgency = 0.0f;
    
    for (const auto& obs : obstacles) {
        ThreatResult threat = assess(obs, ego_velocity, ego_position);
        
        if (threat.threat_score > max_threat.threat_score) {
            max_threat = threat;
        }
    }
    
    return max_threat;
}

Eigen::Vector3f ThreatAssessment::computeAvoidanceVector(
    const std::vector<ObstacleDetector::TrackedObstacle>& obstacles,
    const Eigen::Vector3f& ego_velocity,
    const Eigen::Vector3f& ego_position) const {
    
    Eigen::Vector3f total_avoidance = Eigen::Vector3f::Zero();
    float total_weight = 0.0f;
    
    for (const auto& obs : obstacles) {
        ThreatResult threat = assess(obs, ego_velocity, ego_position);
        
        if (threat.threat_level >= ThreatLevel::MEDIUM) {
            float weight = threat.threat_score * threat.avoidance_urgency;
            total_avoidance += weight * threat.avoidance_vector;
            total_weight += weight;
        }
    }
    
    if (total_weight > 0) {
        total_avoidance /= total_weight;
        // Normalize to unit vector
        if (total_avoidance.norm() > 0) {
            total_avoidance.normalize();
        }
    }
    
    return total_avoidance;
}

bool ThreatAssessment::requiresImmediateAvoidance(
    const std::vector<ObstacleDetector::TrackedObstacle>& obstacles,
    const Eigen::Vector3f& ego_velocity) const {
    
    for (const auto& obs : obstacles) {
        ThreatResult threat = assess(obs, ego_velocity);
        if (threat.requires_immediate_avoidance) {
            return true;
        }
    }
    return false;
}

float ThreatAssessment::computeThreatScore(
    const ObstacleDetector::TrackedObstacle& obstacle,
    const Eigen::Vector3f& ego_velocity,
    const Eigen::Vector3f& ego_position) const {
    
    float score = 0.0f;
    
    // 1. Class weight (0-1)
    float class_weight = 1.0f;
    auto it = config_.class_weights.find(obstacle.class_id);
    if (it != config_.class_weights.end()) {
        class_weight = it->second / 10.0f;  // Normalize to 0-1
    }
    score += 0.4f * class_weight;
    
    // 2. Distance factor (closer = higher threat)
    float distance = obstacle.position_enu.norm();
    float distance_score = 0.0f;
    if (distance < config_.critical_distance_m) {
        distance_score = 1.0f - distance / config_.critical_distance_m;
    } else if (distance < config_.high_distance_m) {
        distance_score = 0.5f * (1.0f - (distance - config_.critical_distance_m) / 
                                 (config_.high_distance_m - config_.critical_distance_m));
    } else if (distance < config_.medium_distance_m) {
        distance_score = 0.2f * (1.0f - (distance - config_.high_distance_m) / 
                                 (config_.medium_distance_m - config_.high_distance_m));
    }
    score += 0.3f * distance_score;
    
    // 3. Relative velocity factor
    Eigen::Vector3f rel_vel = obstacle.velocity_enu - ego_velocity;
    float rel_speed = rel_vel.norm();
    float ego_speed = ego_velocity.norm();
    float closing_speed = -rel_vel.dot(obstacle.position_enu) / (distance + 1e-6);
    
    float velocity_score = 0.0f;
    if (closing_speed > 0) {
        // Approaching
        velocity_score = std::min(1.0f, closing_speed / 20.0f);  // Normalize to 20 m/s max
    } else if (rel_speed > ego_speed) {
        // Moving faster than us
        velocity_score = 0.3f;
    }
    score += 0.2f * velocity_score * config_.velocity_weight;
    
    // 4. Size factor (larger = more threatening)
    float size = obstacle.size.norm();
    float size_score = std::min(1.0f, size / 10.0f);  // Normalize to 10m max
    score += 0.1f * size_score * config_.size_weight;
    
    // 5. Confidence factor
    score *= obstacle.confidence;
    
    return std::clamp(score, 0.0f, 1.0f);
}

float ThreatAssessment::computeTTC(
    const ObstacleDetector::TrackedObstacle& obstacle,
    const Eigen::Vector3f& ego_velocity) const {
    
    Eigen::Vector3f rel_pos = obstacle.position_enu - ego_position;
    Eigen::Vector3f rel_vel = obstacle.velocity_enu - ego_velocity;
    
    float distance = rel_pos.norm();
    float closing_speed = -rel_vel.dot(rel_pos) / (distance + 1e-6);
    
    if (closing_speed <= 0.1f) {
        return -1.0f;  // Not approaching
    }
    
    return distance / closing_speed;
}

Eigen::Vector3f ThreatAssessment::computeObstacleAvoidance(
    const ObstacleDetector::TrackedObstacle& obstacle,
    const Eigen::Vector3f& ego_velocity) const {
    
    Eigen::Vector3f rel_pos = obstacle.position_enu;
    Eigen::Vector3f rel_vel = obstacle.velocity_enu - ego_velocity;
    
    // Avoidance direction: perpendicular to relative position, away from obstacle
    Eigen::Vector3f avoidance_dir;
    
    if (rel_pos.norm() > 1e-3) {
        // Find perpendicular direction in horizontal plane
        Eigen::Vector3f horizontal_pos = rel_pos;
        horizontal_pos.z() = 0;
        
        if (horizontal_pos.norm() > 1e-3) {
            // Perpendicular in horizontal plane (rotate 90 degrees)
            avoidance_dir = Eigen::Vector3f(-horizontal_pos.y(), horizontal_pos.x(), 0);
            avoidance_dir.normalize();
        } else {
            // Obstacle directly above/below - avoid horizontally
            avoidance_dir = Eigen::Vector3f(1, 0, 0);
        }
    } else {
        avoidance_dir = Eigen::Vector3f(1, 0, 0);
    }
    
    // Add vertical component if needed
    if (std::abs(rel_pos.z()) < 5.0f) {
        // Similar altitude - add vertical separation
        avoidance_dir.z() = (rel_pos.z() > 0) ? -0.5f : 0.5f;
        avoidance_dir.normalize();
    }
    
    // Weight by threat score
    float threat_score = computeThreatScore(obstacle, ego_velocity, Eigen::Vector3f::Zero());
    avoidance_dir *= threat_score;
    
    return avoidance_dir;
}

}  // namespace swarm::ai