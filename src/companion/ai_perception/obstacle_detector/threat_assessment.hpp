#ifndef THREAT_ASSESSMENT_HPP
#define THREAT_ASSESSMENT_HPP

#include "obstacle_detector.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>

namespace swarm::ai {

class ThreatAssessment {
public:
    struct Config {
        // Threat assessment horizon
        float time_horizon_s = 5.0f;
        
        // Critical distance thresholds
        float critical_distance_m = 50.0f;
        float high_distance_m = 100.0f;
        float medium_distance_m = 200.0f;
        
        // Critical TTC thresholds
        float critical_ttc_s = 3.0f;
        float high_ttc_s = 5.0f;
        float medium_ttc_s = 10.0f;
        
        // Obstacle class weights (higher = more threatening)
        std::map<int, float> class_weights = {
            {1, 10.0f},   // power_line
            {15, 10.0f},  // wire
            {16, 10.0f},  // cable
            {7, 9.0f},    // aircraft
            {6, 8.0f},    // drone
            {2, 7.0f},    // tower
            {10, 7.0f},   // wind_turbine
            {11, 7.0f},   // crane
            {12, 6.0f},   // antenna
            {17, 6.0f},   // pole
            {5, 5.0f},    // bird
            {3, 4.0f},    // building
            {18, 4.0f},   // fence
            {19, 4.0f},   // wall
            {8, 3.0f},    // vehicle
            {9, 3.0f},    // person
            {4, 2.0f},    // tree
            {13, 2.0f},   // balloon
            {14, 2.0f},   // kite
            {0, 1.0f}     // unknown
        };
        
        // Velocity weight
        float velocity_weight = 1.5f;
        
        // Size weight (larger obstacles more threatening)
        float size_weight = 0.5f;
        
        // Immediate avoidance threshold
        float immediate_avoidance_score = 0.8f;
    };
    
    struct ThreatResult {
        ThreatLevel threat_level;
        float threat_score;  // 0.0 to 1.0
        float time_to_collision_s;
        bool requires_immediate_avoidance;
        Eigen::Vector3f avoidance_vector;  // Suggested avoidance direction (ENU)
        float avoidance_urgency;  // 0.0 to 1.0
        std::string reasoning;
    };
    
    ThreatAssessment() = default;
    ~ThreatAssessment() = default;
    
    void configure(const Config& config) { config_ = config; }
    
    // Assess threat for single obstacle
    ThreatResult assess(const ObstacleDetector::TrackedObstacle& obstacle,
                        const Eigen::Vector3f& ego_velocity,
                        const Eigen::Vector3f& ego_position = Eigen::Vector3f::Zero()) const;
    
    // Assess threats for multiple obstacles (returns highest threat)
    ThreatResult assessMultiple(const std::vector<ObstacleDetector::TrackedObstacle>& obstacles,
                                const Eigen::Vector3f& ego_velocity,
                                const Eigen::Vector3f& ego_position = Eigen::Vector3f::Zero()) const;
    
    // Compute combined avoidance vector for all threats
    Eigen::Vector3f computeAvoidanceVector(const std::vector<ObstacleDetector::TrackedObstacle>& obstacles,
                                           const Eigen::Vector3f& ego_velocity,
                                           const Eigen::Vector3f& ego_position = Eigen::Vector3f::Zero()) const;
    
    // Check if any obstacle requires immediate avoidance
    bool requiresImmediateAvoidance(const std::vector<ObstacleDetector::TrackedObstacle>& obstacles,
                                    const Eigen::Vector3f& ego_velocity) const;

private:
    Config config_;
    
    // Compute threat score for single obstacle
    float computeThreatScore(const ObstacleDetector::TrackedObstacle& obstacle,
                             const Eigen::Vector3f& ego_velocity,
                             const Eigen::Vector3f& ego_position) const;
    
    // Compute TTC
    float computeTTC(const ObstacleDetector::TrackedObstacle& obstacle,
                     const Eigen::Vector3f& ego_velocity) const;
    
    // Compute avoidance vector for single obstacle
    Eigen::Vector3f computeObstacleAvoidance(const ObstacleDetector::TrackedObstacle& obstacle,
                                             const Eigen::Vector3f& ego_velocity) const;
};

}  // namespace swarm::ai

#endif  // THREAT_ASSESSMENT_HPP