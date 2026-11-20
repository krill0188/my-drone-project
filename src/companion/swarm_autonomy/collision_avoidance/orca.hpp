#ifndef ORCA_HPP
#define ORCA_HPP

#include "swarm_formation.hpp"
#include <vector>
#include <array>
#include <cmath>

namespace swarm::autonomy {

// Optimal Reciprocal Collision Avoidance (ORCA) for swarm
class ORCA {
public:
    struct Config {
        float time_horizon = 2.0f;          // Time horizon for collision avoidance
        float time_horizon_obst = 1.0f;     // Time horizon for static obstacles
        float neighbor_dist = 20.0f;        // Max distance to consider neighbors
        float max_speed = 15.0f;            // Max speed
        float safety_margin = 0.5f;         // Safety margin
        bool use_3d = true;                 // 3D ORCA
    };
    
    struct Agent {
        uint16_t id;
        Vector3 position;
        Vector3 velocity;
        Vector3 pref_velocity;  // Preferred velocity
        float radius;
        float max_speed;
    };
    
    struct Obstacle {
        Vector3 position;
        float radius;
    };
    
    struct ORCAPlane {
        Vector3 normal;    // Normal pointing towards valid region
        float offset;      // Distance from origin
    };
    
    ORCA(const Config& config = Config{});
    
    // Compute collision-free velocity for agent
    Vector3 computeVelocity(const Agent& agent, 
                            const std::vector<Agent>& neighbors,
                            const std::vector<Obstacle>& obstacles);
    
    // Compute ORCA planes for agent-neighbor pair
    std::vector<ORCAPlane> computeORCAPlanes(const Agent& agent, const Agent& neighbor);
    
    // Compute ORCA planes for agent-obstacle pair
    std::vector<ORCAPlane> computeObstaclePlanes(const Agent& agent, const Obstacle& obstacle);
    
    // Linear programming to find optimal velocity
    Vector3 linearProgram(const std::vector<ORCAPlane>& planes, 
                          const Vector3& pref_velocity,
                          float max_speed);
    
    // Check if velocity is valid
    bool isValidVelocity(const Vector3& vel, const std::vector<ORCAPlane>& planes);

private:
    Config _config;
    
    // Compute time to collision
    float computeTTC(const Agent& a, const Agent& b);
    
    // Compute relative position and velocity
    void computeRelative(const Agent& a, const Agent& b, 
                         Vector3& rel_pos, Vector3& rel_vel);
    
    // Compute ORCA plane for agent-agent
    ORCAPlane computeORCAPlaneAgent(const Agent& a, const Agent& b);
    
    // Compute ORCA plane for agent-obstacle
    ORCAPlane computeORCAPlaneObstacle(const Agent& agent, const Obstacle& obst);
    
    // 3D linear program
    Vector3 linearProgram3D(const std::vector<ORCAPlane>& planes,
                           const Vector3& pref_vel,
                           float max_speed);
    
    // Helper functions
    float dot(const Vector3& a, const Vector3& b) const;
    Vector3 cross(const Vector3& a, const Vector3& b) const;
    float norm(const Vector3& v) const;
    Vector3 normalize(const Vector3& v) const;
};

} // namespace swarm::autonomy

#endif // ORCA_HPP