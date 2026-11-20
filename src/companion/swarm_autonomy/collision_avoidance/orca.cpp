#include "orca.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace swarm::autonomy {

ORCA::ORCA(const Config& config) : _config(config) {}

Vector3 ORCA::computeVelocity(const Agent& agent, 
                              const std::vector<Agent>& neighbors,
                              const std::vector<Obstacle>& obstacles) {
    
    // Build ORCA planes
    std::vector<ORCAPlane> planes;
    
    // Agent-agent planes
    for (const auto& neighbor : neighbors) {
        if (neighbor.id == agent.id) continue;
        
        float dist = norm(neighbor.position - agent.position);
        if (dist > _config.neighbor_dist) continue;
        
        auto planes_ab = computeORCAPlanes(agent, neighbor);
        planes.insert(planes.end(), planes_ab.begin(), planes_ab.end());
    }
    
    // Agent-obstacle planes
    for (const auto& obst : obstacles) {
        float dist = norm(obst.position - agent.position);
        if (dist > _config.neighbor_dist) continue;
        
        auto planes_obst = computeORCAPlanesObstacle(agent, obst);
        planes.insert(planes.end(), planes_obst.begin(), planes_obst.end());
    }
    
    // Find optimal velocity using linear programming
    Vector3 new_vel = linearProgram(planes, agent.pref_velocity, agent.max_speed);
    
    // Clamp to max speed
    float speed = norm(new_vel);
    if (speed > agent.max_speed) {
        new_vel = normalize(new_vel) * agent.max_speed;
    }
    
    return new_vel;
}

std::vector<ORCA::ORCAPlane> ORCA::computeORCAPlanes(const Agent& a, const Agent& b) {
    std::vector<ORCAPlane> planes;
    
    Vector3 rel_pos = b.position - a.position;
    Vector3 rel_vel = b.velocity - a.velocity;
    
    float dist = norm(rel_pos);
    float combined_radius = a.radius + b.radius + _config.safety_margin;
    
    if (dist < combined_radius) {
        // Already colliding - push apart
        Vector3 normal = (dist > 0.001f) ? normalize(rel_pos) : Vector3{1, 0, 0};
        float offset = combined_radius - dist;
        
        ORCAPlane plane;
        plane.normal = normal;
        plane.offset = dot(normal, a.position) + offset;
        planes.push_back(plane);
    } else {
        // Compute ORCA plane
        Vector3 u = rel_vel - (rel_pos * (dot(rel_vel, rel_pos) / (dist * dist)));
        float u_norm = norm(u);
        
        if (u_norm > 0.001f) {
            // Moving towards collision
            float ttc = (dist - combined_radius) / u_norm;
            
            if (ttc < _config.time_horizon) {
                Vector3 w = rel_pos + u * _config.time_horizon;
                float w_norm = norm(w);
                
                if (w_norm > 0.001f) {
                    Vector3 normal = normalize(w);
                    float offset = dot(normal, a.position) + combined_radius;
                    
                    ORCAPlane plane;
                    plane.normal = normal;
                    plane.offset = offset;
                    planes.push_back(plane);
                }
            }
        }
    }
    
    return planes;
}

std::vector<ORCA::ORCAPlane> ORCA::computeORCAPlanes(const Agent& agent, const Obstacle& obstacle) {
    std::vector<ORCAPlane> planes;
    
    Vector3 rel_pos = obstacle.position - agent.position;
    Vector3 rel_vel = -agent.velocity;  // Obstacle is static
    
    float dist = norm(rel_pos);
    float combined_radius = agent.radius + obstacle.radius + _config.safety_margin;
    
    if (dist < combined_radius) {
        // Colliding with obstacle
        Vector3 normal = (dist > 0.001f) ? normalize(rel_pos) : Vector3{1, 0, 0};
        float offset = combined_radius - dist;
        
        ORCAPlane plane;
        plane.normal = normal;
        plane.offset = dot(normal, agent.position) + offset;
        planes.push_back(plane);
    } else {
        // Check if on collision course
        float proj = dot(rel_vel, rel_pos);
        if (proj < 0) {  // Moving towards obstacle
            float ttc = (dist - combined_radius) / (-proj / norm(rel_vel));
            
            if (ttc < _config.time_horizon_obst) {
                Vector3 w = rel_pos + (rel_vel * _config.time_horizon_obst);
                float w_norm = norm(w);
                
                if (w_norm > 0.001f) {
                    Vector3 normal = normalize(w);
                    float offset = dot(normal, agent.position) + combined_radius;
                    
                    ORCAPlane plane;
                    plane.normal = normal;
                    plane.offset = offset;
                    planes.push_back(plane);
                }
            }
        }
    }
    
    return planes;
}

Vector3 ORCA::linearProgram(const std::vector<ORCAPlane>& planes, 
                           const Vector3& pref_velocity,
                           float max_speed) {
    // Simplified 3D linear programming for ORCA
    // Start with preferred velocity
    Vector3 new_vel = pref_velocity;
    
    // Clamp to max speed
    float speed = norm(new_vel);
    if (speed > max_speed) {
        new_vel = normalize(new_vel) * max_speed;
    }
    
    // Project onto half-planes
    for (int iter = 0; iter < 10; ++iter) {  // Max 10 iterations
        bool feasible = true;
        int violating_idx = -1;
        float max_violation = 0.0f;
        
        // Find most violating plane
        for (size_t i = 0; i < planes.size(); ++i) {
            const auto& plane = planes[i];
            float dist = dot(plane.normal, new_vel) - plane.offset;
            if (dist > max_violation) {
                max_violation = dist;
                violating_idx = i;
                feasible = false;
            }
        }
        
        if (feasible) break;
        
        if (violating_idx >= 0) {
            // Project onto violating plane
            const auto& plane = planes[violating_idx];
            float dist = dot(plane.normal, new_vel) - plane.offset;
            new_vel = new_vel - plane.normal * dist;
        }
    }
    
    // Final speed clamp
    float speed = norm(new_vel);
    if (speed > max_speed) {
        new_vel = normalize(new_vel) * max_speed;
    }
    
    return new_vel;
}

bool ORCA::isValidVelocity(const Vector3& vel, const std::vector<ORCAPlane>& planes) {
    for (const auto& plane : planes) {
        if (dot(plane.normal, vel) > plane.offset + 0.001f) {
            return false;
        }
    }
    return true;
}

float ORCA::computeTTC(const Agent& a, const Agent& b) {
    Vector3 rel_pos = b.position - a.position;
    Vector3 rel_vel = b.velocity - a.velocity;
    
    float dist = norm(rel_pos);
    float rel_speed = norm(rel_vel);
    
    if (rel_speed < 0.001f) return std::numeric_limits<float>::infinity();
    
    float combined_radius = a.radius + b.radius + _config.safety_margin;
    if (dist <= combined_radius) return 0.0f;
    
    float proj = dot(rel_vel, rel_pos);
    if (proj >= 0) return std::numeric_limits<float>::infinity();  // Moving apart
    
    return (dist - a.radius - b.radius) / (-proj / norm(rel_vel));
}

void ORCA::computeRelative(const Agent& a, const Agent& b, 
                           Vector3& rel_pos, Vector3& rel_vel) {
    rel_pos = b.position - a.position;
    rel_vel = b.velocity - a.velocity;
}

ORCA::ORCAPlane ORCA::computeORCAPlaneAgent(const Agent& a, const Agent& b) {
    Vector3 rel_pos = b.position - a.position;
    Vector3 rel_vel = b.velocity - a.velocity;
    
    float dist = norm(rel_pos);
    float combined_radius = a.radius + b.radius + _config.safety_margin;
    
    ORCAPlane plane;
    
    if (dist < combined_radius) {
        Vector3 normal = (dist > 0.001f) ? normalize(b.position - a.position) : Vector3{1, 0, 0};
        float offset = combined_radius - dist;
        plane.normal = normal;
        plane.offset = dot(normal, a.position) + offset;
    } else {
        Vector3 u = rel_vel - rel_pos * (dot(rel_vel, rel_pos) / (dist * dist));
        float u_norm = norm(u);
        
        if (u_norm > 0.001f) {
            float ttc = (dist - combined_radius) / u_norm;
            
            if (ttc < _config.time_horizon) {
                Vector3 w = rel_pos + u * _config.time_horizon;
                float w_norm = norm(w);
                
                if (w_norm > 0.001f) {
                    Vector3 normal = normalize(w);
                    float offset = dot(normal, a.position) + combined_radius;
                    plane.normal = normal;
                    plane.offset = offset;
                }
            }
        }
    }
    
    return plane;
}

ORCA::ORCAPlane ORCA::computeORCAPlaneObstacle(const Agent& agent, const Obstacle& obst) {
    Vector3 rel_pos = obst.position - agent.position;
    Vector3 rel_vel = -agent.velocity;  // Obstacle static
    
    float dist = norm(rel_pos);
    float combined_radius = agent.radius + obst.radius + _config.safety_margin;
    
    ORCAPlane plane;
    
    if (dist < combined_radius) {
        Vector3 normal = (dist > 0.001f) ? normalize(rel_pos) : Vector3{1, 0, 0};
        float offset = combined_radius - dist;
        plane.normal = normal;
        plane.offset = dot(normal, agent.position) + offset;
    } else {
        float proj = dot(rel_vel, rel_pos);
        if (proj < 0) {
            float ttc = (dist - combined_radius) / (-proj / norm(rel_vel));
            
            if (ttc < _config.time_horizon_obst) {
                Vector3 w = rel_pos + rel_vel * _config.time_horizon_obst;
                float w_norm = norm(w);
                
                if (w_norm > 0.001f) {
                    Vector3 normal = normalize(w);
                    float offset = dot(normal, agent.position) + combined_radius;
                    plane.normal = normal;
                    plane.offset = offset;
                }
            }
        }
    }
    
    return plane;
}

float ORCA::dot(const Vector3& a, const Vector3& b) const {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 ORCA::cross(const Vector3& a, const Vector3& b) const {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

float ORCA::norm(const Vector3& v) const {
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vector3 ORCA::normalize(const Vector3& v) const {
    float n = norm(v);
    if (n < 0.001f) return {0, 0, 0};
    return {v.x / n, v.y / n, v.z / n};
}

} // namespace swarm::autonomy