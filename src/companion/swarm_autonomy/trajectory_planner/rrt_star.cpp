#include "trajectory_planner.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <limits>
#include <queue>
#include <unordered_map>

namespace swarm::planner {

// ============================================================================
// RRTStarPlanner Implementation
// ============================================================================

RRTStarPlanner::RRTStarPlanner(const PlannerConfig& config) : _config(config) {
    _nodes.reserve(config.max_iterations);
}

std::optional<std::vector<Vector3>> RRTStarPlanner::plan(const Vector3& start, const Vector3& goal,
                                                          const std::vector<Obstacle>& obstacles) {
    // Combine static and dynamic obstacles
    std::vector<Obstacle> all_obstacles = obstacles;
    all_obstacles.insert(all_obstacles.end(), _dynamic_obstacles.begin(), _dynamic_obstacles.end());
    
    // Check if start or goal in collision
    if (isPointInObstacle(start, all_obstacles) || isPointInObstacle(goal, all_obstacles)) {
        return std::nullopt;
    }
    
    // Clear previous tree
    _nodes.clear();
    _nodes.reserve(_config.max_iterations);
    
    // Create start node
    auto start_node = std::make_unique<Node>();
    start_node->position = start;
    start_node->cost = 0.0f;
    start_node->timestamp_us = 0;
    Node* start_ptr = start_node.get();
    _nodes.push_back(std::move(start_node));
    
    // Create goal node (will be connected when reached)
    _goal_node = nullptr;
    
    // Random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<float> dist_xy(-50.0f, 50.0f);  // Workspace bounds
    std::uniform_real_distribution<float> dist_z(0.0f, 50.0f);
    std::uniform_real_distribution<float> dist_goal(0.0f, 1.0f);
    
    float min_x = -100, max_x = 100;
    float min_y = -100, max_y = 100;
    float min_z = 0, max_z = 100;
    
    // Compute workspace bounds from start/goal/obstacles
    min_x = max_x = start.x;
    min_y = max_y = start.y;
    min_z = max_z = start.z;
    
    for (const auto& obs : all_obstacles) {
        min_x = std::min(min_x, obs.center.x - (obs.type == Obstacle::BOX ? obs.size.x : obs.size.y));
        max_x = std::max(max_x, obs.center.x + (obs.type == Obstacle::BOX ? obs.size.x : obs.size.y));
        min_y = std::min(min_y, obs.center.y - (obs.type == Obstacle::BOX ? obs.size.y : obs.size.y));
        max_y = std::max(max_y, obs.center.y + (obs.type == Obstacle::BOX ? obs.size.y : obs.size.y));
        min_z = std::min(min_z, obs.center.z - (obs.type == Obstacle::BOX ? obs.size.z : obs.size.y));
        max_z = std::max(max_z, obs.center.z + (obs.type == Obstacle::BOX ? obs.size.z : obs.size.y));
    }
    min_x = std::min(min_x, goal.x);
    max_x = std::max(max_x, goal.x);
    min_y = std::min(min_y, goal.y);
    max_y = std::max(max_y, goal.y);
    min_z = std::min(min_z, goal.z);
    max_z = std::max(max_z, goal.z);
    
    // Add margins
    float margin = 10.0f;
    min_x -= margin; max_x += margin;
    min_y -= margin; max_y += margin;
    min_z -= margin; max_z += margin;
    
    std::uniform_real_distribution<float> dist_x(min_x, max_x);
    std::uniform_real_distribution<float> dist_y(min_y, max_y);
    std::uniform_real_distribution<float> dist_z_(min_z, max_z);
    
    bool goal_reached = false;
    float goal_radius = _config.resolution * 2.0f;
    
    // RRT* main loop
    for (int iter = 0; iter < _config.max_iterations; ++iter) {
        // Sample random point (with goal bias)
        Vector3 sample;
        if (dist_goal(gen) < _config.goal_bias) {
            sample = goal;
        } else {
            sample = {dist_x(gen), dist_y(gen), dist_z_(gen)};
        }
        
        // Find nearest node
        Node* nearest_node = nearest(sample);
        if (!nearest_node) continue;
        
        // Steer towards sample
        Vector3 new_point = steer(nearest_ptr->position, sample, _config.resolution);
        
        // Check collision
        if (!isCollisionFree(nearest_ptr->position, new_point, all_obstacles)) {
            continue;
        }
        
        // Create new node
        auto new_node = std::make_unique<Node>();
        new_node->position = new_point;
        new_node->parent = nearest_ptr;
        new_node->cost = nearest_ptr->cost + heuristic(nearest_ptr->position, new_point);
        new_node->timestamp_us = iter;
        
        Node* new_ptr = new_node.get();
        _nodes.push_back(std::move(new_node));
        
        // Find near nodes for rewiring
        std::vector<Node*> near_nodes = near(new_point, _config.rewire_radius);
        
        // Choose best parent
        Node* best_parent = nearest_ptr;
        float best_cost = nearest_ptr->cost + heuristic(nearest_ptr->position, new_point);
        
        for (Node* near_node : near_nodes) {
            if (near_node == nearest_ptr) continue;
            if (isCollisionFree(near_node->position, new_point, all_obstacles)) {
                float cost = near_node->cost + heuristic(near_node->position, new_point);
                if (cost < best_cost) {
                    best_cost = cost;
                    best_parent = near_node;
                }
            }
        }
        
        // Update parent and cost
        new_ptr->parent = best_parent;
        new_ptr->cost = best_cost;
        best_parent->children.push_back(new_ptr);
        
        // Rewire
        rewire(new_ptr, near_nodes);
        
        // Check if goal reached
        if (heuristic(new_point, goal) < goal_radius) {
            if (isCollisionFree(new_point, goal, all_obstacles)) {
                _goal_node = new_ptr;
                goal_reached = true;
                break;
            }
        }
    }
    
    if (!goal_reached || !_goal_node) {
        return std::nullopt;
    }
    
    // Extract path
    return extractPath(_goal_node);
}

std::optional<std::vector<Vector3>> RRTStarPlanner::replan(const Vector3& new_start, const Vector3& goal,
                                                            const std::vector<Obstacle>& obstacles) {
    // For replanning, we can reuse part of the existing tree
    // Find the node closest to new_start in existing tree
    Node* closest = nullptr;
    float min_dist = std::numeric_limits<float>::max();
    
    for (auto& node : _nodes) {
        float dist = heuristic(node->position, new_start);
        if (dist < min_dist) {
            min_dist = dist;
            closest = node.get();
        }
    }
    
    if (!closest || min_dist > _config.resolution * 5.0f) {
        // Too far, full replan
        return plan(new_start, goal, obstacles);
    }
    
    // Use existing tree from closest node
    // This is a simplified approach - full implementation would do partial replanning
    return plan(new_start, goal, obstacles);
}

RRTStarPlanner::Node* RRTStarPlanner::nearest(const Vector3& point) {
    Node* nearest = nullptr;
    float min_dist = std::numeric_limits<float>::max();
    
    for (auto& node : _nodes) {
        float dist = heuristic(node->position, point);
        if (dist < min_dist) {
            min_dist = dist;
            nearest = node.get();
        }
    }
    return nearest;
}

Vector3 RRTStarPlanner::steer(const Vector3& from, const Vector3& to, float max_dist) {
    Vector3 dir = to - from;
    float dist = std::sqrt(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
    
    if (dist <= max_dist) {
        return to;
    }
    
    float scale = max_dist / dist;
    return {from.x + dir.x * scale, from.y + dir.y * scale, from.z + dir.z * scale};
}

bool RRTStarPlanner::isCollisionFree(const Vector3& from, const Vector3& to,
                                      const std::vector<Obstacle>& obstacles) {
    // Check line segment against all obstacles
    for (const auto& obs : obstacles) {
        switch (obs.type) {
            case Obstacle::SPHERE:
                if (checkSphereCollision(from, to, obs)) return false;
                break;
            case Obstacle::BOX:
                if (checkBoxCollision(from, to, obs)) return false;
                break;
            case Obstacle::CYLINDER:
                if (checkCylinderCollision(from, to, obs)) return false;
                break;
        }
    }
    return true;
}

bool RRTStarPlanner::isPointInObstacle(const Vector3& point, const std::vector<Obstacle>& obstacles) {
    for (const auto& obs : obstacles) {
        switch (obs.type) {
            case Obstacle::SPHERE: {
                Vector3 diff = point - obs.center;
                float dist2 = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
                float radius = obs.size.y;  // Using size.y as radius for sphere
                if (dist2 < (radius + _config.safety_margin) * (radius + _config.safety_margin)) {
                    return true;
                }
                break;
            }
            case Obstacle::BOX: {
                if (std::abs(point.x - obs.center.x) < (obs.size.x + _config.safety_margin) &&
                    std::abs(point.y - obs.center.y) < (obs.size.y + _config.safety_margin) &&
                    std::abs(point.z - obs.center.z) < (obs.size.z + _config.safety_margin)) {
                    return true;
                }
                break;
            }
            case Obstacle::CYLINDER: {
                float dx = point.x - obs.center.x;
                float dy = point.y - obs.center.y;
                float dist2_xy = dx * dx + dy * dy;
                float radius = obs.size.y;
                float half_height = obs.size.z;
                if (dist2_xy < (radius + _config.safety_margin) * (radius + _config.safety_margin) &&
                    std::abs(point.z - obs.center.z) < (half_height + _config.safety_margin)) {
                    return true;
                }
                break;
            }
        }
    }
    return false;
}

std::vector<RRTStarPlanner::Node*> RRTStarPlanner::near(const Vector3& point, float radius) {
    std::vector<Node*> near_nodes;
    float radius2 = radius * radius;
    
    for (auto& node : _nodes) {
        Vector3 diff = node->position - point;
        float dist2 = diff.x * diff.x + diff.y * diff.y + diff.z * diff.z;
        if (dist2 < radius2) {
            near_nodes.push_back(node.get());
        }
    }
    return near_nodes;
}

void RRTStarPlanner::rewire(Node* new_node, const std::vector<Node*>& near_nodes) {
    for (Node* near_node : near_nodes) {
        if (near_node == new_node || near_node == new_node->parent) continue;
        
        float new_cost = new_node->cost + heuristic(new_node->position, near_node->position);
        if (new_cost < near_node->cost) {
            // Check collision for new connection
            if (isCollisionFree(new_node->position, near_node->position, _dynamic_obstacles)) {
                // Remove from old parent
                if (near_node->parent) {
                    auto& siblings = near_node->parent->children;
                    siblings.erase(std::remove(siblings.begin(), siblings.end(), near_node), siblings.end());
                }
                
                // Connect to new parent
                near_node->parent = new_node;
                near_node->cost = new_cost;
                new_node->children.push_back(near_node);
                
                // Propagate cost change to children
                std::queue<Node*> q;
                q.push(near_node);
                while (!q.empty()) {
                    Node* n = q.front(); q.pop();
                    for (Node* child : n->children) {
                        child->cost = n->cost + heuristic(n->position, child->position);
                        q.push(child);
                    }
                }
            }
        }
    }
}

float RRTStarPlanner::computeCost(Node* node) {
    float cost = 0.0f;
    while (node->parent) {
        cost += heuristic(node->position, node->parent->position);
        node = node->parent;
    }
    return cost;
}

float RRTStarPlanner::heuristic(const Vector3& a, const Vector3& b) {
    Vector3 diff = a - b;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
}

std::vector<Vector3> RRTStarPlanner::extractPath(Node* goal_node) {
    std::vector<Vector3> path;
    Node* current = goal_node;
    while (current) {
        path.push_back(current->position);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

bool RRTStarPlanner::checkSphereCollision(const Vector3& p1, const Vector3& p2, const Obstacle& obs) {
    // Line-sphere intersection test
    Vector3 center = obs.center;
    float radius = obs.size.y + _config.safety_margin;
    
    Vector3 d = p2 - p1;
    Vector3 f = p1 - center;
    
    float a = d.x * d.x + d.y * d.y + d.z * d.z;
    float b = 2.0f * (f.x * d.x + f.y * d.y + f.z * d.z);
    float c = f.x * f.x + f.y * f.y + f.z * f.z - radius * radius;
    
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0) return false;
    
    discriminant = std::sqrt(discriminant);
    float t1 = (-b - discriminant) / (2.0f * a);
    float t2 = (-b + discriminant) / (2.0f * a);
    
    return (t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1);
}

bool RRTStarPlanner::checkBoxCollision(const Vector3& p1, const Vector3& p2, const Obstacle& obs) {
    // Line-box intersection using slab method
    Vector3 half_extents = obs.size;
    half_extents.x += _config.safety_margin;
    half_extents.y += _config.safety_margin;
    half_extents.z += _config.safety_margin;
    
    Vector3 min_bounds = obs.center - half_extents;
    Vector3 max_bounds = obs.center + half_extents;
    
    float tmin = 0.0f, tmax = 1.0f;
    
    for (int i = 0; i < 3; ++i) {
        float dir = (&p2.x)[i] - (&p1.x)[i];
        float origin = (&p1.x)[i];
        float min_b = (&min_bounds.x)[i];
        float max_b = (&max_bounds.x)[i];
        
        if (std::abs(dir) < 1e-6f) {
            if (origin < min_b || origin > max_b) return false;
        } else {
            float t1 = (min_b - origin) / dir;
            float t2 = (max_b - origin) / dir;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
    }
    return true;
}

bool RRTStarPlanner::checkCylinderCollision(const Vector3& p1, const Vector3& p2, const Obstacle& obs) {
    // Simplified: treat as box with cylinder bounds
    return checkBoxCollision(p1, p2, obs);
}

void RRTStarPlanner::setDynamicObstacles(const std::vector<Obstacle>& obstacles) {
    _dynamic_obstacles = obstacles;
}

} // namespace swarm::planner