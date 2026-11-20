#ifndef TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_HPP

#include <vector>
#include <array>
#include <memory>
#include <optional>
#include <functional>
#include "swarm_formation.hpp"

namespace swarm::planner {

struct Vector3 {
    float x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    
    Vector3 operator+(const Vector3& other) const { return {x + other.x, y + other.y, z + other.z}; }
    Vector3 operator-(const Vector3& other) const { return {x - other.x, y - other.y, z - other.z}; }
    Vector3 operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
    Vector3 operator/(float scalar) const { return {x / scalar, y / scalar, z / scalar}; }
};

struct Quaternion {
    float w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
};

struct Waypoint {
    Vector3 position;
    float speed = 5.0f;
    float loiter_time = 0.0f;
    float acceptance_radius = 2.0f;
};

struct Obstacle {
    Vector3 center;
    Vector3 size;  // half-extents for box, radius for sphere
    enum Type { SPHERE, BOX, CYLINDER } type = SPHERE;
    float confidence = 1.0f;
    uint64_t timestamp_us = 0;
};

struct TrajectoryPoint {
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    float yaw = 0.0f;
    uint64_t timestamp_us = 0;
};

struct Trajectory {
    std::vector<TrajectoryPoint> points;
    float total_time = 0.0f;
    float total_distance = 0.0f;
    bool valid = false;
    std::string planner_used;
};

struct PlannerConfig {
    float max_velocity = 10.0f;
    float max_acceleration = 5.0f;
    float max_jerk = 10.0f;
    float resolution = 0.5f;
    float safety_margin = 2.0f;
    int max_iterations = 5000;
    float goal_bias = 0.1f;
    float rewire_radius = 5.0f;
    int mpc_horizon = 20;
    float mpc_dt = 0.1f;
    bool enable_rrt_star = true;
    bool enable_mpc = true;
};

class RRTStarPlanner {
public:
    RRTStarPlanner(const PlannerConfig& config = PlannerConfig());
    
    struct Node {
        Vector3 position;
        Node* parent = nullptr;
        float cost = 0.0f;
        std::vector<Node*> children;
        uint64_t timestamp_us = 0;
    };
    
    // Plan path from start to goal avoiding obstacles
    std::optional<std::vector<Vector3>> plan(const Vector3& start, const Vector3& goal,
                                              const std::vector<Obstacle>& obstacles);
    
    // Set dynamic obstacles (updated at runtime)
    void setDynamicObstacles(const std::vector<Obstacle>& obstacles);
    
    // Get the tree for visualization
    const std::vector<std::unique_ptr<Node>>& getNodes() const { return _nodes; }
    
    // Replan with new start (for online replanning)
    std::optional<std::vector<Vector3>> replan(const Vector3& new_start, const Vector3& goal,
                                               const std::vector<Obstacle>& obstacles);
    
private:
    PlannerConfig _config;
    std::vector<std::unique_ptr<Node>> _nodes;
    std::vector<Obstacle> _dynamic_obstacles;
    Node* _goal_node = nullptr;
    
    // RRT* core functions
    Node* nearest(const Vector3& point);
    Vector3 steer(const Vector3& from, const Vector3& to, float max_dist);
    bool isCollisionFree(const Vector3& from, const Vector3& to, const std::vector<Obstacle>& obstacles);
    bool isPointInObstacle(const Vector3& point, const std::vector<Obstacle>& obstacles);
    std::vector<Node*> near(const Vector3& point, float radius);
    void rewire(Node* new_node, const std::vector<Node*>& near_nodes);
    float computeCost(Node* node);
    float heuristic(const Vector3& a, const Vector3& b);
    std::vector<Vector3> extractPath(Node* goal_node);
    
    // Obstacle checking
    bool checkSphereCollision(const Vector3& p1, const Vector3& p2, const Obstacle& obs);
    bool checkBoxCollision(const Vector3& p1, const Vector3& p2, const Obstacle& obs);
    bool checkCylinderCollision(const Vector3& p1, const Vector3& p2, const Obstacle& obs);
};

class MPCPlanner {
public:
    MPCPlanner(const PlannerConfig& config = PlannerConfig());
    
    // MPC-based trajectory optimization
    Trajectory optimize(const Vector3& current_pos, const Vector3& current_vel,
                        const std::vector<Vector3>& reference_path,
                        const std::vector<Obstacle>& obstacles);
    
    // Update reference path
    void setReferencePath(const std::vector<Vector3>& path);
    
    // Get optimized control inputs
    std::vector<Vector3> getControlInputs() const { return _control_inputs; }
    
private:
    PlannerConfig _config;
    std::vector<Vector3> _reference_path;
    std::vector<Vector3> _control_inputs;
    std::vector<Vector3> _predicted_states;
    
    // MPC optimization
    void solveMPC(const Vector3& current_pos, const Vector3& current_vel,
                  const std::vector<Obstacle>& obstacles);
    
    // Cost function components
    float pathFollowingCost(const std::vector<Vector3>& predicted_states);
    float obstacleAvoidanceCost(const std::vector<Vector3>& predicted_states,
                                const std::vector<Obstacle>& obstacles);
    float controlEffortCost(const std::vector<Vector3>& control_inputs);
    float smoothnessCost(const std::vector<Vector3>& predicted_states);
    
    // Constraints
    bool checkConstraints(const std::vector<Vector3>& states, const std::vector<Vector3>& controls,
                          const std::vector<Obstacle>& obstacles);
};

class TrajectoryPlanner {
public:
    TrajectoryPlanner(const PlannerConfig& config = PlannerConfig());
    
    // Main planning interface
    Trajectory planMission(const std::vector<Waypoint>& waypoints,
                           const std::vector<Obstacle>& known_obstacles);
    
    // Replan from current state
    Trajectory replan(const Vector3& current_pos, const Vector3& current_vel,
                      const std::vector<Obstacle>& current_obstacles);
    
    // Local replanning for dynamic obstacles
    Trajectory localReplan(const Vector3& current_pos, const Vector3& current_vel,
                           const std::vector<Obstacle>& dynamic_obstacles,
                           const Trajectory& current_trajectory);
    
    // Emergency replan (fast, conservative)
    Trajectory emergencyReplan(const Vector3& current_pos, const Vector3& current_vel,
                               const std::vector<Obstacle>& obstacles);
    
    // Get current trajectory
    const Trajectory& getCurrentTrajectory() const { return _current_trajectory; }
    
    // Update dynamic obstacles
    void updateObstacles(const std::vector<Obstacle>& obstacles);
    
    // Config
    void setConfig(const PlannerConfig& config);
    const PlannerConfig& getConfig() const { return _config; }
    
private:
    PlannerConfig _config;
    RRTStarPlanner _rrt_planner;
    MPCPlanner _mpc_planner;
    Trajectory _current_trajectory;
    std::vector<Obstacle> _known_obstacles;
    size_t _current_waypoint_index = 0;
    
    // Helper functions
    Trajectory interpolatePath(const std::vector<Vector3>& waypoints, float dt);
    Trajectory smoothTrajectory(const Trajectory& traj);
    bool validateTrajectory(const Trajectory& traj, const std::vector<Obstacle>& obstacles);
    Trajectory mergeTrajectories(const Trajectory& global, const Trajectory& local);
};

// Formation-aware trajectory planner for swarm
class FormationTrajectoryPlanner {
public:
    FormationTrajectoryPlanner(const PlannerConfig& config = PlannerConfig());
    
    // Plan trajectory for leader considering followers
    Trajectory planFormationTrajectory(const std::vector<Waypoint>& waypoints,
                                       const std::vector<Obstacle>& obstacles,
                                       const std::vector<formation::FormationSlot>& follower_slots);
    
    // Replan for leader with formation constraints
    Trajectory replanFormation(const Vector3& leader_pos, const Vector3& leader_vel,
                               const std::vector<Obstacle>& obstacles,
                               const std::vector<formation::FormationSlot>& follower_slots);
    
    // Get follower trajectories
    std::vector<Trajectory> getFollowerTrajectories() const;
    
private:
    PlannerConfig _config;
    TrajectoryPlanner _leader_planner;
    std::vector<Trajectory> _follower_trajectories;
    std::vector<formation::FormationSlot> _follower_slots;
    
    // Formation constraint: followers must stay within tolerance
    void applyFormationConstraints(Trajectory& leader_traj, 
                                   std::vector<Trajectory>& follower_trajs);
};

} // namespace swarm::planner

#endif // TRAJECTORY_PLANNER_HPP