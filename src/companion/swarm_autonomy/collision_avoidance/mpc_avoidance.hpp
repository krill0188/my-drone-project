#ifndef MPC_AVOIDANCE_HPP
#define MPC_AVOIDANCE_HPP

#include "swarm_formation.hpp"
#include "swarm_map.hpp"
#include <vector>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>

namespace swarm::autonomy {

// MPC-based collision avoidance for swarm
class MPCAvoidance {
public:
    struct Config {
        int horizon = 20;                   // Prediction horizon
        float dt = 0.1f;                    // Time step
        float max_speed = 15.0f;            // Max speed
        float max_accel = 5.0f;             // Max acceleration
        float safety_distance = 5.0f;       // Minimum separation
        float obstacle_margin = 2.0f;       // Obstacle margin
        
        // Cost weights
        float w_position = 10.0f;           // Position tracking
        float w_velocity = 1.0f;            // Velocity smoothness
        float w_acceleration = 0.1f;        // Acceleration minimization
        float w_collision = 100.0f;         // Collision avoidance
        float w_obstacle = 50.0f;           // Obstacle avoidance
    };
    
    struct AgentState {
        Vector3 position;
        Vector3 velocity;
        Vector3 acceleration;
        float radius;
    };
    
    struct Obstacle {
        Vector3 position;
        float radius;
    };
    
    struct Neighbor {
        uint16_t id;
        Vector3 position;
        Vector3 velocity;
        float radius;
    };
    
    struct MPCSolution {
        std::vector<Vector3> positions;     // Predicted positions
        std::vector<Vector3> velocities;    // Predicted velocities
        std::vector<Vector3> accelerations; // Control inputs
        float cost;
        bool feasible;
        int iterations;
    };
    
    MPCAvoidance(const Config& config = Config{});
    
    // Solve MPC for collision-free trajectory
    MPCSolution solve(const AgentState& ego,
                      const std::vector<Neighbor>& neighbors,
                      const std::vector<Obstacle>& obstacles,
                      const Vector3& goal_position,
                      const Vector3& goal_velocity = {0, 0, 0});
    
    // Solve with reference trajectory
    MPCSolution solveWithReference(const AgentState& ego,
                                   const std::vector<Vector3>& ref_positions,
                                   const std::vector<Vector3>& ref_velocities,
                                   const std::vector<Neighbor>& neighbors,
                                   const std::vector<Obstacle>& obstacles);
    
    // Update dynamic obstacles
    void updateObstacles(const std::vector<Obstacle>& obstacles);
    
    // Update neighbor predictions
    void updateNeighbors(const std::vector<Neighbor>& neighbors);
    
    // Emergency stop trajectory
    MPCSolution emergencyStop(const AgentState& ego, 
                              const std::vector<Neighbor>& neighbors,
                              const std::vector<Obstacle>& obstacles);
    
    // Get current solution
    const MPCSolution& getLastSolution() const { return _last_solution; }
    
    // Set config
    void setConfig(const Config& config) { _config = config; }
    const Config& getConfig() const { return _config; }

private:
    Config _config;
    MPCSolution _last_solution;
    
    // QP matrices
    Eigen::SparseMatrix<double> _H;        // Hessian
    Eigen::VectorXd _g;                    // Gradient
    Eigen::SparseMatrix<double> _A;        // Constraint matrix
    Eigen::VectorXd _lb, _ub;              // Bounds
    
    // Build QP matrices
    void buildQP(const AgentState& ego,
                 const std::vector<Vector3>& ref_positions,
                 const std::vector<Vector3>& ref_velocities,
                 const std::vector<Neighbor>& neighbors,
                 const std::vector<Obstacle>& obstacles);
    
    // Build cost function
    void buildCost(const AgentState& ego,
                   const std::vector<Vector3>& ref_positions,
                   const std::vector<Vector3>& ref_velocities);
    
    // Build collision avoidance constraints
    void buildCollisionConstraints(const AgentState& ego,
                                   const std::vector<Neighbor>& neighbors,
                                   const std::vector<Obstacle>& obstacles);
    
    // Build dynamics constraints
    void buildDynamicsConstraints(const AgentState& ego);
    
    // Build bounds
    void buildBounds(const AgentState& ego);
    
    // Solve QP using OSQP or custom solver
    bool solveQP(Eigen::VectorXd& solution);
    
    // Extract solution
    MPCSolution extractSolution(const Eigen::VectorXd& solution, 
                                const AgentState& ego);
    
    // Collision checking
    bool checkCollision(const Vector3& pos1, float r1, const Vector3& pos2, float r2) const;
    float distanceToObstacle(const Vector3& pos, const Obstacle& obst) const;
    
    // Soft constraint for collision (barrier function)
    double barrierCost(double distance, double safety_dist) const;
    
    // Predict neighbor trajectory (constant velocity)
    std::vector<Vector3> predictNeighborTrajectory(const Neighbor& neighbor) const;
    
    // Static obstacle prediction (stationary)
    std::vector<Vector3> predictObstacleTrajectory(const Obstacle& obst) const;
};

} // namespace swarm::autonomy

#endif // MPC_AVOIDANCE_HPP