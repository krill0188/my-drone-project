#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/msg/ai_inference.hpp"
#include "swarm_msgs/msg/formation_state.hpp"
#include "swarm_msgs/srv/upload_mission.hpp"
#include "swarm_autonomy/trajectory_planner/trajectory_planner.hpp"
#include "swarm_autonomy/formation_control/formation_controller.hpp"
#include <memory>
#include <mutex>

namespace swarm::autonomy {

class TrajectoryPlannerNode : public rclcpp::Node {
public:
    TrajectoryPlannerNode() : Node("trajectory_planner_node") {
        // Parameters
        _drone_id = declare_parameter("drone_id", 1);
        _is_leader = declare_parameter("is_leader", false);
        _planner_config.max_velocity = declare_parameter("max_velocity", 10.0);
        _planner_config.max_acceleration = declare_parameter("max_acceleration", 5.0);
        _planner_config.resolution = declare_parameter("resolution", 0.5);
        _planner_config.safety_margin = declare_parameter("safety_margin", 2.0);
        _planner_config.mpc_horizon = declare_parameter("mpc_horizon", 20);
        _planner_config.mpc_dt = declare_parameter("mpc_dt", 0.1);
        
        // Initialize planner
        _planner = std::make_unique<TrajectoryPlanner>(_planner_config);
        
        // If leader, also initialize formation planner
        if (_is_leader) {
            _formation_planner = std::make_unique<FormationTrajectoryPlanner>(_planner_config);
        }
        
        // Subscribers
        _agent_state_sub = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", 10,
            std::bind(&TrajectoryPlannerNode::agentStateCallback, this, std::placeholders::_1));
        
        _ai_inference_sub = create_subscription<swarm_msgs::msg::AIInference>(
            "/swarm/ai_inference", 10,
            std::bind(&TrajectoryPlannerNode::aiInferenceCallback, this, std::placeholders::_1));
        
        _formation_state_sub = create_subscription<swarm_msgs::msg::FormationState>(
            "/swarm/formation_state", 10,
            std::bind(&TrajectoryPlannerNode::formationStateCallback, this, std::placeholders::_1));
        
        // Service for mission upload
        _mission_upload_srv = create_service<swarm_msgs::srv::UploadMission>(
            "/swarm/upload_mission",
            std::bind(&TrajectoryPlannerNode::missionUploadCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Publishers
        _trajectory_pub = create_publisher<swarm_msgs::msg::Trajectory>(
            "/drone" + std::to_string(_drone_id) + "/trajectory", 10);
        
        _replan_trigger_pub = create_publisher<std_msgs::msg::Bool>(
            "/swarm/replan_trigger", 10);
        
        // Timer for trajectory publishing
        _publish_timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrajectoryPlannerNode::publishTrajectory, this));
        
        RCLCPP_INFO(get_logger(), "Trajectory Planner Node initialized (drone_id=%d, is_leader=%s)",
                   _drone_id, _is_leader ? "true" : "false");
    }

private:
    int _drone_id;
    bool _is_leader;
    PlannerConfig _planner_config;
    
    std::unique_ptr<TrajectoryPlanner> _planner;
    std::unique_ptr<FormationTrajectoryPlanner> _formation_planner;
    
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr _agent_state_sub;
    rclcpp::Subscription<swarm_msgs::msg::AIInference>::SharedPtr _ai_inference_sub;
    rclcpp::Subscription<swarm_msgs::msg::FormationState>::SharedPtr _formation_state_sub;
    
    rclcpp::Service<swarm_msgs::srv::UploadMission>::SharedPtr _mission_upload_srv;
    rclcpp::Publisher<swarm_msgs::msg::Trajectory>::SharedPtr _trajectory_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _replan_trigger_pub;
    rclcpp::TimerBase::SharedPtr _publish_timer;
    
    // State
    swarm_msgs::msg::AgentState _own_state;
    bool _own_state_valid = false;
    std::vector<PlannerConfig::Obstacle> _dynamic_obstacles;
    std::mutex _obstacles_mutex;
    Trajectory _current_trajectory;
    bool _trajectory_valid = false;
    
    void agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        if (msg->sysid == _drone_id) {
            std::lock_guard<std::mutex> lock(_obstacles_mutex);
            _own_state = *msg;
            _own_state_valid = true;
        }
    }
    
    void aiInferenceCallback(const swarm_msgs::msg::AIInference::SharedPtr msg) {
        // Convert AI inference to dynamic obstacles
        if (msg->type == 0) {  // OBSTACLE
            std::lock_guard<std::mutex> lock(_obstacles_mutex);
            
            PlannerConfig::Obstacle obs;
            obs.center = {msg->position_enu.x, msg->position_enu.y, msg->position_enu.z};
            obs.size = {msg->size.x, msg->size.y, msg->size.z};
            obs.confidence = msg->confidence;
            obs.timestamp_us = msg->timestamp_us;
            
            // Determine obstacle type from class
            if (msg->class_id == 1) {  // POWER_LINE
                obs.type = PlannerConfig::Obstacle::CYLINDER;
            } else if (msg->class_id >= 2 && msg->class_id <= 3) {  // TREE, VEGETATION
                obs.type = PlannerConfig::Obstacle::CYLINDER;
            } else {
                obs.type = PlannerConfig::Obstacle::SPHERE;
            }
            
            // Update or add obstacle
            bool found = false;
            for (auto& o : _dynamic_obstacles) {
                Vector3 diff = {o.center.x - msg->position_enu.x,
                               o.center.y - msg->position_enu.y,
                               o.center.z - msg->position_enu.z};
                float dist = std::sqrt(diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);
                if (dist < 5.0f) {  // Same obstacle
                    o = {
                        .center = {msg->position_enu.x, msg->position_enu.y, msg->position_enu.z},
                        .size = {msg->size.x, msg->size.y, msg->size.z},
                        .type = (msg->class_id == 1) ? Obstacle::CYLINDER : Obstacle::SPHERE,
                        .confidence = msg->confidence,
                        .timestamp_us = msg->timestamp_us
                    };
                    found = true;
                    break;
                }
            }
            
            if (!found) {
                PlannerConfig::Obstacle obs;
                obs.center = {msg->position_enu.x, msg->position_enu.y, msg->position_enu.z};
                obs.size = {msg->size.x, msg->size.y, msg->size.z};
                obs.type = (msg->class_id == 1) ? Obstacle::CYLINDER : Obstacle::SPHERE;
                obs.confidence = msg->confidence;
                obs.timestamp_us = msg->timestamp_us;
                _dynamic_obstacles.push_back(obs);
            }
        }
    }
    
    void formationStateCallback(const swarm_msgs::msg::FormationState::SharedPtr msg) {
        // Update formation state for leader
        if (_is_leader) {
            // Update follower slots for formation planner
        }
    }
    
    void missionUploadCallback(
        const std::shared_ptr<swarm_msgs::srv::UploadMission::Request> request,
        std::shared_ptr<swarm_msgs::srv::UploadMission::Response> response) {
        
        RCLCPP_INFO(get_logger(), "Mission upload received: %zu waypoints", request->waypoints.size());
        
        // Convert waypoints
        std::vector<PlannerConfig::Waypoint> waypoints;
        for (const auto& wp : request->waypoints) {
            PlannerConfig::Waypoint wp_config;
            wp_config.position = {wp.position.x, wp.position.y, wp.position.z};
            wp_config.speed = wp.speed;
            wp_config.loiter_time = wp.loiter_time;
            wp_config.acceptance_radius = wp.acceptance_radius;
            waypoints.push_back(wp_config);
        }
        
        // Convert known obstacles from mission
        std::vector<PlannerConfig::Obstacle> known_obstacles;
        for (const auto& obs : request->obstacles) {
            PlannerConfig::Obstacle obs_config;
            obs_config.center = {obs.position.x, obs.position.y, obs.position.z};
            obs_config.size = {obs.size.x, obs.size.y, obs.size.z};
            obs_config.type = static_cast<Obstacle::Type>(obs.type);
            known_obstacles.push_back(obs_config);
        }
        
        // Plan mission
        if (_is_leader) {
            // Get formation slots from formation state
            std::vector<FormationSlot> slots;
            // Would extract from formation state
            _current_trajectory = _formation_planner->planFormationTrajectory(waypoints, {}, {});
        } else {
            _current_trajectory = _planner->planMission(waypoints, known_obstacles);
        }
        
        if (_current_trajectory.valid) {
            response->success = true;
            response->message = "Mission planned successfully";
            _trajectory_valid = true;
        } else {
            response->success = false;
            response->message = "Mission planning failed";
        }
    }
    
    void publishTrajectory() {
        if (!_trajectory_valid || !_own_state_valid) return;
        
        // Check if replan needed (obstacle detected)
        {
            std::lock_guard<std::mutex> lock(_obstacles_mutex);
            if (!_dynamic_obstacles.empty()) {
                // Trigger replan
                auto replan_msg = std_msgs::msg::Bool();
                replan_msg.data = true;
                _replan_trigger_pub->publish(replan_msg);
                
                // Replan
                Vector3 current_pos = {_own_state.position.x, _own_state.position.y, _own_state.position.z};
                Vector3 current_vel = {_own_state.velocity.x, _own_state.velocity.y, _own_state.velocity.z};
                
                std::vector<Obstacle> all_obstacles;
                // Combine known and dynamic obstacles
                
                Trajectory new_traj = _planner->localReplan(
                    {_own_state.position.x, _own_state.position.y, _own_state.position.z},
                    {_own_state.velocity.x, _own_state.velocity.y, _own_state.velocity.z},
                    _dynamic_obstacles,
                    _current_trajectory
                );
                
                if (new_traj.valid) {
                    _current_trajectory = new_traj;
                }
            }
        }
        
        // Publish current trajectory
        // Would publish trajectory points
    }
};

} // namespace swarm::autonomy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarm::autonomy::TrajectoryPlannerNode>());
    rclcpp::shutdown();
    return 0;
}