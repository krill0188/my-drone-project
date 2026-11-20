#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/msg/formation_state.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/srv/upload_mission.hpp"
#include "swarm_autonomy/formation_control/formation_controller.hpp"

namespace swarm::autonomy {

class FormationStateNode : public rclcpp::Node {
public:
    FormationStateNode() : Node("formation_state_node") {
        // Parameters
        _drone_count = declare_parameter("drone_count", 4);
        _leader_id = declare_parameter("leader_id", 0);
        _formation_type = declare_parameter("formation_type", "vee");
        _spacing = declare_parameter("spacing", 5.0);
        
        // Publishers
        _formation_state_pub = create_publisher<swarm_msgs::msg::FormationState>(
            "/swarm/formation_state", 10);
        
        // Subscribers
        _agent_state_sub = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", 10,
            std::bind(&FormationStateNode::agentStateCallback, this, std::placeholders::_1));
        
        _leader_state_sub = create_subscription<swarm_msgs::msg::AgentState>(
            "/drone0/state", 10,
            std::bind(&FormationStateNode::leaderStateCallback, this, std::placeholders::_1));
        
        // Service for reconfiguration
        _reconfigure_srv = create_service<swarm_msgs::srv::ReconfigureFormation>(
            "/swarm/reconfigure_formation",
            std::bind(&FormationStateNode::reconfigureCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Timer for publishing formation state
        _timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&FormationStateNode::publishFormationState, this));
        
        RCLCPP_INFO(get_logger(), "Formation State Node initialized");
    }

private:
    int _drone_count;
    int _leader_id;
    std::string _formation_type;
    double _spacing;
    
    rclcpp::Publisher<swarm_msgs::msg::FormationState>::SharedPtr _formation_state_pub;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr _agent_state_sub;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr _leader_state_sub;
    rclcpp::Service<swarm_msgs::srv::ReconfigureFormation>::SharedPtr _reconfigure_srv;
    rclcpp::TimerBase::SharedPtr _timer;
    
    std::unordered_map<uint16_t, swarm_msgs::msg::AgentState> _agent_states;
    swarm_msgs::msg::AgentState _leader_state;
    bool _leader_state_valid = false;
    uint32_t _formation_id = 0;
    uint32_t _sequence = 0;
    
    void agentStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        _agent_states[msg->sysid] = *msg;
    }
    
    void leaderStateCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        _leader_state = *msg;
        _leader_state_valid = true;
    }
    
    void reconfigureCallback(
        const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> req,
        std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> res) {
        // Update formation parameters
        _formation_type = req->formation_type;
        _spacing = req->spacing;
        _formation_id = req->formation_id;
        
        res->success = true;
        res->message = "Formation reconfigured";
        RCLCPP_INFO(get_logger(), "Formation reconfigured: type=%s, spacing=%.1f", 
                   _formation_type.c_str(), _spacing);
    }
    
    void publishFormationState() {
        if (!_leader_state_valid) return;
        
        auto msg = swarm_msgs::msg::FormationState();
        msg.timestamp_us = this->now().nanoseconds() / 1000;
        msg.formation_id = _formation_id;
        msg.leader_id = _leader_id;
        
        // Collect follower IDs
        for (const auto& [id, state] : _agent_states) {
            if (id != _leader_id && state.role == 2) {  // FOLLOWER
                msg.follower_ids.push_back(id);
            }
        }
        
        // Set geometry
        if (_formation_type == "line") msg.geometry_type = 0;
        else if (_formation_type == "vee") msg.geometry_type = 1;
        else if (_formation_type == "diamond") msg.geometry_type = 2;
        else if (_formation_type == "column") msg.geometry_type = 3;
        else msg.geometry_type = 4;
        
        msg.spacing_m = _spacing;
        msg.max_lateral_error_m = 0.5;
        msg.max_vertical_error_m = 0.3;
        msg.max_velocity_error_mps = 0.5;
        msg.max_yaw_error_deg = 5.0;
        msg.rejoin_timeout_s = 3.0;
        msg.replan_propagation_ms = 200.0;
        
        // Compute formation state for each follower
        // This is simplified - real implementation uses FormationController
        msg.follower_sysids = msg.follower_ids;
        msg.formation_state = 1;  // HOLDING
        msg.sequence = _sequence++;
        
        _formation_state_pub->publish(msg);
    }
};

} // namespace swarm::autonomy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarm::autonomy::FormationStateNode>());
    rclcpp::shutdown();
    return 0;
}