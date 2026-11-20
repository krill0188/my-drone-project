#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/srv/reconfigure_formation.hpp"
#include "swarm_msgs/msg/formation_state.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_autonomy/formation_control/formation_controller.hpp"
#include <memory>

namespace swarm::autonomy {

class ReconfigureServiceNode : public rclcpp::Node {
public:
    ReconfigureServiceNode() : Node("reconfigure_service") {
        // Parameters
        _formation_controller = std::make_shared<FormationController>();
        
        // Service for formation reconfiguration
        _reconfigure_srv = create_service<swarm_msgs::srv::ReconfigureFormation>(
            "/swarm/reconfigure_formation",
            std::bind(&ReconfigureServiceNode::reconfigureCallback, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        // Subscriber for current formation state
        _formation_state_sub = create_subscription<swarm_msgs::msg::FormationState>(
            "/swarm/formation_state", 10,
            std::bind(&ReconfigureServiceNode::formationStateCallback, this, std::placeholders::_1));
        
        // Publisher for formation commands
        _formation_cmd_pub = create_publisher<swarm_msgs::msg::FormationCommand>(
            "/swarm/formation_command", 10);
        
        RCLCPP_INFO(get_logger(), "Formation Reconfigure Service initialized");
    }

private:
    std::shared_ptr<FormationController> _formation_controller;
    
    rclcpp::Service<swarm_msgs::srv::ReconfigureFormation>::SharedPtr _reconfigure_srv;
    rclcpp::Subscription<swarm_msgs::msg::FormationState>::SharedPtr _formation_state_sub;
    rclcpp::Publisher<swarm_msgs::msg::FormationCommand>::SharedPtr _formation_cmd_pub;
    
    swarm_msgs::msg::FormationState _current_formation;
    bool _formation_valid = false;
    std::mutex _mutex;
    
    void formationStateCallback(const swarm_msgs::msg::FormationState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(_mutex);
        _current_formation = *msg;
        _formation_valid = true;
    }
    
    void reconfigureCallback(
        const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
        std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        
        RCLCPP_INFO(get_logger(), "Reconfigure request: type=%s, geometry=%d, spacing=%.1f",
                   request->command.c_str(), request->geometry_type, request->spacing);
        
        std::lock_guard<std::mutex> lock(_mutex);
        
        if (!_formation_valid) {
            response->success = false;
            response->message = "No current formation state available";
            return;
        }
        
        try {
            if (request->command == "change_geometry") {
                response->success = changeGeometry(request, response);
            } else if (request->command == "add_follower") {
                response->success = addFollower(request, response);
            } else if (request->command == "remove_follower") {
                response->success = removeFollower(request, response);
            } else if (request->command == "change_spacing") {
                response->success = changeSpacing(request, response);
            } else if (request->command == "change_altitude") {
                response->success = changeAltitude(request, response);
            } else if (request->command == "emergency_dissolve") {
                response->success = emergencyDissolve(response);
            } else if (request->command == "reform") {
                response->success = reformFormation(request, response);
            } else {
                response->success = false;
                response->message = "Unknown command: " + request->command;
            }
        } catch (const std::exception& e) {
            response->success = false;
            response->message = "Exception: " + std::string(e.what());
            RCLCPP_ERROR(get_logger(), "Reconfigure failed: %s", e.what());
        }
    }
    
    bool changeGeometry(const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
                       std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        // Validate geometry type
        if (request->geometry_type < 0 || request->geometry_type > 4) {
            response->message = "Invalid geometry type (0-4)";
            return false;
        }
        
        // Create formation command
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::CHANGE_GEOMETRY;
        cmd.geometry_type = request->geometry_type;
        cmd.spacing = request->spacing > 0 ? request->spacing : _current_formation.spacing_m;
        cmd.altitude_offset = _current_formation.altitude_offset_m;
        cmd.sequence = _current_formation.sequence + 1;
        
        // Publish command
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Geometry change commanded: type=" + std::to_string(request->geometry_type);
        return true;
    }
    
    bool addFollower(const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
                    std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        if (request->follower_id == 0) {
            response->message = "Invalid follower ID";
            return false;
        }
        
        // Check if already in formation
        for (uint16_t id : _current_formation.follower_ids) {
            if (id == request->follower_id) {
                response->message = "Follower already in formation";
                return false;
            }
        }
        
        // Create command
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::ADD_FOLLOWER;
        cmd.follower_id = request->follower_id;
        cmd.sequence = _current_formation.sequence + 1;
        
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Add follower commanded: ID=" + std::to_string(request->follower_id);
        return true;
    }
    
    bool removeFollower(const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
                       std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        if (request->follower_id == 0) {
            response->message = "Invalid follower ID";
            return false;
        }
        
        // Check if in formation
        bool found = false;
        for (uint16_t id : _current_formation.follower_ids) {
            if (id == request->follower_id) {
                found = true;
                break;
            }
        }
        if (!found) {
            response->message = "Follower not in formation";
            return false;
        }
        
        // Create command
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::REMOVE_FOLLOWER;
        cmd.follower_id = request->follower_id;
        cmd.sequence = _current_formation.sequence + 1;
        
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Remove follower commanded: ID=" + std::to_string(request->follower_id);
        return true;
    }
    
    bool changeSpacing(const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
                      std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        if (request->spacing <= 0.5 || request->spacing > 50.0) {
            response->message = "Invalid spacing (0.5-50.0m)";
            return false;
        }
        
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::CHANGE_SPACING;
        cmd.spacing = request->spacing;
        cmd.sequence = _current_formation.sequence + 1;
        
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Spacing change commanded: " + std::to_string(request->spacing) + "m";
        return true;
    }
    
    bool changeAltitude(const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
                       std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::CHANGE_ALTITUDE;
        cmd.altitude_offset = request->altitude_offset;
        cmd.sequence = _current_formation.sequence + 1;
        
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Altitude change commanded: " + std::to_string(request->altitude_offset) + "m";
        return true;
    }
    
    bool emergencyDissolve(std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::EMERGENCY_DISSOLVE;
        cmd.sequence = _current_formation.sequence + 1;
        
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Emergency dissolve commanded";
        return true;
    }
    
    bool reformFormation(const std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Request> request,
                        std::shared_ptr<swarm_msgs::srv::ReconfigureFormation::Response> response) {
        // Reform with current geometry but reassign slots
        auto cmd = swarm_msgs::msg::FormationCommand();
        cmd.timestamp = now();
        cmd.command_type = swarm_msgs::msg::FormationCommand::REFORM;
        cmd.geometry_type = _current_formation.geometry_type;
        cmd.spacing = _current_formation.spacing_m;
        cmd.altitude_offset = _current_formation.altitude_offset_m;
        cmd.sequence = _current_formation.sequence + 1;
        
        _formation_cmd_pub->publish(cmd);
        
        response->message = "Formation reform commanded";
        return true;
    }
};

} // namespace swarm::autonomy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarm::autonomy::ReconfigureServiceNode>());
    rclcpp::shutdown();
    return 0;
}