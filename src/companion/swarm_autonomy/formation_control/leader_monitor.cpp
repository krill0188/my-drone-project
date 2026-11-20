#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/msg/formation_state.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <cmath>
#include <unordered_map>

namespace swarm::autonomy {

class LeaderMonitorNode : public rclcpp::Node {
public:
    LeaderMonitorNode() : Node("leader_monitor") {
        // Parameters
        _drone_count = declare_parameter("drone_count", 4);
        _leader_id = declare_parameter("leader_id", 1);
        _update_rate_hz = declare_parameter("update_rate_hz", 10.0);
        _visualize = declare_parameter("visualize", true);
        
        // QoS for formation state
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
        
        // Subscribers
        _formation_state_sub = create_subscription<swarm_msgs::msg::FormationState>(
            "/swarm/formation_state", qos,
            std::bind(&LeaderMonitorNode::formationStateCallback, this, std::placeholders::_1));
        
        _agent_states_sub = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", qos,
            std::bind(&LeaderMonitorNode::agentStatesCallback, this, std::placeholders::_1));
        
        // Publishers for visualization
        if (_visualize) {
            _formation_markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
                "/swarm/formation_markers", 10);
            _trajectory_markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
                "/swarm/trajectory_markers", 10);
            _error_markers_pub = create_publisher<visualization_msgs::msg::MarkerArray>(
                "/swarm/error_markers", 10);
        }
        
        // Timer for visualization updates
        _viz_timer = create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LeaderMonitorNode::publishVisualization, this));
        
        // Statistics
        _stats_timer = create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&LeaderMonitorNode::logStatistics, this));
        
        RCLCPP_INFO(get_logger(), "Leader Monitor Node initialized (drones=%d, leader=%d)", 
                   _drone_count, _leader_id);
    }

private:
    int _drone_count;
    int _leader_id;
    double _update_rate_hz;
    bool _visualize;
    
    rclcpp::Subscription<swarm_msgs::msg::FormationState>::SharedPtr _formation_state_sub;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr _agent_states_sub;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _formation_markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _trajectory_markers_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr _error_markers_pub;
    
    rclcpp::TimerBase::SharedPtr _viz_timer;
    rclcpp::TimerBase::SharedPtr _stats_timer;
    
    // State storage
    swarm_msgs::msg::FormationState _last_formation_state;
    std::unordered_map<uint16_t, swarm_msgs::msg::AgentState> _agent_states;
    std::vector<geometry_msgs::msg::Point> _leader_trajectory;
    bool _formation_state_valid = false;
    
    // Statistics
    uint64_t _messages_received = 0;
    uint64_t _formation_updates = 0;
    double _avg_lateral_error = 0.0;
    double _avg_vertical_error = 0.0;
    int _samples_count = 0;
    
    void formationStateCallback(const swarm_msgs::msg::FormationState::SharedPtr msg) {
        _last_formation_state = *msg;
        _formation_state_valid = true;
        _formation_updates++;
        _messages_received++;
        
        // Compute error statistics
        double lat_sum = 0.0, vert_sum = 0.0;
        int count = 0;
        for (size_t i = 0; i < msg->position_errors_flu.size(); ++i) {
            const auto& err = msg->position_errors_flu[i];
            double lateral = std::sqrt(err.x * err.x + err.y * err.y);
            double vertical = std::abs(err.z);
            lat_sum += lateral;
            vert_sum += vertical;
            count++;
        }
        if (count > 0) {
            _avg_lateral_error = 0.9 * _avg_lateral_error + 0.1 * (lat_sum / count);
            _avg_vertical_error = 0.9 * _avg_vertical_error + 0.1 * (vert_sum / count);
            _samples_count++;
        }
    }
    
    void agentStatesCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        _agent_states[msg->sysid] = *msg;
        _messages_received++;
    }
    
    void publishVisualization() {
        if (!_visualize || !_formation_state_valid) return;
        
        // Formation geometry markers
        auto formation_markers = createFormationMarkers();
        _formation_markers_pub->publish(formation_markers);
        
        // Leader trajectory
        auto trajectory_markers = createTrajectoryMarkers();
        _trajectory_markers_pub->publish(trajectory_markers);
        
        // Position error markers
        auto error_markers = createErrorMarkers();
        _error_markers_pub->publish(error_markers);
    }
    
    visualization_msgs::msg::MarkerArray createFormationMarkers() {
        visualization_msgs::msg::MarkerArray array;
        
        // Leader marker
        visualization_msgs::msg::Marker leader_marker;
        leader_marker.header.frame_id = "map";
        leader_marker.header.stamp = now();
        leader_marker.ns = "formation";
        leader_marker.id = 0;
        leader_marker.type = visualization_msgs::msg::Marker::SPHERE;
        leader_marker.action = visualization_msgs::msg::Marker::ADD;
        leader_marker.pose.position.x = _last_formation_state.leader_position.x;
        leader_marker.pose.position.y = _last_formation_state.leader_position.y;
        leader_marker.pose.position.z = _last_formation_state.leader_position.z;
        leader_marker.pose.orientation.w = 1.0;
        leader_marker.scale.x = 1.0;
        leader_marker.scale.y = 1.0;
        leader_marker.scale.z = 1.0;
        leader_marker.color.r = 1.0;
        leader_marker.color.g = 0.0;
        leader_marker.color.b = 0.0;
        leader_marker.color.a = 1.0;
        array.markers.push_back(leader_marker);
        
        // Formation geometry lines
        visualization_msgs::msg::Marker geometry_lines;
        geometry_lines.header.frame_id = "map";
        geometry_lines.header.stamp = now();
        geometry_lines.ns = "formation_geometry";
        geometry_lines.id = 1;
        geometry_lines.type = visualization_msgs::msg::Marker::LINE_LIST;
        geometry_lines.action = visualization_msgs::msg::Marker::ADD;
        geometry_lines.scale.x = 0.1;
        geometry_lines.color.r = 0.0;
        geometry_lines.color.g = 1.0;
        geometry_lines.color.b = 0.0;
        geometry_lines.color.a = 0.8;
        
        // Add lines from leader to each follower
        for (size_t i = 0; i < _last_formation_state.follower_ids.size(); ++i) {
            geometry_msgs::msg::Point p1;
            p1.x = _last_formation_state.leader_position.x;
            p1.y = _last_formation_state.leader_position.y;
            p1.z = _last_formation_state.leader_position.z;
            
            geometry_msgs::msg::Point p2;
            p2.x = _last_formation_state.leader_position.x + _last_formation_state.position_errors_flu[i].x;
            p2.y = _last_formation_state.leader_position.y + _last_formation_state.position_errors_flu[i].y;
            p2.z = _last_formation_state.leader_position.z + _last_formation_state.position_errors_flu[i].z;
            
            geometry_lines.points.push_back(p1);
            geometry_lines.points.push_back(p2);
        }
        array.markers.push_back(geometry_lines);
        
        // Follower spheres
        for (size_t i = 0; i < _last_formation_state.follower_ids.size(); ++i) {
            visualization_msgs::msg::Marker follower_marker;
            follower_marker.header.frame_id = "map";
            follower_marker.header.stamp = now();
            follower_marker.ns = "followers";
            follower_marker.id = 10 + i;
            follower_marker.type = visualization_msgs::msg::Marker::SPHERE;
            follower_marker.action = visualization_msgs::msg::Marker::ADD;
            
            follower_marker.pose.position.x = _last_formation_state.leader_position.x + _last_formation_state.position_errors_flu[i].x;
            follower_marker.pose.position.y = _last_formation_state.leader_position.y + _last_formation_state.position_errors_flu[i].y;
            follower_marker.pose.position.z = _last_formation_state.leader_position.z + _last_formation_state.position_errors_flu[i].z;
            follower_marker.pose.orientation.w = 1.0;
            follower_marker.scale.x = 0.5;
            follower_marker.scale.y = 0.5;
            follower_marker.scale.z = 0.5;
            
            // Color based on status
            switch (_last_formation_state.follower_statuses[i]) {
                case 0: // FORMING
                    follower_marker.color.r = 1.0; follower_marker.color.g = 1.0; follower_marker.color.b = 0.0; break;
                case 1: // HOLDING
                    follower_marker.color.r = 0.0; follower_marker.color.g = 1.0; follower_marker.color.b = 0.0; break;
                case 2: // REJOINING
                    follower_marker.color.r = 1.0; follower_marker.color.g = 0.5; follower_marker.color.b = 0.0; break;
                case 3: // LOST
                    follower_marker.color.r = 1.0; follower_marker.color.g = 0.0; follower_marker.color.b = 0.0; break;
                default:
                    follower_marker.color.r = 0.5; follower_marker.color.g = 0.5; follower_marker.color.b = 0.5;
            }
            follower_marker.color.a = 1.0;
            array.markers.push_back(follower_marker);
        }
        
        return array;
    }
    
    visualization_msgs::msg::MarkerArray createTrajectoryMarkers() {
        visualization_msgs::msg::MarkerArray array;
        
        // Update leader trajectory
        if (_formation_state_valid) {
            geometry_msgs::msg::Point pt;
            pt.x = _last_formation_state.leader_position.x;
            pt.y = _last_formation_state.leader_position.y;
            pt.z = _last_formation_state.leader_position.z;
            _leader_trajectory.push_back(pt);
            
            // Keep last 1000 points
            if (_leader_trajectory.size() > 1000) {
                _leader_trajectory.erase(_leader_trajectory.begin());
            }
        }
        
        visualization_msgs::msg::Marker traj_marker;
        traj_marker.header.frame_id = "map";
        traj_marker.header.stamp = now();
        traj_marker.ns = "trajectory";
        traj_marker.id = 0;
        traj_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        traj_marker.action = visualization_msgs::msg::Marker::ADD;
        traj_marker.scale.x = 0.05;
        traj_marker.color.r = 0.0;
        traj_marker.color.g = 0.0;
        traj_marker.color.b = 1.0;
        traj_marker.color.a = 0.8;
        traj_marker.points = _leader_trajectory;
        
        array.markers.push_back(traj_marker);
        return array;
    }
    
    visualization_msgs::msg::MarkerArray createErrorMarkers() {
        visualization_msgs::msg::MarkerArray array;
        
        if (!_formation_state_valid) return array;
        
        for (size_t i = 0; i < _last_formation_state.follower_ids.size(); ++i) {
            visualization_msgs::msg::Marker error_marker;
            error_marker.header.frame_id = "map";
            error_marker.header.stamp = now();
            error_marker.ns = "position_error";
            error_marker.id = 100 + i;
            error_marker.type = visualization_msgs::msg::Marker::ARROW;
            error_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Arrow from target position to actual position
            geometry_msgs::msg::Point start;
            start.x = _last_formation_state.leader_position.x + _last_formation_state.relative_positions[i].x;
            start.y = _last_formation_state.leader_position.y + _last_formation_state.relative_positions[i].y;
            start.z = _last_formation_state.leader_position.z + _last_formation_state.relative_positions[i].z;
            
            geometry_msgs::msg::Point end;
            end.x = start.x + _last_formation_state.position_errors_flu[i].x;
            end.y = start.y + _last_formation_state.position_errors_flu[i].y;
            end.z = start.z + _last_formation_state.position_errors_flu[i].z;
            
            error_marker.points.push_back(start);
            error_marker.points.push_back(end);
            error_marker.scale.x = 0.1;
            error_marker.scale.y = 0.2;
            error_marker.scale.z = 0.0;
            
            // Color by error magnitude
            double lateral = std::sqrt(
                _last_formation_state.position_errors_flu[i].x * _last_formation_state.position_errors_flu[i].x +
                _last_formation_state.position_errors_flu[i].y * _last_formation_state.position_errors_flu[i].y);
            double vertical = std::abs(_last_formation_state.position_errors_flu[i].z);
            
            if (lateral > 0.5 || vertical > 0.3) {
                error_marker.color.r = 1.0; error_marker.color.g = 0.0; error_marker.color.b = 0.0;
            } else {
                error_marker.color.r = 0.0; error_marker.color.g = 1.0; error_marker.color.b = 0.0;
            }
            error_marker.color.a = 0.8;
            
            array.markers.push_back(error_marker);
        }
        
        return array;
    }
    
    void logStatistics() {
        RCLCPP_INFO(get_logger(), 
            "Stats: msgs=%lu, formations=%lu, avg_lat_err=%.3fm, avg_vert_err=%.3fm",
            _messages_received, _formation_updates, _avg_lateral_error, _avg_vertical_error);
    }
};

} // namespace swarm::autonomy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarm::autonomy::LeaderMonitorNode>());
    rclcpp::shutdown();
    return 0;
}