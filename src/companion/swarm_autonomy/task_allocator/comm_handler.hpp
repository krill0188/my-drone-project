#ifndef COMM_HANDLER_HPP
#define COMM_HANDLER_HPP

#include "cbba.hpp"
#include "task.hpp"
#include "mission.hpp"
#include "agent_state.hpp"
#include "swarm_comm/mesh_manager.hpp"
#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/task_allocation.hpp>
#include <swarm_msgs/srv/upload_mission.hpp>
#include <swarm_msgs/srv/request_rtl.hpp>
#include <swarm_msgs/srv/emergency_stop.hpp>
#include <swarm_msgs/srv/get_swarm_status.hpp>
#include <functional>
#include <mutex>

namespace swarm::autonomy {

// Communication handler for MAVLink/ROS 2 task allocation messages
class TaskCommHandler {
public:
    struct Config {
        float bid_publish_rate_hz = 1.0f;      // Bid publication rate
        float award_publish_rate_hz = 1.0f;    // Award publication rate
        float heartbeat_rate_hz = 10.0f;       // Heartbeat rate
        bool use_mavlink = true;               // Use MAVLink
        bool use_ros2 = true;                  // Use ROS 2
        int mavlink_stream_id = 0;             // MAVLink stream ID
    };
    
    TaskCommHandler(CbbaAllocator& allocator,
                    const std::function<std::vector<Task>()>& get_tasks,
                    const std::function<AgentState()>& get_own_state,
                    MeshManager* mesh_manager,
                    rclcpp::Node::SharedPtr ros2_node,
                    const Config& config = Config{});
    
    ~TaskCommHandler();
    
    // Start/stop communication
    void start();
    void stop();
    
    // Publish current bundle/bids
    void publishBundle();
    void publishBid(const std::array<uint8_t, 16>& task_id, float cost);
    
    // Handle incoming messages
    void handleMavlinkMessage(uint16_t src_sysid, uint16_t src_compid, 
                              const mavlink_message_t& msg);
    
    // ROS 2 service callbacks
    void handleUploadMission(const std::shared_ptr<swarm_msgs::srv::UploadMission::Request> request,
                            std::shared_ptr<swarm_msgs::srv::UploadMission::Response> response);
    
    void handleRequestRTL(const std::shared_ptr<swarm_msgs::srv::RequestRTL::Request> request,
                         std::shared_ptr<swarm_msgs::srv::RequestRTL::Response> response);
    
    void handleEmergencyStop(const std::shared_ptr<swarm_msgs::srv::EmergencyStop::Request> request,
                            std::shared_ptr<swarm_msgs::srv::EmergencyStop::Response> response);
    
    void handleGetSwarmStatus(const std::shared_ptr<swarm_msgs::srv::GetSwarmStatus::Request> request,
                             std::shared_ptr<swarm_msgs::srv::GetSwarmStatus::Response> response);

private:
    CbbaAllocator& _allocator;
    std::function<std::vector<Task>()> _get_tasks;
    std::function<AgentState()> _get_own_state;
    MeshManager* _mesh;
    rclcpp::Node::SharedPtr _ros2_node;
    Config _config;
    
    std::atomic<bool> _running{false};
    std::thread _publish_thread;
    std::thread _heartbeat_thread;
    
    // ROS 2 publishers/subscribers
    rclcpp::Publisher<swarm_msgs::msg::TaskAllocation>::SharedPtr _bid_pub;
    rclcpp::Publisher<swarm_msgs::msg::TaskAllocation>::SharedPtr _award_pub;
    rclcpp::Publisher<swarm_msgs::msg::TaskAllocation>::SharedPtr _bundle_pub;
    rclcpp::Subscription<swarm_msgs::msg::TaskAllocation>::SharedPtr _bid_sub;
    rclcpp::Subscription<swarm_msgs::msg::TaskAllocation>::SharedPtr _award_sub;
    
    rclcpp::Service<swarm_msgs::srv::UploadMission>::SharedPtr _upload_mission_srv;
    rclcpp::Service<swarm_msgs::srv::RequestRTL>::SharedPtr _rtl_srv;
    rclcpp::Service<swarm_msgs::srv::EmergencyStop>::SharedPtr _estop_srv;
    rclcpp::Service<swarm_msgs::srv::GetSwarmStatus>::SharedPtr _status_srv;
    
    // MAVLink message handling
    void handleMavlinkTaskBid(uint16_t src_sysid, const mavlink_message_t& msg);
    void handleMavlinkTaskAward(uint16_t src_sysid, const mavlink_message_t& msg);
    void handleMavlinkTaskBundle(uint16_t src_sysid, const mavlink_message_t& msg);
    
    // ROS 2 message handling
    void handleRos2Bid(const swarm_msgs::msg::TaskAllocation::SharedPtr msg);
    void handleRos2Award(const swarm_msgs::msg::TaskAllocation::SharedPtr msg);
    
    // Publishing
    void publishBundleRos2();
    void publishBidRos2(const std::array<uint8_t, 16>& task_id, float cost);
    void publishAwardRos2(const std::array<uint8_t, 16>& task_id, uint16_t drone_id, float cost);
    
    // Heartbeat
    void sendHeartbeat();
    void sendHeartbeatRos2();
    
    // Serialization helpers
    void packTaskAllocation(const CbbaAllocator::Bid& bid, 
                           swarm_msgs::msg::TaskAllocation& msg);
    void unpackTaskAllocation(const swarm_msgs::msg::TaskAllocation& msg,
                             CbbaAllocator::Bid& bid);
    
    // Timing
    uint64_t nowUs() const;
};

} // namespace swarm::autonomy

#endif // COMM_HANDLER_HPP