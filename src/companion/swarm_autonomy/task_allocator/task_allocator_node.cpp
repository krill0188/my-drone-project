#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/msg/task_allocation.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/msg/swarm_map.hpp"
#include "swarm_autonomy/task_allocator/task_allocator.hpp"
#include "swarm_autonomy/task_allocator/cbba.hpp"

namespace swarm::autonomy {

class TaskAllocatorNode : public rclcpp::Node {
public:
    TaskAllocatorNode() : Node("task_allocator_node") {
        // Parameters
        _drone_id = declare_parameter("drone_id", 1);
        _update_rate_hz = declare_parameter("update_rate_hz", 1.0);
        
        // Load capabilities
        _capabilities.has_rtk_gps = declare_parameter("capabilities.has_rtk_gps", true);
        _capabilities.has_stereo_camera = declare_parameter("capabilities.has_stereo_camera", true);
        _capabilities.has_lidar_2d = declare_parameter("capabilities.has_lidar_2d", false);
        _capabilities.has_lidar_3d = declare_parameter("capabilities.has_lidar_3d", false);
        _capabilities.has_uwb = declare_parameter("capabilities.has_uwb", true);
        _capabilities.has_5g_modem = declare_parameter("capabilities.has_5g_modem", false);
        _capabilities.has_ai_compute = declare_parameter("capabilities.has_ai_compute", true);
        _capabilities.max_flight_time_s = declare_parameter("capabilities.max_flight_time_s", 1800);
        _capabilities.max_payload_kg = declare_parameter("capabilities.max_payload_kg", 0.5);
        _capabilities.comm_tier_bitmap = declare_parameter("capabilities.comm_tier_bitmap", 0b111);
        
        // Initialize CBBA allocator
        _allocator = std::make_unique<CbbaAllocator>(_drone_id, _capabilities);
        _allocator->init();
        
        // Publishers
        _task_bid_pub = create_publisher<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_bid", 10);
        
        _task_award_pub = create_publisher<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_award", 10);
        
        // Subscribers
        _agent_states_sub = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", 10,
            std::bind(&TaskAllocatorNode::agentStatesCallback, this, std::placeholders::_1));
        
        _task_bid_sub = create_subscription<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_bid", 10,
            std::bind(&TaskAllocatorNode::taskBidCallback, this, std::placeholders::_1));
        
        _swarm_map_sub = create_subscription<swarm_msgs::msg::SwarmMap>(
            "/swarm/swarm_map", 10,
            std::bind(&TaskAllocatorNode::swarmMapCallback, this, std::placeholders::_1));
        
        // Timer for CBBA auction loop
        _timer = create_wall_timer(
            std::chrono::duration<double>(1.0 / _update_rate_hz),
            std::bind(&TaskAllocatorNode::auctionLoop, this));
        
        RCLCPP_INFO(get_logger(), "Task Allocator Node initialized (drone_id=%d)", _drone_id);
    }

private:
    uint16_t _drone_id;
    double _update_rate_hz;
    CbbaAllocator::CapabilityBitmap _capabilities;
    
    std::unique_ptr<CbbaAllocator> _allocator;
    
    rclcpp::Publisher<swarm_msgs::msg::TaskAllocation>::SharedPtr _task_bid_pub;
    rclcpp::Publisher<swarm_msgs::msg::TaskAllocation>::SharedPtr _task_award_pub;
    
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr _agent_states_sub;
    rclcpp::Subscription<swarm_msgs::msg::TaskAllocation>::SharedPtr _task_bid_sub;
    rclcpp::Subscription<swarm_msgs::msg::SwarmMap>::SharedPtr _swarm_map_sub;
    
    rclcpp::TimerBase::SharedPtr _timer;
    
    std::vector<CbbaAllocator::Task> _available_tasks;
    std::unordered_map<uint16_t, swarm_msgs::msg::AgentState> _agent_states;
    swarm_msgs::msg::SwarmMap _swarm_map;
    bool _map_valid = false;
    
    void agentStatesCallback(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        _agent_states[msg->sysid] = *msg;
    }
    
    void taskBidCallback(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
        // Convert to CBBA bid and process
        CbbaAllocator::Bid bid;
        bid.drone_id = msg->drone_id;
        bid.task_id = std::array<uint8_t, 16>();
        std::copy(msg->task_id.begin(), msg->task_id.end(), bid.task_id.begin());
        bid.cost = msg->cost;
        bid.capabilities = msg->capabilities;
        bid.sequence = msg->sequence;
        
        _allocator->processBid(bid);
    }
    
    void swarmMapCallback(const swarm_msgs::msg::SwarmMap::SharedPtr msg) {
        _swarm_map = *msg;
        _map_valid = true;
    }
    
    void auctionLoop() {
        if (!_map_valid || _available_tasks.empty()) return;
        
        // Build bundle
        _allocator->buildBundle(_available_tasks);
        
        // Communicate with neighbors (via mesh)
        _allocator->communicateBundles();
        
        // Check convergence
        if (_allocator->checkConvergence()) {
            // Publish awards for won tasks
            const auto& allocated = _allocator->getAllocatedTasks();
            for (const auto& task_id : allocated) {
                auto msg = swarm_msgs::msg::TaskAllocation();
                msg.timestamp_us = this->now().nanoseconds() / 1000;
                msg.drone_id = _drone_id;
                msg.task_id = std::vector<uint8_t>(task_id.begin(), task_id.end());
                msg.cost = 0.0f;  // Would be filled from bundle
                _task_award_pub->publish(msg);
            }
        }
        
        // Publish bids for current bundle
        // In real implementation, would iterate over bundle tasks
    }
};

} // namespace swarm::autonomy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<swarm::autonomy::TaskAllocatorNode>());
    rclcpp::shutdown();
    return 0;
}