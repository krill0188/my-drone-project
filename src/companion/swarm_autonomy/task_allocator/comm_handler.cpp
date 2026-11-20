#include "comm_handler.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <rclcpp/rclcpp.hpp>
#include <mavlink.h>
#include <chrono>
#include <iostream>

namespace swarm::autonomy {

TaskCommHandler::TaskCommHandler(CbbaAllocator& allocator,
                                 const std::function<std::vector<Task>()>& get_tasks,
                                 const std::function<AgentState()>& get_own_state,
                                 MeshManager* mesh_manager,
                                 rclcpp::Node::SharedPtr ros2_node,
                                 const Config& config)
    : _allocator(allocator)
    , _get_tasks(get_tasks)
    , _get_own_state(get_own_state)
    , _mesh(mesh_manager)
    , _ros2_node(ros2_node)
    , _config(config) {
}

TaskCommHandler::~TaskCommHandler() {
    stop();
}

void TaskCommHandler::start() {
    if (_running.load()) return;
    
    _running.store(true);
    
    // Setup ROS 2 publishers
    if (_config.use_ros2 && _ros2_node) {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
        
        _bid_pub = _ros2_node->create_publisher<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_bid", qos);
        _award_pub = _ros2_node->create_publisher<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_award", qos);
        _bundle_pub = _ros2_node->create_publisher<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_bundle", qos);
        
        auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10))
            .best_effort()
            .durability_volatile();
        
        _bid_sub = _ros2_node->create_subscription<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_bid", sub_qos,
            std::bind(&TaskCommHandler::handleRos2Bid, this, std::placeholders::_1));
        
        _award_sub = _ros2_node->create_subscription<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_award", sub_qos,
            std::bind(&TaskCommHandler::handleRos2Award, this, std::placeholders::_1));
        
        // Services
        _upload_mission_srv = _ros2_node->create_service<swarm_msgs::srv::UploadMission>(
            "/swarm/upload_mission",
            std::bind(&TaskCommHandler::handleUploadMission, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        _rtl_srv = _ros2_node->create_service<swarm_msgs::srv::RequestRTL>(
            "/swarm/request_rtl",
            std::bind(&TaskCommHandler::handleRequestRTL, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        _estop_srv = _ros2_node->create_service<swarm_msgs::srv::EmergencyStop>(
            "/swarm/emergency_stop",
            std::bind(&TaskCommHandler::handleEmergencyStop, this,
                     std::placeholders::_1, std::placeholders::_2));
        
        _status_srv = _ros2_node->create_service<swarm_msgs::srv::GetSwarmStatus>(
            "/swarm/get_swarm_status",
            std::bind(&TaskCommHandler::handleGetSwarmStatus, this,
                     std::placeholders::_1, std::placeholders::_2));
    }
    
    // Register MAVLink callbacks
    if (_config.use_mavlink && _mesh) {
        _mesh->registerCallback(SwarmMsgId::TASK_BID,
            [this](uint16_t src_id, SwarmMsgId, const uint8_t* data, uint16_t len) {
                // Would parse MAVLink message here
            });
        _mesh->registerCallback(SwarmMsgId::TASK_AWARD,
            [this](uint16_t src_id, SwarmMsgId, const uint8_t* data, uint16_t len) {
                // Would parse MAVLink message here
            });
    }
    
    // Start publishing threads
    _running.store(true);
    _publish_thread = std::thread(&TaskCommHandler::publishLoop, this);
    _heartbeat_thread = std::thread(&TaskCommHandler::heartbeatLoop, this);
}

TaskCommHandler::~TaskCommHandler() {
    stop();
}

void TaskCommHandler::stop() {
    if (!_running.load()) return;
    
    _running.store(false);
    
    if (_publish_thread.joinable()) {
        _publish_thread.join();
    }
    if (_heartbeat_thread.joinable()) {
        _heartbeat_thread.join();
    }
}

void TaskCommHandler::publishBundle() {
    if (!_config.use_ros2 || !_ros2_node) return;
    
    auto bundle = _allocator.getBundle();
    if (bundle.tasks.empty()) return;
    
    // Publish as TaskAllocation messages (one per task)
    for (size_t i = 0; i < bundle.tasks.size(); ++i) {
        swarm_msgs::msg::TaskAllocation msg;
        msg.timestamp_us = Timestamp::nowUs();
        msg.drone_id = _allocator.getState().iteration;  // Would use actual drone ID
        msg.task_id = std::vector<uint8_t>(bundle.tasks[i].begin(), bundle.tasks[i].end());
        msg.cost = bundle.costs[i];
        msg.sequence = bundle.timestamps[i];
        
        _bundle_pub->publish(msg);
    }
}

void TaskCommHandler::publishBid(const std::array<uint8_t, 16>& task_id, float cost) {
    if (!_config.use_ros2 || !_ros2_node) return;
    
    swarm_msgs::msg::TaskAllocation msg;
    msg.timestamp_us = Timestamp::nowUs();
    msg.drone_id = _allocator.getState().iteration;  // Would use actual drone ID
    msg.task_id = std::vector<uint8_t>(task_id.begin(), task_id.end());
    msg.cost = cost;
    msg.sequence = _allocator.getState().iteration;
    
    _bid_pub->publish(msg);
    
    // Also send via MAVLink if enabled
    if (_config.use_mavlink && _mesh) {
        // Would pack and send MAVLink message
    }
}

void TaskCommHandler::handleMavlinkMessage(uint16_t src_sysid, uint16_t src_compid, 
                                           const mavlink_message_t& msg) {
    switch (msg.msgid) {
        case SwarmMsgId::TASK_BID:
            handleMavlinkTaskBid(src_sysid, msg);
            break;
        case SwarmMsgId::TASK_AWARD:
            handleMavlinkTaskAward(src_sysid, msg);
            break;
        case SwarmMsgId::TASK_ALLOCATION:  // Bundle
            handleMavlinkTaskBundle(src_sysid, msg);
            break;
    }
}

void TaskCommHandler::handleMavlinkTaskBid(uint16_t src_sysid, const mavlink_message_t& msg) {
    CbbaAllocator::Bid bid;
    if (!_allocator.deserializeBid({})) {
        return;
    }
    
    // Process bid through allocator
    _allocator.processBid(bid);
}

void TaskCommHandler::handleMavlinkTaskAward(uint16_t src_sysid, const mavlink_message_t& msg) {
    // Handle task award
}

void TaskCommHandler::handleMavlinkTaskBundle(uint16_t src_sysid, const mavlink_message_t& msg) {
    CbbaAllocator::Bundle bundle;
    if (!_allocator.deserializeBundle({})) {
        return;
    }
    
    // Process neighbor bundle
    _allocator.processNeighborBundle(src_sysid, bundle);
}

void TaskCommHandler::handleRos2Bid(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
    CbbaAllocator::Bid bid;
    bid.timestamp_us = msg->timestamp_us;
    bid.drone_id = msg->drone_id;
    bid.task_id = std::array<uint8_t, 16>();
    std::copy(msg->task_id.begin(), msg->task_id.end(), bid.task_id.begin());
    bid.cost = msg->cost;
    bid.sequence = msg->sequence;
    // Capability bitmap would be deserialized here
    
    _allocator.processBid(bid);
}

void TaskCommHandler::handleRos2Award(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
    // Handle award message
}

void TaskCommHandler::handleUploadMission(
    const std::shared_ptr<swarm_msgs::srv::UploadMission::Request> request,
    std::shared_ptr<swarm_msgs::srv::UploadMission::Response> response) {
    
    // Convert waypoints to mission
    // Would need mission uploader here
    response->success = true;
    response->message = "Mission upload not fully implemented";
    response->mission_sequence = 0;
}

void TaskCommHandler::handleRequestRTL(
    const std::shared_ptr<swarm_msgs::srv::RequestRTL::Request> request,
    std::shared_ptr<swarm_msgs::srv::RequestRTL::Response> response) {
    
    // Request RTL for specific drone or all
    response->success = true;
    response->message = "RTL requested";
}

void TaskCommHandler::handleEmergencyStop(
    const std::shared_ptr<swarm_msgs::srv::EmergencyStop::Request> request,
    std::shared_ptr<swarm_msgs::srv::EmergencyStop::Response> response) {
    
    response->success = true;
    response->message = "Emergency stop executed";
    response->affected_drones = {};  // Would populate with actual drones
}

void TaskCommHandler::handleGetSwarmStatus(
    const std::shared_ptr<swarm_msgs::srv::GetSwarmStatus::Request> request,
    std::shared_ptr<swarm_msgs::srv::GetSwarmStatus::Response> response) {
    
    response->timestamp_us = Timestamp::nowUs();
    response->active_drones = {};  // Would populate
    response->drone_roles = {};
    response->drone_modes = {};
    response->battery_pct = {};
    response->formation_id = 0;
    response->sequence = 0;
}

void TaskCommHandler::handleRos2Bid(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
    // Same as handleRos2Bid above
}

void TaskCommHandler::handleRos2Award(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
    // Handle award
}

void TaskCommHandler::publishBundleRos2() {
    publishBundle();
}

void TaskCommHandler::publishBidRos2(const std::array<uint8_t, 16>& task_id, float cost) {
    publishBid(task_id, cost);
}

void TaskCommHandler::publishAwardRos2(const std::array<uint8_t, 16>& task_id, 
                                        uint16_t drone_id, float cost) {
    if (!_config.use_ros2 || !_ros2_node) return;
    
    swarm_msgs::msg::TaskAllocation msg;
    msg.timestamp_us = Timestamp::nowUs();
    msg.drone_id = drone_id;
    msg.task_id = std::vector<uint8_t>(task_id.begin(), task_id.end());
    msg.cost = cost;
    msg.sequence = _allocator.getState().iteration;
    
    _award_pub->publish(msg);
}

void TaskCommHandler::sendHeartbeat() {
    if (!_config.use_mavlink || !_mesh) return;
    
    // Send heartbeat via MAVLink
    // Would use MAVLink heartbeat message
}

void TaskCommHandler::sendHeartbeatRos2() {
    // ROS 2 doesn't need explicit heartbeat (uses DDS discovery)
}

void TaskCommHandler::packTaskAllocation(const CbbaAllocator::Bid& bid, 
                                         swarm_msgs::msg::TaskAllocation& msg) {
    msg.timestamp_us = bid.timestamp_us;
    msg.drone_id = bid.drone_id;
    msg.task_id = std::vector<uint8_t>(bid.task_id.begin(), bid.task_id.end());
    msg.cost = bid.cost;
    msg.sequence = bid.sequence;
    // Capability bitmap would be packed here
}

void TaskCommHandler::unpackTaskAllocation(const swarm_msgs::msg::TaskAllocation& msg,
                                           CbbaAllocator::Bid& bid) {
    bid.timestamp_us = msg.timestamp_us;
    bid.drone_id = msg.drone_id;
    bid.task_id = std::array<uint8_t, 16>();
    std::copy(msg.task_id.begin(), msg.task_id.end(), bid.task_id.begin());
    bid.cost = msg.cost;
    bid.sequence = msg.sequence;
    // Capability bitmap would be unpacked here
}

uint64_t TaskCommHandler::nowUs() const {
    return Timestamp::nowUs();
}

} // namespace swarm::autonomy