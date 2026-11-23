#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/msg/ai_inference.hpp"
#include "swarm_msgs/msg/task_allocation.hpp"
#include "swarm_msgs/msg/swarm_map.hpp"
#include "swarm_msgs/msg/mission_status.hpp"
#include "swarm_msgs/msg/system_health.hpp"
#include "swarm_msgs/msg/formation_state.hpp"
#include "swarm_msgs/msg/comm_status.hpp"
#include "swarm_msgs/msg/safety_event.hpp"
#include "swarm_msgs/msg/telemetry_sync.hpp"
#include "swarm_interface_telemetry_streamer/telemetry_streamer.hpp"
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class TelemetryStreamerNode : public rclcpp::Node {
public:
    TelemetryStreamerNode() : Node("telemetry_streamer_node") {
        declareParameters();
        loadParameters();
        
        // Initialize telemetry streamer
        streamer_ = std::make_unique<swarm::interface::TelemetryStreamer>();
        swarm::interface::TelemetryStreamerConfig config;
        
        config.output_directory = get_parameter("output_directory").as_string();
        config.bag_prefix = get_parameter("bag_prefix").as_string();
        config.max_bag_size_mb = get_parameter("max_bag_size_mb").as_int();
        config.max_bag_duration_s = get_parameter("max_bag_duration_s").as_int();
        config.topics_to_record = get_parameter("topics_to_record").as_string_array();
        config.enable_time_sync = get_parameter("enable_time_sync").as_bool();
        config.sync_tolerance_ms = get_parameter("sync_tolerance_ms").as_int();
        config.time_source = get_parameter("time_source").as_string();
        config.compression_format = get_parameter("compression_format").as_string();
        config.compression_level = get_parameter("compression_level").as_int();
        config.enable_realtime_stream = get_parameter("enable_realtime_stream").as_bool();
        config.stream_host = get_parameter("stream_host").as_string();
        config.stream_port = get_parameter("stream_port").as_int();
        config.write_buffer_size_mb = get_parameter("write_buffer_size_mb").as_int();
        config.flush_interval_ms = get_parameter("flush_interval_ms").as_int();
        config.max_queue_size = get_parameter("max_queue_size").as_int();
        config.auto_cleanup_old = get_parameter("auto_cleanup_old").as_bool();
        config.max_storage_gb = get_parameter("max_storage_gb").as_int();
        config.max_age_days = get_parameter("max_age_days").as_int();
        
        if (!streamer_->initialize(config)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize telemetry streamer");
        }
        
        // Setup subscriptions
        setupSubscriptions();
        
        // Setup publishers
        setupPublishers();
        
        // Setup services
        setupServices();
        
        // Set callbacks
        streamer_->setBagRotateCallback(
            std::bind(&TelemetryStreamerNode::onBagRotated, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        streamer_->setSyncCallback(
            std::bind(&TelemetryStreamerNode::onSyncPoint, this, 
                      std::placeholders::_1));
        
        // Start streaming
        streamer_->start();
        
        // Timer for stats
        stats_timer_ = create_wall_timer(
            10s, std::bind(&TelemetryStreamerNode::publishStats, this));
        
        // Timer for sync
        sync_timer_ = create_wall_timer(
            1s, std::bind(&TelemetryStreamerNode::requestSync, this));
        
        RCLCPP_INFO(get_logger(), "Telemetry Streamer Node initialized");
    }
    
private:
    // Configuration
    void declareParameters() {
        declare_parameter("output_directory", "/logs/telemetry");
        declare_parameter("bag_prefix", "swarm_telemetry");
        declare_parameter("max_bag_size_mb", 1024);
        declare_parameter("max_bag_duration_s", 3600);
        declare_parameter("topics_to_record", std::vector<std::string>({
            "/swarm/agent_states",
            "/swarm/ai_inference",
            "/swarm/task_allocation",
            "/swarm/swarm_map",
            "/swarm/mission_status",
            "/swarm/system_health",
            "/swarm/formation_state",
            "/swarm/comm_status",
            "/swarm/safety_events"
        }));
        declare_parameter("enable_time_sync", true);
        declare_parameter("sync_tolerance_ms", 10);
        declare_parameter("time_source", "system");
        declare_parameter("compression_format", "zstd");
        declare_parameter("compression_level", 3);
        declare_parameter("enable_realtime_stream", true);
        declare_parameter("stream_host", "0.0.0.0");
        declare_parameter("stream_port", 8080);
        declare_parameter("write_buffer_size_mb", 64);
        declare_parameter("flush_interval_ms", 1000);
        declare_parameter("max_queue_size", 10000);
        declare_parameter("auto_cleanup_old", true);
        declare_parameter("max_storage_gb", 50);
        declare_parameter("max_age_days", 30);
        declare_parameter("drone_id", 1);
    }
    
    void loadParameters() {
        drone_id_ = get_parameter("drone_id").as_int();
    }
    
    void setupSubscriptions() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        
        // Agent state
        agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", qos,
            [this](const swarm_msgs::msg::AgentState::SharedPtr msg) {
                if (msg->sysid == drone_id_) {
                    streamer_->onAgentState(msg);
                }
            });
        
        // AI inference
        ai_inference_sub_ = create_subscription<swarm_msgs::msg::AIInference>(
            "/swarm/ai_inference", qos,
            [this](const swarm_msgs::msg::AIInference::SharedPtr msg) {
                streamer_->onAIInference(msg);
            });
        
        // Task allocation
        task_alloc_sub_ = create_subscription<swarm_msgs::msg::TaskAllocation>(
            "/swarm/task_allocation", qos,
            [this](const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
                streamer_->onTaskAllocation(msg);
            });
        
        // Swarm map
        swarm_map_sub_ = create_subscription<swarm_msgs::msg::SwarmMap>(
            "/swarm/swarm_map", qos,
            [this](const swarm_msgs::msg::SwarmMap::SharedPtr msg) {
                streamer_->onSwarmMap(msg);
            });
        
        // Mission status
        mission_status_sub_ = create_subscription<swarm_msgs::msg::MissionStatus>(
            "/swarm/mission_status", qos,
            [this](const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
                streamer_->onMissionStatus(msg);
            });
        
        // System health
        health_sub_ = create_subscription<swarm_msgs::msg::SystemHealth>(
            "/swarm/system_health", qos,
            [this](const swarm_msgs::msg::SystemHealth::SharedPtr msg) {
                streamer_->onSystemHealth(msg);
            });
        
        // Formation state
        formation_sub_ = create_subscription<swarm_msgs::msg::FormationState>(
            "/swarm/formation_state", qos,
            [this](const swarm_msgs::msg::FormationState::SharedPtr msg) {
                streamer_->onFormationState(msg);
            });
        
        // Communication status
        comm_sub_ = create_subscription<swarm_msgs::msg::CommStatus>(
            "/swarm/comm_status", qos,
            [this](const swarm_msgs::msg::CommStatus::SharedPtr msg) {
                streamer_->onCommStatus(msg);
            });
        
        // Safety events
        safety_sub_ = create_subscription<swarm_msgs::msg::SafetyEvent>(
            "/swarm/safety_events", qos,
            [this](const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
                streamer_->onSafetyEvent(msg);
            });
        
        // Time sync
        sync_sub_ = create_subscription<swarm_msgs::msg::TelemetrySync>(
            "/swarm/telemetry_sync", qos,
            [this](const swarm_msgs::msg::TelemetrySync::SharedPtr msg) {
                swarm::interface::SyncPoint sync;
                sync.firmware_time_us = msg->firmware_time_us;
                sync.companion_time_us = msg->companion_time_us;
                sync.gps_time_us = msg->gps_time_us;
                sync.offset_us = msg->offset_us;
                sync.drift_ppm = msg->drift_ppm;
                streamer_->addSyncPoint(sync);
            });
    }
    
    void setupPublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        // Stats
        stats_pub_ = create_publisher<swarm_msgs::msg::TelemetryStats>(
            "/drone" + std::to_string(drone_id_) + "/telemetry_stats", qos);
        
        // Sync status
        sync_pub_ = create_publisher<swarm_msgs::msg::TelemetrySyncStatus>(
            "/drone" + std::to_string(drone_id_) + "/telemetry_sync", qos);
        
        // Bag rotate events
        bag_rotate_pub_ = create_publisher<swarm_msgs::msg::BagRotateEvent>(
            "/drone" + std::to_string(drone_id_) + "/bag_rotate", qos);
    }
    
    void setupServices() {
        // Synchronize logs service
        sync_logs_service_ = create_service<swarm_msgs::srv::SynchronizeLogs>(
            "/drone" + std::to_string(drone_id_) + "/synchronize_logs",
            [this](const std::shared_ptr<rmw_request_id_t>,
                   const std::shared_ptr<swarm_msgs::srv::SynchronizeLogs::Request> request,
                   std::shared_ptr<swarm_msgs::srv::SynchronizeLogs::Response> response) {
                auto result = streamer_->synchronizeLogs(
                    request->firmware_log_path,
                    request->companion_log_path,
                    request->output_path);
                
                response->success = result.success;
                response->firmware_log_path = result.firmware_log_path;
                response->companion_log_path = result.companion_log_path;
                response->synced_log_path = result.synced_log_path;
                response->max_offset_us = result.max_offset_us;
                response->avg_offset_us = result.avg_offset_us;
                response->matched_messages = result.matched_messages;
            });
        
        // Rotate bag service
        rotate_bag_service_ = create_service<swarm_msgs::srv::RotateBag>(
            "/drone" + std::to_string(drone_id_) + "/rotate_bag",
            [this](const std::shared_ptr<rmw_request_id_t>,
                   const std::shared_ptr<swarm_msgs::srv::RotateBag::Request> request,
                   std::shared_ptr<swarm_msgs::srv::RotateBag::Response> response) {
                streamer_->rotateBag();
                response->success = true;
                response->message = "Bag rotation triggered";
            });
        
        // Cleanup logs service
        cleanup_logs_service_ = create_service<swarm_msgs::srv::CleanupLogs>(
            "/drone" + std::to_string(drone_id_) + "/cleanup_logs",
            [this](const std::shared_ptr<rmw_request_id_t>,
                   const std::shared_ptr<swarm_msgs::srv::CleanupLogs::Request> request,
                   std::shared_ptr<swarm_msgs::srv::CleanupLogs::Response> response) {
                streamer_->cleanupOldLogs();
                response->success = true;
                response->message = "Log cleanup triggered";
            });
    }
    
    void onBagRotated(const std::string& old_path, const std::string& new_path) {
        auto msg = swarm_msgs::msg::BagRotateEvent();
        msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.old_path = old_path;
        msg.new_path = new_path;
        bag_rotate_pub_->publish(msg);
        
        RCLCPP_INFO(get_logger(), "Bag rotated: %s -> %s", old_path.c_str(), new_path.c_str());
    }
    
    void onSyncPoint(const swarm::interface::SyncPoint& sync) {
        auto msg = swarm_msgs::msg::TelemetrySyncStatus();
        msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.drone_id = drone_id_;
        msg.firmware_time_us = sync.firmware_time_us;
        msg.companion_time_us = sync.companion_time_us;
        msg.gps_time_us = sync.gps_time_us;
        msg.offset_us = sync.offset_us;
        msg.drift_ppm = sync.drift_ppm;
        msg.synced = std::abs(sync.offset_us) < 1000.0;
        
        sync_pub_->publish(msg);
    }
    
    void requestSync() {
        // Request time sync from firmware
        // Would publish to firmware time sync topic
    }
    
    void publishStats() {
        auto stats = streamer_->getStats();
        
        auto msg = swarm_msgs::msg::TelemetryStats();
        msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        msg.drone_id = drone_id_;
        msg.messages_written = stats.messages_written;
        msg.messages_dropped = stats.messages_dropped;
        msg.bytes_written = stats.bytes_written;
        msg.bags_rotated = stats.bags_rotated;
        msg.current_bag_size_mb = stats.current_bag_size_mb;
        msg.sync_points = stats.sync_points;
        msg.avg_write_latency_ms = stats.avg_write_latency_ms;
        msg.is_recording = stats.is_recording;
        msg.current_bag_path = stats.current_bag_path;
        
        stats_pub_->publish(msg);
    }
    
    // Members
    std::unique_ptr<swarm::interface::TelemetryStreamer> streamer_;
    uint16_t drone_id_ = 1;
    
    // ROS interfaces
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    rclcpp::Subscription<swarm_msgs::msg::AIInference>::SharedPtr ai_inference_sub_;
    rclcpp::Subscription<swarm_msgs::msg::TaskAllocation>::SharedPtr task_alloc_sub_;
    rclcpp::Subscription<swarm_msgs::msg::SwarmMap>::SharedPtr swarm_map_sub_;
    rclcpp::Subscription<swarm_msgs::msg::MissionStatus>::SharedPtr mission_status_sub_;
    rclcpp::Subscription<swarm_msgs::msg::SystemHealth>::SharedPtr health_sub_;
    rclcpp::Subscription<swarm_msgs::msg::FormationState>::SharedPtr formation_sub_;
    rclcpp::Subscription<swarm_msgs::msg::CommStatus>::SharedPtr comm_sub_;
    rclcpp::Subscription<swarm_msgs::msg::SafetyEvent>::SharedPtr safety_sub_;
    rclcpp::Subscription<swarm_msgs::msg::TelemetrySync>::SharedPtr sync_sub_;
    
    rclcpp::Publisher<swarm_msgs::msg::TelemetryStats>::SharedPtr stats_pub_;
    rclcpp::Publisher<swarm_msgs::msg::TelemetrySyncStatus>::SharedPtr sync_pub_;
    rclcpp::Publisher<swarm_msgs::msg::BagRotateEvent>::SharedPtr bag_rotate_pub_;
    
    rclcpp::Service<swarm_msgs::srv::SynchronizeLogs>::SharedPtr sync_logs_service_;
    rclcpp::Service<swarm_msgs::srv::RotateBag>::SharedPtr rotate_bag_service_;
    rclcpp::Service<swarm_msgs::srv::CleanupLogs>::SharedPtr cleanup_logs_service_;
    
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr sync_timer_;
    
    uint16_t drone_id_ = 1;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TelemetryStreamerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}