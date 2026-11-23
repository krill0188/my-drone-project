#ifndef TELEMETRY_STREAMER_HPP
#define TELEMETRY_STREAMER_HPP

#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/agent_state.hpp>
#include <swarm_msgs/msg/ai_inference.hpp>
#include <swarm_msgs/msg/task_allocation.hpp>
#include <swarm_msgs/msg/swarm_map.hpp>
#include <swarm_msgs/msg/mission_status.hpp>
#include <swarm_msgs/msg/system_health.hpp>
#include <swarm_msgs/msg/formation_state.hpp>
#include <swarm_msgs/msg/comm_status.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <fstream>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>

namespace swarm::interface {

struct TelemetryStreamerConfig {
    // Output settings
    std::string output_directory = "/logs/telemetry";
    std::string bag_prefix = "swarm_telemetry";
    size_t max_bag_size_mb = 1024;  // 1GB per bag
    size_t max_bag_duration_s = 3600;  // 1 hour per bag
    
    // Topics to record
    std::vector<std::string> topics_to_record = {
        "/swarm/agent_states",
        "/swarm/ai_inference",
        "/swarm/task_allocation",
        "/swarm/swarm_map",
        "/swarm/mission_status",
        "/swarm/system_health",
        "/swarm/formation_state",
        "/swarm/comm_status",
        "/swarm/safety_events"
    };
    
    // Sync settings
    bool enable_time_sync = true;
    int sync_tolerance_ms = 10;
    std::string time_source = "system";  // "system", "gps", "ptp"
    
    // Compression
    std::string compression_format = "zstd";
    int compression_level = 3;
    
    // Real-time streaming
    bool enable_realtime_stream = true;
    std::string stream_protocol = "websocket";  // "websocket", "grpc", "mqtt"
    std::string stream_host = "0.0.0.0";
    int stream_port = 8080;
    
    // Performance
    size_t write_buffer_size_mb = 64;
    int flush_interval_ms = 1000;
    int max_queue_size = 10000;
    
    // Cleanup
    bool auto_cleanup_old = true;
    int max_storage_gb = 50;
    int max_age_days = 30;
};

struct TelemetryMessage {
    std::string topic_name;
    rclcpp::SerializedMessage serialized_msg;
    rclcpp::Time timestamp;
    std::string frame_id;
};

struct SyncPoint {
    uint64_t firmware_time_us = 0;
    uint64_t companion_time_us = 0;
    uint64_t gps_time_us = 0;
    double offset_us = 0.0;
    double drift_ppm = 0.0;
};

class TelemetryStreamer {
public:
    TelemetryStreamer();
    ~TelemetryStreamer();
    
    TelemetryStreamer(const TelemetryStreamer&) = delete;
    TelemetryStreamer& operator=(const TelemetryStreamer&) = delete;
    TelemetryStreamer(TelemetryStreamer&&) noexcept;
    TelemetryStreamer& operator=(TelemetryStreamer&&) noexcept;
    
    // Initialize
    bool initialize(const TelemetryStreamerConfig& config);
    
    // Start/stop streaming
    bool start();
    void stop();
    
    // Write message (thread-safe)
    void writeMessage(const std::string& topic_name, 
                      const rclcpp::SerializedMessage& msg,
                      const rclcpp::Time& timestamp = rclcpp::Time());
    
    // ROS 2 message callbacks
    void onAgentState(const swarm_msgs::msg::AgentState::SharedPtr msg);
    void onAIInference(const swarm_msgs::msg::AIInference::SharedPtr msg);
    void onTaskAllocation(const swarm_msgs::msg::TaskAllocation::SharedPtr msg);
    void onSwarmMap(const swarm_msgs::msg::SwarmMap::SharedPtr msg);
    void onMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg);
    void onSystemHealth(const swarm_msgs::msg::SystemHealth::SharedPtr msg);
    void onFormationState(const swarm_msgs::msg::FormationState::SharedPtr msg);
    void onCommStatus(const swarm_msgs::msg::CommStatus::SharedPtr msg);
    void onSafetyEvent(const swarm_msgs::msg::SafetyEvent::SharedPtr msg);
    
    // Time synchronization
    void addSyncPoint(const SyncPoint& sync);
    double getTimeOffset() const;
    bool isTimeSynced() const;
    
    // Bag management
    void rotateBag();
    void closeCurrentBag();
    
    // Real-time streaming
    void startRealtimeStream();
    void stopRealtimeStream();
    void broadcastMessage(const TelemetryMessage& msg);
    
    // Log synchronization (for post-flight analysis)
    struct LogSyncResult {
        bool success = false;
        std::string firmware_log_path;
        std::string companion_log_path;
        std::string synced_log_path;
        double max_offset_us = 0.0;
        double avg_offset_us = 0.0;
        int matched_messages = 0;
    };
    
    LogSyncResult synchronizeLogs(const std::string& firmware_log_path,
                                   const std::string& companion_log_path,
                                   const std::string& output_path);
    
    // Cleanup
    void cleanupOldLogs();
    
    // Status
    struct Stats {
        uint64_t messages_written = 0;
        uint64_t messages_dropped = 0;
        uint64_t bytes_written = 0;
        uint64_t bags_rotated = 0;
        double current_bag_size_mb = 0.0;
        uint64_t sync_points = 0;
        double avg_write_latency_ms = 0.0;
        bool is_recording = false;
        std::string current_bag_path;
    };
    const Stats& getStats() const;
    
    // Config
    const TelemetryStreamerConfig& getConfig() const { return config_; }
    
    // State
    bool isInitialized() const { return initialized_; }
    bool isRecording() const { return recording_; }
    
    // Callbacks
    using BagRotateCallback = std::function<void(const std::string& old_path, const std::string& new_path)>;
    void setBagRotateCallback(BagRotateCallback cb) { bag_rotate_callback_ = cb; }
    
    using SyncCallback = std::function<void(const SyncPoint&)>;
    void setSyncCallback(SyncCallback cb) { sync_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    TelemetryStreamerConfig config_;
    bool initialized_ = false;
    bool recording_ = false;
    
    std::function<void(const std::string&, const std::string&)> bag_rotate_callback_;
    std::function<void(const SyncPoint&)> sync_callback_;
};

}  // namespace swarm::interface

#endif  // TELEMETRY_STREAMER_HPP