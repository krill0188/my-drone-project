#ifndef ROSBAG_WRITER_HPP
#define ROSBAG_WRITER_HPP

#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <swarm_msgs/msg/agent_state.hpp>
#include <swarm_msgs/msg/ai_inference.hpp>
#include <swarm_msgs/msg/task_allocation.hpp>
#include <swarm_msgs/msg/swarm_map.hpp>
#include <swarm_msgs/msg/mission_status.hpp>
#include <swarm_msgs/msg/system_health.hpp>
#include <swarm_msgs/msg/formation_state.hpp>
#include <swarm_msgs/msg/comm_status.hpp>
#include <swarm_msgs/msg/safety_event.hpp>
#include <swarm_msgs/msg/telemetry_sync.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <thread>
#include <atomic>
#include <functional>
#include <filesystem>

namespace swarm::interface {

struct RosbagWriterConfig {
    std::string output_directory = "/logs/rosbag";
    std::string bag_prefix = "swarm_companion";
    size_t max_bag_size_mb = 1024;
    size_t max_bag_duration_s = 3600;
    std::vector<std::string> topics_to_record = {
        "/swarm/ai_inference",
        "/swarm/comm_status",
        "/swarm/task_allocation",
        "/swarm/swarm_map",
        "/swarm/mission_status",
        "/swarm/system_health",
        "/swarm/formation_state",
        "/swarm/safety_events",
        "/swarm/telemetry_sync"
    };
    std::string compression_format = "zstd";
    int compression_level = 3;
    size_t write_buffer_size_mb = 64;
    size_t flush_interval_ms = 1000;
    size_t max_queue_size = 10000;
    bool auto_cleanup_old = true;
    size_t max_storage_gb = 50;
    int max_age_days = 30;
    bool enable_indexing = true;
};

struct RosbagWriterStats {
    uint64_t messages_written = 0;
    uint64_t messages_dropped = 0;
    uint64_t bytes_written = 0;
    uint64_t bags_rotated = 0;
    double current_bag_size_mb = 0.0;
    std::map<std::string, uint64_t> messages_per_topic;
    std::map<std::string, uint64_t> bytes_per_topic;
    double avg_write_latency_ms = 0.0;
    bool is_recording = false;
    std::string current_bag_path;
};

class RosbagWriter {
public:
    using BagRotateCallback = std::function<void(const std::string& old_path, const std::string& new_path)>;
    
    RosbagWriter();
    ~RosbagWriter();
    
    RosbagWriter(const RosbagWriter&) = delete;
    RosbagWriter& operator=(const RosbagWriter&) = delete;
    RosbagWriter(RosbagWriter&&) noexcept = default;
    RosbagWriter& operator=(RosbagWriter&&) noexcept = default;
    
    bool initialize(const RosbagWriterConfig& config);
    void start();
    void stop();
    bool isRecording() const { return _recording.load(); }
    bool isInitialized() const { return _initialized; }
    
    // Write methods for each message type
    void writeAIInference(const swarm_msgs::msg::AIInference::SharedPtr msg);
    void writeCommStatus(const swarm_msgs::msg::CommStatus::SharedPtr msg);
    void writeTaskAllocation(const swarm_msgs::msg::TaskAllocation::SharedPtr msg);
    void writeSwarmMap(const swarm_msgs::msg::SwarmMap::SharedPtr msg);
    void writeMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg);
    void writeSystemHealth(const swarm_msgs::msg::SystemHealth::SharedPtr msg);
    void writeFormationState(const swarm_msgs::msg::FormationState::SharedPtr msg);
    void writeSafetyEvent(const swarm_msgs::msg::SafetyEvent::SharedPtr msg);
    void writeTelemetrySync(const swarm_msgs::msg::TelemetrySync::SharedPtr msg);
    
    // Generic write
    template<typename MsgT>
    void writeMessage(const std::string& topic_name, const MsgT& msg, const rclcpp::Time& timestamp);
    
    // Bag management
    void rotateBag();
    void closeCurrentBag();
    
    // Cleanup
    void cleanupOldLogs();
    
    // Statistics
    const RosbagWriterStats& getStats() const { return _stats; }
    
    // Callbacks
    void setBagRotateCallback(BagRotateCallback cb) { _bag_rotate_cb = cb; }
    
    // Indexing
    struct MessageIndex {
        std::string topic_name;
        uint64_t timestamp_ns;
        uint64_t file_offset;
        size_t message_size;
    };
    
    std::vector<MessageIndex> getMessageIndex(const std::string& topic_name, 
                                               uint64_t start_time_ns, 
                                               uint64_t end_time_ns) const;
    
    const RosbagWriterConfig& getConfig() const { return _config; }

private:
    struct QueuedMessage {
        std::string topic_name;
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Time timestamp;
    };
    
    bool initWriter();
    std::string generateBagPath() const;
    void createTopics();
    void writerLoop();
    void queueMessage(std::unique_ptr<rosbag2_storage::SerializedBagMessage> msg);
    void flushQueue();
    void checkBagRotation();
    void updateStats(const std::string& topic, size_t bytes, double latency_ms);
    
    template<typename MsgT>
    void serializeAndQueue(const std::string& topic, const MsgT& msg, uint64_t timestamp_ns);
    
    // Config and state
    RosbagWriterConfig _config;
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> _writer;
    std::string _current_bag_path;
    
    std::atomic<bool> _initialized{false};
    std::atomic<bool> _recording{false};
    std::atomic<bool> _should_stop{false};
    
    std::thread _writer_thread;
    std::mutex _queue_mutex;
    std::condition_variable _queue_cv;
    std::queue<std::unique_ptr<rosbag2_storage::SerializedBagMessage>> _message_queue;
    
    RosbagWriterStats _stats;
    mutable std::mutex _stats_mutex;
    std::chrono::steady_clock::time_point _bag_start_time;
    uint64_t _current_bag_bytes = 0;
    
    // Indexing
    struct IndexEntry {
        std::string topic_name;
        uint64_t timestamp_ns;
        uint64_t file_offset;
        size_t message_size;
    };
    mutable std::mutex _index_mutex;
    std::vector<IndexEntry> _message_index;
    
    // Callback
    BagRotateCallback _bag_rotate_cb;
};

template<typename MsgT>
void RosbagWriter::writeMessage(const std::string& topic_name, const MsgT& msg, const rclcpp::Time& timestamp) {
    if (!_recording.load()) return;
    
    serializeAndQueue<MsgT>(topic_name, msg, timestamp.nanoseconds());
}

template<typename MsgT>
void RosbagWriter::serializeAndQueue(const std::string& topic, const MsgT& msg, uint64_t timestamp_ns) {
    if (!_writer) return;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    rclcpp::Serialization<MsgT> serializer;
    rclcpp::SerializedMessage serialized_msg;
    serializer.serialize_message(&msg, &serialized_msg);
    
    auto bag_msg = std::make_unique<rosbag2_storage::SerializedBagMessage>();
    bag_msg->topic_name = topic;
    bag_msg->time_stamp = timestamp_ns;
    bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
    bag_msg->serialized_data->buffer = serialized_msg.get_rcl_serialized_message().buffer;
    bag_msg->serialized_data->buffer_length = serialized_msg.get_rcl_serialized_message().buffer_length;
    bag_msg->serialized_data->buffer_capacity = serialized_msg.get_rcl_serialized_message().buffer_capacity;
    
    queueMessage(std::move(bag_msg));
    
    auto end = std::chrono::high_resolution_clock::now();
    double latency = std::chrono::duration<double, std::milli>(end - start).count();
    updateStats(topic, serialized_msg.size(), latency);
}

}  // namespace swarm::interface

#endif // ROSBAG_WRITER_HPP