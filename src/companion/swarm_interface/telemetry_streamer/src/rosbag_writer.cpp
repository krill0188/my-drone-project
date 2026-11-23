#include "rosbag_writer.hpp"
#include <chrono>
#include <thread>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <sys/statvfs.h>
#include <unistd.h>

namespace swarm::interface {

using namespace std::chrono_literals;

RosbagWriter::RosbagWriter() = default;

RosbagWriter::~RosbagWriter() {
    stop();
}

bool RosbagWriter::initialize(const RosbagWriterConfig& config) {
    _config = config;
    
    // Create output directory
    std::filesystem::create_directories(_config.output_directory);
    
    // Initialize writer
    if (!initWriter()) {
        return false;
    }
    
    _initialized = true;
    return true;
}

void RosbagWriter::start() {
    if (!_initialized) return;
    _recording = true;
    _should_stop = false;
    _bag_start_time = std::chrono::steady_clock::now();
    _writer_thread = std::thread(&RosbagWriter::writerLoop, this);
}

void RosbagWriter::stop() {
    _recording = false;
    _should_stop = true;
    _queue_cv.notify_all();
    
    if (_writer_thread.joinable()) {
        _writer_thread.join();
    }
    
    flushQueue();
    closeCurrentBag();
}

bool RosbagWriter::initWriter() {
    try {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = generateBagPath();
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        _writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        _writer->open(storage_options, converter_options);
        
        createTopics();
        
        _current_bag_path = storage_options.uri;
        _current_bag_bytes = 0;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize rosbag writer: " << e.what() << std::endl;
        return false;
    }
}

std::string RosbagWriter::generateBagPath() const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << _config.output_directory << "/" << _config.bag_prefix << "_"
       << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
       << "_" << std::setfill('0') << std::setw(3) << ms.count()
       << ".db3";
    
    return ss.str();
}

void RosbagWriter::createTopics() {
    if (!_writer) return;
    
    std::map<std::string, std::string> topic_types = {
        {"/swarm/ai_inference", "swarm_msgs/msg/AIInference"},
        {"/swarm/comm_status", "swarm_msgs/msg/CommStatus"},
        {"/swarm/task_allocation", "swarm_msgs/msg/TaskAllocation"},
        {"/swarm/swarm_map", "swarm_msgs/msg/SwarmMap"},
        {"/swarm/mission_status", "swarm_msgs/msg/MissionStatus"},
        {"/swarm/system_health", "swarm_msgs/msg/SystemHealth"},
        {"/swarm/formation_state", "swarm_msgs/msg/FormationState"},
        {"/swarm/safety_events", "swarm_msgs/msg/SafetyEvent"},
        {"/swarm/telemetry_sync", "swarm_msgs/msg/TelemetrySync"}
    };
    
    for (const auto& [name, type] : topic_types) {
        rosbag2_storage::TopicMetadata topic;
        topic.name = name;
        topic.type = type;
        topic.serialization_format = "cdr";
        topic.offered_qos_profiles = "";
        _writer->create_topic(topic);
    }
}

void RosbagWriter::writerLoop() {
    while (!_should_stop) {
        std::unique_ptr<rosbag2_storage::SerializedBagMessage> msg;
        
        {
            std::unique_lock<std::mutex> lock(_queue_mutex);
            _queue_cv.wait_for(lock, 10ms, [this]() {
                return !_message_queue.empty() || _should_stop;
            });
            
            if (!_message_queue.empty()) {
                msg = std::move(_message_queue.front());
                _message_queue.pop();
            }
        }
        
        if (msg && _writer) {
            try {
                auto write_start = std::chrono::high_resolution_clock::now();
                _writer->write(msg);
                auto write_end = std::chrono::high_resolution_clock::now();
                
                double write_latency = std::chrono::duration<double, std::milli>(write_end - write_start).count();
                
                // Update index if enabled
                if (_config.enable_indexing) {
                    std::lock_guard<std::mutex> index_lock(_index_mutex);
                    IndexEntry entry;
                    entry.topic_name = msg->topic_name;
                    entry.timestamp_ns = msg->time_stamp;
                    entry.file_offset = _current_bag_bytes;
                    entry.message_size = msg->serialized_data->buffer_length;
                    _message_index.push_back(entry);
                }
                
                _current_bag_bytes += msg->serialized_data->buffer_length;
                
                checkBagRotation();
            } catch (const std::exception& e) {
                std::cerr << "Rosbag write error: " << e.what() << std::endl;
            }
        }
    }
    
    // Flush remaining messages
    flushQueue();
}

void RosbagWriter::queueMessage(std::unique_ptr<rosbag2_storage::SerializedBagMessage> msg) {
    std::lock_guard<std::mutex> lock(_queue_mutex);
    
    if (_message_queue.size() >= _config.max_queue_size) {
        std::lock_guard<std::mutex> stats_lock(_stats_mutex);
        _stats.messages_dropped++;
        return;
    }
    
    _message_queue.push(std::move(msg));
    _queue_cv.notify_one();
}

void RosbagWriter::flushQueue() {
    std::unique_ptr<rosbag2_storage::SerializedBagMessage> msg;
    while (true) {
        {
            std::lock_guard<std::mutex> lock(_queue_mutex);
            if (_message_queue.empty()) break;
            msg = std::move(_message_queue.front());
            _message_queue.pop();
        }
        
        if (msg && _writer) {
            try {
                _writer->write(msg);
                _current_bag_bytes += msg->serialized_data->buffer_length;
            } catch (...) {}
        }
    }
}

void RosbagWriter::checkBagRotation() {
    double size_mb = _current_bag_bytes / (1024.0 * 1024.0);
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - _bag_start_time).count();
    
    if (size_mb >= _config.max_bag_size_mb || 
        static_cast<size_t>(elapsed) >= _config.max_bag_duration_s) {
        rotateBag();
    }
}

void RosbagWriter::updateStats(const std::string& topic, size_t bytes, double latency_ms) {
    std::lock_guard<std::mutex> lock(_stats_mutex);
    _stats.messages_written++;
    _stats.bytes_written += bytes;
    _stats.messages_per_topic[topic]++;
    _stats.bytes_per_topic[topic] += bytes;
    _stats.avg_write_latency_ms = 0.9 * _stats.avg_write_latency_ms + 0.1 * latency_ms;
    _stats.current_bag_size_mb = _current_bag_bytes / (1024.0 * 1024.0);
    _stats.current_bag_path = _current_bag_path;
    _stats.is_recording = _recording.load();
}

void RosbagWriter::rotateBag() {
    closeCurrentBag();
    
    std::string old_path = _current_bag_path;
    std::string new_path = generateBagPath();
    
    try {
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = new_path;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        _writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
        _writer->open(storage_options, converter_options);
        createTopics();
        
        _current_bag_path = new_path;
        _current_bag_bytes = 0;
        _bag_start_time = std::chrono::steady_clock::now();
        
        {
            std::lock_guard<std::mutex> lock(_stats_mutex);
            _stats.bags_rotated++;
        }
        
        if (_bag_rotate_cb) {
            _bag_rotate_cb(old_path, new_path);
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Bag rotation failed: " << e.what() << std::endl;
    }
}

void RosbagWriter::closeCurrentBag() {
    if (_writer) {
        _writer.reset();
    }
}

void RosbagWriter::cleanupOldLogs() {
    if (!_config.auto_cleanup_old) return;
    
    std::filesystem::path log_dir(_config.output_directory);
    auto now = std::chrono::system_clock::now();
    size_t total_size = 0;
    
    struct LogFile {
        std::filesystem::path path;
        std::chrono::system_clock::time_point modified;
        size_t size;
    };
    std::vector<LogFile> logs;
    
    for (const auto& entry : std::filesystem::directory_iterator(log_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".db3") {
            auto ftime = std::filesystem::last_write_time(entry);
            auto size = std::filesystem::file_size(entry);
            total_size += size;
            
            LogFile lf;
            lf.path = entry.path();
            lf.modified = std::chrono::clock_cast<std::chrono::system_clock>(ftime);
            lf.size = size;
            logs.push_back(lf);
        }
    }
    
    std::sort(logs.begin(), logs.end(),
              [](const LogFile& a, const LogFile& b) {
                  return a.modified < b.modified;
              });
    
    auto cutoff = now - std::chrono::hours(24 * _config.max_age_days);
    size_t max_storage_bytes = _config.max_storage_gb * 1024 * 1024 * 1024;
    
    for (const auto& log : logs) {
        if (log.modified < cutoff || total_size > max_storage_bytes) {
            std::filesystem::remove(log.path);
            total_size -= log.size;
        }
    }
}

std::vector<RosbagWriter::MessageIndex> RosbagWriter::getMessageIndex(
    const std::string& topic_name, 
    uint64_t start_time_ns, 
    uint64_t end_time_ns) const {
    
    std::vector<MessageIndex> result;
    
    std::lock_guard<std::mutex> lock(_index_mutex);
    for (const auto& entry : _message_index) {
        if (entry.topic_name == topic_name &&
            entry.timestamp_ns >= start_time_ns &&
            entry.timestamp_ns <= end_time_ns) {
            MessageIndex idx;
            idx.topic_name = entry.topic_name;
            idx.timestamp_ns = entry.timestamp_ns;
            idx.file_offset = entry.file_offset;
            idx.message_size = entry.message_size;
            result.push_back(idx);
        }
    }
    
    return result;
}

// Write methods for specific message types
void RosbagWriter::writeAIInference(const swarm_msgs::msg::AIInference::SharedPtr msg) {
    writeMessage("/swarm/ai_inference", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeCommStatus(const swarm_msgs::msg::CommStatus::SharedPtr msg) {
    writeMessage("/swarm/comm_status", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeTaskAllocation(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
    writeMessage("/swarm/task_allocation", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeSwarmMap(const swarm_msgs::msg::SwarmMap::SharedPtr msg) {
    writeMessage("/swarm/swarm_map", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
    writeMessage("/swarm/mission_status", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeSystemHealth(const swarm_msgs::msg::SystemHealth::SharedPtr msg) {
    writeMessage("/swarm/system_health", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeFormationState(const swarm_msgs::msg::FormationState::SharedPtr msg) {
    writeMessage("/swarm/formation_state", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeSafetyEvent(const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
    writeMessage("/swarm/safety_events", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

void RosbagWriter::writeTelemetrySync(const swarm_msgs::msg::TelemetrySync::SharedPtr msg) {
    writeMessage("/swarm/telemetry_sync", *msg, rclcpp::Time(msg->timestamp_us * 1us));
}

}  // namespace swarm::interface