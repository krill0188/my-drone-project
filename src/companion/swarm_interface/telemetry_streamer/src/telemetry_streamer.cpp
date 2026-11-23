#include "telemetry_streamer.hpp"
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <chrono>
#include <thread>
#include <filesystem>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <random>
#include <mutex>
#include <condition_variable>

namespace swarm::interface {

using namespace std::chrono_literals;

class TelemetryStreamer::Impl {
public:
    Impl() : writer_(std::make_unique<rosbag2_cpp::writers::SequentialWriter>()),
             running_(false),
             should_stop_(false) {}
    
    ~Impl() {
        stop();
    }
    
    bool initialize(const TelemetryStreamerConfig& config) {
        config_ = config;
        
        // Create output directory
        std::filesystem::create_directories(config_.output_directory);
        
        // Initialize writer with storage options
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = getNextBagPath();
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        writer_->open(storage_options, converter_options);
        
        // Create topics in bag
        createTopics();
        
        // Start writer thread
        running_ = true;
        should_stop_ = false;
        writer_thread_ = std::thread(&Impl::writerLoop, this);
        
        // Start sync thread if enabled
        if (config_.enable_time_sync) {
            sync_thread_ = std::thread(&Impl::syncLoop, this);
        }
        
        // Start realtime stream if enabled
        if (config_.enable_realtime_stream) {
            startRealtimeStream();
        }
        
        // Start cleanup thread
        cleanup_thread_ = std::thread(&Impl::cleanupLoop, this);
        
        return true;
    }
    
    void start() {
        recording_ = true;
    }
    
    void stop() {
        recording_ = false;
        should_stop_ = true;
        
        if (writer_thread_.joinable()) writer_thread_.join();
        if (sync_thread_.joinable()) sync_thread_.join();
        if (cleanup_thread_.joinable()) cleanup_thread_.join();
        if (stream_thread_.joinable()) stream_thread_.join();
        
        closeCurrentBag();
    }
    
    void writeMessage(const std::string& topic_name, 
                      const rclcpp::SerializedMessage& msg,
                      const rclcpp::Time& timestamp) {
        if (!recording_) return;
        
        TelemetryMessage msg;
        msg.topic_name = topic_name;
        msg.serialized_msg = msg;
        msg.timestamp = timestamp;
        
        // Add to write queue
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (write_queue_.size() >= config_.max_queue_size) {
                stats_.messages_dropped++;
                return;
            }
            write_queue_.push(std::move(msg));
        }
        queue_cv_.notify_one();
    }
    
    void onAgentState(const swarm_msgs::msg::AgentState::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::AgentState> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/agent_states", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onAIInference(const swarm_msgs::msg::AIInference::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::AIInference> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/ai_inference", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onTaskAllocation(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::TaskAllocation> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/task_allocation", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onSwarmMap(const swarm_msgs::msg::SwarmMap::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::SwarmMap> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/swarm_map", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::MissionStatus> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/mission_status", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onSystemHealth(const swarm_msgs::msg::SystemHealth::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::SystemHealth> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/system_health", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onFormationState(const swarm_msgs::msg::FormationState::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::FormationState> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/formation_state", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onCommStatus(const swarm_msgs::msg::CommStatus::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::CommStatus> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/comm_status", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void onSafetyEvent(const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
        rclcpp::SerializedMessage serialized_msg;
        rclcpp::Serialization<swarm_msgs::msg::SafetyEvent> serializer;
        serializer.serialize_message(msg.get(), &serialized_msg);
        writeMessage("/swarm/safety_events", serialized_msg, rclcpp::Time(msg->timestamp_us * 1us));
    }
    
    void addSyncPoint(const SyncPoint& sync) {
        std::lock_guard<std::mutex> lock(sync_mutex_);
        sync_points_.push_back(sync);
        
        // Keep last 1000 sync points
        if (sync_points_.size() > 1000) {
            sync_points_.erase(sync_points_.begin());
        }
        
        // Calculate offset
        if (sync.firmware_time_us > 0 && sync.companion_time_us > 0) {
            double offset = static_cast<double>(sync.companion_time_us - sync.firmware_time_us);
            
            // Update running average
            if (sync_points_.size() == 1) {
                current_offset_us_ = offset;
            } else {
                current_offset_us_ = 0.9 * current_offset_us_ + 0.1 * offset;
            }
            
            // Calculate drift
            if (sync_points_.size() > 1) {
                auto& prev = sync_points_[sync_points_.size() - 2];
                double dt_fw = (sync.firmware_time_us - prev.firmware_time_us) / 1e6;
                double dt_comp = (sync.companion_time_us - prev.companion_time_us) / 1e6;
                if (dt_fw > 0) {
                    drift_ppm_ = (dt_comp - dt_fw) / dt_fw * 1e6;
                }
            }
            
            if (sync_callback_) {
                sync_callback_(sync);
            }
        }
    }
    
    double getTimeOffset() const {
        return current_offset_us_;
    }
    
    bool isTimeSynced() const {
        return std::abs(current_offset_us_) < 1000.0;  // Within 1ms
    }
    
    void rotateBag() {
        closeCurrentBag();
        
        std::string old_path = current_bag_path_;
        std::string new_path = getNextBagPath();
        
        // Open new bag
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = new_path;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        writer_->open(storage_options, converter_options);
        createTopics();
        
        current_bag_path_ = new_path;
        stats_.bags_rotated++;
        
        if (bag_rotate_callback_) {
            bag_rotate_callback_(old_path, new_path);
        }
    }
    
    void closeCurrentBag() {
        writer_.reset();
        writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    }
    
    void startRealtimeStream() {
        // WebSocket/TCP server for real-time streaming
        // Simplified - would use actual WebSocket library
        stream_running_ = true;
        stream_thread_ = std::thread(&Impl::streamLoop, this);
    }
    
    void stopRealtimeStream() {
        stream_running_ = false;
        if (stream_thread_.joinable()) stream_thread_.join();
    }
    
    void broadcastMessage(const TelemetryMessage& msg) {
        // Send to connected WebSocket clients
        // Simplified - would use actual WebSocket library
    }
    
    LogSyncResult synchronizeLogs(const std::string& firmware_log_path,
                                   const std::string& companion_log_path,
                                   const std::string& output_path) {
        LogSyncResult result;
        
        try {
            // Open both bags
            rosbag2_cpp::Reader fw_reader;
            fw_reader.open(firmware_log_path);
            
            rosbag2_cpp::Reader comp_reader;
            comp_reader.open(companion_log_path);
            
            // Open output bag
            rosbag2_cpp::writers::SequentialWriter out_writer;
            rosbag2_storage::StorageOptions out_storage;
            out_storage.uri = output_path;
            out_storage.storage_id = "sqlite3";
            rosbag2_cpp::ConverterOptions out_converter;
            out_converter.input_serialization_format = "cdr";
            out_converter.output_serialization_format = "cdr";
            out_writer.open(out_storage, out_converter);
            
            // Copy topics from firmware log
            while (fw_reader.has_next()) {
                auto msg = fw_reader.read_next();
                out_writer.write(msg);
                result.matched_messages++;
            }
            
            // Copy topics from companion log
            while (comp_reader.has_next()) {
                auto msg = comp_reader.read_next();
                out_writer.write(msg);
                result.matched_messages++;
            }
            
            // Calculate time offsets
            // Would compare timestamps between firmware and companion messages
            result.max_offset_us = 0.0;
            result.avg_offset_us = 0.0;
            result.success = true;
            result.firmware_log_path = firmware_log_path;
            result.companion_log_path = companion_log_path;
            result.synced_log_path = output_path;
            
        } catch (const std::exception& e) {
            result.success = false;
        }
        
        return result;
    }
    
    void cleanupOldLogs() {
        if (!config_.auto_cleanup_old) return;
        
        std::filesystem::path log_dir(config_.output_directory);
        auto now = std::chrono::system_clock::now();
        size_t total_size = 0;
        
        // Collect log files with timestamps
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
        
        // Sort by modification time (oldest first)
        std::sort(logs.begin(), logs.end(), 
                  [](const LogFile& a, const LogFile& b) {
                      return a.modified < b.modified;
                  });
        
        // Remove old logs
        auto cutoff = now - std::chrono::hours(24 * config_.max_age_days);
        for (const auto& log : logs) {
            if (log.modified < cutoff || total_size > config_.max_storage_gb * 1024 * 1024 * 1024) {
                std::filesystem::remove(log.path);
                total_size -= log.size;
            }
        }
    }
    
    const Stats& getStats() const {
        return stats_;
    }
    
    void setBagRotateCallback(BagRotateCallback cb) {
        bag_rotate_callback_ = cb;
    }
    
    void setSyncCallback(SyncCallback cb) {
        sync_callback_ = cb;
    }

private:
    void createTopics() {
        for (const auto& topic : config_.topics_to_record) {
            rosbag2_cpp::TopicMetadata topic_meta;
            topic_meta.name = topic;
            topic_meta.type = "swarm_msgs/msg/Unknown";  // Would use actual type
            topic_meta.serialization_format = "cdr";
            writer_->create_topic(topic_meta);
        }
    }
    
    std::string getNextBagPath() {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << config_.output_directory << "/" << config_.bag_prefix 
           << "_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".db3";
        return ss.str();
    }
    
    void writerLoop() {
        while (!should_stop_) {
            TelemetryMessage msg;
            bool has_msg = false;
            
            {
                std::unique_lock<std::mutex> lock(queue_mutex_);
                queue_cv_.wait_for(lock, 10ms, [this] { 
                    return !write_queue_.empty() || should_stop_; 
                });
                
                if (!write_queue_.empty()) {
                    msg = std::move(write_queue_.front());
                    write_queue_.pop();
                    has_msg = true;
                }
            }
            
            if (has_msg) {
                auto start = std::chrono::high_resolution_clock::now();
                
                rosbag2_storage::SerializedBagMessage bag_msg;
                bag_msg.topic_name = msg.topic_name;
                bag_msg.time_stamp = msg.timestamp.nanoseconds();
                bag_msg.serialized_data = msg.serialized_msg.get_rcl_serialized_message().buffer;
                
                writer_->write(bag_msg);
                
                auto end = std::chrono::high_resolution_clock::now();
                double latency = std::chrono::duration<double, std::milli>(end - start).count();
                
                stats_.messages_written++;
                stats_.bytes_written += msg.serialized_msg.size();
                stats_.avg_write_latency_ms = 0.9 * stats_.avg_write_latency_ms + 0.1 * latency;
                
                // Check bag rotation
                checkBagRotation();
            }
        }
        
        // Flush remaining messages
        while (!write_queue_.empty()) {
            TelemetryMessage msg = std::move(write_queue_.front());
            write_queue_.pop();
            
            rosbag2_storage::SerializedBagMessage bag_msg;
            bag_msg.topic_name = msg.topic_name;
            bag_msg.time_stamp = msg.timestamp.nanoseconds();
            bag_msg.serialized_data = msg.serialized_msg.get_rcl_serialized_message().buffer;
            
            writer_->write(bag_msg);
            stats_.messages_written++;
            stats_.bytes_written += msg.serialized_msg.size();
        }
    }
    
    void checkBagRotation() {
        if (stats_.current_bag_size_mb > config_.max_bag_size_mb) {
            rotateBag();
        }
    }
    
    void syncLoop() {
        while (!should_stop_) {
            std::this_thread::sleep_for(1s);
            
            // Periodic sync check
            // Would request sync from firmware
        }
    }
    
    void streamLoop() {
        // WebSocket server loop for real-time streaming
        // Would use actual WebSocket library
        while (stream_running_ && !should_stop_) {
            std::this_thread::sleep_for(100ms);
        }
    }
    
    void cleanupLoop() {
        while (!should_stop_) {
            std::this_thread::sleep_for(1h);
            if (!should_stop_) {
                cleanupOldLogs();
            }
        }
    }
    
    // State
    TelemetryStreamerConfig config_;
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    std::string current_bag_path_;
    
    std::atomic<bool> running_{false};
    std::atomic<bool> should_stop_{false};
    std::atomic<bool> recording_{false};
    std::atomic<bool> stream_running_{false};
    
    std::thread writer_thread_;
    std::thread sync_thread_;
    std::thread stream_thread_;
    std::thread cleanup_thread_;
    
    std::queue<TelemetryMessage> write_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    
    std::vector<SyncPoint> sync_points_;
    std::mutex sync_mutex_;
    double current_offset_us_ = 0.0;
    double drift_ppm_ = 0.0;
    
    std::atomic<bool> stream_running_{false};
    std::thread stream_thread_;
    
    Stats stats_;
    std::mutex stats_mutex_;
    
    std::function<void(const std::string&, const std::string&)> bag_rotate_callback_;
    std::function<void(const SyncPoint&)> sync_callback_;
    
    bool initialized_ = false;
    bool recording_ = false;
    bool has_agent_context_ = false;
};

TelemetryStreamer::TelemetryStreamer() : pimpl_(std::make_unique<Impl>()) {}
TelemetryStreamer::~TelemetryStreamer() = default;
TelemetryStreamer::TelemetryStreamer(TelemetryStreamer&&) noexcept = default;
TelemetryStreamer& TelemetryStreamer::operator=(TelemetryStreamer&&) noexcept = default;

bool TelemetryStreamer::initialize(const TelemetryStreamerConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

void TelemetryStreamer::start() {
    pimpl_->start();
}

void TelemetryStreamer::stop() {
    pimpl_->stop();
}

void TelemetryStreamer::writeMessage(const std::string& topic_name, 
                                     const rclcpp::SerializedMessage& msg,
                                     const rclcpp::Time& timestamp) {
    pimpl_->writeMessage(topic_name, msg, timestamp);
}

void TelemetryStreamer::onAgentState(const swarm_msgs::msg::AgentState::SharedPtr msg) {
    pimpl_->onAgentState(msg);
}

void TelemetryStreamer::onAIInference(const swarm_msgs::msg::AIInference::SharedPtr msg) {
    pimpl_->onAIInference(msg);
}

void TelemetryStreamer::onTaskAllocation(const swarm_msgs::msg::TaskAllocation::SharedPtr msg) {
    pimpl_->onTaskAllocation(msg);
}

void TelemetryStreamer::onSwarmMap(const swarm_msgs::msg::SwarmMap::SharedPtr msg) {
    pimpl_->onSwarmMap(msg);
}

void TelemetryStreamer::onMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
    pimpl_->onMissionStatus(msg);
}

void TelemetryStreamer::onSystemHealth(const swarm_msgs::msg::SystemHealth::SharedPtr msg) {
    pimpl_->onSystemHealth(msg);
}

void TelemetryStreamer::onFormationState(const swarm_msgs::msg::FormationState::SharedPtr msg) {
    pimpl_->onFormationState(msg);
}

void TelemetryStreamer::onCommStatus(const swarm_msgs::msg::CommStatus::SharedPtr msg) {
    pimpl_->onCommStatus(msg);
}

void TelemetryStreamer::onSafetyEvent(const swarm_msgs::msg::SafetyEvent::SharedPtr msg) {
    pimpl_->onSafetyEvent(msg);
}

void TelemetryStreamer::addSyncPoint(const SyncPoint& sync) {
    pimpl_->addSyncPoint(sync);
}

double TelemetryStreamer::getTimeOffset() const {
    return pimpl_->getTimeOffset();
}

bool TelemetryStreamer::isTimeSynced() const {
    return pimpl_->isTimeSynced();
}

void TelemetryStreamer::rotateBag() {
    pimpl_->rotateBag();
}

void TelemetryStreamer::closeCurrentBag() {
    pimpl_->closeCurrentBag();
}

void TelemetryStreamer::startRealtimeStream() {
    pimpl_->startRealtimeStream();
}

void TelemetryStreamer::stopRealtimeStream() {
    pimpl_->stopRealtimeStream();
}

void TelemetryStreamer::broadcastMessage(const TelemetryMessage& msg) {
    pimpl_->broadcastMessage(msg);
}

TelemetryStreamer::LogSyncResult TelemetryStreamer::synchronizeLogs(
    const std::string& firmware_log_path,
    const std::string& companion_log_path,
    const std::string& output_path) {
    return pimpl_->synchronizeLogs(firmware_log_path, companion_log_path, output_path);
}

void TelemetryStreamer::cleanupOldLogs() {
    pimpl_->cleanupOldLogs();
}

const TelemetryStreamer::Stats& TelemetryStreamer::getStats() const {
    return pimpl_->getStats();
}

bool TelemetryStreamer::isInitialized() const {
    return initialized_;
}

bool TelemetryStreamer::isRecording() const {
    return recording_;
}

void TelemetryStreamer::setBagRotateCallback(BagRotateCallback cb) {
    bag_rotate_callback_ = cb;
}

void TelemetryStreamer::setSyncCallback(SyncCallback cb) {
    sync_callback_ = cb;
}

}  // namespace swarm::interface