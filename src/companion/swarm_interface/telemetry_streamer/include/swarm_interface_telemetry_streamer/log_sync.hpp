#ifndef LOG_SYNC_HPP
#define LOG_SYNC_HPP

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <mutex>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace swarm::interface {

struct LogSyncConfig {
    std::string output_directory = "/logs/synced";
    std::string bag_prefix = "swarm_synced";
    double max_time_offset_us = 100000.0;  // 100ms max offset
    double max_drift_ppm = 1000.0;          // 1000 ppm max drift
    bool interpolate_timestamps = true;
    bool write_sync_report = true;
};

struct SyncPoint {
    uint64_t firmware_time_us = 0;
    uint64_t companion_time_us = 0;
    uint64_t gps_time_us = 0;
    double offset_us = 0.0;
    double drift_ppm = 0.0;
    uint64_t sequence = 0;
};

struct LogSyncResult {
    bool success = false;
    std::string firmware_log_path;
    std::string companion_log_path;
    std::string synced_log_path;
    double max_offset_us = 0.0;
    double avg_offset_us = 0.0;
    double max_drift_ppm = 0.0;
    double avg_drift_ppm = 0.0;
    uint64_t matched_messages = 0;
    uint64_t total_firmware_messages = 0;
    uint64_t total_companion_messages = 0;
    std::vector<SyncPoint> sync_points;
    std::string error_message;
};

struct TopicStats {
    uint64_t firmware_count = 0;
    uint64_t companion_count = 0;
    double min_offset_us = 0.0;
    double max_offset_us = 0.0;
    double avg_offset_us = 0.0;
    double min_drift_ppm = 0.0;
    double max_drift_ppm = 0.0;
    double avg_drift_ppm = 0.0;
};

class LogSync {
public:
    LogSync();
    ~LogSync();
    
    LogSync(const LogSync&) = delete;
    LogSync& operator=(const LogSync&) = delete;
    LogSync(LogSync&&) noexcept = default;
    LogSync& operator=(LogSync&&) noexcept = default;
    
    bool initialize(const LogSyncConfig& config);
    
    // Main synchronization function
    LogSyncResult synchronizeLogs(
        const std::string& firmware_log_path,
        const std::string& companion_log_path,
        const std::string& output_path = "");
    
    // Time alignment utilities
    static double estimateTimeOffset(
        const std::vector<uint64_t>& firmware_timestamps,
        const std::vector<uint64_t>& companion_timestamps);
    
    static double estimateDrift(
        const std::vector<uint64_t>& firmware_timestamps,
        const std::vector<uint64_t>& companion_timestamps);
    
    // Apply time correction to messages
    static uint64_t applyTimeCorrection(
        uint64_t timestamp_us,
        double offset_us,
        double drift_ppm,
        uint64_t reference_time_us);
    
    // Generate sync report
    void generateSyncReport(const LogSyncResult& result, 
                            const std::string& report_path) const;
    
    // Extract sync points from logs
    std::vector<SyncPoint> extractSyncPoints(
        const std::string& firmware_log_path,
        const std::string& companion_log_path);
    
    // Validate synchronization quality
    struct SyncQuality {
        bool is_synchronized = false;
        double max_offset_us = 0.0;
        double avg_offset_us = 0.0;
        double max_drift_ppm = 0.0;
        double avg_drift_ppm = 0.0;
        uint64_t matched_pairs = 0;
        std::string quality_level;  // "EXCELLENT", "GOOD", "FAIR", "POOR"
    };
    
    static SyncQuality assessSyncQuality(const LogSyncResult& result);
    
    const LogSyncConfig& getConfig() const { return _config; }

private:
    struct MessageRecord {
        std::string topic_name;
        uint64_t timestamp_us;
        std::vector<uint8_t> data;
        bool is_firmware;
    };
    
    // Read all messages from a bag
    std::vector<MessageRecord> readAllMessages(
        const std::string& bag_path,
        bool is_firmware);
    
    // Match messages by topic and approximate time
    std::vector<std::pair<MessageRecord, MessageRecord>> matchMessages(
        const std::vector<MessageRecord>& firmware_msgs,
        const std::vector<MessageRecord>& companion_msgs,
        double max_time_diff_us = 100000.0);
    
    // Compute time offset and drift from matched pairs
    void computeSyncParameters(
        const std::vector<std::pair<MessageRecord, MessageRecord>>& matches,
        double& offset_us,
        double& drift_ppm,
        std::vector<SyncPoint>& sync_points);
    
    // Write synchronized bag
    bool writeSyncedBag(
        const std::string& output_path,
        const std::vector<MessageRecord>& firmware_msgs,
        const std::vector<MessageRecord>& companion_msgs,
        double offset_us,
        double drift_ppm,
        uint64_t reference_time_us);
    
    // Topic name mapping (firmware uORB -> ROS 2)
    static std::string mapFirmwareTopic(const std::string& uorb_topic);
    static std::string mapCompanionTopic(const std::string& ros2_topic);
    
    LogSyncConfig _config;
    bool _initialized = false;
};

}  // namespace swarm::interface

#endif // LOG_SYNC_HPP