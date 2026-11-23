#include "log_sync.hpp"
#include <chrono>
#include <filesystem>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>

namespace swarm::interface {

LogSync::LogSync() = default;
LogSync::~LogSync() = default;

bool LogSync::initialize(const LogSyncConfig& config) {
    _config = config;
    std::filesystem::create_directories(_config.output_directory);
    _initialized = true;
    return true;
}

LogSyncResult LogSync::synchronizeLogs(
    const std::string& firmware_log_path,
    const std::string& companion_log_path,
    const std::string& output_path) {
    
    LogSyncResult result;
    result.firmware_log_path = firmware_log_path;
    result.companion_log_path = companion_log_path;
    
    if (!_initialized) {
        result.error_message = "LogSync not initialized";
        return result;
    }
    
    // Check input files exist
    if (!std::filesystem::exists(firmware_log_path)) {
        result.error_message = "Firmware log not found: " + firmware_log_path;
        return result;
    }
    if (!std::filesystem::exists(companion_log_path)) {
        result.error_message = "Companion log not found: " + companion_log_path;
        return result;
    }
    
    try {
        // Read all messages from both logs
        auto firmware_msgs = readAllMessages(firmware_log_path, true);
        auto companion_msgs = readAllMessages(companion_log_path, false);
        
        result.total_firmware_messages = firmware_msgs.size();
        result.total_companion_messages = companion_msgs.size();
        
        if (firmware_msgs.empty() || companion_msgs.empty()) {
            result.error_message = "One or both logs are empty";
            return result;
        }
        
        // Match messages by topic and time
        auto matches = matchMessages(firmware_msgs, companion_msgs, _config.max_time_offset_us);
        
        if (matches.empty()) {
            result.error_message = "No matching messages found between logs";
            return result;
        }
        
        result.matched_messages = matches.size();
        
        // Compute synchronization parameters
        double offset_us = 0.0;
        double drift_ppm = 0.0;
        computeSyncParameters(matches, offset_us, drift_ppm, result.sync_points);
        
        // Calculate statistics
        std::vector<double> offsets;
        std::vector<double> drifts;
        offsets.reserve(result.sync_points.size());
        drifts.reserve(result.sync_points.size());
        
        for (const auto& sp : result.sync_points) {
            offsets.push_back(std::abs(sp.offset_us));
            drifts.push_back(std::abs(sp.drift_ppm));
        }
        
        if (!offsets.empty()) {
            result.max_offset_us = *std::max_element(offsets.begin(), offsets.end());
            result.avg_offset_us = std::accumulate(offsets.begin(), offsets.end(), 0.0) / offsets.size();
        }
        if (!drifts.empty()) {
            result.max_drift_ppm = *std::max_element(drifts.begin(), drifts.end());
            result.avg_drift_ppm = std::accumulate(drifts.begin(), drifts.end(), 0.0) / drifts.size();
        }
        
        // Generate output path if not provided
        std::string final_output_path = output_path;
        if (final_output_path.empty()) {
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << _config.output_directory << "/" << _config.bag_prefix << "_"
               << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".db3";
            final_output_path = ss.str();
        }
        result.synced_log_path = final_output_path;
        
        // Write synchronized bag
        uint64_t reference_time = 0;
        if (!matches.empty()) {
            reference_time = matches[0].first.timestamp_us;
        }
        
        bool write_success = writeSyncedBag(
            final_output_path,
            firmware_msgs,
            companion_msgs,
            offset_us,
            drift_ppm,
            reference_time);
        
        if (!write_success) {
            result.error_message = "Failed to write synchronized bag";
            return result;
        }
        
        // Generate sync report if enabled
        if (_config.write_sync_report) {
            std::string report_path = final_output_path;
            size_t dot_pos = report_path.rfind('.');
            if (dot_pos != std::string::npos) {
                report_path = report_path.substr(0, dot_pos) + "_sync_report.txt";
            }
            generateSyncReport(result, report_path);
        }
        
        result.success = true;
        
    } catch (const std::exception& e) {
        result.error_message = "Exception during synchronization: " + std::string(e.what());
    }
    
    return result;
}

std::vector<LogSync::MessageRecord> LogSync::readAllMessages(
    const std::string& bag_path,
    bool is_firmware) {
    
    std::vector<MessageRecord> messages;
    
    try {
        rosbag2_cpp::Reader reader;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = bag_path;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        reader.open(storage_options, converter_options);
        
        while (reader.has_next()) {
            auto msg = reader.read_next();
            if (!msg) continue;
            
            MessageRecord record;
            record.topic_name = msg->topic_name;
            record.timestamp_us = msg->time_stamp / 1000;  // Convert ns to us
            record.is_firmware = is_firmware;
            
            if (msg->serialized_data && msg->serialized_data->buffer) {
                record.data.assign(
                    msg->serialized_data->buffer,
                    msg->serialized_data->buffer + msg->serialized_data->buffer_length
                );
            }
            
            messages.push_back(std::move(record));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error reading bag " << bag_path << ": " << e.what() << std::endl;
    }
    
    return messages;
}

std::vector<std::pair<LogSync::MessageRecord, LogSync::MessageRecord>> 
LogSync::matchMessages(
    const std::vector<MessageRecord>& firmware_msgs,
    const std::vector<MessageRecord>& companion_msgs,
    double max_time_diff_us) {
    
    std::vector<std::pair<MessageRecord, MessageRecord>> matches;
    
    // Group messages by topic
    std::map<std::string, std::vector<MessageRecord>> fw_by_topic;
    std::map<std::string, std::vector<MessageRecord>> comp_by_topic;
    
    for (const auto& msg : firmware_msgs) {
        fw_by_topic[mapFirmwareTopic(msg.topic_name)].push_back(msg);
    }
    for (const auto& msg : companion_msgs) {
        comp_by_topic[mapCompanionTopic(msg.topic_name)].push_back(msg);
    }
    
    // Match messages with same mapped topic
    for (const auto& [topic, fw_msgs] : fw_by_topic) {
        auto comp_it = comp_by_topic.find(topic);
        if (comp_it == comp_by_topic.end()) continue;
        
        const auto& comp_msgs = comp_it->second;
        
        // Sort by timestamp
        auto fw_sorted = fw_msgs;
        auto comp_sorted = comp_msgs;
        std::sort(fw_sorted.begin(), fw_sorted.end(),
                  [](const MessageRecord& a, const MessageRecord& b) {
                      return a.timestamp_us < b.timestamp_us;
                  });
        std::sort(comp_sorted.begin(), comp_sorted.end(),
                  [](const MessageRecord& a, const MessageRecord& b) {
                      return a.timestamp_us < b.timestamp_us;
                  });
        
        // Two-pointer matching
        size_t i = 0, j = 0;
        while (i < fw_sorted.size() && j < comp_sorted.size()) {
            int64_t diff = static_cast<int64_t>(fw_sorted[i].timestamp_us) - 
                          static_cast<int64_t>(comp_sorted[j].timestamp_us);
            
            if (std::abs(diff) <= max_time_diff_us) {
                matches.emplace_back(fw_sorted[i], comp_sorted[j]);
                ++i;
                ++j;
            } else if (diff < 0) {
                ++i;
            } else {
                ++j;
            }
        }
    }
    
    return matches;
}

void LogSync::computeSyncParameters(
    const std::vector<std::pair<MessageRecord, MessageRecord>>& matches,
    double& offset_us,
    double& drift_ppm,
    std::vector<SyncPoint>& sync_points) {
    
    if (matches.size() < 2) {
        offset_us = 0.0;
        drift_ppm = 0.0;
        return;
    }
    
    // Extract timestamps
    std::vector<uint64_t> fw_times;
    std::vector<uint64_t> comp_times;
    fw_times.reserve(matches.size());
    comp_times.reserve(matches.size());
    
    for (const auto& match : matches) {
        fw_times.push_back(match.first.timestamp_us);
        comp_times.push_back(match.second.timestamp_us);
    }
    
    // Estimate offset (median of differences)
    std::vector<double> diffs;
    diffs.reserve(matches.size());
    for (size_t i = 0; i < matches.size(); ++i) {
        diffs.push_back(static_cast<double>(comp_times[i]) - static_cast<double>(fw_times[i]));
    }
    std::sort(diffs.begin(), diffs.end());
    offset_us = diffs[diffs.size() / 2];
    
    // Estimate drift using linear regression
    // drift = (Δcomp - Δfw) / Δfw
    if (matches.size() >= 2) {
        double sum_fw = 0, sum_comp = 0, sum_fw2 = 0, sum_fw_comp = 0;
        for (size_t i = 0; i < matches.size(); ++i) {
            double fw = static_cast<double>(fw_times[i]);
            double comp = static_cast<double>(comp_times[i]);
            sum_fw += fw;
            sum_comp += comp;
            sum_fw2 += fw * fw;
            sum_fw_comp += fw * comp;
        }
        
        double n = static_cast<double>(matches.size());
        double denom = n * sum_fw2 - sum_fw * sum_fw;
        
        if (std::abs(denom) > 1e-10) {
            double slope = (n * sum_fw_comp - sum_fw * sum_comp) / denom;
            drift_ppm = (slope - 1.0) * 1e6;
        } else {
            drift_ppm = 0.0;
        }
    } else {
        drift_ppm = 0.0;
    }
    
    // Create sync points (one per 100 matches or at least 10 points)
    size_t step = std::max<size_t>(1, matches.size() / 10);
    for (size_t i = 0; i < matches.size(); i += step) {
        SyncPoint sp;
        sp.firmware_time_us = matches[i].first.timestamp_us;
        sp.companion_time_us = matches[i].second.timestamp_us;
        sp.offset_us = static_cast<double>(matches[i].second.timestamp_us) - 
                       static_cast<double>(matches[i].first.timestamp_us);
        sp.drift_ppm = drift_ppm;
        sp.sequence = i / step;
        sync_points.push_back(sp);
    }
    
    // Ensure last point is included
    if (matches.size() > 1) {
        SyncPoint sp;
        sp.firmware_time_us = matches.back().first.timestamp_us;
        sp.companion_time_us = matches.back().second.timestamp_us;
        sp.offset_us = static_cast<double>(matches.back().second.timestamp_us) - 
                       static_cast<double>(matches.back().first.timestamp_us);
        sp.drift_ppm = drift_ppm;
        sp.sequence = sync_points.size();
        sync_points.push_back(sp);
    }
}

bool LogSync::writeSyncedBag(
    const std::string& output_path,
    const std::vector<MessageRecord>& firmware_msgs,
    const std::vector<MessageRecord>& companion_msgs,
    double offset_us,
    double drift_ppm,
    uint64_t reference_time_us) {
    
    try {
        rosbag2_cpp::writers::SequentialWriter writer;
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = output_path;
        storage_options.storage_id = "sqlite3";
        
        rosbag2_cpp::ConverterOptions converter_options;
        converter_options.input_serialization_format = "cdr";
        converter_options.output_serialization_format = "cdr";
        
        writer.open(storage_options, converter_options);
        
        // Track created topics
        std::map<std::string, bool> topics_created;
        
        // Write firmware messages with corrected timestamps
        for (const auto& msg : firmware_msgs) {
            std::string topic = mapFirmwareTopic(msg.topic_name);
            
            if (!topics_created[topic]) {
                rosbag2_storage::TopicMetadata topic_meta;
                topic_meta.name = topic;
                topic_meta.type = "swarm_msgs/msg/Unknown";  // Would use actual type
                topic_meta.serialization_format = "cdr";
                topic_meta.offered_qos_profiles = "";
                writer.create_topic(topic_meta);
                topics_created[topic] = true;
            }
            
            // Apply time correction
            uint64_t corrected_time_ns = applyTimeCorrection(
                msg.timestamp_us, offset_us, drift_ppm, reference_time_us) * 1000;
            
            auto bag_msg = std::make_unique<rosbag2_storage::SerializedBagMessage>();
            bag_msg->topic_name = topic;
            bag_msg->time_stamp = corrected_time_ns;
            bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            bag_msg->serialized_data->buffer = const_cast<uint8_t*>(msg.data.data());
            bag_msg->serialized_data->buffer_length = msg.data.size();
            bag_msg->serialized_data->buffer_capacity = msg.data.size();
            
            writer.write(bag_msg);
        }
        
        // Write companion messages (already in ROS 2 time)
        for (const auto& msg : companion_msgs) {
            std::string topic = mapCompanionTopic(msg.topic_name);
            
            if (!topics_created[topic]) {
                rosbag2_storage::TopicMetadata topic_meta;
                topic_meta.name = topic;
                topic_meta.type = "swarm_msgs/msg/Unknown";
                topic_meta.serialization_format = "cdr";
                topic_meta.offered_qos_profiles = "";
                writer.create_topic(topic_meta);
                topics_created[topic] = true;
            }
            
            auto bag_msg = std::make_unique<rosbag2_storage::SerializedBagMessage>();
            bag_msg->topic_name = topic;
            bag_msg->time_stamp = msg.timestamp_us * 1000;
            bag_msg->serialized_data = std::make_shared<rcutils_uint8_array_t>();
            bag_msg->serialized_data->buffer = const_cast<uint8_t*>(msg.data.data());
            bag_msg->serialized_data->buffer_length = msg.data.size();
            bag_msg->serialized_data->buffer_capacity = msg.data.size();
            
            writer.write(bag_msg);
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error writing synced bag: " << e.what() << std::endl;
        return false;
    }
}

uint64_t LogSync::applyTimeCorrection(
    uint64_t timestamp_us,
    double offset_us,
    double drift_ppm,
    uint64_t reference_time_us) {
    
    if (!_initialized) return timestamp_us;
    
    // corrected = (timestamp - reference) * (1 + drift) + reference + offset
    double dt = static_cast<double>(timestamp_us) - static_cast<double>(reference_time_us);
    double corrected = dt * (1.0 + drift_ppm / 1e6) + static_cast<double>(reference_time_us) + offset_us;
    
    return static_cast<uint64_t>(std::round(corrected));
}

double LogSync::estimateTimeOffset(
    const std::vector<uint64_t>& firmware_timestamps,
    const std::vector<uint64_t>& companion_timestamps) {
    
    if (firmware_timestamps.size() != companion_timestamps.size() || 
        firmware_timestamps.empty()) {
        return 0.0;
    }
    
    std::vector<double> diffs;
    diffs.reserve(firmware_timestamps.size());
    for (size_t i = 0; i < firmware_timestamps.size(); ++i) {
        diffs.push_back(static_cast<double>(companion_timestamps[i]) - 
                       static_cast<double>(firmware_timestamps[i]));
    }
    std::sort(diffs.begin(), diffs.end());
    return diffs[diffs.size() / 2];
}

double LogSync::estimateDrift(
    const std::vector<uint64_t>& firmware_timestamps,
    const std::vector<uint64_t>& companion_timestamps) {
    
    if (firmware_timestamps.size() != companion_timestamps.size() || 
        firmware_timestamps.size() < 2) {
        return 0.0;
    }
    
    double sum_fw = 0, sum_comp = 0, sum_fw2 = 0, sum_fw_comp = 0;
    for (size_t i = 0; i < firmware_timestamps.size(); ++i) {
        double fw = static_cast<double>(firmware_timestamps[i]);
        double comp = static_cast<double>(companion_timestamps[i]);
        sum_fw += fw;
        sum_comp += comp;
        sum_fw2 += fw * fw;
        sum_fw_comp += fw * comp;
    }
    
    double n = static_cast<double>(firmware_timestamps.size());
    double denom = n * sum_fw2 - sum_fw * sum_fw;
    
    if (std::abs(denom) < 1e-10) return 0.0;
    
    double slope = (n * sum_fw_comp - sum_fw * sum_comp) / denom;
    return (slope - 1.0) * 1e6;
}

std::vector<LogSync::SyncPoint> LogSync::extractSyncPoints(
    const std::string& firmware_log_path,
    const std::string& companion_log_path) {
    
    auto firmware_msgs = readAllMessages(firmware_log_path, true);
    auto companion_msgs = readAllMessages(companion_log_path, false);
    
    auto matches = matchMessages(firmware_msgs, companion_msgs, _config.max_time_offset_us);
    
    double offset_us, drift_ppm;
    std::vector<SyncPoint> sync_points;
    computeSyncParameters(matches, offset_us, drift_ppm, sync_points);
    
    return sync_points;
}

LogSync::SyncQuality LogSync::assessSyncQuality(const LogSyncResult& result) {
    SyncQuality quality;
    quality.max_offset_us = result.max_offset_us;
    quality.avg_offset_us = result.avg_offset_us;
    quality.max_drift_ppm = result.max_drift_ppm;
    quality.avg_drift_ppm = result.avg_drift_ppm;
    quality.matched_pairs = result.matched_messages;
    
    // Quality thresholds
    if (result.max_offset_us < 1000.0 && result.max_drift_ppm < 10.0) {
        quality.quality_level = "EXCELLENT";
        quality.is_synchronized = true;
    } else if (result.max_offset_us < 10000.0 && result.max_drift_ppm < 100.0) {
        quality.quality_level = "GOOD";
        quality.is_synchronized = true;
    } else if (result.max_offset_us < 100000.0 && result.max_drift_ppm < 1000.0) {
        quality.quality_level = "FAIR";
        quality.is_synchronized = true;
    } else {
        quality.quality_level = "POOR";
        quality.is_synchronized = false;
    }
    
    return quality;
}

void LogSync::generateSyncReport(const LogSyncResult& result, 
                                 const std::string& report_path) const {
    std::ofstream report(report_path);
    if (!report.is_open()) return;
    
    report << "========================================\n";
    report << "LOG SYNCHRONIZATION REPORT\n";
    report << "========================================\n\n";
    
    report << "Firmware Log: " << result.firmware_log_path << "\n";
    report << "Companion Log: " << result.companion_log_path << "\n";
    report << "Synced Log: " << result.synced_log_path << "\n";
    report << "Success: " << (result.success ? "YES" : "NO") << "\n\n";
    
    if (!result.success) {
        report << "Error: " << result.error_message << "\n";
        return;
    }
    
    report << "----------------------------------------\n";
    report << "MESSAGE STATISTICS\n";
    report << "----------------------------------------\n";
    report << "Firmware Messages: " << result.total_firmware_messages << "\n";
    report << "Companion Messages: " << result.total_companion_messages << "\n";
    report << "Matched Pairs: " << result.matched_messages << "\n";
    report << "Match Rate: " 
           << (result.total_firmware_messages > 0 ? 
               (100.0 * result.matched_messages / result.total_firmware_messages) : 0.0)
           << "%\n\n";
    
    report << "----------------------------------------\n";
    report << "TIME SYNCHRONIZATION\n";
    report << "----------------------------------------\n";
    report << "Max Offset: " << result.max_offset_us << " us\n";
    report << "Avg Offset: " << result.avg_offset_us << " us\n";
    report << "Max Drift: " << result.max_drift_ppm << " ppm\n";
    report << "Avg Drift: " << result.avg_drift_ppm << " ppm\n\n";
    
    report << "----------------------------------------\n";
    report << "SYNC POINTS\n";
    report << "----------------------------------------\n";
    for (const auto& sp : result.sync_points) {
        report << "Seq " << sp.sequence 
               << ": FW=" << sp.firmware_time_us 
               << " Comp=" << sp.companion_time_us 
               << " Offset=" << sp.offset_us << " us"
               << " Drift=" << sp.drift_ppm << " ppm\n";
    }
    
    report << "\n----------------------------------------\n";
    report << "QUALITY ASSESSMENT\n";
    report << "----------------------------------------\n";
    auto quality = assessSyncQuality(result);
    report << "Quality Level: " << quality.quality_level << "\n";
    report << "Synchronized: " << (quality.is_synchronized ? "YES" : "NO") << "\n";
    
    report << "\n========================================\n";
    report << "END OF REPORT\n";
    report << "========================================\n";
}

std::string LogSync::mapFirmwareTopic(const std::string& uorb_topic) {
    // Map uORB topic names to ROS 2 topic names
    static const std::map<std::string, std::string> topic_map = {
        {"sensor_combined", "/swarm/sensor_combined"},
        {"vehicle_gps_position", "/swarm/vehicle_gps_position"},
        {"vehicle_local_position", "/swarm/vehicle_local_position"},
        {"vehicle_attitude", "/swarm/vehicle_attitude"},
        {"vehicle_status", "/swarm/vehicle_status"},
        {"actuator_outputs", "/swarm/actuator_outputs"},
        {"battery_status", "/swarm/battery_status"},
        {"vehicle_air_data", "/swarm/vehicle_air_data"},
        {"estimator_status", "/swarm/estimator_status"},
        {"vehicle_odometry", "/swarm/vehicle_odometry"},
        {"vehicle_command", "/swarm/vehicle_command"},
        {"vehicle_control_mode", "/swarm/vehicle_control_mode"},
        {"manual_control_setpoint", "/swarm/manual_control_setpoint"},
        {"rc_channels", "/swarm/rc_channels"},
        {"safety", "/swarm/safety"},
        {"vehicle_land_detected", "/swarm/vehicle_land_detected"},
        {"vehicle_imu", "/swarm/vehicle_imu"},
        {"vehicle_magnetometer", "/swarm/vehicle_magnetometer"},
        {"vehicle_barometer", "/swarm/vehicle_barometer"},
        {"distance_sensor", "/swarm/distance_sensor"},
        {"optical_flow", "/swarm/optical_flow"},
        {"collision_report", "/swarm/collision_report"},
        {"obstacle_distance", "/swarm/obstacle_distance"},
        {"swarm_formation_state", "/swarm/formation_state"},
        {"swarm_task_allocation", "/swarm/task_allocation"},
        {"swarm_ai_inference", "/swarm/ai_inference"},
        {"swarm_comm_status", "/swarm/comm_status"},
        {"swarm_telemetry_sync", "/swarm/telemetry_sync"},
        {"log_message", "/swarm/log_message"}
    };
    
    auto it = topic_map.find(uorb_topic);
    if (it != topic_map.end()) return it->second;
    return "/swarm/" + uorb_topic;
}

std::string LogSync::mapCompanionTopic(const std::string& ros2_topic) {
    // Companion topics are already ROS 2 format
    if (ros2_topic.rfind("/swarm/", 0) == 0) {
        return ros2_topic;
    }
    return "/swarm/" + ros2_topic;
}

}  // namespace swarm::interface