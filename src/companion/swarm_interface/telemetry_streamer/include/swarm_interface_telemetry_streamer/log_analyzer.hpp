#ifndef LOG_ANALYZER_HPP
#define LOG_ANALYZER_HPP

#include <rosbag2_cpp/reader.hpp>
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
#include <numeric>
#include <optional>

namespace swarm::interface {

struct LogAnalyzerConfig {
    std::string output_directory = "/logs/analysis";
    bool generate_plots = false;
    bool generate_csv = true;
    bool generate_json = true;
    bool detect_anomalies = true;
    double anomaly_threshold_std = 3.0;  // Standard deviations for anomaly detection
};

struct FlightMetrics {
    // Time
    uint64_t flight_start_us = 0;
    uint64_t flight_end_us = 0;
    double flight_duration_s = 0.0;
    
    // Position
    double total_distance_m = 0.0;
    double max_altitude_m = 0.0;
    double min_altitude_m = 0.0;
    double avg_altitude_m = 0.0;
    
    // Velocity
    double max_velocity_mps = 0.0;
    double avg_velocity_mps = 0.0;
    
    // Attitude
    double max_roll_deg = 0.0;
    double max_pitch_deg = 0.0;
    double max_yaw_rate_dps = 0.0;
    
    // Battery
    double min_battery_voltage_v = 0.0;
    double max_battery_current_a = 0.0;
    double avg_battery_current_a = 0.0;
    double battery_consumed_mah = 0.0;
    
    // GPS
    double avg_gps_hdop = 0.0;
    double max_gps_hdop = 0.0;
    int gps_satellites_avg = 0;
    int gps_fix_lost_count = 0;
    
    // Estimator
    double avg_estimator_innovation = 0.0;
    double max_estimator_innovation = 0.0;
    int estimator_reset_count = 0;
    
    // Actuator
    double max_actuator_output = 0.0;
    double avg_actuator_output = 0.0;
    int actuator_saturation_count = 0;
    
    // Communication
    double avg_link_quality = 0.0;
    double min_link_quality = 0.0;
    int packet_loss_count = 0;
    double avg_latency_ms = 0.0;
    double max_latency_ms = 0.0;
    
    // AI Inference
    int total_inferences = 0;
    double avg_inference_time_ms = 0.0;
    double max_inference_time_ms = 0.0;
    std::map<std::string, int> detections_by_class;
    
    // Safety Events
    int safety_events_count = 0;
    std::map<std::string, int> safety_events_by_type;
    int geofence_breaches = 0;
    int emergency_stops = 0;
    
    // Formation
    double avg_formation_error_m = 0.0;
    double max_formation_error_m = 0.0;
    int formation_loss_count = 0;
    
    // Task Allocation
    int tasks_allocated = 0;
    int tasks_completed = 0;
    int tasks_failed = 0;
    double avg_allocation_time_ms = 0.0;
};

struct AnomalyEvent {
    uint64_t timestamp_us = 0;
    std::string topic;
    std::string field;
    double value = 0.0;
    double expected_min = 0.0;
    double expected_max = 0.0;
    double severity = 0.0;  // 0-1
    std::string description;
};

struct AnalysisResult {
    bool success = false;
    std::string log_path;
    FlightMetrics metrics;
    std::vector<AnomalyEvent> anomalies;
    std::map<std::string, std::vector<double>> time_series_data;
    std::string error_message;
    
    // Report paths
    std::string csv_report_path;
    std::string json_report_path;
    std::string summary_report_path;
};

class LogAnalyzer {
public:
    LogAnalyzer();
    ~LogAnalyzer();
    
    LogAnalyzer(const LogAnalyzer&) = delete;
    LogAnalyzer& operator=(const LogAnalyzer&) = delete;
    LogAnalyzer(LogAnalyzer&&) noexcept = default;
    LogAnalyzer& operator=(LogAnalyzer&&) noexcept = default;
    
    bool initialize(const LogAnalyzerConfig& config);
    
    // Analyze a single log file
    AnalysisResult analyzeLog(const std::string& log_path);
    
    // Analyze multiple logs (e.g., swarm)
    std::vector<AnalysisResult> analyzeSwarmLogs(const std::vector<std::string>& log_paths);
    
    // Generate reports
    void generateCSVReport(const AnalysisResult& result, const std::string& output_path) const;
    void generateJSONReport(const AnalysisResult& result, const std::string& output_path) const;
    void generateSummaryReport(const AnalysisResult& result, const std::string& output_path) const;
    
    // Specific analyzers
    FlightMetrics analyzeFlightMetrics(const std::vector<MessageRecord>& messages);
    std::vector<AnomalyEvent> detectAnomalies(const std::vector<MessageRecord>& messages);
    
    // Time series extraction
    std::vector<std::pair<uint64_t, double>> extractTimeSeries(
        const std::vector<MessageRecord>& messages,
        const std::string& topic,
        const std::string& field);
    
    // Statistical analysis
    struct Statistics {
        double mean = 0.0;
        double std_dev = 0.0;
        double min = 0.0;
        double max = 0.0;
        double median = 0.0;
        double q1 = 0.0;
        double q3 = 0.0;
        size_t count = 0;
    };
    
    static Statistics computeStatistics(const std::vector<double>& values);
    
    const LogAnalyzerConfig& getConfig() const { return _config; }

private:
    struct MessageRecord {
        std::string topic_name;
        uint64_t timestamp_us;
        std::vector<uint8_t> data;
        std::map<std::string, double> parsed_fields;  // Cached parsed fields
    };
    
    // Read messages from bag
    std::vector<MessageRecord> readMessages(const std::string& bag_path);
    
    // Parse specific message types
    void parseVehicleLocalPosition(const std::vector<uint8_t>& data, 
                                   std::map<std::string, double>& fields);
    void parseVehicleAttitude(const std::vector<uint8_t>& data, 
                              std::map<std::string, double>& fields);
    void parseBatteryStatus(const std::vector<uint8_t>& data, 
                            std::map<std::string, double>& fields);
    void parseVehicleGpsPosition(const std::vector<uint8_t>& data, 
                                 std::map<std::string, double>& fields);
    void parseEstimatorStatus(const std::vector<uint8_t>& data, 
                              std::map<std::string, double>& fields);
    void parseActuatorOutputs(const std::vector<uint8_t>& data, 
                              std::map<std::string, double>& fields);
    void parseCommStatus(const std::vector<uint8_t>& data, 
                         std::map<std::string, double>& fields);
    void parseAIInference(const std::vector<uint8_t>& data, 
                          std::map<std::string, double>& fields);
    void parseSafetyEvent(const std::vector<uint8_t>& data, 
                          std::map<std::string, double>& fields);
    void parseFormationState(const std::vector<uint8_t>& data, 
                             std::map<std::string, double>& fields);
    void parseTaskAllocation(const std::vector<uint8_t>& data, 
                             std::map<std::string, double>& fields);
    
    // Anomaly detection helpers
    void checkValueAnomaly(const std::string& topic, const std::string& field,
                           double value, uint64_t timestamp,
                           const Statistics& stats,
                           std::vector<AnomalyEvent>& anomalies);
    
    // Flight phase detection
    enum class FlightPhase {
        UNKNOWN,
        PREFLIGHT,
        TAKEOFF,
        CLIMB,
        CRUISE,
        HOVER,
        DESCENT,
        LANDING,
        POSTFLIGHT
    };
    
    FlightPhase detectFlightPhase(const MessageRecord& msg, 
                                  const MessageRecord* prev_msg);
    
    LogAnalyzerConfig _config;
    bool _initialized = false;
};

}  // namespace swarm::interface

#endif // LOG_ANALYZER_HPP