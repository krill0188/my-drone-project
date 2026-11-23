#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <map>
#include <cmath>

#include "swarm_interface_telemetry_streamer/log_analyzer.hpp"

using namespace swarm::interface;

class LogAnalyzerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        analyzer_ = std::make_unique<LogAnalyzer>();
        
        LogAnalyzerConfig config;
        config.output_directory = "/tmp/test_analysis";
        config.generate_plots = false;
        config.generate_csv = true;
        config.generate_json = true;
        config.detect_anomalies = true;
        config.anomaly_threshold_std = 3.0;
        
        EXPECT_TRUE(analyzer_->initialize(config));
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::unique_ptr<LogAnalyzer> analyzer_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(LogAnalyzerTest, Initialize_ValidConfig_ReturnsTrue) {
    EXPECT_TRUE(analyzer_->getConfig().output_directory == "/tmp/test_analysis");
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(LogAnalyzerTest, ComputeStatistics_EmptyVector_ReturnsZeros) {
    std::vector<double> values;
    auto stats = LogAnalyzer::computeStatistics(values);
    
    EXPECT_EQ(stats.count, 0);
    EXPECT_DOUBLE_EQ(stats.mean, 0.0);
    EXPECT_DOUBLE_EQ(stats.std_dev, 0.0);
}

TEST_F(LogAnalyzerTest, ComputeStatistics_SingleValue) {
    std::vector<double> values = {5.0};
    auto stats = LogAnalyzer::computeStatistics(values);
    
    EXPECT_EQ(stats.count, 1);
    EXPECT_DOUBLE_EQ(stats.mean, 5.0);
    EXPECT_DOUBLE_EQ(stats.std_dev, 0.0);
    EXPECT_DOUBLE_EQ(stats.min, 5.0);
    EXPECT_DOUBLE_EQ(stats.max, 5.0);
    EXPECT_DOUBLE_EQ(stats.median, 5.0);
}

TEST_F(LogAnalyzerTest, ComputeStatistics_MultipleValues) {
    std::vector<double> values = {1.0, 2.0, 3.0, 4.0, 5.0};
    auto stats = LogAnalyzer::computeStatistics(values);
    
    EXPECT_EQ(stats.count, 5);
    EXPECT_DOUBLE_EQ(stats.mean, 3.0);
    EXPECT_NEAR(stats.std_dev, std::sqrt(2.0), 0.01);  // sqrt((4+1+0+1+4)/5) = sqrt(2)
    EXPECT_DOUBLE_EQ(stats.min, 1.0);
    EXPECT_DOUBLE_EQ(stats.max, 5.0);
    EXPECT_DOUBLE_EQ(stats.median, 3.0);
    EXPECT_DOUBLE_EQ(stats.q1, 2.0);
    EXPECT_DOUBLE_EQ(stats.q3, 4.0);
}

TEST_F(LogAnalyzerTest, ComputeStatistics_NegativeValues) {
    std::vector<double> values = {-5.0, -3.0, -1.0, 1.0, 3.0, 5.0};
    auto stats = LogAnalyzer::computeStatistics(values);
    
    EXPECT_DOUBLE_EQ(stats.mean, 0.0);
    EXPECT_DOUBLE_EQ(stats.min, -5.0);
    EXPECT_DOUBLE_EQ(stats.max, 5.0);
    EXPECT_DOUBLE_EQ(stats.median, 0.0);  // Average of -1 and 1
}

// ============================================================================
// Anomaly Detection Tests
// ============================================================================

TEST_F(LogAnalyzerTest, CheckValueAnomaly_WithinRange_NoAnomaly) {
    std::vector<AnomalyEvent> anomalies;
    Statistics stats;
    stats.mean = 10.0;
    stats.std_dev = 1.0;
    
    analyzer_->checkValueAnomaly("test_topic", "test_field", 11.0, 1000000, stats, anomalies);
    
    EXPECT_TRUE(anomalies.empty());
}

TEST_F(LogAnalyzerTest, CheckValueAnomaly_AboveThreshold_DetectsAnomaly) {
    std::vector<AnomalyEvent> anomalies;
    Statistics stats;
    stats.mean = 10.0;
    stats.std_dev = 1.0;
    
    // Value at 15.0, threshold = 3.0 * 1.0 = 3.0, range = [7, 13]
    analyzer_->checkValueAnomaly("test_topic", "test_field", 15.0, 1000000, stats, anomalies);
    
    EXPECT_EQ(anomalies.size(), 1);
    EXPECT_EQ(anomalies[0].topic, "test_topic");
    EXPECT_EQ(anomalies[0].field, "test_field");
    EXPECT_DOUBLE_EQ(anomalies[0].value, 15.0);
    EXPECT_DOUBLE_EQ(anomalies[0].expected_min, 7.0);
    EXPECT_DOUBLE_EQ(anomalies[0].expected_max, 13.0);
    EXPECT_GT(anomalies[0].severity, 0.0);
}

TEST_F(LogAnalyzerTest, CheckValueAnomaly_BelowThreshold_DetectsAnomaly) {
    std::vector<AnomalyEvent> anomalies;
    Statistics stats;
    stats.mean = 10.0;
    stats.std_dev = 1.0;
    
    analyzer_->checkValueAnomaly("test_topic", "test_field", 5.0, 1000000, stats, anomalies);
    
    EXPECT_EQ(anomalies.size(), 1);
    EXPECT_DOUBLE_EQ(anomalies[0].value, 5.0);
}

// ============================================================================
// Time Series Extraction Tests
// ============================================================================

TEST_F(LogAnalyzerTest, ExtractTimeSeries_FindsMatchingMessages) {
    std::vector<MessageRecord> messages;
    
    for (int i = 0; i < 5; ++i) {
        MessageRecord msg;
        msg.topic_name = "/swarm/vehicle_local_position";
        msg.timestamp_us = 1000000 + i * 1000000;
        msg.parsed_fields["z"] = -10.0 - i;  // Altitude decreasing
        messages.push_back(msg);
    }
    
    auto ts = analyzer_->extractTimeSeries(messages, "vehicle_local_position", "z");
    
    EXPECT_EQ(ts.size(), 5);
    for (size_t i = 0; i < ts.size(); ++i) {
        EXPECT_EQ(ts[i].first, 1000000 + i * 1000000);
        EXPECT_DOUBLE_EQ(ts[i].second, -10.0 - i);
    }
}

TEST_F(LogAnalyzerTest, ExtractTimeSeries_WrongTopic_ReturnsEmpty) {
    std::vector<MessageRecord> messages;
    
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_attitude";
    msg.timestamp_us = 1000000;
    msg.parsed_fields["roll"] = 0.1;
    messages.push_back(msg);
    
    auto ts = analyzer_->extractTimeSeries(messages, "vehicle_local_position", "z");
    
    EXPECT_TRUE(ts.empty());
}

TEST_F(LogAnalyzerTest, ExtractTimeSeries_WrongField_ReturnsEmpty) {
    std::vector<MessageRecord> messages;
    
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_local_position";
    msg.timestamp_us = 1000000;
    msg.parsed_fields["x"] = 10.0;
    messages.push_back(msg);
    
    auto ts = analyzer_->extractTimeSeries(messages, "vehicle_local_position", "z");
    
    EXPECT_TRUE(ts.empty());
}

// ============================================================================
// Flight Phase Detection Tests
// ============================================================================

TEST_F(LogAnalyzerTest, DetectFlightPhase_Preflight) {
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_local_position";
    msg.parsed_fields["z"] = -0.5;  // Near ground
    msg.parsed_fields["vz"] = 0.1;
    
    auto phase = analyzer_->detectFlightPhase(msg, nullptr);
    EXPECT_EQ(phase, LogAnalyzer::FlightPhase::PREFLIGHT);
}

TEST_F(LogAnalyzerTest, DetectFlightPhase_Climb) {
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_local_position";
    msg.parsed_fields["z"] = -10.0;
    msg.parsed_fields["vz"] = -2.0;  // Climbing (negative in NED)
    
    auto phase = analyzer_->detectFlightPhase(msg, nullptr);
    EXPECT_EQ(phase, LogAnalyzer::FlightPhase::CLIMB);
}

TEST_F(LogAnalyzerTest, DetectFlightPhase_Cruise) {
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_local_position";
    msg.parsed_fields["z"] = -50.0;
    msg.parsed_fields["vz"] = 0.1;
    
    auto phase = analyzer_->detectFlightPhase(msg, nullptr);
    EXPECT_EQ(phase, LogAnalyzer::FlightPhase::CRUISE);
}

TEST_F(LogAnalyzerTest, DetectFlightPhase_Descent) {
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_local_position";
    msg.parsed_fields["z"] = -20.0;
    msg.parsed_fields["vz"] = 2.0;  // Descending (positive in NED)
    
    auto phase = analyzer_->detectFlightPhase(msg, nullptr);
    EXPECT_EQ(phase, LogAnalyzer::FlightPhase::DESCENT);
}

TEST_F(LogAnalyzerTest, DetectFlightPhase_Landing) {
    MessageRecord msg;
    msg.topic_name = "/swarm/vehicle_local_position";
    msg.parsed_fields["z"] = -1.0;
    msg.parsed_fields["vz"] = 0.5;
    
    auto phase = analyzer_->detectFlightPhase(msg, nullptr);
    EXPECT_EQ(phase, LogAnalyzer::FlightPhase::LANDING);
}

// ============================================================================
// Message Parsing Tests (Mock)
// ============================================================================

TEST_F(LogAnalyzerTest, ParseVehicleLocalPosition_ValidData) {
    std::vector<uint8_t> data(48);
    double values[6] = {10.0, 20.0, -30.0, 1.0, 2.0, -0.5};
    std::memcpy(data.data(), values, 48);
    
    std::map<std::string, double> fields;
    analyzer_->parseVehicleLocalPosition(data, fields);
    
    EXPECT_DOUBLE_EQ(fields["x"], 10.0);
    EXPECT_DOUBLE_EQ(fields["y"], 20.0);
    EXPECT_DOUBLE_EQ(fields["z"], -30.0);
    EXPECT_DOUBLE_EQ(fields["vx"], 1.0);
    EXPECT_DOUBLE_EQ(fields["vy"], 2.0);
    EXPECT_DOUBLE_EQ(fields["vz"], -0.5);
}

TEST_F(LogAnalyzerTest, ParseVehicleAttitude_ValidData) {
    std::vector<uint8_t> data(80);  // 10 doubles
    double values[10] = {1,0,0,0, 0.1, 0.2, 0.3, 0.01, 0.02, 0.03};
    std::memcpy(data.data(), values, 80);
    
    std::map<std::string, double> fields;
    analyzer_->parseVehicleAttitude(data, fields);
    
    EXPECT_DOUBLE_EQ(fields["roll"], 0.1);
    EXPECT_DOUBLE_EQ(fields["pitch"], 0.2);
    EXPECT_DOUBLE_EQ(fields["yaw"], 0.3);
    EXPECT_DOUBLE_EQ(fields["roll_rate"], 0.01);
    EXPECT_DOUBLE_EQ(fields["pitch_rate"], 0.02);
    EXPECT_DOUBLE_EQ(fields["yaw_rate"], 0.03);
}

TEST_F(LogAnalyzerTest, ParseBatteryStatus_ValidData) {
    std::vector<uint8_t> data(16);
    double values[2] = {22.2, 15.5};
    std::memcpy(data.data(), values, 16);
    
    std::map<std::string, double> fields;
    analyzer_->parseBatteryStatus(data, fields);
    
    EXPECT_DOUBLE_EQ(fields["voltage"], 22.2);
    EXPECT_DOUBLE_EQ(fields["current"], 15.5);
}

TEST_F(LogAnalyzerTest, ParseActuatorOutputs_ValidData) {
    std::vector<uint8_t> data(64);
    double values[8] = {0.5, 0.6, 0.7, 0.8, 0.9, 0.8, 0.7, 0.6};
    std::memcpy(data.data(), values, 64);
    
    std::map<std::string, double> fields;
    analyzer_->parseActuatorOutputs(data, fields);
    
    for (int i = 0; i < 8; ++i) {
        EXPECT_DOUBLE_EQ(fields["output" + std::to_string(i)], values[i]);
    }
}

// ============================================================================
// Flight Metrics Analysis Tests
// ============================================================================

TEST_F(LogAnalyzerTest, AnalyzeFlightMetrics_BasicFlight) {
    std::vector<MessageRecord> messages;
    
    // Simulate a simple flight: takeoff, hover, land
    for (int i = 0; i < 100; ++i) {
        MessageRecord msg;
        msg.topic_name = "/swarm/vehicle_local_position";
        msg.timestamp_us = 1000000 + i * 100000;
        msg.parsed_fields["x"] = i * 0.1;
        msg.parsed_fields["y"] = 0.0;
        msg.parsed_fields["z"] = -std::min(i * 0.5, 50.0);  // Climb to 50m
        msg.parsed_fields["vx"] = 0.1;
        msg.parsed_fields["vy"] = 0.0;
        msg.parsed_fields["vz"] = (i < 100) ? -0.5 : 0.0;
        messages.push_back(msg);
    }
    
    auto metrics = analyzer_->analyzeFlightMetrics(messages);
    
    EXPECT_GT(metrics.flight_duration_s, 0.0);
    EXPECT_GT(metrics.total_distance_m, 0.0);
    EXPECT_GT(metrics.max_altitude_m, 0.0);
    EXPECT_NEAR(metrics.max_altitude_m, 50.0, 1.0);
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(LogAnalyzerTest, Config_DefaultValues) {
    auto config = analyzer_->getConfig();
    
    EXPECT_EQ(config.output_directory, "/tmp/test_analysis");
    EXPECT_FALSE(config.generate_plots);
    EXPECT_TRUE(config.generate_csv);
    EXPECT_TRUE(config.generate_json);
    EXPECT_TRUE(config.detect_anomalies);
    EXPECT_DOUBLE_EQ(config.anomaly_threshold_std, 3.0);
}

// ============================================================================
// Report Generation Tests
// ============================================================================

TEST_F(LogAnalyzerTest, GenerateCSVReport_CreatesFile) {
    AnalysisResult result;
    result.success = true;
    result.log_path = "/tmp/test.db3";
    result.metrics.flight_duration_s = 100.0;
    result.metrics.total_distance_m = 500.0;
    result.metrics.max_altitude_m = 100.0;
    
    std::string output_path = "/tmp/test_report.csv";
    analyzer_->generateCSVReport(result, output_path);
    
    // Verify file exists
    std::ifstream f(output_path);
    EXPECT_TRUE(f.is_open());
    
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    EXPECT_TRUE(content.find("Flight Duration") != std::string::npos);
    EXPECT_TRUE(content.find("100") != std::string::npos);
    EXPECT_TRUE(content.find("500") != std::string::npos);
    
    std::filesystem::remove(output_path);
}

TEST_F(LogAnalyzerTest, GenerateJSONReport_CreatesFile) {
    AnalysisResult result;
    result.success = true;
    result.log_path = "/tmp/test.db3";
    result.metrics.flight_duration_s = 100.0;
    result.metrics.max_altitude_m = 100.0;
    
    std::string output_path = "/tmp/test_report.json";
    analyzer_->generateJSONReport(result, output_path);
    
    std::ifstream f(output_path);
    EXPECT_TRUE(f.is_open());
    
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    EXPECT_TRUE(content.find("flight_duration_s") != std::string::npos);
    EXPECT_TRUE(content.find("100") != std::string::npos);
    
    std::filesystem::remove(output_path);
}

TEST_F(LogAnalyzerTest, GenerateSummaryReport_CreatesFile) {
    AnalysisResult result;
    result.success = true;
    result.log_path = "/tmp/test.db3";
    result.metrics.flight_duration_s = 100.0;
    result.metrics.max_altitude_m = 100.0;
    
    std::string output_path = "/tmp/test_report.txt";
    analyzer_->generateSummaryReport(result, output_path);
    
    std::ifstream f(output_path);
    EXPECT_TRUE(f.is_open());
    
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    EXPECT_TRUE(content.find("FLIGHT LOG ANALYSIS SUMMARY") != std::string::npos);
    EXPECT_TRUE(content.find("100") != std::string::npos);
    
    std::filesystem::remove(output_path);
}

// ============================================================================
// AnomalyEvent Structure Tests
// ============================================================================

TEST_F(LogAnalyzerTest, AnomalyEvent_Structure) {
    AnomalyEvent anomaly;
    anomaly.timestamp_us = 1000000;
    anomaly.topic = "/swarm/battery_status";
    anomaly.field = "voltage";
    anomaly.value = 18.0;
    anomaly.expected_min = 20.0;
    anomaly.expected_max = 25.0;
    anomaly.severity = 0.8;
    anomaly.description = "Low battery voltage";
    
    EXPECT_EQ(anomaly.timestamp_us, 1000000);
    EXPECT_EQ(anomaly.topic, "/swarm/battery_status");
    EXPECT_EQ(anomaly.field, "voltage");
    EXPECT_DOUBLE_EQ(anomaly.value, 18.0);
    EXPECT_DOUBLE_EQ(anomaly.expected_min, 20.0);
    EXPECT_DOUBLE_EQ(anomaly.expected_max, 25.0);
    EXPECT_DOUBLE_EQ(anomaly.severity, 0.8);
    EXPECT_EQ(anomaly.description, "Low battery voltage");
}

// ============================================================================
// FlightMetrics Structure Tests
// ============================================================================

TEST_F(LogAnalyzerTest, FlightMetrics_DefaultValues) {
    FlightMetrics metrics;
    
    EXPECT_EQ(metrics.flight_start_us, 0);
    EXPECT_EQ(metrics.flight_end_us, 0);
    EXPECT_DOUBLE_EQ(metrics.flight_duration_s, 0.0);
    EXPECT_DOUBLE_EQ(metrics.total_distance_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_altitude_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.min_altitude_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.avg_altitude_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_velocity_mps, 0.0);
    EXPECT_DOUBLE_EQ(metrics.avg_velocity_mps, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_roll_deg, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_pitch_deg, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_yaw_rate_dps, 0.0);
    EXPECT_DOUBLE_EQ(metrics.min_battery_voltage_v, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_battery_current_a, 0.0);
    EXPECT_DOUBLE_EQ(metrics.avg_battery_current_a, 0.0);
    EXPECT_DOUBLE_EQ(metrics.battery_consumed_mah, 0.0);
    EXPECT_DOUBLE_EQ(metrics.avg_gps_hdop, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_gps_hdop, 0.0);
    EXPECT_EQ(metrics.gps_satellites_avg, 0);
    EXPECT_EQ(metrics.gps_fix_lost_count, 0);
    EXPECT_DOUBLE_EQ(metrics.avg_estimator_innovation, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_estimator_innovation, 0.0);
    EXPECT_EQ(metrics.estimator_reset_count, 0);
    EXPECT_DOUBLE_EQ(metrics.max_actuator_output, 0.0);
    EXPECT_DOUBLE_EQ(metrics.avg_actuator_output, 0.0);
    EXPECT_EQ(metrics.actuator_saturation_count, 0);
    EXPECT_DOUBLE_EQ(metrics.avg_link_quality, 0.0);
    EXPECT_DOUBLE_EQ(metrics.min_link_quality, 0.0);
    EXPECT_EQ(metrics.packet_loss_count, 0);
    EXPECT_DOUBLE_EQ(metrics.avg_latency_ms, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_latency_ms, 0.0);
    EXPECT_EQ(metrics.total_inferences, 0);
    EXPECT_DOUBLE_EQ(metrics.avg_inference_time_ms, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_inference_time_ms, 0.0);
    EXPECT_EQ(metrics.safety_events_count, 0);
    EXPECT_EQ(metrics.geofence_breaches, 0);
    EXPECT_EQ(metrics.emergency_stops, 0);
    EXPECT_DOUBLE_EQ(metrics.avg_formation_error_m, 0.0);
    EXPECT_DOUBLE_EQ(metrics.max_formation_error_m, 0.0);
    EXPECT_EQ(metrics.formation_loss_count, 0);
    EXPECT_EQ(metrics.tasks_allocated, 0);
    EXPECT_EQ(metrics.tasks_completed, 0);
    EXPECT_EQ(metrics.tasks_failed, 0);
    EXPECT_DOUBLE_EQ(metrics.avg_allocation_time_ms, 0.0);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(LogAnalyzerTest, AnalyzeLog_NonExistentFile_ReturnsError) {
    AnalysisResult result = analyzer_->analyzeLog("/non/existent/log.db3");
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST_F(LogAnalyzerTest, AnalyzeSwarmLogs_MultipleLogs) {
    // Just test it doesn't crash with empty list
    auto results = analyzer_->analyzeSwarmLogs({});
    EXPECT_TRUE(results.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}