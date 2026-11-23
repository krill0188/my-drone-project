#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <algorithm>

#include "swarm_interface_telemetry_streamer/log_sync.hpp"

using namespace swarm::interface;

class LogSyncTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        sync_ = std::make_unique<LogSync>();
        
        LogSyncConfig config;
        config.output_directory = "/tmp/test_sync";
        config.bag_prefix = "test_synced";
        config.max_time_offset_us = 100000.0;
        config.max_drift_ppm = 1000.0;
        config.interpolate_timestamps = true;
        config.write_sync_report = true;
        
        EXPECT_TRUE(sync_->initialize(config));
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::unique_ptr<LogSync> sync_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(LogSyncTest, Initialize_ValidConfig_ReturnsTrue) {
    EXPECT_TRUE(sync_->getConfig().output_directory == "/tmp/test_sync");
}

TEST_F(LogSyncTest, Initialize_InvalidOutputDir_CreatesDirectory) {
    auto sync = std::make_unique<LogSync>();
    LogSyncConfig config;
    config.output_directory = "/tmp/test_sync_new_dir";
    
    bool result = sync->initialize(config);
    EXPECT_TRUE(result);
}

// ============================================================================
// Time Offset Estimation Tests
// ============================================================================

TEST_F(LogSyncTest, EstimateTimeOffset_PerfectSync_ReturnsZero) {
    std::vector<uint64_t> fw = {1000000, 2000000, 3000000, 4000000, 5000000};
    std::vector<uint64_t> comp = {1000000, 2000000, 3000000, 4000000, 5000000};
    
    double offset = LogSync::estimateTimeOffset(fw, comp);
    EXPECT_DOUBLE_EQ(offset, 0.0);
}

TEST_F(LogSyncTest, EstimateTimeOffset_ConstantOffset_ReturnsOffset) {
    std::vector<uint64_t> fw = {1000000, 2000000, 3000000, 4000000, 5000000};
    std::vector<uint64_t> comp = {1000005, 2000005, 3000005, 4000005, 5000005};  // +5us
    
    double offset = LogSync::estimateTimeOffset(fw, comp);
    EXPECT_NEAR(offset, 5.0, 0.1);
}

TEST_F(LogSyncTest, EstimateTimeOffset_NegativeOffset) {
    std::vector<uint64_t> fw = {1000000, 2000000, 3000000};
    std::vector<uint64_t> comp = {999990, 1999990, 2999990};  // -10us
    
    double offset = LogSync::estimateTimeOffset(fw, comp);
    EXPECT_NEAR(offset, -10.0, 0.1);
}

TEST_F(LogSyncTest, EstimateTimeOffset_DifferentSizes_ReturnsZero) {
    std::vector<uint64_t> fw = {1000000, 2000000};
    std::vector<uint64_t> comp = {1000000, 2000000, 3000000};
    
    double offset = LogSync::estimateTimeOffset(fw, comp);
    EXPECT_DOUBLE_EQ(offset, 0.0);
}

TEST_F(LogSyncTest, EstimateTimeOffset_EmptyVectors_ReturnsZero) {
    std::vector<uint64_t> fw;
    std::vector<uint64_t> comp;
    
    double offset = LogSync::estimateTimeOffset(fw, comp);
    EXPECT_DOUBLE_EQ(offset, 0.0);
}

// ============================================================================
// Drift Estimation Tests
// ============================================================================

TEST_F(LogSyncTest, EstimateDrift_NoDrift_ReturnsZero) {
    std::vector<uint64_t> fw = {1000000, 2000000, 3000000, 4000000, 5000000};
    std::vector<uint64_t> comp = {1000000, 2000000, 3000000, 4000000, 5000000};
    
    double drift = LogSync::estimateDrift(fw, comp);
    EXPECT_NEAR(drift, 0.0, 1.0);
}

TEST_F(LogSyncTest, EstimateDrift_PositiveDrift) {
    // Companion clock runs 100ppm fast
    std::vector<uint64_t> fw = {1000000, 2000000, 3000000, 4000000, 5000000};
    std::vector<uint64_t> comp = {1000000, 2000100, 3000200, 4000300, 5000400};
    
    double drift = LogSync::estimateDrift(fw, comp);
    EXPECT_NEAR(drift, 100.0, 10.0);  // ~100 ppm
}

TEST_F(LogSyncTest, EstimateDrift_NegativeDrift) {
    // Companion clock runs 50ppm slow
    std::vector<uint64_t> fw = {1000000, 2000000, 3000000, 4000000, 5000000};
    std::vector<uint64_t> comp = {1000000, 1999950, 2999900, 3999850, 4999800};
    
    double drift = LogSync::estimateDrift(fw, comp);
    EXPECT_NEAR(drift, -50.0, 10.0);  // ~-50 ppm
}

TEST_F(LogSyncTest, EstimateDrift_TwoPointsOnly) {
    std::vector<uint64_t> fw = {1000000, 2000000};
    std::vector<uint64_t> comp = {1000000, 2000100};  // 100us over 1s = 100ppm
    
    double drift = LogSync::estimateDrift(fw, comp);
    EXPECT_NEAR(drift, 100.0, 10.0);
}

TEST_F(LogSyncTest, EstimateDrift_InsufficientPoints_ReturnsZero) {
    std::vector<uint64_t> fw = {1000000};
    std::vector<uint64_t> comp = {1000000};
    
    double drift = LogSync::estimateDrift(fw, comp);
    EXPECT_DOUBLE_EQ(drift, 0.0);
}

// ============================================================================
// Time Correction Tests
// ============================================================================

TEST_F(LogSyncTest, ApplyTimeCorrection_NoOffsetNoDrift) {
    uint64_t corrected = LogSync::applyTimeCorrection(
        1500000,  // timestamp
        0.0,      // offset
        0.0,      // drift
        1000000   // reference
    );
    EXPECT_EQ(corrected, 1500000);
}

TEST_F(LogSyncTest, ApplyTimeCorrection_WithOffset) {
    uint64_t corrected = LogSync::applyTimeCorrection(
        1500000,
        100.0,    // +100us offset
        0.0,
        1000000
    );
    EXPECT_EQ(corrected, 1500100);
}

TEST_F(LogSyncTest, ApplyTimeCorrection_WithDrift) {
    // 100ppm drift over 500ms = 50us
    uint64_t corrected = LogSync::applyTimeCorrection(
        1500000,
        0.0,
        100.0,    // 100 ppm
        1000000
    );
    // dt = 500000, drift = 500000 * 100/1e6 = 50us
    EXPECT_NEAR(static_cast<double>(corrected), 1500050.0, 1.0);
}

TEST_F(LogSyncTest, ApplyTimeCorrection_OffsetAndDrift) {
    uint64_t corrected = LogSync::applyTimeCorrection(
        1500000,
        100.0,    // offset
        100.0,    // drift
        1000000
    );
    EXPECT_NEAR(static_cast<double>(corrected), 1500150.0, 1.0);
}

TEST_F(LogSyncTest, ApplyTimeCorrection_ReferenceAtTimestamp) {
    uint64_t corrected = LogSync::applyTimeCorrection(
        1000000,  // timestamp == reference
        100.0,
        100.0,
        1000000
    );
    EXPECT_EQ(corrected, 1000100);  // Only offset applied at reference
}

// ============================================================================
// Topic Mapping Tests
// ============================================================================

TEST_F(LogSyncTest, MapFirmwareTopic_KnownTopics) {
    EXPECT_EQ(LogSync::mapFirmwareTopic("sensor_combined"), "/swarm/sensor_combined");
    EXPECT_EQ(LogSync::mapFirmwareTopic("vehicle_gps_position"), "/swarm/vehicle_gps_position");
    EXPECT_EQ(LogSync::mapFirmwareTopic("vehicle_attitude"), "/swarm/vehicle_attitude");
    EXPECT_EQ(LogSync::mapFirmwareTopic("actuator_outputs"), "/swarm/actuator_outputs");
    EXPECT_EQ(LogSync::mapFirmwareTopic("battery_status"), "/swarm/battery_status");
    EXPECT_EQ(LogSync::mapFirmwareTopic("estimator_status"), "/swarm/estimator_status");
    EXPECT_EQ(LogSync::mapFirmwareTopic("vehicle_status"), "/swarm/vehicle_status");
    EXPECT_EQ(LogSync::mapFirmwareTopic("swarm_formation_state"), "/swarm/formation_state");
    EXPECT_EQ(LogSync::mapFirmwareTopic("swarm_task_allocation"), "/swarm/task_allocation");
    EXPECT_EQ(LogSync::mapFirmwareTopic("swarm_ai_inference"), "/swarm/ai_inference");
    EXPECT_EQ(LogSync::mapFirmwareTopic("swarm_comm_status"), "/swarm/comm_status");
    EXPECT_EQ(LogSync::mapFirmwareTopic("swarm_telemetry_sync"), "/swarm/telemetry_sync");
}

TEST_F(LogSyncTest, MapFirmwareTopic_UnknownTopic) {
    EXPECT_EQ(LogSync::mapFirmwareTopic("unknown_topic"), "/swarm/unknown_topic");
}

TEST_F(LogSyncTest, MapCompanionTopic_AlreadyPrefixed) {
    EXPECT_EQ(LogSync::mapCompanionTopic("/swarm/ai_inference"), "/swarm/ai_inference");
    EXPECT_EQ(LogSync::mapCompanionTopic("/swarm/comm_status"), "/swarm/comm_status");
}

TEST_F(LogSyncTest, MapCompanionTopic_NotPrefixed) {
    EXPECT_EQ(LogSync::mapCompanionTopic("ai_inference"), "/swarm/ai_inference");
    EXPECT_EQ(LogSync::mapCompanionTopic("comm_status"), "/swarm/comm_status");
}

// ============================================================================
// Sync Quality Assessment Tests
// ============================================================================

TEST_F(LogSyncTest, AssessSyncQuality_Excellent) {
    LogSyncResult result;
    result.success = true;
    result.max_offset_us = 500.0;      // < 1ms
    result.max_drift_ppm = 5.0;        // < 10 ppm
    result.matched_messages = 100;
    
    auto quality = LogSync::assessSyncQuality(result);
    
    EXPECT_TRUE(quality.is_synchronized);
    EXPECT_EQ(quality.quality_level, "EXCELLENT");
}

TEST_F(LogSyncTest, AssessSyncQuality_Good) {
    LogSyncResult result;
    result.success = true;
    result.max_offset_us = 5000.0;     // < 10ms
    result.max_drift_ppm = 50.0;       // < 100 ppm
    result.matched_messages = 100;
    
    auto quality = LogSync::assessSyncQuality(result);
    
    EXPECT_TRUE(quality.is_synchronized);
    EXPECT_EQ(quality.quality_level, "GOOD");
}

TEST_F(LogSyncTest, AssessSyncQuality_Fair) {
    LogSyncResult result;
    result.success = true;
    result.max_offset_us = 50000.0;    // < 100ms
    result.max_drift_ppm = 500.0;      // < 1000 ppm
    result.matched_messages = 100;
    
    auto quality = LogSync::assessSyncQuality(result);
    
    EXPECT_TRUE(quality.is_synchronized);
    EXPECT_EQ(quality.quality_level, "FAIR");
}

TEST_F(LogSyncTest, AssessSyncQuality_Poor) {
    LogSyncResult result;
    result.success = true;
    result.max_offset_us = 200000.0;   // > 100ms
    result.max_drift_ppm = 2000.0;     // > 1000 ppm
    result.matched_messages = 100;
    
    auto quality = LogSync::assessSyncQuality(result);
    
    EXPECT_FALSE(quality.is_synchronized);
    EXPECT_EQ(quality.quality_level, "POOR");
}

TEST_F(LogSyncTest, AssessSyncQuality_FailedSync) {
    LogSyncResult result;
    result.success = false;
    result.max_offset_us = 0.0;
    result.max_drift_ppm = 0.0;
    result.matched_messages = 0;
    
    auto quality = LogSync::assessSyncQuality(result);
    
    EXPECT_FALSE(quality.is_synchronized);
    EXPECT_EQ(quality.quality_level, "POOR");
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(LogSyncTest, Config_DefaultValues) {
    auto config = sync_->getConfig();
    
    EXPECT_EQ(config.output_directory, "/tmp/test_sync");
    EXPECT_EQ(config.bag_prefix, "test_synced");
    EXPECT_DOUBLE_EQ(config.max_time_offset_us, 100000.0);
    EXPECT_DOUBLE_EQ(config.max_drift_ppm, 1000.0);
    EXPECT_TRUE(config.interpolate_timestamps);
    EXPECT_TRUE(config.write_sync_report);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(LogSyncTest, SynchronizeLogs_NonExistentFiles) {
    LogSyncResult result = sync_->synchronizeLogs(
        "/non/existent/firmware.ulg",
        "/non/existent/companion.db3",
        "/tmp/output.db3"
    );
    
    EXPECT_FALSE(result.success);
    EXPECT_FALSE(result.error_message.empty());
}

TEST_F(LogSyncTest, GenerateSyncReport_CreatesFile) {
    LogSyncResult result;
    result.success = true;
    result.firmware_log_path = "/tmp/fw.ulg";
    result.companion_log_path = "/tmp/comp.db3";
    result.synced_log_path = "/tmp/synced.db3";
    result.max_offset_us = 100.0;
    result.avg_offset_us = 50.0;
    result.max_drift_ppm = 10.0;
    result.avg_drift_ppm = 5.0;
    result.matched_messages = 100;
    result.total_firmware_messages = 150;
    result.total_companion_messages = 120;
    result.sync_points = {
        {1000000, 1000100, 1000050, 100.0, 10.0, 0},
        {2000000, 2000100, 2000050, 100.0, 10.0, 1}
    };
    
    std::string report_path = "/tmp/test_sync_report.txt";
    sync_->generateSyncReport(result, report_path);
    
    // Verify file exists and has content
    std::ifstream f(report_path);
    EXPECT_TRUE(f.is_open());
    std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    EXPECT_TRUE(content.find("LOG SYNCHRONIZATION REPORT") != std::string::npos);
    EXPECT_TRUE(content.find("EXCELLENT") != std::string::npos);
    
    std::filesystem::remove(report_path);
}

TEST_F(LogSyncTest, SyncPoint_Structure) {
    SyncPoint sp;
    sp.firmware_time_us = 1000000;
    sp.companion_time_us = 1000005;
    sp.gps_time_us = 1000002;
    sp.offset_us = 5.0;
    sp.drift_ppm = 10.0;
    sp.sequence = 0;
    
    EXPECT_EQ(sp.firmware_time_us, 1000000);
    EXPECT_EQ(sp.companion_time_us, 1000005);
    EXPECT_EQ(sp.gps_time_us, 1000002);
    EXPECT_DOUBLE_EQ(sp.offset_us, 5.0);
    EXPECT_DOUBLE_EQ(sp.drift_ppm, 10.0);
    EXPECT_EQ(sp.sequence, 0);
}

TEST_F(LogSyncTest, LogSyncResult_Structure) {
    LogSyncResult result;
    result.success = true;
    result.firmware_log_path = "/tmp/fw.ulg";
    result.companion_log_path = "/tmp/comp.db3";
    result.synced_log_path = "/tmp/synced.db3";
    result.max_offset_us = 100.0;
    result.avg_offset_us = 50.0;
    result.max_drift_ppm = 10.0;
    result.avg_drift_ppm = 5.0;
    result.matched_messages = 100;
    result.total_firmware_messages = 150;
    result.total_companion_messages = 120;
    result.error_message = "";
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.firmware_log_path, "/tmp/fw.ulg");
    EXPECT_EQ(result.matched_messages, 100);
    EXPECT_TRUE(result.error_message.empty());
}

// ============================================================================
// Topic Stats Tests
// ============================================================================

TEST_F(LogSyncTest, TopicStats_DefaultValues) {
    TopicStats stats;
    
    EXPECT_EQ(stats.firmware_count, 0);
    EXPECT_EQ(stats.companion_count, 0);
    EXPECT_DOUBLE_EQ(stats.min_offset_us, 0.0);
    EXPECT_DOUBLE_EQ(stats.max_offset_us, 0.0);
    EXPECT_DOUBLE_EQ(stats.avg_offset_us, 0.0);
    EXPECT_DOUBLE_EQ(stats.min_drift_ppm, 0.0);
    EXPECT_DOUBLE_EQ(stats.max_drift_ppm, 0.0);
    EXPECT_DOUBLE_EQ(stats.avg_drift_ppm, 0.0);
}

// ============================================================================
// Extract Sync Points Tests (Mock)
// ============================================================================

TEST_F(LogSyncTest, ExtractSyncPoints_InvalidPaths_ReturnsEmpty) {
    auto points = sync_->extractSyncPoints(
        "/non/existent/fw.ulg",
        "/non/existent/comp.db3"
    );
    
    EXPECT_TRUE(points.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}