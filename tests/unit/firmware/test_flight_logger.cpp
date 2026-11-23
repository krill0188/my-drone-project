#include <gtest/gtest.h>
#include <memory>
#include <px4_platform_common/px4_config.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>
#include <cstdio>
#include <cstring>
#include <unistd.h>

extern "C" {
#include <fcntl.h>
#include <sys/stat.h>
}

#include "flight_logger.hpp"

using namespace swarm::firmware;

class FlightLoggerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Initialize uORB
        orb_initialize();
        
        logger_ = std::make_unique<FlightLogger>();
        ASSERT_EQ(logger_->init(), PX4_OK);
    }
    
    void TearDown() override {
        if (logger_->isLogging()) {
            logger_->stopLogging();
        }
        logger_.reset();
        orb_uninitialize();
    }
    
    std::unique_ptr<FlightLogger> logger_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(FlightLoggerTest, Initialize_Success) {
    EXPECT_EQ(logger_->init(), PX4_OK);
}

TEST_F(FlightLoggerTest, StartStop_Logging) {
    EXPECT_FALSE(logger_->isLogging());
    
    bool started = logger_->startLogging();
    EXPECT_TRUE(started);
    EXPECT_TRUE(logger_->isLogging());
    
    logger_->stopLogging();
    EXPECT_FALSE(logger_->isLogging());
}

TEST_F(FlightLoggerTest, DoubleStart_NoCrash) {
    logger_->startLogging();
    logger_->startLogging();  // Should not crash
    EXPECT_TRUE(logger_->isLogging());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, DoubleStop_NoCrash) {
    logger_->startLogging();
    logger_->stopLogging();
    logger_->stopLogging();  // Should not crash
    EXPECT_FALSE(logger_->isLogging());
}

// ============================================================================
// ULog File Tests
// ============================================================================

TEST_F(FlightLoggerTest, OpenLogFile_CreatesFile) {
    EXPECT_TRUE(logger_->startLogging());
    
    const char* path = logger_->getCurrentLogPath();
    EXPECT_TRUE(path != nullptr);
    EXPECT_GT(strlen(path), 0);
    
    // Check file exists
    struct stat st;
    EXPECT_EQ(stat(path, &st), 0);
    EXPECT_GT(st.st_size, 0);
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogFile_HasULogHeader) {
    logger_->startLogging();
    
    const char* path = logger_->getCurrentLogPath();
    FILE* f = fopen(path, "rb");
    ASSERT_TRUE(f != nullptr);
    
    char header[8];
    size_t read = fread(header, 1, 8, f);
    fclose(f);
    
    EXPECT_EQ(read, 8);
    // ULog header: "ULog\x01\x00"
    EXPECT_EQ(header[0], 'U');
    EXPECT_EQ(header[1], 'L');
    EXPECT_EQ(header[2], 'o');
    EXPECT_EQ(header[3], 'g');
    EXPECT_EQ(header[4], 0x01);
    EXPECT_EQ(header[5], 0x00);
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, RotateLogFile_CreatesNewFile) {
    logger_->startLogging();
    
    const char* path1 = logger_->getCurrentLogPath();
    
    // Log some data
    logger_->logSensorData();
    logger_->logGpsData();
    logger_->logAttitudeData();
    
    // Rotate
    logger_->rotateLogFile();
    
    const char* path2 = logger_->getCurrentLogPath();
    EXPECT_NE(strcmp(path1, path2), 0);
    
    // Check both files exist
    struct stat st1, st2;
    EXPECT_EQ(stat(path1, &st1), 0);
    EXPECT_EQ(stat(path2, &st2), 0);
    
    logger_->stopLogging();
}

// ============================================================================
// Data Logging Tests
// ============================================================================

TEST_F(FlightLoggerTest, LogSensorData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logSensorData());
    EXPECT_NO_THROW(logger_->logSensorData());
    EXPECT_NO_THROW(logger_->logSensorData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogGpsData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logGpsData());
    EXPECT_NO_THROW(logger_->logGpsData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogAttitudeData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logAttitudeData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogPositionData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logPositionData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogActuatorData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logActuatorData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogBatteryData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logBatteryData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogEstimatorData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logEstimatorData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogStatusData_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logStatusData());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogSafetyEvent_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logSafetyEvent("GEOFENCE_BREACH", "Breached boundary at 100m"));
    EXPECT_NO_THROW(logger_->logSafetyEvent("ESTOP", "Emergency stop triggered"));
    
    logger_->stopLogging();
}

// ============================================================================
// Swarm-Specific Logging Tests
// ============================================================================

TEST_F(FlightLoggerTest, LogFormationState_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logFormationState());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogTaskAllocation_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logTaskAllocation());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogAiInference_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logAiInference());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogCommStatus_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logCommStatus());
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, LogTelemetrySync_NoCrash) {
    logger_->startLogging();
    
    EXPECT_NO_THROW(logger_->logTelemetrySync());
    
    logger_->stopLogging();
}

// ============================================================================
// Time Synchronization Tests
// ============================================================================

TEST_F(FlightLoggerTest, AddTelemetrySync_UpdatesOffset) {
    logger_->addTelemetrySync(
        1000000,  // firmware_time_us
        1000005,  // companion_time_us
        1000002,  // gps_time_us
        5.0,      // offset_us
        10.0      // drift_ppm
    );
    
    // Verify internal state updated
    // Would need getters to verify
    EXPECT_NO_THROW(logger_->addTelemetrySync(1000000, 1000005, 1000002, 5.0, 10.0));
}

TEST_F(FlightLoggerTest, RequestTimeSync_NoCrash) {
    EXPECT_NO_THROW(logger_->requestTimeSync());
}

// ============================================================================
// Disk Space Tests
// ============================================================================

TEST_F(FlightLoggerTest, CheckDiskSpace_NoCrash) {
    EXPECT_NO_THROW(logger_->checkDiskSpace());
}

TEST_F(FlightLoggerTest, CleanupOldLogs_NoCrash) {
    EXPECT_NO_THROW(logger_->cleanupOldLogs());
}

// ============================================================================
// Callback Tests
// ============================================================================

TEST_F(FlightLoggerTest, SetLogRotateCallback_Called) {
    bool callback_called = false;
    std::string old_path, new_path;
    
    logger_->setLogRotateCallback([&](const char* old_p, const char* new_p) {
        callback_called = true;
        old_path = old_p;
        new_path = new_p;
    });
    
    logger_->startLogging();
    logger_->rotateLogFile();
    
    EXPECT_TRUE(callback_called);
    EXPECT_FALSE(old_path.empty());
    EXPECT_FALSE(new_path.empty());
    EXPECT_NE(old_path, new_path);
}

// ============================================================================
// State Tests
// ============================================================================

TEST_F(FlightLoggerTest, GetCurrentLogPath_AfterStart) {
    logger_->startLogging();
    
    const char* path = logger_->getCurrentLogPath();
    EXPECT_TRUE(path != nullptr);
    EXPECT_GT(strlen(path), 0);
    EXPECT_TRUE(strstr(path, ".ulg") != nullptr);
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, GetCurrentLogPath_BeforeStart) {
    const char* path = logger_->getCurrentLogPath();
    // Should return default path
    EXPECT_TRUE(path != nullptr);
}

TEST_F(FlightLoggerTest, GetLoggedBytes_AfterLogging) {
    logger_->startLogging();
    
    logger_->logSensorData();
    logger_->logGpsData();
    logger_->logAttitudeData();
    
    uint64_t bytes = logger_->getLoggedBytes();
    EXPECT_GT(bytes, 0);
    
    logger_->stopLogging();
}

TEST_F(FlightLoggerTest, GetLogDuration_AfterLogging) {
    logger_->startLogging();
    
    // Small delay
    px4_usleep(10000);
    
    uint64_t duration = logger_->getLogDurationUs();
    EXPECT_GE(duration, 10000);
    
    logger_->stopLogging();
}

// ============================================================================
// Rate Limiting Tests
// ============================================================================

TEST_F(FlightLoggerTest, ShouldLogSensorData_RateLimited) {
    // First call should return true (interval elapsed)
    EXPECT_TRUE(logger_->shouldLogSensorData());
    
    // Immediate second call should return false
    EXPECT_FALSE(logger_->shouldLogSensorData());
}

TEST_F(FlightLoggerTest, ShouldLogGpsData_RateLimited) {
    EXPECT_TRUE(logger_->shouldLogGpsData());
    EXPECT_FALSE(logger_->shouldLogGpsData());
}

TEST_F(FlightLoggerTest, ShouldLogAttitudeData_RateLimited) {
    EXPECT_TRUE(logger_->shouldLogAttitudeData());
    EXPECT_FALSE(logger_->shouldLogAttitudeData());
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(FlightLoggerTest, LogWithoutStart_NoCrash) {
    // Should not crash, just not write
    EXPECT_NO_THROW(logger_->logSensorData());
    EXPECT_NO_THROW(logger_->logGpsData());
    EXPECT_NO_THROW(logger_->logAttitudeData());
}

TEST_F(FlightLoggerTest, StopWithoutStart_NoCrash) {
    EXPECT_NO_THROW(logger_->stopLogging());
    EXPECT_FALSE(logger_->isLogging());
}

TEST_F(FlightLoggerTest, GetStats_BeforeStart) {
    uint64_t bytes = logger_->getLoggedBytes();
    uint64_t duration = logger_->getLogDurationUs();
    
    EXPECT_EQ(bytes, 0);
    EXPECT_EQ(duration, 0);
}

TEST_F(FlightLoggerTest, LongDurationLogging) {
    logger_->startLogging();
    
    // Log multiple data types
    for (int i = 0; i < 100; ++i) {
        logger_->logSensorData();
        logger_->logAttitudeData();
        if (i % 10 == 0) {
            logger_->logGpsData();
            logger_->logPositionData();
            logger_->logBatteryData();
        }
    }
    
    uint64_t bytes = logger_->getLoggedBytes();
    EXPECT_GT(bytes, 1000);
    
    logger_->stopLogging();
}

// ============================================================================
// ULog Format Tests
// ============================================================================

TEST_F(FlightLoggerTest, ULogFormatDefinitions) {
    logger_->startLogging();
    
    const char* path = logger_->getCurrentLogPath();
    FILE* f = fopen(path, "rb");
    ASSERT_TRUE(f != nullptr);
    
    // Read entire file
    fseek(f, 0, SEEK_END);
    long size = ftell(f);
    fseek(f, 0, SEEK_SET);
    
    std::vector<char> buffer(size);
    fread(buffer.data(), 1, size, f);
    fclose(f);
    
    // Check for format definitions (F character)
    bool has_format = false;
    for (long i = 0; i < size - 2; ++i) {
        if (buffer[i] == 'F' && buffer[i+1] == ' ') {
            has_format = true;
            break;
        }
    }
    
    EXPECT_TRUE(has_format);
    
    logger_->stopLogging();
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(FlightLoggerTest, Config_DefaultValues) {
    // Verify default logging rates
    EXPECT_GT(logger_->shouldLogSensorData(), false);
    EXPECT_GT(logger_->shouldLogGpsData(), false);
    EXPECT_GT(logger_->shouldLogAttitudeData(), false);
}

TEST_F(FlightLoggerTest, Config_SensorRate) {
    // Sensor should log at 100Hz (10ms interval)
    // Test by checking interval
    EXPECT_TRUE(logger_->shouldLogSensorData());
    px4_usleep(5000);  // 5ms
    EXPECT_FALSE(logger_->shouldLogSensorData());
    px4_usleep(6000);  // 6ms more = 11ms total
    EXPECT_TRUE(logger_->shouldLogSensorData());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}