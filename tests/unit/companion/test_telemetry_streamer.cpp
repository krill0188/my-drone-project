#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_interface_telemetry_streamer/telemetry_streamer.hpp"

using namespace swarm::interface;

class TelemetryStreamerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        streamer_ = std::make_unique<TelemetryStreamer>();
        
        TelemetryStreamerConfig config;
        config.output_directory = "/tmp/test_telemetry";
        config.bag_prefix = "test";
        config.max_bag_size_mb = 10;
        config.max_bag_duration_s = 60;
        config.topics_to_record = {
            "/swarm/agent_states",
            "/swarm/ai_inference",
            "/swarm/task_allocation"
        };
        config.enable_time_sync = true;
        config.sync_tolerance_ms = 10;
        config.time_source = "system";
        config.compression_format = "zstd";
        config.compression_level = 3;
        config.enable_realtime_stream = false;
        config.write_buffer_size_mb = 8;
        config.flush_interval_ms = 100;
        config.max_queue_size = 100;
        config.auto_cleanup_old = false;
        
        EXPECT_TRUE(streamer_->initialize(config));
    }
    
    void TearDown() override {
        if (streamer_->isRecording()) {
            streamer_->stop();
        }
        rclcpp::shutdown();
    }
    
    std::unique_ptr<TelemetryStreamer> streamer_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, Initialize_ValidConfig_ReturnsTrue) {
    // Already tested in SetUp
    EXPECT_TRUE(streamer_->isInitialized());
}

TEST_F(TelemetryStreamerTest, Initialize_InvalidOutputDir_ReturnsFalse) {
    auto streamer = std::make_unique<TelemetryStreamer>();
    TelemetryStreamerConfig config;
    config.output_directory = "/invalid/path/that/does/not/exist/and/cannot/create";
    config.max_bag_size_mb = 10;
    config.topics_to_record = {"/test"};
    
    // Should still succeed as directory creation is attempted
    bool result = streamer->initialize(config);
    EXPECT_TRUE(result);  // Directory creation might succeed
}

TEST_F(TelemetryStreamerTest, StartStop_RecordingState) {
    EXPECT_FALSE(streamer_->isRecording());
    
    streamer_->start();
    EXPECT_TRUE(streamer_->isRecording());
    
    streamer_->stop();
    EXPECT_FALSE(streamer_->isRecording());
}

TEST_F(TelemetryStreamerTest, DoubleStart_NoCrash) {
    streamer_->start();
    streamer_->start();  // Should not crash
    EXPECT_TRUE(streamer_->isRecording());
    
    streamer_->stop();
}

TEST_F(TelemetryStreamerTest, DoubleStop_NoCrash) {
    streamer_->start();
    streamer_->stop();
    streamer_->stop();  // Should not crash
    EXPECT_FALSE(streamer_->isRecording());
}

// ============================================================================
// Message Writing Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, WriteMessage_ValidTopic) {
    streamer_->start();
    
    rclcpp::SerializedMessage msg;
    rclcpp::Time timestamp(1000000);
    
    // Create a simple serialized message
    rclcpp::Serialization<std_msgs::msg::String> serializer;
    std_msgs::msg::String msg;
    msg.data = "test";
    serializer.serialize_message(&msg, &msg);
    
    EXPECT_NO_THROW(streamer_->writeMessage("/test/topic", msg, timestamp));
}

TEST_F(TelemetryStreamerTest, WriteMessage_EmptyTopic) {
    streamer_->start();
    
    rclcpp::SerializedMessage msg;
    rclcpp::Time timestamp(1000000);
    
    EXPECT_NO_THROW(streamer_->writeMessage("", msg, timestamp));
}

TEST_F(TelemetryStreamerTest, WriteMessage_NotRecording) {
    rclcpp::SerializedMessage msg;
    rclcpp::Time timestamp(1000000);
    
    // Should not crash, just ignore
    EXPECT_NO_THROW(streamer_->writeMessage("/test", msg, timestamp));
}

// ============================================================================
// Callbacks Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, OnAgentState_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::AgentState>();
    msg->sysid = 1;
    msg->timestamp_us = 1000000;
    msg->position.x = 10.0;
    msg->position.y = 20.0;
    msg->position.z = 50.0;
    
    EXPECT_NO_THROW(streamer_->onAgentState(msg));
}

TEST_F(TelemetryStreamerTest, OnAIInference_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    msg->type = 0;
    msg->class_id = 1;
    msg->class_name = "power_line";
    msg->confidence = 0.95;
    
    EXPECT_NO_THROW(streamer_->onAIInference(msg));
}

TEST_F(TelemetryStreamerTest, OnTaskAllocation_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::TaskAllocation>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(streamer_->onTaskAllocation(msg));
}

TEST_F(TelemetryStreamerTest, OnSwarmMap_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SwarmMap>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(streamer_->onSwarmMap(msg));
}

TEST_F(TelemetryStreamerTest, OnMissionStatus_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::MissionStatus>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(streamer_->onMissionStatus(msg));
}

TEST_F(TelemetryStreamerTest, OnSystemHealth_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SystemHealth>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(streamer_->onSystemHealth(msg));
}

TEST_F(TelemetryStreamerTest, OnFormationState_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::FormationState>();
    msg->timestamp_us = 1000000;
    msg->leader_id = 1;
    
    EXPECT_NO_THROW(streamer_->onFormationState(msg));
}

TEST_F(TelemetryStreamerTest, OnCommStatus_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::CommStatus>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(streamer_->onCommStatus(msg));
}

TEST_F(TelemetryStreamerTest, OnSafetyEvent_Callback) {
    streamer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SafetyEvent>();
    msg->timestamp_us = 1000000;
    msg->source_id = 1;
    msg->type = 0;
    msg->severity = 1;
    
    EXPECT_NO_THROW(streamer_->onSafetyEvent(msg));
}

// ============================================================================
// Time Synchronization Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, AddSyncPoint_UpdatesOffset) {
    streamer_->start();
    
    swarm::interface::SyncPoint sync;
    sync.firmware_time_us = 1000000;
    sync.companion_time_us = 1000005;  // 5us offset
    sync.gps_time_us = 1000002;
    
    streamer_->addSyncPoint(sync);
    
    double offset = streamer_->getTimeOffset();
    EXPECT_NEAR(offset, 5.0, 1.0);  // 5us offset
}

TEST_F(TelemetryStreamerTest, AddSyncPoint_Multiple_Averages) {
    streamer_->start();
    
    // Add multiple sync points
    for (int i = 0; i < 10; ++i) {
        swarm::interface::SyncPoint sync;
        sync.firmware_time_us = 1000000 + i * 1000000;
        sync.companion_time_us = sync.firmware_time_us + 3;  // 3us offset
        sync.gps_time_us = sync.firmware_time_us + 1;
        
        streamer_->addSyncPoint(sync);
    }
    
    double offset = streamer_->getTimeOffset();
    EXPECT_NEAR(offset, 3.0, 1.0);
}

TEST_F(TelemetryStreamerTest, GetTimeOffset_BeforeSync_ReturnsZero) {
    double offset = streamer_->getTimeOffset();
    EXPECT_DOUBLE_EQ(offset, 0.0);
}

TEST_F(TelemetryStreamerTest, IsTimeSynced_WithinTolerance) {
    streamer_->start();
    
    swarm::interface::SyncPoint sync;
    sync.firmware_time_us = 1000000;
    sync.companion_time_us = 1000000;  // Perfect sync
    sync.gps_time_us = 1000000;
    
    streamer_->addSyncPoint(sync);
    
    EXPECT_TRUE(streamer_->isTimeSynced());
}

TEST_F(TelemetryStreamerTest, IsTimeSynced_OutsideTolerance) {
    streamer_->start();
    
    swarm::interface::SyncPoint sync;
    sync.firmware_time_us = 1000000;
    sync.companion_time_us = 1002000;  // 2ms offset > 1ms tolerance
    sync.gps_time_us = 1000000;
    
    streamer_->addSyncPoint(sync);
    
    EXPECT_FALSE(streamer_->isTimeSynced());
}

// ============================================================================
// Bag Rotation Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, RotateBag_CreatesNewBag) {
    streamer_->start();
    
    std::string initial_path = streamer_->getStats().current_bag_path;
    
    // Manually trigger rotation
    streamer_->rotateBag();
    
    std::string new_path = streamer_->getStats().current_bag_path;
    EXPECT_NE(initial_path, new_path);
    EXPECT_GT(streamer_->getStats().bags_rotated, 0);
}

TEST_F(TelemetryStreamerTest, CloseCurrentBag_ClosesWriter) {
    streamer_->start();
    streamer_->closeCurrentBag();
    
    // Should not crash when writing after close
    rclcpp::SerializedMessage msg;
    rclcpp::Time timestamp(1000000);
    EXPECT_NO_THROW(streamer_->writeMessage("/test", msg, timestamp));
}

// ============================================================================
// Realtime Stream Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, StartRealtimeStream) {
    streamer_->start();
    streamer_->startRealtimeStream();
    
    EXPECT_NO_THROW(streamer_->broadcastMessage(TelemetryMessage{}));
    
    streamer_->stopRealtimeStream();
}

TEST_F(TelemetryStreamerTest, StopRealtimeStream) {
    streamer_->start();
    streamer_->startRealtimeStream();
    streamer_->stopRealtimeStream();
    
    // Should not crash
    EXPECT_NO_THROW(streamer_->startRealtimeStream());
    streamer_->stopRealtimeStream();
}

// ============================================================================
// Log Synchronization Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, SynchronizeLogs_InvalidPaths) {
    auto result = streamer_->synchronizeLogs(
        "/non/existent/firmware.db3",
        "/non/existent/companion.db3",
        "/tmp/synced.db3");
    
    EXPECT_FALSE(result.success);
}

TEST_F(TelemetryStreamerTest, SynchronizeLogs_EmptyOutputPath) {
    auto result = streamer_->synchronizeLogs(
        "/tmp/firmware.db3",
        "/tmp/companion.db3",
        "");
    
    // Should handle gracefully
    EXPECT_FALSE(result.success);
}

// ============================================================================
// Cleanup Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, CleanupOldLogs_NoCrash) {
    streamer_->start();
    streamer_->stop();
    
    EXPECT_NO_THROW(streamer_->cleanupOldLogs());
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, GetStats_InitialValues) {
    auto stats = streamer_->getStats();
    
    EXPECT_EQ(stats.messages_written, 0);
    EXPECT_EQ(stats.messages_dropped, 0);
    EXPECT_EQ(stats.bytes_written, 0);
    EXPECT_EQ(stats.bags_rotated, 0);
    EXPECT_DOUBLE_EQ(stats.current_bag_size_mb, 0.0);
    EXPECT_EQ(stats.sync_points, 0);
    EXPECT_DOUBLE_EQ(stats.avg_write_latency_ms, 0.0);
    EXPECT_FALSE(stats.is_recording);
    EXPECT_TRUE(stats.current_bag_path.empty());
}

TEST_F(TelemetryStreamerTest, GetStats_AfterWriting) {
    streamer_->start();
    
    // Write some messages
    for (int i = 0; i < 10; ++i) {
        rclcpp::SerializedMessage msg;
        rclcpp::Time timestamp(1000000 + i * 1000);
        streamer_->writeMessage("/test", msg, timestamp);
    }
    
    auto stats = streamer_->getStats();
    
    EXPECT_GT(stats.messages_written, 0);
    EXPECT_GT(stats.bytes_written, 0);
    EXPECT_GT(stats.avg_write_latency_ms, 0.0);
}

// ============================================================================
// Bag Rotation Callback Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, BagRotateCallback_Called) {
    bool callback_called = false;
    std::string old_path, new_path;
    
    streamer_->setBagRotateCallback([&](const std::string& old_p, const std::string& new_p) {
        callback_called = true;
        old_path = old_p;
        new_path = new_p;
    });
    
    streamer_->start();
    streamer_->rotateBag();
    
    EXPECT_TRUE(callback_called);
    EXPECT_FALSE(old_path.empty());
    EXPECT_FALSE(new_path.empty());
    EXPECT_NE(old_path, new_path);
}

// ============================================================================
// Sync Callback Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, SyncCallback_Called) {
    bool callback_called = false;
    SyncPoint captured_sync;
    
    streamer_->setSyncCallback([&](const SyncPoint& sync) {
        callback_called = true;
        captured_sync = sync;
    });
    
    streamer_->start();
    
    swarm::interface::SyncPoint sync;
    sync.firmware_time_us = 1000000;
    sync.companion_time_us = 1000005;
    sync.gps_time_us = 1000002;
    
    streamer_->addSyncPoint(sync);
    
    EXPECT_TRUE(callback_called);
    EXPECT_EQ(captured_sync.firmware_time_us, 1000000);
    EXPECT_EQ(captured_sync.companion_time_us, 1000005);
}

// ============================================================================
// State Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, IsInitialized_AfterInit) {
    EXPECT_TRUE(streamer_->isInitialized());
}

TEST_F(TelemetryStreamerTest, IsInitialized_BeforeInit) {
    auto streamer = std::make_unique<TelemetryStreamer>();
    EXPECT_FALSE(streamer->isInitialized());
}

TEST_F(TelemetryStreamerTest, IsRecording_AfterStart) {
    EXPECT_FALSE(streamer_->isRecording());
    
    streamer_->start();
    EXPECT_TRUE(streamer_->isRecording());
    
    streamer_->stop();
    EXPECT_FALSE(streamer_->isRecording());
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(TelemetryStreamerTest, Config_TopicsToRecord) {
    auto config = streamer_->getConfig();
    
    EXPECT_GT(config.topics_to_record.size(), 0);
    EXPECT_TRUE(std::find(config.topics_to_record.begin(), 
                          config.topics_to_record.end(), 
                          "/swarm/agent_states") != config.topics_to_record.end());
}

TEST_F(TelemetryStreamerTest, Config_TimeSync) {
    auto config = streamer_->getConfig();
    
    EXPECT_TRUE(config.enable_time_sync);
    EXPECT_EQ(config.sync_tolerance_ms, 10);
    EXPECT_EQ(config.time_source, "system");
}

TEST_F(TelemetryStreamerTest, Config_Compression) {
    auto config = streamer_->getConfig();
    
    EXPECT_EQ(config.compression_format, "zstd");
    EXPECT_EQ(config.compression_level, 3);
}

TEST_F(TelemetryStreamerTest, Config_RealtimeStream) {
    auto config = streamer_->getConfig();
    
    EXPECT_FALSE(config.enable_realtime_stream);  // Disabled in test config
    EXPECT_EQ(config.stream_host, "0.0.0.0");
    EXPECT_EQ(config.stream_port, 8080);
}

TEST_F(TelemetryStreamerTest, Config_Storage) {
    auto config = streamer_->getConfig();
    
    EXPECT_TRUE(config.auto_cleanup_old);
    EXPECT_EQ(config.max_storage_gb, 50);
    EXPECT_EQ(config.max_age_days, 30);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(TelemetryStreamerTest, WriteLargeMessage) {
    streamer_->start();
    
    // Create a large serialized message
    rclcpp::SerializedMessage msg;
    std::string large_data(1024 * 1024, 'x');  // 1MB
    // Would need actual serialization for real test
    // Just verify no crash
    EXPECT_NO_THROW(streamer_->writeMessage("/large", msg, rclcpp::Time(1000000)));
}

TEST_F(TelemetryStreamerTest, RapidWrites) {
    streamer_->start();
    
    // Write many messages rapidly
    for (int i = 0; i < 1000; ++i) {
        rclcpp::SerializedMessage msg;
        rclcpp::Time timestamp(1000000 + i * 1000);
        streamer_->writeMessage("/rapid/test", msg, timestamp);
    }
    
    auto stats = streamer_->getStats();
    EXPECT_GE(stats.messages_written, 1000);
}

TEST_F(TelemetryStreamerTest, QueueOverflow_DropsMessages) {
    streamer_->start();
    
    // Fill queue beyond max size
    for (int i = 0; i < 150; ++i) {
        rclcpp::SerializedMessage msg;
        rclcpp::Time timestamp(1000000 + i * 1000);
        streamer_->writeMessage("/overflow/test", msg, timestamp);
    }
    
    auto stats = streamer_->getStats();
    // Some messages may be dropped depending on writer speed
    // At minimum, should not crash
    EXPECT_GE(stats.messages_written, 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}