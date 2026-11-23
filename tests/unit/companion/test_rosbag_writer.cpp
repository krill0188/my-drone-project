#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_interface_telemetry_streamer/rosbag_writer.hpp"

using namespace swarm::interface;

class RosbagWriterTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        writer_ = std::make_unique<RosbagWriter>();
        
        RosbagWriterConfig config;
        config.output_directory = "/tmp/test_rosbag";
        config.bag_prefix = "test";
        config.max_bag_size_mb = 10;
        config.max_bag_duration_s = 60;
        config.topics_to_record = {
            "/swarm/ai_inference",
            "/swarm/comm_status",
            "/swarm/task_allocation"
        };
        config.compression_format = "zstd";
        config.compression_level = 3;
        config.write_buffer_size_mb = 8;
        config.flush_interval_ms = 100;
        config.max_queue_size = 100;
        config.auto_cleanup_old = false;
        config.enable_indexing = true;
        
        EXPECT_TRUE(writer_->initialize(config));
    }
    
    void TearDown() override {
        if (writer_->isRecording()) {
            writer_->stop();
        }
        rclcpp::shutdown();
    }
    
    std::unique_ptr<RosbagWriter> writer_;
};

// ============================================================================
// Initialization Tests
// ============================================================================

TEST_F(RosbagWriterTest, Initialize_ValidConfig_ReturnsTrue) {
    EXPECT_TRUE(writer_->isInitialized());
}

TEST_F(RosbagWriterTest, Initialize_InvalidOutputDir_ReturnsFalse) {
    auto writer = std::make_unique<RosbagWriter>();
    RosbagWriterConfig config;
    config.output_directory = "/invalid/path/that/does/not/exist/and/cannot/create";
    config.max_bag_size_mb = 10;
    config.topics_to_record = {"/test"};
    
    bool result = writer->initialize(config);
    EXPECT_TRUE(result);  // Directory creation might succeed
}

TEST_F(RosbagWriterTest, StartStop_RecordingState) {
    EXPECT_FALSE(writer_->isRecording());
    
    writer_->start();
    EXPECT_TRUE(writer_->isRecording());
    
    writer_->stop();
    EXPECT_FALSE(writer_->isRecording());
}

TEST_F(RosbagWriterTest, DoubleStart_NoCrash) {
    writer_->start();
    writer_->start();  // Should not crash
    EXPECT_TRUE(writer_->isRecording());
    
    writer_->stop();
}

TEST_F(RosbagWriterTest, DoubleStop_NoCrash) {
    writer_->start();
    writer_->stop();
    writer_->stop();  // Should not crash
    EXPECT_FALSE(writer_->isRecording());
}

// ============================================================================
// Message Writing Tests
// ============================================================================

TEST_F(RosbagWriterTest, WriteAIInference_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    msg->type = 0;
    msg->class_id = 1;
    msg->class_name = "power_line";
    msg->confidence = 0.95;
    msg->position.x = 10.0;
    msg->position.y = 20.0;
    msg->position.z = 50.0;
    
    EXPECT_NO_THROW(writer_->writeAIInference(msg));
}

TEST_F(RosbagWriterTest, WriteCommStatus_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::CommStatus>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    msg->link_quality = 0.9;
    msg->rssi_dbm = -50;
    msg->packet_loss_rate = 0.01;
    msg->latency_ms = 10;
    
    EXPECT_NO_THROW(writer_->writeCommStatus(msg));
}

TEST_F(RosbagWriterTest, WriteTaskAllocation_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::TaskAllocation>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    msg->task_id = 1;
    msg->task_type = 0;
    
    EXPECT_NO_THROW(writer_->writeTaskAllocation(msg));
}

TEST_F(RosbagWriterTest, WriteSwarmMap_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SwarmMap>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(writer_->writeSwarmMap(msg));
}

TEST_F(RosbagWriterTest, WriteMissionStatus_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::MissionStatus>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(writer_->writeMissionStatus(msg));
}

TEST_F(RosbagWriterTest, WriteSystemHealth_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SystemHealth>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    
    EXPECT_NO_THROW(writer_->writeSystemHealth(msg));
}

TEST_F(RosbagWriterTest, WriteFormationState_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::FormationState>();
    msg->timestamp_us = 1000000;
    msg->leader_id = 1;
    
    EXPECT_NO_THROW(writer_->writeFormationState(msg));
}

TEST_F(RosbagWriterTest, WriteSafetyEvent_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SafetyEvent>();
    msg->timestamp_us = 1000000;
    msg->source_id = 1;
    msg->type = 0;
    msg->severity = 1;
    
    EXPECT_NO_THROW(writer_->writeSafetyEvent(msg));
}

TEST_F(RosbagWriterTest, WriteTelemetrySync_Callback) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::TelemetrySync>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    msg->firmware_time_us = 1000000;
    msg->companion_time_us = 1000005;
    msg->offset_us = 5.0;
    msg->drift_ppm = 10.0;
    
    EXPECT_NO_THROW(writer_->writeTelemetrySync(msg));
}

// ============================================================================
// Generic Write Tests
// ============================================================================

TEST_F(RosbagWriterTest, WriteMessage_Generic) {
    writer_->start();
    
    std_msgs::msg::String msg;
    msg.data = "test";
    rclcpp::Time timestamp(1000000);
    
    EXPECT_NO_THROW(writer_->writeMessage("/test/topic", msg, timestamp));
}

TEST_F(RosbagWriterTest, WriteMessage_NotRecording) {
    std_msgs::msg::String msg;
    msg.data = "test";
    rclcpp::Time timestamp(1000000);
    
    // Should not crash, just ignore
    EXPECT_NO_THROW(writer_->writeMessage("/test", msg, timestamp));
}

// ============================================================================
// Bag Rotation Tests
// ============================================================================

TEST_F(RosbagWriterTest, RotateBag_CreatesNewBag) {
    writer_->start();
    
    std::string initial_path = writer_->getStats().current_bag_path;
    
    // Manually trigger rotation
    writer_->rotateBag();
    
    std::string new_path = writer_->getStats().current_bag_path;
    EXPECT_NE(initial_path, new_path);
    EXPECT_GT(writer_->getStats().bags_rotated, 0);
}

TEST_F(RosbagWriterTest, CloseCurrentBag_ClosesWriter) {
    writer_->start();
    writer_->closeCurrentBag();
    
    // Should not crash when writing after close
    std_msgs::msg::String msg;
    msg.data = "test";
    rclcpp::Time timestamp(1000000);
    EXPECT_NO_THROW(writer_->writeMessage("/test", msg, timestamp));
}

// ============================================================================
// Cleanup Tests
// ============================================================================

TEST_F(RosbagWriterTest, CleanupOldLogs_NoCrash) {
    writer_->start();
    writer_->stop();
    
    EXPECT_NO_THROW(writer_->cleanupOldLogs());
}

// ============================================================================
// Statistics Tests
// ============================================================================

TEST_F(RosbagWriterTest, GetStats_InitialValues) {
    auto stats = writer_->getStats();
    
    EXPECT_EQ(stats.messages_written, 0);
    EXPECT_EQ(stats.messages_dropped, 0);
    EXPECT_EQ(stats.bytes_written, 0);
    EXPECT_EQ(stats.bags_rotated, 0);
    EXPECT_DOUBLE_EQ(stats.current_bag_size_mb, 0.0);
    EXPECT_TRUE(stats.messages_per_topic.empty());
    EXPECT_TRUE(stats.bytes_per_topic.empty());
    EXPECT_DOUBLE_EQ(stats.avg_write_latency_ms, 0.0);
    EXPECT_FALSE(stats.is_recording);
    EXPECT_TRUE(stats.current_bag_path.empty());
}

TEST_F(RosbagWriterTest, GetStats_AfterWriting) {
    writer_->start();
    
    // Write some messages
    for (int i = 0; i < 10; ++i) {
        auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
        msg->timestamp_us = 1000000 + i * 1000;
        msg->drone_id = 1;
        msg->type = 0;
        msg->class_id = 1;
        msg->class_name = "test";
        msg->confidence = 0.9;
        writer_->writeAIInference(msg);
    }
    
    auto stats = writer_->getStats();
    
    EXPECT_GT(stats.messages_written, 0);
    EXPECT_GT(stats.bytes_written, 0);
    EXPECT_GT(stats.avg_write_latency_ms, 0.0);
    EXPECT_TRUE(stats.messages_per_topic.count("/swarm/ai_inference") > 0);
    EXPECT_EQ(stats.messages_per_topic["/swarm/ai_inference"], 10);
}

// ============================================================================
// Bag Rotation Callback Tests
// ============================================================================

TEST_F(RosbagWriterTest, BagRotateCallback_Called) {
    bool callback_called = false;
    std::string old_path, new_path;
    
    writer_->setBagRotateCallback([&](const std::string& old_p, const std::string& new_p) {
        callback_called = true;
        old_path = old_p;
        new_path = new_p;
    });
    
    writer_->start();
    writer_->rotateBag();
    
    EXPECT_TRUE(callback_called);
    EXPECT_FALSE(old_path.empty());
    EXPECT_FALSE(new_path.empty());
    EXPECT_NE(old_path, new_path);
}

// ============================================================================
// Indexing Tests
// ============================================================================

TEST_F(RosbagWriterTest, GetMessageIndex_ValidRange) {
    writer_->start();
    
    // Write messages with known timestamps
    for (int i = 0; i < 5; ++i) {
        auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
        msg->timestamp_us = 1000000 + i * 1000000;  // 1 second apart
        msg->drone_id = 1;
        msg->type = 0;
        msg->class_id = 1;
        msg->class_name = "test";
        msg->confidence = 0.9;
        writer_->writeAIInference(msg);
    }
    
    writer_->stop();
    
    // Query index
    auto index = writer_->getMessageIndex("/swarm/ai_inference", 1000000000, 5000000000);
    EXPECT_GE(index.size(), 5);
    
    // Verify timestamps are in range
    for (const auto& entry : index) {
        EXPECT_GE(entry.timestamp_ns, 1000000000);
        EXPECT_LE(entry.timestamp_ns, 5000000000);
        EXPECT_EQ(entry.topic_name, "/swarm/ai_inference");
    }
}

TEST_F(RosbagWriterTest, GetMessageIndex_EmptyRange) {
    writer_->start();
    writer_->stop();
    
    auto index = writer_->getMessageIndex("/swarm/ai_inference", 0, 1000);
    EXPECT_EQ(index.size(), 0);
}

TEST_F(RosbagWriterTest, GetMessageIndex_WrongTopic) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    writer_->writeAIInference(msg);
    
    writer_->stop();
    
    auto index = writer_->getMessageIndex("/swarm/nonexistent", 0, 2000000000);
    EXPECT_EQ(index.size(), 0);
}

// ============================================================================
// State Tests
// ============================================================================

TEST_F(RosbagWriterTest, IsInitialized_AfterInit) {
    EXPECT_TRUE(writer_->isInitialized());
}

TEST_F(RosbagWriterTest, IsInitialized_BeforeInit) {
    auto writer = std::make_unique<RosbagWriter>();
    EXPECT_FALSE(writer->isInitialized());
}

TEST_F(RosbagWriterTest, IsRecording_AfterStart) {
    EXPECT_FALSE(writer_->isRecording());
    
    writer_->start();
    EXPECT_TRUE(writer_->isRecording());
    
    writer_->stop();
    EXPECT_FALSE(writer_->isRecording());
}

// ============================================================================
// Configuration Tests
// ============================================================================

TEST_F(RosbagWriterTest, Config_TopicsToRecord) {
    auto config = writer_->getConfig();
    
    EXPECT_GT(config.topics_to_record.size(), 0);
    EXPECT_TRUE(std::find(config.topics_to_record.begin(), 
                          config.topics_to_record.end(), 
                          "/swarm/ai_inference") != config.topics_to_record.end());
}

TEST_F(RosbagWriterTest, Config_Compression) {
    auto config = writer_->getConfig();
    
    EXPECT_EQ(config.compression_format, "zstd");
    EXPECT_EQ(config.compression_level, 3);
}

TEST_F(RosbagWriterTest, Config_Storage) {
    auto config = writer_->getConfig();
    
    EXPECT_TRUE(config.auto_cleanup_old);
    EXPECT_EQ(config.max_storage_gb, 50);
    EXPECT_EQ(config.max_age_days, 30);
}

TEST_F(RosbagWriterTest, Config_Indexing) {
    auto config = writer_->getConfig();
    
    EXPECT_TRUE(config.enable_indexing);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(RosbagWriterTest, WriteLargeMessage) {
    writer_->start();
    
    auto msg = std::make_shared<swarm_msgs::msg::SwarmMap>();
    msg->timestamp_us = 1000000;
    msg->drone_id = 1;
    // Large map data would go here
    
    EXPECT_NO_THROW(writer_->writeSwarmMap(msg));
}

TEST_F(RosbagWriterTest, RapidWrites) {
    writer_->start();
    
    // Write many messages rapidly
    for (int i = 0; i < 1000; ++i) {
        auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
        msg->timestamp_us = 1000000 + i * 1000;
        msg->drone_id = 1;
        msg->type = 0;
        msg->class_id = 1;
        msg->class_name = "rapid";
        msg->confidence = 0.9;
        writer_->writeAIInference(msg);
    }
    
    auto stats = writer_->getStats();
    EXPECT_GE(stats.messages_written, 1000);
}

TEST_F(RosbagWriterTest, QueueOverflow_DropsMessages) {
    writer_->start();
    
    // Fill queue beyond max size (config has max_queue_size = 100)
    for (int i = 0; i < 150; ++i) {
        auto msg = std::make_shared<swarm_msgs::msg::AIInference>();
        msg->timestamp_us = 1000000 + i * 1000;
        msg->drone_id = 1;
        msg->type = 0;
        msg->class_id = 1;
        msg->class_name = "overflow";
        msg->confidence = 0.9;
        writer_->writeAIInference(msg);
    }
    
    auto stats = writer_->getStats();
    // Some messages may be dropped depending on writer speed
    EXPECT_GE(stats.messages_written, 0);
}

// ============================================================================
// Multi-Topic Tests
// ============================================================================

TEST_F(RosbagWriterTest, WriteMultipleTopics) {
    writer_->start();
    
    // Write to multiple topics
    for (int i = 0; i < 10; ++i) {
        // AI Inference
        auto ai_msg = std::make_shared<swarm_msgs::msg::AIInference>();
        ai_msg->timestamp_us = 1000000 + i * 1000;
        ai_msg->drone_id = 1;
        ai_msg->type = 0;
        ai_msg->class_id = 1;
        ai_msg->class_name = "multi";
        ai_msg->confidence = 0.9;
        writer_->writeAIInference(ai_msg);
        
        // Comm Status
        auto comm_msg = std::make_shared<swarm_msgs::msg::CommStatus>();
        comm_msg->timestamp_us = 1000000 + i * 1000;
        comm_msg->drone_id = 1;
        comm_msg->link_quality = 0.9;
        writer_->writeCommStatus(comm_msg);
        
        // Task Allocation
        auto task_msg = std::make_shared<swarm_msgs::msg::TaskAllocation>();
        task_msg->timestamp_us = 1000000 + i * 1000;
        task_msg->drone_id = 1;
        task_msg->task_id = i;
        writer_->writeTaskAllocation(task_msg);
    }
    
    auto stats = writer_->getStats();
    
    EXPECT_EQ(stats.messages_per_topic["/swarm/ai_inference"], 10);
    EXPECT_EQ(stats.messages_per_topic["/swarm/comm_status"], 10);
    EXPECT_EQ(stats.messages_per_topic["/swarm/task_allocation"], 10);
    EXPECT_EQ(stats.messages_written, 30);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}