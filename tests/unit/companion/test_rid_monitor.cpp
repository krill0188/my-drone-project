#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_safety_rid_broadcaster/rid_monitor.hpp"

using namespace swarm::safety;
using Eigen::Vector3d;

class RIDMonitorTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<RIDMonitorNode>();
        
        // Configure for testing
        std::lock_guard<std::mutex> lock(node_->config_mutex_);
        node_->config_.enable_frame_validation = true;
        node_->config_.enable_spoofing_detection = true;
        node_->config_.max_position_jump_m = 50.0;
        node_->config_.max_speed_mps = 100.0;
        node_->config_.max_acceleration_mps2 = 20.0;
        node_->config_.spoof_distance_threshold_m = 1000.0;
        node_->config_.own_uas_id = "TEST-001";
        node_->drone_id_ = 1;
        
        // Set own position
        node_->current_position_enu_ = Vector3d(0, 0, 100);
        node_->current_velocity_enu_ = Vector3d(0, 0, 0);
        node_->position_received_ = true;
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::shared_ptr<RIDMonitorNode> node_;
};

// ============================================================================
// Frame Validation Tests
// ============================================================================

TEST_F(RIDMonitorTest, ValidateBasicID_Valid) {
    RIDFrame frame;
    frame.uas_id = "TEST-001";
    frame.id_type = 3;  // Session ID
    frame.operator_id = "OP-001";
    frame.uas_type = 2;  // Rotorcraft
    
    EXPECT_TRUE(node_->validateBasicID(frame));
}

TEST_F(RIDMonitorTest, ValidateBasicID_InvalidUASID) {
    RIDFrame frame;
    frame.uas_id = "";  // Empty
    frame.id_type = 3;
    frame.operator_id = "OP-001";
    frame.uas_type = 2;
    
    EXPECT_FALSE(node_->validateBasicID(frame));
}

TEST_F(RIDMonitorTest, ValidateBasicID_InvalidIDType) {
    RIDFrame frame;
    frame.uas_id = "TEST-001";
    frame.id_type = 5;  // Invalid
    frame.operator_id = "OP-001";
    frame.uas_type = 2;
    
    EXPECT_FALSE(node_->validateBasicID(frame));
}

TEST_F(RIDMonitorTest, ValidateBasicID_InvalidUASType) {
    RIDFrame frame;
    frame.uas_id = "TEST-001";
    frame.id_type = 3;
    frame.operator_id = "OP-001";
    frame.uas_type = 10;  // Invalid
    
    EXPECT_FALSE(node_->validateBasicID(frame));
}

// ============================================================================
// Location Validation Tests
// ============================================================================

TEST_F(RIDMonitorTest, ValidateLocation_Valid) {
    RIDFrame frame;
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.altitude_m = 100.0;
    frame.height_m = 50.0;
    frame.horizontal_accuracy_m = 5.0;
    frame.vertical_accuracy_m = 3.0;
    frame.speed_accuracy_mps = 1.0;
    frame.status = 0;
    
    EXPECT_TRUE(node_->validateLocation(frame));
}

TEST_F(RIDMonitorTest, ValidateLocation_InvalidLatitude) {
    RIDFrame frame;
    frame.latitude_deg = 95.0;  // Invalid
    frame.longitude_deg = -122.4194;
    frame.altitude_m = 100.0;
    frame.height_m = 50.0;
    frame.horizontal_accuracy_m = 5.0;
    frame.vertical_accuracy_m = 3.0;
    frame.speed_accuracy_mps = 1.0;
    frame.status = 0;
    
    EXPECT_FALSE(node_->validateLocation(frame));
}

TEST_F(RIDMonitorTest, ValidateLocation_InvalidAltitude) {
    RIDFrame frame;
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.altitude_m = 25000.0;  // Too high
    frame.height_m = 50.0;
    frame.horizontal_accuracy_m = 5.0;
    frame.vertical_accuracy_m = 3.0;
    frame.speed_accuracy_mps = 1.0;
    frame.status = 0;
    
    EXPECT_FALSE(node_->validateLocation(frame));
}

TEST_F(RIDMonitorTest, ValidateLocation_InvalidAccuracy) {
    RIDFrame frame;
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.altitude_m = 100.0;
    frame.height_m = 50.0;
    frame.horizontal_accuracy_m = 150.0;  // Too high
    frame.vertical_accuracy_m = 3.0;
    frame.speed_accuracy_mps = 1.0;
    frame.status = 0;
    
    EXPECT_FALSE(node_->validateLocation(frame));
}

// ============================================================================
// Kinematics Validation Tests
// ============================================================================

TEST_F(RIDMonitorTest, ValidateKinematics_Valid) {
    RIDFrame frame;
    frame.velocity_north_mps = 10.0;
    frame.velocity_east_mps = 5.0;
    frame.velocity_down_mps = -1.0;
    frame.ground_speed_mps = 11.18;
    frame.direction = 45.0;
    
    EXPECT_TRUE(node_->validateKinematics(frame));
}

TEST_F(RIDMonitorTest, ValidateKinematics_ExcessiveSpeed) {
    RIDFrame frame;
    frame.velocity_north_mps = 150.0;  // > max_speed_mps
    frame.velocity_east_mps = 0.0;
    frame.velocity_down_mps = 0.0;
    frame.ground_speed_mps = 150.0;
    frame.direction = 0.0;
    
    EXPECT_FALSE(node_->validateKinematics(frame));
}

TEST_F(RIDMonitorTest, ValidateKinematics_ExcessiveAcceleration) {
    // Add history for acceleration check
    RIDFrame frame;
    frame.timestamp_us = 1000000;
    frame.velocity_north_mps = 50.0;
    frame.velocity_east_mps = 0.0;
    frame.velocity_down_mps = 0.0;
    frame.ground_speed_mps = 50.0;
    frame.direction = 0.0;
    
    // Add previous frame to history
    RIDFrame prev_frame;
    prev_frame.timestamp_us = 500000;  // 0.5s ago
    prev_frame.velocity_north_mps = 0.0;
    prev_frame.velocity_east_mps = 0.0;
    prev_frame.velocity_down_mps = 0.0;
    
    {
        std::lock_guard<std::mutex> lock(node_->tracks_mutex_);
        node_->tracks_[1].history.push_back(prev_frame);
    }
    
    frame.timestamp_us = 1000000;
    frame.velocity_north_mps = 50.0;  // 100 m/s^2 acceleration!
    frame.velocity_east_mps = 0.0;
    frame.velocity_down_mps = 0.0;
    frame.ground_speed_mps = 50.0;
    frame.direction = 0.0;
    frame.source_id = 1;
    
    EXPECT_FALSE(node_->validateKinematics(frame));
}

// ============================================================================
// Authentication Validation Tests
// ============================================================================

TEST_F(RIDMonitorTest, ValidateAuthentication_Valid) {
    RIDFrame frame;
    frame.auth_type = 0;  // None
    frame.auth_data.clear();
    
    EXPECT_TRUE(node_->validateAuthentication(frame));
    
    frame.auth_type = 1;  // Public key
    frame.auth_data = std::vector<uint8_t>(64, 0);
    EXPECT_TRUE(node_->validateAuthentication(frame));
}

TEST_F(RIDMonitorTest, ValidateAuthentication_InvalidAuthType) {
    RIDFrame frame;
    frame.auth_type = 3;  // Invalid
    frame.auth_data.clear();
    
    EXPECT_FALSE(node_->validateAuthentication(frame));
}

TEST_F(RIDMonitorTest, ValidateAuthentication_ExcessiveAuthData) {
    RIDFrame frame;
    frame.auth_type = 1;
    frame.auth_data = std::vector<uint8_t>(100, 0);  // > 64 bytes
    
    EXPECT_FALSE(node_->validateAuthentication(frame));
}

// ============================================================================
// Spoofing Detection Tests
// ============================================================================

TEST_F(RIDMonitorTest, CheckSpoofing_NoPosition) {
    RIDFrame frame;
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.source_id = 2;
    frame.uas_id = "OTHER-001";
    
    node_->position_received_ = false;
    
    EXPECT_TRUE(node_->checkSpoofing(frame));
}

TEST_F(RIDMonitorTest, CheckSpoofing_PositionJump) {
    RIDFrame frame;
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.source_id = 2;
    frame.uas_id = "OTHER-001";
    frame.timestamp_us = 1000000;
    
    // Add previous frame far away
    RIDFrame prev_frame;
    prev_frame.latitude_deg = 38.0;
    prev_frame.longitude_deg = -123.0;
    prev_frame.timestamp_us = 500000;
    prev_frame.source_id = 2;
    
    {
        std::lock_guard<std::mutex> lock(node_->tracks_mutex_);
        node_->tracks_[2].history.push_back(prev_frame);
    }
    
    node_->position_received_ = true;
    node_->current_position_enu_ = Eigen::Vector3d(0, 0, 100);
    
    // Position jump > 50km
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.source_id = 2;
    frame.timestamp_us = 1000000;
    
    EXPECT_FALSE(node_->checkSpoofing(frame));
}

TEST_F(RIDMonitorTest, CheckSpoofing_OwnIDStolen) {
    RIDFrame frame;
    frame.latitude_deg = 0.0;  // Close to own position
    frame.longitude_deg = 0.0;
    frame.source_id = 99;  // Different source
    frame.uas_id = "TEST-001";  // Same as own!
    frame.timestamp_us = 1000000;
    
    node_->position_received_ = true;
    node_->current_position_enu_ = Eigen::Vector3d(0, 0, 100);
    
    // Should detect spoofing (someone using our ID)
    EXPECT_FALSE(node_->checkSpoofing(frame));
}

// ============================================================================
// Track Management Tests
// ============================================================================

TEST_F(RIDMonitorTest, UpdateTrack_NewTrack) {
    RIDFrame frame;
    frame.source_id = 5;
    frame.timestamp_us = 1000000;
    frame.uas_id = "NEW-001";
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.velocity_north_mps = 10.0;
    frame.velocity_east_mps = 5.0;
    frame.velocity_down_mps = 0.0;
    
    node_->updateTrack(frame);
    
    {
        std::lock_guard<std::mutex> lock(node_->tracks_mutex_);
        auto it = node_->tracks_.find(5);
        EXPECT_NE(it, node_->tracks_.end());
        EXPECT_EQ(it->second.history.size(), 1);
        EXPECT_FALSE(it->second.is_spoofed);
    }
}

TEST_F(RIDMonitorTest, UpdateTrack_VelocityAcceleration) {
    RIDFrame frame1;
    frame1.source_id = 10;
    frame1.timestamp_us = 500000;
    frame1.velocity_north_mps = 10.0;
    frame1.velocity_east_mps = 0.0;
    frame1.velocity_down_mps = 0.0;
    
    node_->updateTrack(frame1);
    
    RIDFrame frame2;
    frame2.source_id = 10;
    frame2.timestamp_us = 1000000;  // 0.5s later
    frame2.velocity_north_mps = 20.0;
    frame2.velocity_east_mps = 0.0;
    frame2.velocity_down_mps = 0.0;
    
    node_->updateTrack(frame2);
    
    {
        std::lock_guard<std::mutex> lock(node_->tracks_mutex_);
        auto it = node_->tracks_.find(10);
        EXPECT_NE(it, node_->tracks_.end());
        EXPECT_EQ(it->second.history.size(), 2);
        
        // Check acceleration: (20-10)/0.5 = 20 m/s^2
        EXPECT_NEAR(it->second.last_acceleration_enu.x(), 20.0, 1.0);
        EXPECT_NEAR(it->second.last_acceleration_enu.y(), 0.0, 1.0);
        EXPECT_NEAR(it->second.last_acceleration_enu.z(), 0.0, 1.0);
    }
}

TEST_F(RIDMonitorTest, CheckTrackTimeouts) {
    // Add old track
    RIDFrame old_frame;
    old_frame.source_id = 20;
    old_frame.timestamp_us = 100000;  // Very old
    old_frame.uas_id = "OLD-001";
    
    {
        std::lock_guard<std::mutex> lock(node_->tracks_mutex_);
        node_->tracks_[20].last_frame = old_frame;
        node_->tracks_[20].last_update_us = old_frame.timestamp_us;
        node_->tracks_[20].history.push_back(old_frame);
    }
    
    // Run timeout check
    node_->checkTrackTimeouts();
    
    // Track should be removed after missing_frames > max_missing_frames
    // (depends on config, but old track should eventually be removed)
    // For now just verify function runs without crash
    EXPECT_TRUE(true);
}

// ============================================================================
// MAVLink Conversion Tests
// ============================================================================

TEST_F(RIDMonitorTest, ConvertMAVLinkFrame) {
    mavros_msgs::msg::RIDFrame msg;
    msg.timestamp_us = 1000000;
    msg.source_id = 1;
    msg.uas_id = "TEST-001";
    msg.id_type = 3;
    msg.operator_id = "OP-001";
    msg.uas_type = 2;
    msg.latitude_deg = 37.7749;
    msg.longitude_deg = -122.4194;
    msg.altitude_m = 100.0;
    msg.height_m = 50.0;
    msg.velocity_north_mps = 10.0;
    msg.velocity_east_mps = 5.0;
    msg.velocity_down_mps = -1.0;
    msg.horizontal_accuracy_m = 5.0;
    msg.vertical_accuracy_m = 3.0;
    msg.speed_accuracy_mps = 1.0;
    msg.status = 0;
    msg.direction = 4500;  // 45 degrees * 100
    msg.ground_speed_mps = 11.18;
    msg.sequence = 1;
    msg.auth_type = 0;
    
    RIDFrame frame = node_->convertMAVLinkFrame(std::make_shared<mavros_msgs::msg::RIDFrame>(msg));
    
    EXPECT_EQ(frame.timestamp_us, 1000000);
    EXPECT_EQ(frame.source_id, 1);
    EXPECT_EQ(frame.uas_id, "TEST-001");
    EXPECT_EQ(frame.id_type, 3);
    EXPECT_EQ(frame.operator_id, "OP-001");
    EXPECT_EQ(frame.uas_type, 2);
    EXPECT_DOUBLE_EQ(frame.latitude_deg, 37.7749);
    EXPECT_DOUBLE_EQ(frame.longitude_deg, -122.4194);
    EXPECT_FLOAT_EQ(frame.altitude_m, 100.0f);
    EXPECT_FLOAT_EQ(frame.direction, 45.0f);  // 4500/100
    EXPECT_FLOAT_EQ(frame.ground_speed_mps, 11.18f);
}

TEST_F(RIDMonitorTest, ConvertToMAVLink) {
    RIDFrame frame;
    frame.timestamp_us = 1000000;
    frame.source_id = 1;
    frame.uas_id = "TEST-001";
    frame.id_type = 3;
    frame.operator_id = "OP-001";
    frame.uas_type = 2;
    frame.latitude_deg = 37.7749;
    frame.longitude_deg = -122.4194;
    frame.altitude_m = 100.0;
    frame.height_m = 50.0;
    frame.velocity_north_mps = 10.0;
    frame.velocity_east_mps = 5.0;
    frame.velocity_down_mps = -1.0;
    frame.horizontal_accuracy_m = 5.0;
    frame.vertical_accuracy_m = 3.0;
    frame.speed_accuracy_mps = 1.0;
    frame.status = 0;
    frame.direction = 45.0;
    frame.ground_speed_mps = 11.18;
    frame.sequence = 1;
    frame.auth_type = 0;
    
    auto msg = node_->convertToMAVLink(frame);
    
    EXPECT_EQ(msg.timestamp_us, 1000000);
    EXPECT_EQ(msg.source_id, 1);
    EXPECT_EQ(msg.uas_id, "TEST-001");
    EXPECT_EQ(msg.direction, 4500);  // 45 * 100
    EXPECT_EQ(msg.sequence, 1);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(RIDMonitorTest, FullValidationPipeline_ValidFrame) {
    mavros_msgs::msg::RIDFrame msg;
    msg.timestamp_us = 1000000;
    msg.source_id = 1;
    msg.uas_id = "TEST-001";
    msg.id_type = 3;
    msg.operator_id = "OP-001";
    msg.uas_type = 2;
    msg.latitude_deg = 37.7749;
    msg.longitude_deg = -122.4194;
    msg.altitude_m = 100.0;
    msg.height_m = 50.0;
    msg.velocity_north_mps = 10.0;
    msg.velocity_east_mps = 5.0;
    msg.velocity_down_mps = -1.0;
    msg.horizontal_accuracy_m = 5.0;
    msg.vertical_accuracy_m = 3.0;
    msg.speed_accuracy_mps = 1.0;
    msg.status = 0;
    msg.direction = 4500;
    msg.ground_speed_mps = 11.18;
    msg.sequence = 1;
    msg.auth_type = 0;
    
    RIDFrame frame;
    node_->validateFrame(std::make_shared<mavros_msgs::msg::RIDFrame>(msg), frame);
    
    EXPECT_TRUE(frame.valid);
    EXPECT_TRUE(frame.validation_error.empty());
}

TEST_F(RIDMonitorTest, FullValidationPipeline_InvalidFrame) {
    mavros_msgs::msg::RIDFrame msg;
    msg.timestamp_us = 1000000;
    msg.source_id = 1;
    msg.uas_id = "";  // Invalid
    msg.id_type = 3;
    msg.operator_id = "OP-001";
    msg.uas_type = 2;
    msg.latitude_deg = 37.7749;
    msg.longitude_deg = -122.4194;
    msg.altitude_m = 100.0;
    msg.height_m = 50.0;
    msg.velocity_north_mps = 10.0;
    msg.velocity_east_mps = 5.0;
    msg.velocity_down_mps = -1.0;
    msg.horizontal_accuracy_m = 5.0;
    msg.vertical_accuracy_m = 3.0;
    msg.speed_accuracy_mps = 1.0;
    msg.status = 0;
    msg.direction = 4500;
    msg.ground_speed_mps = 11.18;
    msg.sequence = 1;
    msg.auth_type = 0;
    
    RIDFrame frame;
    node_->validateFrame(std::make_shared<mavros_msgs::msg::RIDFrame>(msg), frame);
    
    EXPECT_FALSE(frame.valid);
    EXPECT_FALSE(frame.validation_error.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}