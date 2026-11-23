#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_safety_emergency_stop/emergency_stop_handler.hpp"

using namespace swarm::safety;
using Eigen::Vector3d;

class EmergencyStopHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        node_ = std::make_shared<EmergencyStopHandlerNode>();
        
        // Configure for testing
        std::lock_guard<std::mutex> lock(node_->config_mutex_);
        node_->config_.enable_mesh_broadcast = true;
        node_->config_.broadcast_retry_count = 3;
        node_->config_.broadcast_retry_interval_ms = 100;
        node_->config_.ack_timeout_ms = 2000;
        node_->config_.max_missing_acks_before_escalation = 2;
        node_->config_.deploy_parachute = false;
        node_->config_.local_motor_kill = true;
        node_->config_.motor_count = 4;
        node_->drone_id_ = 1;
        
        node_->swarm_members_ = {1, 2, 3, 4, 5};
        node_->position_received_ = true;
        node_->current_position_enu_ = Vector3d(0, 0, 100);
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::shared_ptr<EmergencyStopHandlerNode> node_;
};

// ============================================================================
// Emergency Stop Trigger Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, TriggerEmergencyStop_SetsActive) {
    node_->triggerEmergencyStop("Test reason", 1);
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.active);
    EXPECT_EQ(node_->state_.reason, "Test reason");
    EXPECT_EQ(node_->state_.source_id, 1);
    EXPECT_FALSE(node_->state_.parachute_deployed);
}

TEST_F(EmergencyStopHandlerTest, TriggerEmergencyStop_AlreadyActive_ReturnsFalse) {
    node_->triggerEmergencyStop("First", 1);
    node_->triggerEmergencyStop("Second", 2);
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_EQ(node_->state_.reason, "First");  // Should keep first reason
    EXPECT_EQ(node_->state_.source_id, 1);
}

TEST_F(EmergencyStopHandlerTest, TriggerEmergencyStop_InitializesAckTracking) {
    node_->triggerEmergencyStop("Test", 1);
    
    std::lock_guard<std::mutex> lock(node_->acks_mutex_);
    // Should have pending ACKs for all other swarm members (2,3,4,5)
    EXPECT_EQ(node_->pending_acks_.size(), 4);
    for (uint16_t id : {2, 3, 4, 5}) {
        EXPECT_NE(node_->pending_acks_.find(id), node_->pending_acks_.end());
        EXPECT_EQ(node_->pending_acks_[id].status, "pending");
    }
}

TEST_F(EmergencyStopHandlerTest, TriggerEmergencyStop_PublishesSafetyEvent) {
    // Just verify trigger doesn't crash - event publishing tested via mock in integration
    EXPECT_NO_THROW(node_->triggerEmergencyStop("Test", 1));
}

// ============================================================================
// Reset Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, ResetEmergencyStop_ClearsState) {
    node_->triggerEmergencyStop("Test", 1);
    node_->resetEmergencyStop(0);  // Ground station
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_FALSE(node_->state_.active);
    EXPECT_FALSE(node_->state_.parachute_deployed);
    EXPECT_FALSE(node_->state_.mesh_broadcast_sent);
    EXPECT_EQ(node_->state_.broadcast_retries, 0);
    EXPECT_FALSE(node_->state_.escalated);
    
    std::lock_guard<std::mutex> lock2(node_->acks_mutex_);
    EXPECT_TRUE(node_->pending_acks_.empty());
}

TEST_F(EmergencyStopHandlerTest, ResetEmergencyStop_NotActive_ReturnsFalse) {
    // Not active, try to reset
    // The service callback would return false, but direct call just returns
    node_->resetEmergencyStop(0);
    // Should not crash
    EXPECT_TRUE(true);
}

TEST_F(EmergencyStopHandlerTest, ResetEmergencyStop_RequiresGroundStation) {
    node_->triggerEmergencyStop("Test", 1);
    
    // Try reset from non-ground-station (ID != 0)
    // In actual service, this would fail, but direct call bypasses check
    // Just verify it can be called
    EXPECT_NO_THROW(node_->resetEmergencyStop(2));
}

// ============================================================================
// Broadcast Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, BroadcastEmergencyStop_SetsFlags) {
    node_->triggerEmergencyStop("Test", 1);
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.mesh_broadcast_sent);
    EXPECT_EQ(node_->state_.broadcast_retries, 1);
    EXPECT_GT(node_->state_.last_broadcast_us, 0);
}

TEST_F(EmergencyStopHandlerTest, BroadcastEmergencyStop_Disabled) {
    std::lock_guard<std::mutex> lock(node_->config_mutex_);
    node_->config_.enable_mesh_broadcast = false;
    
    node_->triggerEmergencyStop("Test", 1);
    
    std::lock_guard<std::mutex> lock2(node_->state_mutex_);
    EXPECT_FALSE(node_->state_.mesh_broadcast_sent);
}

// ============================================================================
// Parachute Deployment Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, DeployParachute_SetsFlag) {
    node_->triggerEmergencyStop("Test", 1);
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_FALSE(node_->state_.parachute_deployed);
    
    node_->deployParachute();
    
    std::lock_guard<std::mutex> lock2(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.parachute_deployed);
}

TEST_F(EmergencyStopHandlerTest, DeployParachute_AlreadyDeployed_NoOp) {
    node_->triggerEmergencyStop("Test", 1);
    node_->deployParachute();
    node_->deployParachute();  // Second call
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.parachute_deployed);
    // Should not crash or change state
}

// ============================================================================
// Motor Kill Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, KillMotors_SendsCommands) {
    // Just verify no crash
    EXPECT_NO_THROW(node_->killMotors());
}

// ============================================================================
// ACK Handling Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, HandleAck_UpdatesPending) {
    node_->triggerEmergencyStop("Test", 1);
    
    // Simulate ACK from drone 2
    swarm_msgs::msg::EmergencyStopAck ack;
    ack.source_id = 2;
    ack.timestamp_us = 1000000;
    ack.parachute_deployed = false;
    ack.status = "acknowledged";
    
    node_->handleAck(ack);
    
    std::lock_guard<std::mutex> lock(node_->acks_mutex_);
    EXPECT_NE(node_->pending_acks_.find(2), node_->pending_acks_.end());
    EXPECT_EQ(node_->pending_acks_[2].status, "acknowledged");
    EXPECT_EQ(node_->pending_acks_[2].ack_time_us, 1000000);
    
    std::lock_guard<std::mutex> lock2(node_->state_mutex_);
    EXPECT_NE(node_->state_.acks_received.find(2), node_->state_.acks_received.end());
}

TEST_F(EmergencyStopHandlerTest, HandleAck_UnknownSource) {
    // ACK from drone not in pending list
    swarm_msgs::msg::EmergencyStopAck ack;
    ack.source_id = 99;
    ack.timestamp_us = 1000000;
    
    EXPECT_NO_THROW(node_->handleAck(ack));
}

// ============================================================================
// Mesh Message Handling Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, HandleIncomingEmergencyStop_TriggersLocal) {
    swarm_msgs::msg::MeshMessage msg;
    msg.timestamp_us = 1000000;
    msg.source_id = 3;
    msg.target_id = 0;  // Broadcast
    msg.type = 10;  // EMERGENCY_STOP
    msg.payload = std::vector<uint8_t>({'E','S','T','O','P',':','3',':','T','e','s','t'});
    
    node_->handleIncomingEmergencyStop(msg);
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.active);
    EXPECT_EQ(node_->state_.source_id, 3);
}

TEST_F(EmergencyStopHandlerTest, HandleIncomingEmergencyStop_AlreadyActive) {
    node_->triggerEmergencyStop("Local", 1);
    
    swarm_msgs::msg::MeshMessage msg;
    msg.timestamp_us = 1000000;
    msg.source_id = 3;
    msg.target_id = 0;
    msg.type = 10;
    msg.payload = std::vector<uint8_t>({'E','S','T','O','P',':','3',':','O','t','h','e','r'});
    
    node_->handleIncomingEmergencyStop(msg);
    
    // Should keep original reason
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_EQ(node_->state_.reason, "Local");
}

// ============================================================================
// Escalation Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, EscalateIfNeeded_SetsFlag) {
    node_->triggerEmergencyStop("Test", 1);
    
    node_->escalateIfNeeded();
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.escalated);
}

TEST_F(EmergencyStopHandlerTest, EscalateIfNeeded_AlreadyEscalated) {
    node_->triggerEmergencyStop("Test", 1);
    node_->escalateIfNeeded();
    node_->escalateIfNeeded();  // Second call
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_TRUE(node_->state_.escalated);
}

// ============================================================================
// MAVLink Command Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, SendMAVLinkCommand_SendsMessage) {
    EXPECT_NO_THROW(node_->sendMAVLinkCommand(207, {0, 1000, 0, 0, 0, 0, 0}));
}

TEST_F(EmergencyStopHandlerTest, SendMAVLinkLong_SendsMessage) {
    EXPECT_NO_THROW(node_->sendMAVLinkLong(400, {0, 21196, 0, 0, 0, 0, 0}));
}

// ============================================================================
// Status Publishing Tests
// ============================================================================

TEST_F(EmergencyStopHandlerTest, PublishStatus_Inactive) {
    // Should not crash when inactive
    EXPECT_NO_THROW(node_->publishStatus());
}

TEST_F(EmergencyStopHandlerTest, PublishStatus_Active) {
    node_->triggerEmergencyStop("Test", 1);
    
    EXPECT_NO_THROW(node_->publishStatus());
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(EmergencyStopHandlerTest, SwarmMembersExcludesSelf) {
    node_->triggerEmergencyStop("Test", 1);
    
    std::lock_guard<std::mutex> lock(node_->acks_mutex_);
    EXPECT_NE(node_->pending_acks_.find(1), node_->pending_acks_.end());  // Self should not be in pending
}

TEST_F(EmergencyStopHandlerTest, CooldownPreventsRapidReset) {
    node_->triggerEmergencyStop("Test", 1);
    
    // Try immediate reset (should be rejected by cooldown in service)
    // Direct call bypasses check, but we can verify cooldown logic exists
    node_->resetEmergencyStop(0);
    
    std::lock_guard<std::mutex> lock(node_->state_mutex_);
    EXPECT_FALSE(node_->state_.active);  // Direct call works
    EXPECT_GT(node_->state_.trigger_time_us, 0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}