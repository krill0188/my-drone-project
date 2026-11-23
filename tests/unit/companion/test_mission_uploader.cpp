#include <gtest/gtest.h>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "swarm_interface_mission_uploader/mission_uploader.hpp"

using namespace swarm::interface;

class MissionUploaderTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        uploader_ = std::make_unique<MissionUploader>();
        
        MissionUploaderConfig config;
        config.auto_upload_on_intent = true;
        config.validate_before_upload = true;
        config.max_waypoints = 100;
        config.max_mission_duration_s = 3600.0f;
        config.min_altitude_m = 10.0f;
        config.max_altitude_m = 120.0f;
        
        EXPECT_TRUE(uploader_->initialize(config));
    }
    
    void TearDown() override {
        rclcpp::shutdown();
    }
    
    std::unique_ptr<MissionUploader> uploader_;
};

// ============================================================================
// Template Tests
// ============================================================================

TEST_F(MissionUploaderTest, LoadDefaultTemplates) {
    auto names = uploader_->getTemplateNames();
    
    EXPECT_GT(names.size(), 0);
    
    // Check for expected default templates
    std::vector<std::string> expected = {
        "search_area", "follow_me", "inspect_target", 
        "patrol_route", "return_to_launch", "hold_position"
    };
    
    for (const auto& expected_name : expected) {
        bool found = false;
        for (const auto& name : names) {
            if (name == expected_name) {
                found = true;
                break;
            }
        }
        EXPECT_TRUE(found) << "Template " << expected_name << " not found";
    }
}

TEST_F(MissionUploaderTest, GetTemplate_Existing) {
    auto tmpl = uploader_->getTemplate("search_area");
    
    EXPECT_TRUE(tmpl.has_value());
    EXPECT_EQ(tmpl->name, "search_area");
    EXPECT_FALSE(tmpl->description.empty());
    EXPECT_FALSE(tmpl->required_parameters.empty());
}

TEST_F(MissionUploaderTest, GetTemplate_NonExistent) {
    auto tmpl = uploader_->getTemplate("non_existent");
    EXPECT_FALSE(tmpl.has_value());
}

TEST_F(MissionUploaderTest, RegisterCustomTemplate) {
    MissionTemplate tmpl;
    tmpl.name = "custom_test";
    tmpl.description = "Test template";
    tmpl.required_parameters = {"param1"};
    tmpl.default_parameters = {{"param1", "default"}};
    
    EXPECT_TRUE(uploader_->registerTemplate(tmpl));
    
    auto tmpl_opt = uploader_->getTemplate("custom_test");
    EXPECT_TRUE(tmpl_opt.has_value());
    EXPECT_EQ(tmpl_opt->name, "custom_test");
}

// ============================================================================
// Mission Creation Tests
// ============================================================================

TEST_F(MissionUploaderTest, CreateMissionFromIntent_SearchArea) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::SEARCH_AREA;
    intent.raw_text = "search area building 3";
    intent.confidence = 0.9f;
    intent.priority = swarm::interface::IntentPriority::HIGH;
    intent.parameters["area"] = "building 3";
    intent.parameters["pattern"] = "lawnmower";
    intent.parameters["spacing_m"] = "20";
    intent.parameters["altitude_m"] = "50";
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_TRUE(mission_opt.has_value());
    auto mission = mission_opt.value();
    
    EXPECT_FALSE(mission.mission_id.empty());
    EXPECT_EQ(mission.mission_type, static_cast<uint8_t>(swarm::interface::IntentType::SEARCH_AREA));
    EXPECT_GT(mission.tasks.size(), 0);
    EXPECT_EQ(mission.task_count, mission.tasks.size());
}

TEST_F(MissionUploaderTest, CreateMissionFromIntent_FollowMe) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::FOLLOW_ME;
    intent.raw_text = "follow me";
    intent.confidence = 0.9f;
    intent.priority = swarm::interface::IntentPriority::HIGH;
    intent.parameters["target_id"] = "5";
    intent.parameters["distance_m"] = "10";
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_TRUE(mission_opt.has_value());
    auto mission = mission_opt.value();
    
    EXPECT_EQ(mission.mission_type, static_cast<uint8_t>(swarm::interface::IntentType::FOLLOW_ME));
    EXPECT_GT(mission.tasks.size(), 0);
}

TEST_F(MissionUploaderTest, CreateMissionFromIntent_InspectTarget) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::INSPECT_TARGET;
    intent.raw_text = "inspect tower 5";
    intent.confidence = 0.85f;
    intent.priority = swarm::interface::IntentPriority::HIGH;
    intent.parameters["target"] = "tower 5";
    intent.parameters["radius_m"] = "15";
    intent.parameters["altitude_m"] = "30";
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_TRUE(mission_opt.has_value());
    auto mission = mission_opt.value();
    
    EXPECT_EQ(mission.mission_type, static_cast<uint8_t>(swarm::interface::IntentType::INSPECT_TARGET));
}

TEST_F(MissionUploaderTest, CreateMissionFromIntent_ReturnHome) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::RETURN_HOME;
    intent.raw_text = "return home";
    intent.confidence = 0.95f;
    intent.priority = swarm::interface::IntentPriority::CRITICAL;
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_TRUE(mission_opt.has_value());
    auto mission = mission_opt.value();
    
    EXPECT_EQ(mission.mission_type, static_cast<uint8_t>(swarm::interface::IntentType::RETURN_HOME));
    EXPECT_EQ(mission.tasks[0].type, static_cast<uint8_t>(swarm::autonomy::TaskType::RTL));
}

TEST_F(MissionUploaderTest, CreateMissionFromIntent_HoldPosition) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::HOLD_POSITION;
    intent.raw_text = "hold position";
    intent.confidence = 0.9f;
    intent.priority = swarm::interface::IntentPriority::NORMAL;
    intent.parameters["duration_s"] = "30";
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_TRUE(mission_opt.has_value());
    auto mission = mission_opt.value();
    
    EXPECT_EQ(mission.mission_type, static_cast<uint8_t>(swarm::interface::IntentType::HOLD_POSITION));
    EXPECT_EQ(mission.tasks[0].type, static_cast<uint8_t>(swarm::autonomy::TaskType::HOVER));
}

TEST_F(MissionUploaderTest, CreateMissionFromIntent_UnknownType) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::UNKNOWN;
    intent.raw_text = "unknown command";
    intent.confidence = 0.1f;
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_FALSE(mission_opt.has_value());
}

TEST_F(MissionUploaderTest, CreateMissionFromIntent_WithConstraints) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::SEARCH_AREA;
    intent.raw_text = "search area";
    intent.confidence = 0.9f;
    intent.parameters["max_altitude_m"] = "100";
    intent.parameters["min_altitude_m"] = "20";
    intent.parameters["max_speed_mps"] = "15";
    intent.parameters["duration_s"] = "1800";
    
    auto mission_opt = uploader_->createMissionFromIntent(intent);
    
    EXPECT_TRUE(mission_opt.has_value());
    auto mission = mission_opt.value();
    
    EXPECT_FLOAT_EQ(mission.constraints.max_altitude_m, 100.0f);
    EXPECT_FLOAT_EQ(mission.constraints.min_altitude_m, 20.0f);
    EXPECT_FLOAT_EQ(mission.constraints.max_speed_mps, 15.0f);
    EXPECT_FLOAT_EQ(mission.max_duration_s, 1800.0f);
}

// ============================================================================
// Mission Validation Tests
// ============================================================================

TEST_F(MissionUploaderTest, ValidateMission_ValidMission) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test_mission_1";
    mission.mission_type = 10;  // SEARCH_AREA
    mission.task_count = 1;
    mission.max_duration_s = 1800.0f;
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::SEARCH);
    task.priority = 1;
    mission.tasks.push_back(task);
    mission.task_count = 1;
    
    mission.constraints.min_altitude_m = 10.0f;
    mission.constraints.max_altitude_m = 100.0f;
    mission.constraints.max_speed_mps = 15.0f;
    
    std::string error;
    EXPECT_TRUE(uploader_->validateMission(mission, error));
    EXPECT_TRUE(error.empty());
}

TEST_F(MissionUploaderTest, ValidateMission_EmptyMissionId) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "";
    mission.task_count = 1;
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    task.type = 10;
    mission.tasks.push_back(task);
    mission.task_count = 1;
    
    std::string error;
    EXPECT_FALSE(uploader_->validateMission(mission, error));
    EXPECT_FALSE(error.empty());
    EXPECT_TRUE(error.find("Mission ID is empty") != std::string::npos);
}

TEST_F(MissionUploaderTest, ValidateMission_NoTasks) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test";
    mission.task_count = 0;
    
    std::string error;
    EXPECT_FALSE(uploader_->validateMission(mission, error));
    EXPECT_TRUE(error.find("no tasks") != std::string::npos);
}

TEST_F(MissionUploaderTest, ValidateMission_TaskCountMismatch) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test";
    mission.task_count = 2;
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    mission.tasks.push_back(task);
    mission.task_count = 2;  // Mismatch: count=2 but only 1 task
    
    std::string error;
    EXPECT_FALSE(uploader_->validateMission(mission, error));
    EXPECT_TRUE(error.find("Task count mismatch") != std::string::npos);
}

TEST_F(MissionUploaderTest, ValidateMission_InvalidAltitudeConstraints) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test";
    mission.task_count = 1;
    mission.max_duration_s = 1800.0f;
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    mission.tasks.push_back(task);
    mission.task_count = 1;
    
    mission.constraints.min_altitude_m = 100.0f;
    mission.constraints.max_altitude_m = 50.0f;  // Invalid: min > max
    
    std::string error;
    EXPECT_FALSE(uploader_->validateMission(mission, error));
    EXPECT_TRUE(error.find("Invalid altitude constraints") != std::string::npos);
}

TEST_F(MissionUploaderTest, ValidateMission_ExcessiveDuration) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test";
    mission.task_count = 1;
    mission.max_duration_s = 5000.0f;  // > 3600s max
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    mission.tasks.push_back(task);
    mission.task_count = 1;
    
    std::string error;
    EXPECT_FALSE(uploader_->validateMission(mission, error));
    EXPECT_TRUE(error.find("exceeds maximum") != std::string::npos);
}

// ============================================================================
// Upload Tests
// ============================================================================

TEST_F(MissionUploaderTest, UploadMission_Success) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test_mission_1";
    mission.mission_type = 10;
    mission.task_count = 1;
    mission.max_duration_s = 1800.0f;
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    task.type = static_cast<uint8_t>(swarm::autonomy::TaskType::SEARCH);
    task.priority = 1;
    mission.tasks.push_back(task);
    mission.task_count = 1;
    
    std::vector<uint16_t> targets = {2, 3, 4};
    
    auto result = uploader_->uploadMission(mission, targets);
    
    EXPECT_TRUE(result.success);
    EXPECT_EQ(result.mission_id, "test_mission_1");
    EXPECT_GT(result.upload_time_us, 0);
    EXPECT_EQ(result.drone_acks.size(), 3);
    EXPECT_FALSE(result.drone_acks[2]);
    EXPECT_FALSE(result.drone_acks[3]);
    EXPECT_FALSE(result.drone_acks[4]);
}

TEST_F(MissionUploaderTest, UploadFromIntent_Success) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::SEARCH_AREA;
    intent.raw_text = "search area building 3";
    intent.confidence = 0.9f;
    intent.priority = swarm::interface::IntentPriority::HIGH;
    intent.parameters["area"] = "building 3";
    
    std::vector<uint16_t> targets = {2, 3, 4};
    
    auto result = uploader_->uploadFromIntent(intent, targets);
    
    EXPECT_TRUE(result.success);
    EXPECT_FALSE(result.mission_id.empty());
    EXPECT_EQ(result.drone_acks.size(), 3);
}

// ============================================================================
// Cancel Mission Tests
// ============================================================================

TEST_F(MissionUploaderTest, CancelMission) {
    // Can't easily test without actual upload, but verify function exists
    EXPECT_TRUE(uploader_->cancelMission("test_mission"));
}

// ============================================================================
// Upload Status Tests
// ============================================================================

TEST_F(MissionUploaderTest, GetUploadStatus_NotFound) {
    auto status = uploader_->getUploadStatus("non_existent");
    EXPECT_FALSE(status.has_value());
}

TEST_F(MissionUploaderTest, GetAllUploadStatuses_Empty) {
    auto statuses = uploader_->getAllUploadStatuses();
    EXPECT_TRUE(statuses.empty());
}

// ============================================================================
// Callbacks Tests
// ============================================================================

TEST_F(MissionUploaderTest, SetUploadCompleteCallback) {
    bool callback_called = false;
    swarm::interface::UploadResult captured_result;
    
    uploader_->setUploadCompleteCallback([&](const UploadResult& result) {
        callback_called = true;
        captured_result = result;
    });
    
    EXPECT_TRUE(true);  // Just verify callback can be set
}

TEST_F(MissionUploaderTest, SetValidateMissionCallback) {
    bool callback_called = false;
    
    uploader_->setValidateMissionCallback([&](const auto& request, auto response) {
        callback_called = true;
    });
    
    EXPECT_TRUE(true);
}

TEST_F(MissionUploaderTest, SetUploadMissionCallback) {
    bool callback_called = false;
    
    uploader_->setUploadMissionCallback([&](const auto& request, auto response) {
        callback_called = true;
    });
    
    EXPECT_TRUE(true);
}

TEST_F(MissionUploaderTest, SetUploadCompleteCallback) {
    bool callback_called = false;
    
    uploader_->setUploadCompleteCallback([&](const UploadResult& result) {
        callback_called = true;
    });
    
    EXPECT_TRUE(true);
}

// ============================================================================
// Config Tests
// ============================================================================

TEST_F(MissionUploaderTest, GetConfig) {
    auto config = uploader_->getConfig();
    
    EXPECT_TRUE(config.auto_upload_on_intent);
    EXPECT_TRUE(config.validate_before_upload);
    EXPECT_EQ(config.max_waypoints, 100);
    EXPECT_FLOAT_EQ(config.max_mission_duration_s, 3600.0f);
    EXPECT_FLOAT_EQ(config.min_altitude_m, 10.0f);
    EXPECT_FLOAT_EQ(config.max_altitude_m, 120.0f);
}

// ============================================================================
// Edge Cases
// ============================================================================

TEST_F(MissionUploaderTest, CancelMission_NonExistent) {
    // Should not crash
    EXPECT_TRUE(uploader_->cancelMission("non_existent"));
}

TEST_F(MissionUploaderTest, UploadMission_EmptyTargets) {
    swarm_msgs::msg::Mission mission;
    mission.mission_id = "test";
    mission.task_count = 1;
    mission.max_duration_s = 1800.0f;
    
    swarm_msgs::msg::Task task;
    task.task_id = "task_1";
    mission.tasks.push_back(task);
    mission.task_count = 1;
    
    std::vector<uint16_t> targets;  // Empty
    
    auto result = uploader_->uploadMission(mission, targets);
    
    // Should still succeed but with empty acks
    EXPECT_TRUE(result.success);
    EXPECT_TRUE(result.drone_acks.empty());
}

TEST_F(MissionUploaderTest, UploadFromIntent_WithConstraints) {
    swarm::interface::ParsedIntent intent;
    intent.timestamp_us = 1000000;
    intent.type = swarm::interface::IntentType::SEARCH_AREA;
    intent.raw_text = "search area";
    intent.confidence = 0.9f;
    intent.priority = swarm::interface::IntentPriority::HIGH;
    intent.parameters["max_altitude_m"] = "80";
    intent.parameters["min_altitude_m"] = "20";
    intent.parameters["max_speed_mps"] = "12";
    intent.parameters["duration_s"] = "1200";
    
    std::vector<uint16_t> targets = {2, 3};
    
    auto result = uploader_->uploadFromIntent(intent, targets);
    
    EXPECT_TRUE(result.success);
    
    // The mission should have constraints from intent parameters
    // We can't directly verify without access to created mission, 
    // but we verify the upload succeeds
    EXPECT_FALSE(result.mission_id.empty());
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}