#ifndef MISSION_UPLOADER_HPP
#define MISSION_UPLOADER_HPP

#include <rclcpp/rclcpp.hpp>
#include <swarm_msgs/msg/human_intent_cmd.hpp>
#include <swarm_msgs/msg/mission.hpp>
#include <swarm_msgs/msg/task.hpp>
#include <swarm_msgs/srv/upload_mission.hpp>
#include <swarm_msgs/srv/validate_mission.hpp>
#include <swarm_interface_intent_parser/intent_parser.hpp>
#include <vector>
#include <string>
#include <map>
#include <memory>
#include <mutex>
#include <chrono>

namespace swarm::interface {

struct MissionUploaderConfig {
    // Upload settings
    bool auto_upload_on_intent = true;
    bool validate_before_upload = true;
    int upload_timeout_s = 10;
    int max_retry_count = 3;
    
    // Mission templates
    std::string mission_templates_path;
    std::map<std::string, std::string> template_files;
    
    // Safety
    bool require_ground_station_approval = false;
    std::vector<uint16_t> approved_ground_station_ids;
    
    // Mission limits
    int max_waypoints = 100;
    float max_mission_duration_s = 3600.0f;  // 1 hour
    float min_altitude_m = 10.0f;
    float max_altitude_m = 120.0f;
    
    // Communication
    int upload_retry_interval_ms = 500;
    int ack_timeout_ms = 5000;
};

struct MissionTemplate {
    std::string name;
    std::string description;
    swarm_msgs::msg::Mission base_mission;
    std::vector<std::string> required_parameters;
    std::map<std::string, std::string> default_parameters;
};

struct UploadResult {
    bool success = false;
    std::string mission_id;
    std::string error_message;
    uint64_t upload_time_us = 0;
    std::map<uint16_t, bool> drone_acks;
};

struct MissionUploadStatus {
    std::string mission_id;
    enum class State {
        IDLE,
        VALIDATING,
        UPLOADING,
        WAITING_ACKS,
        ACTIVE,
        COMPLETED,
        FAILED,
        CANCELLED
    } state = State::IDLE;
    uint64_t start_time_us = 0;
    uint64_t last_update_us = 0;
    std::map<uint16_t, bool> pending_acks;
    int retry_count = 0;
    std::string error_message;
};

class MissionUploader {
public:
    MissionUploader();
    ~MissionUploader();
    
    MissionUploader(const MissionUploader&) = delete;
    MissionUploader& operator=(const MissionUploader&) = delete;
    MissionUploader(MissionUploader&&) noexcept;
    MissionUploader& operator=(MissionUploader&&) noexcept;
    
    // Initialize
    bool initialize(const MissionUploaderConfig& config);
    
    // Load mission templates from file
    bool loadTemplates(const std::string& path);
    
    // Register a new template
    bool registerTemplate(const MissionTemplate& template_);
    
    // Get available templates
    std::vector<std::string> getTemplateNames() const;
    std::optional<MissionTemplate> getTemplate(const std::string& name) const;
    
    // Create mission from intent
    std::optional<swarm_msgs::msg::Mission> createMissionFromIntent(
        const swarm::interface::ParsedIntent& intent);
    
    // Upload mission to swarm
    UploadResult uploadMission(const swarm_msgs::msg::Mission& mission,
                               const std::vector<uint16_t>& target_drones);
    
    // Upload mission from intent directly
    UploadResult uploadFromIntent(const swarm::interface::ParsedIntent& intent,
                                  const std::vector<uint16_t>& target_drones);
    
    // Validate mission
    bool validateMission(const swarm_msgs::msg::Mission& mission,
                         std::string& error_message) const;
    
    // Cancel active mission
    bool cancelMission(const std::string& mission_id);
    
    // Get upload status
    std::optional<MissionUploadStatus> getUploadStatus(const std::string& mission_id) const;
    
    // Get all active uploads
    std::vector<MissionUploadStatus> getAllUploadStatuses() const;
    
    // Service callbacks
    using UploadMissionCallback = std::function<void(const swarm_msgs::srv::UploadMission::Request::SharedPtr,
                                                      swarm_msgs::srv::UploadMission::Response::SharedPtr)>;
    void setUploadMissionCallback(UploadMissionCallback cb);
    
    using ValidateMissionCallback = std::function<void(const swarm_msgs::srv::ValidateMission::Request::SharedPtr,
                                                        swarm_msgs::srv::ValidateMission::Response::SharedPtr)>;
    void setValidateMissionCallback(ValidateMissionCallback cb);
    
    // Upload complete callback
    using UploadCompleteCallback = std::function<void(const UploadResult&)>;
    void setUploadCompleteCallback(UploadCompleteCallback cb);
    
    // Get config
    const MissionUploaderConfig& getConfig() const { return config_; }
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    MissionUploaderConfig config_;
    bool initialized_ = false;
    
    std::function<void(const UploadResult&)> upload_complete_callback_;
    std::function<void(const swarm_msgs::srv::UploadMission::Request::SharedPtr,
                       swarm_msgs::srv::UploadMission::Response::SharedPtr)> upload_mission_callback_;
    std::function<void(const swarm_msgs::srv::ValidateMission::Request::SharedPtr,
                       swarm_msgs::srv::ValidateMission::Response::SharedPtr)> validate_mission_callback_;
};

}  // namespace swarm::interface

#endif  // MISSION_UPLOADER_HPP