#include "rclcpp/rclcpp.hpp"
#include "swarm_msgs/msg/human_intent_cmd.hpp"
#include "swarm_msgs/msg/mission.hpp"
#include "swarm_msgs/msg/mission_status.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/srv/upload_mission.hpp"
#include "swarm_msgs/srv/validate_mission.hpp"
#include "swarm_interface_mission_uploader/mission_uploader.hpp"
#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

class MissionUploaderNode : public rclcpp::Node {
public:
    MissionUploaderNode() : Node("mission_uploader_node") {
        declareParameters();
        loadParameters();
        
        // Initialize mission uploader
        mission_uploader_ = std::make_unique<swarm::interface::MissionUploader>();
        swarm::interface::MissionUploaderConfig uploader_config;
        
        uploader_config.auto_upload_on_intent = get_parameter("auto_upload_on_intent").as_bool();
        uploader_config.validate_before_upload = get_parameter("validate_before_upload").as_bool();
        uploader_config.upload_timeout_s = get_parameter("upload_timeout_s").as_int();
        uploader_config.max_retry_count = get_parameter("max_retry_count").as_int();
        uploader_config.mission_templates_path = get_parameter("mission_templates_path").as_string();
        uploader_config.require_ground_station_approval = get_parameter("require_ground_station_approval").as_bool();
        uploader_config.max_waypoints = get_parameter("max_waypoints").as_int();
        uploader_config.max_mission_duration_s = get_parameter("max_mission_duration_s").as_double();
        uploader_config.min_altitude_m = get_parameter("min_altitude_m").as_double();
        uploader_config.max_altitude_m = get_parameter("max_altitude_m").as_double();
        
        if (!mission_uploader_->initialize(uploader_config)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize mission uploader");
        }
        
        // Setup subscriptions
        setupSubscriptions();
        
        // Setup publishers
        setupPublishers();
        
        // Setup services
        setupServices();
        
        // Set callbacks
        mission_uploader_->setUploadCompleteCallback(
            std::bind(&MissionUploaderNode::onUploadComplete, this, std::placeholders::_1));
        
        mission_uploader_->setUploadMissionCallback(
            std::bind(&MissionUploaderNode::onUploadMissionRequest, this, 
                      std::placeholders::_1, std::placeholders::_2));
        
        mission_uploader_->setValidateMissionCallback(
            std::bind(&MissionUploaderNode::onValidateMissionRequest, this,
                      std::placeholders::_1, std::placeholders::_2));
        
        // Timer for status
        status_timer_ = create_wall_timer(
            5s, std::bind(&MissionUploaderNode::publishStatus, this));
        
        RCLCPP_INFO(get_logger(), "Mission Uploader Node initialized");
    }
    
private:
    // Configuration
    void declareParameters() {
        // Upload settings
        declare_parameter("auto_upload_on_intent", true);
        declare_parameter("validate_before_upload", true);
        declare_parameter("upload_timeout_s", 10);
        declare_parameter("max_retry_count", 3);
        declare_parameter("mission_templates_path", "/config/mission_templates.json");
        declare_parameter("require_ground_station_approval", false);
        declare_parameter("max_waypoints", 100);
        declare_parameter("max_mission_duration_s", 3600.0);
        declare_parameter("min_altitude_m", 10.0);
        declare_parameter("max_altitude_m", 120.0);
        declare_parameter("drone_id", 1);
        
        // Swarm members for upload targets
        declare_parameter("swarm_members", std::vector<int64_t>{1, 2, 3, 4, 5});
    }
    
    void loadParameters() {
        drone_id_ = get_parameter("drone_id").as_int();
        auto members = get_parameter("swarm_members").as_integer_array();
        swarm_members_.clear();
        for (auto m : members) {
            swarm_members_.push_back(static_cast<uint16_t>(m));
        }
    }
    
    void setupSubscriptions() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        // Human intent command input
        intent_sub_ = create_subscription<swarm_msgs::msg::HumanIntentCmd>(
            "/drone" + std::to_string(drone_id_) + "/human_intent_cmd", qos,
            [this](const swarm_msgs::msg::HumanIntentCmd::SharedPtr msg) {
                onIntentReceived(msg);
            });
        
        // Mission status updates from drones
        mission_status_sub_ = create_subscription<swarm_msgs::msg::MissionStatus>(
            "/swarm/mission_status", qos,
            [this](const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
                onMissionStatus(msg);
            });
        
        // Agent state for context
        agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
            "/swarm/agent_states", rclcpp::QoS(10).best_effort(),
            [this](const swarm_msgs::msg::AgentState::SharedPtr msg) {
                if (msg->sysid == drone_id_) {
                    latest_agent_state_ = *msg;
                    has_agent_state_ = true;
                }
            });
    }
    
    void setupPublishers() {
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        
        // Uploaded mission
        mission_pub_ = create_publisher<swarm_msgs::msg::Mission>(
            "/drone" + std::to_string(drone_id_) + "/uploaded_mission", qos);
        
        // Upload result
        upload_result_pub_ = create_publisher<swarm_msgs::msg::MissionUploadResult>(
            "/drone" + std::to_string(drone_id_) + "/mission_upload_result", qos);
        
        // Mission status
        mission_status_pub_ = create_publisher<swarm_msgs::msg::MissionStatus>(
            "/drone" + std::to_string(drone_id_) + "/mission_status", qos);
    }
    
    void setupServices() {
        // Upload mission service
        upload_service_ = create_service<swarm_msgs::srv::UploadMission>(
            "/drone" + std::to_string(drone_id_) + "/upload_mission",
            [this](const std::shared_ptr<rmw_request_id_t>,
                   const std::shared_ptr<swarm_msgs::srv::UploadMission::Request> request,
                   std::shared_ptr<swarm_msgs::srv::UploadMission::Response> response) {
                handleUploadMission(request, response);
            });
        
        // Validate mission service
        validate_service_ = create_service<swarm_msgs::srv::ValidateMission>(
            "/drone" + std::to_string(drone_id_) + "/validate_mission",
            [this](const std::shared_ptr<rmw_request_id_t>,
                   const std::shared_ptr<swarm_msgs::srv::ValidateMission::Request> request,
                   std::shared_ptr<swarm_msgs::srv::ValidateMission::Response> response) {
                handleValidateMission(request, response);
            });
    }
    
    void onIntentReceived(const swarm_msgs::msg::HumanIntentCmd::SharedPtr msg) {
        if (!uploader_config_.auto_upload_on_intent) return;
        
        // Convert HumanIntentCmd to ParsedIntent
        swarm::interface::ParsedIntent intent;
        intent.timestamp_us = msg->timestamp_us;
        intent.drone_id = msg->drone_id;
        intent.type = static_cast<swarm::interface::IntentType>(msg->intent_type);
        intent.raw_text = msg->command;
        intent.confidence = msg->confidence;
        intent.language = msg->language;
        intent.parameters = parseJsonToMap(msg->parameters);
        intent.priority = static_cast<swarm::interface::IntentPriority>(msg->priority);
        intent.source = static_cast<swarm::interface::IntentSource>(msg->source_id == 0 ? 0 : 1);  // Simplified
        
        // Auto-upload if enabled
        if (uploader_config_.auto_upload_on_intent) {
            auto result = mission_uploader_->uploadFromIntent(intent, swarm_members_);
            
            // Publish result
            publishUploadResult(result);
            
            if (result.success) {
                RCLCPP_INFO(get_logger(), "Auto-uploaded mission %s for intent: %s", 
                           result.mission_id.c_str(), intent.raw_text.c_str());
            } else {
                RCLCPP_WARN(get_logger(), "Auto-upload failed: %s", result.error_message.c_str());
            }
        }
    }
    
    void onMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
        // Update upload status based on drone feedback
        // Would track ACKs, progress, etc.
    }
    
    void handleUploadMission(
        const std::shared_ptr<swarm_msgs::srv::UploadMission::Request> request,
        std::shared_ptr<swarm_msgs::srv::UploadMission::Response> response) {
        
        // Deserialize mission from request
        swarm_msgs::msg::Mission mission = request->mission;
        
        // Validate if requested
        std::string error;
        if (uploader_config_.validate_before_upload) {
            std::string error;
            if (!mission_uploader_->validateMission(mission, error)) {
                response->success = false;
                response->message = "Validation failed: " + error;
                return;
            }
        }
        
        // Upload to target drones
        std::vector<uint16_t> targets;
        if (request->target_drones.empty()) {
            // Default to all swarm members except self
            for (uint16_t id : swarm_members_) {
                if (id != drone_id_) targets.push_back(id);
            }
        } else {
            for (auto id : request->target_drones) {
                targets.push_back(static_cast<uint16_t>(id));
            }
        }
        
        auto result = mission_uploader_->uploadMission(mission, targets);
        
        response->success = result.success;
        response->mission_id = result.mission_id;
        response->message = result.success ? "Upload successful" : result.error_message;
        
        // Publish result
        publishUploadResult(result);
    }
    
    void handleValidateMission(
        const std::shared_ptr<swarm_msgs::srv::ValidateMission::Request> request,
        std::shared_ptr<swarm_msgs::srv::ValidateMission::Response> response) {
        
        swarm_msgs::msg::Mission mission = request->mission;
        std::string error;
        bool valid = mission_uploader_->validateMission(mission, error);
        
        response->valid = valid;
        response->error_message = error;
        
        if (!valid) {
            RCLCPP_WARN(get_logger(), "Mission validation failed: %s", error.c_str());
        }
    }
    
    void onMissionStatus(const swarm_msgs::msg::MissionStatus::SharedPtr msg) {
        // Handle mission status updates from drones
        // Track ACKs, progress, completion
    }
    
    void onUploadComplete(const swarm::interface::UploadResult& result) {
        RCLCPP_INFO(get_logger(), "Upload complete: %s (success: %d)", 
                   result.mission_id.c_str(), result.success);
        publishUploadResult(result);
    }
    
    void publishUploadResult(const swarm::interface::UploadResult& result) {
        auto msg = swarm_msgs::msg::MissionUploadResult();
        msg.timestamp_us = result.upload_time_us;
        msg.mission_id = result.mission_id;
        msg.success = result.success;
        msg.error_message = result.error_message;
        
        for (const auto& [id, ack] : result.drone_acks) {
            msg.acknowledged_drones.push_back(id);
        }
        
        upload_result_pub_->publish(msg);
    }
    
    void handleUploadMission(
        const std::shared_ptr<swarm_msgs::srv::UploadMission::Request> request,
        std::shared_ptr<swarm_msgs::srv::UploadMission::Response> response) {
        // Wrapper for service callback
        handleUploadMission(request, response);
    }
    
    void handleValidateMission(
        const std::shared_ptr<swarm_msgs::srv::ValidateMission::Request> request,
        std::shared_ptr<swarm_msgs::srv::ValidateMission::Response> response) {
        handleValidateMission(request, response);
    }
    
    void publishStatus() {
        // Publish uploader status
        auto statuses = mission_uploader_->getAllUploadStatuses();
        
        for (const auto& status : statuses) {
            auto msg = swarm_msgs::msg::MissionStatus();
            msg.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            msg.mission_id = status.mission_id;
            msg.state = static_cast<uint8_t>(status.state);
            msg.progress = 0.0f;  // Would compute from ACKs
            msg.remaining_time_s = 0.0f;
            mission_status_pub_->publish(msg);
        }
    }
    
    void publishUploadResult(const swarm::interface::UploadResult& result) {
        auto msg = swarm_msgs::msg::MissionUploadResult();
        msg.timestamp_us = result.upload_time_us;
        msg.mission_id = result.mission_id;
        msg.success = result.success;
        msg.error_message = result.error_message;
        
        for (const auto& [id, ack] : result.drone_acks) {
            msg.acknowledged_drones.push_back(id);
        }
        
        upload_result_pub_->publish(msg);
    }
    
    std::map<std::string, std::string> parseJsonToMap(const std::string& json_str) {
        std::map<std::string, std::string> result;
        try {
            auto j = nlohmann::json::parse(json_str);
            for (auto& [key, value] : j.items()) {
                result[key] = value.is_string() ? value.get<std::string>() : value.dump();
            }
        } catch (...) {
            // Ignore parse errors
        }
        return result;
    }
    
    // Members
    std::unique_ptr<swarm::interface::MissionUploader> mission_uploader_;
    swarm::interface::MissionUploaderConfig uploader_config_;
    uint16_t drone_id_ = 1;
    std::vector<uint16_t> swarm_members_;
    
    // ROS interfaces
    rclcpp::Subscription<swarm_msgs::msg::HumanIntentCmd>::SharedPtr intent_sub_;
    rclcpp::Subscription<swarm_msgs::msg::MissionStatus>::SharedPtr mission_status_sub_;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    
    rclcpp::Publisher<swarm_msgs::msg::Mission>::SharedPtr mission_pub_;
    rclcpp::Publisher<swarm_msgs::msg::MissionUploadResult>::SharedPtr upload_result_pub_;
    rclcpp::Publisher<swarm_msgs::msg::MissionStatus>::SharedPtr mission_status_pub_;
    
    rclcpp::Service<swarm_msgs::srv::UploadMission>::SharedPtr upload_service_;
    rclcpp::Service<swarm_msgs::srv::ValidateMission>::SharedPtr validate_service_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    
    swarm::interface::MissionUploaderConfig uploader_config_;
    swarm_msgs::msg::AgentState latest_agent_state_;
    bool has_agent_state_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MissionUploaderNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}