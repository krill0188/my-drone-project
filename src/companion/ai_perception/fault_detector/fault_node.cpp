#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "mavros_msgs/msg/esc_status.hpp"
#include "mavros_msgs/msg/esc_status_item.hpp"
#include "swarm_msgs/msg/ai_inference.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/msg/system_health.hpp"
#include "swarm_msgs/srv/assess_fault.hpp"
#include "swarm_ai_perception/fault_detector/fault_classifier.hpp"
#include "swarm_ai_perception/fault_detector/health_monitor.hpp"
#include "swarm_ai_perception/fault_detector/severity_assessment.hpp"
#include <chrono>
#include <memory>
#include <mutex>
#include <deque>

using namespace std::chrono_literals;

class FaultNode : public rclcpp::Node {
public:
    FaultNode() : Node("fault_detector_node") {
        declareParameters();
        loadParameters();
        
        // Initialize components
        fault_classifier_ = std::make_unique<swarm::ai::FaultClassifier>();
        if (!fault_classifier_->initialize(classifier_config_)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize fault classifier");
            rclcpp::shutdown();
            return;
        }
        
        health_monitor_ = std::make_unique<swarm::ai::HealthMonitor>();
        if (!health_monitor_->initialize(health_config_)) {
            RCLCPP_WARN(get_logger(), "Health monitor not initialized");
        }
        
        severity_assessment_.configure(severity_config_);
        
        // Setup subscribers
        setupSubscribers();
        
        // Setup publishers
        setupPublishers();
        
        // Setup service
        setupService();
        
        // Setup timer for periodic detection
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / classifier_config_.target_fps),
            std::bind(&FaultNode::detectionCallback, this));
        
        // Set callbacks
        fault_classifier_->setAIInferenceCallback(
            std::bind(&FaultNode::publishAIInference, this, std::placeholders::_1));
        
        fault_classifier_->setDetectionCallback(
            std::bind(&FaultNode::publishFaultDetection, this, std::placeholders::_1));
        
        health_monitor_->setHealthCallback(
            std::bind(&FaultNode::publishSystemHealth, this, std::placeholders::_1));
        
        health_monitor_->setSensorAlertCallback(
            std::bind(&FaultNode::onSensorAlert, this, std::placeholders::_1, 
                      std::placeholders::_2, std::placeholders::_3));
        
        RCLCPP_INFO(get_logger(), "Fault detector node started at %d Hz", classifier_config_.target_fps);
    }
    
private:
    // Configuration
    swarm::ai::FaultClassifierConfig classifier_config_;
    swarm::ai::HealthMonitorConfig health_config_;
    swarm::ai::SeverityAssessmentConfig severity_config_;
    
    // Components
    std::unique_ptr<swarm::ai::FaultClassifier> fault_classifier_;
    std::unique_ptr<swarm::ai::HealthMonitor> health_monitor_;
    swarm::ai::SeverityAssessment severity_assessment_;
    
    // ROS interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<swarm_msgs::srv::AssessFault>::SharedPtr assess_service_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_sub_;
    rclcpp::Subscription<mavros_msgs::msg::EscStatus>::SharedPtr esc_sub_;
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    rclcpp::Subscription<swarm_msgs::msg::CommStatus>::SharedPtr comm_sub_;
    
    // Publishers
    rclcpp::Publisher<swarm_msgs::msg::AIInference>::SharedPtr ai_inference_pub_;
    rclcpp::Publisher<swarm_msgs::msg::FaultDetection>::SharedPtr fault_detection_pub_;
    rclcpp::Publisher<swarm_msgs::msg::SystemHealth>::SharedPtr system_health_pub_;
    rclcpp::Publisher<swarm_msgs::msg::SeverityAssessment>::SharedPtr severity_pub_;
    
    // State
    swarm::ai::FaultClassifier::SensorFrame latest_frame_;
    bool has_imu_ = false;
    bool has_gps_ = false;
    bool has_battery_ = false;
    bool has_esc_ = false;
    bool has_comm_ = false;
    Eigen::Vector3f drone_position_enu_ = Eigen::Vector3f::Zero();
    Eigen::Quaternionf drone_orientation_ = Eigen::Quaternionf::Identity();
    Eigen::Vector3f drone_velocity_enu_ = Eigen::Vector3f::Zero();
    bool is_leader_ = false;
    uint16_t drone_id_ = 1;
    std::vector<uint16_t> formation_members_;
    
    // Mutexes
    std::mutex frame_mutex_;
    
    // Pending service requests
    struct PendingRequest {
        std::shared_ptr<swarm_msgs::srv::AssessFault::Request> request;
        std::shared_ptr<swarm_msgs::srv::AssessFault::Response> response;
    };
    std::vector<PendingRequest> pending_requests_;
    std::mutex pending_mutex_;
    
    void declareParameters() {
        // Classifier parameters
        declare_parameter("model_path", "");
        declare_parameter("model_type", "lstm");
        declare_parameter("sequence_length", 50);
        declare_parameter("sequence_stride", 5);
        declare_parameter("num_features", 128);
        declare_parameter("num_motors", 4);
        declare_parameter("num_cells", 6);
        declare_parameter("confidence_threshold", 0.7);
        declare_parameter("target_fps", 10);
        declare_parameter("drone_id", 1);
        
        // Health monitor parameters
        declare_parameter("baseline_window_s", 60);
        declare_parameter("anomaly_threshold_sigma", 3.0);
        
        // Severity parameters
        declare_parameter("enable_multi_fault_escalation", true);
        declare_parameter("enable_time_escalation", true);
        declare_parameter("consider_formation_role", true);
        
        // Topics
        declare_parameter("imu_topic", "/mavros/imu/data");
        declare_parameter("gps_topic", "/mavros/global_position/global");
        declare_parameter("battery_topic", "/mavros/battery");
        declare_parameter("esc_topic", "/mavros/esc_status");
        declare_parameter("agent_state_topic", "/swarm/agent_states");
        declare_parameter("comm_topic", "/swarm/comm_status");
    }
    
    void loadParameters() {
        // Classifier config
        classifier_config_.model_path = get_parameter("model_path").as_string();
        classifier_config_.model_type = get_parameter("model_type").as_string();
        classifier_config_.sequence_length = get_parameter("sequence_length").as_int();
        classifier_config_.sequence_stride = get_parameter("sequence_stride").as_int();
        classifier_config_.num_features = get_parameter("num_features").as_int();
        classifier_config_.num_motors = get_parameter("num_motors").as_int();
        classifier_config_.num_cells = get_parameter("num_cells").as_int();
        classifier_config_.confidence_threshold = get_parameter("confidence_threshold").as_double();
        classifier_config_.target_fps = get_parameter("target_fps").as_int();
        drone_id_ = get_parameter("drone_id").as_int();
        
        classifier_config_.engine_config.preferred_runtime = "tensorrt";
        classifier_config_.engine_config.fallback_runtime = "onnxruntime";
        
        // Health config
        health_config_.baseline_window_s = get_parameter("baseline_window_s").as_int();
        health_config_.anomaly_threshold_sigma = get_parameter("anomaly_threshold_sigma").as_double();
        
        // Severity config
        severity_config_.enable_multi_fault_escalation = get_parameter("enable_multi_fault_escalation").as_bool();
        severity_config_.enable_time_escalation = get_parameter("enable_time_escalation").as_bool();
        severity_config_.consider_formation_role = get_parameter("consider_formation_role").as_bool();
    }
    
    void setupSubscribers() {
        // IMU
        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            get_parameter("imu_topic").as_string(), 10,
            std::bind(&FaultNode::imuCallback, this, std::placeholders::_1));
        
        // GPS
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            get_parameter("gps_topic").as_string(), 10,
            std::bind(&FaultNode::gpsCallback, this, std::placeholders::_1));
        
        // Battery
        battery_sub_ = create_subscription<sensor_msgs::msg::BatteryState>(
            get_parameter("battery_topic").as_string(), 10,
            std::bind(&FaultNode::batteryCallback, this, std::placeholders::_1));
        
        // ESC Status
        esc_sub_ = create_subscription<mavros_msgs::msg::EscStatus>(
            get_parameter("esc_topic").as_string(), 10,
            std::bind(&FaultNode::escCallback, this, std::placeholders::_1));
        
        // Agent state
        agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
            get_parameter("agent_state_topic").as_string(), 10,
            std::bind(&FaultNode::agentStateCallback, this, std::placeholders::_1));
        
        // Comm status (custom message)
        // comm_sub_ = create_subscription<swarm_msgs::msg::CommStatus>(...);
    }
    
    void setupPublishers() {
        // AI Inference
        ai_inference_pub_ = create_publisher<swarm_msgs::msg::AIInference>(
            "/drone" + std::to_string(drone_id_) + "/ai_inference", 10);
        
        // Fault detection
        fault_detection_pub_ = create_publisher<swarm_msgs::msg::FaultDetection>(
            "/drone" + std::to_string(drone_id_) + "/fault_detection", 10);
        
        // System health
        system_health_pub_ = create_publisher<swarm_msgs::msg::SystemHealth>(
            "/drone" + std::to_string(drone_id_) + "/system_health", 10);
        
        // Severity assessment
        severity_pub_ = create_publisher<swarm_msgs::msg::SeverityAssessment>(
            "/drone" + std::to_string(drone_id_) + "/severity_assessment", 10);
    }
    
    void setupService() {
        assess_service_ = create_service<swarm_msgs::srv::AssessFault>(
            "/drone" + std::to_string(drone_id_) + "/assess_fault",
            std::bind(&FaultNode::assessFaultService, this, 
                      std::placeholders::_1, std::placeholders::_2));
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_.accel = Eigen::Vector3f(msg->linear_acceleration.x,
                                               msg->linear_acceleration.y,
                                               msg->linear_acceleration.z);
        latest_frame_.gyro = Eigen::Vector3f(msg->angular_velocity.x,
                                              msg->angular_velocity.y,
                                              msg->angular_velocity.z);
        // Magnetometer would come from separate topic
        latest_frame_.mag = Eigen::Vector3f::Zero();
        has_imu_ = true;
    }
    
    void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        // Convert lat/lon to ENU (simplified - would use proper projection)
        latest_frame_.gps_pos_enu = Eigen::Vector3f(msg->longitude, msg->latitude, msg->altitude);
        latest_frame_.gps_hdop = msg->position_covariance[0] > 0 ? 
            std::sqrt(msg->position_covariance[0]) : 1.0f;
        latest_frame_.gps_satellites = 0;  // Would come from separate topic
        latest_frame_.gps_valid = (msg->status.status >= sensor_msgs::msg::NavSatStatus::STATUS_FIX);
        has_gps_ = true;
    }
    
    void batteryCallback(const sensor_msgs::msg::BatteryState::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_.battery_voltage = msg->voltage;
        latest_frame_.battery_current = msg->current;
        latest_frame_.battery_temp = 25.0f;  // Would come from separate sensor
        latest_frame_.battery_soc = msg->percentage;
        latest_frame_.cell_voltages = std::vector<float>(msg->cell_voltage.begin(), msg->cell_voltage.end());
        has_battery_ = true;
    }
    
    void escCallback(const mavros_msgs::msg::EscStatus::ConstSharedPtr& msg) {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        latest_frame_.motor_rpm.clear();
        latest_frame_.motor_current.clear();
        latest_frame_.motor_voltage.clear();
        latest_frame_.motor_temp.clear();
        latest_frame_.esc_temp.clear();
        
        for (const auto& esc : msg->esc) {
            latest_frame_.motor_rpm.push_back(esc.rpm);
            latest_frame_.motor_current.push_back(esc.current);
            latest_frame_.motor_voltage.push_back(esc.voltage);
            latest_frame_.motor_temp.push_back(esc.temperature);
            latest_frame_.esc_temp.push_back(esc.temperature);
        }
        has_esc_ = true;
    }
    
    void agentStateCallback(const swarm_msgs::msg::AgentState::ConstSharedPtr& msg) {
        if (msg->sysid == drone_id_) {
            drone_position_enu_ = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
            drone_orientation_ = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x,
                                                   msg->orientation.y, msg->orientation.z);
            drone_velocity_enu_ = Eigen::Vector3f(msg->velocity.x, msg->velocity.y, msg->velocity.z);
            is_leader_ = msg->is_leader;
            formation_members_ = msg->formation_members;
        }
    }
    
    void detectionCallback() {
        // Add current frame to classifier
        {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            latest_frame_.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            fault_classifier_->addSensorFrame(latest_frame_);
        }
        
        // Update health monitor
        health_monitor_->updateFromFrame(latest_frame_, latest_frame_.timestamp_us);
        
        // Run fault detection
        swarm::ai::FaultClassifier::FaultDetection detection = 
            fault_classifier_->detect(drone_id_, latest_frame_.timestamp_us);
        
        if (detection.total_ms > 0) {
            // Assess severity
            auto health = health_monitor_->getHealth(drone_id_, latest_frame_.timestamp_us);
            swarm::ai::SeverityAssessment::SeverityResult severity = 
                severity_assessment_.assess({detection}, health, is_leader_,
                                           drone_position_enu_, formation_members_);
            
            // Update detection with severity info
            detection.severity = severity.fault_severities.count(detection.fault_type) ?
                severity.fault_severities.at(detection.fault_type) : FaultSeverity::INFO;
            detection.recommended_action = severity.fault_actions.count(detection.fault_type) ?
                severity.fault_actions.at(detection.fault_type) : RecommendedAction::CONTINUE;
            detection.requires_replan = severity.requires_replan;
            
            // Publish severity
            publishSeverityAssessment(severity, latest_frame_.timestamp_us);
        }
    }
    
    void assessFaultService(
        const std::shared_ptr<swarm_msgs::srv::AssessFault::Request> request,
        std::shared_ptr<swarm_msgs::srv::AssessFault::Response> response) {
        
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_requests_.push_back({request, response});
    }
    
    void processPendingRequests(const swarm::ai::SeverityAssessment::SeverityResult& severity) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        
        for (auto& req : pending_requests_) {
            fillServiceResponse(req.response, severity);
        }
        pending_requests_.clear();
    }
    
    void fillServiceResponse(
        std::shared_ptr<swarm_msgs::srv::AssessFault::Response> response,
        const swarm::ai::SeverityAssessment::SeverityResult& severity) {
        
        response->timestamp_us = severity.timestamp_us;
        response->drone_id = severity.drone_id;
        response->system_severity = static_cast<uint8_t>(severity.system_severity);
        response->system_action = static_cast<uint8_t>(severity.system_action);
        response->requires_replan = severity.requires_replan;
        response->replan_reason = severity.replan_reason;
        response->formation_risk = severity.formation_risk_score;
        response->summary = severity.summary;
        
        for (const auto& [type, sev] : severity.fault_severities) {
            swarm_msgs::msg::FaultSeverity fs;
            fs.fault_type = static_cast<uint8_t>(type);
            fs.severity = static_cast<uint8_t>(sev);
            fs.action = static_cast<uint8_t>(severity.fault_actions.at(type));
            response->faults.push_back(fs);
        }
        
        for (const auto& warn : severity.escalation_warnings) {
            response->warnings.push_back(warn);
        }
    }
    
    void publishAIInference(const swarm::ai::AIInferenceResult& inf) {
        auto msg = swarm_msgs::msg::AIInference();
        msg.timestamp_us = inf.timestamp_us;
        msg.drone_id = inf.drone_id;
        msg.type = static_cast<uint8_t>(inf.type);
        msg.class_id = inf.class_id;
        msg.class_name = inf.class_name;
        msg.confidence = inf.confidence;
        msg.position_enu = {inf.position_enu.x(), inf.position_enu.y(), inf.position_enu.z()};
        msg.size = {inf.size.x(), inf.size.y(), inf.size.z()};
        msg.orientation = {inf.orientation.w(), inf.orientation.x(), inf.orientation.y(), inf.orientation.z()};
        
        msg.metadata.latency_ms = inf.metadata.latency_ms;
        msg.metadata.model_name = inf.metadata.model_name;
        msg.metadata.model_version = inf.metadata.model_version;
        msg.metadata.runtime = inf.metadata.runtime;
        
        msg.metadata.fault.fault_type = inf.metadata.fault.fault_type;
        msg.metadata.fault.severity = inf.metadata.fault.severity;
        msg.metadata.fault.affected_component = inf.metadata.fault.affected_component;
        msg.metadata.fault.degradation_pct = inf.metadata.fault.degradation_pct;
        msg.metadata.fault.requires_replan = inf.metadata.fault.requires_replan;
        msg.metadata.fault.recommended_action = inf.metadata.fault.recommended_action;
        
        ai_inference_pub_->publish(msg);
    }
    
    void publishFaultDetection(const swarm::ai::FaultClassifier::FaultDetection& detection) {
        auto msg = swarm_msgs::msg::FaultDetection();
        msg.timestamp_us = detection.timestamp_us;
        msg.drone_id = detection.drone_id;
        msg.fault_type = static_cast<uint8_t>(detection.fault_type);
        msg.fault_name = detection.fault_name;
        msg.confidence = detection.confidence;
        msg.severity = static_cast<uint8_t>(detection.severity);
        msg.affected_component = detection.affected_component;
        msg.degradation_pct = detection.degradation_pct;
        msg.recommended_action = static_cast<uint8_t>(detection.recommended_action);
        msg.requires_replan = detection.requires_replan;
        msg.time_since_onset_s = detection.time_since_onset_s;
        
        for (float score : detection.class_scores) {
            msg.class_scores.push_back(score);
        }
        
        for (const auto& [key, val] : detection.diagnostics) {
            msg.diagnostic_keys.push_back(key);
            msg.diagnostic_values.push_back(val);
        }
        
        fault_detection_pub_->publish(msg);
    }
    
    void publishSystemHealth(const swarm::ai::HealthMonitor::SystemHealth& health) {
        auto msg = swarm_msgs::msg::SystemHealth();
        msg.timestamp_us = health.timestamp_us;
        msg.drone_id = health.drone_id;
        msg.overall_score = health.overall_score;
        
        for (const auto& [name, sensor] : health.sensors) {
            swarm_msgs::msg::SensorHealth sh;
            sh.name = name;
            sh.health_score = sensor.health_score;
            sh.mean = sensor.mean;
            sh.stddev = sensor.stddev;
            sh.last_value = sensor.last_value;
            sh.last_update_us = sensor.last_update_us;
            sh.latency_ms = sensor.latency_ms;
            sh.is_healthy = sensor.is_healthy;
            sh.anomaly_reason = sensor.anomaly_reason;
            msg.sensors.push_back(sh);
        }
        
        for (const auto& name : health.degraded_sensors) {
            msg.degraded_sensors.push_back(name);
        }
        for (const auto& name : health.failed_sensors) {
            msg.failed_sensors.push_back(name);
        }
        
        for (const auto& [sub, score] : health.subsystem_scores) {
            msg.subsystem_names.push_back(sub);
            msg.subsystem_scores.push_back(score);
        }
        
        system_health_pub_->publish(msg);
    }
    
    void publishSeverityAssessment(const swarm::ai::SeverityAssessment::SeverityResult& severity,
                                    uint64_t timestamp_us) {
        auto msg = swarm_msgs::msg::SeverityAssessment();
        msg.timestamp_us = severity.timestamp_us;
        msg.drone_id = severity.drone_id;
        msg.system_severity = static_cast<uint8_t>(severity.system_severity);
        msg.system_action = static_cast<uint8_t>(severity.system_action);
        msg.requires_replan = severity.requires_replan;
        msg.replan_reason = severity.replan_reason;
        msg.is_leader = severity.is_leader;
        msg.formation_risk_score = severity.formation_risk_score;
        msg.summary = severity.summary;
        
        for (const auto& [type, sev] : severity.fault_severities) {
            swarm_msgs::msg::FaultSeverity fs;
            fs.fault_type = static_cast<uint8_t>(type);
            fs.severity = static_cast<uint8_t>(sev);
            fs.action = static_cast<uint8_t>(severity.fault_actions.at(type));
            msg.faults.push_back(fs);
        }
        
        for (const auto& warn : severity.escalation_warnings) {
            msg.warnings.push_back(warn);
        }
        
        severity_pub_->publish(msg);
        
        // Process pending service requests
        processPendingRequests(severity);
    }
    
    void onSensorAlert(const std::string& sensor_name, float health_score, const std::string& reason) {
        RCLCPP_WARN(get_logger(), "Sensor alert: %s health=%.3f reason=%s", 
                   sensor_name.c_str(), health_score, reason.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FaultNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}