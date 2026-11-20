#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "swarm_msgs/msg/ai_inference.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_ai_perception/obstacle_detector/obstacle_detector.hpp"
#include "swarm_ai_perception/obstacle_detector/localization.hpp"
#include "swarm_ai_perception/obstacle_detector/threat_assessment.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

class ObstacleNode : public rclcpp::Node {
public:
    ObstacleNode() : Node("obstacle_detector_node") {
        declareParameters();
        loadParameters();
        
        // Initialize obstacle detector
        obstacle_detector_ = std::make_unique<swarm::ai::ObstacleDetector>();
        if (!obstacle_detector_->initialize(detector_config_)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize obstacle detector");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize localization engine
        localization_engine_ = std::make_unique<swarm::ai::LocalizationEngine>();
        if (!localization_engine_->initialize(localization_config_)) {
            RCLCPP_WARN(get_logger(), "Localization engine not initialized (no camera calibration)");
        }
        
        // Initialize threat assessment
        threat_assessment_.configure(threat_config_);
        
        // Setup subscribers
        setupSubscribers();
        
        // Setup publishers
        setupPublishers();
        
        // Setup timer for periodic processing
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / detector_config_.target_fps),
            std::bind(&ObstacleNode::processCallback, this));
        
        // Set callbacks
        obstacle_detector_->setAIInferenceCallback(
            std::bind(&ObstacleNode::publishAIInference, this, std::placeholders::_1));
        
        obstacle_detector_->setDetectionCallback(
            std::bind(&ObstacleNode::publishDetectionResult, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Obstacle detector node started at %d Hz", detector_config_.target_fps);
    }
    
private:
    // Configuration
    swarm::ai::ObstacleDetectorConfig detector_config_;
    swarm::ai::LocalizationEngine::Config localization_config_;
    swarm::ai::ThreatAssessment::Config threat_config_;
    
    // Components
    std::unique_ptr<swarm::ai::ObstacleDetector> obstacle_detector_;
    std::unique_ptr<swarm::ai::LocalizationEngine> localization_engine_;
    swarm::ai::ThreatAssessment threat_assessment_;
    
    // ROS interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Image transport
    image_transport::ImageTransport::SharedPtr it_;
    image_transport::Subscriber left_sub_;
    image_transport::Subscriber right_sub_;
    image_transport::Subscriber depth_sub_;
    
    // Camera info
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
    
    // Agent state for ego velocity
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    
    // Publishers
    rclcpp::Publisher<swarm_msgs::msg::AIInference>::SharedPtr ai_inference_pub_;
    rclcpp::Publisher<swarm_msgs::msg::ObstacleArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<swarm_msgs::msg::ThreatAssessment>::SharedPtr threat_pub_;
    image_transport::Publisher::SharedPtr debug_image_pub_;
    
    // State
    cv::Mat latest_left_image_;
    cv::Mat latest_right_image_;
    cv::Mat latest_depth_image_;
    bool has_left_image_ = false;
    bool has_right_image_ = false;
    bool has_depth_image_ = false;
    bool has_stereo_calib_ = false;
    Eigen::Vector3f ego_velocity_ = Eigen::Vector3f::Zero();
    uint16_t drone_id_ = 1;
    
    // Mutexes
    std::mutex left_mutex_, right_mutex_, depth_mutex_;
    
    void declareParameters() {
        // Detector parameters
        declare_parameter("model_path", "");
        declare_parameter("model_type", "yolov8n");
        declare_parameter("confidence_threshold", 0.5);
        declare_parameter("nms_threshold", 0.45);
        declare_parameter("power_line_threshold", 0.7);
        declare_parameter("input_width", 640);
        declare_parameter("input_height", 640);
        declare_parameter("target_fps", 20);
        declare_parameter("enable_stereo", true);
        declare_parameter("enable_depth", true);
        declare_parameter("baseline_m", 0.12);
        declare_parameter("focal_length_px", 450.0);
        declare_parameter("drone_id", 1);
        
        // Topics
        declare_parameter("left_image_topic", "/camera/left/image_raw");
        declare_parameter("right_image_topic", "/camera/right/image_raw");
        declare_parameter("depth_topic", "/camera/depth/image_raw");
        declare_parameter("left_info_topic", "/camera/left/camera_info");
        declare_parameter("right_info_topic", "/camera/right/camera_info");
        declare_parameter("agent_state_topic", "/swarm/agent_states");
        
        // Threat assessment
        declare_parameter("time_horizon_s", 5.0);
        declare_parameter("critical_distance_m", 50.0);
        declare_parameter("critical_ttc_s", 3.0);
        declare_parameter("immediate_avoidance_score", 0.8);
    }
    
    void loadParameters() {
        // Detector config
        detector_config_.model_path = get_parameter("model_path").as_string();
        detector_config_.model_type = get_parameter("model_type").as_string();
        detector_config_.confidence_threshold = get_parameter("confidence_threshold").as_double();
        detector_config_.nms_threshold = get_parameter("nms_threshold").as_double();
        detector_config_.power_line_threshold = get_parameter("power_line_threshold").as_double();
        detector_config_.input_width = get_parameter("input_width").as_int();
        detector_config_.input_height = get_parameter("input_height").as_int();
        detector_config_.target_fps = get_parameter("target_fps").as_int();
        detector_config_.enable_stereo = get_parameter("enable_stereo").as_bool();
        detector_config_.enable_depth = get_parameter("enable_depth").as_bool();
        detector_config_.baseline_m = get_parameter("baseline_m").as_double();
        detector_config_.focal_length_px = get_parameter("focal_length_px").as_double();
        drone_id_ = get_parameter("drone_id").as_int();
        
        detector_config_.engine_config.preferred_runtime = "tensorrt";
        detector_config_.engine_config.fallback_runtime = "onnxruntime";
        
        // Localization config (will be updated from camera_info)
        localization_config_.baseline_m = detector_config_.baseline_m;
        localization_config_.focal_length_px = detector_config_.focal_length_px;
        localization_config_.depth_scale = 0.001;  // mm to meters
        
        // Threat config
        threat_config_.time_horizon_s = get_parameter("time_horizon_s").as_double();
        threat_config_.critical_distance_m = get_parameter("critical_distance_m").as_double();
        threat_config_.critical_ttc_s = get_parameter("critical_ttc_s").as_double();
        threat_config_.immediate_avoidance_score = get_parameter("immediate_avoidance_score").as_double();
    }
    
    void setupSubscribers() {
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Left camera
        left_sub_ = it_->subscribe(
            get_parameter("left_image_topic").as_string(), 1,
            std::bind(&ObstacleNode::leftImageCallback, this, std::placeholders::_1));
        
        // Right camera (stereo)
        if (detector_config_.enable_stereo) {
            right_sub_ = it_->subscribe(
                get_parameter("right_image_topic").as_string(), 1,
                std::bind(&ObstacleNode::rightImageCallback, this, std::placeholders::_1));
        }
        
        // Depth camera
        if (detector_config_.enable_depth) {
            depth_sub_ = it_->subscribe(
                get_parameter("depth_topic").as_string(), 1,
                std::bind(&ObstacleNode::depthImageCallback, this, std::placeholders::_1));
        }
        
        // Camera info for calibration
        left_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            get_parameter("left_info_topic").as_string(), 1,
            std::bind(&ObstacleNode::leftInfoCallback, this, std::placeholders::_1));
        
        right_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            get_parameter("right_info_topic").as_string(), 1,
            std::bind(&ObstacleNode::rightInfoCallback, this, std::placeholders::_1));
        
        // Agent state for ego velocity
        agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
            get_parameter("agent_state_topic").as_string(), 10,
            std::bind(&ObstacleNode::agentStateCallback, this, std::placeholders::_1));
    }
    
    void setupPublishers() {
        // AI Inference (MAVLink/ROS bridge)
        ai_inference_pub_ = create_publisher<swarm_msgs::msg::AIInference>(
            "/drone" + std::to_string(drone_id_) + "/ai_inference", 10);
        
        // Obstacle array
        obstacles_pub_ = create_publisher<swarm_msgs::msg::ObstacleArray>(
            "/drone" + std::to_string(drone_id_) + "/obstacles", 10);
        
        // Threat assessment
        threat_pub_ = create_publisher<swarm_msgs::msg::ThreatAssessment>(
            "/drone" + std::to_string(drone_id_) + "/threat_assessment", 10);
        
        // Debug image
        debug_image_pub_ = it_->advertise(
            "/drone" + std::to_string(drone_id_) + "/obstacle_debug", 1);
    }
    
    void leftImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::lock_guard<std::mutex> lock(left_mutex_);
            latest_left_image_ = img.clone();
            has_left_image_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Left image conversion failed: %s", e.what());
        }
    }
    
    void rightImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::lock_guard<std::mutex> lock(right_mutex_);
            latest_right_image_ = img.clone();
            has_right_image_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Right image conversion failed: %s", e.what());
        }
    }
    
    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "16UC1")->image;
            std::lock_guard<std::mutex> lock(depth_mutex_);
            latest_depth_image_ = img.clone();
            has_depth_image_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Depth image conversion failed: %s", e.what());
        }
    }
    
    void leftInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (!has_stereo_calib_) {
            // Extract camera matrix
            cv::Mat K(3, 3, CV_64F);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    K.at<double>(i, j) = msg->k[i * 3 + j];
                }
            }
            localization_config_.camera_matrix = K;
            
            // Extract distortion coefficients
            cv::Mat D(1, msg->d.size(), CV_64F);
            for (size_t i = 0; i < msg->d.size(); ++i) {
                D.at<double>(0, i) = msg->d[i];
            }
            localization_config_.dist_coeffs = D;
            
            has_stereo_calib_ = true;
            RCLCPP_INFO(get_logger(), "Left camera calibration received");
        }
    }
    
    void rightInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (has_stereo_calib_ && localization_config_.R.empty()) {
            // For now, assume rectified stereo with horizontal baseline
            localization_config_.R = cv::Mat::eye(3, 3, CV_64F);
            localization_config_.T = cv::Mat::zeros(3, 1, CV_64F);
            localization_config_.T.at<double>(0, 0) = -localization_config_.baseline_m;
            
            // Update localization engine
            localization_engine_->initialize(localization_config_);
        }
    }
    
    void agentStateCallback(const swarm_msgs::msg::AgentState::ConstSharedPtr& msg) {
        if (msg->sysid == drone_id_) {
            ego_velocity_ = Eigen::Vector3f(
                msg->velocity.x, msg->velocity.y, msg->velocity.z);
        }
    }
    
    void processCallback() {
        if (!has_left_image_) return;
        
        // Get latest images
        cv::Mat left_img, right_img, depth_img;
        {
            std::lock_guard<std::mutex> lock(left_mutex_);
            left_img = latest_left_image_.clone();
        }
        
        if (detector_config_.enable_stereo) {
            std::lock_guard<std::mutex> lock(right_mutex_);
            right_img = latest_right_image_.clone();
        }
        
        if (detector_config_.enable_depth) {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            depth_img = latest_depth_image_.clone();
        }
        
        uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        // Run detection
        swarm::ai::ObstacleDetector::DetectionResult result;
        
        if (detector_config_.enable_stereo && has_right_image_ && !right_img.empty()) {
            result = obstacle_detector_->detectStereo(left_img, right_img, drone_id_, timestamp_us);
        } else if (detector_config_.enable_depth && has_depth_image_ && !depth_img.empty()) {
            result = obstacle_detector_->detectWithDepth(left_img, depth_img, drone_id_, timestamp_us);
        } else {
            result = obstacle_detector_->detect(left_img, drone_id_, timestamp_us);
        }
        
        // Refine 3D positions using localization engine
        if (has_depth_image_ && !depth_img.empty()) {
            refineWithLocalization(result, depth_img);
        }
        
        // Threat assessment
        swarm::ai::ThreatAssessment::ThreatResult threat = 
            threat_assessment_.assessMultiple(result.obstacles, ego_velocity_);
        
        // Publish threat assessment
        publishThreatAssessment(threat, timestamp_us);
        
        // Publish debug image
        publishDebugImage(left_img, result, threat);
    }
    
    void refineWithLocalization(swarm::ai::ObstacleDetector::DetectionResult& result,
                                const cv::Mat& depth_map) {
        for (auto& obs : result.obstacles) {
            Eigen::Vector3f position, size;
            Eigen::Quaternionf orientation;
            
            if (localization_engine_->localizeObstacle(obs, depth_map, position, size, orientation)) {
                obs.position_enu = position;
                obs.size = size;
                obs.orientation = orientation;
            }
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
        
        msg.metadata.obstacle.threat_level = static_cast<uint8_t>(inf.metadata.obstacle.threat_level);
        msg.metadata.obstacle.relative_velocity = {inf.metadata.obstacle.relative_velocity.x(),
                                                    inf.metadata.obstacle.relative_velocity.y(),
                                                    inf.metadata.obstacle.relative_velocity.z()};
        msg.metadata.obstacle.time_to_collision_s = inf.metadata.obstacle.time_to_collision_s;
        msg.metadata.obstacle.requires_immediate_avoidance = inf.metadata.obstacle.requires_immediate_avoidance;
        
        ai_inference_pub_->publish(msg);
    }
    
    void publishDetectionResult(const swarm::ai::ObstacleDetector::DetectionResult& result) {
        auto msg = swarm_msgs::msg::ObstacleArray();
        msg.timestamp_us = result.timestamp_us;
        msg.drone_id = result.drone_id;
        msg.frame_id = result.frame_id;
        
        for (const auto& obs : result.obstacles) {
            swarm_msgs::msg::Obstacle obstacle;
            obstacle.track_id = obs.track_id;
            obstacle.class_id = obs.class_id;
            obstacle.class_name = obs.class_name;
            obstacle.confidence = obs.confidence;
            obstacle.bbox_2d = {obs.bbox_2d.x(), obs.bbox_2d.y(), obs.bbox_2d.z(), obs.bbox_2d.w()};
            obstacle.position_enu = {obs.position_enu.x(), obs.position_enu.y(), obs.position_enu.z()};
            obstacle.velocity_enu = {obs.velocity_enu.x(), obs.velocity_enu.y(), obs.velocity_enu.z()};
            obstacle.size = {obs.size.x(), obs.size.y(), obs.size.z()};
            obstacle.orientation = {obs.orientation.w(), obs.orientation.x(), obs.orientation.y(), obs.orientation.z()};
            obstacle.threat_level = static_cast<uint8_t>(obs.metadata.obstacle.threat_level);
            obstacle.time_to_collision_s = obs.metadata.obstacle.time_to_collision_s;
            obstacle.requires_immediate_avoidance = obs.metadata.obstacle.requires_immediate_avoidance;
            
            msg.obstacles.push_back(obstacle);
        }
        
        obstacles_pub_->publish(msg);
    }
    
    void publishThreatAssessment(const swarm::ai::ThreatAssessment::ThreatResult& threat,
                                  uint64_t timestamp_us) {
        auto msg = swarm_msgs::msg::ThreatAssessment();
        msg.timestamp_us = timestamp_us;
        msg.drone_id = drone_id_;
        msg.threat_level = static_cast<uint8_t>(threat.threat_level);
        msg.threat_score = threat.threat_score;
        msg.time_to_collision_s = threat.time_to_collision_s;
        msg.requires_immediate_avoidance = threat.requires_immediate_avoidance;
        msg.avoidance_vector = {threat.avoidance_vector.x(), threat.avoidance_vector.y(), threat.avoidance_vector.z()};
        msg.avoidance_urgency = threat.avoidance_urgency;
        msg.reasoning = threat.reasoning;
        
        threat_pub_->publish(msg);
    }
    
    void publishDebugImage(const cv::Mat& image,
                           const swarm::ai::ObstacleDetector::DetectionResult& result,
                           const swarm::ai::ThreatAssessment::ThreatResult& threat) {
        cv::Mat debug = image.clone();
        
        // Draw detections
        for (const auto& obs : result.obstacles) {
            cv::Point pt1(obs.bbox_2d[0], obs.bbox_2d[1]);
            cv::Point pt2(obs.bbox_2d[2], obs.bbox_2d[3]);
            
            // Color by threat level
            cv::Scalar color;
            switch (obs.metadata.obstacle.threat_level) {
                case swarm::ai::ThreatLevel::CRITICAL: color = cv::Scalar(0, 0, 255); break;
                case swarm::ai::ThreatLevel::HIGH: color = cv::Scalar(0, 165, 255); break;
                case swarm::ai::ThreatLevel::MEDIUM: color = cv::Scalar(0, 255, 255); break;
                default: color = cv::Scalar(0, 255, 0); break;
            }
            
            cv::rectangle(debug, pt1, pt2, color, 2);
            
            // Label
            std::string label = obs.class_name + " " + std::to_string(obs.confidence).substr(0, 4);
            if (obs.metadata.obstacle.time_to_collision_s > 0) {
                label += " TTC:" + std::to_string(obs.metadata.obstacle.time_to_collision_s).substr(0, 4);
            }
            cv::putText(debug, label, pt1 - cv::Point(0, 5),
                       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
        }
        
        // Draw overall threat
        std::string threat_text = "Threat: " + std::to_string(threat.threat_score).substr(0, 4);
        cv::putText(debug, threat_text, cv::Point(10, 30),
                   cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 2);
        
        if (threat.requires_immediate_avoidance) {
            cv::putText(debug, "IMMEDIATE AVOIDANCE REQUIRED", cv::Point(10, 70),
                       cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
        
        // Publish
        try {
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug).toImageMsg();
            debug_image_pub_->publish(msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Debug image publish failed: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}