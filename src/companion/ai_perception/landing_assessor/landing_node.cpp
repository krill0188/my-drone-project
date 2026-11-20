#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "image_transport/image_transport.hpp"
#include "swarm_msgs/msg/ai_inference.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/srv/assess_landing_site.hpp"
#include "swarm_ai_perception/landing_assessor/landing_classifier.hpp"
#include "swarm_ai_perception/landing_assessor/downward_processor.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

class LandingNode : public rclcpp::Node {
public:
    LandingNode() : Node("landing_assessor_node") {
        declareParameters();
        loadParameters();
        
        // Initialize landing classifier
        landing_classifier_ = std::make_unique<swarm::ai::LandingClassifier>();
        if (!landing_classifier_->initialize(classifier_config_)) {
            RCLCPP_ERROR(get_logger(), "Failed to initialize landing classifier");
            rclcpp::shutdown();
            return;
        }
        
        // Initialize downward processor
        downward_processor_ = std::make_unique<swarm::ai::DownwardProcessor>();
        if (!downward_processor_->initialize(processor_config_)) {
            RCLCPP_WARN(get_logger(), "Downward processor not initialized");
        }
        
        // Setup subscribers
        setupSubscribers();
        
        // Setup publishers
        setupPublishers();
        
        // Setup service
        setupService();
        
        // Setup timer for periodic assessment
        timer_ = create_wall_timer(
            std::chrono::milliseconds(1000 / classifier_config_.target_fps),
            std::bind(&LandingNode::assessmentCallback, this));
        
        // Set callbacks
        landing_classifier_->setAIInferenceCallback(
            std::bind(&LandingNode::publishAIInference, this, std::placeholders::_1));
        
        landing_classifier_->setAssessmentCallback(
            std::bind(&LandingNode::publishLandingAssessment, this, std::placeholders::_1));
        
        downward_processor_->setDataCallback(
            std::bind(&LandingNode::onDownwardData, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "Landing assessor node started at %d Hz", classifier_config_.target_fps);
    }
    
private:
    // Configuration
    swarm::ai::LandingClassifierConfig classifier_config_;
    swarm::ai::DownwardProcessorConfig processor_config_;
    
    // Components
    std::unique_ptr<swarm::ai::LandingClassifier> landing_classifier_;
    std::unique_ptr<swarm::ai::DownwardProcessor> downward_processor_;
    
    // ROS interfaces
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<swarm_msgs::srv::AssessLandingSite>::SharedPtr assess_service_;
    
    // Image transport
    image_transport::ImageTransport::SharedPtr it_;
    image_transport::Subscriber downward_sub_;
    image_transport::Subscriber depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    
    // Agent state
    rclcpp::Subscription<swarm_msgs::msg::AgentState>::SharedPtr agent_state_sub_;
    
    // Publishers
    rclcpp::Publisher<swarm_msgs::msg::AIInference>::SharedPtr ai_inference_pub_;
    rclcpp::Publisher<swarm_msgs::msg::LandingAssessment>::SharedPtr landing_assessment_pub_;
    image_transport::Publisher::SharedPtr debug_image_pub_;
    
    // State
    cv::Mat latest_downward_image_;
    cv::Mat latest_depth_image_;
    bool has_downward_image_ = false;
    bool has_depth_image_ = false;
    bool has_camera_calib_ = false;
    Eigen::Vector3f drone_position_enu_ = Eigen::Vector3f::Zero();
    Eigen::Quaternionf drone_orientation_ = Eigen::Quaternionf::Identity();
    Eigen::Vector3f drone_velocity_enu_ = Eigen::Vector3f::Zero();
    uint16_t drone_id_ = 1;
    
    // Mutexes
    std::mutex downward_mutex_, depth_mutex_;
    
    // Pending service requests
    struct PendingRequest {
        rclcpp::Service<swarm_msgs::srv::AssessLandingSite>::SharedPtr service;
        std::shared_ptr<swarm_msgs::srv::AssessLandingSite::Request> request;
        std::shared_ptr<swarm_msgs::srv::AssessLandingSite::Response> response;
    };
    std::vector<PendingRequest> pending_requests_;
    std::mutex pending_mutex_;
    
    void declareParameters() {
        // Classifier parameters
        declare_parameter("model_path", "");
        declare_parameter("model_type", "mobilenetv3");
        declare_parameter("confidence_threshold", 0.6);
        declare_parameter("slope_threshold_deg", 10.0);
        declare_parameter("min_clearance_radius_m", 2.0);
        declare_parameter("min_firmness_score", 0.5);
        declare_parameter("input_width", 224);
        declare_parameter("input_height", 224);
        declare_parameter("target_fps", 10);
        declare_parameter("drone_id", 1);
        
        // Topics
        declare_parameter("downward_image_topic", "/camera/downward/image_raw");
        declare_parameter("depth_topic", "/camera/depth/image_raw");
        declare_parameter("camera_info_topic", "/camera/downward/camera_info");
        declare_parameter("agent_state_topic", "/swarm/agent_states");
        
        // Processor parameters
        declare_parameter("processor_focal_length_px", 450.0);
        declare_parameter("processor_enable_stabilization", true);
        declare_parameter("processor_enable_optical_flow", true);
    }
    
    void loadParameters() {
        // Classifier config
        classifier_config_.model_path = get_parameter("model_path").as_string();
        classifier_config_.model_type = get_parameter("model_type").as_string();
        classifier_config_.confidence_threshold = get_parameter("confidence_threshold").as_double();
        classifier_config_.slope_threshold_deg = get_parameter("slope_threshold_deg").as_double();
        classifier_config_.min_clearance_radius_m = get_parameter("min_clearance_radius_m").as_double();
        classifier_config_.min_firmness_score = get_parameter("min_firmness_score").as_double();
        classifier_config_.input_width = get_parameter("input_width").as_int();
        classifier_config_.input_height = get_parameter("input_height").as_int();
        classifier_config_.target_fps = get_parameter("target_fps").as_int();
        drone_id_ = get_parameter("drone_id").as_int();
        
        classifier_config_.engine_config.preferred_runtime = "tensorrt";
        classifier_config_.engine_config.fallback_runtime = "onnxruntime";
        
        // Processor config
        processor_config_.image_width = classifier_config_.input_width;
        processor_config_.image_height = classifier_config_.input_height;
        processor_config_.focal_length_px = get_parameter("processor_focal_length_px").as_double();
        processor_config_.enable_stabilization = get_parameter("processor_enable_stabilization").as_bool();
        processor_config_.enable_optical_flow = get_parameter("processor_enable_optical_flow").as_bool();
    }
    
    void setupSubscribers() {
        it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this());
        
        // Downward camera
        downward_sub_ = it_->subscribe(
            get_parameter("downward_image_topic").as_string(), 1,
            std::bind(&LandingNode::downwardImageCallback, this, std::placeholders::_1));
        
        // Depth camera
        depth_sub_ = it_->subscribe(
            get_parameter("depth_topic").as_string(), 1,
            std::bind(&LandingNode::depthImageCallback, this, std::placeholders::_1));
        
        // Camera info
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            get_parameter("camera_info_topic").as_string(), 1,
            std::bind(&LandingNode::cameraInfoCallback, this, std::placeholders::_1));
        
        // Agent state
        agent_state_sub_ = create_subscription<swarm_msgs::msg::AgentState>(
            get_parameter("agent_state_topic").as_string(), 10,
            std::bind(&LandingNode::agentStateCallback, this, std::placeholders::_1));
    }
    
    void setupPublishers() {
        // AI Inference
        ai_inference_pub_ = create_publisher<swarm_msgs::msg::AIInference>(
            "/drone" + std::to_string(drone_id_) + "/ai_inference", 10);
        
        // Landing assessment
        landing_assessment_pub_ = create_publisher<swarm_msgs::msg::LandingAssessment>(
            "/drone" + std::to_string(drone_id_) + "/landing_assessment", 10);
        
        // Debug image
        debug_image_pub_ = it_->advertise(
            "/drone" + std::to_string(drone_id_) + "/landing_debug", 1);
    }
    
    void setupService() {
        assess_service_ = create_service<swarm_msgs::srv::AssessLandingSite>(
            "/drone" + std::to_string(drone_id_) + "/assess_landing_site",
            std::bind(&LandingNode::assessLandingService, this, 
                      std::placeholders::_1, std::placeholders::_2));
    }
    
    void downwardImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        try {
            cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
            std::lock_guard<std::mutex> lock(downward_mutex_);
            latest_downward_image_ = img.clone();
            has_downward_image_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "Downward image conversion failed: %s", e.what());
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
    
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg) {
        if (!has_camera_calib_) {
            cv::Mat K(3, 3, CV_64F);
            for (int i = 0; i < 3; ++i) {
                for (int j = 0; j < 3; ++j) {
                    K.at<double>(i, j) = msg->k[i * 3 + j];
                }
            }
            
            cv::Mat D(1, msg->d.size(), CV_64F);
            for (size_t i = 0; i < msg->d.size(); ++i) {
                D.at<double>(0, i) = msg->d[i];
            }
            
            downward_processor_->setCameraCalibration(K, D);
            has_camera_calib_ = true;
            RCLCPP_INFO(get_logger(), "Downward camera calibration received");
        }
    }
    
    void agentStateCallback(const swarm_msgs::msg::AgentState::ConstSharedPtr& msg) {
        if (msg->sysid == drone_id_) {
            drone_position_enu_ = Eigen::Vector3f(msg->position.x, msg->position.y, msg->position.z);
            drone_orientation_ = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, 
                                                   msg->orientation.y, msg->orientation.z);
            drone_velocity_enu_ = Eigen::Vector3f(msg->velocity.x, msg->velocity.y, msg->velocity.z);
        }
    }
    
    void assessmentCallback() {
        if (!has_downward_image_) return;
        
        // Get latest images
        cv::Mat downward_img, depth_img;
        {
            std::lock_guard<std::mutex> lock(downward_mutex_);
            downward_img = latest_downward_image_.clone();
        }
        
        if (has_depth_image_) {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            depth_img = latest_depth_image_.clone();
        }
        
        // Process through downward processor first
        if (downward_processor_->isInitialized()) {
            downward_processor_->process(downward_img, 
                std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count(),
                drone_position_enu_, drone_orientation_, drone_velocity_enu_);
        }
        
        // Run landing assessment
        uint64_t timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        
        // Get ground normal from downward processor if available
        Eigen::Vector3f ground_normal = Eigen::Vector3f::UnitZ();
        if (downward_processor_->isInitialized()) {
            auto proc_data = downward_processor_->getLatestData();
            if (proc_data.ground_plane_valid) {
                ground_normal = proc_data.ground_normal_enu;
            }
        }
        
        // Assess landing site
        swarm::ai::LandingClassifier::LandingAssessment assessment;
        if (!depth_img.empty()) {
            assessment = landing_classifier_->assess(downward_img, drone_id_, timestamp_us,
                                                     depth_img, drone_position_enu_, drone_orientation_);
        } else {
            assessment = landing_classifier_->assessWithNormal(downward_img, drone_id_, timestamp_us,
                                                                ground_normal, depth_img);
        }
        
        // Process pending service requests
        processPendingRequests(assessment);
    }
    
    void onDownwardData(const swarm::ai::DownwardProcessor::DownwardData& data) {
        // Downward processor callback - data already processed
    }
    
    void assessLandingService(
        const std::shared_ptr<swarm_msgs::srv::AssessLandingSite::Request> request,
        std::shared_ptr<swarm_msgs::srv::AssessLandingSite::Response> response) {
        
        // Queue request for next assessment cycle
        std::lock_guard<std::mutex> lock(pending_mutex_);
        pending_requests_.push_back({assess_service_, request, response});
    }
    
    void processPendingRequests(const swarm::ai::LandingClassifier::LandingAssessment& assessment) {
        std::lock_guard<std::mutex> lock(pending_mutex_);
        
        for (auto& req : pending_requests_) {
            fillServiceResponse(req.response, assessment);
        }
        pending_requests_.clear();
    }
    
    void fillServiceResponse(
        std::shared_ptr<swarm_msgs::srv::AssessLandingSite::Response> response,
        const swarm::ai::LandingClassifier::LandingAssessment& assessment) {
        
        response->timestamp_us = assessment.timestamp_us;
        response->drone_id = assessment.drone_id;
        response->suitable = (assessment.suitability >= swarm::ai::LandingSuitability::SUITABLE);
        response->suitability_score = assessment.suitability_score;
        response->surface_type = static_cast<uint8_t>(assessment.surface_type);
        response->slope_deg = assessment.slope_deg;
        response->clearance_radius_m = assessment.clearance_radius_m;
        response->firmness_score = assessment.firmness_score;
        response->has_obstructions = assessment.has_obstructions;
        response->recommended_offset_x = assessment.recommended_offset_m.x();
        response->recommended_offset_y = assessment.recommended_offset_m.y();
        response->wind_exposure = assessment.wind_exposure;
        response->roughness_index = assessment.roughness_index;
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
        
        msg.metadata.landing.slope_deg = inf.metadata.landing.slope_deg;
        msg.metadata.landing.clearance_radius_m = inf.metadata.landing.clearance_radius_m;
        msg.metadata.landing.surface_type = inf.metadata.landing.surface_type;
        msg.metadata.landing.firmness_score = inf.metadata.landing.firmness_score;
        
        ai_inference_pub_->publish(msg);
    }
    
    void publishLandingAssessment(const swarm::ai::LandingClassifier::LandingAssessment& assessment) {
        auto msg = swarm_msgs::msg::LandingAssessment();
        msg.timestamp_us = assessment.timestamp_us;
        msg.drone_id = assessment.drone_id;
        msg.suitable = (assessment.suitability >= swarm::ai::LandingSuitability::SUITABLE);
        msg.suitability_score = assessment.suitability_score;
        msg.surface_type = static_cast<uint8_t>(assessment.surface_type);
        msg.surface_confidence = assessment.surface_confidence;
        msg.slope_deg = assessment.slope_deg;
        msg.slope_confidence = assessment.slope_confidence;
        msg.normal_enu = {assessment.normal_enu.x(), assessment.normal_enu.y(), assessment.normal_enu.z()};
        msg.clearance_radius_m = assessment.clearance_radius_m;
        msg.clearance_confidence = assessment.clearance_confidence;
        msg.has_obstructions = assessment.has_obstructions;
        msg.firmness_score = assessment.firmness_score;
        msg.firmness_confidence = assessment.firmness_confidence;
        msg.suitability = static_cast<uint8_t>(assessment.suitability);
        msg.recommended_offset = {assessment.recommended_offset_m.x(), assessment.recommended_offset_m.y()};
        msg.wind_exposure = assessment.wind_exposure;
        msg.roughness_index = assessment.roughness_index;
        
        landing_assessment_pub_->publish(msg);
    }
    
    void publishDebugImage(const cv::Mat& image,
                           const swarm::ai::LandingClassifier::LandingAssessment& assessment) {
        cv::Mat debug = image.clone();
        
        // Draw assessment info
        std::string suitability_str;
        switch (assessment.suitability) {
            case swarm::ai::LandingSuitability::IDEAL: suitability_str = "IDEAL"; break;
            case swarm::ai::LandingSuitability::SUITABLE: suitability_str = "SUITABLE"; break;
            case swarm::ai::LandingSuitability::MARGINAL: suitability_str = "MARGINAL"; break;
            default: suitability_str = "UNSUITABLE"; break;
        }
        
        cv::Scalar color;
        if (assessment.suitability >= swarm::ai::LandingSuitability::SUITABLE) color = cv::Scalar(0, 255, 0);
        else if (assessment.suitability == swarm::ai::LandingSuitability::MARGINAL) color = cv::Scalar(0, 255, 255);
        else color = cv::Scalar(0, 0, 255);
        
        cv::putText(debug, "Landing: " + suitability_str + " (" + 
                    std::to_string(assessment.suitability_score).substr(0, 4) + ")",
                   cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, color, 2);
        
        cv::putText(debug, "Surface: " + swarm::ai::LANDING_SURFACE_NAMES[static_cast<int>(assessment.surface_type)] +
                    " (" + std::to_string(assessment.surface_confidence).substr(0, 4) + ")",
                   cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        cv::putText(debug, "Slope: " + std::to_string(assessment.slope_deg).substr(0, 4) + " deg",
                   cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        cv::putText(debug, "Clearance: " + std::to_string(assessment.clearance_radius_m).substr(0, 4) + " m",
                   cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        cv::putText(debug, "Firmness: " + std::to_string(assessment.firmness_score).substr(0, 4),
                   cv::Point(10, 150), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255), 1);
        
        if (assessment.has_obstructions) {
            cv::putText(debug, "OBSTRUCTIONS DETECTED", cv::Point(10, 180),
                       cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
        }
        
        // Draw clearance circle
        int cx = debug.cols / 2;
        int cy = debug.rows / 2;
        int radius_px = 50;  // Visual representation
        cv::circle(debug, cv::Point(cx, cy), radius_px, color, 2);
        
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
    auto node = std::make_shared<LandingNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}