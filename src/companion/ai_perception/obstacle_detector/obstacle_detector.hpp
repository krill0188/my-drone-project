#ifndef OBSTACLE_DETECTOR_HPP
#define OBSTACLE_DETECTOR_HPP

#include "../common/ai_inference.hpp"
#include "../model_runtime/inference_engine.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <map>

namespace swarm::ai {

// Obstacle class definitions (matching ObstacleClass enum in ai_inference.hpp)
static const std::vector<std::string> OBSTACLE_CLASS_NAMES = {
    "unknown",           // 0
    "power_line",        // 1
    "tower",             // 2
    "building",          // 3
    "tree",              // 4
    "bird",              // 5
    "drone",             // 6
    "aircraft",          // 7
    "vehicle",           // 8
    "person",            // 9
    "wind_turbine",      // 10
    "crane",             // 11
    "antenna",           // 12
    "balloon",           // 13
    "kite",              // 14
    "wire",              // 15
    "cable",             // 16
    "pole",              // 17
    "fence",             // 18
    "wall"               // 19
};

// Obstacle priority for threat assessment (higher = more dangerous)
static const std::map<int, int> OBSTACLE_PRIORITY = {
    {1, 10},   // power_line - highest priority
    {15, 10},  // wire
    {16, 10},  // cable
    {7, 9},    // aircraft
    {6, 8},    // drone
    {2, 7},    // tower
    {10, 7},   // wind_turbine
    {11, 7},   // crane
    {12, 6},   // antenna
    {17, 6},   // pole
    {5, 5},    // bird
    {3, 4},    // building
    {18, 4},   // fence
    {19, 4},   // wall
    {8, 3},    // vehicle
    {9, 3},    // person
    {4, 2},    // tree
    {13, 2},   // balloon
    {14, 2},   // kite
    {0, 1}     // unknown
};

struct ObstacleDetectorConfig {
    // Model
    std::string model_path;
    std::string model_type = "yolov8n";  // "yolov8n", "yolov8s", "yolov8m", "transformer"
    
    // Inference engine config
    InferenceEngineConfig engine_config;
    
    // Detection thresholds
    float confidence_threshold = 0.5f;
    float nms_threshold = 0.45f;
    float power_line_threshold = 0.7f;  // Higher threshold for critical obstacles
    
    // Input
    int input_width = 640;
    int input_height = 640;
    bool letterbox = true;
    
    // Stereo/depth fusion
    bool enable_stereo = true;
    bool enable_depth = true;
    float baseline_m = 0.12f;  // Stereo baseline in meters
    float focal_length_px = 450.0f;
    
    // Tracking
    bool enable_tracking = true;
    float track_iou_threshold = 0.5f;
    int max_track_age = 30;  // frames
    int min_hits = 3;
    
    // Output
    int max_detections = 100;
    bool publish_3d = true;
    bool publish_tracking = true;
    
    // Performance
    int target_fps = 20;
    bool async_inference = true;
    int inference_queue_size = 2;
};

struct TrackedObstacle {
    int track_id;
    int class_id;
    std::string class_name;
    float confidence;
    Eigen::Vector4f bbox_2d;  // x1, y1, x2, y2 in image coords
    Eigen::Vector3f position_enu;  // 3D position
    Eigen::Vector3f velocity_enu;  // 3D velocity
    Eigen::Vector3f size;  // 3D size
    Eigen::Quaternionf orientation;
    int age = 0;
    int hits = 0;
    int misses = 0;
    bool confirmed = false;
    uint64_t last_update_us = 0;
};

struct DetectionResult {
    uint64_t timestamp_us;
    uint16_t drone_id;
    std::vector<TrackedObstacle> obstacles;
    float preprocess_ms = 0.0f;
    float inference_ms = 0.0f;
    float postprocess_ms = 0.0f;
    float total_ms = 0.0f;
    int frame_id = 0;
};

class ObstacleDetector {
public:
    ObstacleDetector();
    ~ObstacleDetector();
    
    ObstacleDetector(const ObstacleDetector&) = delete;
    ObstacleDetector& operator=(const ObstacleDetector&) = delete;
    ObstacleDetector(ObstacleDetector&&) noexcept;
    ObstacleDetector& operator=(ObstacleDetector&&) noexcept;
    
    // Initialize
    bool initialize(const ObstacleDetectorConfig& config);
    
    // Detect from single camera
    DetectionResult detect(const cv::Mat& image, uint16_t drone_id, uint64_t timestamp_us);
    
    // Detect from stereo pair
    DetectionResult detectStereo(const cv::Mat& left_image, const cv::Mat& right_image,
                                  uint16_t drone_id, uint64_t timestamp_us);
    
    // Detect with depth map
    DetectionResult detectWithDepth(const cv::Mat& image, const cv::Mat& depth_map,
                                     uint16_t drone_id, uint64_t timestamp_us);
    
    // Get current tracks
    std::vector<TrackedObstacle> getTracks() const;
    
    // Clear tracks
    void clearTracks();
    
    // Get config
    const ObstacleDetectorConfig& getConfig() const { return config_; }
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get inference engine
    InferenceEngine* getInferenceEngine() { return inference_engine_.get(); }
    
    // Set callback for detections
    using DetectionCallback = std::function<void(const DetectionResult&)>;
    void setDetectionCallback(DetectionCallback cb) { detection_callback_ = cb; }
    
    // Set callback for AI inference results (for MAVLink/ROS publishing)
    using AIInferenceCallback = std::function<void(const AIInferenceResult&)>;
    void setAIInferenceCallback(AIInferenceCallback cb) { ai_inference_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    ObstacleDetectorConfig config_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    bool initialized_ = false;
    
    std::function<void(const DetectionResult&)> detection_callback_;
    std::function<void(const AIInferenceResult&)> ai_inference_callback_;
};

}  // namespace swarm::ai

#endif  // OBSTACLE_DETECTOR_HPP