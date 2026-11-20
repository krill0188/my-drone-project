#ifndef LANDMARK_LOCALIZATION_HPP
#define LANDMARK_LOCALIZATION_HPP

#include "../../../common/ai_inference.hpp"
#include <Eigen/Dense>
#include <vector>
#include <map>
#include <string>
#include <optional>
#include <deque>

namespace swarm::autonomy {

struct LandmarkLocalizationConfig {
    // Camera parameters
    struct CameraConfig {
        int image_width = 1280;
        int image_height = 720;
        float focal_length_px = 800.0f;
        Eigen::Matrix3f camera_matrix;
        Eigen::VectorXf dist_coeffs;  // k1, k2, p1, p2, k3
    } camera;
    
    // Landmark database
    struct LandmarkDBConfig {
        std::string database_path;  // Path to landmark database
        float max_match_distance_m = 100.0f;
        float min_landmark_size_m = 0.5f;
        float max_landmark_size_m = 50.0f;
        int min_matches_for_pose = 4;
        float reprojection_error_threshold_px = 3.0f;
    } landmark_db;
    
    // Visual odometry integration
    struct VOConfig {
        bool enable_vo_fusion = true;
        float vo_weight = 0.7f;
        float landmark_weight = 0.3f;
        int keyframe_interval = 10;
    } vo;
    
    // Feature detection
    struct FeatureConfig {
        int max_features = 1000;
        float feature_quality = 0.01f;
        int min_distance = 10;
        int patch_size = 31;
        bool use_orb = true;  // ORB vs SIFT
    } feature;
    
    // Matching
    struct MatchingConfig {
        float ratio_threshold = 0.75f;  // Lowe's ratio test
        int max_matches_per_landmark = 50;
        float geometric_verification_threshold = 0.01f;  // Sampson distance
        bool use_ransac = true;
        int ransac_iterations = 1000;
        float ransac_threshold = 3.0f;  // pixels
    } matching;
    
    // Pose optimization
    struct OptimizationConfig {
        bool enable_bundle_adjustment = true;
        int ba_window_size = 10;  // Keyframes
        int ba_max_iterations = 50;
        float huber_threshold = 1.0f;
    } optimization;
    
    // Output
    int output_rate_hz = 20;
    bool publish_debug = true;
};

struct Landmark {
    int id;
    std::string name;
    std::string type;  // "building", "tower", "bridge", "marker", "natural"
    Eigen::Vector3f position_enu;  // Surveyed ENU position
    Eigen::Vector3f size;          // Approximate 3D size
    std::vector<cv::KeyPoint> keypoints;  // Reference keypoints
    cv::Mat descriptors;              // Reference descriptors
    float visibility_score = 1.0f;    // 0-1, how often visible
    std::vector<int> connected_landmarks;  // Graph edges
};

struct VisualLandmarkObservation {
    int landmark_id;
    std::vector<cv::KeyPoint> image_keypoints;
    cv::Mat descriptors;
    std::vector<cv::DMatch> matches;
    Eigen::Vector3f bearing_enu;  // Unit vector to landmark
    float distance_estimate_m = -1.0f;
    float confidence = 0.0f;
    int num_inliers = 0;
};

struct LandmarkPoseEstimate {
    uint64_t timestamp_us;
    uint16_t drone_id;
    bool valid = false;
    
    // Absolute pose (ENU)
    Eigen::Vector3f position_enu;
    Eigen::Quaternionf orientation_enu;
    Eigen::Vector3f velocity_enu;
    
    // Covariance
    Eigen::Matrix3f position_cov;
    Eigen::Matrix3f orientation_cov;
    
    // Quality metrics
    int num_landmarks_matched = 0;
    int num_landmarks_inliers = 0;
    float avg_reprojection_error_px = 0.0f;
    float confidence = 0.0f;
    
    // Geometric dilution of precision
    float gdop = 0.0f;
    
    // VO fusion
    bool vo_fused = false;
    float vo_weight_used = 0.0f;
};

class LandmarkLocalization {
public:
    LandmarkLocalization();
    ~LandmarkLocalization();
    
    LandmarkLocalization(const LandmarkLocalization&) = delete;
    LandmarkLocalization& operator=(const LandmarkLocalization&) = delete;
    LandmarkLocalization(LandmarkLocalization&&) noexcept;
    LandmarkLocalization& operator=(LandmarkLocalization&&) noexcept;
    
    // Initialize
    bool initialize(const LandmarkLocalizationConfig& config);
    
    // Load landmark database
    bool loadLandmarkDatabase(const std::string& path);
    
    // Add landmark programmatically
    void addLandmark(const Landmark& landmark);
    
    // Process new camera frame
    LandmarkPoseEstimate processFrame(const cv::Mat& image, uint64_t timestamp_us,
                                      const Eigen::Vector3f& vo_position_enu = Eigen::Vector3f::Zero(),
                                      const Eigen::Quaternionf& vo_orientation_enu = Eigen::Quaternionf::Identity(),
                                      const Eigen::Vector3f& vo_velocity_enu = Eigen::Vector3f::Zero());
    
    // Get current estimate
    std::optional<LandmarkPoseEstimate> getLatestEstimate() const;
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get config
    const LandmarkLocalizationConfig& getConfig() const { return config_; }
    
    // Get loaded landmarks
    const std::map<int, Landmark>& getLandmarks() const;
    
    // Reset
    void reset();
    
    // Set callback
    using EstimateCallback = std::function<void(const LandmarkPoseEstimate&)>;
    void setEstimateCallback(EstimateCallback cb) { estimate_callback_ = cb; }
    
    // Stats
    struct Stats {
        uint64_t total_frames = 0;
        uint64_t successful_estimates = 0;
        uint64_t failed_estimates = 0;
        float avg_matches = 0.0f;
        float avg_inliers = 0.0f;
        float avg_reprojection_error = 0.0f;
        float avg_computation_ms = 0.0f;
    };
    const Stats& getStats() const { return stats_; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    LandmarkLocalizationConfig config_;
    bool initialized_ = false;
    
    Stats stats_;
    std::mutex stats_mutex_;
    
    std::function<void(const LandmarkPoseEstimate&)> estimate_callback_;
};

}  // namespace swarm::autonomy

#endif  // LANDMARK_LOCALIZATION_HPP