#ifndef DOWNWARD_PROCESSOR_HPP
#define DOWNWARD_PROCESSOR_HPP

#include "landing_classifier.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <functional>

namespace swarm::ai {

struct DownwardProcessorConfig {
    // Camera
    int image_width = 640;
    int image_height = 480;
    float focal_length_px = 450.0f;
    
    // Processing
    bool enable_undistort = true;
    bool enable_stabilization = true;
    float stabilization_alpha = 0.9f;  // EMA factor
    
    // Feature detection
    int max_features = 500;
    float feature_quality = 0.01f;
    int min_distance = 10;
    
    // Optical flow
    bool enable_optical_flow = true;
    int pyramid_levels = 3;
    int window_size = 21;
    
    // Depth estimation
    bool enable_depth_from_flow = true;
    float baseline_m = 0.0f;  // For stereo, 0 = monocular
    
    // Ground plane estimation
    bool enable_ground_plane = true;
    int ground_plane_min_points = 100;
    float ground_plane_distance_thresh = 0.1f;
    
    // Output
    bool publish_debug = true;
};

struct DownwardData {
    uint64_t timestamp_us;
    cv::Mat image;
    cv::Mat depth_map;           // Estimated or measured depth
    cv::Mat stabilized_image;
    
    // Camera pose
    Eigen::Vector3f position_enu;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f velocity_enu;
    
    // Ground plane
    Eigen::Vector3f ground_normal_enu;
    float ground_distance_m;
    bool ground_plane_valid = false;
    
    // Optical flow
    std::vector<cv::Point2f> prev_features;
    std::vector<cv::Point2f> curr_features;
    std::vector<cv::Point2f> flow_vectors;
    Eigen::Vector2f avg_flow_px;
    
    // Quality metrics
    float image_quality = 0.0f;      // 0-1
    float texture_score = 0.0f;
    float exposure_score = 0.0f;
};

class DownwardProcessor {
public:
    DownwardProcessor();
    ~DownwardProcessor();
    
    DownwardProcessor(const DownwardProcessor&) = delete;
    DownwardProcessor& operator=(const DownwardProcessor&) = delete;
    DownwardProcessor(DownwardProcessor&&) noexcept;
    DownwardProcessor& operator=(DownwardProcessor&&) noexcept;
    
    // Initialize
    bool initialize(const DownwardProcessorConfig& config);
    
    // Process new downward camera frame
    DownwardData process(const cv::Mat& image, uint64_t timestamp_us,
                         const Eigen::Vector3f& position_enu,
                         const Eigen::Quaternionf& orientation,
                         const Eigen::Vector3f& velocity_enu);
    
    // Set camera calibration
    void setCameraCalibration(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs);
    
    // Get latest processed data
    const DownwardData& getLatestData() const { return latest_data_; }
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Set callback
    using DataCallback = std::function<void(const DownwardData&)>;
    void setDataCallback(DataCallback cb) { data_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    DownwardProcessorConfig config_;
    DownwardData latest_data_;
    bool initialized_ = false;
    
    std::function<void(const DownwardData&)> data_callback_;
};

}  // namespace swarm::ai

#endif  // DOWNWARD_PROCESSOR_HPP