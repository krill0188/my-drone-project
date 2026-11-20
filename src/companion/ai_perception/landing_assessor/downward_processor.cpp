#include "downward_processor.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <chrono>
#include <algorithm>
#include <iostream>

namespace swarm::ai {

class DownwardProcessor::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    bool initialize(const DownwardProcessorConfig& config) {
        config_ = config;
        
        // Initialize feature detector
        feature_detector_ = cv::GFTTDetector::create(
            config.max_features, config.feature_quality, config.min_distance);
        
        // Initialize optical flow
        if (config.enable_optical_flow) {
            // LK optical flow parameters
            lk_params_ = cv::TrackerLKParams();
            lk_params_.winSize = cv::Size(config.window_size, config.window_size);
            lk_params_.maxLevel = config.pyramid_levels;
        }
        
        // Camera matrix
        if (config_.image_width > 0 && config_.image_height > 0) {
            camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
            camera_matrix_.at<double>(0, 0) = config_.focal_length_px;
            camera_matrix_.at<double>(1, 1) = config_.focal_length_px;
            camera_matrix_.at<double>(0, 2) = config_.image_width / 2.0;
            camera_matrix_.at<double>(1, 2) = config_.image_height / 2.0;
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        }
        
        // Previous frame for optical flow
        prev_gray_.create(config_.image_height, config_.image_width, CV_8UC1);
        has_prev_frame_ = false;
        
        initialized_ = true;
        return true;
    }
    
    DownwardData process(const cv::Mat& image, uint64_t timestamp_us,
                         const Eigen::Vector3f& position_enu,
                         const Eigen::Quaternionf& orientation,
                         const Eigen::Vector3f& velocity_enu) {
        DownwardData data;
        data.timestamp_us = timestamp_us;
        data.position_enu = position_enu;
        data.orientation = orientation;
        data.velocity_enu = velocity_enu;
        
        if (!initialized_ || image.empty()) {
            return data;
        }
        
        // Convert to grayscale
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        
        // Undistort if needed
        cv::Mat undistorted = gray;
        if (config_.enable_undistort && !camera_matrix_.empty()) {
            cv::undistort(gray, undistorted, camera_matrix_, dist_coeffs_);
        }
        
        // Stabilization using optical flow
        cv::Mat stabilized = undistorted;
        if (config_.enable_stabilization && has_prev_frame_) {
            stabilized = stabilizeFrame(undistorted);
        }
        
        // Detect features
        std::vector<cv::Point2f> features;
        feature_detector_->detect(stabilized, features);
        
        // Optical flow
        std::vector<cv::Point2f> flow_vectors;
        Eigen::Vector2f avg_flow(0, 0);
        
        if (config_.enable_optical_flow && has_prev_frame_ && !prev_features_.empty() && !features.empty()) {
            std::vector<uchar> status;
            std::vector<float> err;
            
            cv::calcOpticalFlowPyrLK(prev_gray_, stabilized, prev_features_, features,
                                    status, err, lk_params_.winSize, lk_params_.maxLevel);
            
            // Filter good points
            std::vector<cv::Point2f> good_prev, good_curr;
            for (size_t i = 0; i < status.size(); ++i) {
                if (status[i]) {
                    good_prev.push_back(prev_features_[i]);
                    good_curr.push_back(features[i]);
                    flow_vectors.push_back(features[i] - prev_features_[i]);
                }
            }
            
            // Compute average flow
            if (!flow_vectors.empty()) {
                float sum_x = 0, sum_y = 0;
                for (const auto& f : flow_vectors) {
                    sum_x += f.x;
                    sum_y += f.y;
                }
                avg_flow.x() = sum_x / flow_vectors.size();
                avg_flow.y() = sum_y / flow_vectors.size();
            }
            
            data.prev_features = good_prev;
            data.curr_features = good_curr;
            data.flow_vectors = flow_vectors;
            data.avg_flow_px = avg_flow;
        }
        
        // Update previous frame
        prev_gray_ = stabilized.clone();
        prev_features_ = features;
        has_prev_frame_ = true;
        
        // Estimate ground plane from depth (if available) or from flow
        estimateGroundPlane(data, stabilized);
        
        // Compute quality metrics
        computeQualityMetrics(stabilized, data);
        
        // Store results
        data.image = image.clone();
        data.stabilized_image = stabilized.clone();
        
        latest_data_ = data;
        
        if (data_callback_) data_callback_(data);
        
        return data;
    }
    
    void setCameraCalibration(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
        camera_matrix_ = camera_matrix.clone();
        dist_coeffs_ = dist_coeffs.clone();
    }
    
    const DownwardData& getLatestData() const { return latest_data_; }

private:
    DownwardProcessorConfig config_;
    cv::Ptr<cv::GFTTDetector> feature_detector_;
    cv::TrackerLKParams lk_params_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_features_;
    bool has_prev_frame_ = false;
    DownwardData latest_data_;
    bool initialized_ = false;
    
    std::function<void(const DownwardData&)> data_callback_;
    
    cv::Mat stabilizeFrame(const cv::Mat& current) {
        // Estimate global motion using phase correlation or feature matching
        // For simplicity, use EMA of previous frames
        cv::Mat stabilized;
        if (!prev_stabilized_.empty()) {
            // Simple EMA stabilization
            cv::addWeighted(current, 1.0f - config_.stabilization_alpha,
                          prev_stabilized_, config_.stabilization_alpha, 0, stabilized);
        } else {
            stabilized = current.clone();
        }
        prev_stabilized_ = stabilized.clone();
        return stabilized;
    }
    
    void estimateGroundPlane(DownwardData& data, const cv::Mat& image) {
        if (!config_.enable_ground_plane) return;
        
        // In a real implementation, this would use:
        // 1. Depth map from stereo/ToF/LiDAR
        // 2. Optical flow divergence for monocular depth
        // 3. VIO/IMU for gravity vector
        
        // For now, assume camera is nadir-pointing and ground is flat
        // Ground normal in ENU = -Z body rotated by orientation
        Eigen::Vector3f body_down(0, 0, -1);
        data.ground_normal_enu = data.orientation * body_down;
        data.ground_normal_enu.normalize();
        
        // Ground distance = altitude
        data.ground_distance_m = data.position_enu.z();
        data.ground_plane_valid = data.ground_distance_m > 0.5f && data.ground_distance_m < 100.0f;
        
        // TODO: Refine using optical flow divergence
        // div(flow) ~ -vz / z  (for downward camera)
    }
    
    void computeQualityMetrics(const cv::Mat& image, DownwardData& data) {
        // Image quality: Laplacian variance (focus measure)
        cv::Mat laplacian;
        cv::Laplacian(image, laplacian, CV_32F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplacian, mean, stddev);
        data.image_quality = std::min(1.0f, stddev[0] / 100.0f);
        
        // Texture score: local standard deviation
        cv::Mat blur;
        cv::blur(image, blur, cv::Size(5, 5));
        cv::Mat diff = image - blur;
        cv::Scalar t_mean, t_std;
        cv::meanStdDev(diff, t_mean, t_std);
        data.texture_score = std::min(1.0f, t_std[0] / 50.0f);
        
        // Exposure score: histogram analysis
        cv::Mat hist;
        int bins = 256;
        float range[] = {0, 256};
        const float* hist_range = {range};
        cv::calcHist(&image, 1, 0, cv::Mat(), hist, 1, &bins, &hist_range);
        
        // Check for over/under exposure
        float total = cv::sum(hist)[0];
        float under = 0, over = 0;
        for (int i = 0; i < 20; ++i) under += hist.at<float>(i);
        for (int i = 236; i < 256; ++i) over += hist.at<float>(i);
        data.exposure_score = 1.0f - std::min(1.0f, (under + over) / total * 5.0f);
    }
    
    cv::Mat prev_stabilized_;
};

DownwardProcessor::DownwardProcessor() : pimpl_(std::make_unique<Impl>()) {}
DownwardProcessor::~DownwardProcessor() = default;
DownwardProcessor::DownwardProcessor(DownwardProcessor&&) noexcept = default;
DownwardProcessor& DownwardProcessor::operator=(DownwardProcessor&&) noexcept = default;

bool DownwardProcessor::initialize(const DownwardProcessorConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

DownwardProcessor::DownwardData DownwardProcessor::process(const cv::Mat& image, uint64_t timestamp_us,
                                                            const Eigen::Vector3f& position_enu,
                                                            const Eigen::Quaternionf& orientation,
                                                            const Eigen::Vector3f& velocity_enu) {
    return pimpl_->process(image, timestamp_us, position_enu, orientation, velocity_enu);
}

void DownwardProcessor::setCameraCalibration(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs) {
    pimpl_->setCameraCalibration(camera_matrix, dist_coeffs);
}

bool DownwardProcessor::isInitialized() const {
    return initialized_;
}

void DownwardProcessor::setDataCallback(DataCallback cb) {
    data_callback_ = cb;
}

}  // namespace swarm::ai