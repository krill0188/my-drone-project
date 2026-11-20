#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "obstacle_detector.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace swarm::ai {

class LocalizationEngine {
public:
    struct Config {
        cv::Mat camera_matrix;      // 3x3 intrinsic matrix
        cv::Mat dist_coeffs;        // Distortion coefficients
        cv::Mat R;                  // Rotation from left to right camera (stereo)
        cv::Mat T;                  // Translation from left to right camera (stereo)
        float baseline_m = 0.12f;   // Stereo baseline in meters
        float focal_length_px = 450.0f;
        float depth_scale = 0.001f; // Depth map scale (mm to meters)
        float max_depth = 100.0f;   // Max valid depth in meters
    };
    
    LocalizationEngine() = default;
    ~LocalizationEngine() = default;
    
    // Initialize with camera calibration
    bool initialize(const Config& config);
    
    // Stereo triangulation
    bool triangulateStereo(const cv::Point2f& pt_left, const cv::Point2f& pt_right,
                           cv::Point3f& point_3d) const;
    
    // Depth map projection
    bool projectDepth(const cv::Point2f& pt, const cv::Mat& depth_map,
                      cv::Point3f& point_3d) const;
    
    // Localize obstacle from bounding box
    bool localizeFromBbox(const cv::Rect2d& bbox, const cv::Mat& depth_map,
                          cv::Point3f& center_3d, cv::Size3f& size_3d) const;
    
    // Localize tracked obstacle
    bool localizeObstacle(const ObstacleDetector::TrackedObstacle& obstacle,
                          const cv::Mat& depth_map,
                          Eigen::Vector3f& position_enu,
                          Eigen::Vector3f& size,
                          Eigen::Quaternionf& orientation) const;
    
    // Get camera matrices as Eigen
    Eigen::Matrix3f getCameraMatrix() const;
    Eigen::Matrix3f getInvCameraMatrix() const;
    
    // Pixel to ray in camera frame
    bool pixelToRay(const cv::Point2f& pt, Eigen::Vector3f& ray) const;
    
    // Ray-plane intersection
    bool rayIntersectPlane(const Eigen::Vector3f& ray_origin,
                           const Eigen::Vector3f& ray_dir,
                           const Eigen::Vector3f& plane_normal,
                           float plane_dist,
                           Eigen::Vector3f& intersection) const;

private:
    Config config_;
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat inv_camera_matrix_;
    cv::Mat R_, T_;
    cv::Mat P1_, P2_;  // Projection matrices
    bool stereo_calibrated_ = false;
    float depth_scale_ = 0.001f;
};

}  // namespace swarm::ai

#endif  // LOCALIZATION_HPP