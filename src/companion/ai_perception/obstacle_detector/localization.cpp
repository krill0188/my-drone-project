#include "localization.hpp"
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>

namespace swarm::ai {

LocalizationEngine::LocalizationEngine() = default;
LocalizationEngine::~LocalizationEngine() = default;

bool LocalizationEngine::initialize(const Config& config) {
    config_ = config;
    
    // Validate camera matrix
    if (config.camera_matrix.rows != 3 || config.camera_matrix.cols != 3) {
        return false;
    }
    
    camera_matrix_ = config.camera_matrix.clone();
    dist_coeffs_ = config.dist_coeffs.clone();
    
    // Precompute inverse camera matrix
    camera_matrix_.convertTo(inv_camera_matrix_, CV_64F);
    cv::invert(inv_camera_matrix_, inv_camera_matrix_);
    
    // Stereo calibration
    if (!config.R.empty() && !config.T.empty()) {
        R_ = config.R.clone();
        T_ = config.T.clone();
        stereo_calibrated_ = true;
        
        // Compute projection matrices
        cv::hconcat(camera_matrix_, cv::Mat::zeros(3, 1, CV_64F), P1_);
        cv::Mat RT = cv::Mat::eye(4, 4, CV_64F);
        R_.copyTo(RT(cv::Rect(0, 0, 3, 3)));
        T_.copyTo(RT(cv::Rect(3, 0, 1, 3)));
        cv::Mat P2_temp = camera_matrix_ * RT(cv::Rect(0, 0, 3, 4));
        P2_temp.copyTo(P2_);
    }
    
    // Depth scale
    depth_scale_ = config.depth_scale;
    
    return true;
}

bool LocalizationEngine::triangulateStereo(const cv::Point2f& pt_left, const cv::Point2f& pt_right,
                                            cv::Point3f& point_3d) const {
    if (!stereo_calibrated_) return false;
    
    // Undistort points
    std::vector<cv::Point2f> pts_left = {pt_left}, pts_right = {pt_right};
    std::vector<cv::Point2f> pts_left_ud, pts_right_ud;
    cv::undistortPoints(pts_left, pts_left_ud, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
    cv::undistortPoints(pts_right, pts_right_ud, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
    
    // Triangulate
    cv::Mat pts_4d;
    cv::triangulatePoints(P1_, P2_, pts_left_ud, pts_right_ud, pts_4d);
    
    // Convert from homogeneous
    cv::Mat pt_3d_h = pts_4d.col(0);
    pt_3d_h = pt_3d_h / pt_3d_h.at<double>(3);
    
    point_3d.x = pt_3d_h.at<double>(0);
    point_3d.y = pt_3d_h.at<double>(1);
    point_3d.z = pt_3d_h.at<double>(2);
    
    return point_3d.z > 0;
}

bool LocalizationEngine::projectDepth(const cv::Point2f& pt, const cv::Mat& depth_map,
                                       cv::Point3f& point_3d) const {
    int x = static_cast<int>(pt.x);
    int y = static_cast<int>(pt.y);
    
    if (x < 0 || x >= depth_map.cols || y < 0 || y >= depth_map.rows) {
        return false;
    }
    
    float depth = depth_map.at<uint16_t>(y, x) * depth_scale_;
    if (depth <= 0 || depth > config_.max_depth) {
        return false;
    }
    
    // Use inverse camera matrix
    cv::Mat pt_cam = inv_camera_matrix_ * (cv::Mat_<double>(3, 1) << x, y, 1);
    pt_cam *= depth;
    
    point_3d.x = pt_cam.at<double>(0);
    point_3d.y = pt_cam.at<double>(1);
    point_3d.z = pt_cam.at<double>(2);
    
    return true;
}

bool LocalizationEngine::localizeFromBbox(const cv::Rect2d& bbox, const cv::Mat& depth_map,
                                           cv::Point3f& center_3d, cv::Size3f& size_3d) const {
    // Center point
    cv::Point2f center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
    
    if (!depth_map.empty()) {
        if (!projectDepth(center, depth_map, center_3d)) {
            return false;
        }
    } else if (stereo_calibrated_) {
        // Would need matching point in right image
        return false;
    } else {
        // Monocular - estimate from bbox size and known object size
        // This requires prior knowledge of object size
        return false;
    }
    
    // Estimate 3D size from bbox corners
    std::vector<cv::Point2f> corners = {
        {bbox.x, bbox.y},
        {bbox.x + bbox.width, bbox.y},
        {bbox.x, bbox.y + bbox.height},
        {bbox.x + bbox.width, bbox.y + bbox.height}
    };
    
    std::vector<cv::Point3f> corners_3d;
    for (const auto& c : corners) {
        cv::Point3f p3d;
        if (projectDepth(c, depth_map, p3d)) {
            corners_3d.push_back(p3d);
        }
    }
    
    if (corners_3d.size() >= 2) {
        float min_x = corners_3d[0].x, max_x = corners_3d[0].x;
        float min_y = corners_3d[0].y, max_y = corners_3d[0].y;
        float min_z = corners_3d[0].z, max_z = corners_3d[0].z;
        
        for (const auto& p : corners_3d) {
            min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
            min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
        }
        
        size_3d.width = max_x - min_x;
        size_3d.height = max_y - min_y;
        size_3d.depth = max_z - min_z;
    }
    
    return true;
}

bool LocalizationEngine::localizeObstacle(const ObstacleDetector::TrackedObstacle& obstacle,
                                           const cv::Mat& depth_map,
                                           Eigen::Vector3f& position_enu,
                                           Eigen::Vector3f& size,
                                           Eigen::Quaternionf& orientation) const {
    cv::Point3f center_3d, size_3d_cv;
    
    // Convert Eigen bbox to cv::Rect2d
    cv::Rect2d bbox(obstacle.bbox_2d[0], obstacle.bbox_2d[1],
                    obstacle.bbox_2d[2] - obstacle.bbox_2d[0],
                    obstacle.bbox_2d[3] - obstacle.bbox_2d[1]);
    
    if (!localizeFromBbox(bbox, depth_map, center_3d, size_3d_cv)) {
        // Fallback: use existing position if available
        if (obstacle.position_enu.norm() > 0) {
            position_enu = obstacle.position_enu;
            size = obstacle.size;
            orientation = obstacle.orientation;
            return true;
        }
        return false;
    }
    
    // Convert to ENU frame (camera to body to ENU)
    // Camera frame: X right, Y down, Z forward
    // ENU frame: X east, Y north, Z up
    // Assuming camera is aligned with body frame
    position_enu = Eigen::Vector3f(center_3d.z, -center_3d.x, -center_3d.y);
    size = Eigen::Vector3f(size_3d_cv.depth, size_3d_cv.width, size_3d_cv.height);
    orientation = Eigen::Quaternionf::Identity();  // Would need orientation estimation
    
    return true;
}

Eigen::Matrix3f LocalizationEngine::getCameraMatrix() const {
    Eigen::Matrix3f K;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K(i, j) = camera_matrix_.at<double>(i, j);
        }
    }
    return K;
}

Eigen::Matrix3f LocalizationEngine::getInvCameraMatrix() const {
    Eigen::Matrix3f K_inv;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            K_inv(i, j) = inv_camera_matrix_.at<double>(i, j);
        }
    }
    return K_inv;
}

bool LocalizationEngine::pixelToRay(const cv::Point2f& pt, Eigen::Vector3f& ray) const {
    cv::Mat pt_h = (cv::Mat_<double>(3, 1) << pt.x, pt.y, 1);
    cv::Mat ray_cam = inv_camera_matrix_ * pt_h;
    
    ray = Eigen::Vector3f(ray_cam.at<double>(0), ray_cam.at<double>(1), ray_cam.at<double>(2));
    ray.normalize();
    
    return true;
}

bool LocalizationEngine::rayIntersectPlane(const Eigen::Vector3f& ray_origin,
                                            const Eigen::Vector3f& ray_dir,
                                            const Eigen::Vector3f& plane_normal,
                                            float plane_dist,
                                            Eigen::Vector3f& intersection) const {
    float denom = ray_dir.dot(plane_normal);
    if (std::abs(denom) < 1e-6) return false;
    
    float t = (plane_dist - ray_origin.dot(plane_normal)) / denom;
    if (t < 0) return false;
    
    intersection = ray_origin + t * ray_dir;
    return true;
}

}  // namespace swarm::ai