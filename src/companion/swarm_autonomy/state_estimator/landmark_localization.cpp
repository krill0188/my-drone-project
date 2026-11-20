#include "landmark_localization.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>

namespace swarm::autonomy {

using json = nlohmann::json;

class LandmarkLocalization::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    LandmarkLocalizationConfig config_;
    std::map<int, Landmark> landmarks_;
    std::unique_ptr<cv::Feature2D> detector_;
    std::unique_ptr<cv::DescriptorMatcher> matcher_;
    
    // Keyframe history for bundle adjustment
    struct Keyframe {
        uint64_t timestamp_us;
        cv::Mat image;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        Eigen::Vector3f position_enu;
        Eigen::Quaternionf orientation_enu;
        std::vector<VisualLandmarkObservation> observations;
    };
    std::deque<Keyframe> keyframes_;
    int frame_counter_ = 0;
    
    // Last estimate for VO fusion
    LandmarkPoseEstimate last_estimate_;
    bool has_last_estimate_ = false;
    
    bool initialize(const LandmarkLocalizationConfig& config) {
        config_ = config;
        
        // Initialize feature detector
        if (config_.feature.use_orb) {
            detector_ = cv::ORB::create(
                config_.feature.max_features,
                1.2f, 8, config_.feature.patch_size, 0, 2,
                cv::ORB::HARRIS_SCORE, config_.feature.patch_size, 20);
        } else {
            detector_ = cv::SIFT::create(config_.feature.max_features);
        }
        
        // Initialize matcher
        if (config_.feature.use_orb) {
            matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        } else {
            matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        }
        
        // Camera matrix
        if (config_.camera.camera_matrix.rows == 0) {
            config_.camera.camera_matrix = cv::Mat::eye(3, 3, CV_32F);
            config_.camera.camera_matrix.at<float>(0, 0) = config_.camera.focal_length_px;
            config_.camera.camera_matrix.at<float>(1, 1) = config_.camera.focal_length_px;
            config_.camera.camera_matrix.at<float>(0, 2) = config_.camera.image_width / 2.0f;
            config_.camera.camera_matrix.at<float>(1, 2) = config_.camera.image_height / 2.0f;
        }
        
        if (config_.camera.dist_coeffs.size() == 0) {
            config_.camera.dist_coeffs = cv::Mat::zeros(5, 1, CV_32F);
        }
        
        return true;
    }
    
    bool loadLandmarkDatabase(const std::string& path) {
        std::ifstream file(path);
        if (!file.is_open()) return false;
        
        json j;
        file >> j;
        
        for (const auto& lm_json : j["landmarks"]) {
            Landmark lm;
            lm.id = lm_json["id"];
            lm.name = lm_json["name"];
            lm.type = lm_json["type"];
            
            auto pos = lm_json["position_enu"];
            lm.position_enu = Eigen::Vector3f(pos[0], pos[1], pos[2]);
            
            auto sz = lm_json["size"];
            lm.size = Eigen::Vector3f(sz[0], sz[1], sz[2]);
            
            lm.visibility_score = lm_json.value("visibility_score", 1.0f);
            
            if (lm_json.contains("connected_landmarks")) {
                for (int id : lm_json["connected_landmarks"]) {
                    lm.connected_landmarks.push_back(id);
                }
            }
            
            // Note: keypoints and descriptors would be loaded from binary files
            // For now, we'll extract them on first observation
            
            landmarks_[lm.id] = std::move(lm);
        }
        
        return true;
    }
    
    void addLandmark(const Landmark& landmark) {
        landmarks_[landmark.id] = landmark;
    }
    
    LandmarkPoseEstimate processFrame(const cv::Mat& image, uint64_t timestamp_us,
                                      const Eigen::Vector3f& vo_position_enu,
                                      const Eigen::Quaternionf& vo_orientation_enu,
                                      const Eigen::Vector3f& vo_velocity_enu) {
        auto start = std::chrono::high_resolution_clock::now();
        LandmarkPoseEstimate estimate;
        estimate.timestamp_us = timestamp_us;
        estimate.drone_id = 1;  // Would be parameter
        
        frame_counter_++;
        
        // Detect features
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        detector_->detectAndCompute(image, cv::noArray(), keypoints, descriptors);
        
        // Match against landmarks
        std::vector<VisualLandmarkObservation> observations;
        for (const auto& [id, landmark] : landmarks_) {
            if (landmark.descriptors.empty()) continue;
            
            VisualLandmarkObservation obs = matchLandmark(landmark, keypoints, descriptors);
            if (obs.num_inliers >= config_.landmark_db.min_matches_for_pose) {
                observations.push_back(obs);
            }
        }
        
        // Solve for pose if enough matches
        if ((int)observations.size() >= config_.landmark_db.min_matches_for_pose) {
            estimate = solvePose(observations, image.size(), keypoints);
            
            // Fuse with VO if available
            if (config_.vo.enable_vo_fusion && 
                vo_position_enu.norm() > 0 && 
                has_last_estimate_) {
                estimate = fuseWithVO(estimate, vo_position_enu, vo_orientation_enu, vo_velocity_enu);
            }
        } else {
            // Fallback to VO
            if (vo_position_enu.norm() > 0) {
                estimate.valid = true;
                estimate.position_enu = vo_position_enu;
                estimate.orientation_enu = vo_orientation_enu;
                estimate.velocity_enu = vo_velocity_enu;
                estimate.confidence = 0.3f;  // Lower confidence
            }
        }
        
        // Add keyframe if needed
        if (frame_counter_ % config_.vo.keyframe_interval == 0 && estimate.valid) {
            addKeyframe(image, keypoints, descriptors, estimate);
        }
        
        // Update last estimate
        if (estimate.valid) {
            last_estimate_ = estimate;
            has_last_estimate_ = true;
        }
        
        // Update stats
        auto end = std::chrono::high_resolution_clock::now();
        float comp_ms = std::chrono::duration<float, std::milli>(end - start).count();
        
        {
            std::lock_guard<std::mutex> lock(stats_mutex_);
            stats_.total_frames++;
            if (estimate.valid) stats_.successful_estimates++;
            else stats_.failed_estimates++;
            stats_.avg_computation_ms = 0.9f * stats_.avg_computation_ms + 0.1f * comp_ms;
        }
        
        // Callback
        if (estimate_callback_) {
            estimate_callback_(estimate);
        }
        
        return estimate;
    }
    
    std::optional<LandmarkPoseEstimate> getLatestEstimate() const {
        if (has_last_estimate_) return last_estimate_;
        return std::nullopt;
    }
    
    void reset() {
        keyframes_.clear();
        has_last_estimate_ = false;
        frame_counter_ = 0;
    }
    
    void setEstimateCallback(std::function<void(const LandmarkPoseEstimate&)> cb) {
        estimate_callback_ = cb;
    }

private:
    std::function<void(const LandmarkPoseEstimate&)> estimate_callback_;
    LandmarkLocalizationConfig::Stats stats_;
    std::mutex stats_mutex_;
    
    VisualLandmarkObservation matchLandmark(const Landmark& landmark, 
                                             const std::vector<cv::KeyPoint>& keypoints,
                                             const cv::Mat& descriptors) {
        VisualLandmarkObservation obs;
        obs.landmark_id = landmark.id;
        
        if (descriptors.empty() || landmark.descriptors.empty()) return obs;
        
        // KNN match
        std::vector<std::vector<cv::DMatch>> knn_matches;
        matcher_->knnMatch(descriptors, landmark.descriptors, knn_matches, 2);
        
        // Ratio test
        std::vector<cv::DMatch> good_matches;
        for (const auto& knn : knn_matches) {
            if (knn.size() == 2) {
                if (knn[0].distance < config_.matching.ratio_threshold * knn[1].distance) {
                    good_matches.push_back(knn[0]);
                }
            }
        }
        
        if (good_matches.size() < 4) return obs;
        
        // Extract matched keypoints
        std::vector<cv::Point2f> image_pts, landmark_pts;
        for (const auto& m : good_matches) {
            image_pts.push_back(keypoints[m.queryIdx].pt);
            landmark_pts.push_back(landmark.keypoints[m.trainIdx].pt);
        }
        
        // Geometric verification (RANSAC)
        if (config_.matching.use_ransac && image_pts.size() >= 8) {
            cv::Mat fundamental;
            std::vector<uchar> inlier_mask;
            fundamental = cv::findFundamentalMat(image_pts, landmark_pts, 
                                                 cv::FM_RANSAC, 
                                                 config_.matching.ransac_threshold,
                                                 0.99, inlier_mask);
            
            int inliers = 0;
            for (size_t i = 0; i < inlier_mask.size(); ++i) {
                if (inlier_mask[i]) inliers++;
            }
            
            if (inliers >= config_.landmark_db.min_matches_for_pose) {
                obs.num_inliers = inliers;
                obs.matches = good_matches;
                obs.image_keypoints = keypoints;
                obs.descriptors = descriptors;
                
                // Estimate bearing and distance (simplified)
                // Would use PnP with known 3D landmark points
                // For now, compute average bearing
                Eigen::Vector3f bearing_sum = Eigen::Vector3f::Zero();
                for (const auto& m : good_matches) {
                    if (inlier_mask[m.queryIdx]) {
                        cv::Point2f pt = keypoints[m.queryIdx].pt;
                        // Normalize to camera frame
                        Eigen::Vector3f ray((pt.x - config_.camera.camera_matrix.at<float>(0,2)) / config_.camera.camera_matrix.at<float>(0,0),
                                           (pt.y - config_.camera.camera_matrix.at<float>(1,2)) / config_.camera.camera_matrix.at<float>(1,1),
                                           1.0f);
                        ray.normalize();
                        bearing_sum += ray;
                    }
                }
                if (obs.num_inliers > 0) {
                    obs.bearing_enu = bearing_sum / obs.num_inliers;
                    obs.distance_estimate_m = landmark.size.norm();  // Rough estimate
                    obs.confidence = std::min(1.0f, obs.num_inliers / 20.0f);
                }
            }
        }
        
        return obs;
    }
    
    LandmarkPoseEstimate solvePose(const std::vector<VisualLandmarkObservation>& observations,
                                   cv::Size image_size,
                                   const std::vector<cv::KeyPoint>& keypoints) {
        LandmarkPoseEstimate estimate;
        
        // Collect 3D-2D correspondences for PnP
        std::vector<cv::Point3f> object_points;
        std::vector<cv::Point2f> image_points;
        
        for (const auto& obs : observations) {
            if (obs.num_inliers < 4) continue;
            
            const Landmark& landmark = landmarks_.at(obs.landmark_id);
            
            // Use landmark position as 3D point (simplified - would use corners)
            // For now, use center point
            cv::Point3f obj_pt(landmark.position_enu.x(), landmark.position_enu.y(), landmark.position_enu.z());
            
            // Find corresponding image point (average of matched keypoints)
            cv::Point2f img_pt(0, 0);
            int count = 0;
            for (const auto& m : obs.matches) {
                img_pt += keypoints[m.queryIdx].pt;
                count++;
            }
            if (count > 0) img_pt *= (1.0f / count);
            
            object_points.push_back(obj_pt);
            image_points.push_back(img_pt);
        }
        
        if (object_points.size() < 4) return estimate;
        
        // Solve PnP
        cv::Mat rvec, tvec;
        bool success = cv::solvePnP(object_points, image_points,
                                    config_.camera.camera_matrix, config_.camera.dist_coeffs,
                                    rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
        
        if (!success) return estimate;
        
        // Refine with solvePnPRefineLM
        cv::solvePnPRefineLM(object_points, image_points,
                             config_.camera.camera_matrix, config_.camera.dist_coeffs,
                             rvec, tvec);
        
        // Convert to Eigen
        cv::Mat R;
        cv::Rodrigues(rvec, R);
        
        estimate.valid = true;
        estimate.position_enu = Eigen::Vector3f(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
        
        Eigen::Quaternionf q;
        Eigen::Matrix3f R_eig;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R_eig(i, j) = R.at<double>(i, j);
            }
        }
        q = Eigen::Quaternionf(R_eig);
        estimate.orientation_enu = q.normalized();
        
        // Estimate velocity from last keyframe (simplified)
        estimate.velocity_enu = Eigen::Vector3f::Zero();
        
        // Compute reprojection error
        float total_error = 0;
        int error_count = 0;
        for (size_t i = 0; i < object_points.size(); ++i) {
            cv::Point2f proj;
            cv::projectPoints(object_points[i], rvec, tvec,
                             config_.camera.camera_matrix, config_.camera.dist_coeffs, proj);
            float err = cv::norm(proj - image_points[i]);
            total_error += err;
            error_count++;
        }
        estimate.avg_reprojection_error_px = error_count > 0 ? total_error / error_count : 0;
        
        // Confidence based on inliers and reprojection error
        int total_inliers = 0;
        for (const auto& obs : observations) total_inliers += obs.num_inliers;
        estimate.num_landmarks_matched = observations.size();
        estimate.num_landmarks_inliers = total_inliers;
        estimate.confidence = std::min(1.0f, total_inliers / 30.0f) * 
                             std::max(0.0f, 1.0f - estimate.avg_reprojection_error_px / 10.0f);
        
        // GDOP (simplified)
        estimate.gdop = std::sqrt(object_points.size() / 4.0f);
        
        return estimate;
    }
    
    LandmarkPoseEstimate fuseWithVO(const LandmarkPoseEstimate& landmark_est,
                                     const Eigen::Vector3f& vo_pos,
                                     const Eigen::Quaternionf& vo_orient,
                                     const Eigen::Vector3f& vo_vel) {
        LandmarkPoseEstimate fused = landmark_est;
        
        // Weighted fusion
        float w_landmark = config_.vo.landmark_weight * landmark_est.confidence;
        float w_vo = config_.vo.vo_weight * 0.8f;  // VO base confidence
        float w_sum = w_landmark + w_vo;
        
        if (w_sum > 0) {
            fused.position_enu = (w_landmark * landmark_est.position_enu + w_vo * vo_pos) / w_sum;
            fused.orientation_enu = landmark_est.orientation_enu.slerp(w_vo / w_sum, vo_orient);
            fused.velocity_enu = (w_landmark * landmark_est.velocity_enu + w_vo * vo_vel) / w_sum;
            fused.confidence = std::min(1.0f, (landmark_est.confidence + 0.8f) / 2.0f);
            fused.vo_fused = true;
            fused.vo_weight_used = w_vo / w_sum;
        }
        
        return fused;
    }
    
    void addKeyframe(const cv::Mat& image,
                     const std::vector<cv::KeyPoint>& keypoints,
                     const cv::Mat& descriptors,
                     const LandmarkPoseEstimate& estimate) {
        Keyframe kf;
        kf.timestamp_us = estimate.timestamp_us;
        kf.image = image.clone();
        kf.keypoints = keypoints;
        kf.descriptors = descriptors.clone();
        kf.position_enu = estimate.position_enu;
        kf.orientation_enu = estimate.orientation_enu;
        kf.observations.clear();  // Would store observations
        
        keyframes_.push_back(kf);
        
        // Limit keyframe window
        if (keyframes_.size() > config_.optimization.ba_window_size) {
            keyframes_.pop_front();
        }
        
        // Run bundle adjustment if enabled
        if (config_.optimization.enable_bundle_adjustment && keyframes_.size() >= 5) {
            runBundleAdjustment();
        }
    }
    
    void runBundleAdjustment() {
        // Simplified bundle adjustment
        // In practice, would use Ceres or g2o for full BA
        // For now, just log
    }
    
    const std::map<int, Landmark>& getLandmarks() const {
        return landmarks_;
    }
    
    const LandmarkLocalizationConfig::Stats& getStats() const {
        return stats_;
    }
};

LandmarkLocalization::LandmarkLocalization() : pimpl_(std::make_unique<Impl>()) {}
LandmarkLocalization::~LandmarkLocalization() = default;
LandmarkLocalization::LandmarkLocalization(LandmarkLocalization&&) noexcept = default;
LandmarkLocalization& LandmarkLocalization::operator=(LandmarkLocalization&&) noexcept = default;

bool LandmarkLocalization::initialize(const LandmarkLocalizationConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

bool LandmarkLocalization::loadLandmarkDatabase(const std::string& path) {
    return pimpl_->loadLandmarkDatabase(path);
}

void LandmarkLocalization::addLandmark(const Landmark& landmark) {
    pimpl_->addLandmark(landmark);
}

LandmarkLocalization::LandmarkPoseEstimate LandmarkLocalization::processFrame(const cv::Mat& image, uint64_t timestamp_us,
                                                                               const Eigen::Vector3f& vo_position_enu,
                                                                               const Eigen::Quaternionf& vo_orientation_enu,
                                                                               const Eigen::Vector3f& vo_velocity_enu) {
    return pimpl_->processFrame(image, timestamp_us, vo_position_enu, vo_orientation_enu, vo_velocity_enu);
}

std::optional<LandmarkLocalization::LandmarkPoseEstimate> LandmarkLocalization::getLatestEstimate() const {
    return pimpl_->getLatestEstimate();
}

bool LandmarkLocalization::isInitialized() const {
    return initialized_;
}

const LandmarkLocalizationConfig& LandmarkLocalization::getConfig() const {
    return config_;
}

const std::map<int, Landmark>& LandmarkLocalization::getLandmarks() const {
    return pimpl_->getLandmarks();
}

void LandmarkLocalization::reset() {
    pimpl_->reset();
}

void LandmarkLocalization::setEstimateCallback(EstimateCallback cb) {
    pimpl_->setEstimateCallback(cb);
}

const LandmarkLocalization::Stats& LandmarkLocalization::getStats() const {
    return stats_;
}

}  // namespace swarm::autonomy