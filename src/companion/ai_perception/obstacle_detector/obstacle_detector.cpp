#include "obstacle_detector.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <random>

namespace swarm::ai {

class ObstacleDetector::Impl {
public:
    Impl() : next_track_id_(1), frame_id_(0) {}
    ~Impl() = default;
    
    struct TrackedObstacleInternal {
        TrackedObstacle obstacle;
        Eigen::Vector4f last_bbox_2d;
        Eigen::Vector3f last_position_enu;
        int missed_frames = 0;
    };
    
    std::vector<TrackedObstacleInternal> tracks_;
    int next_track_id_;
    int frame_id_;
    Eigen::Vector3f ego_velocity_ = Eigen::Vector3f::Zero();
    
    // Stereo calibration
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    cv::Mat R_;  // Rotation from left to right
    cv::Mat T_;  // Translation from left to right
    bool stereo_calibrated_ = false;
    
    // Depth scale
    float depth_scale_ = 0.001f;  // mm to meters
    
    bool initialize(const ObstacleDetectorConfig& config) {
        config_ = config;
        
        // Initialize inference engine
        inference_engine_ = std::make_unique<InferenceEngine>();
        config.engine_config.obstacle_model_path = config.model_path;
        config.engine_config.confidence_threshold = config.confidence_threshold;
        config.engine_config.nms_threshold = config.nms_threshold;
        config.engine_config.input_width = config.input_width;
        config.engine_config.input_height = config.input_height;
        config.engine_config.letterbox = config.letterbox;
        
        if (!inference_engine_->initialize(config.engine_config)) {
            std::cerr << "Failed to initialize inference engine" << std::endl;
            return false;
        }
        
        // Set callbacks
        inference_engine_->setInferenceCallback([this](const AIInferenceResult& inf) {
            if (ai_inference_callback_) ai_inference_callback_(inf);
        });
        
        initialized_ = true;
        return true;
    }
    
    DetectionResult detect(const cv::Mat& image, uint16_t drone_id, uint64_t timestamp_us) {
        if (!initialized_) return emptyResult(drone_id, timestamp_us);
        
        auto start_total = std::chrono::high_resolution_clock::now();
        frame_id_++;
        
        // Run inference
        InferenceEngine::InferenceResult result = inference_engine_->inferObstacle(image, drone_id, timestamp_us);
        
        if (result.total_ms < 0) {
            return emptyResult(drone_id, timestamp_us);
        }
        
        // Convert to tracked obstacles
        std::vector<TrackedObstacle> tracked = updateTracking(result.detections, timestamp_us, image.size());
        
        // Assess threats
        for (auto& track : tracked) {
            assessThreat(track);
        }
        
        // Build result
        DetectionResult det_result;
        det_result.timestamp_us = timestamp_us;
        det_result.drone_id = drone_id;
        det_result.obstacles = tracked;
        det_result.preprocess_ms = result.preprocess_ms;
        det_result.inference_ms = result.inference_ms;
        det_result.postprocess_ms = result.postprocess_ms;
        det_result.total_ms = result.total_ms;
        det_result.frame_id = frame_id_;
        
        // Callback
        if (detection_callback_) detection_callback_(det_result);
        
        return det_result;
    }
    
    DetectionResult detectStereo(const cv::Mat& left_image, const cv::Mat& right_image,
                                  uint16_t drone_id, uint64_t timestamp_us) {
        // Run detection on left image
        DetectionResult result = detect(left_image, drone_id, timestamp_us);
        
        // Refine 3D positions using stereo
        if (stereo_calibrated_ && result.obstacles.size() > 0) {
            refinePositionsStereo(result.obstacles, left_image, right_image);
        }
        
        return result;
    }
    
    DetectionResult detectWithDepth(const cv::Mat& image, const cv::Mat& depth_map,
                                     uint16_t drone_id, uint64_t timestamp_us) {
        // Run detection on image
        DetectionResult result = detect(image, drone_id, timestamp_us);
        
        // Refine 3D positions using depth map
        if (!depth_map.empty() && result.obstacles.size() > 0) {
            refinePositionsDepth(result.obstacles, depth_map);
        }
        
        return result;
    }
    
    std::vector<TrackedObstacle> getTracks() const {
        std::vector<TrackedObstacle> result;
        for (const auto& t : tracks_) {
            if (t.obstacle.confirmed) {
                result.push_back(t.obstacle);
            }
        }
        return result;
    }
    
    void clearTracks() {
        tracks_.clear();
        next_track_id_ = 1;
    }
    
    void setStereoCalibration(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                              const cv::Mat& R, const cv::Mat& T) {
        camera_matrix_ = camera_matrix.clone();
        dist_coeffs_ = dist_coeffs.clone();
        R_ = R.clone();
        T_ = T.clone();
        stereo_calibrated_ = true;
    }
    
    void setDepthScale(float scale) {
        depth_scale_ = scale;
    }
    
    void setEgoVelocity(const Eigen::Vector3f& vel) {
        ego_velocity_ = vel;
    }
    
    void setDetectionCallback(std::function<void(const DetectionResult&)> cb) {
        detection_callback_ = cb;
    }
    
    void setAIInferenceCallback(std::function<void(const AIInferenceResult&)> cb) {
        ai_inference_callback_ = cb;
        if (inference_engine_) {
            inference_engine_->setInferenceCallback(cb);
        }
    }

private:
    ObstacleDetectorConfig config_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    bool initialized_ = false;
    
    std::function<void(const DetectionResult&)> detection_callback_;
    std::function<void(const AIInferenceResult&)> ai_inference_callback_;
    
    DetectionResult emptyResult(uint16_t drone_id, uint64_t timestamp_us) {
        DetectionResult r;
        r.timestamp_us = timestamp_us;
        r.drone_id = drone_id;
        r.obstacles.clear();
        r.total_ms = -1;
        r.frame_id = frame_id_;
        return r;
    }
    
    std::vector<TrackedObstacle> updateTracking(const std::vector<InferenceEngine::Detection>& detections,
                                                 uint64_t timestamp_us, cv::Size image_size) {
        std::vector<TrackedObstacle> confirmed_tracks;
        
        // Match detections to existing tracks
        std::vector<bool> detection_matched(detections.size(), false);
        std::vector<bool> track_matched(tracks_.size(), false);
        
        for (size_t t = 0; t < tracks_.size(); ++t) {
            if (tracks_[t].missed_frames > config_.max_track_age) continue;
            
            float best_iou = config_.track_iou_threshold;
            int best_det = -1;
            
            for (size_t d = 0; d < detections.size(); ++d) {
                if (detection_matched[d]) continue;
                if (detections[d].class_id != tracks_[t].obstacle.class_id) continue;
                
                float iou = computeIoU(tracks_[t].last_bbox_2d, detections[d].bbox_xyxy);
                if (iou > best_iou) {
                    best_iou = iou;
                    best_det = d;
                }
            }
            
            if (best_det >= 0) {
                // Update track
                detection_matched[best_det] = true;
                track_matched[t] = true;
                
                auto& track = tracks_[t].obstacle;
                const auto& det = detections[best_det];
                
                track.confidence = det.confidence;
                track.bbox_2d = det.bbox_xyxy;
                track.last_update_us = timestamp_us;
                track.age++;
                track.hits++;
                track.misses = 0;
                
                tracks_[t].last_bbox_2d = det.bbox_xyxy;
                
                // Update 3D position if available
                if (det.position_enu.norm() > 0) {
                    // Simple velocity estimation
                    if (tracks_[t].last_position_enu.norm() > 0) {
                        float dt = (timestamp_us - track.last_update_us) / 1e6f;
                        if (dt > 0) {
                            track.velocity_enu = (det.position_enu - tracks_[t].last_position_enu) / dt;
                        }
                    }
                    track.position_enu = det.position_enu;
                    tracks_[t].last_position_enu = det.position_enu;
                }
                
                if (det.size.norm() > 0) {
                    track.size = det.size;
                }
                if (det.orientation.norm() > 0) {
                    track.orientation = det.orientation;
                }
                
                if (track.hits >= config_.min_hits) {
                    track.confirmed = true;
                }
            } else {
                // Track missed
                tracks_[t].missed_frames++;
            }
        }
        
        // Create new tracks for unmatched detections
        for (size_t d = 0; d < detections.size(); ++d) {
            if (detection_matched[d]) continue;
            if (detections[d].confidence < config_.confidence_threshold) continue;
            
            TrackedObstacleInternal new_track;
            new_track.obstacle.track_id = next_track_id_++;
            new_track.obstacle.class_id = detections[d].class_id;
            new_track.obstacle.class_name = detections[d].class_name;
            new_track.obstacle.confidence = detections[d].confidence;
            new_track.obstacle.bbox_2d = detections[d].bbox_xyxy;
            new_track.obstacle.position_enu = detections[d].position_enu;
            new_track.obstacle.size = detections[d].size;
            new_track.obstacle.orientation = detections[d].orientation;
            new_track.obstacle.velocity_enu = Eigen::Vector3f::Zero();
            new_track.obstacle.age = 1;
            new_track.obstacle.hits = 1;
            new_track.obstacle.misses = 0;
            new_track.obstacle.confirmed = false;
            new_track.obstacle.last_update_us = timestamp_us;
            
            new_track.last_bbox_2d = detections[d].bbox_xyxy;
            new_track.last_position_enu = detections[d].position_enu;
            
            tracks_.push_back(new_track);
        }
        
        // Remove dead tracks and collect confirmed
        std::vector<TrackedObstacleInternal> alive_tracks;
        for (auto& t : tracks_) {
            if (t.missed_frames <= config_.max_track_age) {
                alive_tracks.push_back(t);
                if (t.obstacle.confirmed) {
                    confirmed_tracks.push_back(t.obstacle);
                }
            }
        }
        tracks_ = std::move(alive_tracks);
        
        return confirmed_tracks;
    }
    
    void refinePositionsStereo(std::vector<TrackedObstacle>& obstacles,
                               const cv::Mat& left_image, const cv::Mat& right_image) {
        if (!stereo_calibrated_) return;
        
        for (auto& obs : obstacles) {
            // Get center of bbox
            float cx = (obs.bbox_2d[0] + obs.bbox_2d[2]) / 2;
            float cy = (obs.bbox_2d[1] + obs.bbox_2d[3]) / 2;
            
            // Search for matching point in right image (simplified - would use stereo matching)
            // For now, use triangulation with assumed disparity
            float disparity = config_.baseline_m * config_.focal_length_px / 
                             std::max(1.0f, obs.position_enu.z());
            
            if (disparity > 1.0f && disparity < left_image.cols) {
                // Triangulate
                float x = (cx - camera_matrix_.at<double>(0, 2)) * obs.position_enu.z() / camera_matrix_.at<double>(0, 0);
                float y = (cy - camera_matrix_.at<double>(1, 2)) * obs.position_enu.z() / camera_matrix_.at<double>(1, 1);
                
                obs.position_enu = Eigen::Vector3f(x, y, obs.position_enu.z());
            }
        }
    }
    
    void refinePositionsDepth(std::vector<TrackedObstacle>& obstacles, const cv::Mat& depth_map) {
        for (auto& obs : obstacles) {
            // Get center of bbox
            int cx = int((obs.bbox_2d[0] + obs.bbox_2d[2]) / 2);
            int cy = int((obs.bbox_2d[1] + obs.bbox_2d[3]) / 2);
            
            // Check bounds
            if (cx >= 0 && cx < depth_map.cols && cy >= 0 && cy < depth_map.rows) {
                float depth = depth_map.at<uint16_t>(cy, cx) * depth_scale_;
                
                if (depth > 0.1f && depth < config_.engine_config.max_workspace_size_mb) {
                    // Convert to 3D using camera intrinsics
                    float fx = camera_matrix_.empty() ? config_.focal_length_px : camera_matrix_.at<double>(0, 0);
                    float fy = camera_matrix_.empty() ? config_.focal_length_px : camera_matrix_.at<double>(1, 1);
                    float cx_cam = camera_matrix_.empty() ? depth_map.cols / 2.0f : camera_matrix_.at<double>(0, 2);
                    float cy_cam = camera_matrix_.empty() ? depth_map.rows / 2.0f : camera_matrix_.at<double>(1, 2);
                    
                    float x = (cx - cx_cam) * depth / fx;
                    float y = (cy - cy_cam) * depth / fy;
                    
                    obs.position_enu = Eigen::Vector3f(x, y, depth);
                }
            }
        }
    }
    
    void assessThreat(TrackedObstacle& obs) {
        // Get priority
        int priority = 1;
        auto it = OBSTACLE_PRIORITY.find(obs.class_id);
        if (it != OBSTACLE_PRIORITY.end()) {
            priority = it->second;
        }
        
        // Calculate time to collision
        float rel_speed = (obs.velocity_enu - ego_velocity_).norm();
        float distance = obs.position_enu.norm();
        
        if (rel_speed > 0.1f && distance > 0) {
            obs.metadata.obstacle.time_to_collision_s = distance / rel_speed;
        } else {
            obs.metadata.obstacle.time_to_collision_s = -1.0f;
        }
        
        obs.metadata.obstacle.relative_velocity = obs.velocity_enu - ego_velocity_;
        
        // Determine threat level
        ThreatLevel threat = ThreatLevel::LOW;
        
        if (priority >= 8) {  // Power lines, aircraft, drones
            if (distance < 50.0f || obs.metadata.obstacle.time_to_collision_s > 0 && 
                obs.metadata.obstacle.time_to_collision_s < 5.0f) {
                threat = ThreatLevel::CRITICAL;
            } else if (distance < 100.0f) {
                threat = ThreatLevel::HIGH;
            } else {
                threat = ThreatLevel::MEDIUM;
            }
        } else if (priority >= 5) {  // Towers, turbines, birds
            if (distance < 30.0f) threat = ThreatLevel::CRITICAL;
            else if (distance < 60.0f) threat = ThreatLevel::HIGH;
            else threat = ThreatLevel::MEDIUM;
        } else {  // Buildings, trees, etc.
            if (distance < 20.0f) threat = ThreatLevel::HIGH;
            else if (distance < 50.0f) threat = ThreatLevel::MEDIUM;
            else threat = ThreatLevel::LOW;
        }
        
        obs.metadata.obstacle.threat_level = threat;
        obs.metadata.obstacle.requires_immediate_avoidance = (threat >= ThreatLevel::HIGH);
    }
    
    float computeIoU(const Eigen::Vector4f& box1, const Eigen::Vector4f& box2) {
        float x1 = std::max(box1[0], box2[0]);
        float y1 = std::max(box1[1], box2[1]);
        float x2 = std::min(box1[2], box2[2]);
        float y2 = std::min(box1[3], box2[3]);
        
        float inter = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);
        float area1 = (box1[2] - box1[0]) * (box1[3] - box1[1]);
        float area2 = (box2[2] - box2[0]) * (box2[3] - box2[1]);
        float union_area = area1 + area2 - inter;
        
        return union_area > 0 ? inter / union_area : 0.0f;
    }
};

ObstacleDetector::ObstacleDetector() : pimpl_(std::make_unique<Impl>()) {}
ObstacleDetector::~ObstacleDetector() = default;
ObstacleDetector::ObstacleDetector(ObstacleDetector&&) noexcept = default;
ObstacleDetector& ObstacleDetector::operator=(ObstacleDetector&&) noexcept = default;

bool ObstacleDetector::initialize(const ObstacleDetectorConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

ObstacleDetector::DetectionResult ObstacleDetector::detect(const cv::Mat& image, 
                                                            uint16_t drone_id, 
                                                            uint64_t timestamp_us) {
    return pimpl_->detect(image, drone_id, timestamp_us);
}

ObstacleDetector::DetectionResult ObstacleDetector::detectStereo(const cv::Mat& left_image, 
                                                                  const cv::Mat& right_image,
                                                                  uint16_t drone_id, 
                                                                  uint64_t timestamp_us) {
    return pimpl_->detectStereo(left_image, right_image, drone_id, timestamp_us);
}

ObstacleDetector::DetectionResult ObstacleDetector::detectWithDepth(const cv::Mat& image, 
                                                                     const cv::Mat& depth_map,
                                                                     uint16_t drone_id, 
                                                                     uint64_t timestamp_us) {
    return pimpl_->detectWithDepth(image, depth_map, drone_id, timestamp_us);
}

std::vector<ObstacleDetector::TrackedObstacle> ObstacleDetector::getTracks() const {
    return pimpl_->getTracks();
}

void ObstacleDetector::clearTracks() {
    pimpl_->clearTracks();
}

bool ObstacleDetector::isInitialized() const {
    return initialized_;
}

InferenceEngine* ObstacleDetector::getInferenceEngine() {
    return inference_engine_.get();
}

void ObstacleDetector::setDetectionCallback(DetectionCallback cb) {
    detection_callback_ = cb;
    pimpl_->setDetectionCallback(cb);
}

void ObstacleDetector::setAIInferenceCallback(AIInferenceCallback cb) {
    ai_inference_callback_ = cb;
    pimpl_->setAIInferenceCallback(cb);
}

}  // namespace swarm::ai