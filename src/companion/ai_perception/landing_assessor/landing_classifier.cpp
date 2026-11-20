#include "landing_classifier.hpp"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace swarm::ai {

class LandingClassifier::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    struct SurfacePatch {
        cv::Rect roi;
        SurfaceType type;
        float confidence;
        float slope_deg;
        Eigen::Vector3f normal;
        float firmness;
        float roughness;
    };
    
    bool initialize(const LandingClassifierConfig& config) {
        config_ = config;
        
        // Initialize inference engine
        inference_engine_ = std::make_unique<InferenceEngine>();
        config.engine_config.landing_model_path = config.model_path;
        config.engine_config.confidence_threshold = config.confidence_threshold;
        config.engine_config.input_width = config.input_width;
        config.engine_config.input_height = config.input_height;
        
        if (!inference_engine_->initialize(config.engine_config)) {
            std::cerr << "Failed to initialize inference engine for landing classifier" << std::endl;
            return false;
        }
        
        // Set callbacks
        inference_engine_->setInferenceCallback([this](const AIInferenceResult& inf) {
            if (ai_inference_callback_) ai_inference_callback_(inf);
        });
        
        initialized_ = true;
        return true;
    }
    
    LandingAssessment assess(const cv::Mat& downward_image, 
                             uint16_t drone_id, 
                             uint64_t timestamp_us,
                             const cv::Mat& depth_map,
                             const Eigen::Vector3f& drone_position_enu,
                             const Eigen::Quaternionf& drone_orientation) {
        
        auto start_total = std::chrono::high_resolution_clock::now();
        LandingAssessment result;
        result.timestamp_us = timestamp_us;
        result.drone_id = drone_id;
        
        if (!initialized_ || downward_image.empty()) {
            result.total_ms = -1;
            return result;
        }
        
        // Preprocess
        auto start_pre = std::chrono::high_resolution_clock::now();
        cv::Mat processed = preprocessImage(downward_image);
        auto end_pre = std::chrono::high_resolution_clock::now();
        result.preprocess_ms = std::chrono::duration<float, std::milli>(end_pre - start_pre).count();
        
        // Run inference
        auto start_inf = std::chrono::high_resolution_clock::now();
        InferenceEngine::InferenceResult inf_result = inference_engine_->inferLanding(
            processed, drone_id, timestamp_us);
        auto end_inf = std::chrono::high_resolution_clock::now();
        result.inference_ms = std::chrono::duration<float, std::milli>(end_inf - start_inf).count();
        
        if (inf_result.total_ms < 0) {
            result.total_ms = -1;
            return result;
        }
        
        // Postprocess
        auto start_post = std::chrono::high_resolution_clock::now();
        
        // Extract surface classification
        if (!inf_result.detections.empty()) {
            const auto& det = inf_result.detections[0];
            result.surface_type = static_cast<SurfaceType>(det.class_id);
            result.surface_confidence = det.confidence;
        }
        
        // Analyze slope from depth map
        if (!depth_map.empty()) {
            analyzeSlope(depth_map, result, drone_orientation);
        }
        
        // Analyze clearance
        if (!depth_map.empty()) {
            analyzeClearance(depth_map, result, drone_position_enu);
        }
        
        // Estimate firmness from texture
        estimateFirmness(downward_image, depth_map, result);
        
        // Estimate roughness
        estimateRoughness(depth_map, result);
        
        // Compute overall suitability
        computeSuitability(result);
        
        auto end_post = std::chrono::high_resolution_clock::now();
        result.postprocess_ms = std::chrono::duration<float, std::milli>(end_post - start_post).count();
        
        auto end_total = std::chrono::high_resolution_clock::now();
        result.total_ms = std::chrono::duration<float, std::milli>(end_total - start_total).count();
        
        // Callback
        if (assessment_callback_) assessment_callback_(result);
        
        // Convert to AIInferenceResult for MAVLink/ROS publishing
        if (ai_inference_callback_) {
            AIInferenceResult ai_inf;
            ai_inf.timestamp_us = timestamp_us;
            ai_inf.drone_id = drone_id;
            ai_inf.type = InferenceType::LANDING_SITE;
            ai_inf.class_id = static_cast<int>(result.surface_type);
            ai_inf.class_name = LANDING_SURFACE_NAMES[static_cast<int>(result.surface_type)];
            ai_inf.confidence = result.suitability_score;
            ai_inf.position_enu = drone_position_enu;
            ai_inf.size = Eigen::Vector3f(result.clearance_radius_m * 2, 
                                           result.clearance_radius_m * 2, 0.1f);
            ai_inf.orientation = Eigen::Quaternionf::Identity();
            
            ai_inf.metadata.latency_ms = static_cast<int>(result.total_ms);
            ai_inf.metadata.model_name = config_.model_type;
            ai_inf.metadata.model_version = "1.0.0";
            ai_inf.metadata.runtime = inf_result.runtime_used;
            
            ai_inf.metadata.landing.slope_deg = result.slope_deg;
            ai_inf.metadata.landing.clearance_radius_m = result.clearance_radius_m;
            ai_inf.metadata.landing.surface_type = static_cast<uint8_t>(result.surface_type);
            ai_inf.metadata.landing.firmness_score = result.firmness_score;
            
            ai_inference_callback_(ai_inf);
        }
        
        return result;
    }
    
    LandingAssessment assessWithNormal(const cv::Mat& downward_image,
                                        uint16_t drone_id,
                                        uint64_t timestamp_us,
                                        const Eigen::Vector3f& surface_normal_enu,
                                        const cv::Mat& depth_map) {
        // Convert surface normal to slope
        LandingAssessment result = assess(downward_image, drone_id, timestamp_us, depth_map);
        
        if (result.total_ms > 0) {
            // Override slope from provided normal
            float slope_rad = std::acos(std::clamp(surface_normal_enu.z(), -1.0f, 1.0f));
            result.slope_deg = slope_rad * 180.0f / M_PI;
            result.normal_enu = surface_normal_enu;
            
            // Recompute suitability with better slope estimate
            computeSuitability(result);
        }
        
        return result;
    }
    
    void setAssessmentCallback(std::function<void(const LandingAssessment&)> cb) {
        assessment_callback_ = cb;
    }
    
    void setAIInferenceCallback(std::function<void(const AIInferenceResult&)> cb) {
        ai_inference_callback_ = cb;
        if (inference_engine_) {
            inference_engine_->setInferenceCallback(cb);
        }
    }

private:
    LandingClassifierConfig config_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    bool initialized_ = false;
    
    std::function<void(const LandingAssessment&)> assessment_callback_;
    std::function<void(const AIInferenceResult&)> ai_inference_callback_;
    
    cv::Mat preprocessImage(const cv::Mat& image) {
        cv::Mat resized;
        cv::resize(image, resized, cv::Size(config_.input_width, config_.input_height));
        
        // Normalize
        resized.convertTo(resized, CV_32F, 1.0f / 255.0f);
        
        // ImageNet normalization
        std::vector<cv::Mat> channels(3);
        cv::split(resized, channels);
        
        float mean[3] = {0.485f, 0.456f, 0.406f};
        float std[3] = {0.229f, 0.224f, 0.225f};
        
        for (int c = 0; c < 3; ++c) {
            channels[c] = (channels[c] - mean[c]) / std[c];
        }
        
        cv::merge(channels, resized);
        return resized;
    }
    
    void analyzeSlope(const cv::Mat& depth_map, LandingAssessment& result,
                      const Eigen::Quaternionf& drone_orientation) {
        // Fit plane to depth map
        // Convert depth map to 3D points
        std::vector<cv::Point3f> points;
        int step = std::max(1, depth_map.cols / 50);  // Sample every N pixels
        
        for (int y = 0; y < depth_map.rows; y += step) {
            for (int x = 0; x < depth_map.cols; x += step) {
                float depth = depth_map.at<uint16_t>(y, x) * 0.001f;  // mm to m
                if (depth > 0.1f && depth < 50.0f) {
                    // Simple pinhole camera model
                    float fx = 450.0f, fy = 450.0f;
                    float cx = depth_map.cols / 2.0f, cy = depth_map.rows / 2.0f;
                    
                    float X = (x - cx) * depth / fx;
                    float Y = (y - cy) * depth / fy;
                    float Z = depth;
                    
                    points.emplace_back(X, Y, Z);
                }
            }
        }
        
        if (points.size() < 100) return;
        
        // Fit plane using SVD
        cv::Mat pts_mat(points.size(), 3, CV_32F);
        for (size_t i = 0; i < points.size(); ++i) {
            pts_mat.at<float>(i, 0) = points[i].x;
            pts_mat.at<float>(i, 1) = points[i].y;
            pts_mat.at<float>(i, 2) = points[i].z;
        }
        
        cv::Mat mean, eigenvectors, eigenvalues;
        cv::PCA pca(pts_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);
        
        // Normal is the eigenvector with smallest eigenvalue (last row)
        cv::Mat normal_cv = pca.eigenvectors.row(2);
        Eigen::Vector3f normal_cam(normal_cv.at<float>(0), normal_cv.at<float>(1), normal_cv.at<float>(2));
        normal_cam.normalize();
        
        // Ensure normal points upward (positive Z in camera frame = forward)
        if (normal_cam.z() < 0) normal_cam = -normal_cam;
        
        // Transform to ENU using drone orientation
        // Camera frame: X right, Y down, Z forward
        // ENU: X east, Y north, Z up
        Eigen::Vector3f normal_enu = drone_orientation * Eigen::Vector3f(normal_cam.z(), -normal_cam.x(), -normal_cam.y());
        normal_enu.normalize();
        
        result.normal_enu = normal_enu;
        result.slope_deg = std::acos(std::clamp(normal_enu.z(), -1.0f, 1.0f)) * 180.0f / M_PI;
        result.slope_confidence = std::max(0.0f, 1.0f - result.slope_deg / 45.0f);
    }
    
    void analyzeClearance(const cv::Mat& depth_map, LandingAssessment& result,
                          const Eigen::Vector3f& drone_position_enu) {
        // Find largest clear circular area in center region
        int center_x = depth_map.cols / 2;
        int center_y = depth_map.rows / 2;
        int max_radius_px = std::min(center_x, center_y) - 10;
        
        float best_radius_m = 0.0f;
        float best_confidence = 0.0f;
        
        // Search for clear area at different radii
        for (int r_px = 10; r_px < max_radius_px; r_px += 5) {
            float r_m = estimateRadiusMeters(r_px, depth_map, center_x, center_y);
            
            if (r_m > 0) {
                // Check if area is clear
                bool clear = checkCircularAreaClear(depth_map, center_x, center_y, r_px, 1.0f);  // 1m threshold
                
                if (clear) {
                    best_radius_m = r_m;
                    best_confidence = 1.0f;
                } else {
                    break;  // Stop at first obstruction
                }
            }
        }
        
        result.clearance_radius_m = best_radius_m;
        result.clearance_confidence = best_confidence;
        result.has_obstructions = (best_radius_m < config_.min_clearance_radius_m);
    }
    
    float estimateRadiusMeters(int radius_px, const cv::Mat& depth_map, int cx, int cy) {
        // Sample depth at radius
        std::vector<float> depths;
        for (int angle = 0; angle < 360; angle += 10) {
            float rad = angle * M_PI / 180.0f;
            int x = cx + radius_px * std::cos(rad);
            int y = cy + radius_px * std::sin(rad);
            
            if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
                float d = depth_map.at<uint16_t>(y, x) * 0.001f;
                if (d > 0.1f) depths.push_back(d);
            }
        }
        
        if (depths.empty()) return -1.0f;
        
        // Average depth
        float avg_depth = 0.0f;
        for (float d : depths) avg_depth += d;
        avg_depth /= depths.size();
        
        // Convert pixel radius to meters
        float fx = 450.0f;
        return radius_px * avg_depth / fx;
    }
    
    bool checkCircularAreaClear(const cv::Mat& depth_map, int cx, int cy, int radius_px, float height_threshold_m) {
        for (int y = cy - radius_px; y <= cy + radius_px; y += 2) {
            for (int x = cx - radius_px; x <= cx + radius_px; x += 2) {
                float dx = x - cx;
                float dy = y - cy;
                if (dx*dx + dy*dy <= radius_px*radius_px) {
                    if (x >= 0 && x < depth_map.cols && y >= 0 && y < depth_map.rows) {
                        float d = depth_map.at<uint16_t>(y, x) * 0.001f;
                        if (d > 0.1f && d < height_threshold_m) {
                            return false;  // Obstruction found
                        }
                    }
                }
            }
        }
        return true;
    }
    
    void estimateFirmness(const cv::Mat& image, const cv::Mat& depth_map, LandingAssessment& result) {
        // Estimate firmness from texture analysis
        // Soft surfaces (grass, sand) have different texture than hard (concrete)
        
        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image.clone();
        }
        
        // Compute texture features
        cv::Mat laplacian;
        cv::Laplacian(gray, laplacian, CV_32F);
        cv::Scalar mean, stddev;
        cv::meanStdDev(laplacian, mean, stddev);
        
        // High frequency content = rougher/harder surface
        float texture_score = std::min(1.0f, stddev[0] / 50.0f);
        
        // Also use surface type classification
        float type_firmness = 0.5f;
        switch (result.surface_type) {
            case SurfaceType::CONCRETE: type_firmness = 1.0f; break;
            case SurfaceType::DIRT: type_firmness = 0.6f; break;
            case SurfaceType::GRASS: type_firmness = 0.4f; break;
            case SurfaceType::SAND: type_firmness = 0.3f; break;
            case SurfaceType::WATER: type_firmness = 0.0f; break;
            default: type_firmness = 0.5f; break;
        }
        
        result.firmness_score = 0.7f * type_firmness + 0.3f * texture_score;
        result.firmness_confidence = result.surface_confidence;
    }
    
    void estimateRoughness(const cv::Mat& depth_map, LandingAssessment& result) {
        if (depth_map.empty()) {
            result.roughness_index = 0.0f;
            return;
        }
        
        // Compute surface roughness from depth variation
        cv::Mat depth_f;
        depth_map.convertTo(depth_f, CV_32F, 0.001f);
        
        // Compute local standard deviation
        cv::Mat mean, stddev;
        int kernel_size = 5;
        cv::blur(depth_f, mean, cv::Size(kernel_size, kernel_size));
        cv::Mat diff = depth_f - mean;
        cv::Mat sq_diff = diff.mul(diff);
        cv::blur(sq_diff, sq_diff, cv::Size(kernel_size, kernel_size));
        cv::sqrt(sq_diff, stddev);
        
        cv::Scalar avg_std = cv::mean(stddev);
        result.roughness_index = std::min(1.0f, avg_std[0] / 0.1f);  // Normalize to 10cm
    }
    
    void computeSuitability(LandingAssessment& result) {
        float score = 0.0f;
        float weights_sum = 0.0f;
        
        // Surface type suitability (30%)
        float surface_score = 0.0f;
        switch (result.surface_type) {
            case SurfaceType::CONCRETE: surface_score = 1.0f; break;
            case SurfaceType::GRASS: surface_score = 0.8f; break;
            case SurfaceType::DIRT: surface_score = 0.7f; break;
            case SurfaceType::SAND: surface_score = 0.5f; break;
            case SurfaceType::WATER: surface_score = 0.0f; break;
            default: surface_score = 0.3f; break;
        }
        score += 0.3f * surface_score * result.surface_confidence;
        weights_sum += 0.3f;
        
        // Slope suitability (25%)
        float slope_score = 0.0f;
        if (result.slope_deg <= 5.0f) slope_score = 1.0f;
        else if (result.slope_deg <= 10.0f) slope_score = 0.8f;
        else if (result.slope_deg <= 15.0f) slope_score = 0.5f;
        else if (result.slope_deg <= 20.0f) slope_score = 0.2f;
        else slope_score = 0.0f;
        score += 0.25f * slope_score;
        weights_sum += 0.25f;
        
        // Clearance suitability (25%)
        float clearance_score = std::min(1.0f, result.clearance_radius_m / config_.min_clearance_radius_m);
        score += 0.25f * clearance_score;
        weights_sum += 0.25f;
        
        // Firmness suitability (20%)
        float firmness_score = result.firmness_score;
        score += 0.2f * firmness_score;
        weights_sum += 0.2f;
        
        result.suitability_score = score / weights_sum;
        
        // Determine suitability level
        if (result.suitability_score >= 0.85f) {
            result.suitability = LandingSuitability::IDEAL;
        } else if (result.suitability_score >= 0.7f) {
            result.suitability = LandingSuitability::SUITABLE;
        } else if (result.suitability_score >= 0.5f) {
            result.suitability = LandingSuitability::MARGINAL;
        } else {
            result.suitability = LandingSuitability::UNSUITABLE;
        }
    }
};

LandingClassifier::LandingClassifier() : pimpl_(std::make_unique<Impl>()) {}
LandingClassifier::~LandingClassifier() = default;
LandingClassifier::LandingClassifier(LandingClassifier&&) noexcept = default;
LandingClassifier& LandingClassifier::operator=(LandingClassifier&&) noexcept = default;

bool LandingClassifier::initialize(const LandingClassifierConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

LandingClassifier::LandingAssessment LandingClassifier::assess(const cv::Mat& downward_image, 
                                                                uint16_t drone_id, 
                                                                uint64_t timestamp_us,
                                                                const cv::Mat& depth_map,
                                                                const Eigen::Vector3f& drone_position_enu,
                                                                const Eigen::Quaternionf& drone_orientation) {
    return pimpl_->assess(downward_image, drone_id, timestamp_us, depth_map, 
                          drone_position_enu, drone_orientation);
}

LandingClassifier::LandingAssessment LandingClassifier::assessWithNormal(const cv::Mat& downward_image,
                                                                          uint16_t drone_id,
                                                                          uint64_t timestamp_us,
                                                                          const Eigen::Vector3f& surface_normal_enu,
                                                                          const cv::Mat& depth_map) {
    return pimpl_->assessWithNormal(downward_image, drone_id, timestamp_us, surface_normal_enu, depth_map);
}

bool LandingClassifier::isInitialized() const {
    return initialized_;
}

InferenceEngine* LandingClassifier::getInferenceEngine() {
    return inference_engine_.get();
}

void LandingClassifier::setAssessmentCallback(AssessmentCallback cb) {
    assessment_callback_ = cb;
    pimpl_->setAssessmentCallback(cb);
}

void LandingClassifier::setAIInferenceCallback(AIInferenceCallback cb) {
    ai_inference_callback_ = cb;
    pimpl_->setAIInferenceCallback(cb);
}

}  // namespace swarm::ai