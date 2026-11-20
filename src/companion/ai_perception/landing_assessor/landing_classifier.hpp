#ifndef LANDING_CLASSIFIER_HPP
#define LANDING_CLASSIFIER_HPP

#include "../common/ai_inference.hpp"
#include "../model_runtime/inference_engine.hpp"
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <memory>
#include <functional>

namespace swarm::ai {

// Landing surface types (matching SurfaceType enum in ai_inference.hpp)
static const std::vector<std::string> LANDING_SURFACE_NAMES = {
    "grass",       // 0
    "concrete",    // 1
    "dirt",        // 2
    "sand",        // 3
    "water",       // 4
    "unknown"      // 5
};

// Landing suitability levels
enum class LandingSuitability : uint8_t {
    UNSUITABLE = 0,
    MARGINAL = 1,
    SUITABLE = 2,
    IDEAL = 3
};

struct LandingClassifierConfig {
    // Model
    std::string model_path;
    std::string model_type = "mobilenetv3";  // "mobilenetv3", "efficientnet", "custom"
    
    // Inference engine config
    InferenceEngineConfig engine_config;
    
    // Classification thresholds
    float confidence_threshold = 0.6f;
    float slope_threshold_deg = 10.0f;      // Max acceptable slope
    float min_clearance_radius_m = 2.0f;    // Min clear radius
    float min_firmness_score = 0.5f;        // Min firmness (0-1)
    
    // Input
    int input_width = 224;
    int input_height = 224;
    
    // Multi-scale analysis
    bool enable_multi_scale = true;
    std::vector<float> analysis_scales = {1.0f, 0.5f, 0.25f};  // Full, half, quarter res
    
    // Safety margins
    float obstacle_margin_m = 1.0f;         // Margin around obstacles
    int min_landing_area_px = 10000;        // Min landing area in pixels
    
    // Performance
    int target_fps = 10;
    bool async_inference = true;
};

struct LandingAssessment {
    uint64_t timestamp_us;
    uint16_t drone_id;
    
    // Primary classification
    SurfaceType surface_type = SurfaceType::UNKNOWN;
    float surface_confidence = 0.0f;
    
    // Geometric assessment
    float slope_deg = 0.0f;
    float slope_confidence = 0.0f;
    Eigen::Vector3f normal_enu = Eigen::Vector3f::UnitZ();  // Surface normal
    
    // Clearance
    float clearance_radius_m = 0.0f;
    float clearance_confidence = 0.0f;
    bool has_obstructions = false;
    std::vector<Eigen::Vector3f> obstruction_positions;  // ENU positions
    
    // Firmness
    float firmness_score = 0.0f;  // 0.0 = soft, 1.0 = hard
    float firmness_confidence = 0.0f;
    
    // Overall suitability
    LandingSuitability suitability = LandingSuitability::UNSUITABLE;
    float suitability_score = 0.0f;  // 0.0 to 1.0
    
    // Recommended landing position (relative to image center)
    Eigen::Vector2f recommended_offset_m = Eigen::Vector2f::Zero();
    
    // Wind exposure (0 = sheltered, 1 = exposed)
    float wind_exposure = 0.5f;
    
    // Roughness index (0 = smooth, 1 = rough)
    float roughness_index = 0.0f;
    
    // Performance
    float total_ms = 0.0f;
    float preprocess_ms = 0.0f;
    float inference_ms = 0.0f;
    float postprocess_ms = 0.0f;
    
    bool isValid() const {
        return suitability_score > 0.0f && total_ms > 0;
    }
};

struct LandingClassifier {
    LandingClassifier() = default;
    ~LandingClassifier() = default;
    
    LandingClassifier(const LandingClassifier&) = delete;
    LandingClassifier& operator=(const LandingClassifier&) = delete;
    LandingClassifier(LandingClassifier&&) noexcept;
    LandingClassifier& operator=(LandingClassifier&&) noexcept;
    
    // Initialize
    bool initialize(const LandingClassifierConfig& config);
    
    // Assess landing site from downward camera
    LandingAssessment assess(const cv::Mat& downward_image, 
                             uint16_t drone_id, 
                             uint64_t timestamp_us,
                             const cv::Mat& depth_map = cv::Mat(),
                             const Eigen::Vector3f& drone_position_enu = Eigen::Vector3f::Zero(),
                             const Eigen::Quaternionf& drone_orientation = Eigen::Quaternionf::Identity());
    
    // Assess with pre-computed surface normal (from VIO/LiDAR)
    LandingAssessment assessWithNormal(const cv::Mat& downward_image,
                                        uint16_t drone_id,
                                        uint64_t timestamp_us,
                                        const Eigen::Vector3f& surface_normal_enu,
                                        const cv::Mat& depth_map = cv::Mat());
    
    // Get config
    const LandingClassifierConfig& getConfig() const { return config_; }
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get inference engine
    InferenceEngine* getInferenceEngine() { return inference_engine_.get(); }
    
    // Set callback
    using AssessmentCallback = std::function<void(const LandingAssessment&)>;
    void setAssessmentCallback(AssessmentCallback cb) { assessment_callback_ = cb; }
    
    using AIInferenceCallback = std::function<void(const AIInferenceResult&)>;
    void setAIInferenceCallback(AIInferenceCallback cb) { ai_inference_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    LandingClassifierConfig config_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    bool initialized_ = false;
    
    std::function<void(const LandingAssessment&)> assessment_callback_;
    std::function<void(const AIInferenceResult&)> ai_inference_callback_;
};

}  // namespace swarm::ai

#endif  // LANDING_CLASSIFIER_HPP