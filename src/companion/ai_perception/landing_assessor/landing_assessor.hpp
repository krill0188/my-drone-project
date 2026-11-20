#ifndef LANDING_ASSESSOR_HPP
#define LANDING_ASSESSOR_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <opencv2/opencv.hpp>
#include "inference_engine.hpp"

namespace swarm::ai {

enum class SurfaceType : uint8_t {
    GRASS = 0,
    CONCRETE = 1,
    DIRT = 2,
    SAND = 3,
    WATER = 4,
    UNKNOWN = 5,
};

struct LandingAssessment {
    cv::Point3f position_enu;       // Center of landing site
    float slope_deg;                // Max slope in degrees
    float clearance_radius_m;       // Clear area radius
    SurfaceType surface_type;
    float surface_confidence;
    float firmness_score;           // 0.0 - 1.0
    float overall_score;            // 0.0 - 1.0 (weighted)
    bool suitable;                  // Meets minimum criteria
    cv::Rect2d bbox_2d;             // Image coordinates
    cv::Size3f size_3d;             // 3D extent
    uint64_t timestamp_us;
};

struct LandingAssessorConfig {
    InferenceEngine::ModelConfig model_config;
    float min_clearance_radius_m = 2.0f;
    float max_slope_deg = 10.0f;
    float min_firmness_score = 0.3f;
    float min_overall_score = 0.7f;
    float assessment_altitude_m = 10.0f;  // Optimal assessment altitude
};

class LandingAssessor {
public:
    LandingAssessor() = default;
    
    bool init(const LandingAssessorConfig& config);
    
    // Assess landing site from downward camera
    std::optional<LandingAssessment> assess(const cv::Mat& downward_image,
                                            const cv::Mat& camera_matrix,
                                            const cv::Mat& dist_coeffs,
                                            const cv::Point3f& drone_position_enu,
                                            const cv::Quatf& drone_orientation);
    
    // Get last assessment
    std::optional<LandingAssessment> getLastAssessment() const {
        return _last_assessment ? std::make_optional(_last_assessment.value()) : std::nullopt;
    }

private:
    LandingAssessorConfig _config;
    std::shared_ptr<InferenceEngine> _inference_engine;
    std::optional<LandingAssessment> _last_assessment;
    
    // Private methods
    void runInference(const cv::Mat& input);
    LandingAssessment postprocessAssessment(const InferenceEngine::InferenceResult& result,
                                             const cv::Mat& camera_matrix,
                                             const cv::Mat& dist_coeffs,
                                             const cv::Point3f& drone_position_enu,
                                             const cv::Quatf& drone_orientation);
    float computeSlope(const cv::Mat& depth_map, const cv::Rect& roi);
    float computeClearance(const cv::Mat& depth_map, const cv::Point3f& center_enu);
    SurfaceType classifySurface(const cv::Mat& image_patch);
    float estimateFirmness(const cv::Mat& image_patch, const cv::Mat& depth_patch);
};

} // namespace swarm::ai

#endif // LANDING_ASSESSOR_HPP