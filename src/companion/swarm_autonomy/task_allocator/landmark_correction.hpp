#ifndef LANDMARK_CORRECTION_HPP
#define LANDMARK_CORRECTION_HPP

#include "swarm_map.hpp"
#include "ai_inference.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>

namespace swarm::autonomy {

// Landmark-based absolute localization correction
class LandmarkCorrection {
public:
    struct Config {
        float max_landmark_age_s = 300.0f;          // Max landmark age (5 min)
        float min_landmark_confidence = 0.7f;       // Min confidence to use
        float max_position_correction_m = 10.0f;    // Max position correction
        float landmark_match_threshold = 0.8f;      // Match threshold
        int max_landmarks_per_update = 10;          // Max landmarks per correction
        float correction_weight = 0.5f;             // Correction weight (0-1)
        bool enable_loop_closure = true;            // Enable loop closure detection
        float loop_closure_threshold = 5.0f;        // Loop closure distance threshold
    };
    
    struct LandmarkObservation {
        uint64_t timestamp_us;
        uint16_t source_drone;
        std::array<uint8_t, 16> landmark_id;
        Vector3 position_enu;
        Vector3 position_covariance;  // Diagonal of covariance matrix
        SemanticClass semantic_class;
        float confidence;
        std::string descriptor;  // Visual descriptor hash
    };
    
    struct CorrectionResult {
        bool corrected = false;
        Vector3 position_correction;    // ENU correction vector
        Vector3 velocity_correction;    // ENU velocity correction
        float position_uncertainty_before;
        float position_uncertainty_after;
        int landmarks_used = 0;
        int landmarks_rejected = 0;
        bool loop_closure_detected = false;
    };
    
    struct LandmarkStats {
        int total_landmarks = 0;
        int active_landmarks = 0;
        int loop_closures_detected = 0;
        float avg_correction_m = 0.0f;
        float avg_uncertainty_reduction = 0.0f;
    };
    
    LandmarkCorrection(const Config& config = Config{});
    
    // Process visual landmark detection
    bool processLandmarkDetection(const LandmarkObservation& obs, SwarmMap& map);
    
    // Process AI landing site assessment as landmark
    bool processLandingSiteAsLandmark(const AIInferenceResult& inference, SwarmMap& map);
    
    // Compute position correction using landmarks
    CorrectionResult computeCorrection(const AgentState& state, const SwarmMap& map);
    
    // Apply correction to state estimate
    bool applyCorrection(AgentState& state, const CorrectionResult& correction);
    
    // Get landmark statistics
    const LandmarkStats& getStats() const { return _stats; }
    
    // Get active landmarks
    std::vector<Landmark> getActiveLandmarks(const SwarmMap& map) const;
    
    // Detect loop closure
    bool detectLoopClosure(const AgentState& state, const SwarmMap& map, 
                           std::optional<Vector3>& loop_closure_position);
    
    // Register known landmark (from mission planning)
    bool registerKnownLandmark(const Landmark& landmark);
    
    // Remove stale landmarks
    void pruneStaleLandmarks(SwarmMap& map);
    
    // Get correction stats
    struct CorrectionStats {
        int total_corrections = 0;
        int successful_corrections = 0;
        int failed_corrections = 0;
        float avg_correction_m = 0.0f;
        float avg_uncertainty_reduction = 0.0f;
        int loop_closures = 0;
    };
    
    const CorrectionStats& getCorrectionStats() const { return _correction_stats; }

private:
    Config _config;
    mutable std::mutex _mutex;
    
    // Known landmarks from mission planning
    std::unordered_map<std::array<uint8_t, 16>, Landmark, ArrayHash> _known_landmarks;
    
    // Detected landmarks from observations
    std::unordered_map<std::array<uint8_t, 16>, Landmark, ArrayHash> _detected_landmarks;
    
    struct LandmarkStats _stats;
    struct CorrectionStats _correction_stats;
    
    // Hash for UUID array
    struct ArrayHash {
        size_t operator()(const std::array<uint8_t, 16>& arr) const noexcept {
            size_t hash = 0;
            for (uint8_t b : arr) {
                hash = hash * 31 + b;
            }
            return hash;
        }
    };
    
    // Match observed landmark to known/detected
    std::optional<std::pair<std::array<uint8_t, 16>, Landmark>> matchLandmark(
        const LandmarkObservation& obs, const SwarmMap& map);
    
    // Compute pose correction using matched landmarks
    CorrectionResult computePoseCorrection(const AgentState& state,
                                            const std::vector<std::pair<Landmark, LandmarkObservation>>& matches);
    
    // Validate landmark observation
    bool validateObservation(const LandmarkObservation& obs) const;
    
    // Triangulate position from multiple landmarks
    Vector3 triangulatePosition(const std::vector<std::pair<Vector3, Vector3>>& landmark_pairs);
    
    // Detect loop closure using landmark re-observation
    bool checkLoopClosure(const Landmark& known, const LandmarkObservation& observed) const;
    
    // Update landmark estimate with new observation
    void updateLandmarkEstimate(Landmark& landmark, const LandmarkObservation& obs);
    
    // Prune stale landmarks
    void pruneStaleLandmarks(SwarmMap& map);
    
    // Hash for UUID array
    struct ArrayHash {
        size_t operator()(const std::array<uint8_t, 16>& arr) const noexcept {
            size_t hash = 0;
            for (uint8_t b : arr) {
                hash = hash * 31 + b;
            }
            return hash;
        }
    };
};

} // namespace swarm::autonomy

#endif // LANDMARK_CORRECTION_HPP