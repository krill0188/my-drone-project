#include "landmark_correction.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

namespace swarm::autonomy {

LandmarkCorrection::LandmarkCorrection(const Config& config) : _config(config) {}

bool LandmarkCorrection::processLandmarkDetection(const LandmarkObservation& obs, SwarmMap& map) {
    if (!validateObservation(obs)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(_mutex);
    
    // Try to match with known landmarks
    auto match = matchLandmark(obs, map);
    
    if (match) {
        // Update existing landmark
        auto& landmark = _detected_landmarks[match->first];
        updateLandmarkEstimate(landmark, obs);
        return true;
    } else {
        // New landmark - add to detected
        Landmark landmark;
        landmark.id = obs.landmark_id;
        landmark.position_enu = obs.position_enu;
        landmark.type = LandmarkType::VISUAL;
        landmark.confidence = obs.confidence;
        landmark.timestamp_us = obs.timestamp_us;
        landmark.source_drone = obs.source_drone;
        landmark.descriptor = obs.descriptor;
        
        _detected_landmarks[obs.landmark_id] = landmark;
        
        // Also add to map
        map.landmarks.push_back(landmark);
        
        return true;
    }
}

bool LandmarkCorrection::processLandingSiteAsLandmark(const AIInferenceResult& inference, SwarmMap& map) {
    if (inference.type != AIInferenceType::LANDING_SITE) return false;
    if (inference.confidence < _config.min_landmark_confidence) return false;
    
    LandmarkObservation obs;
    obs.timestamp_us = inference.timestamp_us;
    obs.source_drone = inference.drone_id;
    obs.position_enu = inference.position_enu;
    obs.position_covariance = {inference.metadata.landing.slope_deg, 
                               inference.metadata.landing.clearance_radius_m, 
                               0};
    obs.semantic_class = SemanticClass::LANDING_PAD;
    obs.confidence = inference.confidence;
    obs.descriptor = "landing_site_" + inference.metadata.model_name;
    
    // Generate ID from position
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);
    for (auto& b : obs.landmark_id) {
        b = dis(gen);
    }
    
    return processLandmarkDetection(obs, map);
}

LandmarkCorrection::CorrectionResult LandmarkCorrection::computeCorrection(
    const AgentState& state, const SwarmMap& map) {
    
    CorrectionResult result;
    result.position_uncertainty_before = state.pos_covariance_trace;
    
    // Collect visible landmarks
    std::vector<std::pair<Landmark, LandmarkObservation>> matches;
    
    // Get active landmarks in range
    auto landmarks = getActiveLandmarks(map);
    
    // Simulate observations for nearby landmarks
    std::vector<LandmarkObservation> observations;
    for (const auto& lm : landmarks) {
        // Check if landmark is in sensor range
        float dx = state.position.x - lm.position_enu.x;
        float dy = state.position.y - lm.position_enu.y;
        float dz = state.position.z - lm.position_enu.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < 100.0f) {  // Within 100m
            LandmarkObservation obs;
            obs.timestamp_us = state.timestamp_us;
            obs.source_drone = state.sysid;  // Simplified
            obs.landmark_id = lm.id;
            obs.position_enu = lm.position_enu;
            obs.confidence = lm.confidence;
            obs.descriptor = lm.descriptor;
            obs.semantic_class = SemanticClass::LANDING_PAD;  // Simplified
            
            observations.push_back(obs);
        }
    }
    
    if (observations.size() < 2) {
        // Not enough landmarks for correction
        return result;
    }
    
    // Match observations to landmarks
    std::vector<std::pair<Landmark, LandmarkObservation>> matches;
    for (const auto& obs : observations) {
        auto match = matchLandmark(obs, map);
        if (match) {
            matches.emplace_back(match->second, obs);
        }
    }
    
    if (matches.size() < 2) {
        return result;
    }
    
    // Compute pose correction using matched landmarks
    result = computePoseCorrection(state, matches);
    
    if (result.corrected) {
        // Check for loop closure
        std::optional<Vector3> loop_closure_pos;
        if (detectLoopClosure(state, map, loop_closure_pos)) {
            result.loop_closure_detected = true;
        }
        
        // Update stats
        {
            std::lock_guard<std::mutex> lock(_mutex);
            _correction_stats.total_corrections++;
            _correction_stats.successful_corrections++;
            _correction_stats.avg_correction_m = 
                0.9f * _correction_stats.avg_correction_m + 0.1f * 
                std::sqrt(result.position_correction.x * result.position_correction.x +
                         result.position_correction.y * result.position_correction.y +
                         result.position_correction.z * result.position_correction.z);
            
            float reduction = (result.position_uncertainty_before - result.position_uncertainty_after) / 
                            result.position_uncertainty_before;
            _correction_stats.avg_uncertainty_reduction = 
                0.9f * _correction_stats.avg_uncertainty_reduction + 0.1f * reduction;
            
            if (result.loop_closure_detected) {
                _correction_stats.loop_closures++;
                _stats.loop_closures_detected++;
            }
        }
    }
    
    return result;
}

bool LandmarkCorrection::applyCorrection(AgentState& state, const CorrectionResult& correction) {
    if (!correction.corrected) return false;
    
    // Apply position correction (weighted)
    float weight = _config.correction_weight;
    state.position.x += correction.position_correction.x * weight;
    state.position.y += correction.position_correction.y * weight;
    state.position.z += correction.position_correction.z * weight;
    
    // Apply velocity correction
    state.velocity.x += correction.velocity_correction.x * weight;
    state.velocity.y += correction.velocity_correction.y * weight;
    state.velocity.z += correction.velocity_correction.z * weight;
    
    // Update covariance
    state.pos_covariance_trace = correction.position_uncertainty_after;
    
    return true;
}

std::vector<Landmark> LandmarkCorrection::getActiveLandmarks(const SwarmMap& map) const {
    std::vector<Landmark> active;
    uint64_t now = 0; // Simplified
    
    for (const auto& [id, landmark] : _detected_landmarks) {
        if (now - landmark.timestamp_us < static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            if (landmark.confidence >= _config.min_landmark_confidence) {
                active.push_back(landmark);
            }
        }
    }
    
    // Also add from map
    for (const auto& lm : map.landmarks) {
        if (now - lm.timestamp_us < static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            active.push_back(lm);
        }
    }
    
    // Remove duplicates
    std::sort(active.begin(), active.end(), 
        [](const Landmark& a, const Landmark& b) {
            return a.id < b.id;
        });
    active.erase(std::unique(active.begin(), active.end(),
        [](const Landmark& a, const Landmark& b) { return a.id == b.id; }),
        active.end());
    
    return active;
}

bool LandmarkCorrection::detectLoopClosure(const AgentState& state, const SwarmMap& map,
                                            std::optional<Vector3>& loop_closure_position) {
    if (!_config.enable_loop_closure) return false;
    
    // Check if current position is near a previously visited landmark
    auto landmarks = getActiveLandmarks(map);
    
    for (const auto& lm : landmarks) {
        float dx = state.position.x - lm.position_enu.x;
        float dy = state.position.y - lm.position_enu.y;
        float dz = state.position.z - lm.position_enu.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < _config.loop_closure_threshold) {
            // Check if this landmark was visited before
            // In real implementation, would check visitation history
            loop_closure_position = lm.position_enu;
            
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _stats.loop_closures_detected++;
            }
            
            return true;
        }
    }
    
    return false;
}

bool LandmarkCorrection::registerKnownLandmark(const Landmark& landmark) {
    std::lock_guard<std::mutex> lock(_mutex);
    _known_landmarks[landmark.id] = landmark;
    return true;
}

void LandmarkCorrection::pruneStaleLandmarks(SwarmMap& map) {
    std::lock_guard<std::mutex> lock(_mutex);
    uint64_t now = 0; // Simplified
    
    // Prune detected landmarks
    for (auto it = _detected_landmarks.begin(); it != _detected_landmarks.end();) {
        if (now - it->second.timestamp_us > static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            it = _detected_landmarks.erase(it);
        } else {
            ++it;
        }
    }
    
    // Prune map landmarks
    for (auto it = map.landmarks.begin(); it != map.landmarks.end();) {
        if (now - it->timestamp_us > static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            it = map.landmarks.erase(it);
        } else {
            ++it;
        }
    }
}

std::optional<std::pair<std::array<uint8_t, 16>, Landmark>> 
LandmarkCorrection::matchLandmark(const LandmarkObservation& obs, const SwarmMap& map) {
    // Try exact ID match first
    auto it = _detected_landmarks.find(obs.landmark_id);
    if (it != _detected_landmarks.end()) {
        // Check position consistency
        float dx = it->second.position_enu.x - obs.position_enu.x;
        float dy = it->second.position_enu.y - obs.position_enu.y;
        float dz = it->second.position_enu.z - obs.position_enu.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < 5.0f) {  // Within 5m
            return {{it->first, it->second}};
        }
    }
    
    // Try matching with known landmarks
    for (const auto& [id, landmark] : _known_landmarks) {
        float dx = landmark.position_enu.x - obs.position_enu.x;
        float dy = landmark.position_enu.y - obs.position_enu.y;
        float dz = landmark.position_enu.z - obs.position_enu.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < 5.0f && landmark.confidence > _config.landmark_match_threshold) {
            // Update observed landmark ID to match known
            return {{id, landmark}};
        }
    }
    
    // Try descriptor matching
    if (!obs.descriptor.empty()) {
        for (const auto& [id, landmark] : _detected_landmarks) {
            if (landmark.descriptor == obs.descriptor) {
                float dx = landmark.position_enu.x - obs.position_enu.x;
                float dy = landmark.position_enu.y - obs.position_enu.y;
                float dz = landmark.position_enu.z - obs.position_enu.z;
                float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                
                if (dist < 5.0f) {
                    return {{id, landmark}};
                }
            }
        }
    }
    
    return std::nullopt;
}

LandmarkCorrection::CorrectionResult LandmarkCorrection::computePoseCorrection(
    const AgentState& state,
    const std::vector<std::pair<Landmark, LandmarkObservation>>& matches) {
    
    CorrectionResult result;
    result.position_uncertainty_before = state.pos_covariance_trace;
    
    if (matches.size() < 2) {
        return result;
    }
    
    // Use weighted least squares to compute position correction
    // Simplified: average of landmark position differences
    Vector3 total_correction{0, 0, 0};
    float total_weight = 0.0f;
    
    for (const auto& [landmark, obs] : matches) {
        // Compute expected position from landmark
        Vector3 expected_pos = landmark.position_enu;
        Vector3 observed_pos = obs.position_enu;
        
        Vector3 error = expected_pos - observed_pos;
        float weight = obs.confidence * landmark.confidence;
        
        total_correction.x += error.x * weight;
        total_correction.y += error.y * weight;
        total_correction.z += error.z * weight;
        total_weight += weight;
    }
    
    if (total_weight > 0) {
        result.position_correction.x = total_correction.x / total_weight;
        result.position_correction.y = total_correction.y / total_weight;
        result.position_correction.z = total_correction.z / total_weight;
        
        // Limit correction magnitude
        float correction_mag = std::sqrt(
            result.position_correction.x * result.position_correction.x +
            result.position_correction.y * result.position_correction.y +
            result.position_correction.z * result.position_correction.z);
        
        if (correction_mag > _config.max_position_correction_m) {
            float scale = _config.max_position_correction_m / correction_mag;
            result.position_correction.x *= scale;
            result.position_correction.y *= scale;
            result.position_correction.z *= scale;
        }
        
        result.corrected = true;
        result.landmarks_used = matches.size();
        
        // Estimate uncertainty reduction
        float uncertainty_reduction = 1.0f / (1.0f + total_weight * 10.0f);
        result.position_uncertainty_after = result.position_uncertainty_before * uncertainty_reduction;
    }
    
    return result;
}

bool LandmarkCorrection::applyCorrection(AgentState& state, const CorrectionResult& correction) {
    if (!correction.corrected) return false;
    
    float weight = _config.correction_weight;
    state.position.x += correction.position_correction.x * weight;
    state.position.y += correction.position_correction.y * weight;
    state.position.z += correction.position_correction.z * weight;
    
    state.velocity.x += correction.velocity_correction.x * weight;
    state.velocity.y += correction.velocity_correction.y * weight;
    state.velocity.z += correction.velocity_correction.z * weight;
    
    state.pos_covariance_trace = correction.position_uncertainty_after;
    
    return true;
}

std::vector<Landmark> LandmarkCorrection::getActiveLandmarks(const SwarmMap& map) const {
    std::vector<Landmark> active;
    uint64_t now = 0; // Simplified
    
    for (const auto& [id, landmark] : _detected_landmarks) {
        if (now - landmark.timestamp_us < static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            if (landmark.confidence >= _config.min_landmark_confidence) {
                active.push_back(landmark);
            }
        }
    }
    
    for (const auto& lm : map.landmarks) {
        if (now - lm.timestamp_us < static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            if (lm.confidence >= _config.min_landmark_confidence) {
                active.push_back(lm);
            }
        }
    }
    
    // Deduplicate
    std::sort(active.begin(), active.end(), 
        [](const Landmark& a, const Landmark& b) { return a.id < b.id; });
    active.erase(std::unique(active.begin(), active.end(),
        [](const Landmark& a, const Landmark& b) { return a.id == b.id; }),
        active.end());
    
    return active;
}

bool LandmarkCorrection::detectLoopClosure(const AgentState& state, const SwarmMap& map,
                                            std::optional<Vector3>& loop_closure_position) {
    if (!_config.enable_loop_closure) return false;
    
    auto landmarks = getActiveLandmarks(map);
    
    for (const auto& lm : landmarks) {
        float dx = state.position.x - lm.position_enu.x;
        float dy = state.position.y - lm.position_enu.y;
        float dz = state.position.z - lm.position_enu.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        
        if (dist < _config.loop_closure_threshold) {
            loop_closure_position = lm.position_enu;
            
            {
                std::lock_guard<std::mutex> lock(_mutex);
                _stats.loop_closures_detected++;
                _correction_stats.loop_closures++;
            }
            
            return true;
        }
    }
    
    return false;
}

bool LandmarkCorrection::registerKnownLandmark(const Landmark& landmark) {
    std::lock_guard<std::mutex> lock(_mutex);
    _known_landmarks[landmark.id] = landmark;
    return true;
}

void LandmarkCorrection::pruneStaleLandmarks(SwarmMap& map) {
    std::lock_guard<std::mutex> lock(_mutex);
    uint64_t now = 0; // Simplified
    
    for (auto it = _detected_landmarks.begin(); it != _detected_landmarks.end();) {
        if (now - it->second.timestamp_us > static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            it = _detected_landmarks.erase(it);
        } else {
            ++it;
        }
    }
    
    for (auto it = map.landmarks.begin(); it != map.landmarks.end();) {
        if (now - it->timestamp_us > static_cast<uint64_t>(_config.max_landmark_age_s * 1e6)) {
            it = map.landmarks.erase(it);
        } else {
            ++it;
        }
    }
}

} // namespace swarm::autonomy