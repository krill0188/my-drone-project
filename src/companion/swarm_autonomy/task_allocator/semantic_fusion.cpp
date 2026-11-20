#include "semantic_fusion.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

namespace swarm::autonomy {

SemanticFusion::SemanticFusion(const Config& config) : _config(config) {
    // Default class weights
    _class_weights[SemanticClass::OBSTACLE] = 2.0f;
    _class_weights[SemanticClass::STRUCTURE] = 1.5f;
    _class_weights[SemanticClass::VEGETATION] = 1.0f;
    _class_weights[SemanticClass::WATER] = 1.5f;
    _class_weights[SemanticClass::NO_FLY] = 3.0f;
    _class_weights[SemanticClass::LANDING_PAD] = 2.0f;
    _class_weights[SemanticClass::FREE] = 0.5f;
    _class_weights[SemanticClass::UNKNOWN] = 0.1f;
}

bool SemanticFusion::fuseAIInference(const AIInferenceResult& inference, SwarmMap& map) {
    if (inference.confidence < _config.ai_confidence_threshold) {
        return false;
    }
    
    SemanticObservation obs;
    obs.timestamp_us = inference.timestamp_us;
    obs.source_drone = inference.drone_id;
    obs.position_enu = inference.position_3d;
    obs.confidence = inference.confidence;
    obs.from_ai = true;
    obs.model_name = inference.metadata.model_name;
    
    // Map AI class to semantic class
    switch (inference.type) {
        case AIInferenceType::OBSTACLE:
            switch (inference.metadata.obstacle.threat_level) {
                case ThreatLevel::CRITICAL:
                case ThreatLevel::HIGH:
                    obs.semantic_class = SemanticClass::OBSTACLE;
                    break;
                default:
                    obs.semantic_class = SemanticClass::OBSTACLE;
            }
            break;
        case AIInferenceType::LANDING_SITE:
            obs.semantic_class = SemanticClass::LANDING_PAD;
            break;
        default:
            obs.semantic_class = SemanticClass::OBSTACLE;
    }
    
    return fuseObservation(obs, map);
}

bool SemanticFusion::fuseSensorReading(const SemanticObservation& observation, SwarmMap& map) {
    if (observation.confidence < 0.1f) return false;
    return fuseObservation(observation, map);
}

SemanticFusion::FusionResult SemanticFusion::fuseObservations(
    const std::vector<SemanticObservation>& observations, SwarmMap& map) {
    
    FusionResult result;
    
    for (const auto& obs : observations) {
        if (obs.from_ai) {
            if (fuseAIInferenceFromObs(obs, map)) {
                result.ai_inferences_fused++;
            }
        } else {
            if (fuseSensorReading(obs, map)) {
                result.sensor_readings_fused++;
            }
        }
        result.cells_updated++;
        
        // Check for conflicts
        // Would check for conflicts here
    }
    
    result.avg_confidence = 0.0f;
    if (!observations.empty()) {
        float sum_conf = 0.0f;
        for (const auto& obs : observations) {
            sum_conf += obs.confidence;
        }
        result.avg_confidence = sum_conf / observations.size();
    }
    
    // Resolve conflicts
    result.conflicts_resolved = resolveConflicts(map);
    
    return result;
}

bool SemanticFusion::fuseObservation(const SemanticObservation& obs, SwarmMap& map) {
    auto index_opt = getCellIndex(obs.position_enu, map);
    if (!index_opt) return false;
    
    size_t index = *index_opt;
    
    // Store in history
    std::lock_guard<std::mutex> lock(_mutex);
    auto& history = _cell_histories[index];
    
    CellHistory::Entry entry;
    entry.timestamp_us = obs.timestamp_us;
    entry.cls = obs.semantic_class;
    entry.confidence = obs.confidence;
    entry.source_drone = obs.source_drone;
    entry.from_ai = obs.from_ai;
    history.entries.push_back(entry);
    
    // Trim history
    if (history.entries.size() > _config.max_history_per_cell) {
        history.entries.erase(history.entries.begin(), 
                             history.entries.end() - _config.max_history_per_cell);
    }
    
    // Fuse into cell
    return fuseIntoCell(index, obs, map);
}

bool SemanticFusion::fuseIntoCell(size_t index, const SemanticObservation& obs, SwarmMap& map) {
    if (index >= map.semantic_class.size()) return false;
    
    uint8_t current_class = map.semantic_class[index];
    float current_conf = map.semantic_confidence[index];
    uint16_t current_drone = map.semantic_source_drone[index];
    
    // Apply class weight
    float class_weight = getClassWeight(obs.semantic_class);
    float effective_confidence = obs.confidence * getClassWeight(obs.semantic_class);
    
    // Apply model trust for AI
    if (obs.from_ai && !obs.model_name.empty()) {
        auto it = _model_trust.find(obs.model_name);
        if (it != _model_trust.end()) {
            effective_confidence *= it->second;
        }
    }
    
    // Apply sensor vs AI weight
    if (obs.from_ai) {
        effective_confidence *= _config.ai_confidence_weight;
    } else {
        effective_confidence *= _config.sensor_confidence_weight;
    }
    
    if (current_class == 0) {  // UNKNOWN
        map.semantic_class[index] = static_cast<uint8_t>(obs.semantic_class);
        map.semantic_confidence[index] = effective_confidence;
        map.semantic_source_drone[index] = obs.source_drone;
        map.semantic_timestamp_us[index] = obs.timestamp_us;
        _stats.ai_fusions++;
        return true;
    }
    
    if (static_cast<uint8_t>(obs.semantic_class) == current_class) {
        // Same class - merge confidences
        float weight_current = 1.0f;  // Would use contributor weight
        float weight_new = effective_confidence;
        
        float new_conf = (current_conf * weight_current + effective_confidence * weight_new) / 
                         (weight_current + weight_new);
        map.semantic_confidence[index] = std::clamp(new_conf, 0.0f, 1.0f);
        return true;
    }
    
    // Conflict resolution
    if (effective_confidence > current_conf + _config.conflict_resolution_threshold) {
        // New observation wins
        map.semantic_class[index] = static_cast<uint8_t>(obs.semantic_class);
        map.semantic_confidence[index] = effective_confidence;
        map.semantic_source_drone[index] = obs.source_drone;
        map.semantic_timestamp_us[index] = obs.timestamp_us;
        return true;
    }
    
    // Check spatial consistency
    if (checkSpatialConsistency(*reinterpret_cast<size_t*>(&obs), 
                                obs.semantic_class, map)) {
        map.semantic_class[index] = static_cast<uint8_t>(obs.semantic_class);
        map.semantic_confidence[index] = effective_confidence;
        map.semantic_source_drone[index] = obs.source_drone;
        map.semantic_timestamp_us[index] = obs.timestamp_us;
        return true;
    }
    
    return false;
}

SemanticFusion::FusionResult SemanticFusion::fuseObservations(
    const std::vector<SemanticObservation>& observations, SwarmMap& map) {
    
    FusionResult result;
    
    for (const auto& obs : observations) {
        if (obs.from_ai) {
            if (fuseAIInferenceFromObs(obs, map)) {
                result.ai_inferences_fused++;
            }
        } else {
            if (fuseSensorReading(obs, map)) {
                result.sensor_readings_fused++;
            }
        }
        result.cells_updated++;
    }
    
    result.avg_confidence = 0.0f;
    if (!observations.empty()) {
        float sum_conf = 0.0f;
        for (const auto& obs : observations) {
            sum_conf += obs.confidence;
        }
        result.avg_confidence = sum_conf / observations.size();
    }
    
    result.conflicts_resolved = resolveConflicts(map);
    
    return result;
}

bool SemanticFusion::fuseAIInferenceFromObs(const SemanticObservation& obs, SwarmMap& map) {
    if (obs.confidence < _config.ai_confidence_threshold) return false;
    return fuseIntoCell(*reinterpret_cast<const size_t*>(&obs), obs, map);
}

int SemanticFusion::resolveConflicts(SwarmMap& map) {
    int resolved = 0;
    
    // Check for cells with conflicting history
    for (const auto& [index, history] : _cell_histories) {
        if (history.entries.size() < 2) continue;
        
        // Check for conflicting classes in recent history
        std::unordered_map<SemanticClass, float> class_confidence;
        for (const auto& entry : history.entries) {
            class_confidence[entry.cls] += entry.confidence;
        }
        
        if (class_confidence.size() > 1) {
            // Find dominant class
            auto max_it = std::max_element(class_confidence.begin(), class_confidence.end(),
                [](const auto& a, const auto& b) { return a.second < b.second; });
            
            SemanticClass dominant = max_it->first;
            float dominant_conf = max_it->second;
            
            // Check if dominant is significantly higher
            float second_best = 0.0f;
            for (const auto& [cls, conf] : class_confidence) {
                if (cls != dominant && conf > second_best) {
                    second_best = conf;
                }
            }
            
            if (dominant_conf > second_best + _config.conflict_resolution_threshold) {
                // Resolve to dominant
                map.semantic_class[index] = static_cast<uint8_t>(dominant);
                map.semantic_confidence[index] = dominant_conf / 
                    std::accumulate(class_confidence.begin(), class_confidence.end(), 0.0f,
                                   [](float sum, const auto& p) { return sum + p.second; });
                resolved++;
            }
        }
    }
    
    return resolved;
}

void SemanticFusion::decayObservations(SwarmMap& map, uint64_t current_time_us) {
    for (size_t i = 0; i < map.semantic_confidence.size(); ++i) {
        if (map.semantic_timestamp_us[i] == 0) continue;
        
        uint64_t age_us = current_time_us - map.semantic_timestamp_us[i];
        if (age_us > 1000000) {  // Older than 1 second
            float decayed = decayConfidence(map.semantic_confidence[i], age_us);
            map.semantic_confidence[i] = decayed;
            
            if (decayed < 0.1f) {
                map.semantic_class[i] = 0;  // UNKNOWN
                map.semantic_confidence[i] = 0.0f;
            }
        }
    }
    
    // Also decay history
    for (auto it = _cell_histories.begin(); it != _cell_histories.end();) {
        auto& history = it->second;
        history.entries.erase(
            std::remove_if(history.entries.begin(), history.entries.end(),
                [current_time_us](const CellHistory::Entry& entry) {
                    return current_time_us - entry.timestamp_us > 30000000;  // 30 seconds
                }),
            history.entries.end());
        
        if (history.entries.empty()) {
            it = _cell_histories.erase(it);
        } else {
            ++it;
        }
    }
}

SemanticFusion::RegionStats SemanticFusion::getRegionStats(const SwarmMap& map, const Polygon& region) const {
    RegionStats stats;
    
    // Would iterate over cells in region
    // Simplified implementation
    stats.total_cells = 0;
    stats.avg_confidence = 0.0f;
    
    return stats;
}

void SemanticFusion::setClassWeight(SemanticClass cls, float weight) {
    std::lock_guard<std::mutex> lock(_mutex);
    _class_weights[cls] = std::clamp(weight, 0.01f, 10.0f);
}

float SemanticFusion::getClassWeight(SemanticClass cls) const {
    auto it = _class_weights.find(cls);
    if (it != _class_weights.end()) return it->second;
    return 1.0f;
}

void SemanticFusion::setModelTrust(const std::string& model_name, float trust) {
    std::lock_guard<std::mutex> lock(_mutex);
    _model_trust[model_name] = std::clamp(trust, 0.0f, 1.0f);
}

float SemanticFusion::getModelTrust(const std::string& model_name) const {
    auto it = _model_trust.find(model_name);
    if (it != _model_trust.end()) return it->second;
    return 1.0f;
}

const SemanticFusion::Stats& SemanticFusion::getStats() const {
    return _stats;
}

std::vector<SemanticFusion::SemanticObservation> SemanticFusion::exportObservations(const SwarmMap& map) const {
    std::vector<SemanticObservation> observations;
    
    for (const auto& [index, history] : _cell_histories) {
        for (const auto& entry : history.entries) {
            SemanticObservation obs;
            obs.timestamp_us = entry.timestamp_us;
            obs.source_drone = entry.source_drone;
            obs.semantic_class = entry.cls;
            obs.confidence = entry.confidence;
            obs.from_ai = entry.from_ai;
            
            // Convert index to position
            if (index < map.occupancy_probability.size()) {
                // Would convert index to ENU position
            }
            
            observations.push_back(obs);
        }
    }
    
    return observations;
}

bool SemanticFusion::fuseObservation(const SemanticObservation& obs, SwarmMap& map) {
    return fuseIntoCell(*reinterpret_cast<const size_t*>(&obs), obs, map);
}

bool SemanticFusion::resolveConflict(size_t index, SwarmMap& map) {
    // Check history for this cell
    auto it = _cell_histories.find(index);
    if (it == _cell_histories.end()) return false;
    
    auto& history = it->second;
    if (history.entries.size() < 2) return false;
    
    // Check for conflicting classes
    std::unordered_map<SemanticClass, float> class_weights;
    for (const auto& entry : history.entries) {
        class_weights[entry.cls] += entry.confidence;
    }
    
    if (class_weights.size() <= 1) return false;
    
    // Find top two classes
    auto max_it = std::max_element(class_weights.begin(), class_weights.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });
    
    SemanticClass winner = max_it->first;
    float winner_weight = max_it->second;
    
    // Remove winner and find second best
    class_weights.erase(max_it);
    auto second_it = std::max_element(class_weights.begin(), class_weights.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; });
    
    if (second_it == class_weights.end()) return false;
    
    float second_best = second_it->second;
    
    // If winner is significantly better, resolve
    if (winner_weight > second_best + _config.conflict_resolution_threshold) {
        map.semantic_class[index] = static_cast<uint8_t>(winner);
        float total = 0.0f;
        for (const auto& p : class_weights) total += p.second;
        map.semantic_confidence[index] = winner_weight / total;
        return true;
    }
    
    return false;
}

bool SemanticFusion::temporalFusion(size_t index, const SemanticObservation& obs, SwarmMap& map) {
    // Weighted by recency
    auto it = _cell_histories.find(index);
    if (it == _cell_histories.end()) return false;
    
    auto& history = it->second;
    if (history.entries.empty()) return false;
    
    // Weight recent observations more heavily
    float weighted_sum = 0.0f;
    float weight_sum = 0.0f;
    uint64_t now = obs.timestamp_us;
    
    for (const auto& entry : history.entries) {
        uint64_t age = now - entry.timestamp_us;
        float time_weight = std::exp(-static_cast<float>(age) * _config.temporal_decay_rate / 1e6);
        
        float total_weight = entry.confidence * time_weight;
        weighted_sum += static_cast<float>(entry.cls) * total_weight;
        weight_sum += total_weight;
    }
    
    if (weight_sum > 0) {
        SemanticClass fused = static_cast<SemanticClass>(std::round(weighted_sum / weight_sum));
        map.semantic_class[index] = static_cast<uint8_t>(fused);
        map.semantic_confidence[index] = weight_sum / history.entries.size();
        return true;
    }
    
    return false;
}

bool SemanticFusion::checkSpatialConsistency(size_t index, SemanticClass cls, const SwarmMap& map) {
    // Check neighbors for consistency
    // Simplified - would check neighboring cells in real implementation
    return true;
}

std::optional<size_t> SemanticFusion::getCellIndex(const Vector3& pos, const SwarmMap& map) const {
    if (map.grid_width == 0 || map.grid_height == 0) return std::nullopt;
    
    // Convert ENU position to grid index
    Vector3 rel = pos - map.origin;
    
    int x = static_cast<int>(rel.x / map.resolution_m);
    int y = static_cast<int>(rel.y / map.resolution_m);
    int z = static_cast<int>(rel.z / map.resolution_m);
    
    if (x < 0 || x >= static_cast<int>(map.grid_width) ||
        y < 0 || y >= static_cast<int>(map.grid_height) ||
        z < 0 || z >= static_cast<int>(map.grid_depth)) {
        return std::nullopt;
    }
    
    return static_cast<size_t>(z * map.grid_width * map.grid_height + 
                               y * map.grid_width + x);
}

float SemanticFusion::decayConfidence(float confidence, uint64_t age_us) const {
    float age_s = static_cast<float>(age_us) / 1e6;
    return confidence * std::exp(-age_s * _config.temporal_decay_rate);
}

void SemanticFusion::setClassWeight(SemanticClass cls, float weight) {
    std::lock_guard<std::mutex> lock(_mutex);
    _class_weights[cls] = std::clamp(weight, 0.01f, 10.0f);
}

float SemanticFusion::getClassWeight(SemanticClass cls) const {
    auto it = _class_weights.find(cls);
    if (it != _class_weights.end()) return it->second;
    return 1.0f;
}

void SemanticFusion::setModelTrust(const std::string& model_name, float trust) {
    std::lock_guard<std::mutex> lock(_mutex);
    _model_trust[model_name] = std::clamp(trust, 0.0f, 1.0f);
}

float SemanticFusion::getModelTrust(const std::string& model_name) const {
    auto it = _model_trust.find(model_name);
    if (it != _model_trust.end()) return it->second;
    return 1.0f;
}

const SemanticFusion::Stats& SemanticFusion::getStats() const {
    return _stats;
}

std::vector<SemanticFusion::SemanticObservation> SemanticFusion::exportObservations(const SwarmMap& map) const {
    std::vector<SemanticObservation> observations;
    
    for (const auto& [index, history] : _cell_histories) {
        for (const auto& entry : history.entries) {
            SemanticObservation obs;
            obs.timestamp_us = entry.timestamp_us;
            obs.source_drone = entry.source_drone;
            obs.semantic_class = entry.cls;
            obs.confidence = entry.confidence;
            obs.from_ai = entry.from_ai;
            observations.push_back(obs);
        }
    }
    
    return observations;
}

} // namespace swarm::autonomy