#include "map_fusion.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <iostream>

namespace swarm::autonomy {

MapFusion::MapFusion(const Config& config) : _config(config) {}

MapFusion::FusionResult MapFusion::fuse(const SwarmMap& source, SwarmMap& target) {
    std::lock_guard<std::mutex> lock(_mutex);
    
    FusionResult result;
    result.success = false;
    result.cells_updated = 0;
    result.cells_conflicted = 0;
    result.contributors_merged = 0;
    result.avg_consistency = 0.0f;
    
    if (!source.validate() || !target.validate()) {
        _stats.failed_fusions++;
        return result;
    }
    
    // Check grid compatibility
    if (source.grid_width != target.grid_width ||
        source.grid_height != target.grid_height ||
        source.grid_depth != target.grid_depth ||
        std::abs(source.resolution_m - target.resolution_m) > 0.01f) {
        // Grid mismatch - would need resampling in real implementation
        _stats.failed_fusions++;
        return result;
    }
    
    size_t total_cells = target.getTotalCells();
    if (total_cells == 0) {
        _stats.failed_fusions++;
        return result;
    }
    
    float total_consistency = 0.0f;
    int consistency_checks = 0;
    
    // Fuse each cell
    for (size_t i = 0; i < total_cells; ++i) {
        // Occupancy fusion
        bool occ_updated = fuseOccupancyCell(source, target, i);
        if (occ_updated) result.cells_updated++;
        
        // Semantic fusion
        bool sem_updated = fuseSemanticCell(
            target.semantic_class[i], target.semantic_confidence[i],
            source.semantic_class[i], source.semantic_confidence[i],
            source.semantic_source_drone[i], source.semantic_timestamp_us[i]);
        if (sem_updated) result.cells_updated++;
        
        // Check consistency
        float consistency = checkCellConsistency(source, target, i);
        total_consistency += consistency;
        consistency_checks++;
        
        if (consistency < _config.consistency_threshold) {
            result.cells_conflicted++;
        }
    }
    
    // Fuse semantic labels
    fuseSemantic(source, target);
    
    // Fuse no-fly zones
    fuseNoFlyZones(source, target);
    
    // Fuse landmarks
    fuseLandmarks(source, target);
    
    // Update contributor info
    for (uint16_t drone_id : source.contributors) {
        updateContributor(drone_id, source.last_update_us, 1);
        result.contributors_merged++;
    }
    
    // Merge contributor lists
    for (uint16_t drone_id : source.contributors) {
        if (std::find(target.contributors.begin(), target.contributors.end(), drone_id) 
            == target.contributors.end()) {
            target.contributors.push_back(drone_id);
        }
    }
    
    // Update metadata
    target.version = std::max(target.version, source.version) + 1;
    target.last_update_us = std::max(target.last_update_us, source.last_update_us);
    
    if (consistency_checks > 0) {
        result.avg_consistency = total_consistency / consistency_checks;
    }
    
    result.success = true;
    _stats.successful_fusions++;
    _stats.total_fusions++;
    _stats.total_cells_updated += result.cells_updated;
    _stats.total_conflicts_resolved += result.cells_conflicted;
    _stats.avg_consistency = 0.9f * _stats.avg_consistency + 0.1f * result.avg_consistency;
    
    return result;
}

MapFusion::FusionResult MapFusion::fuseMultiple(const std::vector<SwarmMap*>& sources, SwarmMap& target) {
    FusionResult combined_result;
    combined_result.success = false;
    combined_result.cells_updated = 0;
    combined_result.cells_conflicted = 0;
    combined_result.contributors_merged = 0;
    combined_result.avg_consistency = 0.0f;
    
    for (const auto* source : sources) {
        if (source) {
            auto result = fuse(*source, target);
            combined_result.cells_updated += result.cells_updated;
            combined_result.cells_conflicted += result.cells_conflicted;
            combined_result.contributors_merged += result.contributors_merged;
            combined_result.avg_consistency += result.avg_consistency;
            if (!result.success) {
                combined_result.success = false;
            }
        }
    }
    
    if (!sources.empty()) {
        combined_result.avg_consistency /= sources.size();
    }
    
    return combined_result;
}

bool MapFusion::fuseOccupancy(const SwarmMap& source, SwarmMap& target) {
    // Fuse each cell
    size_t total_cells = target.getTotalCells();
    for (size_t i = 0; i < total_cells; ++i) {
        fuseOccupancyCell(source, target, i);
    }
    return true;
}

bool MapFusion::fuseSemantic(const SwarmMap& source, SwarmMap& target) {
    size_t total_cells = target.getTotalCells();
    for (size_t i = 0; i < total_cells; ++i) {
        fuseSemanticCell(
            target.semantic_class[i], target.semantic_confidence[i],
            source.semantic_class[i], source.semantic_confidence[i],
            source.semantic_source_drone[i], source.semantic_timestamp_us[i]);
    }
    return true;
}

bool MapFusion::fuseNoFlyZones(const SwarmMap& source, SwarmMap& target) {
    for (const auto& source_zone : source.no_fly_zones) {
        // Check if zone already exists in target
        auto it = std::find_if(target.no_fly_zones.begin(), target.no_fly_zones.end(),
            [&source_zone](const NoFlyZone& tz) { return tz.id == source_zone.id; });
        
        if (it == target.no_fly_zones.end()) {
            // New zone - add it
            target.no_fly_zones.push_back(source_zone);
        } else {
            // Fuse existing zone
            fuseNoFlyZone(*it, source_zone);
        }
    }
    return true;
}

bool MapFusion::fuseLandmarks(const SwarmMap& source, SwarmMap& target) {
    for (const auto& source_lm : source.landmarks) {
        auto it = std::find_if(target.landmarks.begin(), target.landmarks.end(),
            [&source_lm](const Landmark& tl) { return tl.id == source_lm.id; });
        
        if (it == target.landmarks.end()) {
            target.landmarks.push_back(source_lm);
        } else {
            fuseLandmark(*it, source_lm);
        }
    }
    return true;
}

MapFusion::FusionResult MapFusion::fuseOccupancyCell(const SwarmMap& source, SwarmMap& target, size_t index) {
    FusionResult result;
    result.success = true;
    
    float source_prob = source.occupancy_probability[index];
    uint32_t source_hits = source.occupancy_hits[index];
    uint32_t source_misses = source.occupancy_misses[index];
    uint64_t source_time = source.occupancy_last_update_us[index];
    
    float target_prob = target.occupancy_probability[index];
    uint32_t target_hits = target.occupancy_hits[index];
    uint32_t target_misses = target.occupancy_misses[index];
    uint64_t target_time = target.occupancy_last_update_us[index];
    
    // Calculate sensor reliability based on recency
    uint64_t now = source.occupancy_last_update_us[index]; // Simplified
    float source_reliability = 1.0f; // Would calculate based on recency
    float target_reliability = 1.0f;
    
    // Bayesian update
    float fused_prob = bayesianUpdate(target_prob, source_prob, source_reliability, 
                                      source_hits, source_misses);
    
    target.occupancy_probability[index] = fused_prob;
    target.occupancy_hits[index] += source_hits;
    target.occupancy_misses[index] += source_misses;
    target.occupancy_last_update_us[index] = std::max(target_time, source_time);
    
    return true;
}

float MapFusion::bayesianUpdate(float prior, float sensor_prob, float sensor_reliability,
                                 int hits, int misses) {
    // Bayesian update with sensor reliability weighting
    // P(occ|z) = P(z|occ)P(occ) / [P(z|occ)P(occ) + P(z|free)P(free)]
    
    // Convert to log-odds for numerical stability
    float prior_logodds = (prior > 0 && prior < 1) ? std::log(prior / (1 - prior)) : 0;
    
    // Sensor log-odds
    float sensor_logodds = (sensor_prob > 0 && sensor_prob < 1) ? 
        std::log(sensor_prob / (1 - sensor_prob)) : 0;
    
    // Apply reliability weighting
    float weighted_sensor_logodds = sensor_logodds * sensor_reliability;
    
    // Posterior log-odds
    float posterior_logodds = prior_logodds + weighted_sensor_logodds;
    
    // Convert back to probability
    float posterior = 1.0f / (1.0f + std::exp(-posterior_logodds));
    
    return std::clamp(posterior, 0.001f, 0.999f);
}

bool MapFusion::fuseSemanticCell(uint8_t& target_class, float& target_confidence,
                                 uint8_t source_class, float source_confidence,
                                 uint16_t source_drone, uint64_t timestamp_us) {
    if (source_class == 0) return false;  // UNKNOWN
    
    if (target_class == 0) {
        // Target is unknown - adopt source
        target_class = source_class;
        target_confidence = source_confidence;
        return true;
    }
    
    if (target_class == source_class) {
        // Same class - merge confidences
        float weight_target = 1.0f;
        float weight_source = 1.0f;
        
        // Weight by confidence
        float new_confidence = (target_confidence * weight_target + source_confidence * weight_source) / 
                               (weight_target + weight_source);
        target_confidence = std::clamp(new_confidence, 0.0f, 1.0f);
        return true;
    }
    
    // Conflict - use higher confidence
    if (source_confidence > target_confidence + 0.1f) {
        target_class = source_class;
        target_confidence = source_confidence;
        return true;
    }
    
    return false;
}

bool MapFusion::fuseNoFlyZone(NoFlyZone& target, const NoFlyZone& source) {
    // Verify signatures
    if (!source.signature.empty()) {
        // In real implementation, would verify cryptographic signature
    }
    
    // Take more restrictive (larger) zone
    // Simplified: update validity period to union
    target.valid_from_us = std::min(target.valid_from_us, source.valid_from_us);
    target.valid_until_us = std::max(target.valid_until_us, source.valid_until_us);
    
    // Update boundary to union (simplified)
    // Real implementation would compute polygon union
    return true;
}

bool MapFusion::fuseLandmark(Landmark& target, const Landmark& source) {
    // Keep landmark with higher confidence
    if (source.confidence > target.confidence) {
        target.confidence = source.confidence;
        target.position_enu = source.position_enu;
        target.timestamp_us = source.timestamp_us;
        target.source_drone = source.source_drone;
    }
    return true;
}

bool MapFusion::fuseSemantic(const SwarmMap& source, SwarmMap& target) {
    size_t total_cells = target.getTotalCells();
    for (size_t i = 0; i < total_cells; ++i) {
        fuseSemanticCell(
            target.semantic_class[i], target.semantic_confidence[i],
            source.semantic_class[i], source.semantic_confidence[i],
            source.semantic_source_drone[i], source.semantic_timestamp_us[i]);
    }
    return true;
}

std::unordered_map<uint16_t, float> MapFusion::getContributorWeights(const SwarmMap& map) const {
    std::unordered_map<uint16_t, float> weights;
    for (uint16_t drone_id : map.contributors) {
        weights[drone_id] = getContributorWeight(drone_id);
    }
    return weights;
}

void MapFusion::decayStaleCells(SwarmMap& map) {
    uint64_t now = map.last_update_us; // Simplified
    size_t total_cells = map.getTotalCells();
    
    for (size_t i = 0; i < total_cells; ++i) {
        uint64_t age = now - map.occupancy_last_update_us[i];
        if (age > _config.max_cell_age_us) {
            // Decay probability toward 0.5 (unknown)
            float decay = _config.decay_rate * (static_cast<float>(age) / _config.max_cell_age_us);
            map.occupancy_probability[i] = 0.5f + (map.occupancy_probability[i] - 0.5f) * (1.0f - decay);
            map.semantic_confidence[i] *= (1.0f - decay);
            if (map.semantic_confidence[i] < 0.1f) {
                map.semantic_class[i] = 0;  // UNKNOWN
            }
        }
    }
}

bool MapFusion::validateConsistency(const SwarmMap& map) const {
    // Check grid dimensions
    size_t expected_cells = map.grid_width * map.grid_height * map.grid_depth;
    if (map.getTotalCells() != expected_cells) return false;
    
    // Check array sizes
    if (map.occupancy_probability.size() != map.getTotalCells()) return false;
    if (map.occupancy_hits.size() != map.getTotalCells()) return false;
    if (map.semantic_class.size() != map.getTotalCells()) return false;
    
    // Check probability bounds
    for (float p : map.occupancy_probability) {
        if (p < 0.0f || p > 1.0f) return false;
    }
    
    // Check confidence bounds
    for (float c : map.semantic_confidence) {
        if (c < 0.0f || c > 1.0f) return false;
    }
    
    return true;
}

MapFusion::Stats MapFusion::getStats() const {
    std::lock_guard<std::mutex> lock(_mutex);
    return _stats;
}

void MapFusion::setContributorWeight(uint16_t drone_id, float weight) {
    std::lock_guard<std::mutex> lock(_mutex);
    _custom_weights[drone_id] = std::clamp(weight, 0.1f, 10.0f);
}

float MapFusion::getContributorWeight(uint16_t drone_id) const {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _custom_weights.find(drone_id);
    if (it != _custom_weights.end()) return it->second;
    
    auto cit = _contributors.find(drone_id);
    if (cit != _contributors.end()) return cit->second.weight;
    
    return _config.contributor_weight_base;
}

bool MapFusion::fuseSemanticCell(uint8_t& target_class, float& target_confidence,
                                 uint8_t source_class, float source_confidence,
                                 uint16_t source_drone, uint64_t timestamp_us) {
    if (source_class == 0) return false;
    
    float source_weight = getContributorWeight(source_drone) * source_confidence * 
                          _config.sensor_quality_weight;
    
    // Recency weight
    uint64_t now = 0; // Simplified
    float age_factor = 1.0f;
    if (timestamp_us > 0) {
        uint64_t age = 0; // Simplified
        age_factor = std::exp(-static_cast<float>(age) / 1e7); // Decay over ~10s
    }
    source_weight *= age_factor * _config.recency_weight;
    
    if (target_class == 0) {
        target_class = source_class;
        target_confidence = source_confidence;
        return true;
    }
    
    if (target_class == source_class) {
        // Weighted average
        float target_weight = 1.0f; // Would use contributor weight
        float new_conf = (target_confidence * target_weight + source_confidence * source_weight) / 
                         (target_weight + source_weight);
        target_confidence = std::clamp(new_conf, 0.0f, 1.0f);
        return true;
    }
    
    // Conflict resolution
    float target_weight = 1.0f;
    if (source_weight > target_weight) {
        target_class = source_class;
        target_confidence = source_confidence;
        return true;
    }
    
    return false;
}

bool MapFusion::fuseNoFlyZone(NoFlyZone& target, const NoFlyZone& source) {
    // Expand to union
    target.valid_from_us = std::min(target.valid_from_us, source.valid_from_us);
    target.valid_until_us = std::max(target.valid_until_us, source.valid_until_us);
    return true;
}

bool MapFusion::fuseLandmark(Landmark& target, const Landmark& source) {
    if (source.confidence > target.confidence) {
        target.confidence = source.confidence;
        target.position_enu = source.position_enu;
        target.timestamp_us = source.timestamp_us;
        target.source_drone = source.source_drone;
    }
    return true;
}

float MapFusion::calculateContributorWeight(const ContributorInfo& info) const {
    float weight = _config.contributor_weight_base;
    weight *= info.weight;
    
    // Recency boost
    uint64_t now = 0; // Simplified
    float age_factor = 1.0f;
    if (info.last_update_us > 0) {
        age_factor = std::exp(-static_cast<float>(now - info.last_update_us) / 1e7);
    }
    weight *= age_factor;
    
    // Activity boost
    weight *= (1.0f + 0.1f * std::min(info.cells_contributed, 100));
    
    return weight;
}

float MapFusion::checkCellConsistency(const SwarmMap& map1, const SwarmMap& map2, size_t index) const {
    float occ_diff = std::abs(map1.occupancy_probability[index] - map2.occupancy_probability[index]);
    
    if (map1.semantic_class[index] == map2.semantic_class[index]) {
        float conf_diff = std::abs(map1.semantic_confidence[index] - map2.semantic_confidence[index]);
        return 1.0f - (occ_diff + conf_diff) / 2.0f;
    } else {
        // Different semantic class
        return 1.0f - occ_diff - 0.5f;
    }
}

float MapFusion::resolveConflict(float target_val, float source_val, 
                                 float target_weight, float source_weight) {
    if (target_weight + source_weight == 0) return target_val;
    return (target_val * target_weight + source_val * source_weight) / (target_weight + source_weight);
}

void MapFusion::updateContributor(uint16_t drone_id, uint64_t timestamp_us, int cells_added) {
    auto& info = _contributors[drone_id];
    info.drone_id = drone_id;
    info.last_update_us = timestamp_us;
    info.cells_contributed += cells_added;
    info.weight = calculateContributorWeight(info);
}

} // namespace swarm::autonomy