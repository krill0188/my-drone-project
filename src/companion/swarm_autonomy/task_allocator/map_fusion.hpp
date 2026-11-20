#ifndef MAP_FUSION_HPP
#define MAP_FUSION_HPP

#include "swarm_map.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>

namespace swarm::autonomy {

// Swarm map fusion using Bayesian update with contributor weighting
class MapFusion {
public:
    struct Config {
        float contributor_weight_base = 1.0f;      // Base weight for contributors
        float sensor_quality_weight = 2.0f;        // Weight for sensor quality
        float recency_weight = 1.5f;               // Weight for data freshness
        float consistency_threshold = 0.3f;        // Max allowed inconsistency
        uint64_t max_cell_age_us = 30000000;       // 30 seconds max cell age
        float decay_rate = 0.1f;                   // Decay rate for stale cells
        int max_contributors_per_cell = 10;        // Max contributors to track per cell
    };
    
    struct ContributorInfo {
        uint16_t drone_id;
        float weight;
        uint64_t last_update_us;
        int cells_contributed;
    };
    
    struct FusionResult {
        bool success;
        int cells_updated;
        int cells_conflicted;
        int contributors_merged;
        float avg_consistency;
        std::vector<uint16_t> new_contributors;
    };
    
    MapFusion(const Config& config = Config{});
    
    // Fuse a source map into the target map
    FusionResult fuse(const SwarmMap& source, SwarmMap& target);
    
    // Fuse multiple maps
    FusionResult fuseMultiple(const std::vector<SwarmMap*>& sources, SwarmMap& target);
    
    // Fuse occupancy grid only
    bool fuseOccupancy(const SwarmMap& source, SwarmMap& target);
    
    // Fuse semantic labels
    bool fuseSemantic(const SwarmMap& source, SwarmMap& target);
    
    // Fuse no-fly zones
    bool fuseNoFlyZones(const SwarmMap& source, SwarmMap& target);
    
    // Fuse landmarks
    bool fuseLandmarks(const SwarmMap& source, SwarmMap& target);
    
    // Get contributor weights
    std::unordered_map<uint16_t, float> getContributorWeights(const SwarmMap& map) const;
    
    // Decay stale cells
    void decayStaleCells(SwarmMap& map);
    
    // Validate map consistency
    bool validateConsistency(const SwarmMap& map) const;
    
    // Get fusion statistics
    struct Stats {
        int total_fusions = 0;
        int successful_fusions = 0;
        int failed_fusions = 0;
        int total_cells_updated = 0;
        int total_conflicts_resolved = 0;
        float avg_consistency = 0.0f;
    };
    
    const Stats& getStats() const { return _stats; }
    
    void setContributorWeight(uint16_t drone_id, float weight);
    float getContributorWeight(uint16_t drone_id) const;

private:
    Config _config;
    std::unordered_map<uint16_t, ContributorInfo> _contributors;
    std::unordered_map<uint16_t, float> _custom_weights;
    mutable std::mutex _mutex;
    
    Stats _stats;
    
    // Bayesian occupancy update
    float bayesianUpdate(float prior, float sensor_prob, float sensor_reliability, 
                         int hits, int misses);
    
    // Semantic label fusion
    bool fuseSemanticCell(uint8_t& target_class, float& target_confidence,
                          uint8_t source_class, float source_confidence,
                          uint16_t source_drone, uint64_t timestamp_us);
    
    // No-fly zone fusion
    bool fuseNoFlyZone(NoFlyZone& target, const NoFlyZone& source);
    
    // Landmark fusion
    bool fuseLandmark(Landmark& target, const Landmark& source);
    
    // Contributor weight calculation
    float calculateContributorWeight(const ContributorInfo& info) const;
    
    // Consistency check
    float checkCellConsistency(const SwarmMap& map1, const SwarmMap& map2, 
                               size_t index) const;
    
    // Conflict resolution
    float resolveConflict(float target_val, float source_val, 
                          float target_weight, float source_weight);
    
    // Update contributor info
    void updateContributor(uint16_t drone_id, uint64_t timestamp_us, int cells_added);
};

} // namespace swarm::autonomy

#endif // MAP_FUSION_HPP