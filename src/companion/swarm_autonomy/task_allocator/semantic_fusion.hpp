#ifndef SEMANTIC_FUSION_HPP
#define SEMANTIC_FUSION_HPP

#include "swarm_map.hpp"
#include "ai_inference.hpp"
#include <vector>
#include <unordered_map>
#include <mutex>

namespace swarm::autonomy {

// Semantic label fusion from AI inference and sensor data
class SemanticFusion {
public:
    struct Config {
        float ai_confidence_threshold = 0.7f;    // Min AI confidence to act
        float sensor_confidence_weight = 1.0f;   // Weight for sensor data
        float ai_confidence_weight = 2.0f;       // Weight for AI inference
        float temporal_decay_rate = 0.1f;        // Decay per second
        float spatial_consistency_radius = 5.0f; // Spatial consistency check radius
        int max_history_per_cell = 10;           // Max history entries per cell
        float conflict_resolution_threshold = 0.3f; // Confidence diff to override
    };
    
    struct SemanticObservation {
        uint64_t timestamp_us;
        uint16_t source_drone;
        Vector3 position_enu;
        SemanticClass semantic_class;
        float confidence;
        bool from_ai;  // true = AI inference, false = sensor
        std::string model_name;  // For AI inferences
    };
    
    struct FusionResult {
        int cells_updated = 0;
        int conflicts_resolved = 0;
        int ai_inferences_fused = 0;
        int sensor_readings_fused = 0;
        float avg_confidence = 0.0f;
    };
    
    SemanticFusion(const Config& config = Config{});
    
    // Fuse AI inference result into map
    bool fuseAIInference(const AIInferenceResult& inference, SwarmMap& map);
    
    // Fuse sensor reading into map
    bool fuseSensorReading(const SemanticObservation& observation, SwarmMap& map);
    
    // Batch fuse multiple observations
    FusionResult fuseObservations(const std::vector<SemanticObservation>& observations, SwarmMap& map);
    
    // Resolve conflicts in map
    int resolveConflicts(SwarmMap& map);
    
    // Decay old observations
    void decayObservations(SwarmMap& map, uint64_t current_time_us);
    
    // Get semantic stats for a region
    struct RegionStats {
        std::unordered_map<SemanticClass, int> class_counts;
        float avg_confidence = 0.0f;
        int total_cells = 0;
    };
    
    RegionStats getRegionStats(const SwarmMap& map, const Polygon& region) const;
    
    // Set class-specific weights
    void setClassWeight(SemanticClass cls, float weight);
    float getClassWeight(SemanticClass cls) const;
    
    // Set model-specific trust
    void setModelTrust(const std::string& model_name, float trust);
    float getModelTrust(const std::string& model_name) const;
    
    // Get fusion statistics
    struct Stats {
        int total_fusions = 0;
        int ai_fusions = 0;
        int sensor_fusions = 0;
        int conflicts_resolved = 0;
        float avg_confidence = 0.0f;
    };
    
    const Stats& getStats() const { return _stats; }
    
    // Export semantic map for visualization
    std::vector<SemanticObservation> exportObservations(const SwarmMap& map) const;

private:
    Config _config;
    std::unordered_map<SemanticClass, float> _class_weights;
    std::unordered_map<std::string, float> _model_trust;
    mutable std::mutex _mutex;
    
    Stats _stats;
    
    // Cell history for temporal fusion
    struct CellHistory {
        struct Entry {
            uint64_t timestamp_us;
            SemanticClass cls;
            float confidence;
            uint16_t source_drone;
            bool from_ai;
        };
        std::vector<Entry> entries;
    };
    
    std::unordered_map<size_t, CellHistory> _cell_histories;  // index -> history
    
    // Fuse single observation into cell
    bool fuseIntoCell(size_t index, const SemanticObservation& obs, SwarmMap& map);
    
    // Conflict resolution
    bool resolveConflict(size_t index, SwarmMap& map);
    
    // Temporal fusion
    bool temporalFusion(size_t index, const SemanticObservation& obs, SwarmMap& map);
    
    // Spatial consistency check
    bool checkSpatialConsistency(size_t index, SemanticClass cls, const SwarmMap& map);
    
    // Get cell index from position
    std::optional<size_t> getCellIndex(const Vector3& pos, const SwarmMap& map) const;
    
    // Decay function
    float decayConfidence(float confidence, uint64_t age_us) const;
};

} // namespace swarm::autonomy

#endif // SEMANTIC_FUSION_HPP