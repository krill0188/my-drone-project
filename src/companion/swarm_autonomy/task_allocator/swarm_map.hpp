#ifndef SWARM_MAP_HPP
#define SWARM_MAP_HPP

#include <vector>
#include <array>
#include <string>
#include <optional>
#include <cstdint>
#include "geometry.hpp"

namespace swarm::autonomy {

// Semantic classes for map cells
enum class SemanticClass : uint8_t {
    UNKNOWN = 0,
    FREE = 1,
    OBSTACLE = 2,
    STRUCTURE = 3,
    VEGETATION = 4,
    WATER = 5,
    NO_FLY = 6,
    LANDING_PAD = 7
};

// Landmark types
enum class LandmarkType : uint8_t {
    VISUAL = 0,
    UWB_ANCHOR = 1,
    APRILTAG = 2,
    KNOWN_STRUCTURE = 3
};

// No-fly zone (runtime)
struct NoFlyZone {
    std::array<uint8_t, 16> id;          // UUID
    Polygon boundary;                     // 2D polygon in ENU
    float min_altitude_m = 0.0f;
    float max_altitude_m = 10000.0f;
    uint64_t valid_from_us = 0;
    uint64_t valid_until_us = 0;
    std::string authority;                // Regulatory authority
    std::string signature;                // Cryptographic signature
    uint8_t zone_type = 0;                // 0=permanent, 1=temporary, 2=emergency
    bool verified = false;                // Signature verified
};

// Landmark for absolute localization
struct Landmark {
    std::array<uint8_t, 16> id;           // UUID
    Vector3 position_enu;                 // ENU coordinates
    LandmarkType type;
    float confidence = 0.0f;              // 0.0 - 1.0
    uint64_t timestamp_us = 0;
    uint16_t source_drone = 0;            // Origin drone
    std::string descriptor;               // Visual descriptor hash
};

// Occupancy cell
struct OccupancyCell {
    float probability = 0.5f;             // 0.0 (free) - 1.0 (occupied)
    uint32_t hits = 0;
    uint32_t misses = 0;
    uint64_t last_update_us = 0;
    SemanticClass semantic = SemanticClass::UNKNOWN;
    float semantic_confidence = 0.0f;
    uint64_t semantic_timestamp_us = 0;
    uint16_t source_drone = 0;
};

// Swarm map entity
class SwarmMap {
public:
    SwarmMap() = default;
    
    // Map metadata
    uint64_t version = 0;                 // Monotonically increasing (vector clock)
    Vector3 origin;                       // ENU frame origin (mission home)
    float resolution_m = 0.5f;            // Grid cell size (0.1-1.0m)
    uint32_t grid_width = 0;
    uint32_t grid_height = 0;
    uint32_t grid_depth = 1;              // 2D=1, 3D>1
    
    // Occupancy grid (flattened 1D arrays)
    std::vector<float> occupancy_probability;   // 0.0=free, 1.0=occupied
    std::vector<uint32_t> occupancy_hits;
    std::vector<uint32_t> occupancy_misses;
    std::vector<uint64_t> occupancy_last_update_us;
    
    // Semantic labels (same dimensions)
    std::vector<uint8_t> semantic_class;
    std::vector<float> semantic_confidence;
    std::vector<uint64_t> semantic_timestamp_us;
    std::vector<uint16_t> semantic_source_drone;
    
    // No-fly zones
    std::vector<NoFlyZone> no_fly_zones;
    
    // Landmarks
    std::vector<Landmark> landmarks;
    
    // Contributors
    std::vector<uint16_t> contributors;   // Source drone sysids
    uint64_t last_update_us = 0;
    
    // Methods
    bool initialize(const Vector3& origin, float resolution, 
                    uint32_t width, uint32_t height, uint32_t depth = 1);
    
    // Cell access
    size_t getIndex(uint32_t x, uint32_t y, uint32_t z = 0) const;
    std::optional<uint32_t> getXY(uint64_t index) const;
    
    // Occupancy
    float getOccupancy(uint32_t x, uint32_t y, uint32_t z = 0) const;
    void setOccupancy(uint32_t x, uint32_t y, uint32_t z, float prob, uint16_t source);
    void updateOccupancyBayesian(uint32_t x, uint32_t y, uint32_t z, 
                                  float sensor_prob, float sensor_reliability, uint16_t source);
    
    // Semantic
    SemanticClass getSemantic(uint32_t x, uint32_t y, uint32_t z = 0) const;
    void setSemantic(uint32_t x, uint32_t y, uint32_t z, 
                     SemanticClass cls, float confidence, uint16_t source);
    
    // Fusion
    bool fuseMap(const SwarmMap& other, float contributor_weight = 1.0f);
    bool fuseOccupancy(const SwarmMap& other, float weight);
    bool fuseSemantic(const SwarmMap& other, float weight);
    
    // No-fly zones
    bool addNoFlyZone(const NoFlyZone& zone);
    bool removeNoFlyZone(const std::array<uint8_t, 16>& id);
    std::optional<const NoFlyZone*> getNoFlyZone(const std::array<uint8_t, 16>& id) const;
    bool checkNoFlyZone(const Vector3& position) const;
    bool checkNoFlyZone(const Polygon& polygon) const;
    bool verifyNoFlyZoneSignatures() const;
    
    // Landmarks
    bool addLandmark(const Landmark& landmark);
    bool removeLandmark(const std::array<uint8_t, 16>& id);
    std::optional<const Landmark*> getLandmark(const std::array<uint8_t, 16>& id) const;
    std::vector<const Landmark*> getLandmarksByType(LandmarkType type) const;
    std::vector<const Landmark*> getLandmarksNear(const Vector3& position, float radius) const;
    
    // Decay stale cells
    void decayStaleCells(uint64_t max_age_us = 30000000);  // 30 seconds default
    
    // Bounds checking
    bool isInBounds(uint32_t x, uint32_t y, uint32_t z = 0) const;
    Vector3 indexToENU(uint32_t x, uint32_t y, uint32_t z = 0) const;
    std::optional<std::array<uint32_t, 3>> enuToIndex(const Vector3& pos) const;
    
    // Serialization
    std::string toJson() const;
    static std::optional<SwarmMap> fromJson(const std::string& json);
    std::string toYaml() const;
    static std::optional<SwarmMap> fromYaml(const std::string& yaml);
    
    // Validation
    bool validate() const;
    size_t getTotalCells() const;
    float getMemoryUsageMB() const;
    
private:
    size_t computeIndex(uint32_t x, uint32_t y, uint32_t z) const;
};

} // namespace swarm::autonomy

#endif // SWARM_MAP_HPP