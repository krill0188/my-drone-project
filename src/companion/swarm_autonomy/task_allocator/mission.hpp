#ifndef MISSION_HPP
#define MISSION_HPP

#include <vector>
#include <array>
#include <string>
#include <optional>
#include <cstdint>
#include "task.hpp"
#include "swarm_map.hpp"
#include "geometry.hpp"

namespace swarm::autonomy {

// Mission types
enum class MissionType : uint8_t {
    WAYPOINT = 0,
    SEARCH = 1,
    INSPECT = 2,
    PATROL = 3,
    ESCORT = 4
};

// Mission status
enum class MissionStatus : uint8_t {
    PLANNED = 0,
    UPLOADED = 1,
    ACTIVE = 2,
    PAUSED = 3,
    COMPLETED = 4,
    FAILED = 5,
    ABORTED = 6
};

// Weather constraints
struct WeatherConstraints {
    float max_wind_speed_mps = 10.0f;
    bool precipitation_allowed = false;
    float min_visibility_m = 1000.0f;
    float max_temperature_c = 50.0f;
    float min_temperature_c = -20.0f;
};

// Jurisdiction for regulatory compliance
struct Jurisdiction {
    std::string country_code;      // ISO 3166-1 alpha-2
    std::string region_code;       // State/province
    std::string authority;         // e.g., "EASA", "FAA"
    std::string regulation_basis;  // e.g., "SC-VTOL", "Part 107"
    uint8_t max_altitude_m = 120;  // Regulatory altitude limit
    bool bvlos_allowed = false;
    bool night_flight_allowed = false;
};

// No-fly zone definition
struct NoFlyZone {
    std::array<uint8_t, 16> id;           // UUID
    Polygon boundary;                      // 2D polygon in ENU
    float min_altitude_m = 0.0f;
    float max_altitude_m = 10000.0f;
    uint64_t valid_from_us = 0;
    uint64_t valid_until_us = 0;
    std::string authority;                 // Regulatory authority
    std::string signature;                 // Cryptographic signature
    uint8_t zone_type = 0;                 // 0=permanent, 1=temporary, 2=emergency
};

struct MissionConstraints {
    Polygon geofence;                      // Hard boundary (FR-011)
    float max_altitude_m = 120.0f;         // AMSL
    float min_altitude_m = 10.0f;          // AGL
    float max_speed_mps = 15.0f;
    uint32_t max_duration_s = 3600;        // Time budget
    float min_battery_pct = 0.20f;         // 20% reserve
    std::vector<NoFlyZone> no_fly_zones;   // Dynamic updates
    WeatherConstraints weather;
    bool require_rid = true;               // Remote ID mandatory
    Jurisdiction jurisdiction;             // Regulatory domain
};

// Mission entity
class Mission {
public:
    Mission() = default;
    
    // Identification
    std::array<uint8_t, 16> id;            // UUID (RFC 4122)
    std::string name;
    MissionType type = MissionType::WAYPOINT;
    uint8_t priority = 128;                 // 1=highest, 255=lowest
    
    // Constraints
    MissionConstraints constraints;
    
    // Decomposed tasks
    std::vector<Task> tasks;
    
    // Assigned swarm
    std::vector<uint16_t> assigned_swarm;  // Drone sysids
    
    // Status
    MissionStatus status = MissionStatus::PLANNED;
    float progress = 0.0f;                  // 0.0 - 1.0
    
    // Metadata
    std::string created_by;                 // Operator ID
    uint64_t created_at = 0;                // Unix ms
    uint64_t started_at = 0;
    uint64_t completed_at = 0;
    
    // Methods
    bool validate() const;
    bool isGeofenceCompliant() const;
    bool isTimeFeasible() const;
    bool isEnergyFeasible(const std::vector<uint16_t>& drone_ids) const;
    void updateProgress();
    std::optional<Task&> getNextPendingTask();
    std::vector<Task*> getTasksByDrone(uint16_t drone_id);
    std::vector<Task*> getTasksByStatus(uint8_t status);
    
    // Serialization
    std::string toJson() const;
    static std::optional<Mission> fromJson(const std::string& json);
    
    // YAML serialization
    std::string toYaml() const;
    static std::optional<Mission> fromYaml(const std::string& yaml);
    
private:
    bool checkTaskDependencies() const;
    float calculateTotalDistance() const;
    uint32_t calculateTotalTime() const;
};

} // namespace swarm::autonomy

#endif // MISSION_HPP