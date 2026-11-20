#ifndef TASK_HPP
#define TASK_HPP

#include <vector>
#include <array>
#include <string>
#include <optional>
#include <cstdint>
#include "geometry.hpp"

namespace swarm::autonomy {

// Task types
enum class TaskType : uint8_t {
    COVER_AREA = 0,
    INSPECT_POI = 1,
    RELAY_COMMS = 2,
    HOLD_POSITION = 3,
    SCOUT = 4,
    ESCORT = 5
};

// Task status
enum class TaskStatus : uint8_t {
    PENDING = 0,
    ALLOCATED = 1,
    EXECUTING = 2,
    COMPLETED = 3,
    FAILED = 4,
    REEVALUATING = 5
};

// Search pattern for COVER_AREA tasks
enum class SearchPattern : uint8_t {
    LAWNMOWER = 0,
    SPIRAL = 1,
    ADAPTIVE = 2
};

// Inspection type for INSPECT_POI tasks
enum class InspectionType : uint8_t {
    VISUAL = 0,
    THERMAL = 1,
    LIDAR = 2,
    MULTISPECTRAL = 3
};

// Task geometry (variant based on type)
struct TaskGeometry {
    TaskType type;
    
    // COVER_AREA: search polygon with pattern
    std::optional<Polygon> area;
    std::optional<SearchPattern> pattern;
    
    // INSPECT_POI: 3D point + view angles
    std::optional<Vector3> poi_position;
    std::optional<Vector3> view_direction;
    std::optional<InspectionType> inspection_type;
    std::optional<float> required_resolution_mpp;
    std::optional<float> inspection_distance_m;
    
    // HOLD_POSITION: 3D point + radius
    std::optional<Vector3> hold_position;
    std::optional<float> hold_radius;
    
    // RELAY_COMMS: optimal link position
    std::optional<Vector3> relay_position;
    
    // ESCORT/SCOUT: waypoint sequence
    std::vector<Vector3> waypoints;
    
    // Validation
    bool validate() const;
};

// Success criteria (measurable, technology-agnostic)
struct SuccessCriteria {
    // COVER_AREA
    float min_coverage_pct = 0.95f;       // ≥ 95% (FR-005, SC-002)
    
    // INSPECT_POI
    float max_inspection_dist_m = 5.0f;   // ≤ 5m
    float required_resolution_mpp = 0.01f; // m/pixel
    
    // HOLD_POSITION
    float max_hold_error_m = 2.0f;        // ≤ 2m
    
    // General
    uint32_t max_duration_s = 3600;       // Time budget
    bool require_data_return = true;      // Sensor data uploaded
    float min_data_quality = 0.8f;        // Quality threshold
    
    bool check(const Task& task) const;
};

// Task entity
class Task {
public:
    Task() = default;
    
    // Identification
    std::array<uint8_t, 16> id;           // UUID (RFC 4122)
    std::array<uint8_t, 16> mission_id;   // Parent mission
    TaskType type = TaskType::COVER_AREA;
    uint8_t priority = 128;                // 1=highest
    
    // Geometry
    TaskGeometry geometry;
    
    // Dependencies
    std::vector<std::array<uint8_t, 16>> dependencies;  // Precedence constraints
    
    // Allocation
    std::optional<uint16_t> assigned_drone;  // Drone sysid if allocated
    
    // Status
    TaskStatus status = TaskStatus::PENDING;
    float progress = 0.0f;                 // 0.0 - 1.0
    uint64_t created_at = 0;
    uint64_t started_at = 0;
    uint64_t completed_at = 0;
    
    // Success criteria
    SuccessCriteria success_criteria;
    
    // Methods
    bool validate() const;
    bool isReady() const;                  // All dependencies completed
    bool isAllocatable() const;            // Ready and unallocated
    bool isComplete() const;
    bool isFailed() const;
    
    void start(uint16_t drone_id);
    void complete();
    void fail(const std::string& reason);
    void requeue();
    void updateProgress(float p);
    
    // Dependency management
    void addDependency(const std::array<uint8_t, 16>& task_id);
    void removeDependency(const std::array<uint8_t, 16>& task_id);
    bool hasDependency(const std::array<uint8_t, 16>& task_id) const;
    std::vector<std::array<uint8_t, 16>> getDependencies() const;
    
    // Geometry helpers
    std::optional<Polygon> getArea() const;
    std::optional<Vector3> getPOI() const;
    std::optional<Vector3> getHoldPosition() const;
    std::optional<Vector3> getRelayPosition() const;
    const std::vector<Vector3>& getWaypoints() const;
    
    // Serialization
    std::string toJson() const;
    static std::optional<Task> fromJson(const std::string& json);
    std::string toYaml() const;
    static std::optional<Task> fromYaml(const std::string& yaml);
    
    // Geometry validation
    static bool validateGeometry(const TaskGeometry& geometry);
    
private:
    bool checkGeometryMatchesType() const;
};

} // namespace swarm::autonomy

#endif // TASK_HPP