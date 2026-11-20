#ifndef TASK_ALLOCATOR_HPP
#define TASK_ALLOCATOR_HPP

#include <cstdint>
#include <vector>
#include <array>
#include <string>
#include <optional>
#include <functional>
#include <unordered_map>
#include "swarm_msgs/msg/task_allocation.hpp"
#include "swarm_msgs/msg/agent_state.hpp"
#include "swarm_msgs/msg/swarm_map.hpp"

namespace swarm::autonomy {

// Task types
enum class TaskType : uint8_t {
    COVER_AREA = 0,
    INSPECT_POI = 1,
    RELAY_COMMS = 2,
    HOLD_POSITION = 3,
    SCOUT = 4,
    ESCORT = 5,
};

struct Vector3 {
    float x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Polygon {
    std::vector<Vector3> vertices;
};

struct TaskGeometry {
    TaskType type;
    std::optional<Polygon> area;           // COVER_AREA
    std::optional<Vector3> poi_position;   // INSPECT_POI
    std::optional<Vector3> hold_position;  // HOLD_POSITION
    std::optional<Vector3> relay_position; // RELAY_COMMS
    std::vector<Vector3> waypoints;        // ESCORT/SCOUT
};

struct CapabilityBitmap {
    bool has_rtk_gps = false;
    bool has_stereo_camera = false;
    bool has_lidar_2d = false;
    bool has_lidar_3d = false;
    bool has_uwb = false;
    bool has_5g_modem = false;
    bool has_ai_compute = false;
    uint16_t max_flight_time_s = 0;
    float max_payload_kg = 0.0f;
    uint8_t comm_tier_bitmap = 0;
};

struct Task {
    std::array<uint8_t, 16> id;           // UUID
    TaskType type;
    TaskGeometry geometry;
    std::vector<std::array<uint8_t, 16>> dependencies;
    std::optional<uint16_t> assigned_drone;
    uint8_t priority = 128;
    uint8_t status = 0;  // 0=PENDING, 1=ALLOCATED, 2=EXECUTING, 3=COMPLETED, 4=FAILED, 5=REEVALUATING
    float progress = 0.0f;
    uint64_t created_at = 0;
    uint64_t started_at = 0;
    uint64_t completed_at = 0;
};

struct Mission {
    std::array<uint8_t, 16> id;
    uint8_t type;  // 0=WAYPOINT, 1=SEARCH, 2=INSPECT, 3=PATROL, 4=ESCORT
    std::vector<Task> tasks;
    uint8_t priority = 128;
    uint8_t status = 0;
    std::vector<uint16_t> assigned_swarm;
    uint64_t created_at = 0;
};

// CBBA Algorithm Implementation
class CbbaAllocator {
public:
    struct Bid {
        uint16_t drone_id;
        std::array<uint8_t, 16> task_id;
        float cost;
        CapabilityBitmap capabilities;
        uint32_t sequence;
    };
    
    struct Bundle {
        std::vector<std::array<uint8_t, 16>> tasks;
        std::vector<float> costs;
        std::vector<uint32_t> timestamps;
    };
    
    struct CBBAState {
        Bundle bundle;
        std::unordered_map<uint16_t, Bundle> neighbor_bundles;
        uint32_t iteration = 0;
        bool converged = false;
    };
    
    CbbaAllocator(uint16_t own_id, const CapabilityBitmap& capabilities)
        : _own_id(own_id), _capabilities(capabilities) {}
    
    // Main CBBA functions
    void init();
    
    // Phase 1: Bundle building (local)
    void buildBundle(const std::vector<Task>& available_tasks);
    
    // Phase 2: Consensus (communicate with neighbors)
    void communicateBundles();
    
    // Process incoming bid
    void processBid(const Bid& bid);
    
    // Check convergence
    bool checkConvergence();
    
    // Get final allocation
    const CBBAState& getState() const { return _state; }
    const std::vector<std::array<uint8_t, 16>>& getAllocatedTasks() const { return _state.bundle.tasks; }
    
    // Handle drone dropout
    void handleDroneDropout(uint16_t dropped_drone_id);
    
    // Re-evaluate tasks
    void reevaluateTasks(const std::vector<Task>& all_tasks);

private:
    uint16_t _own_id;
    CapabilityBitmap _capabilities;
    CBBAState _state;
    
    // Cost function for task bidding
    float computeTaskCost(const Task& task, const swarm_msgs::msg::AgentState& drone_state) const;
    
    // Check if drone has required capabilities for task
    bool hasRequiredCapabilities(const Task& task) const;
    
    // Update bundle with winning bids
    void updateBundle(const Bid& bid);
    
    // Remove tasks from bundle
    void removeTasksFromBundle(const std::vector<std::array<uint8_t, 16>>& task_ids);
};

} // namespace swarm::autonomy

#endif // TASK_ALLOCATOR_HPP