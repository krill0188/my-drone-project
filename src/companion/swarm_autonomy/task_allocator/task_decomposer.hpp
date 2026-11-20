#ifndef TASK_DECOMPOSER_HPP
#define TASK_DECOMPOSER_HPP

#include "task.hpp"
#include "mission.hpp"
#include "swarm_map.hpp"
#include <vector>
#include <array>
#include <cmath>

namespace swarm::autonomy {

// Task decomposer for breaking down high-level missions into atomic tasks
class TaskDecomposer {
public:
    struct Config {
        // Coverage pattern parameters
        float lawnmower_spacing_factor = 0.7f;    // Spacing as fraction of sensor footprint
        float spiral_spacing_factor = 0.8f;       // Spiral spacing factor
        float min_task_area_km2 = 0.01f;          // Minimum sub-task area (km²)
        float max_task_area_km2 = 1.0f;           // Maximum sub-task area (km²)
        float overlap_ratio = 0.1f;               // Overlap between adjacent tasks
        int max_subtasks_per_mission = 50;        // Max sub-tasks per mission
        
        // Perimeter surveillance
        float perimeter_waypoint_spacing = 50.0f;  // Waypoint spacing for perimeter
        
        // Distributed sensing
        float sensing_grid_resolution = 100.0f;    // Grid resolution for sensing tasks
        
        // Dependencies
        bool add_sequential_dependencies = true;   // Add sequential dependencies between tasks
    };
    
    TaskDecomposer(const Config& config = Config{});
    
    // Decompose search mission into coverage tasks
    std::vector<Task> decomposeSearchMission(const Mission& mission, 
                                             const std::vector<CapabilityBitmap>& drone_capabilities,
                                             const SwarmMap* map = nullptr);
    
    // Decompose perimeter surveillance mission
    std::vector<Task> decomposePerimeterMission(const Mission& mission,
                                                const std::vector<CapabilityBitmap>& drone_capabilities,
                                                const SwarmMap* map = nullptr);
    
    // Decompose distributed sensing mission
    std::vector<Task> decomposeDistributedSensingMission(const Mission& mission,
                                                         const std::vector<CapabilityBitmap>& drone_capabilities,
                                                         const SwarmMap* map = nullptr);
    
    // Decompose inspection mission
    std::vector<Task> decomposeInspectionMission(const Mission& mission,
                                                 const std::vector<CapabilityBitmap>& drone_capabilities,
                                                 const SwarmMap* map = nullptr);
    
    // Generic decomposition entry point
    std::vector<Task> decomposeMission(const Mission& mission,
                                       const std::vector<CapabilityBitmap>& drone_capabilities,
                                       const SwarmMap* map = nullptr);

private:
    Config _config;
    
    // Coverage pattern generation
    std::vector<Polygon> generateLawnmowerPattern(const Polygon& area, float spacing, float overlap);
    std::vector<Polygon> generateSpiralPattern(const Polygon& area, float spacing, float overlap);
    std::vector<Polygon> generateAdaptivePattern(const Polygon& area, 
                                                  const SwarmMap* map,
                                                  float base_spacing);
    
    // Task creation helpers
    Task createCoverAreaTask(const Polygon& sub_area, 
                            const SearchPattern pattern,
                            const Mission& parent_mission,
                            int task_index);
    
    Task createInspectPOITask(const Vector3& poi_position,
                              const Vector3& view_direction,
                              InspectionType type,
                              const Mission& parent_mission,
                              int task_index);
    
    Task createRelayTask(const Vector3& relay_position,
                         const Mission& parent_mission,
                         int task_index);
    
    Task createHoldPositionTask(const Vector3& hold_position,
                                float radius,
                                const Mission& parent_mission,
                                int task_index);
    
    Task createScoutTask(const std::vector<Vector3>& waypoints,
                         const Mission& parent_mission,
                         int task_index);
    
    // Dependency management
    void addSequentialDependencies(std::vector<Task>& tasks);
    void addSpatialDependencies(std::vector<Task>& tasks, const SwarmMap* map);
    
    // Utility functions
    float estimateCoverageTime(const Polygon& area, const CapabilityBitmap& caps);
    float calculateSpacing(const CapabilityBitmap& caps, float sensor_footprint);
    std::vector<Polygon> partitionPolygon(const Polygon& area, int num_parts);
    Polygon computeBoundingBox(const Polygon& polygon);
    Vector3 computeCentroid(const Polygon& polygon);
    float polygonArea(const Polygon& polygon);
    
    // Validation
    bool validateDecomposition(const std::vector<Task>& tasks, const Mission& mission);
    
    // Utility
    std::array<uint8_t, 16> generateTaskId();
    Vector3 polygonCentroid(const Polygon& polygon);
    float polygonAreaKm2(const Polygon& polygon);
};

} // namespace swarm::autonomy

#endif // TASK_DECOMPOSER_HPP