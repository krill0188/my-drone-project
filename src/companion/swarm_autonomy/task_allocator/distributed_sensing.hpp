#ifndef DISTRIBUTED_SENSING_HPP
#define DISTRIBUTED_SENSING_HPP

#include "task.hpp"
#include "swarm_map.hpp"
#include <vector>
#include <array>

namespace swarm::autonomy {

// Distributed sensing task allocation
class DistributedSensing {
public:
    struct Config {
        float grid_resolution = 100.0f;         // Grid cell size (m)
        float sensing_radius = 50.0f;           // Sensing radius per drone
        float min_overlap = 0.2f;               // Minimum overlap between cells
        float max_sensing_altitude = 100.0f;    // Max altitude for sensing
        float min_sensing_altitude = 20.0f;     // Min altitude for sensing
        int max_cells_per_drone = 10;           // Max grid cells per drone
        bool enable_adaptive_grid = true;       // Adapt grid to terrain
        float priority_weight = 1.0f;           // Priority weight for cells
    };
    
    struct SensingCell {
        std::array<uint8_t, 16> id;
        Vector3 center;
        Polygon bounds;
        float priority = 1.0f;
        SemanticClass required_sensing = SemanticClass::UNKNOWN;
        std::vector<uint16_t> assigned_drones;
        bool completed = false;
        float coverage = 0.0f;
    };
    
    struct SensingConfig {
        uint8_t type;  // 0=visual, 1=thermal, 2=lidar, 3=multispectral
        float resolution_mpp = 0.01f;       // Meters per pixel
        float min_altitude = 20.0f;
        float max_altitude = 100.0f;
        float required_overlap = 0.2f;
    };
    
    DistributedSensing(const Config& config = Config{});
    
    // Plan distributed sensing mission
    std::vector<Task> planSensingMission(const Mission& mission,
                                         const std::vector<CapabilityBitmap>& drone_capabilities,
                                         const SwarmMap* map = nullptr);
    
    // Generate sensing grid
    std::vector<SensingCell> generateSensingGrid(const Polygon& area,
                                                  const SensingConfig& config,
                                                  const SwarmMap* map = nullptr);
    
    // Assign cells to drones
    bool assignCellsToDrones(std::vector<SensingCell>& cells,
                             const std::vector<CapabilityBitmap>& drone_capabilities);
    
    // Generate tasks for cells
    std::vector<Task> generateCellTasks(const std::vector<SensingCell>& cells,
                                        const Mission& mission);
    
    // Handle dynamic reprioritization
    void reprioritizeCells(std::vector<SensingCell>& cells,
                           const AIInferenceResult& inference);
    
    // Handle drone dropout
    void handleDroneDropout(std::vector<SensingCell>& cells, uint16_t drone_id);
    
    // Get cell statistics
    struct GridStats {
        int total_cells = 0;
        int completed_cells = 0;
        int in_progress_cells = 0;
        int pending_cells = 0;
        float overall_coverage = 0.0f;
        float avg_cell_coverage = 0.0f;
    };
    
    GridStats getGridStats(const std::vector<SensingCell>& cells) const;
    
    // Replan incomplete cells
    void replanIncompleteCells(std::vector<SensingCell>& cells,
                               const std::vector<CapabilityBitmap>& drone_capabilities);

private:
    Config _config;
    
    std::array<uint8_t, 16> generateCellId();
    std::vector<Vector3> generateGridCenters(const Polygon& area, float resolution);
    bool cellInArea(const SensingCell& cell, const Polygon& area) const;
    bool droneCanSenseCell(const CapabilityBitmap& caps, const SensingCell& cell) const;
    float calculateCellCoverage(const SensingCell& cell,
                                const std::vector<CapabilityBitmap>& assigned_caps) const;
};

} // namespace swarm::autonomy

#endif // DISTRIBUTED_SENSING_HPP