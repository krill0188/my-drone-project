#include "distributed_sensing.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace swarm::autonomy {

DistributedSensing::DistributedSensing(const Config& config) : _config(config) {}

std::vector<Task> DistributedSensing::planSensingMission(const Mission& mission,
                                                          const std::vector<CapabilityBitmap>& drone_capabilities,
                                                          const SwarmMap* map) {
    std::vector<Task> tasks;
    
    if (!mission.constraints.geofence.isValid()) {
        return tasks;
    }
    
    // Default sensing config
    SensingConfig sensing_config;
    sensing_config.type = 0;  // Visual
    sensing_config.resolution_mpp = 0.01f;
    sensing_config.min_altitude = _config.min_sensing_altitude;
    sensing_config.max_altitude = _config.max_sensing_altitude;
    sensing_config.required_overlap = _config.min_overlap;
    
    // Generate sensing grid
    std::vector<SensingCell> cells = generateSensingGrid(mission.constraints.geofence.boundary,
                                                          sensing_config, map);
    
    // Assign cells to drones
    assignCellsToDrones(cells, drone_capabilities);
    
    // Generate tasks
    std::vector<Task> tasks = generateCellTasks(cells, mission);
    
    return tasks;
}

std::vector<DistributedSensing::SensingCell> DistributedSensing::generateSensingGrid(
    const Polygon& area, const SensingConfig& config, const SwarmMap* map) {
    
    std::vector<SensingCell> cells;
    std::vector<Vector3> centers = generateGridCenters(area, _config.grid_resolution);
    
    for (const auto& center : centers) {
        SensingCell cell;
        cell.id = generateCellId();
        cell.center = center;
        cell.bounds = Polygon{};  // Would be set properly
        cell.priority = 1.0f;
        cell.required_sensing = SemanticClass::UNKNOWN;
        cell.completed = false;
        cell.coverage = 0.0f;
        cell.required_sensing = static_cast<SemanticClass>(config.type);
        
        cells.push_back(std::move(cell));
    }
    
    return cells;
}

bool DistributedSensing::assignCellsToDrones(std::vector<SensingCell>& cells,
                                             const std::vector<CapabilityBitmap>& drone_capabilities) {
    if (drone_capabilities.empty()) return false;
    
    // Sort cells by priority
    std::sort(cells.begin(), cells.end(),
        [](const SensingCell& a, const SensingCell& b) {
            return a.priority > b.priority;
        });
    
    // Assign cells to drones based on capability
    for (auto& cell : cells) {
        std::vector<uint16_t> capable_drones;
        
        for (size_t i = 0; i < drone_capabilities.size(); ++i) {
            if (droneCanSenseCell(drone_capabilities[i], cell)) {
                capable_drones.push_back(static_cast<uint16_t>(i + 1));
            }
        }
        
        if (!capable_drones.empty()) {
            // Assign to first capable drone (or distribute)
            cell.assigned_drones.push_back(capable_drones[0]);
        }
    }
    
    return true;
}

std::vector<Task> DistributedSensing::generateCellTasks(const std::vector<SensingCell>& cells,
                                                         const Mission& mission) {
    std::vector<Task> tasks;
    
    for (const auto& cell : cells) {
        for (uint16_t drone_id : cell.assigned_drones) {
            Task task;
            task.id = generateCellId();
            task.mission_id = mission.id;
            task.type = TaskType::COVER_AREA;
            task.priority = static_cast<uint8_t>(cell.priority * 100);
            
            task.geometry.type = TaskType::COVER_AREA;
            task.geometry.area = cell.bounds;
            
            task.success_criteria.min_coverage_pct = 0.95f;
            task.success_criteria.max_duration_s = 1800;
            task.success_criteria.require_data_return = true;
            
            task.assigned_drone = cell.assigned_drones[0];
            tasks.push_back(std::move(task));
        }
    }
    
    return tasks;
}

bool DistributedSensing::droneCanSenseCell(const CapabilityBitmap& caps, const SensingCell& cell) const {
    // Check if drone has required sensors
    if (cell.required_sensing == SemanticClass::OBSTACLE) {
        if (!caps.has_stereo_camera && !caps.has_lidar_2d && !caps.has_lidar_3d) {
            return false;
        }
    }
    
    if (cell.required_sensing == SemanticClass::LANDING_PAD) {
        if (!caps.has_stereo_camera) return false;
    }
    
    if (cell.required_sensing == SemanticClass::VEGETATION) {
        if (!caps.has_stereo_camera && !caps.has_lidar_2d) return false;
    }
    
    return true;
}

float DistributedSensing::calculateCellCoverage(const SensingCell& cell,
                                                 const std::vector<CapabilityBitmap>& assigned_caps) const {
    if (assigned_caps.empty()) return 0.0f;
    
    float base_coverage = 1.0f / assigned_caps.size();
    float sensor_quality = 1.0f;
    
    for (const auto& caps : assigned_caps) {
        if (caps.has_lidar_3d) sensor_quality = std::max(sensor_quality, 1.0f);
        else if (caps.has_stereo_camera) sensor_quality = std::max(sensor_quality, 0.9f);
        else if (caps.has_lidar_2d) sensor_quality = std::max(sensor_quality, 0.7f);
        else sensor_quality = std::max(sensor_quality, 0.5f);
    }
    
    return base_coverage * sensor_quality * assigned_caps.size();
}

DistributedSensing::GridStats DistributedSensing::getGridStats(const std::vector<SensingCell>& cells) const {
    GridStats stats;
    stats.total_cells = cells.size();
    
    for (const auto& cell : cells) {
        if (cell.completed) stats.completed_cells++;
        else if (cell.coverage > 0.5f) stats.in_progress_cells++;
        else stats.pending_cells++;
    }
    
    stats.overall_coverage = static_cast<float>(stats.completed_cells) / stats.total_cells;
    stats.avg_cell_coverage = 0.0f;
    for (const auto& cell : cells) {
        stats.avg_cell_coverage += cell.coverage;
    }
    if (!cells.empty()) stats.avg_cell_coverage /= cells.size();
    
    return stats;
}

void DistributedSensing::reprioritizeCells(std::vector<SensingCell>& cells,
                                           const AIInferenceResult& inference) {
    // Boost priority of cells near detected obstacles/threats
    for (auto& cell : cells) {
        if (inference.type == AIInferenceType::OBSTACLE) {
            float dx = cell.center.x - inference.position_enu.x;
            float dy = cell.center.y - inference.position_enu.y;
            float dz = cell.center.z - inference.position_enu.z;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (dist < 200.0f) {  // Within 200m
                cell.priority *= 2.0f;
            }
        }
    }
}

void DistributedSensing::handleDroneDropout(std::vector<SensingCell>& cells, uint16_t drone_id) {
    for (auto& cell : cells) {
        auto it = std::find(cell.assigned_drones.begin(), cell.assigned_drones.end(), drone_id);
        if (it != cell.assigned_drones.end()) {
            cell.assigned_drones.erase(it);
            cell.completed = false;  // Mark for reallocation
        }
    }
}

void DistributedSensing::replanIncompleteCells(std::vector<SensingCell>& cells,
                                               const std::vector<CapabilityBitmap>& drone_capabilities) {
    // Reassign incomplete cells
    for (auto& cell : cells) {
        if (!cell.completed && cell.assigned_drones.empty()) {
            assignCellsToDrones({cell}, drone_capabilities);
        }
    }
}

std::array<uint8_t, 16> DistributedSensing::generateCellId() {
    std::array<uint8_t, 16> id;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);
    for (auto& b : id) b = dis(gen);
    return id;
}

std::vector<Vector3> DistributedSensing::generateGridCenters(const Polygon& area, float resolution) {
    std::vector<Vector3> centers;
    
    Polygon bbox;
    if (area.vertices.empty()) return centers;
    
    bbox.min = area.vertices[0];
    bbox.max = area.vertices[0];
    for (const auto& v : area.vertices) {
        bbox.min.x = std::min(bbox.min.x, v.x);
        bbox.min.y = std::min(bbox.min.y, v.y);
        bbox.max.x = std::max(bbox.max.x, v.x);
        bbox.max.y = std::max(bbox.max.y, v.y);
    }
    
    for (float x = bbox.min.x; x <= bbox.max.x; x += resolution) {
        for (float y = bbox.min.y; y <= bbox.max.y; y += resolution) {
            // Check if point is in polygon
            Vector3 p{x, y, 50.0f};
            // Simplified: just add all grid points
            centers.push_back(p);
        }
    }
    
    return centers;
}

bool DistributedSensing::cellInArea(const SensingCell& cell, const Polygon& area) const {
    // Simplified
    return true;
}

bool DistributedSensing::droneCanSenseCell(const CapabilityBitmap& caps, const SensingCell& cell) const {
    return droneCanSenseCell(caps, cell);
}

} // namespace swarm::autonomy