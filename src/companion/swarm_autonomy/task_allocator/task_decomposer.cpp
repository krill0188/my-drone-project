#include "task_decomposer.hpp"
#include <algorithm>
#include <cmath>
#include <random>
#include <numeric>
#include <iostream>

namespace swarm::autonomy {

TaskDecomposer::TaskDecomposer(const Config& config) : _config(config) {}

std::vector<Task> TaskDecomposer::decomposeMission(const Mission& mission,
                                                    const std::vector<CapabilityBitmap>& drone_capabilities,
                                                    const SwarmMap* map) {
    switch (mission.type) {
        case MissionType::SEARCH:
            return decomposeSearchMission(mission, drone_capabilities, map);
        case MissionType::INSPECT:
            return decomposeInspectionMission(mission, drone_capabilities, map);
        case MissionType::PATROL:
            return decomposePerimeterMission(mission, drone_capabilities, map);
        case MissionType::WAYPOINT:
            // Waypoint missions are handled by formation controller
            return {};
        default:
            return {};
    }
}

std::vector<Task> TaskDecomposer::decomposeSearchMission(const Mission& mission,
                                                          const std::vector<CapabilityBitmap>& drone_capabilities,
                                                          const SwarmMap* map) {
    std::vector<Task> tasks;
    
    if (mission.tasks.empty()) {
        // No explicit tasks, decompose from mission constraints
        if (mission.constraints.geofence.isValid()) {
            // Decompose geofence area
            Polygon search_area = mission.constraints.geofence.boundary;
            
            // Calculate average sensor footprint
            float avg_footprint = 0.0f;
            int count = 0;
            for (const auto& caps : drone_capabilities) {
                if (caps.has_stereo_camera || caps.has_lidar_2d) {
                    avg_footprint += 50.0f;  // Typical 50m footprint
                    count++;
                }
            }
            if (count == 0) avg_footprint = 50.0f;
            else avg_footprint /= count;
            
            float spacing = calculateSpacing(drone_capabilities[0], avg_footprint);
            SearchPattern pattern = SearchPattern::LAWNMOWER;  // Default
            
            // Choose pattern based on area shape
            float area_km2 = polygonAreaKm2(mission.constraints.geofence.boundary);
            if (area_km2 < 0.1f) {
                pattern = SearchPattern::SPIRAL;
            }
            
            // Decompose area
            std::vector<Polygon> sub_areas;
            if (pattern == SearchPattern::LAWNMOWER) {
                sub_areas = generateLawnmowerPattern(mission.constraints.geofence.boundary, 
                                                     spacing, _config.overlap_ratio);
            } else if (pattern == SearchPattern::SPIRAL) {
                sub_areas = generateSpiralPattern(mission.constraints.geofence.boundary,
                                                  spacing, _config.overlap_ratio);
            } else {
                sub_areas = generateAdaptivePattern(mission.constraints.geofence.boundary, 
                                                    nullptr, spacing);
            }
            
            // Create tasks
            for (size_t i = 0; i < sub_areas.size() && i < _config.max_subtasks_per_mission; ++i) {
                Task task = createCoverAreaTask(sub_areas[i], pattern, mission, i);
                tasks.push_back(std::move(task));
            }
        }
    } else {
        // Mission already has tasks defined
        tasks = mission.tasks;
    }
    
    // Add dependencies
    if (_config.add_sequential_dependencies) {
        addSequentialDependencies(tasks);
    }
    
    // Validate
    if (!validateDecomposition(tasks, mission)) {
        std::cerr << "[TaskDecomposer] Warning: Decomposition validation failed" << std::endl;
    }
    
    return tasks;
}

std::vector<Task> TaskDecomposer::decomposePerimeterMission(const Mission& mission,
                                                             const std::vector<CapabilityBitmap>& drone_capabilities,
                                                             const SwarmMap* map) {
    std::vector<Task> tasks;
    
    // Decompose perimeter into waypoint segments
    if (mission.constraints.geofence.isValid()) {
        const Polygon& perimeter = mission.constraints.geofence.boundary;
        
        // Generate waypoints along perimeter
        std::vector<Vector3> waypoints;
        float spacing = _config.perimeter_waypoint_spacing;
        
        for (size_t i = 0; i < perimeter.vertices.size(); ++i) {
            const Vector3& v1 = perimeter.vertices[i];
            const Vector3& v2 = perimeter.vertices[(i + 1) % perimeter.vertices.size()];
            
            float edge_length = std::sqrt(
                (v2.x - v1.x) * (v2.x - v1.x) +
                (v2.y - v1.y) * (v2.y - v1.y)
            );
            
            int num_waypoints = std::max(1, static_cast<int>(edge_length / spacing));
            
            for (int j = 0; j < num_waypoints; ++j) {
                float t = static_cast<float>(j) / num_waypoints;
                Vector3 wp;
                wp.x = v1.x + t * (v2.x - v1.x);
                wp.y = v1.y + t * (v2.y - v1.y);
                wp.z = mission.constraints.max_altitude_m;
                waypoints.push_back(wp);
            }
        }
        
        // Create scout tasks for perimeter segments
        int segment_size = 5;  // Waypoints per scout task
        for (size_t i = 0; i < waypoints.size(); i += segment_size) {
            std::vector<Vector3> segment_waypoints;
            for (size_t j = i; j < std::min(i + segment_size, waypoints.size()); ++j) {
                segment_waypoints.push_back(waypoints[j]);
            }
            
            if (segment_waypoints.size() >= 2) {
                Task task = createScoutTask(segment_waypoints, Mission{}, tasks.size());
                tasks.push_back(std::move(task));
            }
        }
    }
    
    return tasks;
}

std::vector<Task> TaskDecomposer::decomposeDistributedSensingMission(const Mission& mission,
                                                                      const std::vector<CapabilityBitmap>& drone_capabilities,
                                                                      const SwarmMap* map) {
    std::vector<Task> tasks;
    
    // Create grid of sensing positions
    if (mission.constraints.geofence.isValid()) {
        const Polygon& area = mission.constraints.geofence.boundary;
        float resolution = _config.sensing_grid_resolution;
        
        Polygon bbox = computeBoundingBox(area);
        int grid_x = static_cast<int>((bbox.max.x - bbox.min.x) / resolution) + 1;
        int grid_y = static_cast<int>((bbox.max.y - bbox.min.y) / resolution) + 1;
        
        for (int i = 0; i < grid_x && tasks.size() < _config.max_subtasks_per_mission; ++i) {
            for (int j = 0; j < grid_y && tasks.size() < _config.max_subtasks_per_mission; ++j) {
                Vector3 pos;
                pos.x = bbox.min.x + i * resolution;
                pos.y = bbox.min.y + j * resolution;
                pos.z = mission.constraints.max_altitude_m;
                
                // Check if point is inside area
                // Simplified check
                
                Task task = createHoldPositionTask(pos, resolution / 2.0f, Mission{}, tasks.size());
                tasks.push_back(std::move(task));
            }
        }
    }
    
    return tasks;
}

std::vector<Task> TaskDecomposer::decomposeInspectionMission(const Mission& mission,
                                                              const std::vector<CapabilityBitmap>& drone_capabilities,
                                                              const SwarmMap* map) {
    std::vector<Task> tasks;
    
    // Inspection missions typically have explicit POIs
    for (size_t i = 0; i < mission.tasks.size(); ++i) {
        if (mission.tasks[i].type == TaskType::INSPECT_POI) {
            tasks.push_back(mission.tasks[i]);
        }
    }
    
    return tasks;
}

std::vector<Polygon> TaskDecomposer::generateLawnmowerPattern(const Polygon& area, 
                                                              float spacing, float overlap) {
    std::vector<Polygon> sub_areas;
    
    Polygon bbox = computeBoundingBox(area);
    float effective_spacing = spacing * (1.0f - overlap);
    
    int num_strips = static_cast<int>((bbox.max.y - bbox.min.y) / spacing) + 1;
    
    for (int i = 0; i < num_strips; ++i) {
        float y = bbox.min.y + i * spacing;
        float y_end = std::min(y + spacing, bbox.max.y);
        
        // Create strip polygon
        Polygon strip;
        strip.vertices = {
            {bbox.min.x, y, 0},
            {bbox.max.x, y, 0},
            {bbox.max.x, y_end, 0},
            {bbox.min.x, y_end, 0}
        };
        
        // Clip to area (simplified - would use polygon clipping in real implementation)
        sub_areas.push_back(strip);
    }
    
    return sub_areas;
}

std::vector<Polygon> TaskDecomposer::generateSpiralPattern(const Polygon& area,
                                                            float spacing, float overlap) {
    std::vector<Polygon> sub_areas;
    
    Vector3 center = polygonCentroid(area);
    Polygon bbox = computeBoundingBox(area);
    float max_radius = std::max(bbox.max.x - center.x, center.x - bbox.min.x);
    max_radius = std::max(max_radius, std::max(bbox.max.y - center.y, center.y - bbox.min.y));
    
    float current_radius = spacing;
    int spiral_arms = 4;
    
    while (current_radius < max_radius) {
        for (int arm = 0; arm < spiral_arms; ++arm) {
            float angle_start = (arm * 2 * M_PI / spiral_arms) + (current_radius / spacing) * 0.1f;
            float angle_end = angle_start + 2 * M_PI / spiral_arms;
            
            // Create spiral sector
            Polygon sector;
            int num_points = 20;
            sector.vertices.push_back(center);
            
            for (int i = 0; i <= num_points; ++i) {
                float t = static_cast<float>(i) / num_points;
                float angle = angle_start + t * (angle_end - angle_start);
                float r = current_radius * t + current_radius * (1 - t);
                
                Vector3 p;
                p.x = center.x + r * std::cos(angle);
                p.y = center.y + r * std::sin(angle);
                p.z = 0;
                sector.vertices.push_back(p);
            }
            
            sub_areas.push_back(sector);
        }
        
        current_radius += spacing * 2;
    }
    
    return sub_areas;
}

std::vector<Polygon> TaskDecomposer::generateAdaptivePattern(const Polygon& area,
                                                              const SwarmMap* map,
                                                              float base_spacing) {
    // For now, use lawnmower as default
    // In real implementation, would analyze area shape and map obstacles
    return generateLawnmowerPattern(area, base_spacing, _config.overlap_ratio);
}

Task TaskDecomposer::createCoverAreaTask(const Polygon& sub_area,
                                         const SearchPattern pattern,
                                         const Mission& parent_mission,
                                         int task_index) {
    Task task;
    task.id = generateTaskId();
    task.mission_id = parent_mission.id;
    task.type = TaskType::COVER_AREA;
    task.priority = 100 + task_index;
    
    task.geometry.type = TaskType::COVER_AREA;
    task.geometry.area = sub_area;
    task.geometry.pattern = pattern;
    
    task.success_criteria.min_coverage_pct = 0.95f;
    task.success_criteria.max_duration_s = 1800;  // 30 min default
    task.success_criteria.require_data_return = true;
    
    // Estimate duration
    float area_km2 = polygonAreaKm2(sub_area);
    task.success_criteria.max_duration_s = static_cast<uint32_t>(area_km2 * 300 + 300);  // 5 min per km² + 5 min
    
    return task;
}

Task TaskDecomposer::createInspectPOITask(const Vector3& poi_position,
                                           const Vector3& view_direction,
                                           InspectionType type,
                                           const Mission& parent_mission,
                                           int task_index) {
    Task task;
    task.id = generateTaskId();
    task.mission_id = parent_mission.id;
    task.type = TaskType::INSPECT_POI;
    task.priority = 50 + task_index;  // Higher priority for inspections
    
    task.geometry.type = TaskType::INSPECT_POI;
    task.geometry.poi_position = poi_position;
    task.geometry.view_direction = view_direction;
    task.geometry.inspection_type = type;
    
    task.success_criteria.max_inspection_dist_m = 5.0f;
    task.success_criteria.required_resolution_mpp = 0.01f;
    task.success_criteria.max_duration_s = 300;
    task.success_criteria.require_data_return = true;
    
    return task;
}

Task TaskDecomposer::createRelayTask(const Vector3& relay_position,
                                     const Mission& parent_mission,
                                     int task_index) {
    Task task;
    task.id = generateTaskId();
    task.mission_id = parent_mission.id;
    task.type = TaskType::RELAY_COMMS;
    task.priority = 150 + task_index;
    
    task.geometry.type = TaskType::RELAY_COMMS;
    task.geometry.relay_position = relay_position;
    
    task.success_criteria.max_hold_error_m = 5.0f;
    task.success_criteria.max_duration_s = 1800;
    task.success_criteria.require_data_return = false;
    
    return task;
}

Task TaskDecomposer::createHoldPositionTask(const Vector3& hold_position,
                                            float radius,
                                            const Mission& parent_mission,
                                            int task_index) {
    Task task;
    task.id = generateTaskId();
    task.mission_id = parent_mission.id;
    task.type = TaskType::HOLD_POSITION;
    task.priority = 200 + task_index;
    
    task.geometry.type = TaskType::HOLD_POSITION;
    task.geometry.hold_position = hold_position;
    
    task.success_criteria.max_hold_error_m = radius;
    task.success_criteria.max_duration_s = 3600;
    task.success_criteria.require_data_return = false;
    
    return task;
}

Task TaskDecomposer::createScoutTask(const std::vector<Vector3>& waypoints,
                                     const Mission& parent_mission,
                                     int task_index) {
    Task task;
    task.id = generateTaskId();
    task.mission_id = parent_mission.id;
    task.type = TaskType::SCOUT;
    task.priority = 80 + task_index;
    
    task.geometry.type = TaskType::SCOUT;
    task.geometry.waypoints = waypoints;
    
    task.success_criteria.max_duration_s = waypoints.size() * 120;  // 2 min per waypoint
    task.success_criteria.require_data_return = true;
    
    return task;
}

void TaskDecomposer::addSequentialDependencies(std::vector<Task>& tasks) {
    for (size_t i = 1; i < tasks.size(); ++i) {
        tasks[i].addDependency(tasks[i - 1].id);
    }
}

void TaskDecomposer::addSpatialDependencies(std::vector<Task>& tasks, const SwarmMap* map) {
    // Add dependencies based on spatial proximity
    // Tasks that are close should be done by different drones simultaneously
    // This is a simplified implementation
}

std::vector<Polygon> TaskDecomposer::partitionPolygon(const Polygon& area, int num_parts) {
    std::vector<Polygon> parts;
    // Simplified: just return the original area
    // Real implementation would use polygon partitioning algorithms
    parts.push_back(area);
    return parts;
}

Polygon TaskDecomposer::computeBoundingBox(const Polygon& polygon) {
    Polygon bbox;
    if (polygon.vertices.empty()) return bbox;
    
    bbox.min = polygon.vertices[0];
    bbox.max = polygon.vertices[0];
    
    for (const auto& v : polygon.vertices) {
        bbox.min.x = std::min(bbox.min.x, v.x);
        bbox.min.y = std::min(bbox.min.y, v.y);
        bbox.min.z = std::min(bbox.min.z, v.z);
        bbox.max.x = std::max(bbox.max.x, v.x);
        bbox.max.y = std::max(bbox.max.y, v.y);
        bbox.max.z = std::max(bbox.max.z, v.z);
    }
    
    return bbox;
}

Vector3 TaskDecomposer::computeCentroid(const Polygon& polygon) {
    return polygonCentroid(polygon);
}

float TaskDecomposer::polygonArea(const Polygon& polygon) {
    return polygonAreaKm2(polygon) * 1000000.0f;  // Convert km² to m²
}

float TaskDecomposer::estimateCoverageTime(const Polygon& area, const CapabilityBitmap& caps) {
    float area_km2 = polygonAreaKm2(area);
    float base_time = area_km2 * 300.0f;  // 5 min per km²
    
    if (caps.has_ai_compute) base_time *= 0.9f;
    if (caps.has_lidar_3d) base_time *= 0.85f;
    
    return base_time;
}

float TaskDecomposer::calculateSpacing(const CapabilityBitmap& caps, float sensor_footprint) {
    float spacing = sensor_footprint * _config.lawnmower_spacing_factor;
    
    if (caps.has_lidar_3d) spacing *= 1.2f;  // LiDAR can cover wider
    if (caps.has_stereo_camera) spacing *= 1.1f;
    
    return spacing;
}

bool TaskDecomposer::validateDecomposition(const std::vector<Task>& tasks, const Mission& mission) {
    if (tasks.empty()) return false;
    
    // Check all tasks have valid IDs
    for (const auto& task : tasks) {
        bool all_zero = true;
        for (uint8_t b : task.id) {
            if (b != 0) { all_zero = false; break; }
        }
        if (all_zero) return false;
    }
    
    // Check for duplicate IDs
    for (size_t i = 0; i < tasks.size(); ++i) {
        for (size_t j = i + 1; j < tasks.size(); ++j) {
            if (tasks[i].id == tasks[j].id) return false;
        }
    }
    
    // Check dependencies form DAG
    // Simplified: just check no self-dependencies
    for (const auto& task : tasks) {
        for (const auto& dep : task.dependencies) {
            if (dep == task.id) return false;
        }
    }
    
    return true;
}

std::array<uint8_t, 16> TaskDecomposer::generateTaskId() {
    std::array<uint8_t, 16> id;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);
    
    for (auto& b : id) {
        b = dis(gen);
    }
    return id;
}

Vector3 TaskDecomposer::polygonCentroid(const Polygon& polygon) {
    if (polygon.vertices.empty()) return {0, 0, 0};
    
    Vector3 centroid{0, 0, 0};
    for (const auto& v : polygon.vertices) {
        centroid.x += v.x;
        centroid.y += v.y;
        centroid.z += v.z;
    }
    centroid.x /= polygon.vertices.size();
    centroid.y /= polygon.vertices.size();
    centroid.z /= polygon.vertices.size();
    return centroid;
}

float TaskDecomposer::polygonAreaKm2(const Polygon& polygon) {
    if (polygon.vertices.size() < 3) return 0.0f;
    
    float area = 0.0f;
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        const auto& v1 = polygon.vertices[i];
        const auto& v2 = polygon.vertices[(i + 1) % polygon.vertices.size()];
        area += (v1.x * v2.y - v2.x * v1.y);
    }
    return std::abs(area) / 2.0f / 1000000.0f;  // Convert m² to km²
}

} // namespace swarm::autonomy