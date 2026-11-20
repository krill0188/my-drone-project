#include "perimeter_surveillance.hpp"
#include <algorithm>
#include <cmath>
#include <random>

namespace swarm::autonomy {

PerimeterSurveillance::PerimeterSurveillance(const Config& config) : _config(config) {}

std::vector<Task> PerimeterSurveillance::planPerimeterMission(const Mission& mission,
                                                               const std::vector<CapabilityBitmap>& drone_capabilities,
                                                               const SwarmMap* map) {
    std::vector<Task> tasks;
    
    if (!mission.constraints.geofence.isValid()) {
        return tasks;
    }
    
    const Polygon& perimeter = mission.constraints.geofence.boundary;
    
    // Decompose perimeter into segments
    std::vector<PerimeterSegment> segments = decomposePerimeter(perimeter, drone_capabilities);
    
    // Assign drones to segments
    assignDronesToSegments(segments, drone_capabilities);
    
    // Generate tasks for each segment
    for (auto& segment : segments) {
        std::vector<Vector3> waypoints = generateSegmentWaypoints(segment);
        
        if (waypoints.size() < 2) continue;
        
        Task task;
        task.id = generateSegmentId();
        task.mission_id = mission.id;
        task.type = TaskType::COVER_AREA;  // Using cover area for perimeter
        task.priority = 100;
        
        task.geometry.type = TaskType::COVER_AREA;
        task.geometry.area = Polygon{};  // Would be the segment area
        
        task.success_criteria.max_duration_s = 3600;  // 1 hour default
        task.success_criteria.require_data_return = true;
        
        // Assign drones
        for (uint16_t drone_id : segment.assigned_drones) {
            task.assigned_drone = drone_id;
            break;  // Assign first drone
        }
        
        tasks.push_back(std::move(task));
    }
    
    return tasks;
}

std::vector<PerimeterSurveillance::PerimeterSegment> PerimeterSurveillance::decomposePerimeter(
    const Polygon& perimeter, const std::vector<CapabilityBitmap>& drone_capabilities) {
    
    std::vector<PerimeterSegment> segments;
    
    if (perimeter.vertices.size() < 3) return segments;
    
    int num_drones = drone_capabilities.size();
    if (num_drones == 0) return segments;
    
    // Calculate total perimeter length
    float total_length = 0.0f;
    for (size_t i = 0; i < perimeter.vertices.size(); ++i) {
        const Vector3& v1 = perimeter.vertices[i];
        const Vector3& v2 = perimeter.vertices[(i + 1) % perimeter.vertices.size()];
        float dx = v2.x - v1.x;
        float dy = v2.y - v1.y;
        total_length += std::sqrt(dx*dx + dy*dy);
    }
    
    // Determine number of segments (one per drone, min 1)
    int num_segments = std::max(1, num_drones);
    float segment_length = total_length / num_segments;
    
    // Create segments
    float accumulated = 0.0f;
    Vector3 segment_start = perimeter.vertices[0];
    int current_vertex = 0;
    
    for (int seg = 0; seg < num_segments; ++seg) {
        PerimeterSegment segment;
        segment.id = generateSegmentId();
        segment.start = segment_start;
        segment.assigned_drones.clear();
        
        float target_length = segment_length;
        float accumulated = 0.0f;
        Vector3 current = segment_start;
        
        while (accumulated < target_length && current_vertex < perimeter.vertices.size()) {
            int next_vertex = (current_vertex + 1) % perimeter.vertices.size();
            Vector3 v1 = perimeter.vertices[current_vertex];
            Vector3 v2 = perimeter.vertices[next_vertex];
            
            float dx = v2.x - v1.x;
            float dy = v2.y - v1.y;
            float edge_length = std::sqrt(dx*dx + dy*dy);
            
            if (accumulated + edge_length <= target_length || next_vertex == 0) {
                // Take full edge
                accumulated += edge_length;
                current = v2;
                current_vertex = next_vertex;
            } else {
                // Split edge
                float remaining = target_length - accumulated;
                float ratio = remaining / edge_length;
                Vector3 split_point;
                split_point.x = v1.x + ratio * dx;
                split_point.y = v1.y + ratio * dy;
                split_point.z = v1.z + ratio * (v2.z - v1.z);
                
                accumulated = target_length;
                current = split_point;
                break;
            }
        }
        
        segment.end = current;
        segment.length = accumulated;
        segment.start = segment_start;
        segment.waypoints = generateSegmentWaypoints(segment);
        
        segments.push_back(std::move(segment));
        
        segment_start = current;
        
        if (current_vertex >= perimeter.vertices.size()) break;
    }
    
    return segments;
}

bool PerimeterSurveillance::assignDronesToSegments(
    std::vector<PerimeterSegment>& segments,
    const std::vector<CapabilityBitmap>& drone_capabilities) {
    
    // Simple assignment: one drone per segment
    for (size_t i = 0; i < segments.size() && i < drone_capabilities.size(); ++i) {
        segments[i].assigned_drones.push_back(static_cast<uint16_t>(i + 1));
    }
    
    // If more drones than segments, assign extras to longest segments
    if (drone_capabilities.size() > segments.size()) {
        // Sort segments by length descending
        std::vector<size_t> indices(segments.size());
        std::iota(indices.begin(), indices.end(), 0);
        std::sort(indices.begin(), indices.end(),
            [&](size_t a, size_t b) { return segments[a].length > segments[b].length; });
        
        for (size_t i = segments.size(); i < drone_capabilities.size(); ++i) {
            size_t idx = indices[i % segments.size()];
            segments[idx].assigned_drones.push_back(static_cast<uint16_t>(i + 1));
        }
    }
    
    return true;
}

std::vector<Vector3> PerimeterSurveillance::generateSegmentWaypoints(const PerimeterSegment& segment) {
    std::vector<Vector3> waypoints;
    
    if (!segment.start.isValid() || !segment.end.isValid()) return waypoints;
    
    float dx = segment.end.x - segment.start.x;
    float dy = segment.end.y - segment.start.y;
    float length = std::sqrt(dx*dx + dy*dy);
    
    if (length < _config.waypoint_spacing) {
        // Just start and end
        waypoints.push_back(segment.start);
        waypoints.push_back(segment.end);
        return waypoints;
    }
    
    int num_waypoints = static_cast<int>(length / _config.waypoint_spacing) + 1;
    
    for (int i = 0; i <= num_waypoints; ++i) {
        float t = static_cast<float>(i) / num_waypoints;
        Vector3 wp;
        wp.x = segment.start.x + t * (segment.end.x - segment.start.x);
        wp.y = segment.start.y + t * (segment.end.y - segment.start.y);
        wp.z = 100.0f;  // Patrol altitude
        waypoints.push_back(wp);
    }
    
    return waypoints;
}

float PerimeterSurveillance::calculateSegmentCoverage(const PerimeterSegment& segment,
                                                       const std::vector<CapabilityBitmap>& assigned_caps) const {
    // Simplified coverage calculation
    if (assigned_caps.empty()) return 0.0f;
    
    float sensor_fov = _config.sensor_fov_deg * M_PI / 180.0f;
    float coverage_per_drone = 2.0f * _config.standoff_distance * std::tan(sensor_fov / 2.0f);
    float total_coverage = coverage_per_drone * assigned_caps.size();
    
    if (segment.length <= 0) return 0.0f;
    
    float coverage = total_coverage / segment.length;
    
    // Account for overlap
    if (_config.enable_overlap && assigned_caps.size() > 1) {
        coverage *= (1.0f - _config.overlap_ratio * (assigned_caps.size() - 1));
    }
    
    return std::min(1.0f, coverage);
}

void PerimeterSurveillance::handleDroneDropout(std::vector<PerimeterSegment>& segments,
                                                uint16_t dropped_drone_id) {
    for (auto& segment : segments) {
        auto it = std::find(segment.assigned_drones.begin(), 
                           segment.assigned_drones.end(), dropped_drone_id);
        if (it != segment.assigned_drones.end()) {
            segment.assigned_drones.erase(it);
        }
    }
    
    // Reassign remaining drones if needed
    // Would re-run assignment logic
}

std::array<uint8_t, 16> PerimeterSurveillance::generateSegmentId() {
    std::array<uint8_t, 16> id;
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);
    for (auto& b : id) b = dis(gen);
    return id;
}

std::vector<Vector3> PerimeterSurveillance::generateWaypointsForEdge(
    const Vector3& start, const Vector3& end, float spacing, float altitude) {
    
    std::vector<Vector3> waypoints;
    float dx = end.x - start.x;
    float dy = end.y - start.y;
    float length = std::sqrt(dx*dx + dy*dy);
    
    if (length < spacing) {
        waypoints.push_back(start);
        waypoints.push_back(end);
        return waypoints;
    }
    
    int num_points = static_cast<int>(length / spacing) + 1;
    for (int i = 0; i <= num_points; ++i) {
        float t = static_cast<float>(i) / num_points;
        waypoints.push_back({
            start.x + t * (end.x - start.x),
            start.y + t * (end.y - start.y),
            altitude
        });
    }
    
    return waypoints;
}

bool PerimeterSurveillance::validateSegment(const PerimeterSegment& segment) const {
    return segment.length > 0 && segment.waypoints.size() >= 2;
}

} // namespace swarm::autonomy