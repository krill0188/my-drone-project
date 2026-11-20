#include "search_patterns.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>

namespace swarm::autonomy {

SearchPatternGenerator::SearchPatternGenerator(const Config& config) : _config(config) {}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::generateLawnmower(
    const Polygon& area, float spacing, float angle_deg) {
    
    std::vector<Waypoint> waypoints;
    Polygon bbox = getBoundingBox(area);
    
    float angle_rad = angle_deg * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    
    // Rotate bounding box
    std::vector<Vector3> rotated_corners;
    for (const auto& v : bbox.vertices) {
        Vector3 rel = {v.x - bbox.min.x, v.y - bbox.min.y, 0};
        Vector3 rot;
        rot.x = rel.x * cos_a - rel.y * sin_a;
        rot.y = rel.x * sin_a + rel.y * cos_a;
        rot.z = 0;
        rotated_corners.push_back(rot);
    }
    
    // Find rotated bounds
    float min_x = rotated_corners[0].x, max_x = rotated_corners[0].x;
    float min_y = rotated_corners[0].y, max_y = rotated_corners[0].y;
    for (const auto& v : rotated_corners) {
        min_x = std::min(min_x, v.x);
        max_x = std::max(max_x, v.x);
        min_y = std::min(min_y, v.y);
        max_y = std::max(max_y, v.y);
    }
    
    float width = max_x - min_x;
    float height = max_y - min_y;
    
    int num_strips = static_cast<int>(width / spacing) + 1;
    
    for (int i = 0; i < num_strips; ++i) {
        float x = min_x + i * spacing;
        float y_start = min_y;
        float y_end = max_y;
        
        if (i % 2 == 0) {
            // Left to right
            Vector3 wp1 = rotatePoint({x, y_start, 0}, -angle_rad) + Vector3{bbox.min.x, bbox.min.y, 0};
            Vector3 wp2 = rotatePoint({x, y_end, 0}, -angle_rad) + Vector3{bbox.min.x, bbox.min.y, 0};
            
            waypoints.push_back({wp1, _config.cruise_speed, 0.0f, 16});
            waypoints.push_back({wp2, _config.cruise_speed, _config.loiter_time, 16});
        } else {
            // Right to left
            Vector3 wp1 = rotatePoint({x, y_end, 0}, -angle_rad) + Vector3{bbox.min.x, bbox.min.y, 0};
            Vector3 wp2 = rotatePoint({x, y_start, 0}, -angle_rad) + Vector3{bbox.min.x, bbox.min.y, 0};
            
            waypoints.push_back({wp1, _config.cruise_speed, 0.0f, 16});
            waypoints.push_back({wp2, _config.cruise_speed, _config.loiter_time, 16});
        }
    }
    
    // Add altitude
    for (auto& wp : waypoints) {
        wp.position.z = 50.0f;  // Default altitude
    }
    
    return waypoints;
}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::generateSpiral(
    const Polygon& area, float spacing, Vector3 center) {
    
    std::vector<Waypoint> waypoints;
    
    if (center.x == 0 && center.y == 0 && center.z == 0) {
        Polygon bbox = getBoundingBox(area);
        center = polygonCentroid(area);
    }
    
    Polygon bbox = getBoundingBox(area);
    float max_radius = std::max(
        std::max(bbox.max.x - center.x, center.x - bbox.min.x),
        std::max(bbox.max.y - center.y, center.y - bbox.min.y)
    );
    
    float current_radius = spacing;
    int spiral_arms = 4;
    float altitude = 50.0f;
    
    while (current_radius < max_radius) {
        for (int arm = 0; arm < spiral_arms; ++arm) {
            float angle_start = (arm * 2 * M_PI / spiral_arms) + (current_radius / spacing) * 0.1f;
            float angle_end = angle_start + 2 * M_PI / spiral_arms;
            
            int num_points = 20;
            for (int i = 0; i <= num_points; ++i) {
                float t = static_cast<float>(i) / num_points;
                float angle = angle_start + t * (angle_end - angle_start);
                float r = current_radius * t + current_radius * (1 - t);
                
                Vector3 wp;
                wp.x = center.x + r * std::cos(angle);
                wp.y = center.y + r * std::sin(angle);
                wp.z = altitude;
                
                float speed = _config.cruise_speed;
                float loiter = (i == num_points) ? _config.loiter_time : 0.0f;
                
                waypoints.push_back({wp, speed, loiter, 16});
            }
        }
        
        current_radius += spacing * 2;
    }
    
    return waypoints;
}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::generateAdaptive(
    const Polygon& area, const SwarmMap* map, float base_spacing) {
    
    // For now, use lawnmower as fallback
    // Real implementation would analyze map for obstacles, terrain, etc.
    return generateLawnmower(area, base_spacing);
}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::generatePerimeter(
    const Polygon& perimeter, float spacing, float offset_distance) {
    
    std::vector<Waypoint> waypoints;
    
    if (perimeter.vertices.size() < 3) return waypoints;
    
    float altitude = 50.0f;
    
    for (size_t i = 0; i < perimeter.vertices.size(); ++i) {
        const Vector3& v1 = perimeter.vertices[i];
        const Vector3& v2 = perimeter.vertices[(i + 1) % perimeter.vertices.size()];
        
        float dx = v2.x - v1.x;
        float dy = v2.y - v1.y;
        float edge_length = std::sqrt(dx*dx + dy*dy);
        
        int num_waypoints = std::max(1, static_cast<int>(edge_length / spacing));
        
        for (int j = 0; j < num_waypoints; ++j) {
            float t = static_cast<float>(j) / num_waypoints;
            
            Vector3 wp;
            wp.x = v1.x + t * dx;
            wp.y = v1.y + t * dy;
            wp.z = 50.0f;  // Perimeter altitude
            
            // Apply offset perpendicular to edge
            if (offset_distance != 0) {
                float edge_len = std::sqrt(dx*dx + dy*dy);
                if (edge_len > 0) {
                    float nx = -dy / edge_len;
                    float ny = dx / edge_len;
                    wp.x += nx * offset_distance;
                    wp.y += ny * offset_distance;
                }
            }
            
            waypoints.push_back({wp, _config.cruise_speed, 0.0f, 16});
        }
    }
    
    return waypoints;
}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::generateGrid(
    const Polygon& area, float spacing_x, float spacing_y) {
    
    std::vector<Waypoint> waypoints;
    Polygon bbox = getBoundingBox(area);
    
    float altitude = 50.0f;
    
    for (float x = bbox.min.x; x <= bbox.max.x; x += spacing_x) {
        for (float y = bbox.min.y; y <= bbox.max.y; y += spacing_y) {
            Vector3 wp{x, y, 50.0f};
            waypoints.push_back({wp, _config.cruise_speed, _config.loiter_time, 16});
        }
    }
    
    return waypoints;
}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::generateCustom(
    const std::vector<Vector3>& waypoints, float speed) {
    
    std::vector<Waypoint> result;
    for (const auto& wp : waypoints) {
        result.push_back({wp, speed, 0.0f, 16});
    }
    return result;
}

std::vector<SearchPatternGenerator::Waypoint> SearchPatternGenerator::optimizeOrder(
    std::vector<Waypoint>& waypoints) {
    
    if (waypoints.size() < 3) return waypoints;
    
    // Simple nearest neighbor heuristic
    std::vector<Waypoint> optimized;
    std::vector<bool> used(waypoints.size(), false);
    
    // Start with first waypoint
    optimized.push_back(waypoints[0]);
    used[0] = true;
    
    for (size_t i = 1; i < waypoints.size(); ++i) {
        Vector3 last_pos = optimized.back().position;
        float best_dist = std::numeric_limits<float>::infinity();
        size_t best_idx = 0;
        
        for (size_t j = 0; j < waypoints.size(); ++j) {
            if (used[j]) continue;
            
            const Vector3& pos = waypoints[j].position;
            float dx = pos.x - last_pos.x;
            float dy = pos.y - last_pos.y;
            float dz = pos.z - last_pos.z;
            float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            
            if (dist < best_dist) {
                best_dist = dist;
                best_idx = j;
            }
        }
        
        optimized.push_back(waypoints[best_idx]);
        used[best_idx] = true;
    }
    
    return optimized;
}

std::vector<std::vector<SearchPatternGenerator::Waypoint>> SearchPatternGenerator::splitForSwarm(
    const std::vector<Waypoint>& pattern, int num_drones, bool balance_workload) {
    
    std::vector<std::vector<Waypoint>> result(num_drones);
    
    if (num_drones <= 1) {
        result[0] = pattern;
        return result;
    }
    
    if (balance_workload) {
        // Distribute waypoints evenly
        size_t per_drone = pattern.size() / num_drones;
        size_t remainder = pattern.size() % num_drones;
        
        size_t start = 0;
        for (int i = 0; i < num_drones; ++i) {
            size_t count = per_drone + (i < remainder ? 1 : 0);
            result[i].assign(pattern.begin() + start, pattern.begin() + start + count);
            start += count;
        }
    } else {
        // Round-robin assignment
        for (size_t i = 0; i < pattern.size(); ++i) {
            result[i % num_drones].push_back(pattern[i]);
        }
    }
    
    return result;
}

std::vector<Task> SearchPatternGenerator::toTasks(const std::vector<Waypoint>& pattern,
                                                   const Mission& parent_mission) {
    std::vector<Task> tasks;
    
    for (size_t i = 0; i < pattern.size(); ++i) {
        Task task;
        task.id = generateTaskId();
        task.mission_id = parent_mission.id;
        task.type = TaskType::COVER_AREA;
        task.priority = 100;
        
        task.geometry.type = TaskType::COVER_AREA;
        task.geometry.area = Polygon{};  // Would be the area for this waypoint
        task.geometry.pattern = SearchPattern::LAWNMOWER;
        
        task.success_criteria.max_duration_s = 300;
        task.success_criteria.require_data_return = true;
        
        tasks.push_back(std::move(task));
    }
    
    return tasks;
}

bool SearchPatternGenerator::validatePattern(const std::vector<Waypoint>& pattern,
                                              const Polygon& area) const {
    if (pattern.empty()) return false;
    
    // Check all waypoints are within area
    for (const auto& wp : pattern) {
        if (!pointInPolygon(wp.position, area)) {
            return false;
        }
    }
    
    return true;
}

float SearchPatternGenerator::calculateCoverage(const std::vector<Waypoint>& pattern,
                                                 const Polygon& area,
                                                 float sensor_footprint) const {
    // Simplified coverage calculation
    float area_km2 = polygonAreaKm2(area);
    float track_spacing = 50.0f;  // Default
    float track_width = sensor_footprint;
    
    float covered_width = pattern.size() * track_width;
    float total_width = polygonAreaKm2(area) * 1000000.0f / 1000.0f;  // Rough estimate
    
    return std::min(1.0f, covered_width / total_width);
}

float SearchPatternGenerator::estimateMissionTime(const std::vector<Waypoint>& pattern) const {
    if (pattern.size() < 2) return 0.0f;
    
    float total_time = 0.0f;
    for (size_t i = 1; i < pattern.size(); ++i) {
        float dx = pattern[i].position.x - pattern[i-1].position.x;
        float dy = pattern[i].position.y - pattern[i-1].position.y;
        float dz = pattern[i].position.z - pattern[i-1].position.z;
        float dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        total_time += dist / pattern[i].speed + pattern[i].loiter_time;
    }
    
    return total_time;
}

std::vector<Vector3> SearchPatternGenerator::generateLawnmowerWaypoints(
    const Polygon& bbox, float spacing, float angle_deg, float altitude) {
    
    std::vector<Vector3> waypoints;
    // Simplified - would generate actual waypoints
    return waypoints;
}

std::vector<Vector3> SearchPatternGenerator::generateSpiralWaypoints(
    const Polygon& area, float spacing, Vector3 center, float altitude) {
    
    std::vector<Vector3> waypoints;
    // Simplified
    return waypoints;
}

std::vector<Vector3> SearchPatternGenerator::clipToPolygon(
    const std::vector<Vector3>& waypoints, const Polygon& area) {
    
    std::vector<Vector3> clipped;
    for (const auto& wp : waypoints) {
        if (pointInPolygon(wp, area)) {
            clipped.push_back(wp);
        }
    }
    return clipped;
}

float SearchPatternGenerator::calculateTurnTime(float speed, float bank_angle) const {
    // Turn rate = g * tan(bank) / speed
    float turn_rate = 9.81f * std::tan(bank_angle * M_PI / 180.0f) / speed;
    // 180 degree turn time
    return M_PI / turn_rate;
}

Vector3 SearchPatternGenerator::rotatePoint(const Vector3& point, float angle_rad,
                                            const Vector3& center) {
    float dx = point.x - center.x;
    float dy = point.y - center.y;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    
    return {
        center.x + dx * cos_a - dy * sin_a,
        center.y + dx * sin_a + dy * cos_a,
        point.z
    };
}

bool SearchPatternGenerator::pointInPolygon(const Vector3& point, const Polygon& polygon) const {
    if (polygon.vertices.size() < 3) return false;
    
    bool inside = false;
    for (size_t i = 0, j = polygon.vertices.size() - 1; i < polygon.vertices.size(); j = i++) {
        const auto& vi = polygon.vertices[i];
        const auto& vj = polygon.vertices[j];
        
        bool intersect = ((vi.y > point.y) != (vj.y > point.y)) &&
                        (point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x);
        if (intersect) inside = !inside;
    }
    
    return inside;
}

Polygon SearchPatternGenerator::getBoundingBox(const Polygon& polygon) {
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

float SearchPatternGenerator::polygonArea(const Polygon& polygon) {
    if (polygon.vertices.size() < 3) return 0.0f;
    
    float area = 0.0f;
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        const auto& v1 = polygon.vertices[i];
        const auto& v2 = polygon.vertices[(i + 1) % polygon.vertices.size()];
        area += (v1.x * v2.y - v2.x * v1.y);
    }
    return std::abs(area) / 2.0f;
}

Vector3 SearchPatternGenerator::polygonCentroid(const Polygon& polygon) {
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

} // namespace swarm::autonomy