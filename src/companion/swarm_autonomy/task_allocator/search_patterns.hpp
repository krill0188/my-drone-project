#ifndef SEARCH_PATTERNS_HPP
#define SEARCH_PATTERNS_HPP

#include "task.hpp"
#include "swarm_map.hpp"
#include <vector>
#include <array>

namespace swarm::autonomy {

// Search pattern generation for area coverage missions
class SearchPatternGenerator {
public:
    struct Config {
        float default_spacing = 50.0f;         // Default track spacing
        float overlap_ratio = 0.1f;            // Overlap between tracks
        float min_turn_radius = 20.0f;         // Minimum turn radius
        float max_bank_angle = 30.0f;          // Max bank angle (degrees)
        float cruise_speed = 10.0f;            // Cruise speed (m/s)
        float loiter_time = 0.0f;              // Loiter time at waypoints
    };
    
    struct Waypoint {
        Vector3 position;
        float speed;
        float loiter_time;
        uint8_t command;  // MAVLink command
    };
    
    SearchPatternGenerator(const Config& config = Config{});
    
    // Generate lawnmower pattern
    std::vector<Waypoint> generateLawnmower(const Polygon& area, 
                                            float spacing,
                                            float angle_deg = 0.0f);
    
    // Generate spiral pattern
    std::vector<Waypoint> generateSpiral(const Polygon& area,
                                         float spacing,
                                         Vector3 center = {0, 0, 0});
    
    // Generate adaptive pattern based on map
    std::vector<Waypoint> generateAdaptive(const Polygon& area,
                                           const SwarmMap* map,
                                           float base_spacing);
    
    // Generate perimeter pattern
    std::vector<Waypoint> generatePerimeter(const Polygon& perimeter,
                                            float spacing,
                                            float offset_distance = 0.0f);
    
    // Generate grid pattern
    std::vector<Waypoint> generateGrid(const Polygon& area,
                                       float spacing_x,
                                       float spacing_y);
    
    // Generate custom pattern from waypoints
    std::vector<Waypoint> generateCustom(const std::vector<Vector3>& waypoints,
                                         float speed = 10.0f);
    
    // Optimize waypoint order (TSP approximation)
    std::vector<Waypoint> optimizeOrder(std::vector<Waypoint>& waypoints);
    
    // Split pattern for multiple drones
    std::vector<std::vector<Waypoint>> splitForSwarm(const std::vector<Waypoint>& pattern,
                                                     int num_drones,
                                                     bool balance_workload = true);
    
    // Convert to mission tasks
    std::vector<Task> toTasks(const std::vector<Waypoint>& pattern,
                              const Mission& parent_mission);
    
    // Validate pattern
    bool validatePattern(const std::vector<Waypoint>& pattern,
                         const Polygon& area) const;
    
    // Calculate coverage
    float calculateCoverage(const std::vector<Waypoint>& pattern,
                            const Polygon& area,
                            float sensor_footprint) const;
    
    // Estimate mission time
    float estimateMissionTime(const std::vector<Waypoint>& pattern) const;

private:
    Config _config;
    
    // Helper functions
    std::vector<Vector3> generateLawnmowerWaypoints(const Polygon& bbox,
                                                    float spacing,
                                                    float angle_deg,
                                                    float altitude);
    
    std::vector<Vector3> generateSpiralWaypoints(const Polygon& area,
                                                  float spacing,
                                                  Vector3 center,
                                                  float altitude);
    
    std::vector<Vector3> clipToPolygon(const std::vector<Vector3>& waypoints,
                                       const Polygon& area);
    
    float calculateTurnTime(float speed, float bank_angle) const;
    Vector3 rotatePoint(const Vector3& point, float angle_rad, 
                        const Vector3& center);
    
    // Polygon utilities
    bool pointInPolygon(const Vector3& point, const Polygon& polygon) const;
    Polygon getBoundingBox(const Polygon& polygon);
    float polygonArea(const Polygon& polygon);
    Vector3 polygonCentroid(const Polygon& polygon);
};

} // namespace swarm::autonomy

#endif // SEARCH_PATTERNS_HPP