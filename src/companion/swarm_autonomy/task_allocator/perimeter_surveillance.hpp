#ifndef PERIMETER_SURVEILLANCE_HPP
#define PERIMETER_SURVEILLANCE_HPP

#include "task.hpp"
#include "swarm_map.hpp"
#include <vector>
#include <array>

namespace swarm::autonomy {

// Perimeter surveillance task allocation
class PerimeterSurveillance {
public:
    struct Config {
        float waypoint_spacing = 50.0f;         // Waypoint spacing along perimeter
        float standoff_distance = 100.0f;       // Distance from perimeter
        float patrol_speed = 8.0f;              // Patrol speed (m/s)
        float loiter_time = 5.0f;               // Loiter time at waypoints
        float sensor_fov_deg = 60.0f;           // Sensor field of view
        int min_drones_per_segment = 1;         // Min drones per segment
        bool enable_overlap = true;             // Enable coverage overlap
        float overlap_ratio = 0.2f;             // Overlap ratio between drones
    };
    
    struct PerimeterSegment {
        std::array<uint8_t, 16> id;
        Vector3 start;
        Vector3 end;
        float length;
        std::vector<Vector3> waypoints;
        std::vector<uint16_t> assigned_drones;
    };
    
    PerimeterSurveillance(const Config& config = Config{});
    
    // Plan perimeter surveillance mission
    std::vector<Task> planPerimeterMission(const Mission& mission,
                                           const std::vector<CapabilityBitmap>& drone_capabilities,
                                           const SwarmMap* map = nullptr);
    
    // Decompose perimeter into segments
    std::vector<PerimeterSegment> decomposePerimeter(const Polygon& perimeter,
                                                      const std::vector<CapabilityBitmap>& drone_caps);
    
    // Assign drones to segments
    bool assignDronesToSegments(std::vector<PerimeterSegment>& segments,
                                const std::vector<CapabilityBitmap>& drone_capabilities);
    
    // Generate waypoints for segment
    std::vector<Vector3> generateSegmentWaypoints(const PerimeterSegment& segment);
    
    // Calculate coverage for segment
    float calculateSegmentCoverage(const PerimeterSegment& segment,
                                   const std::vector<CapabilityBitmap>& assigned_caps) const;
    
    // Reassign drones on dropout
    void handleDroneDropout(std::vector<PerimeterSegment>& segments,
                            uint16_t dropped_drone_id);

private:
    Config _config;
    
    std::array<uint8_t, 16> generateSegmentId();
    std::vector<Vector3> generateWaypointsForEdge(const Vector3& start, 
                                                  const Vector3& end,
                                                  float spacing,
                                                  float altitude);
    bool validateSegment(const PerimeterSegment& segment) const;
};

} // namespace swarm::autonomy

#endif // PERIMETER_SURVEILLANCE_HPP