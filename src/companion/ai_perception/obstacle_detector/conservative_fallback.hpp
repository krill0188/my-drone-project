#ifndef CONSERVATIVE_FALLBACK_HPP
#define CONSERVATIVE_FALLBACK_HPP

#include "../../../common/ai_inference.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <deque>
#include <optional>

namespace swarm::ai {

struct ConservativeFallbackConfig {
    // Sensor configuration
    bool enable_lidar = true;
    bool enable_sonar = true;
    bool enable_depth_camera = true;
    
    // LiDAR parameters
    struct LidarConfig {
        float max_range_m = 100.0f;
        float min_range_m = 0.2f;
        float angular_resolution_deg = 0.5f;
        float vertical_fov_deg = 30.0f;
        int num_layers = 16;
        float noise_sigma_m = 0.05f;
        float outlier_threshold_m = 2.0f;
    } lidar;
    
    // Sonar parameters
    struct SonarConfig {
        float max_range_m = 10.0f;
        float min_range_m = 0.1f;
        float beam_angle_deg = 30.0f;
        int num_beams = 8;  // Circular array
        float noise_sigma_m = 0.02f;
    } sonar;
    
    // Avoidance parameters
    float safety_margin_m = 5.0f;           // FR-018: separation > 5m
    float emergency_margin_m = 3.0f;        // Emergency stop distance
    float max_avoidance_velocity_mps = 5.0f;
    float max_avoidance_accel_mps2 = 3.0f;
    
    // Geometric algorithms
    enum class AvoidanceAlgorithm {
        VECTOR_FIELD = 0,    // Potential field
        VELOCITY_OBSTACLE = 1,  // VO/ORCA
        RRT_STAR = 2,        // Local RRT*
        HYBRID = 3           // Combination
    };
    
    AvoidanceAlgorithm algorithm = AvoidanceAlgorithm::HYBRID;
    
    // Voxel grid for occupancy
    float voxel_size_m = 0.5f;
    int grid_size_m = 50;  // 50x50x50m grid
    
    // Performance
    int update_rate_hz = 50;
    float max_computation_ms = 10.0f;
    
    // Fallback triggers
    bool trigger_on_ai_failure = true;
    bool trigger_on_low_confidence = true;
    float confidence_threshold = 0.5f;
    bool trigger_on_latency = true;
    float latency_threshold_ms = 100.0f;
};

struct SensorData {
    uint64_t timestamp_us;
    
    // LiDAR point cloud (ENU frame)
    std::vector<Eigen::Vector3f> lidar_points;
    
    // Sonar ranges (8 beams: N, NE, E, SE, S, SW, W, NW)
    std::array<float, 8> sonar_ranges;
    std::array<bool, 8> sonar_valid;
    
    // Depth camera (optional)
    std::vector<Eigen::Vector3f> depth_points;
    
    // Current state
    Eigen::Vector3f position_enu;
    Eigen::Vector3f velocity_enu;
    Eigen::Quaternionf orientation;
};

struct AvoidanceCommand {
    uint64_t timestamp_us;
    bool valid = false;
    
    // Velocity command (ENU frame, m/s)
    Eigen::Vector3f velocity_cmd_enu;
    
    // Acceleration command (ENU frame, m/s^2)
    Eigen::Vector3f accel_cmd_enu;
    
    // Alternative: position waypoint
    Eigen::Vector3f waypoint_enu;
    float waypoint_yaw_rad = 0.0f;
    
    // Metadata
    std::string algorithm_used;
    float computation_ms = 0.0f;
    int num_obstacles_detected = 0;
    float closest_obstacle_dist_m = 0.0f;
    bool emergency_stop = false;
    
    // Debug
    std::vector<Eigen::Vector3f> obstacle_centers;
    std::vector<float> obstacle_radii;
};

class ConservativeFallback {
public:
    ConservativeFallback();
    ~ConservativeFallback();
    
    ConservativeFallback(const ConservativeFallback&) = delete;
    ConservativeFallback& operator=(const ConservativeFallback&) = delete;
    ConservativeFallback(ConservativeFallback&&) noexcept;
    ConservativeFallback& operator=(ConservativeFallback&&) noexcept;
    
    // Initialize
    bool initialize(const ConservativeFallbackConfig& config);
    
    // Update sensor data
    void updateLidar(const std::vector<Eigen::Vector3f>& points, uint64_t timestamp_us);
    void updateSonar(const std::array<float, 8>& ranges, const std::array<bool, 8>& valid, uint64_t timestamp_us);
    void updateDepth(const std::vector<Eigen::Vector3f>& points, uint64_t timestamp_us);
    void updateState(const Eigen::Vector3f& pos, const Eigen::Vector3f& vel, 
                     const Eigen::Quaternionf& orient, uint64_t timestamp_us);
    
    // Compute avoidance command
    AvoidanceCommand computeAvoidance(const Eigen::Vector3f& target_velocity_enu);
    
    // Emergency stop check
    bool checkEmergencyStop() const;
    
    // Get occupancy grid (for debug/visualization)
    struct OccupancyGrid {
        float voxel_size_m;
        Eigen::Vector3i grid_dim;  // x, y, z
        Eigen::Vector3f origin_enu;
        std::vector<uint8_t> occupancy;  // 0=free, 1=occupied, 2=unknown
    };
    std::optional<OccupancyGrid> getOccupancyGrid() const;
    
    // Check if fallback should activate
    bool shouldActivate(float ai_confidence, float ai_latency_ms) const;
    
    // Get config
    const ConservativeFallbackConfig& getConfig() const { return config_; }
    
    // Get stats
    struct Stats {
        uint64_t total_computations = 0;
        uint64_t emergency_stops = 0;
        float avg_computation_ms = 0.0f;
        float max_computation_ms = 0.0f;
        int max_obstacles_detected = 0;
    };
    const Stats& getStats() const { return stats_; }
    
    // Reset
    void reset();
    
    // Set callback
    using AvoidanceCallback = std::function<void(const AvoidanceCommand&)>;
    void setAvoidanceCallback(AvoidanceCallback cb) { avoidance_callback_ = cb; }
    
    using EmergencyCallback = std::function<void(const Eigen::Vector3f&)>;
    void setEmergencyCallback(EmergencyCallback cb) { emergency_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    ConservativeFallbackConfig config_;
    bool initialized_ = false;
    
    Stats stats_;
    std::mutex stats_mutex_;
    
    std::function<void(const AvoidanceCommand&)> avoidance_callback_;
    std::function<void(const Eigen::Vector3f&)> emergency_callback_;
};

}  // namespace swarm::ai

#endif  // CONSERVATIVE_FALLBACK_HPP