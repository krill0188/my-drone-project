#ifndef VIO_UWB_FUSION_HPP
#define VIO_UWB_FUSION_HPP

#include "swarm_formation.hpp"
#include "swarm_comm/common/timestamp.hpp"
#include <vector>
#include <array>
#include <deque>
#include <optional>
#include <Eigen/Dense>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

namespace swarm::formation {

// GPS-denied VIO + UWB factor graph fusion (GTSAM per research.md Decision 4)
class VioUwbFusion {
public:
    struct Config {
        // Factor graph settings
        int fixed_lag_window_size = 50;       // Fixed-lag smoothing window
        float imu_noise_gyro = 0.001f;        // rad/s/sqrt(Hz)
        float imu_noise_accel = 0.1f;         // m/s^2/sqrt(Hz)
        float imu_bias_gyro = 0.0001f;        // rad/s^2
        float imu_bias_accel = 0.01f;         // m/s^3
        float camera_noise = 1.0f;            // pixel
        float uwb_noise = 0.3f;               // meters
        float gps_noise = 2.0f;               // meters
        float uwb_outlier_threshold = 3.0f;   // meters (NLOS rejection)
        
        // Sensor rates
        float imu_rate_hz = 400.0f;
        float camera_rate_hz = 30.0f;
        float uwb_rate_hz = 10.0f;
        float gps_rate_hz = 5.0f;
        
        // NLOS detection
        bool enable_nlos_detection = true;
        float nlos_threshold = 3.0f;          // meters
        
        // Fixed-lag
        bool use_fixed_lag = true;
    };
    
    struct ImuMeasurement {
        uint64_t timestamp_us;
        Eigen::Vector3d accel;      // m/s^2 (body frame)
        Eigen::Vector3d gyro;       // rad/s (body frame)
    };
    
    struct CameraMeasurement {
        uint64_t timestamp_us;
        std::vector<Eigen::Vector2d> keypoints;  // 2D features
        std::vector<int> feature_ids;            // Feature track IDs
        Eigen::Matrix3d K;                       // Camera intrinsics
    };
    
    struct UwbMeasurement {
        uint64_t timestamp_us;
        uint16_t anchor_id;
        double range;              // meters
        double quality;            // 0.0 - 1.0
    };
    
    struct GpsMeasurement {
        uint64_t timestamp_us;
        Eigen::Vector3d position_enu;  // ENU frame
        Eigen::Matrix3d covariance;    // Position covariance
        int fix_type;                  // 0=none, 1=2D, 2=3D, 3=RTK float, 4=RTK fixed
    };
    
    struct StateEstimate {
        uint64_t timestamp_us;
        Eigen::Vector3d position_enu;
        Eigen::Vector3d velocity_enu;
        Eigen::Quaterniond orientation;  // ENU from body
        Eigen::Vector3d accel_bias;
        Eigen::Vector3d gyro_bias;
        Eigen::Matrix<double, 15, 15> covariance;  // 15-state covariance
        bool gps_denied;
        float position_uncertainty_m;
    };
    
    VioUwbFusion(const Config& config = Config());
    
    // Initialize with prior
    void initialize(const Eigen::Vector3d& initial_pos, const Eigen::Quaterniond& initial_ori);
    
    // Add measurements
    void addImuMeasurement(const ImuMeasurement& meas);
    void addCameraMeasurement(const CameraMeasurement& meas);
    void addUwbMeasurement(const UwbMeasurement& meas);
    void addGpsMeasurement(const GpsMeasurement& meas);
    
    // Get current state estimate
    std::optional<StateEstimate> getStateEstimate();
    
    // Check if GPS denied
    bool isGpsDenied() const { return _gps_denied; }
    
    // Get position drift estimate (for GPS-denied validation)
    float getPositionDrift() const;
    
    // Reset filter
    void reset();
    
    // Set GPS denied mode
    void setGpsDenied(bool denied) { _gps_denied = denied; }

private:
    Config _config;
    
    // GTSAM objects
    gtsam::ISAM2 _isam;
    gtsam::NonlinearFactorGraph _graph;
    gtsam::Values _initial_estimate;
    gtsam::Values _current_estimate;
    
    // State
    int _state_count = 0;
    bool _initialized = false;
    bool _gps_denied = false;
    uint64_t _last_gps_time_us = 0;
    
    // Sensor buffers
    std::deque<ImuMeasurement> _imu_buffer;
    std::deque<CameraMeasurement> _camera_buffer;
    std::deque<UwbMeasurement> _uwb_buffer;
    std::deque<GpsMeasurement> _gps_buffer;
    
    // Anchor positions (for UWB)
    std::unordered_map<uint16_t, Eigen::Vector3d> _anchor_positions;
    
    // Noise models
    gtsam::noiseModel::Diagonal::shared_ptr _imu_noise;
    gtsam::noiseModel::Diagonal::shared_ptr _camera_noise;
    gtsam::noiseModel::Diagonal::shared_ptr _uwb_noise;
    gtsam::noiseModel::Diagonal::shared_ptr _gps_noise;
    gtsam::noiseModel::Diagonal::shared_ptr _bias_noise;
    
    // Preintegration
    std::shared_ptr<gtsam::PreintegratedImuMeasurements> _preint_imu;
    
    // NLOS detection
    bool detectNlos(const UwbMeasurement& meas, const gtsam::Pose3& current_pose);
    
    // Factor addition
    void addImuFactor();
    void addCameraFactors();
    void addUwbFactors();
    void addGpsFactors();
    
    // Marginalization
    void marginalizeOldStates();
    
    // State extraction
    StateEstimate extractStateEstimate();
    
    // Covariance computation
    Eigen::Matrix<double, 15, 15> computeCovariance();
};

} // namespace swarm::formation

#endif // VIO_UWB_FUSION_HPP