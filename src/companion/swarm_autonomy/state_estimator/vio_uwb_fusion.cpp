#include "vio_uwb_fusion.hpp"
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <algorithm>
#include <cmath>

namespace swarm::formation {

VioUwbFusion::VioUwbFusion(const Config& config) : _config(config) {
    // Initialize noise models
    // IMU noise: [gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z]
    Eigen::Vector6d imu_sigmas;
    imu_sigmas << _config.imu_noise_gyro, _config.imu_noise_gyro, _config.imu_noise_gyro,
                  _config.imu_noise_accel, _config.imu_noise_accel, _config.imu_noise_accel;
    _imu_noise = gtsam::noiseModel::Diagonal::Sigmas(imu_sigmas);
    
    // Camera noise (pixel)
    _camera_noise = gtsam::noiseModel::Isotropic::Sigma(2, _config.camera_noise);
    
    // UWB noise
    _uwb_noise = gtsam::noiseModel::Isotropic::Sigma(1, _config.uwb_noise);
    
    // GPS noise (3D position)
    Eigen::Vector3d gps_sigmas(_config.gps_noise, _config.gps_noise, _config.gps_noise);
    _gps_noise = gtsam::noiseModel::Diagonal::Sigmas(gps_sigmas);
    
    // Bias random walk noise
    Eigen::Vector6d bias_sigmas;
    bias_sigmas << _config.imu_bias_gyro, _config.imu_bias_gyro, _config.imu_bias_gyro,
                   _config.imu_bias_accel, _config.imu_bias_accel, _config.imu_bias_accel;
    _bias_noise = gtsam::noiseModel::Diagonal::Sigmas(bias_sigmas);
    
    // IMU preintegration parameters
    gtsam::PreintegratedImuMeasurements::Params preint_params = 
        gtsam::PreintegratedImuMeasurements::Params::MakeSharedD(_config.imu_noise_gyro, 
                                                                 _config.imu_noise_accel);
    preint_params.biasAccOmega = _config.imu_bias_accel;
    preint_params.biasGyroOmega = _config.imu_bias_gyro;
    _preint_imu = std::make_shared<gtsam::PreintegratedImuMeasurements>(preint_params);
    
    // ISAM2 parameters
    gtsam::ISAM2Params isam_params;
    isam_params.relinearizeThreshold = 0.01;
    isam_params.relinearizeSkip = 1;
    _isam = gtsam::ISAM2(isam_params);
    
    // Initialize anchor positions (would be loaded from config)
    // For now, empty - would be populated from mission config
}

void VioUwbFusion::initialize(const Eigen::Vector3d& initial_pos, const Eigen::Quaterniond& initial_ori) {
    if (_initialized) return;
    
    // Prior on first pose
    gtsam::Pose3 initial_pose(initial_ori, initial_pos);
    _graph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 0), initial_pose, 
        gtsam::noiseModel::Diagonal::Sigmas(Eigen::Vector6d::Constant(0.1))));
    
    // Prior on velocity (zero)
    _graph.add(gtsam::PriorFactor<gtsam::Vector3>(gtsam::Symbol('v', 0), Eigen::Vector3d::Zero(),
        gtsam::noiseModel::Isotropic::Sigma(3, 0.1)));
    
    // Prior on biases (zero)
    _graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', 0), 
        gtsam::imuBias::ConstantBias(), _bias_noise));
    
    // Initial estimate
    _initial_estimate.insert(gtsam::Symbol('x', 0), initial_pose);
    _initial_estimate.insert(gtsam::Symbol('v', 0), Eigen::Vector3d::Zero());
    _initial_estimate.insert(gtsam::Symbol('b', 0), gtsam::imuBias::ConstantBias());
    
    _isam.update(_graph, _initial_estimate);
    _current_estimate = _isam.calculateEstimate();
    
    _initialized = true;
    _state_count = 1;
    _last_gps_time_us = 0;
}

void VioUwbFusion::addImuMeasurement(const ImuMeasurement& meas) {
    _imu_buffer.push_back(meas);
    
    // Add to preintegration
    double dt = 1.0 / _config.imu_rate_hz;
    _preint_imu->integrateMeasurement(meas.accel, meas.gyro, dt);
    
    // Keep buffer bounded
    if (_imu_buffer.size() > 1000) _imu_buffer.pop_front();
}

void VioUwbFusion::addCameraMeasurement(const CameraMeasurement& meas) {
    _camera_buffer.push_back(meas);
    if (_camera_buffer.size() > 100) _camera_buffer.pop_front();
}

void VioUwbFusion::addUwbMeasurement(const UwbMeasurement& meas) {
    _uwb_buffer.push_back(meas);
    if (_uwb_buffer.size() > 100) _uwb_buffer.pop_front();
}

void VioUwbFusion::addGpsMeasurement(const GpsMeasurement& meas) {
    _gps_buffer.push_back(meas);
    _last_gps_time_us = meas.timestamp_us;
    if (_gps_buffer.size() > 50) _gps_buffer.pop_front();
    
    // Check GPS health
    uint64_t now = _imu_buffer.empty() ? 0 : _imu_buffer.back().timestamp_us;
    if (now > _last_gps_time_us + 5000000) {  // 5 seconds without GPS
        _gps_denied = true;
    } else {
        _gps_denied = false;
    }
}

std::optional<VioUwbFusion::StateEstimate> VioUwbFusion::getStateEstimate() {
    if (!_initialized) return std::nullopt;
    
    // Add pending factors
    addImuFactor();
    addCameraFactors();
    addUwbFactors();
    addGpsFactors();
    
    // Update ISAM2
    _isam.update(_graph, _initial_estimate);
    _current_estimate = _isam.calculateEstimate();
    
    // Clear processed factors
    _graph.resize(0);
    _initial_estimate.clear();
    
    // Marginalize old states if using fixed-lag
    if (_config.use_fixed_lag && _state_count > _config.fixed_lag_window_size) {
        marginalizeOldStates();
    }
    
    return extractStateEstimate();
}

void VioUwbFusion::addImuFactor() {
    if (!_preint_imu || _state_count == 0) return;
    
    // Combined IMU factor between consecutive states
    gtsam::CombinedImuFactor imu_factor(
        gtsam::Symbol('x', _state_count - 1),
        gtsam::Symbol('v', _state_count - 1),
        gtsam::Symbol('x', _state_count),
        gtsam::Symbol('v', _state_count),
        gtsam::Symbol('b', _state_count - 1),
        gtsam::Symbol('b', _state_count),
        _preint_imu
    );
    
    _graph.add(imu_factor);
    
    // Bias random walk
    _graph.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(
        gtsam::Symbol('b', _state_count - 1),
        gtsam::Symbol('b', _state_count),
        gtsam::imuBias::ConstantBias(),
        _bias_noise
    ));
    
    // Reset preintegration for next interval
    gtsam::PreintegratedImuMeasurements::Params params(
        _config.imu_noise_gyro, _config.imu_noise_accel);
    params.biasAccOmega = _config.imu_bias_accel;
    params.biasGyroOmega = _config.imu_bias_gyro;
    _preint_imu = std::make_shared<gtsam::PreintegratedImuMeasurements>(params);
}

void VioUwbFusion::addCameraFactors() {
    // Visual-inertial odometry factors
    // For each camera measurement, add projection factors for tracked features
    // Simplified: assume camera provides relative pose constraint
    
    while (!_camera_buffer.empty()) {
        const auto& meas = _camera_buffer.front();
        
        // In real implementation, would add projection factors for each feature
        // For now, add a relative pose factor if visual odometry available
        
        // Pose between current and previous frame (from visual odometry)
        // gtsam::Pose3 relative_pose = ... from VO
        // _graph.add(gtsam::BetweenFactor<gtsam::Pose3>(...));
        
        _camera_buffer.pop_front();
    }
}

void VioUwbFusion::addUwbFactors() {
    while (!_uwb_buffer.empty()) {
        const auto& meas = _uwb_buffer.front();
        
        // Get anchor position
        auto it = _anchor_positions.find(meas.anchor_id);
        if (it == _anchor_positions.end()) {
            _uwb_buffer.pop_front();
            continue;
        }
        
        Eigen::Vector3d anchor_pos = it->second;
        
        // Check NLOS
        if (_config.enable_nlos_detection) {
            // Get current pose estimate
            gtsam::Pose3 current_pose;
            try {
                current_pose = _current_estimate.at<gtsam::Pose3>(gtsam::Symbol('x', _state_count));
            } catch (...) {
                _uwb_buffer.pop_front();
                continue;
            }
            
            if (detectNlos(meas, current_pose)) {
                // Reject NLOS measurement
                _uwb_buffer.pop_front();
                continue;
            }
        }
        
        // Add UWB range factor
        gtsam::Point3 anchor_point(anchor_pos.x(), anchor_pos.y(), anchor_pos.z());
        _graph.add(gtsam::RangeFactor<gtsam::Pose3>(
            gtsam::Symbol('x', _state_count),
            anchor_point,
            meas.range,
            _uwb_noise
        ));
        
        _uwb_buffer.pop_front();
    }
}

void VioUwbFusion::addGpsFactors() {
    if (_gps_denied) return;
    
    while (!_gps_buffer.empty()) {
        const auto& meas = _gps_buffer.front();
        
        // Use covariance from GPS if available
        gtsam::SharedNoiseModel gps_noise;
        if (meas.covariance.determinant() > 0) {
            gps_noise = gtsam::noiseModel::Gaussian::Covariance(meas.covariance);
        } else {
            gps_noise = _gps_noise;
        }
        
        // GPS position factor (on pose translation)
        gtsam::Point3 gps_pos(meas.position_enu.x(), meas.position_enu.y(), meas.position_enu.z());
        _graph.add(gtsam::GPSFactor(gtsam::Symbol('x', _state_count), gps_pos, gps_noise));
        
        _gps_buffer.pop_front();
    }
}

void VioUwbFusion::marginalizeOldStates() {
    // ISAM2 handles marginalization automatically with fixed-lag
    // This function would manually marginalize if needed
    // ISAM2 does this internally when using fixed-lag smoothing
}

VioUwbFusion::StateEstimate VioUwbFusion::extractStateEstimate() {
    StateEstimate estimate;
    estimate.timestamp_us = _imu_buffer.empty() ? 0 : _imu_buffer.back().timestamp_us;
    estimate.gps_denied = _gps_denied;
    
    try {
        // Get current pose
        gtsam::Pose3 pose = _current_estimate.at<gtsam::Pose3>(gtsam::Symbol('x', _state_count));
        estimate.position_enu = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
        estimate.orientation = Eigen::Quaterniond(pose.rotation().toQuaternion().w(), 
                                                   pose.rotation().toQuaternion().x(),
                                                   pose.rotation().toQuaternion().y(),
                                                   pose.rotation().toQuaternion().z());
        
        // Get velocity
        estimate.velocity_enu = _current_estimate.at<gtsam::Vector3>(gtsam::Symbol('v', _state_count));
        
        // Get biases
        gtsam::imuBias::ConstantBias bias = _current_estimate.at<gtsam::imuBias::ConstantBias>(gtsam::Symbol('b', _state_count));
        estimate.accel_bias = bias.accelerometer();
        estimate.gyro_bias = bias.gyroscope();
        
        // Compute covariance (marginal covariance for current state)
        estimate.covariance = computeCovariance();
        
        // Position uncertainty (trace of position covariance)
        estimate.position_uncertainty_m = std::sqrt(estimate.covariance.block<3,3>(0,0).trace() / 3.0);
        
    } catch (...) {
        // State not available yet
        return StateEstimate{};
    }
    
    return estimate;
}

Eigen::Matrix<double, 15, 15> VioUwbFusion::computeCovariance() {
    Eigen::Matrix<double, 15, 15> cov = Eigen::Matrix<double, 15, 15>::Zero();
    
    try {
        // Get marginal covariance for current state
        gtsam::Marginals marginals(_graph, _current_estimate);
        
        // Position covariance (3x3)
        gtsam::Matrix pos_cov = marginals.marginalCovariance(gtsam::Symbol('x', _state_count));
        if (pos_cov.rows() >= 3) {
            cov.block<3,3>(0,0) = pos_cov.block<3,3>(0,0);
        }
        
        // Velocity covariance (3x3)
        gtsam::Matrix vel_cov = marginals.marginalCovariance(gtsam::Symbol('v', _state_count));
        if (vel_cov.rows() >= 3) {
            cov.block<3,3>(3,3) = vel_cov.block<3,3>(0,0);
        }
        
        // Orientation covariance (3x3 from quaternion)
        // Approximate from pose covariance
        if (pos_cov.rows() >= 6) {
            cov.block<3,3>(6,6) = pos_cov.block<3,3>(3,3);
        }
        
        // Bias covariances (6x6)
        gtsam::Matrix bias_cov = marginals.marginalCovariance(gtsam::Symbol('b', _state_count));
        if (bias_cov.rows() >= 6) {
            cov.block<6,6>(9,9) = bias_cov;
        }
        
    } catch (...) {
        // Return diagonal covariance as fallback
        cov.setIdentity();
        cov *= 0.1;
    }
    
    return cov;
}

bool VioUwbFusion::detectNlos(const UwbMeasurement& meas, const gtsam::Pose3& current_pose) {
    // Compute expected range from current pose to anchor
    auto it = _anchor_positions.find(meas.anchor_id);
    if (it == _anchor_positions.end()) return false;
    
    Eigen::Vector3d anchor_pos = it->second;
    Eigen::Vector3d drone_pos = current_pose.translation();
    
    double expected_range = (drone_pos - anchor_pos).norm();
    double range_diff = std::abs(meas.range - expected_range);
    
    // NLOS if difference exceeds threshold
    return range_diff > _config.nlos_threshold;
}

void VioUwbFusion::reset() {
    _isam = gtsam::ISAM2();
    _graph.resize(0);
    _initial_estimate.clear();
    _current_estimate.clear();
    _imu_buffer.clear();
    _camera_buffer.clear();
    _uwb_buffer.clear();
    _gps_buffer.clear();
    _initialized = false;
    _state_count = 0;
    _gps_denied = false;
    _last_gps_time_us = 0;
}

void VioUwbFusion::setGpsDenied(bool denied) {
    _gps_denied = denied;
}

float VioUwbFusion::getPositionDrift() const {
    // Estimate position drift in GPS-denied mode
    // Would compare VIO+UWB trajectory with last known GPS position
    // For now, return uncertainty as proxy
    return 0.0f;  // Would be computed from actual trajectory comparison
}

} // namespace swarm::formation