#include "health_monitor.hpp"
#include <algorithm>
#include <numeric>
#include <iostream>

namespace swarm::ai {

class HealthMonitor::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    bool initialize(const HealthMonitorConfig& config) {
        config_ = config;
        
        // Initialize sensor health structures
        for (const auto& [name, sensor_config] : config_.sensor_configs) {
            SensorHealth health;
            health.name = name;
            health.max_samples = config_.baseline_window_s * sensor_config.expected_rate_hz;
            sensors_[name] = health;
        }
        
        initialized_ = true;
        return true;
    }
    
    void updateSensor(const std::string& name, float value, 
                      uint64_t timestamp_us, float latency_ms) {
        auto it = sensors_.find(name);
        if (it == sensors_.end()) return;
        
        SensorHealth& health = it->second;
        float prev_score = health.health_score;
        bool was_healthy = health.is_healthy;
        
        health.update(value, timestamp_us, latency_ms);
        
        // Compute health score based on statistical anomaly detection
        float z_score = health.getZScore(value);
        float latency_penalty = std::min(1.0f, latency_ms / config_.sensor_configs.at(name).max_latency_ms);
        
        // Health score: 1.0 - normalized anomaly score
        float anomaly_score = std::min(1.0f, z_score / config_.anomaly_threshold_sigma);
        health.health_score = (1.0f - anomaly_score) * (1.0f - 0.5f * latency_penalty);
        health.health_score = std::clamp(health.health_score, 0.0f, 1.0f);
        
        // Determine if healthy
        health.is_healthy = (health.health_score >= config_.warning_score_threshold);
        
        // Generate anomaly reason
        if (!health.is_healthy && was_healthy) {
            if (z_score > config_.anomaly_threshold_sigma) {
                health.anomaly_reason = "Statistical outlier (z-score: " + std::to_string(z_score).substr(0, 4) + ")";
            } else if (latency_penalty > 0.5f) {
                health.anomaly_reason = "High latency (" + std::to_string(latency_ms).substr(0, 4) + " ms)";
            } else {
                health.anomaly_reason = "Degraded signal quality";
            }
            
            if (alert_callback_) {
                alert_callback_(name, health.health_score, health.anomaly_reason);
            }
        } else if (health.is_healthy && !was_healthy) {
            health.anomaly_reason.clear();
        }
    }
    
    void updateFromFrame(const FaultClassifier::SensorFrame& frame, uint64_t timestamp_us) {
        // IMU
        updateSensor("imu_accel_x", frame.accel.x(), timestamp_us);
        updateSensor("imu_accel_y", frame.accel.y(), timestamp_us);
        updateSensor("imu_accel_z", frame.accel.z(), timestamp_us);
        updateSensor("imu_gyro_x", frame.gyro.x(), timestamp_us);
        updateSensor("imu_gyro_y", frame.gyro.y(), timestamp_us);
        updateSensor("imu_gyro_z", frame.gyro.z(), timestamp_us);
        updateSensor("imu_mag_x", frame.mag.x(), timestamp_us);
        updateSensor("imu_mag_y", frame.mag.y(), timestamp_us);
        updateSensor("imu_mag_z", frame.mag.z(), timestamp_us);
        
        // GPS
        if (frame.gps_valid) {
            updateSensor("gps_pos_x", frame.gps_pos_enu.x(), timestamp_us);
            updateSensor("gps_pos_y", frame.gps_pos_enu.y(), timestamp_us);
            updateSensor("gps_pos_z", frame.gps_pos_enu.z(), timestamp_us);
            updateSensor("gps_vel_x", frame.gps_vel_enu.x(), timestamp_us);
            updateSensor("gps_vel_y", frame.gps_vel_enu.y(), timestamp_us);
            updateSensor("gps_vel_z", frame.gps_vel_enu.z(), timestamp_us);
            updateSensor("gps_hdop", frame.gps_hdop, timestamp_us);
            updateSensor("gps_satellites", (float)frame.gps_satellites, timestamp_us);
        }
        
        // Barometer
        updateSensor("baro_altitude", frame.baro_altitude, timestamp_us);
        updateSensor("baro_pressure", frame.baro_pressure, timestamp_us);
        
        // Motors
        for (size_t i = 0; i < frame.motor_rpm.size(); ++i) {
            updateSensor("motor" + std::to_string(i) + "_rpm", frame.motor_rpm[i], timestamp_us);
        }
        for (size_t i = 0; i < frame.motor_current.size(); ++i) {
            updateSensor("motor" + std::to_string(i) + "_current", frame.motor_current[i], timestamp_us);
        }
        for (size_t i = 0; i < frame.motor_voltage.size(); ++i) {
            updateSensor("motor" + std::to_string(i) + "_voltage", frame.motor_voltage[i], timestamp_us);
        }
        for (size_t i = 0; i < frame.motor_temp.size(); ++i) {
            updateSensor("motor" + std::to_string(i) + "_temp", frame.motor_temp[i], timestamp_us);
        }
        
        // Battery
        updateSensor("battery_voltage", frame.battery_voltage, timestamp_us);
        updateSensor("battery_current", frame.battery_current, timestamp_us);
        updateSensor("battery_temp", frame.battery_temp, timestamp_us);
        updateSensor("battery_soc", frame.battery_soc, timestamp_us);
        
        for (size_t i = 0; i < frame.cell_voltages.size(); ++i) {
            updateSensor("cell" + std::to_string(i) + "_voltage", frame.cell_voltages[i], timestamp_us);
        }
        
        // Communication
        updateSensor("comm_rssi", frame.comm_rssi, timestamp_us);
        updateSensor("comm_snr", frame.comm_snr, timestamp_us);
        updateSensor("comm_packet_loss", frame.comm_packet_loss, timestamp_us);
        updateSensor("comm_latency", frame.comm_latency_ms, timestamp_us);
        
        // VIO
        if (frame.vio_valid) {
            updateSensor("vio_pos_x", frame.vio_pos_enu.x(), timestamp_us);
            updateSensor("vio_pos_y", frame.vio_pos_enu.y(), timestamp_us);
            updateSensor("vio_pos_z", frame.vio_pos_enu.z(), timestamp_us);
        }
        
        // Airspeed
        if (frame.airspeed_valid) {
            updateSensor("airspeed", frame.airspeed, timestamp_us);
        }
    }
    
    SystemHealth getHealth(uint16_t drone_id, uint64_t timestamp_us) const {
        SystemHealth health;
        health.timestamp_us = timestamp_us;
        health.drone_id = drone_id;
        
        float total_score = 0.0f;
        int count = 0;
        
        // Subsystem scores
        std::map<std::string, std::pair<float, int>> subsystem_accum;
        
        for (const auto& [name, sensor] : sensors_) {
            health.sensors[name] = sensor;
            total_score += sensor.health_score;
            count++;
            
            if (!sensor.is_healthy) {
                if (sensor.health_score < config_.critical_score_threshold) {
                    health.failed_sensors.push_back(name);
                } else {
                    health.degraded_sensors.push_back(name);
                }
            }
            
            // Map to subsystems
            std::string subsystem = getSubsystem(name);
            subsystem_accum[subsystem].first += sensor.health_score;
            subsystem_accum[subsystem].second++;
        }
        
        health.overall_score = count > 0 ? total_score / count : 1.0f;
        
        // Compute subsystem scores
        for (const auto& [sub, acc] : subsystem_accum) {
            health.subsystem_scores[sub] = acc.second > 0 ? acc.first / acc.second : 1.0f;
        }
        
        return health;
    }
    
    float getSensorHealth(const std::string& name) const {
        auto it = sensors_.find(name);
        if (it != sensors_.end()) {
            return it->second.health_score;
        }
        return 1.0f;
    }
    
    bool isAnomalous(const std::string& name, float value) const {
        auto it = sensors_.find(name);
        if (it == sensors_.end()) return false;
        
        float z_score = it->second.getZScore(value);
        return z_score > config_.anomaly_threshold_sigma;
    }
    
    std::string getAnomalyReason(const std::string& name, float value) const {
        auto it = sensors_.find(name);
        if (it == sensors_.end()) return "Unknown sensor";
        
        float z_score = it->second.getZScore(value);
        if (z_score > config_.anomaly_threshold_sigma) {
            return "Statistical outlier (z-score: " + std::to_string(z_score).substr(0, 4) + ")";
        }
        return "Normal";
    }
    
    void resetBaseline(const std::string& name) {
        auto it = sensors_.find(name);
        if (it != sensors_.end()) {
            it->second.recent_values.clear();
            it->second.mean = 0.0f;
            it->second.stddev = 0.0f;
            it->second.health_score = 1.0f;
            it->second.is_healthy = true;
            it->second.anomaly_reason.clear();
        }
    }
    
    void resetAllBaselines() {
        for (auto& [name, health] : sensors_) {
            health.recent_values.clear();
            health.mean = 0.0f;
            health.stddev = 0.0f;
            health.health_score = 1.0f;
            health.is_healthy = true;
            health.anomaly_reason.clear();
        }
    }
    
    void setHealthCallback(std::function<void(const SystemHealth&)> cb) {
        health_callback_ = cb;
    }
    
    void setSensorAlertCallback(std::function<void(const std::string&, float, const std::string&)> cb) {
        alert_callback_ = cb;
    }

private:
    HealthMonitorConfig config_;
    std::map<std::string, SensorHealth> sensors_;
    bool initialized_ = false;
    
    std::function<void(const SystemHealth&)> health_callback_;
    std::function<void(const std::string&, float, const std::string&)> alert_callback_;
    
    std::string getSubsystem(const std::string& sensor_name) const {
        if (sensor_name.find("motor") == 0 || sensor_name.find("esc") == 0) {
            return "propulsion";
        } else if (sensor_name.find("gps") == 0 || sensor_name.find("vio") == 0 || 
                   sensor_name.find("baro") == 0 || sensor_name.find("mag") == 0 ||
                   sensor_name.find("imu") == 0 || sensor_name.find("airspeed") == 0) {
            return "navigation";
        } else if (sensor_name.find("battery") == 0 || sensor_name.find("cell") == 0) {
            return "power";
        } else if (sensor_name.find("comm") == 0) {
            return "communication";
        }
        return "other";
    }
};

HealthMonitor::HealthMonitor() : pimpl_(std::make_unique<Impl>()) {}
HealthMonitor::~HealthMonitor() = default;
HealthMonitor::HealthMonitor(HealthMonitor&&) noexcept = default;
HealthMonitor& HealthMonitor::operator=(HealthMonitor&&) noexcept = default;

bool HealthMonitor::initialize(const HealthMonitorConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

void HealthMonitor::updateSensor(const std::string& name, float value, 
                                 uint64_t timestamp_us, float latency_ms) {
    pimpl_->updateSensor(name, value, timestamp_us, latency_ms);
}

void HealthMonitor::updateFromFrame(const FaultClassifier::SensorFrame& frame, uint64_t timestamp_us) {
    pimpl_->updateFromFrame(frame, timestamp_us);
}

HealthMonitor::SystemHealth HealthMonitor::getHealth(uint16_t drone_id, uint64_t timestamp_us) const {
    return pimpl_->getHealth(drone_id, timestamp_us);
}

float HealthMonitor::getSensorHealth(const std::string& name) const {
    return pimpl_->getSensorHealth(name);
}

bool HealthMonitor::isAnomalous(const std::string& name, float value) const {
    return pimpl_->isAnomalous(name, value);
}

std::string HealthMonitor::getAnomalyReason(const std::string& name, float value) const {
    return pimpl_->getAnomalyReason(name, value);
}

void HealthMonitor::resetBaseline(const std::string& name) {
    pimpl_->resetBaseline(name);
}

void HealthMonitor::resetAllBaselines() {
    pimpl_->resetAllBaselines();
}

bool HealthMonitor::isInitialized() const {
    return initialized_;
}

void HealthMonitor::setHealthCallback(HealthCallback cb) {
    health_callback_ = cb;
    pimpl_->setHealthCallback(cb);
}

void HealthMonitor::setSensorAlertCallback(SensorAlertCallback cb) {
    alert_callback_ = cb;
    pimpl_->setSensorAlertCallback(cb);
}

}  // namespace swarm::ai