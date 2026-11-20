#ifndef HEALTH_MONITOR_HPP
#define HEALTH_MONITOR_HPP

#include "fault_classifier.hpp"
#include <Eigen/Dense>
#include <vector>
#include <string>
#include <map>
#include <deque>
#include <functional>

namespace swarm::ai {

struct HealthMonitorConfig {
    // Statistical baseline parameters
    int baseline_window_s = 60;      // Window for baseline computation
    float baseline_alpha = 0.01f;    // EMA factor for online update
    float anomaly_threshold_sigma = 3.0f;  // Sigma threshold
    
    // Sensor-specific configs
    struct SensorConfig {
        float expected_rate_hz;
        float max_latency_ms;
        float min_value, max_value;
        float max_rate_of_change;
    };
    
    std::map<std::string, SensorConfig> sensor_configs = {
        {"imu", {400, 5, -50, 50, 100}},
        {"gps", {10, 200, -1e6, 1e6, 50}},
        {"baro", {50, 50, 0, 10000, 10}},
        {"mag", {50, 20, -1000, 1000, 50}},
        {"motor_rpm", {100, 20, 0, 10000, 500}},
        {"motor_current", {100, 20, 0, 100, 20}},
        {"battery_voltage", {10, 100, 0, 60, 1}},
        {"battery_current", {10, 100, -200, 200, 10}},
        {"cell_voltage", {10, 100, 2.5, 4.5, 0.1}},
        {"comm", {10, 100, -120, 0, 10}}
    };
    
    // Health scoring
    float critical_score_threshold = 0.3f;
    float warning_score_threshold = 0.6f;
    int min_samples_for_baseline = 100;
};

struct SensorHealth {
    std::string name;
    float health_score = 1.0f;  // 0 = failed, 1 = nominal
    float mean = 0.0f;
    float stddev = 0.0f;
    float last_value = 0.0f;
    uint64_t last_update_us = 0;
    float latency_ms = 0.0f;
    bool is_healthy = true;
    std::string anomaly_reason;
    
    // Baseline stats
    std::deque<float> recent_values;
    int max_samples = 1000;
    
    void update(float value, uint64_t timestamp_us, float latency_ms) {
        last_value = value;
        last_update_us = timestamp_us;
        this->latency_ms = latency_ms;
        
        recent_values.push_back(value);
        if (recent_values.size() > max_samples) {
            recent_values.pop_front();
        }
        
        // Recompute stats
        if (recent_values.size() >= 10) {
            float sum = 0, sq_sum = 0;
            for (float v : recent_values) {
                sum += v;
                sq_sum += v * v;
            }
            mean = sum / recent_values.size();
            float var = sq_sum / recent_values.size() - mean * mean;
            stddev = std::sqrt(std::max(0.0f, var));
        }
    }
    
    float getZScore(float value) const {
        if (stddev < 1e-6) return 0.0f;
        return std::abs(value - mean) / stddev;
    }
};

struct SystemHealth {
    uint64_t timestamp_us;
    uint16_t drone_id;
    float overall_score = 1.0f;  // 0 = critical, 1 = nominal
    std::map<std::string, SensorHealth> sensors;
    std::vector<std::string> degraded_sensors;
    std::vector<std::string> failed_sensors;
    std::map<std::string, float> subsystem_scores;  // e.g., "propulsion", "navigation", "power", "comm"
    
    bool isHealthy() const { return overall_score >= 0.7f; }
    bool hasCritical() const { return !failed_sensors.empty(); }
    bool hasWarning() const { return !degraded_sensors.empty(); }
};

class HealthMonitor {
public:
    HealthMonitor();
    ~HealthMonitor();
    
    HealthMonitor(const HealthMonitor&) = delete;
    HealthMonitor& operator=(const HealthMonitor&) = delete;
    HealthMonitor(HealthMonitor&&) noexcept;
    HealthMonitor& operator=(HealthMonitor&&) noexcept;
    
    // Initialize
    bool initialize(const HealthMonitorConfig& config);
    
    // Update sensor reading
    void updateSensor(const std::string& name, float value, 
                      uint64_t timestamp_us, float latency_ms = 0);
    
    // Update multiple sensors at once (from SensorFrame)
    void updateFromFrame(const FaultClassifier::SensorFrame& frame, uint64_t timestamp_us);
    
    // Get current health status
    SystemHealth getHealth(uint16_t drone_id, uint64_t timestamp_us) const;
    
    // Get health score for specific sensor
    float getSensorHealth(const std::string& name) const;
    
    // Check if sensor is anomalous
    bool isAnomalous(const std::string& name, float value) const;
    
    // Get anomaly details
    std::string getAnomalyReason(const std::string& name, float value) const;
    
    // Reset baseline for sensor
    void resetBaseline(const std::string& name);
    
    // Reset all baselines
    void resetAllBaselines();
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get config
    const HealthMonitorConfig& getConfig() const { return config_; }
    
    // Set callback for health changes
    using HealthCallback = std::function<void(const SystemHealth&)>;
    void setHealthCallback(HealthCallback cb) { health_callback_ = cb; }
    
    using SensorAlertCallback = std::function<void(const std::string&, float, const std::string&)>;
    void setSensorAlertCallback(SensorAlertCallback cb) { alert_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    HealthMonitorConfig config_;
    bool initialized_ = false;
    
    std::function<void(const SystemHealth&)> health_callback_;
    std::function<void(const std::string&, float, const std::string&)> alert_callback_;
};

}  // namespace swarm::ai

#endif  // HEALTH_MONITOR_HPP