#ifndef FAULT_DETECTOR_HPP
#define FAULT_DETECTOR_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <optional>
#include <deque>
#include <array>
#include <opencv2/opencv.hpp>
#include "inference_engine.hpp"

namespace swarm::ai {

enum class FaultType : uint8_t {
    IMU_DRIFT = 0,
    GPS_SPOOF = 1,
    MOTOR_DEGRADE = 2,
    BATTERY_IMBALANCE = 3,
    SENSOR_FAULT = 4,
    COMM_DEGRADE = 5,
};

enum class FaultSeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3,
};

enum class RecommendedAction : uint8_t {
    CONTINUE = 0,
    DEGRADED = 1,
    RTL = 2,
    EMERGENCY_LAND = 3,
};

struct FaultReport {
    FaultType type;
    std::string affected_component;  // e.g., "motor_3", "gps_1", "imu_0"
    FaultSeverity severity;
    float degradation_pct;           // 0.0 - 1.0
    float confidence;
    bool requires_replan;
    RecommendedAction recommended_action;
    std::vector<float> feature_vector;  // Raw features for debugging
    uint64_t timestamp_us;
    uint64_t detection_latency_ms;
};

struct FaultDetectorConfig {
    InferenceEngine::ModelConfig model_config;
    float imu_drift_threshold_deg_per_s = 0.5f;
    float gps_spoof_position_jump_m = 50.0f;
    float motor_current_deviation_pct = 20.0f;
    float battery_cell_imbalance_v = 0.1f;
    float comm_loss_rate_threshold = 0.3f;
    int lstm_sequence_length = 50;  // 50 samples @ 10Hz = 5s window
    float min_confidence = 0.7f;
};

class FaultDetector {
public:
    FaultDetector() = default;
    
    bool init(const FaultDetectorConfig& config);
    
    // Update with sensor data (call at 10Hz)
    void update(const swarm_msgs::msg::AgentState& state,
                const std::vector<swarm_msgs::msg::CommQoS>& mesh_qos);
    
    // Get current fault reports
    const std::vector<FaultReport>& getFaultReports() const { return _fault_reports; }
    
    // Get faults by type
    std::vector<FaultReport> getFaultsByType(FaultType type) const;
    
    // Get highest severity fault
    std::optional<FaultReport> getHighestSeverityFault() const;
    
    // Check if any critical/emergency faults
    bool hasCriticalFaults() const;

private:
    FaultDetectorConfig _config;
    std::shared_ptr<InferenceEngine> _inference_engine;
    std::vector<FaultReport> _fault_reports;
    
    // Sensor history for LSTM
    std::deque<std::array<float, 12>> _imu_history;      // accel(3) + gyro(3) + mag(3) + baro(1) + temp(1) + voltage(1)
    std::deque<std::array<float, 8>> _gps_history;       // lat, lon, alt, vel_n, vel_e, vel_d, hdop, sats
    std::deque<std::array<float, 10>> _motor_history;    // 8 motors current + voltage + temp
    std::deque<std::array<float, 6>> _battery_history;   // voltage, current, cell1-4, temp
    std::deque<std::array<float, 4>> _comm_history;      // latency, loss, bw, rssi
    
    // Statistical baselines
    struct Baseline {
        std::array<float, 12> imu_mean, imu_std;
        std::array<float, 8> gps_mean, gps_std;
        std::array<float, 10> motor_mean, motor_std;
        std::array<float, 6> battery_mean, battery_std;
        std::array<float, 4> comm_mean, comm_std;
        bool valid = false;
    } _baseline;
    
    // Private methods
    void extractFeatures(const swarm_msgs::msg::AgentState& state,
                         const std::vector<swarm_msgs::msg::CommQoS>& mesh_qos,
                         std::vector<float>& features);
    void updateBaselines();
    void runLstmInference(const std::vector<float>& features);
    void statisticalFaultCheck(const swarm_msgs::msg::AgentState& state,
                               const std::vector<swarm_msgs::msg::CommQoS>& mesh_qos);
    void classifyFault(const std::vector<float>& lstm_output, 
                       const std::vector<float>& stat_features,
                       FaultReport& report);
    RecommendedAction determineAction(FaultType type, FaultSeverity severity, float degradation);
};

} // namespace swarm::ai

#endif // FAULT_DETECTOR_HPP