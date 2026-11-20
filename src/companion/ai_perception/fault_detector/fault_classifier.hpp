#ifndef FAULT_CLASSIFIER_HPP
#define FAULT_CLASSIFIER_HPP

#include "../common/ai_inference.hpp"
#include "../model_runtime/inference_engine.hpp"
#include <vector>
#include <string>
#include <memory>
#include <functional>
#include <deque>

namespace swarm::ai {

// Fault types (matching FaultType enum in ai_inference.hpp)
static const std::vector<std::string> FAULT_CLASS_NAMES = {
    "imu_drift",          // 0
    "gps_spoof",          // 1
    "motor_degrade",      // 2
    "battery_imbalance",  // 3
    "sensor_fault",       // 4
    "comm_degrade"        // 5
};

// Sensor data structure for time series
struct SensorFrame {
    uint64_t timestamp_us;
    
    // IMU (accel + gyro + mag)
    Eigen::Vector3f accel;
    Eigen::Vector3f gyro;
    Eigen::Vector3f mag;
    
    // GPS
    Eigen::Vector3f gps_pos_enu;
    Eigen::Vector3f gps_vel_enu;
    float gps_hdop = 0.0f;
    int gps_satellites = 0;
    bool gps_valid = false;
    
    // Barometer
    float baro_altitude = 0.0f;
    float baro_pressure = 0.0f;
    
    // Motor/ESC
    std::vector<float> motor_rpm;      // Per motor
    std::vector<float> motor_current;  // Per motor
    std::vector<float> motor_voltage;  // Per motor
    std::vector<float> motor_temp;     // Per motor
    std::vector<float> esc_temp;       // Per ESC
    
    // Battery
    float battery_voltage = 0.0f;
    float battery_current = 0.0f;
    std::vector<float> cell_voltages;  // Per cell
    float battery_temp = 0.0f;
    float battery_soc = 0.0f;          // State of charge 0-1
    
    // Communication
    float comm_rssi = 0.0f;
    float comm_snr = 0.0f;
    float comm_packet_loss = 0.0f;
    float comm_latency_ms = 0.0f;
    
    // Optical flow / VIO
    Eigen::Vector3f vio_pos_enu;
    Eigen::Quaternionf vio_attitude;
    bool vio_valid = false;
    
    // Airspeed (if available)
    float airspeed = 0.0f;
    bool airspeed_valid = false;
};

struct FaultClassifierConfig {
    // Model
    std::string model_path;
    std::string model_type = "lstm";  // "lstm", "gru", "tcn", "transformer"
    
    // Inference engine config
    InferenceEngineConfig engine_config;
    
    // Sequence parameters
    int sequence_length = 50;       // Number of timesteps (at 50Hz = 1 second)
    int sequence_stride = 5;        // Stride between frames
    int num_features = 128;         // Feature dimension per timestep
    int num_motors = 4;             // Number of motors
    int num_cells = 6;              // Number of battery cells
    
    // Detection thresholds
    float confidence_threshold = 0.7f;
    float imu_drift_threshold = 0.05f;     // rad/s drift rate
    float gps_spoof_threshold = 5.0f;      // Position jump (m)
    float motor_degrade_threshold = 0.15f; // 15% performance loss
    float battery_imbalance_threshold = 0.05f; // 5% cell voltage diff
    float sensor_fault_threshold = 3.0f;   // 3-sigma outlier
    float comm_degrade_threshold = 0.3f;   // 30% packet loss
    
    // Normalization
    std::vector<float> feature_mean;
    std::vector<float> feature_std;
    
    // Temporal smoothing
    bool enable_temporal_smoothing = true;
    int smoothing_window = 5;  // frames
    
    // Performance
    int target_fps = 10;
    bool async_inference = true;
};

struct FaultDetection {
    uint64_t timestamp_us;
    uint16_t drone_id;
    
    // Primary classification
    FaultType fault_type = FaultType::SENSOR_FAULT;
    std::string fault_name;
    float confidence = 0.0f;
    FaultSeverity severity = FaultSeverity::INFO;
    
    // Affected component
    std::string affected_component;
    float degradation_pct = 0.0f;  // 0.0 to 1.0
    
    // Detailed scores per fault type
    std::vector<float> class_scores;  // Size = 6
    
    // Recommended action
    RecommendedAction recommended_action = RecommendedAction::CONTINUE;
    
    // Requires replan
    bool requires_replan = false;
    
    // Time since onset
    float time_since_onset_s = 0.0f;
    
    // Additional diagnostics
    std::map<std::string, float> diagnostics;
    
    // Performance
    float total_ms = 0.0f;
    float preprocess_ms = 0.0f;
    float inference_ms = 0.0f;
    float postprocess_ms = 0.0f;
    
    bool isValid() const {
        return confidence > 0.0f && total_ms > 0;
    }
};

class FaultClassifier {
public:
    FaultClassifier();
    ~FaultClassifier();
    
    FaultClassifier(const FaultClassifier&) = delete;
    FaultClassifier& operator=(const FaultClassifier&) = delete;
    FaultClassifier(FaultClassifier&&) noexcept;
    FaultClassifier& operator=(FaultClassifier&&) noexcept;
    
    // Initialize
    bool initialize(const FaultClassifierConfig& config);
    
    // Add sensor frame to buffer
    void addSensorFrame(const SensorFrame& frame);
    
    // Run fault detection on current buffer
    FaultDetection detect(uint16_t drone_id, uint64_t timestamp_us);
    
    // Run detection with explicit frame sequence
    FaultDetection detectSequence(const std::vector<SensorFrame>& frames,
                                   uint16_t drone_id, uint64_t timestamp_us);
    
    // Clear buffer
    void clearBuffer();
    
    // Get buffer status
    int getBufferSize() const;
    bool hasEnoughFrames() const;
    
    // Get config
    const FaultClassifierConfig& getConfig() const { return config_; }
    
    // Check if initialized
    bool isInitialized() const { return initialized_; }
    
    // Get inference engine
    InferenceEngine* getInferenceEngine() { return inference_engine_.get(); }
    
    // Set callback
    using DetectionCallback = std::function<void(const FaultDetection&)>;
    void setDetectionCallback(DetectionCallback cb) { detection_callback_ = cb; }
    
    using AIInferenceCallback = std::function<void(const AIInferenceResult&)>;
    void setAIInferenceCallback(AIInferenceCallback cb) { ai_inference_callback_ = cb; }

private:
    class Impl;
    std::unique_ptr<Impl> pimpl_;
    FaultClassifierConfig config_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    bool initialized_ = false;
    
    std::function<void(const FaultDetection&)> detection_callback_;
    std::function<void(const AIInferenceResult&)> ai_inference_callback_;
};

}  // namespace swarm::ai

#endif  // FAULT_CLASSIFIER_HPP