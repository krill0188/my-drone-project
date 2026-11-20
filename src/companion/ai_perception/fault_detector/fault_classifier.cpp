#include "fault_classifier.hpp"
#include <chrono>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <cmath>
#include <deque>

namespace swarm::ai {

class FaultClassifier::Impl {
public:
    Impl() = default;
    ~Impl() = default;
    
    struct SensorBuffer {
        std::deque<SensorFrame> frames;
        int max_frames = 100;
        int sequence_length = 50;
        int stride = 5;
        
        void add(const SensorFrame& frame) {
            frames.push_back(frame);
            if (frames.size() > max_frames) {
                frames.pop_front();
            }
        }
        
        bool hasEnough() const {
            return frames.size() >= sequence_length;
        }
        
        // Extract sequence for inference
        std::vector<SensorFrame> getSequence() const {
            std::vector<SensorFrame> seq;
            seq.reserve(sequence_length);
            
            int start = frames.size() - sequence_length;
            for (int i = start; i < (int)frames.size(); i += stride) {
                if (i >= 0 && i < (int)frames.size()) {
                    seq.push_back(frames[i]);
                }
            }
            return seq;
        }
        
        void clear() { frames.clear(); }
        size_t size() const { return frames.size(); }
    };
    
    SensorBuffer buffer_;
    std::vector<float> feature_mean_;
    std::vector<float> feature_std_;
    std::deque<FaultDetection> detection_history_;
    int smoothing_window_ = 5;
    
    bool initialize(const FaultClassifierConfig& config) {
        config_ = config;
        buffer_.max_frames = 100;
        buffer_.sequence_length = config.sequence_length;
        buffer_.stride = config.sequence_stride;
        smoothing_window_ = config.smoothing_window;
        
        // Initialize inference engine
        inference_engine_ = std::make_unique<InferenceEngine>();
        config.engine_config.fault_model_path = config.model_path;
        config.engine_config.confidence_threshold = config.confidence_threshold;
        config.engine_config.input_width = config.num_features;
        config.engine_config.input_height = config.sequence_length;
        
        if (!inference_engine_->initialize(config.engine_config)) {
            std::cerr << "Failed to initialize inference engine for fault classifier" << std::endl;
            return false;
        }
        
        // Set callbacks
        inference_engine_->setInferenceCallback([this](const AIInferenceResult& inf) {
            if (ai_inference_callback_) ai_inference_callback_(inf);
        });
        
        // Initialize normalization (would load from model metadata in practice)
        feature_mean_.assign(config.num_features, 0.0f);
        feature_std_.assign(config.num_features, 1.0f);
        
        initialized_ = true;
        return true;
    }
    
    void addSensorFrame(const SensorFrame& frame) {
        buffer_.add(frame);
    }
    
    FaultDetection detect(uint16_t drone_id, uint64_t timestamp_us) {
        if (!buffer_.hasEnough()) {
            FaultDetection result;
            result.timestamp_us = timestamp_us;
            result.drone_id = drone_id;
            result.total_ms = -1;
            return result;
        }
        
        auto frames = buffer_.getSequence();
        return detectSequence(frames, drone_id, timestamp_us);
    }
    
    FaultDetection detectSequence(const std::vector<SensorFrame>& frames,
                                   uint16_t drone_id, uint64_t timestamp_us) {
        auto start_total = std::chrono::high_resolution_clock::now();
        FaultDetection result;
        result.timestamp_us = timestamp_us;
        result.drone_id = drone_id;
        
        if (!initialized_ || frames.empty()) {
            result.total_ms = -1;
            return result;
        }
        
        // Preprocess: extract features
        auto start_pre = std::chrono::high_resolution_clock::now();
        std::vector<float> feature_sequence = extractFeatures(frames);
        auto end_pre = std::chrono::high_resolution_clock::now();
        result.preprocess_ms = std::chrono::duration<float, std::milli>(end_pre - start_pre).count();
        
        // Run inference
        auto start_inf = std::chrono::high_resolution_clock::now();
        InferenceEngine::InferenceResult inf_result = inference_engine_->inferFault(
            feature_sequence, drone_id, timestamp_us);
        auto end_inf = std::chrono::high_resolution_clock::now();
        result.inference_ms = std::chrono::duration<float, std::milli>(end_inf - start_inf).count();
        
        if (inf_result.total_ms < 0) {
            result.total_ms = -1;
            return result;
        }
        
        // Postprocess
        auto start_post = std::chrono::high_resolution_clock::now();
        
        // Extract class scores
        if (!inf_result.detections.empty()) {
            const auto& det = inf_result.detections[0];
            result.confidence = det.confidence;
            result.fault_type = static_cast<FaultType>(det.class_id);
            result.fault_name = det.class_name;
            result.affected_component = det.class_name;
            result.degradation_pct = det.confidence;
            
            // Get all class scores (would need to modify inference engine to return all)
            result.class_scores.assign(6, 0.0f);
            result.class_scores[det.class_id] = det.confidence;
        }
        
        // Determine severity
        result.severity = determineSeverity(result);
        
        // Determine recommended action
        result.recommended_action = determineAction(result);
        
        // Check if replan needed
        result.requires_replan = (result.severity >= FaultSeverity::CRITICAL) ||
                                 (result.recommended_action >= RecommendedAction::RTL);
        
        // Temporal smoothing
        if (config_.enable_temporal_smoothing) {
            smoothDetection(result);
        }
        
        // Update time since onset
        updateOnsetTime(result);
        
        // Add diagnostics
        addDiagnostics(result, frames);
        
        auto end_post = std::chrono::high_resolution_clock::now();
        result.postprocess_ms = std::chrono::duration<float, std::milli>(end_post - start_post).count();
        
        auto end_total = std::chrono::high_resolution_clock::now();
        result.total_ms = std::chrono::duration<float, std::milli>(end_total - start_total).count();
        
        // Callback
        if (detection_callback_) detection_callback_(result);
        
        // Convert to AIInferenceResult for MAVLink/ROS publishing
        if (ai_inference_callback_) {
            AIInferenceResult ai_inf;
            ai_inf.timestamp_us = timestamp_us;
            ai_inf.drone_id = drone_id;
            ai_inf.type = InferenceType::FAULT;
            ai_inf.class_id = static_cast<int>(result.fault_type);
            ai_inf.class_name = result.fault_name;
            ai_inf.confidence = result.confidence;
            ai_inf.position_enu = Eigen::Vector3f::Zero();  // Would be drone position
            ai_inf.size = Eigen::Vector3f::Zero();
            ai_inf.orientation = Eigen::Quaternionf::Identity();
            
            ai_inf.metadata.latency_ms = static_cast<int>(result.total_ms);
            ai_inf.metadata.model_name = config_.model_type;
            ai_inf.metadata.model_version = "1.0.0";
            ai_inf.metadata.runtime = inf_result.runtime_used;
            
            ai_inf.metadata.fault.fault_type = static_cast<uint8_t>(result.fault_type);
            ai_inf.metadata.fault.severity = static_cast<uint8_t>(result.severity);
            ai_inf.metadata.fault.affected_component = result.affected_component;
            ai_inf.metadata.fault.degradation_pct = result.degradation_pct;
            ai_inf.metadata.fault.requires_replan = result.requires_replan;
            ai_inf.metadata.fault.recommended_action = static_cast<uint8_t>(result.recommended_action);
            
            ai_inference_callback_(ai_inf);
        }
        
        return result;
    }
    
    void clearBuffer() {
        buffer_.clear();
        detection_history_.clear();
    }
    
    int getBufferSize() const {
        return buffer_.size();
    }
    
    bool hasEnoughFrames() const {
        return buffer_.hasEnough();
    }
    
    void setDetectionCallback(std::function<void(const FaultDetection&)> cb) {
        detection_callback_ = cb;
    }
    
    void setAIInferenceCallback(std::function<void(const AIInferenceResult&)> cb) {
        ai_inference_callback_ = cb;
        if (inference_engine_) {
            inference_engine_->setInferenceCallback(cb);
        }
    }

private:
    FaultClassifierConfig config_;
    std::unique_ptr<InferenceEngine> inference_engine_;
    bool initialized_ = false;
    
    std::function<void(const FaultDetection&)> detection_callback_;
    std::function<void(const AIInferenceResult&)> ai_inference_callback_;
    
    std::vector<float> extractFeatures(const std::vector<SensorFrame>& frames) {
        // Extract 128 features per timestep from sensor data
        std::vector<float> features;
        features.reserve(frames.size() * config_.num_features);
        
        for (const auto& frame : frames) {
            std::vector<float> frame_features;
            frame_features.reserve(config_.num_features);
            
            // IMU features (18)
            frame_features.insert(frame_features.end(), {
                frame.accel.x(), frame.accel.y(), frame.accel.z(),
                frame.gyro.x(), frame.gyro.y(), frame.gyro.z(),
                frame.mag.x(), frame.mag.y(), frame.mag.z()
            });
            
            // Add magnitudes
            frame_features.push_back(frame.accel.norm());
            frame_features.push_back(frame.gyro.norm());
            frame_features.push_back(frame.mag.norm());
            
            // GPS features (12)
            frame_features.insert(frame_features.end(), {
                frame.gps_pos_enu.x(), frame.gps_pos_enu.y(), frame.gps_pos_enu.z(),
                frame.gps_vel_enu.x(), frame.gps_vel_enu.y(), frame.gps_vel_enu.z(),
                frame.gps_hdop, (float)frame.gps_satellites, frame.gps_valid ? 1.0f : 0.0f,
                frame.baro_altitude, frame.baro_pressure, 0.0f
            });
            
            // Motor features (4 motors × 4 = 16)
            for (int i = 0; i < config_.num_motors; ++i) {
                float rpm = i < frame.motor_rpm.size() ? frame.motor_rpm[i] : 0.0f;
                float curr = i < frame.motor_current.size() ? frame.motor_current[i] : 0.0f;
                float volt = i < frame.motor_voltage.size() ? frame.motor_voltage[i] : 0.0f;
                float temp = i < frame.motor_temp.size() ? frame.motor_temp[i] : 0.0f;
                frame_features.insert(frame_features.end(), {rpm, curr, volt, temp});
            }
            
            // ESC temps (4)
            for (int i = 0; i < config_.num_motors; ++i) {
                float temp = i < frame.esc_temp.size() ? frame.esc_temp[i] : 0.0f;
                frame_features.push_back(temp);
            }
            
            // Battery features (6 cells + 5 = 11)
            frame_features.insert(frame_features.end(), {
                frame.battery_voltage, frame.battery_current, frame.battery_temp,
                frame.battery_soc, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f
            });
            for (int i = 0; i < config_.num_cells; ++i) {
                float v = i < frame.cell_voltages.size() ? frame.cell_voltages[i] : 0.0f;
                frame_features.push_back(v);
            }
            
            // Communication features (4)
            frame_features.insert(frame_features.end(), {
                frame.comm_rssi, frame.comm_snr, frame.comm_packet_loss, frame.comm_latency_ms
            });
            
            // VIO features (7)
            frame_features.insert(frame_features.end(), {
                frame.vio_pos_enu.x(), frame.vio_pos_enu.y(), frame.vio_pos_enu.z(),
                frame.vio_attitude.x(), frame.vio_attitude.y(), frame.vio_attitude.z(),
                frame.vio_valid ? 1.0f : 0.0f
            });
            
            // Airspeed (2)
            frame_features.insert(frame_features.end(), {
                frame.airspeed, frame.airspeed_valid ? 1.0f : 0.0f
            });
            
            // Derived features (fill remaining)
            // Accel variance, gyro variance, motor rpm variance, cell voltage variance, etc.
            while ((int)frame_features.size() < config_.num_features) {
                frame_features.push_back(0.0f);
            }
            
            // Normalize
            for (int i = 0; i < config_.num_features && i < (int)frame_features.size(); ++i) {
                if (i < (int)feature_mean_.size() && i < (int)feature_std_.size() && feature_std_[i] > 0) {
                    frame_features[i] = (frame_features[i] - feature_mean_[i]) / feature_std_[i];
                }
            }
            
            features.insert(features.end(), frame_features.begin(), frame_features.end());
        }
        
        return features;
    }
    
    FaultSeverity determineSeverity(const FaultDetection& det) {
        if (det.confidence >= 0.9f) return FaultSeverity::EMERGENCY;
        if (det.confidence >= 0.75f) return FaultSeverity::CRITICAL;
        if (det.confidence >= 0.5f) return FaultSeverity::WARNING;
        return FaultSeverity::INFO;
    }
    
    RecommendedAction determineAction(const FaultDetection& det) {
        if (det.severity == FaultSeverity::EMERGENCY) return RecommendedAction::EMERGENCY_LAND;
        if (det.severity == FaultSeverity::CRITICAL) return RecommendedAction::RTL;
        if (det.severity == FaultSeverity::WARNING) return RecommendedAction::DEGRADED;
        return RecommendedAction::CONTINUE;
    }
    
    void smoothDetection(FaultDetection& det) {
        detection_history_.push_back(det);
        if (detection_history_.size() > smoothing_window_) {
            detection_history_.pop_front();
        }
        
        if (detection_history_.size() >= 3) {
            // Weighted average of recent detections
            float weight_sum = 0;
            float conf_sum = 0;
            float deg_sum = 0;
            
            for (size_t i = 0; i < detection_history_.size(); ++i) {
                float w = (i + 1) / (float)detection_history_.size();
                conf_sum += w * detection_history_[i].confidence;
                deg_sum += w * detection_history_[i].degradation_pct;
                weight_sum += w;
            }
            
            det.confidence = conf_sum / weight_sum;
            det.degradation_pct = deg_sum / weight_sum;
            
            // Re-evaluate severity/action with smoothed confidence
            det.severity = determineSeverity(det);
            det.recommended_action = determineAction(det);
            det.requires_replan = (det.severity >= FaultSeverity::CRITICAL) ||
                                  (det.recommended_action >= RecommendedAction::RTL);
        }
    }
    
    void updateOnsetTime(FaultDetection& det) {
        if (detection_history_.empty()) {
            det.time_since_onset_s = 0;
            return;
        }
        
        // Find first detection of same fault type with high confidence
        for (auto it = detection_history_.rbegin(); it != detection_history_.rend(); ++it) {
            if (it->fault_type == det.fault_type && it->confidence > 0.5f) {
                float dt = (det.timestamp_us - it->timestamp_us) / 1e6f;
                det.time_since_onset_s = std::max(0.0f, dt);
                break;
            }
        }
    }
    
    void addDiagnostics(FaultDetection& det, const std::vector<SensorFrame>& frames) {
        if (frames.empty()) return;
        
        const auto& latest = frames.back();
        
        switch (det.fault_type) {
            case FaultType::IMU_DRIFT: {
                // Gyro bias estimate
                float gyro_mag = latest.gyro.norm();
                det.diagnostics["gyro_magnitude"] = gyro_mag;
                det.diagnostics["gyro_bias_estimate"] = gyro_mag * 0.01f;  // Simplified
                break;
            }
            case FaultType::GPS_SPOOF: {
                // Position jump detection
                if (frames.size() >= 2) {
                    Eigen::Vector3f pos_diff = frames.back().gps_pos_enu - frames[frames.size()-2].gps_pos_enu;
                    det.diagnostics["position_jump_m"] = pos_diff.norm();
                    det.diagnostics["hdop"] = latest.gps_hdop;
                    det.diagnostics["satellites"] = (float)latest.gps_satellites;
                }
                break;
            }
            case FaultType::MOTOR_DEGRADE: {
                // RPM vs current analysis
                if (!latest.motor_rpm.empty() && !latest.motor_current.empty()) {
                    float rpm_sum = 0, curr_sum = 0;
                    for (size_t i = 0; i < latest.motor_rpm.size(); ++i) {
                        rpm_sum += latest.motor_rpm[i];
                        curr_sum += latest.motor_current[i];
                    }
                    det.diagnostics["avg_rpm"] = rpm_sum / latest.motor_rpm.size();
                    det.diagnostics["avg_current"] = curr_sum / latest.motor_current.size();
                    det.diagnostics["efficiency"] = rpm_sum / (curr_sum + 1e-6);
                }
                break;
            }
            case FaultType::BATTERY_IMBALANCE: {
                if (!latest.cell_voltages.empty()) {
                    float max_v = *std::max_element(latest.cell_voltages.begin(), latest.cell_voltages.end());
                    float min_v = *std::min_element(latest.cell_voltages.begin(), latest.cell_voltages.end());
                    det.diagnostics["max_cell_v"] = max_v;
                    det.diagnostics["min_cell_v"] = min_v;
                    det.diagnostics["cell_diff_v"] = max_v - min_v;
                    det.diagnostics["total_v"] = latest.battery_voltage;
                }
                break;
            }
            case FaultType::SENSOR_FAULT: {
                det.diagnostics["gps_valid"] = latest.gps_valid ? 1.0f : 0.0f;
                det.diagnostics["vio_valid"] = latest.vio_valid ? 1.0f : 0.0f;
                det.diagnostics["baro_alt"] = latest.baro_altitude;
                break;
            }
            case FaultType::COMM_DEGRADE: {
                det.diagnostics["rssi"] = latest.comm_rssi;
                det.diagnostics["snr"] = latest.comm_snr;
                det.diagnostics["packet_loss"] = latest.comm_packet_loss;
                det.diagnostics["latency_ms"] = latest.comm_latency_ms;
                break;
            }
        }
    }
};

FaultClassifier::FaultClassifier() : pimpl_(std::make_unique<Impl>()) {}
FaultClassifier::~FaultClassifier() = default;
FaultClassifier::FaultClassifier(FaultClassifier&&) noexcept = default;
FaultClassifier& FaultClassifier::operator=(FaultClassifier&&) noexcept = default;

bool FaultClassifier::initialize(const FaultClassifierConfig& config) {
    config_ = config;
    return pimpl_->initialize(config);
}

void FaultClassifier::addSensorFrame(const SensorFrame& frame) {
    pimpl_->addSensorFrame(frame);
}

FaultClassifier::FaultDetection FaultClassifier::detect(uint16_t drone_id, uint64_t timestamp_us) {
    return pimpl_->detect(drone_id, timestamp_us);
}

FaultClassifier::FaultDetection FaultClassifier::detectSequence(const std::vector<SensorFrame>& frames,
                                                                 uint16_t drone_id, uint64_t timestamp_us) {
    return pimpl_->detectSequence(frames, drone_id, timestamp_us);
}

void FaultClassifier::clearBuffer() {
    pimpl_->clearBuffer();
}

int FaultClassifier::getBufferSize() const {
    return pimpl_->getBufferSize();
}

bool FaultClassifier::hasEnoughFrames() const {
    return pimpl_->hasEnoughFrames();
}

bool FaultClassifier::isInitialized() const {
    return initialized_;
}

InferenceEngine* FaultClassifier::getInferenceEngine() {
    return inference_engine_.get();
}

void FaultClassifier::setDetectionCallback(DetectionCallback cb) {
    detection_callback_ = cb;
    pimpl_->setDetectionCallback(cb);
}

void FaultClassifier::setAIInferenceCallback(AIInferenceCallback cb) {
    ai_inference_callback_ = cb;
    pimpl_->setAIInferenceCallback(cb);
}

}  // namespace swarm::ai