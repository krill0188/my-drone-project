#ifndef AI_INFERENCE_HPP
#define AI_INFERENCE_HPP

#include <string>
#include <vector>
#include <cstdint>
#include <Eigen/Dense>

namespace swarm::ai {

// Forward declarations
enum class InferenceType : uint8_t {
    OBSTACLE = 0,
    LANDING_SITE = 1,
    FAULT = 2,
    ANOMALY = 3
};

enum class ObstacleClass : uint16_t {
    UNKNOWN = 0,
    POWER_LINE = 1,
    TOWER = 2,
    BUILDING = 3,
    TREE = 4,
    BIRD = 5,
    DRONE = 6,
    AIRCRAFT = 7,
    VEHICLE = 8,
    PERSON = 9,
    WIND_TURBINE = 10,
    CRANE = 11,
    ANTENNA = 12,
    BALLOON = 13,
    KITE = 14,
    WIRE = 15,
    CABLE = 16,
    POLE = 17,
    FENCE = 18,
    WALL = 19
};

enum class ThreatLevel : uint8_t {
    LOW = 0,
    MEDIUM = 1,
    HIGH = 2,
    CRITICAL = 3
};

enum class FaultType : uint8_t {
    IMU_DRIFT = 0,
    GPS_SPOOF = 1,
    MOTOR_DEGRADE = 2,
    BATTERY_IMBALANCE = 3,
    SENSOR_FAULT = 4,
    COMM_DEGRADE = 5
};

enum class FaultSeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3
};

enum class RecommendedAction : uint8_t {
    CONTINUE = 0,
    DEGRADED = 1,
    RTL = 2,
    EMERGENCY_LAND = 3
};

enum class SurfaceType : uint8_t {
    GRASS = 0,
    CONCRETE = 1,
    DIRT = 2,
    SAND = 3,
    WATER = 4,
    UNKNOWN = 5
};

// Obstacle-specific metadata
struct ObstacleMeta {
    ThreatLevel threat_level = ThreatLevel::LOW;
    Eigen::Vector3f relative_velocity = Eigen::Vector3f::Zero();
    float time_to_collision_s = -1.0f;
    bool requires_immediate_avoidance = false;
    
    // Future extensions
    float closest_approach_dist_m = -1.0f;
    Eigen::Vector3f predicted_position_enu = Eigen::Vector3f::Zero();
    bool is_moving = false;
};

// Landing site-specific metadata
struct LandingMeta {
    float slope_deg = 0.0f;
    float clearance_radius_m = 0.0f;
    SurfaceType surface_type = SurfaceType::UNKNOWN;
    float firmness_score = 0.0f;  // 0.0 = soft, 1.0 = hard
    
    // Future extensions
    float roughness_index = 0.0f;
    bool has_obstructions = false;
    float wind_exposure = 0.0f;
};

// Fault-specific metadata
struct FaultMeta {
    FaultType fault_type = FaultType::SENSOR_FAULT;
    FaultSeverity severity = FaultSeverity::INFO;
    std::string affected_component;
    float degradation_pct = 0.0f;  // 0.0 to 1.0
    bool requires_replan = false;
    RecommendedAction recommended_action = RecommendedAction::CONTINUE;
    
    // Future extensions
    float time_since_onset_s = 0.0f;
    std::vector<std::string> affected_sensors;
    bool is_recoverable = true;
};

// Anomaly-specific metadata (for future use)
struct AnomalyMeta {
    std::string anomaly_type;
    float anomaly_score = 0.0f;
    std::vector<float> feature_vector;
    bool requires_investigation = false;
};

// Common inference metadata
struct InferenceMetadata {
    uint64_t timestamp_us = 0;
    int latency_ms = 0;
    std::string model_name;
    std::string model_version;
    std::string runtime;  // "tensorrt_fp16", "tensorrt_int8", "onnxruntime_cuda", "onnxruntime_cpu"
    
    // Type-specific metadata (only one will be populated based on type)
    ObstacleMeta obstacle;
    LandingMeta landing;
    FaultMeta fault;
    AnomalyMeta anomaly;
    
    // Future: sensor provenance
    std::string sensor_source;  // "stereo_left", "stereo_right", "downward", "lidar_2d", "lidar_3d"
    uint32_t sensor_timestamp_us = 0;
};

// Main AI inference result structure
struct AIInferenceResult {
    uint64_t timestamp_us = 0;
    uint16_t drone_id = 0;
    InferenceType type = InferenceType::OBSTACLE;
    int class_id = 0;
    std::string class_name;
    float confidence = 0.0f;  // [0, 1]
    
    // 3D position in ENU frame (meters)
    Eigen::Vector3f position_enu = Eigen::Vector3f::Zero();
    
    // 3D bounding box size (dx, dy, dz in meters)
    Eigen::Vector3f size = Eigen::Vector3f::Zero();
    
    // 3D bounding box orientation as quaternion (w, x, y, z)
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
    
    InferenceMetadata metadata;
    
    // Validation
    bool isValid() const {
        return confidence >= 0.0f && confidence <= 1.0f &&
               type >= InferenceType::OBSTACLE && type <= InferenceType::ANOMALY &&
               orientation.norm() > 0.9f && orientation.norm() < 1.1f;
    }
    
    // Get type-specific metadata
    const ObstacleMeta& getObstacleMeta() const { return metadata.obstacle; }
    const LandingMeta& getLandingMeta() const { return metadata.landing; }
    const FaultMeta& getFaultMeta() const { return metadata.fault; }
    const AnomalyMeta& getAnomalyMeta() const { return metadata.anomaly; }
    
    // Check if requires immediate action
    bool requiresImmediateAction() const {
        switch (type) {
            case InferenceType::OBSTACLE:
                return metadata.obstacle.requires_immediate_avoidance ||
                       metadata.obstacle.threat_level >= ThreatLevel::CRITICAL;
            case InferenceType::FAULT:
                return metadata.fault.severity >= FaultSeverity::CRITICAL ||
                       metadata.fault.recommended_action >= RecommendedAction::RTL;
            case InferenceType::LANDING_SITE:
                return false;  // Landing sites don't require immediate action
            case InferenceType::ANOMALY:
                return metadata.anomaly.requires_investigation;
        }
        return false;
    }
};

// Batch inference result for multiple detections
struct AIInferenceBatch {
    uint64_t timestamp_us = 0;
    uint16_t drone_id = 0;
    std::vector<AIInferenceResult> inferences;
    
    // Statistics
    int obstacle_count = 0;
    int landing_count = 0;
    int fault_count = 0;
    int anomaly_count = 0;
    
    void addInference(const AIInferenceResult& inf) {
        inferences.push_back(inf);
        switch (inf.type) {
            case InferenceType::OBSTACLE: obstacle_count++; break;
            case InferenceType::LANDING_SITE: landing_count++; break;
            case InferenceType::FAULT: fault_count++; break;
            case InferenceType::ANOMALY: anomaly_count++; break;
        }
    }
    
    void clear() {
        inferences.clear();
        obstacle_count = landing_count = fault_count = anomaly_count = 0;
    }
};

// Model info for management
struct ModelInfo {
    std::string name;
    std::string version;
    std::string runtime;  // "tensorrt", "onnxruntime"
    std::string precision;  // "fp32", "fp16", "int8"
    std::vector<int64_t> input_shape;  // e.g., {1, 3, 640, 640}
    std::vector<std::string> output_names;
    std::vector<std::vector<int64_t>> output_shapes;
    size_t engine_size_bytes = 0;
    float avg_latency_ms = 0.0f;
    float peak_memory_mb = 0.0f;
    std::string checksum;  // SHA256 of model file
    uint64_t last_updated_us = 0;
};

// Inference engine configuration
struct InferenceEngineConfig {
    // Model paths
    std::string obstacle_model_path;
    std::string landing_model_path;
    std::string fault_model_path;
    std::string anomaly_model_path;
    
    // Runtime preferences
    std::string preferred_runtime = "tensorrt";  // "tensorrt", "onnxruntime"
    std::string preferred_precision = "fp16";    // "fp32", "fp16", "int8"
    
    // Performance settings
    int max_batch_size = 1;
    int max_workspace_size_mb = 1024;
    bool enable_fp16 = true;
    bool enable_int8 = false;
    std::string calibration_cache_path;
    
    // Inference settings
    float confidence_threshold = 0.5f;
    float nms_threshold = 0.45f;
    int max_detections = 100;
    
    // Device settings
    int gpu_device_id = 0;
    int dla_core = -1;  // -1 = auto, 0 or 1 for DLA
    
    // Fallback
    bool allow_fallback = true;
    std::string fallback_runtime = "onnxruntime";
};

// Inference statistics for monitoring
struct InferenceStats {
    uint64_t total_inferences = 0;
    uint64_t successful_inferences = 0;
    uint64_t failed_inferences = 0;
    uint64_t fallback_inferences = 0;
    
    float avg_latency_ms = 0.0f;
    float min_latency_ms = 0.0f;
    float max_latency_ms = 0.0f;
    float p99_latency_ms = 0.0f;
    
    uint64_t obstacle_detections = 0;
    uint64_t landing_detections = 0;
    uint64_t fault_detections = 0;
    uint64_t anomaly_detections = 0;
    
    float avg_confidence = 0.0f;
    
    void recordInference(float latency_ms, bool success, bool fallback, 
                         InferenceType type, float confidence) {
        total_inferences++;
        if (success) successful_inferences++;
        else failed_inferences++;
        if (fallback) fallback_inferences++;
        
        // Update latency stats (running average)
        if (total_inferences == 1) {
            min_latency_ms = max_latency_ms = avg_latency_ms = latency_ms;
        } else {
            min_latency_ms = std::min(min_latency_ms, latency_ms);
            max_latency_ms = std::max(max_latency_ms, latency_ms);
            avg_latency_ms = 0.99f * avg_latency_ms + 0.01f * latency_ms;
        }
        
        switch (type) {
            case InferenceType::OBSTACLE: obstacle_detections++; break;
            case InferenceType::LANDING_SITE: landing_detections++; break;
            case InferenceType::FAULT: fault_detections++; break;
            case InferenceType::ANOMALY: anomaly_detections++; break;
        }
        
        avg_confidence = 0.99f * avg_confidence + 0.01f * confidence;
    }
    
    bool meetsLatencyRequirement(float max_allowed_ms) const {
        return p99_latency_ms < max_allowed_ms;
    }
};

// Utility functions
inline std::string inferenceTypeToString(InferenceType type) {
    switch (type) {
        case InferenceType::OBSTACLE: return "OBSTACLE";
        case InferenceType::LANDING_SITE: return "LANDING_SITE";
        case InferenceType::FAULT: return "FAULT";
        case InferenceType::ANOMALY: return "ANOMALY";
    }
    return "UNKNOWN";
}

inline std::string threatLevelToString(ThreatLevel level) {
    switch (level) {
        case ThreatLevel::LOW: return "LOW";
        case ThreatLevel::MEDIUM: return "MEDIUM";
        case ThreatLevel::HIGH: return "HIGH";
        case ThreatLevel::CRITICAL: return "CRITICAL";
    }
    return "UNKNOWN";
}

inline std::string faultTypeToString(FaultType type) {
    switch (type) {
        case FaultType::IMU_DRIFT: return "IMU_DRIFT";
        case FaultType::GPS_SPOOF: return "GPS_SPOOF";
        case FaultType::MOTOR_DEGRADE: return "MOTOR_DEGRADE";
        case FaultType::BATTERY_IMBALANCE: return "BATTERY_IMBALANCE";
        case FaultType::SENSOR_FAULT: return "SENSOR_FAULT";
        case FaultType::COMM_DEGRADE: return "COMM_DEGRADE";
    }
    return "UNKNOWN";
}

inline std::string faultSeverityToString(FaultSeverity sev) {
    switch (sev) {
        case FaultSeverity::INFO: return "INFO";
        case FaultSeverity::WARNING: return "WARNING";
        case FaultSeverity::CRITICAL: return "CRITICAL";
        case FaultSeverity::EMERGENCY: return "EMERGENCY";
    }
    return "UNKNOWN";
}

inline std::string recommendedActionToString(RecommendedAction action) {
    switch (action) {
        case RecommendedAction::CONTINUE: return "CONTINUE";
        case RecommendedAction::DEGRADED: return "DEGRADED";
        case RecommendedAction::RTL: return "RTL";
        case RecommendedAction::EMERGENCY_LAND: return "EMERGENCY_LAND";
    }
    return "UNKNOWN";
}

inline std::string surfaceTypeToString(SurfaceType type) {
    switch (type) {
        case SurfaceType::GRASS: return "GRASS";
        case SurfaceType::CONCRETE: return "CONCRETE";
        case SurfaceType::DIRT: return "DIRT";
        case SurfaceType::SAND: return "SAND";
        case SurfaceType::WATER: return "WATER";
        case SurfaceType::UNKNOWN: return "UNKNOWN";
    }
    return "UNKNOWN";
}

}  // namespace swarm::ai

#endif  // AI_INFERENCE_HPP