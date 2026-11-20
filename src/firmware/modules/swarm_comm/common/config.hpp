#ifndef SWARM_CONFIG_HPP
#define SWARM_CONFIG_HPP

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <optional>
#include <variant>

namespace swarm::config {

// Configuration value types
using ConfigValue = std::variant<
    bool,
    int32_t,
    uint32_t,
    int64_t,
    uint64_t,
    float,
    double,
    std::string
>;

// Configuration schema definition
struct ConfigSchema {
    std::string name;
    std::string description;
    ConfigValue default_value;
    std::optional<ConfigValue> min_value;
    std::optional<ConfigValue> max_value;
    bool required = false;
    bool reboot_required = false;
};

// Configuration manager interface
class ConfigManager {
public:
    // Load configuration from YAML file
    static bool loadFromFile(const char* filepath);
    
    // Save configuration to YAML file
    static bool saveToFile(const char* filepath);
    
    // Get value with type safety
    template<typename T>
    static std::optional<T> get(const char* key) {
        // Implementation would lookup in internal map
        return std::nullopt;
    }
    
    // Set value with validation
    template<typename T>
    static bool set(const char* key, T value) {
        // Implementation would validate against schema
        return true;
    }
    
    // Check if key exists
    static bool has(const char* key);
    
    // Get all keys with prefix
    static std::vector<std::string> getKeysWithPrefix(const char* prefix);
    
    // Register schema for validation
    static void registerSchema(const ConfigSchema& schema);
    
    // Validate all registered configs
    static bool validateAll();
    
    // Reset to defaults
    static void resetToDefaults();
    
    // Get config as YAML string
    static std::string toYaml();
    
    // Load from YAML string
    static bool fromYaml(const std::string& yaml);
};

// Parameter groups for different subsystems
namespace params {

// Formation control parameters
struct FormationParams {
    float kp_xy = 0.8f;
    float ki_xy = 0.05f;
    float kd_xy = 0.15f;
    float kp_z = 1.0f;
    float ki_z = 0.1f;
    float kd_z = 0.2f;
    float kp_yaw = 2.0f;
    float max_vel_xy = 5.0f;
    float max_vel_z = 2.0f;
    float max_accel_xy = 3.0f;
    float max_accel_z = 2.0f;
    float formation_spacing = 5.0f;
    float lateral_tolerance = 0.5f;
    float vertical_tolerance = 0.3f;
};

// Task allocation parameters
struct TaskAllocParams {
    float auction_rate_hz = 1.0f;
    float bid_timeout_s = 5.0f;
    float consensus_threshold = 0.66f;  // 2/3 majority
    float capability_weight = 1.0f;
    float distance_weight = 0.5f;
    float energy_weight = 0.3f;
};

// AI perception parameters
struct AiPerceptionParams {
    float obstacle_confidence_threshold = 0.7f;
    float landing_confidence_threshold = 0.7f;
    float fault_confidence_threshold = 0.7f;
    int max_detections = 100;
    float nms_threshold = 0.45f;
    int inference_batch_size = 1;
};

// Mesh communication parameters
struct MeshParams {
    float wifi_tx_power_dbm = 20.0f;
    float link_timeout_s = 10.0f;
    float store_forward_max_age_s = 30.0f;
    float qos_update_interval_s = 1.0f;
    uint8_t max_hops = 5;
};

// Safety parameters
struct SafetyParams {
    float geofence_buffer_m = 10.0f;
    float emergency_rtl_altitude_m = 50.0f;
    float battery_critical_pct = 0.15f;
    float battery_rtl_pct = 0.25f;
    uint32_t estop_timeout_ms = 1000;
    float max_wind_mps = 15.0f;
};

// Platform parameters
struct PlatformParams {
    uint8_t motor_count = 4;
    float motor_kv = 850.0f;
    float propeller_size_inch = 15.0f;
    float max_thrust_n = 20.0f;
    float battery_cells = 6;
    float battery_capacity_ah = 22.0f;
    float mass_kg = 2.5f;
};

} // namespace params

// Helper macros for YAML serialization
#define SWARM_CONFIG_SERIALIZE(cls) \
    template<typename Archive> \
    void serialize(Archive& ar) { \
        ar(YOGURT_NVP(kp_xy), YOGURT_NVP(ki_xy), YOGURT_NVP(kd_xy), \
           YOGURT_NVP(kp_z), YOGURT_NVP(ki_z), YOGURT_NVP(kd_z), \
           YOGURT_NVP(kp_yaw), YOGURT_NVP(max_vel_xy), YOGURT_NVP(max_vel_z), \
           YOGURT_NVP(max_accel_xy), YOGURT_NVP(max_accel_z), \
           YOGURT_NVP(formation_spacing), YOGURT_NVP(lateral_tolerance), YOGURT_NVP(vertical_tolerance)); \
    }

} // namespace swarm::config

#endif // SWARM_CONFIG_HPP