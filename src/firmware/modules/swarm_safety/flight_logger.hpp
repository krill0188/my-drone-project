#ifndef FLIGHT_LOGGER_HPP
#define FLIGHT_LOGGER_HPP

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_timestamps.h>
#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_imu.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_barometer.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/collision_report.h>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/iridiumsbd_status.h>
#include <uORB/topics/telemetry_status.h>
#include <uORB/topics/log_message.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>
#include <array>

namespace swarm::firmware {

// ULog message IDs for custom swarm messages
enum class UlogMessageId : uint16_t {
    SWARM_FORMATION_STATE = 0x8000,
    SWARM_TASK_ALLOCATION = 0x8001,
    SWARM_AI_INFERENCE = 0x8002,
    SWARM_COMM_STATUS = 0x8003,
    SWARM_SYSTEM_HEALTH = 0x8004,
    SWARM_SAFETY_EVENT = 0x8005,
    SWARM_TELEMETRY_SYNC = 0x8006,
};

// Flight logger configuration
struct FlightLoggerConfig {
    // Logging rates (Hz)
    int sensor_log_rate_hz = 100;          // IMU, mag, baro
    int gps_log_rate_hz = 10;              // GPS position
    int attitude_log_rate_hz = 100;        // Attitude, rates
    int position_log_rate_hz = 10;         // Local/global position
    int actuator_log_rate_hz = 100;        // Actuator outputs
    int battery_log_rate_hz = 10;          // Battery status
    int estimator_log_rate_hz = 10;        // Estimator status
    int status_log_rate_hz = 10;           // Vehicle status, mode
    int safety_log_rate_hz = 100;          // Safety events
    
    // Swarm-specific rates
    int formation_log_rate_hz = 10;
    int task_alloc_log_rate_hz = 1;
    int ai_inference_log_rate_hz = 20;
    int comm_status_log_rate_hz = 10;
    int telemetry_sync_log_rate_hz = 10;
    
    // File settings
    char log_dir[64] = "/fs/microsd/log";
    int max_log_size_mb = 100;
    int max_log_duration_min = 60;
    bool enable_compression = false;
    
    // Safety
    bool log_until_disarm = true;
    bool log_while_armed_only = true;
    int min_free_space_mb = 50;
};

class FlightLogger : public ModuleBase<FlightLogger>, public ModuleParams {
public:
    FlightLogger();
    ~FlightLogger();
    
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    
    // Module interface
    int init() override;
    void run() override;
    
    // Start/stop logging
    bool startLogging();
    void stopLogging();
    bool isLogging() const { return _logging_active; }
    
    // Log specific data
    void logSensorData();
    void logGpsData();
    void logAttitudeData();
    void logPositionData();
    void logActuatorData();
    void logBatteryData();
    void logEstimatorData();
    void logStatusData();
    void logSafetyEvent(const char *event_type, const char *details);
    
    // Swarm-specific logging
    void logFormationState();
    void logTaskAllocation();
    void logAiInference();
    void logCommStatus();
    void logTelemetrySync();
    
    // ULog management
    bool openLogFile();
    void closeLogFile();
    void rotateLogFile();
    bool checkDiskSpace();
    void cleanupOldLogs();
    
    // Time synchronization
    void addTelemetrySync(uint64_t firmware_time_us, uint64_t companion_time_us, 
                          uint64_t gps_time_us, double offset_us, double drift_ppm);
    void requestTimeSync();
    
    // Getters
    const char* getCurrentLogPath() const { return _current_log_path; }
    uint64_t getLoggedBytes() const { return _logged_bytes; }
    uint64_t getLogDurationUs() const { return _log_duration_us; }
    
    // Callbacks
    using LogRotateCallback = std::function<void(const char* old_path, const char* new_path)>;
    void setLogRotateCallback(LogRotateCallback cb) { _log_rotate_cb = cb; }
    
    using SyncCallback = std::function<void(uint64_t firmware_time_us, uint64_t companion_time_us, 
                                            uint64_t gps_time_us, double offset_us, double drift_ppm)>;
    void setSyncCallback(SyncCallback cb) { _sync_cb = cb; }

private:
    // uORB subscriptions
    orb_advert_t _sensor_combined_pub{nullptr};
    orb_advert_t _vehicle_gps_position_pub{nullptr};
    orb_advert_t _vehicle_local_position_pub{nullptr};
    orb_advert_t _vehicle_attitude_pub{nullptr};
    orb_advert_t _vehicle_status_pub{nullptr};
    orb_advert_t _actuator_outputs_pub{nullptr};
    orb_advert_t _battery_status_pub{nullptr};
    orb_advert_t _vehicle_air_data_pub{nullptr};
    orb_advert_t _estimator_status_pub{nullptr};
    orb_advert_t _vehicle_odometry_pub{nullptr};
    orb_advert_t _vehicle_command_pub{nullptr};
    orb_advert_t _vehicle_control_mode_pub{nullptr};
    orb_advert_t _manual_control_setpoint_pub{nullptr};
    orb_advert_t _rc_channels_pub{nullptr};
    orb_advert_t _safety_pub{nullptr};
    orb_advert_t _vehicle_land_detected_pub{nullptr};
    orb_advert_t _vehicle_imu_pub{nullptr};
    orb_advert_t _vehicle_magnetometer_pub{nullptr};
    orb_advert_t _vehicle_barometer_pub{nullptr};
    orb_advert_t _distance_sensor_pub{nullptr};
    orb_advert_t _optical_flow_pub{nullptr};
    orb_advert_t _collision_report_pub{nullptr};
    orb_advert_t _obstacle_distance_pub{nullptr};
    orb_advert_t _telemetry_status_pub{nullptr};
    orb_advert_t _log_message_pub{nullptr};
    
    // Swarm-specific subscriptions
    orb_advert_t _swarm_formation_state_pub{nullptr};
    orb_advert_t _swarm_task_allocation_pub{nullptr};
    orb_advert_t _swarm_ai_inference_pub{nullptr};
    orb_advert_t _swarm_comm_status_pub{nullptr};
    orb_advert_t _swarm_telemetry_sync_pub{nullptr};
    
    // uORB subscriptions
    orb_subscription_t _sensor_combined_sub{-1};
    orb_subscription_t _vehicle_gps_position_sub{-1};
    orb_subscription_t _vehicle_local_position_sub{-1};
    orb_subscription_t _vehicle_attitude_sub{-1};
    orb_subscription_t _vehicle_status_sub{-1};
    orb_subscription_t _actuator_outputs_sub{-1};
    orb_subscription_t _battery_status_sub{-1};
    orb_subscription_t _vehicle_air_data_sub{-1};
    orb_subscription_t _estimator_status_sub{-1};
    orb_subscription_t _vehicle_odometry_sub{-1};
    orb_subscription_t _vehicle_command_sub{-1};
    orb_subscription_t _vehicle_control_mode_sub{-1};
    orb_subscription_t _manual_control_setpoint_sub{-1};
    orb_subscription_t _rc_channels_sub{-1};
    orb_subscription_t _safety_sub{-1};
    orb_subscription_t _vehicle_land_detected_sub{-1};
    orb_subscription_t _vehicle_imu_sub{-1};
    orb_subscription_t _vehicle_magnetometer_sub{-1};
    orb_subscription_t _vehicle_barometer_sub{-1};
    orb_subscription_t _distance_sensor_sub{-1};
    orb_subscription_t _optical_flow_sub{-1};
    orb_subscription_t _collision_report_sub{-1};
    orb_subscription_t _obstacle_distance_sub{-1};
    orb_subscription_t _telemetry_status_sub{-1};
    
    // Swarm subscriptions
    orb_subscription_t _swarm_formation_state_sub{-1};
    orb_subscription_t _swarm_task_allocation_sub{-1};
    orb_subscription_t _swarm_ai_inference_sub{-1};
    orb_subscription_t _swarm_comm_status_sub{-1};
    orb_subscription_t _swarm_telemetry_sync_sub{-1};
    
    // Logging state
    int _log_file{-1};
    bool _logging_active{false};
    char _current_log_path[256];
    uint64_t _log_start_time_us{0};
    uint64_t _last_log_time_us{0};
    uint64_t _logged_bytes{0};
    uint64_t _log_duration_us{0};
    int _current_log_size_mb{0};
    
    // Logging intervals (us)
    uint64_t _sensor_interval_us{0};
    uint64_t _gps_interval_us{0};
    uint64_t _attitude_interval_us{0};
    uint64_t _position_interval_us{0};
    uint64_t _actuator_interval_us{0};
    uint64_t _battery_interval_us{0};
    uint64_t _estimator_interval_us{0};
    uint64_t _status_interval_us{0};
    uint64_t _safety_interval_us{0};
    uint64_t _formation_interval_us{0};
    uint64_t _task_alloc_interval_us{0};
    uint64_t _ai_inference_interval_us{0};
    uint64_t _comm_status_interval_us{0};
    uint64_t _telemetry_sync_interval_us{0};
    
    // Last log times
    uint64_t _last_sensor_log_us{0};
    uint64_t _last_gps_log_us{0};
    uint64_t _last_attitude_log_us{0};
    uint64_t _last_position_log_us{0};
    uint64_t _last_actuator_log_us{0};
    uint64_t _last_battery_log_us{0};
    uint64_t _last_estimator_log_us{0};
    uint64_t _last_status_log_us{0};
    uint64_t _last_safety_log_us{0};
    uint64_t _last_formation_log_us{0};
    uint64_t _last_task_alloc_log_us{0};
    uint64_t _last_ai_inference_log_us{0};
    uint64_t _last_comm_status_log_us{0};
    uint64_t _last_telemetry_sync_log_us{0};
    
    // Callbacks
    std::function<void(const char*, const char*)> _log_rotate_cb;
    std::function<void(uint64_t, uint64_t, uint64_t, double, double)> _sync_cb;
    
    // Data buffers
    sensor_combined_s _sensor_combined{};
    vehicle_gps_position_s _gps_position{};
    vehicle_local_position_s _local_position{};
    vehicle_attitude_s _attitude{};
    vehicle_status_s _vehicle_status{};
    actuator_outputs_s _actuator_outputs{};
    battery_status_s _battery_status{};
    vehicle_air_data_s _air_data{};
    estimator_status_s _estimator_status{};
    vehicle_odometry_s _odometry{};
    vehicle_command_s _vehicle_command{};
    vehicle_control_mode_s _control_mode{};
    manual_control_setpoint_s _manual_control{};
    rc_channels_s _rc_channels{};
    safety_s _safety{};
    vehicle_land_detected_s _land_detected{};
    
    // Swarm data
    swarm_formation_state_s _formation_state{};
    swarm_task_allocation_s _task_allocation{};
    swarm_ai_inference_s _ai_inference{};
    swarm_comm_status_s _comm_status{};
    swarm_telemetry_sync_s _telemetry_sync{};
    
    // Time sync
    int64_t _time_offset_us{0};
    double _time_drift_ppm{0.0};
    uint64_t _last_sync_time_us{0};
    uint64_t _last_time_sync_request_us{0};
    
    // Performance counters
    perf_counter_t _log_write_perf{};
    perf_counter_t _log_drop_perf{};
    perf_counter_t _sync_request_perf{};
    
    // Helper methods
    bool shouldLogSensorData() const;
    bool shouldLogGpsData() const;
    bool shouldLogAttitudeData() const;
    bool shouldLogPositionData() const;
    bool shouldLogActuatorData() const;
    bool shouldLogBatteryData() const;
    bool shouldLogEstimatorData() const;
    bool shouldLogStatusData() const;
    bool shouldLogSafetyData() const;
    bool shouldLogFormationData() const;
    bool shouldLogTaskAllocData() const;
    bool shouldLogAiInferenceData() const;
    bool shouldLogCommStatusData() const;
    bool shouldLogTelemetrySyncData() const;
    
    void writeUlogMessage(uint16_t msg_id, const void *data, size_t len);
    void writeUlogString(const char *key, const char *value);
    void writeUlogFormat(const char *format, const char *fields, const char *types);
    void writeUlogData(uint16_t msg_id, const void *data, size_t len);
    void writeUlogDropout(uint16_t count);
    
    // Time sync
    void handleTelemetrySync(const swarm_telemetry_sync_s &sync);
    void sendTimeSyncRequest();
    
    // Disk management
    bool hasEnoughDiskSpace() const;
    void cleanupOldLogs();
    
    // Error handling
    void reportError(const char *fmt, ...);
};

} // namespace swarm::firmware

#endif // FLIGHT_LOGGER_HPP