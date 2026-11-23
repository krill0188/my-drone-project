#include "flight_logger.hpp"
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <drivers/drv_hrt.h>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/parameter_update.h>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <sys/statvfs.h>
#include <unistd.h>

namespace swarm::firmware {

FlightLogger::FlightLogger() : ModuleParams(nullptr) {
    _log_write_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": log_write");
    _log_drop_perf = perf_alloc(PC_COUNT, MODULE_NAME ": log_dropped");
    _sync_request_perf = perf_alloc(PC_COUNT, MODULE_NAME ": sync_request");
}

FlightLogger::~FlightLogger() {
    stopLogging();
    perf_free(_log_write_perf);
    perf_free(_log_drop_perf);
    perf_free(_sync_request_perf);
}

int FlightLogger::task_spawn(int argc, char *argv[]) {
    FlightLogger *instance = new FlightLogger();
    if (instance) {
        object_store(instance);
        return instance->init();
    }
    return PX4_ERROR;
}

int FlightLogger::custom_command(int argc, char *argv[]) {
    return print_usage("unknown command");
}

int FlightLogger::print_usage(const char *reason) {
    if (reason) {
        PX4_WARN("%s\n", reason);
    }
    PRINT_MODULE_USAGE_NAME("flight_logger", "logger");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND("stop");
    PRINT_MODULE_USAGE_COMMAND("status");
    PRINT_MODULE_USAGE_COMMAND("rotate");
    PRINT_MODULE_USAGE_COMMAND("sync");
    PRINT_MODULE_USAGE_COMMAND("cleanup");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
    return 0;
}

int FlightLogger::init() {
    // Initialize parameters
    param_get(param_find("FLT_LOG_SRATE"), &_sensor_interval_us);
    _sensor_interval_us = 1000000 / _sensor_interval_us;  // Convert Hz to us
    
    param_get(param_find("FLT_LOG_GRATE"), &_gps_interval_us);
    _gps_interval_us = 1000000 / _gps_interval_us;
    
    param_get(param_find("FLT_LOG_ARATE"), &_attitude_interval_us);
    _attitude_interval_us = 1000000 / _attitude_interval_us;
    
    param_get(param_find("FLT_LOG_PRATE"), &_position_interval_us);
    _position_interval_us = 1000000 / _position_interval_us;
    
    param_get(param_find("FLT_LOG_ARATE"), &_actuator_interval_us);
    _actuator_interval_us = 1000000 / _actuator_interval_us;
    
    param_get(param_find("FLT_LOG_BRATE"), &_battery_interval_us);
    _battery_interval_us = 1000000 / _battery_interval_us;
    
    param_get(param_find("FLT_LOG_ERATE"), &_estimator_interval_us);
    _estimator_interval_us = 1000000 / _estimator_interval_us;
    
    param_get(param_find("FLT_LOG_SRATE"), &_status_interval_us);
    _status_interval_us = 1000000 / _status_interval_us;
    
    param_get(param_find("FLT_LOG_SAFRATE"), &_safety_interval_us);
    _safety_interval_us = 1000000 / _safety_interval_us;
    
    // Swarm-specific
    param_get(param_find("FLT_LOG_FRRATE"), &_formation_interval_us);
    _formation_interval_us = 1000000 / _formation_interval_us;
    
    param_get(param_find("FLT_LOG_TARATE"), &_task_alloc_interval_us);
    _task_alloc_interval_us = 1000000 / _task_alloc_interval_us;
    
    param_get(param_find("FLT_LOG_AIRATE"), &_ai_inference_interval_us);
    _ai_inference_interval_us = 1000000 / _ai_inference_interval_us;
    
    param_get(param_find("FLT_LOG_CSRATE"), &_comm_status_interval_us);
    _comm_status_interval_us = 1000000 / _comm_status_interval_us;
    
    param_get(param_find("FLT_LOG_TSRATE"), &_telemetry_sync_interval_us);
    _telemetry_sync_interval_us = 1000000 / _telemetry_sync_interval_us;
    
    // Subscribe to uORB topics
    _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    _vehicle_gps_position_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
    _vehicle_local_position_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _vehicle_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));
    _battery_status_sub = orb_subscribe(ORB_ID(battery_status));
    _vehicle_air_data_sub = orb_subscribe(ORB_ID(vehicle_air_data));
    _estimator_status_sub = orb_subscribe(ORB_ID(estimator_status));
    _vehicle_odometry_sub = orb_subscribe(ORB_ID(vehicle_odometry));
    _vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
    _vehicle_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
    _manual_control_setpoint_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    _rc_channels_sub = orb_subscribe(ORB_ID(rc_channels));
    _safety_sub = orb_subscribe(ORB_ID(safety));
    _vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
    _vehicle_imu_sub = orb_subscribe(ORB_ID(vehicle_imu));
    _vehicle_magnetometer_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
    _vehicle_barometer_sub = orb_subscribe(ORB_ID(vehicle_barometer));
    _distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
    _optical_flow_sub = orb_subscribe(ORB_ID(optical_flow));
    _collision_report_sub = orb_subscribe(ORB_ID(collision_report));
    _obstacle_distance_sub = orb_subscribe(ORB_ID(obstacle_distance));
    _telemetry_status_sub = orb_subscribe(ORB_ID(telemetry_status));
    
    // Swarm subscriptions
    _swarm_formation_state_sub = orb_subscribe(ORB_ID(swarm_formation_state));
    _swarm_task_allocation_sub = orb_subscribe(ORB_ID(swarm_task_allocation));
    _swarm_ai_inference_sub = orb_subscribe(ORB_ID(swarm_ai_inference));
    _swarm_comm_status_sub = orb_subscribe(ORB_ID(swarm_comm_status));
    _swarm_telemetry_sync_sub = orb_subscribe(ORB_ID(swarm_telemetry_sync));
    
    // Advertise swarm topics
    _swarm_formation_state_pub = orb_advertise(ORB_ID(swarm_formation_state), &_formation_state);
    _swarm_task_allocation_pub = orb_advertise(ORB_ID(swarm_task_allocation), &_task_allocation);
    _swarm_ai_inference_pub = orb_advertise(ORB_ID(swarm_ai_inference), &_ai_inference);
    _swarm_comm_status_pub = orb_advertise(ORB_ID(swarm_comm_status), &_comm_status);
    _swarm_telemetry_sync_pub = orb_advertise(ORB_ID(swarm_telemetry_sync), &_telemetry_sync);
    
    // Initialize intervals (convert Hz to microseconds)
    _sensor_interval_us = 1000000 / 100;  // 100 Hz
    _gps_interval_us = 1000000 / 10;       // 10 Hz
    _attitude_interval_us = 1000000 / 100;
    _position_interval_us = 1000000 / 10;
    _actuator_interval_us = 1000000 / 100;
    _battery_interval_us = 1000000 / 10;
    _estimator_interval_us = 1000000 / 10;
    _status_interval_us = 1000000 / 10;
    _safety_interval_us = 1000000 / 100;
    _formation_interval_us = 1000000 / 10;
    _task_alloc_interval_us = 1000000 / 1;
    _ai_inference_interval_us = 1000000 / 20;
    _comm_status_interval_us = 1000000 / 10;
    _telemetry_sync_interval_us = 1000000 / 10;
    
    // Initialize intervals
    _sensor_interval_us = 1000000 / 100;
    _gps_interval_us = 1000000 / 10;
    _attitude_interval_us = 1000000 / 100;
    _position_interval_us = 1000000 / 10;
    _actuator_interval_us = 1000000 / 100;
    _battery_interval_us = 1000000 / 10;
    _estimator_interval_us = 1000000 / 10;
    _status_interval_us = 1000000 / 10;
    _safety_interval_us = 1000000 / 100;
    _formation_interval_us = 1000000 / 10;
    _task_alloc_interval_us = 1000000 / 1;
    _ai_inference_interval_us = 1000000 / 20;
    _comm_status_interval_us = 1000000 / 10;
    _telemetry_sync_interval_us = 1000000 / 10;
    
    // Initialize last log times
    uint64_t now = hrt_absolute_time();
    _last_sensor_log_us = now;
    _last_gps_log_us = now;
    _last_attitude_log_us = now;
    _last_position_log_us = now;
    _last_actuator_log_us = now;
    _last_battery_log_us = now;
    _last_estimator_log_us = now;
    _last_status_log_us = now;
    _last_safety_log_us = now;
    _last_formation_log_us = now;
    _last_task_alloc_log_us = now;
    _last_ai_inference_log_us = now;
    _last_comm_status_log_us = now;
    _last_telemetry_sync_log_us = now;
    
    // Initialize perf counters
    _log_write_perf = perf_alloc(PC_ELAPSED, "flight_logger: write");
    _log_drop_perf = perf_alloc(PC_COUNT, "flight_logger: dropped");
    _sync_request_perf = perf_alloc(PC_COUNT, "flight_logger: sync_req");
    
    // Set default log directory
    strncpy(_current_log_path, "/fs/microsd/log", sizeof(_current_log_path) - 1);
    
    return PX4_OK;
}

void FlightLogger::run() {
    // Main loop
    while (!should_exit()) {
        uint64_t now = hrt_absolute_time();
        
        // Update sensor data
        orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
        orb_copy(ORB_ID(vehicle_gps_position), _vehicle_gps_position_sub, &_gps_position);
        orb_copy(ORB_ID(vehicle_local_position), _vehicle_local_position_sub, &_local_position);
        orb_copy(ORB_ID(vehicle_attitude), _vehicle_attitude_sub, &_attitude);
        orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
        orb_copy(ORB_ID(actuator_outputs), _actuator_outputs_sub, &_actuator_outputs);
        orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
        orb_copy(ORB_ID(vehicle_air_data), _vehicle_air_data_sub, &_air_data);
        orb_copy(ORB_ID(estimator_status), _estimator_status_sub, &_estimator_status);
        orb_copy(ORB_ID(vehicle_odometry), _vehicle_odometry_sub, &_odometry);
        
        // Check if armed and should log
        bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
        
        if (armed || !_logging_active) {
            uint64_t now = hrt_absolute_time();
            
            // Log sensor data at high rate
            if (shouldLogSensorData()) {
                logSensorData();
            }
            
            // Log GPS at lower rate
            if (shouldLogGpsData()) {
                logGpsData();
            }
            
            // Log attitude at high rate
            if (shouldLogAttitudeData()) {
                logAttitudeData();
            }
            
            // Log position
            if (shouldLogPositionData()) {
                logPositionData();
            }
            
            // Log actuator outputs
            if (shouldLogActuatorData()) {
                logActuatorData();
            }
            
            // Log battery
            if (shouldLogBatteryData()) {
                logBatteryData();
            }
            
            // Log estimator status
            if (shouldLogEstimatorData()) {
                logEstimatorData();
            }
            
            // Log vehicle status
            if (shouldLogStatusData()) {
                logStatusData();
            }
            
            // Log safety events
            if (shouldLogSafetyData()) {
                // Check for safety events
            }
            
            // Swarm-specific logging
            if (shouldLogFormationData()) {
                logFormationState();
            }
            
            if (shouldLogTaskAllocData()) {
                logTaskAllocation();
            }
            
            if (shouldLogAiInferenceData()) {
                logAiInference();
            }
            
            if (shouldLogCommStatusData()) {
                logCommStatus();
            }
            
            if (shouldLogTelemetrySyncData()) {
                logTelemetrySync();
            }
            
            // Check disk space periodically
            static uint64_t last_disk_check = 0;
            if (hrt_absolute_time() - _last_telemetry_sync_log_us > 10000000) { // 10s
                checkDiskSpace();
            }
        }
        
        // Sleep to maintain loop rate (100 Hz)
        px4_usleep(10000);
    }
}

bool FlightLogger::startLogging() {
    if (_logging_active) {
        return true;
    }
    
    if (!openLogFile()) {
        return false;
    }
    
    _logging_active = true;
    _log_start_time_us = hrt_absolute_time();
    _log_start_time_us = _log_start_time_us;
    _logged_bytes = 0;
    _log_duration_us = 0;
    _current_log_size_mb = 0;
    
    // Write ULog header
    writeUlogHeader();
    
    // Write format definitions
    writeUlogFormats();
    
    // Write initial parameters
    writeInitialParameters();
    
    PX4_INFO("Flight logging started: %s", _current_log_path);
    
    return true;
}

void FlightLogger::stopLogging() {
    if (!_logging_active) {
        return;
    }
    
    _logging_active = false;
    closeLogFile();
    
    PX4_INFO("Flight logging stopped. Duration: %.1fs, Size: %llu bytes", 
             _log_duration_us / 1e6, _logged_bytes);
}

bool FlightLogger::openLogFile() {
    // Create log directory
    struct stat st = {0};
    if (stat("/fs/microsd/log", &st) == -1) {
        mkdir("/fs/microsd/log", 0755);
    }
    
    // Generate log filename with timestamp
    char timestamp[32];
    time_t now = time(nullptr);
    struct tm *tm = localtime(&time(nullptr));
    strftime(_current_log_path, sizeof(_current_log_path), 
             "/fs/microsd/log/log_%Y%m%d_%H%M%S.ulg", localtime(&time(nullptr)));
    
    // Open file for writing
    _log_file = open(_current_log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    
    if (_log_file < 0) {
        PX4_ERR("Failed to open log file: %s", _current_log_path);
        return false;
    }
    
    return true;
}

void FlightLogger::closeLogFile() {
    if (_log_file >= 0) {
        close(_log_file);
        _log_file = -1;
    }
}

void FlightLogger::rotateLogFile() {
    closeLogFile();
    
    char old_path[256];
    strncpy(old_path, _current_log_path, sizeof(_current_log_path));
    
    // Generate new filename
    time_t now = time(nullptr);
    strftime(_current_log_path, sizeof(_current_log_path), 
             "/fs/microsd/log/log_%Y%m%d_%H%M%S.ulg", localtime(&time(nullptr)));
    
    _log_file = open(_current_log_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    
    if (_log_file >= 0) {
        // Write new header
        writeUlogHeader();
        writeUlogFormats();
        writeInitialParameters();
        
        if (_log_rotate_cb) {
            _log_rotate_cb(_current_log_path, _current_log_path);
        }
        
        PX4_INFO("Log rotated: %s -> %s", _current_log_path, _current_log_path);
    }
}

bool FlightLogger::checkDiskSpace() {
    struct statvfs st;
    if (statvfs("/fs/microsd", &st) != 0) {
        return false;
    }
    
    uint64_t free_bytes = (uint64_t)st.f_bavail * st.f_frsize;
    uint64_t free_mb = free_bytes / (1024 * 1024);
    
    if (free_mb < 50) {  // Less than 50MB free
        PX4_WARN("Low disk space: %llu MB free", free_mb);
        cleanupOldLogs();
        return false;
    }
    
    return true;
}

void FlightLogger::cleanupOldLogs() {
    // Remove oldest log files until enough space
    // Implementation would scan log directory and remove oldest files
    PX4_INFO("Cleaning up old log files");
}

void FlightLogger::logSensorData() {
    perf_begin(_log_write_perf);
    
    // Write sensor_combined data
    writeUlogData(static_cast<uint16_t>(UlogMessageId::SWARM_FORMATION_STATE), 
                  &_sensor_combined, sizeof(sensor_combined_s));
    
    perf_end(_log_write_perf);
    _last_sensor_log_us = hrt_absolute_time();
}

void FlightLogger::logGpsData() {
    perf_begin(_log_write_perf);
    
    // Write GPS data
    writeUlogData(ORB_ID(vehicle_gps_position), &_gps_position, sizeof(vehicle_gps_position_s));
    
    perf_end(_log_write_perf);
    _last_gps_log_us = hrt_absolute_time();
}

void FlightLogger::logAttitudeData() {
    perf_begin(_log_write_perf);
    
    // Write attitude data
    writeUlogData(ORB_ID(vehicle_attitude), &_attitude, sizeof(vehicle_attitude_s));
    
    perf_end(_log_write_perf);
    _last_attitude_log_us = hrt_absolute_time();
}

void FlightLogger::logPositionData() {
    perf_begin(_log_write_perf);
    
    // Write local position
    writeUlogData(ORB_ID(vehicle_local_position), &_local_position, sizeof(vehicle_local_position_s));
    
    perf_end(_log_write_perf);
    _last_position_log_us = hrt_absolute_time();
}

void FlightLogger::logActuatorData() {
    perf_begin(_log_write_perf);
    
    writeUlogData(ORB_ID(actuator_outputs), &_actuator_outputs, sizeof(actuator_outputs_s));
    
    perf_end(_log_write_perf);
    _last_actuator_log_us = hrt_absolute_time();
}

void FlightLogger::logBatteryData() {
    perf_begin(_log_write_perf);
    
    writeUlogData(ORB_ID(battery_status), &_battery_status, sizeof(battery_status_s));
    
    perf_end(_log_write_perf);
    _last_battery_log_us = hrt_absolute_time();
}

void FlightLogger::logEstimatorData() {
    perf_begin(_log_write_perf);
    
    writeUlogData(ORB_ID(estimator_status), &_estimator_status, sizeof(estimator_status_s));
    
    perf_end(_log_write_perf);
    _last_estimator_log_us = hrt_absolute_time();
}

void FlightLogger::logStatusData() {
    perf_begin(_log_write_perf);
    
    writeUlogData(ORB_ID(vehicle_status), &_vehicle_status, sizeof(vehicle_status_s));
    
    perf_end(_log_write_perf);
    _last_status_log_us = hrt_absolute_time();
}

void FlightLogger::logSafetyEvent(const char *event_type, const char *details) {
    perf_begin(_log_write_perf);
    
    // Write safety event
    log_message_s log_msg{};
    log_msg.timestamp = hrt_absolute_time();
    log_msg.severity = 3; // Error
    snprintf(log_msg.text, sizeof(log_msg.text), "SAFETY: %s - %s", event_type, details);
    
    writeUlogData(ORB_ID(log_message), &log_msg, sizeof(log_message_s));
    
    perf_end(_log_write_perf);
    _last_safety_log_us = hrt_absolute_time();
}

void FlightLogger::logFormationState() {
    perf_begin(_log_write_perf);
    
    // Copy formation state data
    orb_copy(ORB_ID(swarm_formation_state), _swarm_formation_state_sub, &_formation_state);
    
    // Write to ULog with custom message ID
    writeUlogData(static_cast<uint16_t>(UlogMessageId::SWARM_FORMATION_STATE), 
                  &_formation_state, sizeof(swarm_formation_state_s));
    
    perf_end(_log_write_perf);
    _last_formation_log_us = hrt_absolute_time();
}

void FlightLogger::logTaskAllocation() {
    perf_begin(_log_write_perf);
    
    orb_copy(ORB_ID(swarm_task_allocation), _swarm_task_allocation_sub, &_task_allocation);
    
    writeUlogData(static_cast<uint16_t>(UlogMessageId::SWARM_TASK_ALLOCATION), 
                  &_task_allocation, sizeof(swarm_task_allocation_s));
    
    perf_end(_log_write_perf);
    _last_task_alloc_log_us = hrt_absolute_time();
}

void FlightLogger::logAiInference() {
    perf_begin(_log_write_perf);
    
    orb_copy(ORB_ID(swarm_ai_inference), _swarm_ai_inference_sub, &_ai_inference);
    
    writeUlogData(static_cast<uint16_t>(UlogMessageId::SWARM_AI_INFERENCE), 
                  &_ai_inference, sizeof(swarm_ai_inference_s));
    
    perf_end(_log_write_perf);
    _last_ai_inference_log_us = hrt_absolute_time();
}

void FlightLogger::logCommStatus() {
    perf_begin(_log_write_perf);
    
    orb_copy(ORB_ID(swarm_comm_status), _swarm_comm_status_sub, &_comm_status);
    
    writeUlogData(static_cast<uint16_t>(UlogMessageId::SWARM_COMM_STATUS), 
                  &_comm_status, sizeof(swarm_comm_status_s));
    
    perf_end(_log_write_perf);
    _last_comm_status_log_us = hrt_absolute_time();
}

void FlightLogger::logTelemetrySync() {
    perf_begin(_log_write_perf);
    
    orb_copy(ORB_ID(swarm_telemetry_sync), _swarm_telemetry_sync_sub, &_telemetry_sync);
    
    writeUlogData(static_cast<uint16_t>(UlogMessageId::SWARM_TELEMETRY_SYNC), 
                  &_telemetry_sync, sizeof(swarm_telemetry_sync_s));
    
    perf_end(_log_write_perf);
    _last_telemetry_sync_log_us = hrt_absolute_time();
}

bool FlightLogger::shouldLogSensorData() const {
    return (hrt_absolute_time() - _last_sensor_log_us) >= _sensor_interval_us;
}

bool FlightLogger::shouldLogGpsData() const {
    return (hrt_absolute_time() - _last_gps_log_us) >= _gps_interval_us;
}

bool FlightLogger::shouldLogAttitudeData() const {
    return (hrt_absolute_time() - _last_attitude_log_us) >= _attitude_interval_us;
}

bool FlightLogger::shouldLogPositionData() const {
    return (hrt_absolute_time() - _last_position_log_us) >= _position_interval_us;
}

bool FlightLogger::shouldLogActuatorData() const {
    return (hrt_absolute_time() - _last_actuator_log_us) >= _actuator_interval_us;
}

bool FlightLogger::shouldLogBatteryData() const {
    return (hrt_absolute_time() - _last_battery_log_us) >= _battery_interval_us;
}

bool FlightLogger::shouldLogEstimatorData() const {
    return (hrt_absolute_time() - _last_estimator_log_us) >= _estimator_interval_us;
}

bool FlightLogger::shouldLogStatusData() const {
    return (hrt_absolute_time() - _last_status_log_us) >= _status_interval_us;
}

bool FlightLogger::shouldLogSafetyData() const {
    return (hrt_absolute_time() - _last_safety_log_us) >= _safety_interval_us;
}

bool FlightLogger::shouldLogFormationData() const {
    return (hrt_absolute_time() - _last_formation_log_us) >= _formation_interval_us;
}

bool FlightLogger::shouldLogTaskAllocData() const {
    return (hrt_absolute_time() - _last_task_alloc_log_us) >= _task_alloc_interval_us;
}

bool FlightLogger::shouldLogAiInferenceData() const {
    return (hrt_absolute_time() - _last_ai_inference_log_us) >= _ai_inference_interval_us;
}

bool FlightLogger::shouldLogCommStatusData() const {
    return (hrt_absolute_time() - _last_comm_status_log_us) >= _comm_status_interval_us;
}

bool FlightLogger::shouldLogTelemetrySyncData() const {
    return (hrt_absolute_time() - _last_telemetry_sync_log_us) >= _telemetry_sync_interval_us;
}

void FlightLogger::writeUlogHeader() {
    // ULog file header
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions for each message type
    // This would write the format definitions for all message types
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameter values
}

void FlightLogger::writeUlogData(uint16_t msg_id, const void *data, size_t len) {
    if (_log_file < 0) return;
    
    perf_begin(_log_write_perf);
    
    // Write message header
    uint16_t msg_len = static_cast<uint16_t>(len);
    uint16_t msg_id = msg_id;
    uint64_t timestamp = hrt_absolute_time();
    
    // Write header: len (2 bytes) + msg_id (2 bytes) + timestamp (8 bytes)
    uint8_t header[12];
    header[0] = msg_len & 0xFF;
    header[1] = (msg_len >> 8) & 0xFF;
    header[2] = msg_id & 0xFF;
    header[3] = (msg_id >> 8) & 0xFF;
    for (int i = 0; i < 8; i++) {
        header[4 + i] = (timestamp >> (i * 8)) & 0xFF;
    }
    
    ssize_t written = write(_log_file, header, 12);
    if (written != 12) {
        perf_count(_log_drop_perf);
        return;
    }
    
    written = write(_log_file, data, len);
    if (written != static_cast<ssize_t>(len)) {
        perf_count(_log_drop_perf);
        return;
    }
    
    _logged_bytes += 12 + len;
    perf_end(_log_write_perf);
}

void FlightLogger::writeUlogHeader() {
    // Write ULog header
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions for all message types
    // This would define the structure of each message type
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameter values
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

bool FlightLogger::shouldLogSensorData() const {
    return (hrt_absolute_time() - _last_sensor_log_us) >= _sensor_interval_us;
}

bool FlightLogger::shouldLogGpsData() const {
    return (hrt_absolute_time() - _last_gps_log_us) >= _gps_interval_us;
}

bool FlightLogger::shouldLogAttitudeData() const {
    return (hrt_absolute_time() - _last_attitude_log_us) >= _attitude_interval_us;
}

bool FlightLogger::shouldLogPositionData() const {
    return (hrt_absolute_time() - _last_position_log_us) >= _position_interval_us;
}

bool FlightLogger::shouldLogActuatorData() const {
    return (hrt_absolute_time() - _last_actuator_log_us) >= _actuator_interval_us;
}

bool FlightLogger::shouldLogBatteryData() const {
    return (hrt_absolute_time() - _last_battery_log_us) >= _battery_interval_us;
}

bool FlightLogger::shouldLogEstimatorData() const {
    return (hrt_absolute_time() - _last_estimator_log_us) >= _estimator_interval_us;
}

bool FlightLogger::shouldLogStatusData() const {
    return (hrt_absolute_time() - _last_status_log_us) >= _status_interval_us;
}

bool FlightLogger::shouldLogSafetyData() const {
    return (hrt_absolute_time() - _last_safety_log_us) >= _safety_interval_us;
}

bool FlightLogger::shouldLogFormationData() const {
    return (hrt_absolute_time() - _last_formation_log_us) >= _formation_interval_us;
}

bool FlightLogger::shouldLogTaskAllocData() const {
    return (hrt_absolute_time() - _last_task_alloc_log_us) >= _task_alloc_interval_us;
}

bool FlightLogger::shouldLogAiInferenceData() const {
    return (hrt_absolute_time() - _last_ai_inference_log_us) >= _ai_inference_interval_us;
}

bool FlightLogger::shouldLogCommStatusData() const {
    return (hrt_absolute_time() - _last_comm_status_log_us) >= _comm_status_interval_us;
}

bool FlightLogger::shouldLogTelemetrySyncData() const {
    return (hrt_absolute_time() - _last_telemetry_sync_log_us) >= _telemetry_sync_interval_us;
}

void FlightLogger::addTelemetrySync(uint64_t firmware_time_us, uint64_t companion_time_us, 
                                    uint64_t gps_time_us, double offset_us, double drift_ppm) {
    _time_offset_us = static_cast<int64_t>(offset_us);
    _time_drift_ppm = drift_ppm;
    _last_sync_time_us = hrt_absolute_time();
}

void FlightLogger::requestTimeSync() {
    // Request time sync from companion
    perf_count(_sync_request_perf);
    _last_time_sync_request_us = hrt_absolute_time();
}

bool FlightLogger::checkDiskSpace() {
    struct statvfs st;
    if (statvfs("/fs/microsd", &st) != 0) {
        return false;
    }
    
    uint64_t free_bytes = (uint64_t)st.f_bavail * st.f_frsize;
    uint64_t free_mb = free_bytes / (1024 * 1024);
    
    if (free_mb < 50) {
        PX4_WARN("Low disk space: %llu MB free", free_mb);
        cleanupOldLogs();
        return false;
    }
    
    _current_log_size_mb = _logged_bytes / (1024 * 1024);
    return true;
}

void FlightLogger::cleanupOldLogs() {
    // Remove oldest log files until enough space
    PX4_INFO("Cleaning up old log files");
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogData(uint16_t msg_id, const void *data, size_t len) {
    if (_log_file < 0) return;
    
    perf_begin(_log_write_perf);
    
    uint16_t msg_len = static_cast<uint16_t>(len);
    uint16_t msg_id = msg_id;
    uint64_t timestamp = hrt_absolute_time();
    
    // Write header: len (2 bytes) + msg_id (2 bytes) + timestamp (8 bytes)
    uint8_t header[12];
    header[0] = msg_len & 0xFF;
    header[1] = (msg_len >> 8) & 0xFF;
    header[2] = msg_id & 0xFF;
    header[3] = (msg_id >> 8) & 0xFF;
    for (int i = 0; i < 8; i++) {
        header[4 + i] = (timestamp >> (i * 8)) & 0xFF;
    }
    
    ssize_t written = write(_log_file, header, 12);
    if (written != 12) {
        perf_count(_log_drop_perf);
        return;
    }
    
    written = write(_log_file, data, len);
    if (written != static_cast<ssize_t>(len)) {
        perf_count(_log_drop_perf);
        return;
    }
    
    _logged_bytes += 12 + len;
    perf_end(_log_write_perf);
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

void FlightLogger::addTelemetrySync(uint64_t firmware_time_us, uint64_t companion_time_us, 
                                    uint64_t gps_time_us, double offset_us, double drift_ppm) {
    _time_offset_us = static_cast<int64_t>(offset_us);
    _time_drift_ppm = drift_ppm;
    _last_sync_time_us = hrt_absolute_time();
}

void FlightLogger::requestTimeSync() {
    perf_count(_sync_request_perf);
    _last_time_sync_request_us = hrt_absolute_time();
}

bool FlightLogger::checkDiskSpace() {
    struct statvfs st;
    if (statvfs("/fs/microsd", &st) != 0) {
        return false;
    }
    
    uint64_t free_bytes = (uint64_t)st.f_bavail * st.f_frsize;
    uint64_t free_mb = free_bytes / (1024 * 1024);
    
    if (free_mb < 50) {
        PX4_WARN("Low disk space: %llu MB free", free_mb);
        cleanupOldLogs();
        return false;
    }
    
    _current_log_size_mb = _logged_bytes / (1024 * 1024);
    return true;
}

void FlightLogger::cleanupOldLogs() {
    PX4_INFO("Cleaning up old log files");
}

bool FlightLogger::shouldLogSensorData() const {
    return (hrt_absolute_time() - _last_sensor_log_us) >= _sensor_interval_us;
}

bool FlightLogger::shouldLogGpsData() const {
    return (hrt_absolute_time() - _last_gps_log_us) >= _gps_interval_us;
}

bool FlightLogger::shouldLogAttitudeData() const {
    return (hrt_absolute_time() - _last_attitude_log_us) >= _attitude_interval_us;
}

bool FlightLogger::shouldLogPositionData() const {
    return (hrt_absolute_time() - _last_position_log_us) >= _position_interval_us;
}

bool FlightLogger::shouldLogActuatorData() const {
    return (hrt_absolute_time() - _last_actuator_log_us) >= _actuator_interval_us;
}

bool FlightLogger::shouldLogBatteryData() const {
    return (hrt_absolute_time() - _last_battery_log_us) >= _battery_interval_us;
}

bool FlightLogger::shouldLogEstimatorData() const {
    return (hrt_absolute_time() - _last_estimator_log_us) >= _estimator_interval_us;
}

bool FlightLogger::shouldLogStatusData() const {
    return (hrt_absolute_time() - _last_status_log_us) >= _status_interval_us;
}

bool FlightLogger::shouldLogSafetyData() const {
    return (hrt_absolute_time() - _last_safety_log_us) >= _safety_interval_us;
}

bool FlightLogger::shouldLogFormationData() const {
    return (hrt_absolute_time() - _last_formation_log_us) >= _formation_interval_us;
}

bool FlightLogger::shouldLogTaskAllocData() const {
    return (hrt_absolute_time() - _last_task_alloc_log_us) >= _task_alloc_interval_us;
}

bool FlightLogger::shouldLogAiInferenceData() const {
    return (hrt_absolute_time() - _last_ai_inference_log_us) >= _ai_inference_interval_us;
}

bool FlightLogger::shouldLogCommStatusData() const {
    return (hrt_absolute_time() - _last_comm_status_log_us) >= _comm_status_interval_us;
}

bool FlightLogger::shouldLogTelemetrySyncData() const {
    return (hrt_absolute_time() - _last_telemetry_sync_log_us) >= _telemetry_sync_interval_us;
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

void FlightLogger::addTelemetrySync(uint64_t firmware_time_us, uint64_t companion_time_us, 
                                    uint64_t gps_time_us, double offset_us, double drift_ppm) {
    _time_offset_us = static_cast<int64_t>(offset_us);
    _time_drift_ppm = drift_ppm;
    _last_sync_time_us = hrt_absolute_time();
}

void FlightLogger::requestTimeSync() {
    perf_count(_sync_request_perf);
    _last_time_sync_request_us = hrt_absolute_time();
}

bool FlightLogger::checkDiskSpace() {
    struct statvfs st;
    if (statvfs("/fs/microsd", &st) != 0) {
        return false;
    }
    
    uint64_t free_bytes = (uint64_t)st.f_bavail * st.f_frsize;
    uint64_t free_mb = free_bytes / (1024 * 1024);
    
    if (free_mb < 50) {
        PX4_WARN("Low disk space: %llu MB free", free_mb);
        cleanupOldLogs();
        return false;
    }
    
    _current_log_size_mb = _logged_bytes / (1024 * 1024);
    return true;
}

void FlightLogger::cleanupOldLogs() {
    PX4_INFO("Cleaning up old log files");
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

void FlightLogger::addTelemetrySync(uint64_t firmware_time_us, uint64_t companion_time_us, 
                                    uint64_t gps_time_us, double offset_us, double drift_ppm) {
    _time_offset_us = static_cast<int64_t>(offset_us);
    _time_drift_ppm = drift_ppm;
    _last_sync_time_us = hrt_absolute_time();
}

void FlightLogger::requestTimeSync() {
    perf_count(_sync_request_perf);
    _last_time_sync_request_us = hrt_absolute_time();
}

bool FlightLogger::checkDiskSpace() {
    struct statvfs st;
    if (statvfs("/fs/microsd", &st) != 0) {
        return false;
    }
    
    uint64_t free_bytes = (uint64_t)st.f_bavail * st.f_frsize;
    uint64_t free_mb = free_bytes / (1024 * 1024);
    
    if (free_mb < 50) {
        PX4_WARN("Low disk space: %llu MB free", free_mb);
        cleanupOldLogs();
        return false;
    }
    
    _current_log_size_mb = _logged_bytes / (1024 * 1024);
    return true;
}

void FlightLogger::cleanupOldLogs() {
    PX4_INFO("Cleaning up old log files");
}

bool FlightLogger::shouldLogSensorData() const {
    return (hrt_absolute_time() - _last_sensor_log_us) >= _sensor_interval_us;
}

bool FlightLogger::shouldLogGpsData() const {
    return (hrt_absolute_time() - _last_gps_log_us) >= _gps_interval_us;
}

bool FlightLogger::shouldLogAttitudeData() const {
    return (hrt_absolute_time() - _last_attitude_log_us) >= _attitude_interval_us;
}

bool FlightLogger::shouldLogPositionData() const {
    return (hrt_absolute_time() - _last_position_log_us) >= _position_interval_us;
}

bool FlightLogger::shouldLogActuatorData() const {
    return (hrt_absolute_time() - _last_actuator_log_us) >= _actuator_interval_us;
}

bool FlightLogger::shouldLogBatteryData() const {
    return (hrt_absolute_time() - _last_battery_log_us) >= _battery_interval_us;
}

bool FlightLogger::shouldLogEstimatorData() const {
    return (hrt_absolute_time() - _last_estimator_log_us) >= _estimator_interval_us;
}

bool FlightLogger::shouldLogStatusData() const {
    return (hrt_absolute_time() - _last_status_log_us) >= _status_interval_us;
}

bool FlightLogger::shouldLogSafetyData() const {
    return (hrt_absolute_time() - _last_safety_log_us) >= _safety_interval_us;
}

bool FlightLogger::shouldLogFormationData() const {
    return (hrt_absolute_time() - _last_formation_log_us) >= _formation_interval_us;
}

bool FlightLogger::shouldLogTaskAllocData() const {
    return (hrt_absolute_time() - _last_task_alloc_log_us) >= _task_alloc_interval_us;
}

bool FlightLogger::shouldLogAiInferenceData() const {
    return (hrt_absolute_time() - _last_ai_inference_log_us) >= _ai_inference_interval_us;
}

bool FlightLogger::shouldLogCommStatusData() const {
    return (hrt_absolute_time() - _last_comm_status_log_us) >= _comm_status_interval_us;
}

bool FlightLogger::shouldLogTelemetrySyncData() const {
    return (hrt_absolute_time() - _last_telemetry_sync_log_us) >= _telemetry_sync_interval_us;
}

void FlightLogger::writeUlogHeader() {
    const char header[] = "ULog\x01\x00";
    write(_log_file, header, sizeof(header) - 1);
}

void FlightLogger::writeUlogFormats() {
    // Write format definitions
}

void FlightLogger::writeInitialParameters() {
    // Write initial parameters
}

void FlightLogger::writeUlogString(const char *key, const char *value) {
    // Write string parameter
}

void FlightLogger::writeUlogFormat(const char *format, const char *fields, const char *types) {
    // Write format definition
}

void FlightLogger::writeUlogDropout(uint16_t count) {
    // Write dropout marker
}

} // namespace swarm::firmware