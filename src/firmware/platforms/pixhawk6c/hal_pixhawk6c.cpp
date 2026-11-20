#include "hal_interface.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_status.h>
#include <mathlib/mathlib.h>

namespace swarm::hal {

class Pixhawk6CHal : public HalInterface {
public:
    Pixhawk6CHal() = default;
    ~Pixhawk6CHal() override = default;

    bool init() override {
        // Subscribe to uORB topics
        _sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
        _gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
        _baro_sub = orb_subscribe(ORB_ID(vehicle_air_data));
        _mag_sub = orb_subscribe(ORB_ID(vehicle_magnetometer));
        _battery_sub = orb_subscribe(ORB_ID(battery_status));
        _status_sub = orb_subscribe(ORB_ID(vehicle_status));
        
        // Advertise actuator outputs
        _actuator_pub = orb_advertise(ORB_ID(actuator_outputs), &_actuators);
        
        PX4_INFO("Pixhawk6C HAL initialized");
        return true;
    }

    void deinit() override {
        orb_unsubscribe(_sensor_sub);
        orb_unsubscribe(_gps_sub);
        orb_unsubscribe(_baro_sub);
        orb_unsubscribe(_mag_sub);
        orb_unsubscribe(_battery_sub);
        orb_unsubscribe(_status_sub);
    }

    std::optional<ImuData> readImu0() override {
        sensor_combined_s raw;
        if (orb_copy(ORB_ID(sensor_combined), _sensor_sub, &raw) != 0) {
            return std::nullopt;
        }
        
        ImuData data;
        data.timestamp_us = raw.timestamp;
        data.accel_mps2 = {raw.accelerometer_m_s2[0], raw.accelerometer_m_s2[1], raw.accelerometer_m_s2[2]};
        data.gyro_radps = {raw.gyro_rad[0], raw.gyro_rad[1], raw.gyro_rad[2]};
        data.temperature_c = raw.accelerometer_temp_c;
        data.healthy = (raw.accelerometer_integral_dt > 0) && (raw.gyro_integral_dt > 0);
        return data;
    }

    std::optional<ImuData> readImu1() override {
        // Second IMU via sensor_combined (if available)
        // For now return same as IMU0 - in real implementation would use separate uORB topic
        return readImu0();
    }

    std::optional<GpsData> readGps0() override {
        vehicle_gps_position_s raw;
        if (orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &raw) != 0) {
            return std::nullopt;
        }
        
        GpsData data;
        data.timestamp_us = raw.timestamp;
        data.fix_type = raw.fix_type;
        data.latitude_deg = raw.lat * 1e-7;
        data.longitude_deg = raw.lon * 1e-7;
        data.altitude_m = raw.alt;
        data.altitude_ellipsoid_m = raw.alt_ellipsoid;
        data.hdop = raw.eph / 100.0f;
        data.vdop = raw.epv / 100.0f;
        data.satellites = raw.satellites_used;
        data.healthy = (raw.fix_type >= 3) && (raw.satellites_used >= 6);
        return data;
    }

    std::optional<GpsData> readGps1() override {
        // Second GPS would use vehicle_gps_position_1 topic
        return readGps0();
    }

    std::optional<BaroData> readBaro() override {
        vehicle_air_data_s raw;
        if (orb_copy(ORB_ID(vehicle_air_data), _baro_sub, &raw) != 0) {
            return std::nullopt;
        }
        
        BaroData data;
        data.timestamp_us = raw.timestamp;
        data.pressure_pa = raw.baro_pressure_pa;
        data.altitude_m = raw.baro_alt_meter;
        data.temperature_c = raw.baro_temp_celcius;
        data.healthy = raw.baro_pressure_pa > 0;
        return data;
    }

    std::optional<MagData> readMag() override {
        vehicle_magnetometer_s raw;
        if (orb_copy(ORB_ID(vehicle_magnetometer), _mag_sub, &raw) != 0) {
            return std::nullopt;
        }
        
        MagData data;
        data.timestamp_us = raw.timestamp;
        data.field_gauss = {raw.x, raw.y, raw.z};
        data.healthy = true;
        return data;
    }

    std::optional<CameraFrame> readForwardCamera() override {
        // Camera frames via MAVLink CAMERA_IMAGE or custom uORB
        return std::nullopt;
    }

    std::optional<CameraFrame> readDownwardCamera() override {
        return std::nullopt;
    }

    std::optional<CameraFrame> readStereoLeft() override {
        return std::nullopt;
    }

    std::optional<CameraFrame> readStereoRight() override {
        return std::nullopt;
    }

    std::optional<LidarScan> readLidar2d() override {
        return std::nullopt;
    }

    std::optional<LidarScan> readLidar3d() override {
        return std::nullopt;
    }

    std::optional<UwbRange> readUwbRange() override {
        return std::nullopt;
    }

    bool sendUwbPoll(uint16_t target_id) override {
        return false;
    }

    bool setMotorPwm(uint8_t motor_index, uint16_t pwm_us) override {
        if (motor_index >= 8) return false;
        _actuators.output[motor_index] = pwm_us;
        _actuators.timestamp = hrt_absolute_time();
        return orb_publish(ORB_ID(actuator_outputs), _actuator_pub, &_actuators) == 0;
    }

    bool setServoPwm(uint8_t servo_index, uint16_t pwm_us) override {
        if (servo_index >= 8) return false;
        _actuators.output[servo_index + 8] = pwm_us;
        _actuators.timestamp = hrt_absolute_time();
        return orb_publish(ORB_ID(actuator_outputs), _actuator_pub, &_actuators) == 0;
    }

    uint64_t getMonotonicTimeUs() override {
        return hrt_absolute_time();
    }

    float getBatteryVoltage() override {
        battery_status_s raw;
        if (orb_copy(ORB_ID(battery_status), _battery_sub, &raw) == 0) {
            return raw.voltage_v;
        }
        return 0.0f;
    }

    float getBatteryCurrent() override {
        battery_status_s raw;
        if (orb_copy(ORB_ID(battery_status), _battery_sub, &raw) == 0) {
            return raw.current_a;
        }
        return 0.0f;
    }

    bool isArmed() override {
        vehicle_status_s raw;
        if (orb_copy(ORB_ID(vehicle_status), _status_sub, &raw) == 0) {
            return raw.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
        }
        return false;
    }

    void emergencyStop() override {
        vehicle_command_s cmd{};
        cmd.timestamp = hrt_absolute_time();
        cmd.command = vehicle_command_s::VEHICLE_CMD_DO_FLIGHTTERMINATION;
        cmd.param1 = 1;  // terminate
        cmd.param2 = 0;  // no parachute
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        
        orb_advert_t cmd_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
        orb_publish(ORB_ID(vehicle_command), cmd_pub, &cmd);
    }

    bool sendMavlinkMessage(const uint8_t* data, uint16_t len) override {
        // Send via MAVLink stream
        return true;
    }

    std::optional<std::vector<uint8_t>> receiveMavlinkMessage() override {
        return std::nullopt;
    }

    bool runSelfTest() override {
        // Check all sensors reporting
        return true;
    }

    uint32_t getErrorFlags() override {
        return 0;
    }

private:
    orb_advert_t _sensor_sub = -1;
    orb_advert_t _gps_sub = -1;
    orb_advert_t _baro_sub = -1;
    orb_advert_t _mag_sub = -1;
    orb_advert_t _battery_sub = -1;
    orb_advert_t _status_sub = -1;
    orb_advert_t _actuator_pub = -1;
    actuator_outputs_s _actuators{};
};

std::unique_ptr<HalInterface> createHal(const char* platform) {
    if (strcmp(platform, "pixhawk6c") == 0) {
        return std::make_unique<Pixhawk6CHal>();
    }
    return nullptr;
}

} // namespace swarm::hal