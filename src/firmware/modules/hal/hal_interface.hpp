#ifndef SWARM_HAL_HPP
#define SWARM_HAL_HPP

#include <cstdint>
#include <array>
#include <optional>

namespace swarm::hal {

// Sensor data structures
struct ImuData {
    uint64_t timestamp_us;
    std::array<float, 3> accel_mps2;      // Body frame
    std::array<float, 3> gyro_radps;      // Body frame
    float temperature_c;
    bool healthy;
};

struct GpsData {
    uint64_t timestamp_us;
    uint8_t fix_type;          // 0=none, 1=2D, 2=3D, 3=RTK float, 4=RTK fixed
    double latitude_deg;
    double longitude_deg;
    float altitude_m;          // AMSL
    float altitude_ellipsoid_m;
    float hdop;
    float vdop;
    uint8_t satellites;
    bool healthy;
};

struct BaroData {
    uint64_t timestamp_us;
    float pressure_pa;
    float altitude_m;
    float temperature_c;
    bool healthy;
};

struct MagData {
    uint64_t timestamp_us;
    std::array<float, 3> field_gauss;    // Body frame
    bool healthy;
};

struct CameraFrame {
    uint64_t timestamp_us;
    uint32_t width;
    uint32_t height;
    uint32_t stride;
    uint8_t format;  // 0=RGB8, 1=YUV422, 2=MONO8, 3=RAW12
    const uint8_t* data;
    bool new_frame;
};

struct LidarScan {
    uint64_t timestamp_us;
    uint32_t point_count;
    float angle_min_rad;
    float angle_max_rad;
    float angle_increment_rad;
    float range_min_m;
    float range_max_m;
    const float* ranges;
    const float* intensities;
    bool healthy;
};

struct UwbRange {
    uint64_t timestamp_us;
    uint16_t anchor_id;
    float range_m;
    float quality;  // 0.0-1.0
};

// HAL Interface (abstract base class)
class HalInterface {
public:
    virtual ~HalInterface() = default;
    
    // Initialization
    virtual bool init() = 0;
    virtual void deinit() = 0;
    
    // IMU (dual redundant)
    virtual std::optional<ImuData> readImu0() = 0;
    virtual std::optional<ImuData> readImu1() = 0;
    
    // GPS (dual redundant)
    virtual std::optional<GpsData> readGps0() = 0;
    virtual std::optional<GpsData> readGps1() = 0;
    
    // Barometer
    virtual std::optional<BaroData> readBaro() = 0;
    
    // Magnetometer
    virtual std::optional<MagData> readMag() = 0;
    
    // Cameras
    virtual std::optional<CameraFrame> readForwardCamera() = 0;
    virtual std::optional<CameraFrame> readDownwardCamera() = 0;
    virtual std::optional<CameraFrame> readStereoLeft() = 0;
    virtual std::optional<CameraFrame> readStereoRight() = 0;
    
    // LiDAR (optional)
    virtual std::optional<LidarScan> readLidar2d() = 0;
    virtual std::optional<LidarScan> readLidar3d() = 0;
    
    // UWB
    virtual std::optional<UwbRange> readUwbRange() = 0;
    virtual bool sendUwbPoll(uint16_t target_id) = 0;
    
    // Actuators
    virtual bool setMotorPwm(uint8_t motor_index, uint16_t pwm_us) = 0;
    virtual bool setServoPwm(uint8_t servo_index, uint16_t pwm_us) = 0;
    
    // System
    virtual uint64_t getMonotonicTimeUs() = 0;
    virtual float getBatteryVoltage() = 0;
    virtual float getBatteryCurrent() = 0;
    virtual bool isArmed() = 0;
    virtual void emergencyStop() = 0;
    
    // Communication
    virtual bool sendMavlinkMessage(const uint8_t* data, uint16_t len) = 0;
    virtual std::optional<std::vector<uint8_t>> receiveMavlinkMessage() = 0;
    
    // Health monitoring
    virtual bool runSelfTest() = 0;
    virtual uint32_t getErrorFlags() = 0;
};

// Factory function
std::unique_ptr<HalInterface> createHal(const char* platform);

} // namespace swarm::hal

#endif // SWARM_HAL_HPP