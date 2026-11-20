#ifndef SWARM_SAFETY_HPP
#define SWARM_SAFETY_HPP

#include <cstdint>
#include <vector>
#include <array>
#include <optional>
#include "hal_interface.hpp"
#include "swarm_comm.hpp"

namespace swarm::safety {

// Geofence definitions
struct GeofencePolygon {
    uint32_t id;
    std::vector<Vector3> vertices;  // ENU frame, closed polygon
    float min_altitude_m;
    float max_altitude_m;
    bool is_exclusion;  // true = no-fly zone, false = inclusion zone
};

struct GeofenceConfig {
    std::vector<GeofencePolygon> polygons;
    float buffer_distance_m = 10.0f;  // Safety buffer inside boundary
    bool enabled = true;
    bool trigger_rtl = true;
    float rtl_altitude_m = 50.0f;
};

enum class GeofenceAction : uint8_t {
    NONE = 0,
    WARNING = 1,
    RTL = 2,
    LAND = 3,
    TERMINATE = 4,
};

// Remote ID (ASTM F3411 / EU DRI)
struct RemoteIdConfig {
    std::string operator_id;      // 20 char max
    std::string uas_id;           // 20 char max (serial or session ID)
    std::string uas_type;         // 4 char max (e.g., "QUAD")
    float max_altitude_m;
    bool broadcast_enabled = true;
    uint32_t broadcast_interval_ms = 1000;
};

struct RemoteIdFrame {
    uint64_t timestamp_us;
    std::string operator_id;
    std::string uas_id;
    std::string uas_type;
    float latitude_deg;
    float longitude_deg;
    float altitude_m;       // AMSL
    float height_m;         // AGL
    float velocity_north_mps;
    float velocity_east_mps;
    float velocity_down_mps;
    uint8_t status;         // 0=normal, 1=emergency, 2=lost_link
    uint16_t sequence;
};

// Emergency stop
struct EmergencyStopConfig {
    bool deploy_parachute = false;
    float parachute_delay_s = 1.0f;
    bool require_acknowledgment = true;
    uint32_t ack_timeout_ms = 500;
};

enum class SafetyEventType : uint8_t {
    GEOFENCE_BREACH = 0,
    EMERGENCY_STOP = 1,
    RID_STATUS = 2,
    CRITICAL_FAULT = 3,
    COMMS_LOST = 4,
    LEADER_FAIL = 5,
    BATTERY_CRITICAL = 6,
    SENSOR_FAILURE = 7,
};

enum class SafetySeverity : uint8_t {
    INFO = 0,
    WARNING = 1,
    CRITICAL = 2,
    EMERGENCY = 3,
};

struct SafetyEvent {
    uint64_t timestamp_us;
    uint16_t source_id;
    SafetyEventType type;
    SafetySeverity severity;
    Vector3 position_enu;
    char details[128];
    uint32_t sequence;
};

class GeofenceMonitor {
public:
    GeofenceMonitor(HalInterface& hal) : _hal(hal) {}
    
    bool init(const GeofenceConfig& config);
    void update();
    
    // Check if position violates geofence
    GeofenceAction checkPosition(const Vector3& position_enu) const;
    
    // Get current geofence status
    bool isInsideGeofence() const { return _inside_geofence; }
    const GeofenceConfig& getConfig() const { return _config; }
    
    // Callback when breach detected
    using BreachCallback = std::function<void(GeofenceAction, const Vector3&)>;
    void setBreachCallback(BreachCallback cb) { _breach_cb = cb; }

private:
    HalInterface& _hal;
    GeofenceConfig _config;
    bool _inside_geofence = true;
    GeofenceAction _last_action = GeofenceAction::NONE;
    uint64_t _last_breach_us = 0;
    BreachCallback _breach_cb;
    
    bool pointInPolygon(const Vector3& pt, const GeofencePolygon& poly) const;
    float pointToPolygonDistance(const Vector3& pt, const GeofencePolygon& poly) const;
};

class RidBroadcaster {
public:
    RidBroadcaster(HalInterface& hal) : _hal(hal) {}
    
    bool init(const RemoteIdConfig& config);
    void update();
    
    // Broadcast Remote ID frame
    bool broadcastFrame(const RemoteIdFrame& frame);
    
    // Generate frame from current state
    RemoteIdFrame generateFrame() const;

private:
    HalInterface& _hal;
    RemoteIdConfig _config;
    uint64_t _last_broadcast_us = 0;
    uint16_t _sequence = 0;
};

class EmergencyStopHandler {
public:
    EmergencyStopHandler(HalInterface& hal, comm::MeshManager& mesh) 
        : _hal(hal), _mesh(mesh) {}
    
    bool init(const EmergencyStopConfig& config);
    
    // Trigger emergency stop (local or from mesh)
    bool triggerEmergencyStop(const char* reason, bool deploy_parachute = false);
    
    // Handle incoming emergency stop command
    void handleEmergencyStopCommand(uint16_t source_id, bool deploy_parachute);
    
    // Check if emergency stop is active
    bool isEmergencyStopActive() const { return _estop_active; }
    
    // Reset emergency stop (ground station only)
    bool resetEmergencyStop(uint16_t requester_id);

private:
    HalInterface& _hal;
    comm::MeshManager& _mesh;
    EmergencyStopConfig _config;
    bool _estop_active = false;
    uint64_t _estop_time_us = 0;
    bool _parachute_deployed = false;
    
    void executeMotorKill();
    void deployParachute();
};

class SafetyEventPublisher {
public:
    SafetyEventPublisher(comm::MeshManager& mesh) : _mesh(mesh) {}
    
    void publishEvent(SafetyEventType type, SafetySeverity severity, 
                      const Vector3& position, const char* details);
    
    void publishGeofenceBreach(const Vector3& position, const char* polygon_name);
    void publishEmergencyStop(const char* reason);
    void publishRidStatus(bool broadcasting);
    void publishCriticalFault(const char* component, const char* fault_desc);
    void publishCommsLost(uint16_t lost_drone_id);
    void publishLeaderFail(uint16_t failed_leader_id);
    void publishBatteryCritical(uint16_t drone_id, float battery_pct);
    void publishSensorFailure(const char* sensor_name);

private:
    comm::MeshManager& _mesh;
    uint32_t _sequence = 0;
};

} // namespace swarm::safety

#endif // SWARM_SAFETY_HPP