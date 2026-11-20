/**
 * @file safety_events.cpp
 * @brief Safety Event MAVLink Publisher - DAL A
 * 
 * Publishes safety events via MAVLink 2.0:
 * - GEOFENCE_BREACH (custom msg 30010)
 * - EMERGENCY_STOP (custom msg 30011)
 * - RID_STATUS (custom msg 30012)
 * - CRITICAL_FAULT (custom msg 30013)
 * - COMMS_LOST (custom msg 30014)
 * - LEADER_FAIL (custom msg 30015)
 * - BATTERY_CRITICAL (custom msg 30016)
 * - SENSOR_FAILURE (custom msg 30017)
 * 
 * DAL A: Deterministic, no dynamic allocation, bounded execution time
 */

#include "swarm_safety/swarm_safety.hpp"
#include "hal_interface.hpp"
#include <cstring>
#include <cstdint>

namespace swarm::safety {

// MAVLink message IDs for safety events (custom range 30010-30017)
constexpr uint16_t MAVLINK_MSG_ID_GEOFENCE_BREACH = 30010;
constexpr uint16_t MAVLINK_MSG_ID_EMERGENCY_STOP = 30011;
constexpr uint16_t MAVLINK_MSG_ID_RID_STATUS = 30012;
constexpr uint16_t MAVLINK_MSG_ID_CRITICAL_FAULT = 30013;
constexpr uint16_t MAVLINK_MSG_ID_COMMS_LOST = 30014;
constexpr uint16_t MAVLINK_MSG_ID_LEADER_FAIL = 30015;
constexpr uint16_t MAVLINK_MSG_ID_BATTERY_CRITICAL = 30016;
constexpr uint16_t MAVLINK_MSG_ID_SENSOR_FAILURE = 30017;

// Static buffer for MAVLink messages (DAL A: no dynamic allocation)
namespace {
    constexpr int MAX_MAVLINK_PAYLOAD = 256;
    uint8_t g_mavlink_buffer[MAX_MAVLINK_PAYLOAD];
    uint32_t g_event_sequence = 0;
    HalInterface* g_hal_ptr = nullptr;
}

// CRC16 for MAVLink (X.25)
static uint16_t crc16_mavlink(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0x8408;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

// Pack MAVLink message header
static int pack_mavlink_header(uint8_t* buf, uint16_t msg_id, uint8_t sysid, uint8_t compid, uint8_t seq, uint8_t payload_len) {
    int idx = 0;
    buf[idx++] = 0xFD;  // MAVLink 2.0 magic
    buf[idx++] = payload_len;
    buf[idx++] = 0;  // Incompatibility flags
    buf[idx++] = 0;  // Compatibility flags
    buf[idx++] = seq;
    buf[idx++] = sysid;
    buf[idx++] = compid;
    // Message ID (little endian, 24-bit)
    buf[idx++] = msg_id & 0xFF;
    buf[idx++] = (msg_id >> 8) & 0xFF;
    buf[idx++] = (msg_id >> 16) & 0xFF;
    return idx;
}

// Finalize and send MAVLink message
static bool send_mavlink_message(uint16_t msg_id, uint8_t sysid, uint8_t compid, 
                                  const uint8_t* payload, uint8_t payload_len) {
    if (!g_hal_ptr) return false;
    
    int idx = pack_mavlink_header(g_mavlink_buffer, msg_id, sysid, compid, 
                                   g_event_sequence & 0xFF, payload_len);
    
    // Copy payload
    std::memcpy(&g_mavlink_buffer[idx], payload, payload_len);
    idx += payload_len;
    
    // CRC (payload + header excluding magic)
    uint16_t crc = crc16_mavlink(&g_mavlink_buffer[1], idx - 1);
    g_mavlink_buffer[idx++] = crc & 0xFF;
    g_mavlink_buffer[idx++] = (crc >> 8) & 0xFF;
    
    g_event_sequence++;
    
    return g_hal_ptr->sendMavlinkMessage(g_mavlink_buffer, idx);
}

// ============================================================================
// SafetyEventPublisher Implementation
// ============================================================================

SafetyEventPublisher::SafetyEventPublisher(comm::MeshManager& mesh) : _mesh(mesh) {
    _sequence = 0;
}

void SafetyEventPublisher::setHalInterface(HalInterface* hal) {
    g_hal_ptr = hal;
}

void SafetyEventPublisher::publishEvent(SafetyEventType type, SafetySeverity severity,
                                        const Vector3& position, const char* details) {
    SafetyEvent event;
    event.timestamp_us = g_hal_ptr ? g_hal_ptr->getMonotonicTimeUs() : 0;
    event.source_id = 1;  // Own drone ID
    event.type = type;
    event.severity = severity;
    event.position_enu = position;
    event.sequence = ++_sequence;
    
    if (details) {
        std::strncpy(event.details, details, sizeof(event.details) - 1);
        event.details[sizeof(event.details) - 1] = '\0';
    } else {
        event.details[0] = '\0';
    }
    
    // Route to specific publisher
    switch (type) {
        case SafetyEventType::GEOFENCE_BREACH:
            publishGeofenceBreachMavlink(event);
            break;
        case SafetyEventType::EMERGENCY_STOP:
            publishEmergencyStopMavlink(event);
            break;
        case SafetyEventType::RID_STATUS:
            publishRidStatusMavlink(event);
            break;
        case SafetyEventType::CRITICAL_FFAULT:
            publishCriticalFaultMavlink(event);
            break;
        case SafetyEventType::COMMS_LOST:
            publishCommsLostMavlink(event);
            break;
        case SafetyEventType::LEADER_FAIL:
            publishLeaderFailMavlink(event);
            break;
        case SafetyEventType::BATTERY_CRITICAL:
            publishBatteryCriticalMavlink(event);
            break;
        case SafetyEventType::SENSOR_FAILURE:
            publishSensorFailureMavlink(event);
            break;
    }
    
    // Also publish via mesh for swarm awareness
    publishViaMesh(event);
}

void SafetyEventPublisher::publishGeofenceBreach(const Vector3& position, const char* polygon_name) {
    char details[128];
    std::snprintf(details, sizeof(details), "Geofence breach: %s", polygon_name ? polygon_name : "unknown");
    publishEvent(SafetyEventType::GEOFENCE_BREACH, SafetySeverity::CRITICAL, position, details);
}

void SafetyEventPublisher::publishEmergencyStop(const char* reason) {
    Vector3 pos{0, 0, 0};
    if (g_hal_ptr) {
        auto gps = g_hal_ptr->readGps0();
        if (gps && gps->healthy) {
            pos = Vector3(
                static_cast<float>(gps->longitude_deg * 111320.0),
                static_cast<float>(gps->latitude_deg * 111320.0),
                gps->altitude_m
            );
        }
    }
    publishEvent(SafetyEventType::EMERGENCY_STOP, SafetySeverity::EMERGENCY, pos, reason);
}

void SafetyEventPublisher::publishRidStatus(bool broadcasting) {
    Vector3 pos{0, 0, 0};
    char details[128];
    std::snprintf(details, sizeof(details), "RID: %s", broadcasting ? "broadcasting" : "stopped");
    publishEvent(SafetyEventType::RID_STATUS, broadcasting ? SafetySeverity::INFO : SafetySeverity::WARNING, pos, details);
}

void SafetyEventPublisher::publishCriticalFault(const char* component, const char* fault_desc) {
    Vector3 pos{0, 0, 0};
    char details[128];
    std::snprintf(details, sizeof(details), "Fault: %s - %s", component, fault_desc);
    publishEvent(SafetyEventType::CRITICAL_FAULT, SafetySeverity::CRITICAL, pos, details);
}

void SafetyEventPublisher::publishCommsLost(uint16_t lost_drone_id) {
    Vector3 pos{0, 0, 0};
    char details[128];
    std::snprintf(details, sizeof(details), "Comms lost with drone %u", lost_drone_id);
    publishEvent(SafetyEventType::COMMS_LOST, SafetySeverity::WARNING, pos, details);
}

void SafetyEventPublisher::publishLeaderFail(uint16_t failed_leader_id) {
    Vector3 pos{0, 0, 0};
    char details[128];
    std::snprintf(details, sizeof(details), "Leader %u failed", failed_leader_id);
    publishEvent(SafetyEventType::LEADER_FAIL, SafetySeverity::CRITICAL, pos, details);
}

void SafetyEventPublisher::publishBatteryCritical(uint16_t drone_id, float battery_pct) {
    Vector3 pos{0, 0, 0};
    char details[128];
    std::snprintf(details, sizeof(details), "Drone %u battery critical: %.1f%%", drone_id, battery_pct * 100);
    publishEvent(SafetyEventType::BATTERY_CRITICAL, SafetySeverity::CRITICAL, pos, details);
}

void SafetyEventPublisher::publishSensorFailure(const char* sensor_name) {
    Vector3 pos{0, 0, 0};
    char details[128];
    std::snprintf(details, sizeof(details), "Sensor failure: %s", sensor_name);
    publishEvent(SafetyEventType::SENSOR_FAILURE, SafetySeverity::CRITICAL, pos, details);
}

// ============================================================================
// MAVLink Message Builders
// ============================================================================

#pragma pack(push, 1)

struct MavlinkGeofenceBreach {
    uint64_t timestamp_us;
    float position_x;
    float position_y;
    float position_z;
    uint8_t action;        // GeofenceAction enum
    uint8_t severity;      // SafetySeverity enum
    char polygon_name[32];
};

struct MavlinkEmergencyStop {
    uint64_t timestamp_us;
    float position_x;
    float position_y;
    float position_z;
    uint8_t reason_code;   // 0=local, 1=remote, 2=auto
    bool parachute_deployed;
    char reason[64];
};

struct MavlinkRidStatus {
    uint64_t timestamp_us;
    uint8_t status;        // 0=broadcasting, 1=stopped, 2=error
    uint8_t error_code;
    uint32_t frames_sent;
    uint32_t frames_failed;
};

struct MavlinkCriticalFault {
    uint64_t timestamp_us;
    float position_x;
    float position_y;
    float position_z;
    uint8_t fault_type;    // FaultType enum
    uint8_t severity;      // FaultSeverity enum
    char component[32];
    char fault_desc[64];
};

struct MavlinkCommsLost {
    uint64_t timestamp_us;
    uint16_t lost_drone_id;
    uint8_t comm_type;     // 0=mesh, 1=5G, 2=LoRa
    float last_rssi;
    uint32_t lost_duration_ms;
};

struct MavlinkLeaderFail {
    uint64_t timestamp_us;
    uint16_t failed_leader_id;
    uint16_t new_leader_id;  // 0 if election pending
    uint8_t fail_reason;     // 0=comms, 1=fault, 2=estop
};

struct MavlinkBatteryCritical {
    uint64_t timestamp_us;
    uint16_t drone_id;
    float battery_percentage;
    float voltage_v;
    float current_a;
    uint8_t cell_count;
    float min_cell_v;
};

struct MavlinkSensorFailure {
    uint64_t timestamp_us;
    uint8_t sensor_type;     // 0=IMU, 1=GPS, 2=Baro, 3=Mag, 4=Camera, 5=Lidar, 6=UWB
    uint8_t sensor_instance; // 0 or 1 for redundant
    uint8_t failure_mode;    // 0=stuck, 1=noisy, 2=offline, 3=diverged
    float last_value[3];
};

#pragma pack(pop)

void SafetyEventPublisher::publishGeofenceBreachMavlink(const SafetyEvent& event) {
    MavlinkGeofenceBreach msg;
    msg.timestamp_us = event.timestamp_us;
    msg.position_x = event.position_enu.x();
    msg.position_y = event.position_enu.y();
    msg.position_z = event.position_enu.z();
    msg.action = 2;  // RTL (would come from GeofenceMonitor)
    msg.severity = static_cast<uint8_t>(event.severity);
    std::strncpy(msg.polygon_name, event.details, sizeof(msg.polygon_name) - 1);
    msg.polygon_name[sizeof(msg.polygon_name) - 1] = '\0';
    
    send_mavlink_message(MAVLINK_MSG_ID_GEOFENCE_BREACH, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishEmergencyStopMavlink(const SafetyEvent& event) {
    MavlinkEmergencyStop msg;
    msg.timestamp_us = event.timestamp_us;
    msg.position_x = event.position_enu.x();
    msg.position_y = event.position_enu.y();
    msg.position_z = event.position_enu.z();
    msg.reason_code = 0;  // Would be passed in details
    msg.parachute_deployed = false;  // Would come from EmergencyStopHandler
    std::strncpy(msg.reason, event.details, sizeof(msg.reason) - 1);
    msg.reason[sizeof(msg.reason) - 1] = '\0';
    
    send_mavlink_message(MAVLINK_MSG_ID_EMERGENCY_STOP, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishRidStatusMavlink(const SafetyEvent& event) {
    MavlinkRidStatus msg;
    msg.timestamp_us = event.timestamp_us;
    msg.status = std::strstr(event.details, "broadcasting") ? 0 : 1;
    msg.error_code = 0;
    msg.frames_sent = 0;  // Would come from RidBroadcaster
    msg.frames_failed = 0;
    
    send_mavlink_message(MAVLINK_MSG_ID_RID_STATUS, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishCriticalFaultMavlink(const SafetyEvent& event) {
    MavlinkCriticalFault msg;
    msg.timestamp_us = event.timestamp_us;
    msg.position_x = event.position_enu.x();
    msg.position_y = event.position_enu.y();
    msg.position_z = event.position_enu.z();
    msg.fault_type = 0;  // Would parse from details
    msg.severity = static_cast<uint8_t>(event.severity);
    
    // Parse component and fault_desc from details
    char* dash = std::strstr(event.details, " - ");
    if (dash) {
        size_t comp_len = dash - event.details;
        std::strncpy(msg.component, event.details, std::min(comp_len, sizeof(msg.component) - 1));
        msg.component[sizeof(msg.component) - 1] = '\0';
        std::strncpy(msg.fault_desc, dash + 3, sizeof(msg.fault_desc) - 1);
    } else {
        std::strncpy(msg.component, event.details, sizeof(msg.component) - 1);
        msg.component[sizeof(msg.component) - 1] = '\0';
        msg.fault_desc[0] = '\0';
    }
    
    send_mavlink_message(MAVLINK_MSG_ID_CRITICAL_FAULT, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishCommsLostMavlink(const SafetyEvent& event) {
    MavlinkCommsLost msg;
    msg.timestamp_us = event.timestamp_us;
    msg.lost_drone_id = 0;  // Would parse from details
    msg.comm_type = 0;  // mesh
    msg.last_rssi = -100.0f;
    msg.lost_duration_ms = 0;
    
    // Parse lost_drone_id from details
    // "Comms lost with drone %u"
    uint16_t id = 0;
    std::sscanf(event.details, "Comms lost with drone %hu", &id);
    msg.lost_drone_id = id;
    
    send_mavlink_message(MAVLINK_MSG_ID_COMMS_LOST, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishLeaderFailMavlink(const SafetyEvent& event) {
    MavlinkLeaderFail msg;
    msg.timestamp_us = event.timestamp_us;
    msg.failed_leader_id = 0;  // Would parse
    msg.new_leader_id = 0;
    msg.fail_reason = 0;
    
    uint16_t id = 0;
    std::sscanf(event.details, "Leader %hu failed", &id);
    msg.failed_leader_id = id;
    
    send_mavlink_message(MAVLINK_MSG_ID_LEADER_FAIL, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishBatteryCriticalMavlink(const SafetyEvent& event) {
    MavlinkBatteryCritical msg;
    msg.timestamp_us = event.timestamp_us;
    msg.drone_id = 0;
    msg.battery_percentage = 0.0f;
    msg.voltage_v = 0.0f;
    msg.current_a = 0.0f;
    msg.cell_count = 0;
    msg.min_cell_v = 0.0f;
    
    // Parse: "Drone %u battery critical: %.1f%%"
    uint16_t id = 0;
    float pct = 0.0f;
    std::sscanf(event.details, "Drone %hu battery critical: %f%%", &id, &pct);
    msg.drone_id = id;
    msg.battery_percentage = pct / 100.0f;
    
    send_mavlink_message(MAVLINK_MSG_ID_BATTERY_CRITICAL, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

void SafetyEventPublisher::publishSensorFailureMavlink(const SafetyEvent& event) {
    MavlinkSensorFailure msg;
    msg.timestamp_us = event.timestamp_us;
    msg.sensor_type = 0;  // Would parse from details
    msg.sensor_instance = 0;
    msg.failure_mode = 0;
    msg.last_value[0] = msg.last_value[1] = msg.last_value[2] = 0.0f;
    
    // Parse sensor name from "Sensor failure: %s"
    // Map to enum
    if (std::strstr(event.details, "IMU")) msg.sensor_type = 0;
    else if (std::strstr(event.details, "GPS")) msg.sensor_type = 1;
    else if (std::strstr(event.details, "Baro")) msg.sensor_type = 2;
    else if (std::strstr(event.details, "Mag")) msg.sensor_type = 3;
    else if (std::strstr(event.details, "Camera")) msg.sensor_type = 4;
    else if (std::strstr(event.details, "Lidar")) msg.sensor_type = 5;
    else if (std::strstr(event.details, "UWB")) msg.sensor_type = 6;
    
    send_mavlink_message(MAVLINK_MSG_ID_SENSOR_FAILURE, 1, 1,
                         reinterpret_cast<const uint8_t*>(&msg), sizeof(msg));
}

// ============================================================================
// Mesh Publication
// ============================================================================

void SafetyEventPublisher::publishViaMesh(const SafetyEvent& event) {
    // Serialize event for mesh broadcast
    // Use existing swarm_comm mesh manager
    // This would send to all swarm members for coordinated response
    
    uint8_t mesh_payload[256];
    int idx = 0;
    
    // Event header
    *reinterpret_cast<uint64_t*>(&mesh_payload[idx]) = event.timestamp_us; idx += 8;
    *reinterpret_cast<uint16_t*>(&mesh_payload[idx]) = event.source_id; idx += 2;
    mesh_payload[idx++] = static_cast<uint8_t>(event.type);
    mesh_payload[idx++] = static_cast<uint8_t>(event.severity);
    *reinterpret_cast<uint32_t*>(&mesh_payload[idx]) = event.sequence; idx += 4;
    
    // Position
    *reinterpret_cast<float*>(&mesh_payload[idx]) = event.position_enu.x(); idx += 4;
    *reinterpret_cast<float*>(&mesh_payload[idx]) = event.position_enu.y(); idx += 4;
    *reinterpret_cast<float*>(&mesh_payload[idx]) = event.position_enu.z(); idx += 4;
    
    // Details (truncated)
    size_t details_len = std::min(strlen(event.details), size_t(128));
    mesh_payload[idx++] = static_cast<uint8_t>(details_len);
    std::memcpy(&mesh_payload[idx], event.details, details_len);
    idx += details_len;
    
    // Send via mesh (best effort, store-and-forward)
    // _mesh.broadcastMessage(SwarmMsgId::SAFETY_EVENT, mesh_payload, idx);
}

}  // namespace swarm::safety