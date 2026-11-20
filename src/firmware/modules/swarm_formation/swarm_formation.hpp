#ifndef SWARM_FORMATION_HPP
#define SWARM_FORMATION_HPP

#include <cstdint>
#include <array>
#include <vector>
#include <optional>
#include "swarm_comm.hpp"

namespace swarm::formation {

// Formation geometry types
enum class GeometryType : uint8_t {
    LINE = 0,
    VEE = 1,
    DIAMOND = 2,
    COLUMN = 3,
    CUSTOM = 4,
};

struct Vector3 {
    float x, y, z;
    Vector3() : x(0), y(0), z(0) {}
    Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};

struct Quaternion {
    float w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};

struct FormationGeometry {
    GeometryType type = GeometryType::VEE;
    std::vector<Vector3> relative_positions;  // FLU frame, leader body
    float spacing_m = 5.0f;
    float altitude_offset_m = 0.0f;
    float heading_offset_deg = 0.0f;
};

struct FormationTolerances {
    float max_lateral_error_m = 0.5f;
    float max_vertical_error_m = 0.3f;
    float max_velocity_error_mps = 0.5f;
    float max_yaw_error_deg = 5.0f;
    float rejoin_timeout_s = 3.0f;
    float replan_propagation_ms = 200.0f;
};

struct FormationSlot {
    uint16_t follower_id = 0;
    Vector3 offset_flu;           // Relative position in leader FLU
    float gain_xy = 1.0f;
    float gain_z = 1.0f;
    float gain_yaw = 1.0f;
};

enum class FormationState : uint8_t {
    FORMING = 0,
    HOLDING = 1,
    RECONFIGURING = 2,
    DISSOLVING = 3,
};

enum class FollowerStatus : uint8_t {
    FORMING = 0,
    HOLDING = 1,
    REJOINING = 2,
    LOST = 3,
};

struct AgentState {
    uint64_t timestamp_us;
    uint16_t sysid;
    uint16_t compid;
    Vector3 position;
    Quaternion orientation;
    Vector3 velocity;
    Vector3 angular_rate;
    float battery_pct;
    float battery_voltage_v;
    uint16_t remaining_time_s;
    uint8_t health;  // 0=NOMINAL, 1=DEGRADED, 2=CRITICAL, 3=EMERGENCY
    uint8_t flight_mode;
    bool armed;
    bool gps_fix;
    uint8_t gps_fix_type;
    uint8_t num_satellites;
    float pos_covariance_trace;
    bool vio_valid;
    uint8_t uwb_peers;
    comm::LinkQoS mesh_qos;
    bool cloud_connected;
    uint8_t role;  // 0=IDLE, 1=LEADER, 2=FOLLOWER, 3=SCOUT, 4=RELAY
    uint32_t formation_id;
    FormationSlot formation_slot;
};

struct FormationStateMsg {
    uint64_t timestamp_us;
    uint32_t formation_id;
    uint16_t leader_id;
    std::vector<uint16_t> follower_ids;
    FormationGeometry geometry;
    FormationTolerances tolerances;
    std::vector<Vector3> position_errors_flu;
    std::vector<Vector3> velocity_errors_flu;
    std::vector<float> yaw_errors_deg;
    std::vector<uint8_t> follower_statuses;
    FormationState state;
    uint32_t sequence;
};

// Formation control gains
struct ControlGains {
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
};

class FormationController {
public:
    FormationController(comm::MeshManager& mesh) : _mesh(mesh) {}
    
    // Leader functions
    bool initAsLeader(uint32_t formation_id, const FormationGeometry& geometry, 
                      const FormationTolerances& tolerances);
    bool addFollower(uint16_t follower_id, const FormationSlot& slot);
    bool removeFollower(uint16_t follower_id);
    bool updateGeometry(const FormationGeometry& geometry);
    
    // Compute leader trajectory (waypoint following)
    Vector3 computeLeaderVelocity(const Vector3& current_pos, const Vector3& target_pos, float dt);
    
    // Broadcast formation setpoints to followers
    bool broadcastFormationSetpoints();
    
    // Follower functions
    bool initAsFollower(uint32_t formation_id, uint16_t leader_id);
    bool updateFollowerState(const AgentState& leader_state, const FormationStateMsg& formation_msg);
    Vector3 computeFollowerVelocity(const AgentState& own_state, const Vector3& target_offset_flu);
    
    // State machine
    FormationState getState() const { return _state; }
    void setState(FormationState state) { _state = state; }
    
    // Handle leader replan
    void onLeaderReplan(const Vector3& new_target, const Vector3& new_velocity);
    
    // Handle follower localization loss
    void onLocalizationLost();
    void onLocalizationRestored();
    
    // Get current formation state for publishing
    FormationStateMsg getFormationStateMsg() const;

private:
    comm::MeshManager& _mesh;
    
    // Configuration
    uint32_t _formation_id = 0;
    FormationGeometry _geometry;
    FormationTolerances _tolerances;
    std::vector<FormationSlot> _slots;
    uint16_t _leader_id = 0;
    bool _is_leader = false;
    
    // State
    FormationState _state = FormationState::DISSOLVING;
    uint32_t _sequence = 0;
    
    // Leader-specific
    Vector3 _leader_target_pos;
    Vector3 _leader_current_vel;
    std::vector<Vector3> _follower_targets_flu;
    
    // Follower-specific
    uint16_t _my_slot_index = 0;
    Vector3 _target_offset_flu;
    Vector3 _position_error_flu;
    Vector3 _velocity_error_flu;
    float _yaw_error_deg = 0.0f;
    FollowerStatus _follower_status = FollowerStatus::FORMING;
    uint64_t _last_leader_msg_us = 0;
    bool _localization_lost = false;
    
    // Control
    ControlGains _gains;
    Vector3 _pos_integral_xy{0, 0, 0};
    Vector3 _pos_integral_z{0, 0, 0};
    Vector3 _prev_pos_error_flu{0, 0, 0};
    
    // Private methods
    void computeFollowerOffsets();
    Vector3 rotateToFlu(const Vector3& vec_enu, const Quaternion& leader_orientation);
    Vector3 rotateToEnu(const Vector3& vec_flu, const Quaternion& leader_orientation);
    bool isFormationStable() const;
};

} // namespace swarm::formation

#endif // SWARM_FORMATION_HPP