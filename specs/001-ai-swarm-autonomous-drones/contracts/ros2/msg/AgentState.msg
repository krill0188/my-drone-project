# AgentState.msg
# Real-time drone agent state (10 Hz)
uint64 timestamp_us                    # Monotonic microseconds
uint16 sysid                           # MAVLink system ID
uint16 compid                          # MAVLink component ID

# Pose (ENU frame)
geometry_msgs/Vector3 position
geometry_msgs/Quaternion orientation
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 angular_rate

# Health
float32 battery_pct                    # 0.0-1.0
float32 battery_voltage_v
uint16 remaining_time_s                # Estimated at current draw
uint8 health                           # 0=NOMINAL, 1=DEGRADED, 2=CRITICAL, 3=EMERGENCY

# Mode
uint8 flight_mode                      # 0=MANUAL, 1=GUIDED, 2=AUTO, 3=RTL, 4=LAND, 5=EMERGENCY
bool armed
bool gps_fix
uint8 gps_fix_type                     # 0=none, 1=2D, 2=3D, 3=RTK_FLOAT, 4=RTK_FIXED
uint8 num_satellites

# Localization quality
float32 pos_covariance_trace           # Trace of 3x3 position covariance
bool vio_valid
uint8 uwb_peers                        # Number of UWB ranges

# Communication
CommQoS[] mesh_qos                     # Per-peer QoS
bool cloud_connected

# Role
uint8 role                             # 0=IDLE, 1=LEADER, 2=FOLLOWER, 3=SCOUT, 4=RELAY
uint16 formation_id                    # 0 if none
FormationSlot formation_slot           # Valid if FOLLOWER

---
# FormationSlot
geometry_msgs/Vector3 offset_flu       # Relative position in leader FLU
float32 gain_xy
float32 gain_z
float32 gain_yaw