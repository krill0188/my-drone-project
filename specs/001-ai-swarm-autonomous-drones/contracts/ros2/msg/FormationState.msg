# FormationState.msg
# Formation maintenance state (leader + followers)
uint64 timestamp_us
uint32 formation_id
uint16 leader_id
uint16[] follower_ids

# Geometry
uint8 geometry_type                    # 0=LINE, 1=VEE, 2=DIAMOND, 3=COLUMN, 4=CUSTOM
geometry_msgs/Vector3[] relative_positions  # FLU frame
float32 spacing_m
float32 altitude_offset_m
float32 heading_offset_deg

# Tolerances
float32 max_lateral_error_m
float32 max_vertical_error_m
float32 max_velocity_error_mps
float32 max_yaw_error_deg
float32 rejoin_timeout_s
float32 replan_propagation_ms

# Follower states
uint16[] follower_sysids
geometry_msgs/Vector3[] position_errors_flu
geometry_msgs/Vector3[] velocity_errors_flu
float32[] yaw_errors_deg
uint8[] follower_statuses              # 0=FORMING, 1=HOLDING, 2=REJOINING, 3=LOST

uint8 formation_state                  # 0=FORMING, 1=HOLDING, 2=RECONFIGURING, 3=DISSOLVING
uint32 sequence