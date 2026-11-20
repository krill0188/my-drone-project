# AIInference.msg
# AI perception/fault/landing inference results
uint64 timestamp_us
uint16 drone_id
uint8 type                             # 0=OBSTACLE, 1=LANDING_SITE, 2=FAULT, 3=ANOMALY
uint16 class_id
string class_name
float32 confidence
geometry_msgs/Vector3 position_enu     # Valid for OBSTACLE/LANDING
geometry_msgs/Vector3 size             # 3D bounding box size
geometry_msgs/Quaternion orientation   # Bounding box orientation
InferenceMetadata metadata

---
# InferenceMetadata
uint32 latency_ms
string model_name
string model_version
string runtime                         # "tensorrt_fp16", "onnxruntime"
ObstacleMeta obstacle
LandingMeta landing
FaultMeta fault

---
# ObstacleMeta
uint8 threat_level                     # 0=LOW, 1=MEDIUM, 2=HIGH, 3=CRITICAL
geometry_msgs/Vector3 relative_velocity
float32 time_to_collision_s
bool requires_immediate_avoidance

---
# LandingMeta
float32 slope_deg
float32 clearance_radius_m
uint8 surface_type                     # 0=GRASS, 1=CONCRETE, 2=DIRT, 3=SAND, 4=WATER, 5=UNKNOWN
float32 firmness_score

---
# FaultMeta
uint8 fault_type                       # 0=IMU_DRIFT, 1=GPS_SPOOF, 2=MOTOR_DEGRADE, 3=BATTERY_IMBALANCE, 4=SENSOR_FAULT, 5=COMM_DEGRADE
uint8 severity                         # 0=INFO, 1=WARNING, 2=CRITICAL, 3=EMERGENCY
string affected_component
float32 degradation_pct
bool requires_replan
uint8 recommended_action               # 0=CONTINUE, 1=DEGRADED, 2=RTL, 3=EMERGENCY_LAND