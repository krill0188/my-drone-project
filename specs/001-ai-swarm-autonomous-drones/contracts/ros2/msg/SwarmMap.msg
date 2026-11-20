# SwarmMap.msg
# Shared spatial representation
uint64 timestamp_us
uint64 version                         # Vector clock / monotonically increasing
geometry_msgs/Vector3 origin           # ENU frame origin (mission home)
float32 resolution_m                   # Grid cell size (0.1-1.0)

# Occupancy grid (flattened 2D/3D)
float32[] occupancy_probability        # 0.0=free, 1.0=occupied, size = width*height*depth
uint32[] occupancy_hits
uint32[] occupancy_misses
uint64[] occupancy_last_update_us
uint32 grid_width
uint32 grid_height
uint32 grid_depth

# Semantic labels (per cell, same dimensions)
uint8[] semantic_class                 # 0=UNKNOWN, 1=FREE, 2=OBSTACLE, 3=STRUCTURE, 4=VEGETATION, 5=WATER, 6=NO_FLY, 7=LANDING_PAD
float32[] semantic_confidence
uint64[] semantic_timestamp_us
uint16[] semantic_source_drone

# No-fly zones
NoFlyZone[] no_fly_zones

# Landmarks
Landmark[] landmarks

uint16[] contributors                  # Source drone sysids

---
# NoFlyZone
uint8[16] id                           # UUID
geometry_msgs/Polygon boundary         # 2D polygon in ENU
float32 min_altitude_m
float32 max_altitude_m
uint64 valid_from_us
uint64 valid_until_us
string authority                       # Regulatory authority
string signature                       # Cryptographic signature

---
# Landmark
uint8[16] id                           # UUID
geometry_msgs/Vector3 position_enu
string type                            # "visual", "uwb_anchor", "apriltag", "known_structure"
float32 confidence
uint64 timestamp_us
uint16 source_drone