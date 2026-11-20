# Data Model: AI Swarm Autonomous Drone System

**Generated**: 2025-11-19 | **Phase**: 1 (Design) | **Spec**: [spec.md](spec.md) | **Research**: [research.md](research.md)

---

## Entity Definitions

### 1. DroneAgent

**Description**: Individual UAV with identity, capabilities, state, and role in swarm.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | `uint16` | Unique, 1–65535 | MAVLink sysid + compid composite |
| `type` | `enum` | `QUAD`, `HEX`, `VTOL`, `FIXED_WING` | Airframe type |
| `capabilities` | `Capability[]` | Non-empty | Sensors, payload, compute, endurance |
| `state` | `AgentState` | Required | Real-time dynamic state |
| `role` | `enum` | `LEADER`, `FOLLOWER`, `SCOUT`, `RELAY`, `IDLE` | Current swarm role |
| `formation_slot` | `FormationSlot?` | If follower | Assigned position in formation |
| `task_assignment` | `TaskAssignment?` | If allocated | Current task reference |

**Capability** (struct):
```c
struct Capability {
    bool has_rtk_gps;
    bool has_stereo_camera;
    bool has_lidar_2d;
    bool has_lidar_3d;
    bool has_uwb;
    bool has_5g_modem;
    bool has_ai_compute;      // Jetson Orin / Snapdragon
    uint16 max_flight_time_s; // seconds
    float max_payload_kg;
    uint8  comm_tier_bitmap;  // bit0=WiFi6E, bit1=5G, bit2=LoRa
}
```

**AgentState** (struct, updated at 10 Hz):
```c
struct AgentState {
    // Pose (ENU frame, origin = mission home)
    Vector3 position;      // m
    Quaternion orientation; // ENU from body
    Vector3 velocity;      // m/s
    Vector3 angular_rate;  // rad/s
    
    // Health
    float battery_pct;           // 0.0–1.0
    float battery_voltage_v;
    uint16 remaining_time_s;     // estimated at current draw
    HealthStatus health;         // NOMINAL, DEGRADED, CRITICAL, EMERGENCY
    
    // Mode
    FlightMode mode;             // MANUAL, GUIDED, AUTO, RTL, LAND, EMERGENCY
    bool armed;
    bool gps_fix;                // RTK fixed/float/none
    uint8 num_satellites;
    
    // Localization quality
    float pos_covariance_trace;  // trace of 3x3 position covariance
    bool vio_valid;
    uint8 uwb_peers;             // number of UWB ranges
    
    // Communication
    MeshLinkQuality mesh_qos;    // per-peer QoS
    bool cloud_connected;
    
    // Timestamp
    uint64 timestamp_us;         // monotonic, microseconds
}
```

**Validation Rules**:
- `battery_pct > 0.20` for mission start (FR-001, SC-001)
- `pos_covariance_trace < 0.25` (0.5 m 1σ) for formation hold (FR-002, SC-001)
- `health != CRITICAL` for task allocation (FR-006)
- `mode == AUTO` or `GUIDED` for swarm operations (FR-001, FR-004)

**State Machine**:
```
IDLE → ARMING → ARMED → TAKEOFF → MISSION_ACTIVE → RTL → LANDING → LANED → DISARMED
                              ↓
                         EMERGENCY (from any state via estop/geofence/fault)
                              ↓
                         EMERGENCY_LAND / MOTOR_KILL
```

---

### 2. Formation

**Description**: Geometric template defining relative positions, maintenance tolerances, and transition rules.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | `uint32` | Unique | Formation instance ID |
| `geometry` | `FormationGeometry` | Required | Template definition |
| `tolerances` | `FormationTolerances` | Required | Position/velocity error bounds |
| `leader_id` | `uint16` | Valid DroneAgent | Current leader |
| `follower_ids` | `uint16[]` | 1–49, unique | Assigned followers |
| `state` | `enum` | `FORMING`, `HOLDING`, `RECONFIGURING`, `DISSOLVING` | Lifecycle |
| `transition_rules` | `TransitionRule[]` | Optional | Join/leave/split/merge logic |

**FormationGeometry** (struct):
```c
struct FormationGeometry {
    enum Type { LINE, VEE, DIAMOND, COLUMN, CUSTOM };
    Type type;
    // For CUSTOM: explicit relative positions in leader body frame (FLU)
    Vector3 relative_positions[]; // size = max_followers
    float spacing_m;              // nominal inter-drone distance
    float altitude_offset_m;      // leader-relative
    float heading_offset_deg;     // formation yaw relative to leader
}
```

**FormationTolerances** (struct):
```c
struct FormationTolerances {
    float max_lateral_error_m;    // ≤ 0.5 (FR-002, SC-001)
    float max_vertical_error_m;   // ≤ 0.3 (FR-002, SC-001)
    float max_velocity_error_mps; // ≤ 0.5
    float max_yaw_error_deg;      // ≤ 5.0
    float rejoin_timeout_s;       // ≤ 3.0 (FR-003)
    float replan_propagation_ms;  // ≤ 200 (FR-003, SC-007)
}
```

**TransitionRule** (struct):
```c
struct TransitionRule {
    enum Trigger { FOLLOWER_JOIN, FOLLOWER_LEAVE, LEADER_FAIL, SPLIT, MERGE };
    Trigger trigger;
    string condition;             // e.g., "battery > 30% && comms_ok"
    string action;                // e.g., "reassign_slots", "bully_election"
    uint32 timeout_ms;            // max transition time
}
```

**Validation Rules**:
- `follower_ids` size matches `geometry.relative_positions` for CUSTOM
- `leader_id` not in `follower_ids`
- All `follower_ids` have `role == FOLLOWER` and `formation_slot` assigned
- `tolerances` meet constitutional minimums (SC-001)

**State Machine**:
```
FORMING → HOLDING ↔ RECONFIGURING → DISSOLVING
    ↑           ↓
    └───────────┘ (on leader fail / split / merge)
```

---

### 3. Mission

**Description**: High-level intent with constraints, decomposable into tasks.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | `uuid` | Unique (RFC 4122) | Mission identifier |
| `type` | `enum` | `WAYPOINT`, `SEARCH`, `INSPECT`, `PATROL`, `ESCORT` | Mission category |
| `priority` | `uint8` | 1–255 (1=highest) | Preemption priority |
| `constraints` | `MissionConstraints` | Required | Geofence, altitude, time, energy |
| `tasks` | `Task[]` | Non-empty | Decomposed work units |
| `status` | `enum` | `PLANNED`, `UPLOADED`, `ACTIVE`, `PAUSED`, `COMPLETED`, `FAILED`, `ABORTED` | Lifecycle |
| `assigned_swarm` | `uint16[]` | Valid DroneAgents | Participating drones |
| `created_by` | `string` | Operator ID | Traceability |
| `created_at` | `uint64` | Unix ms | Timestamp |

**MissionConstraints** (struct):
```c
struct MissionConstraints {
    Polygon geofence;              // Hard boundary (FR-011)
    float max_altitude_m;          // AMSL
    float min_altitude_m;          // AGL
    float max_speed_mps;
    uint32 max_duration_s;         // Time budget
    float min_battery_pct;         // 0.20 reserve
    NoFlyZone[] no_fly_zones;      // Dynamic updates
    WeatherConstraints weather;    // Wind, precip, visibility
    bool require_rid;              // Remote ID mandatory
    Jurisdiction jurisdiction;     // Regulatory domain
}
```

**Validation Rules**:
- All task polygons within `geofence` (FR-011)
- `max_duration_s` ≤ min drone `remaining_time_s` × 0.8
- `require_rid` true for BVLOS / controlled airspace (FR-012, SC-010)
- `no_fly_zones` cryptographically signed (Constitution Regulatory)

**State Machine**:
```
PLANNED → UPLOADED → ACTIVE ↔ PAUSED → COMPLETED
                    ↓
                 FAILED (on critical fault)
                    ↓
                 ABORTED (operator/estop)
```

---

### 4. Task

**Description**: Atomic work unit allocated to a drone with dependencies and success criteria.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | `uuid` | Unique | Task identifier |
| `mission_id` | `uuid` | Parent mission | Back-reference |
| `type` | `enum` | `COVER_AREA`, `INSPECT_POI`, `RELAY_COMMS`, `HOLD_POSITION`, `SCOUT`, `ESCORT` | Work category |
| `priority` | `uint8` | 1–255 | Local priority |
| `geometry` | `TaskGeometry` | Required | Spatial definition |
| `dependencies` | `uuid[]` | Acyclic | Precedence constraints |
| `assigned_drone` | `uint16?` | Valid DroneAgent if allocated | Allocation result |
| `status` | `enum` | `PENDING`, `ALLOCATED`, `EXECUTING`, `COMPLETED`, `FAILED`, `REEVALUATING` | Lifecycle |
| `success_criteria` | `SuccessCriteria` | Required | Measurable completion |
| `progress` | `float` | 0.0–1.0 | Execution progress |
| `started_at` | `uint64?` | Unix ms | Actual start |
| `completed_at` | `uint64?` | Unix ms | Actual completion |

**TaskGeometry** (variant):
```c
union TaskGeometry {
    Polygon area;           // COVER_AREA: search polygon with pattern (lawnmower/spiral)
    Vector3 poi_position;   // INSPECT_POI: 3D point + view angles
    Vector3 hold_position;  // HOLD_POSITION: 3D point + radius
    Vector3 relay_position; // RELAY_COMMS: optimal link position
    Waypoint[] waypoints;   // ESCORT/SCOUT: sequence
}
```

**SuccessCriteria** (struct):
```c
struct SuccessCriteria {
    float min_coverage_pct;       // COVER_AREA: ≥ 0.95 (FR-005, SC-002)
    float max_inspection_dist_m;  // INSPECT_POI: ≤ 5.0
    float max_hold_error_m;       // HOLD_POSITION: ≤ 2.0
    uint32 max_duration_s;        // Time budget
    bool require_data_return;     // Sensor data uploaded
}
```

**Validation Rules**:
- No circular dependencies (topological sort valid)
- `geometry` consistent with `type`
- `success_criteria` measurable and technology-agnostic (Constitution)
- `assigned_drone` has required `capabilities` for `type`

**State Machine**:
```
PENDING → ALLOCATED → EXECUTING → COMPLETED
                ↓              ↓
             FAILED ←─────────┘ (on fault/comms loss)
                ↓
           REEVALUATING → ALLOCATED (reallocation, FR-006, SC-003)
```

---

### 5. SwarmMap

**Description**: Shared spatial representation fused from all drones via mesh.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `version` | `uint64` | Monotonically increasing | Map version (vector clock) |
| `origin` | `Vector3` | ENU frame origin | Mission home position |
| `resolution_m` | `float` | 0.1–1.0 | Grid cell size |
| `occupancy_grid` | `OccupancyCell[]` | 2D/3D array | Probabilistic occupancy |
| `semantic_labels` | `SemanticLabel[]` | Per-cell | Class: UNKNOWN, FREE, OBSTACLE, STRUCTURE, VEGETATION, WATER, NO_FLY |
| `no_fly_zones` | `NoFlyZone[]` | Signed, timestamped | Regulatory + dynamic |
| `landmarks` | `Landmark[]` | Visual/UWB anchors | Absolute localization aids |
| `last_update` | `uint64` | Unix ms | Freshness |
| `contributors` | `uint16[]` | DroneAgent IDs | Source drones |

**OccupancyCell** (struct):
```c
struct OccupancyCell {
    float probability;      // 0.0 (free) – 1.0 (occupied)
    uint32 hits;            // Sensor hits
    uint32 misses;          // Sensor misses
    uint64 last_update_us;  // Cell freshness
}
```

**SemanticLabel** (struct):
```c
struct SemanticLabel {
    enum Class { UNKNOWN, FREE, OBSTACLE, STRUCTURE, VEGETATION, WATER, NO_FLY, LANDING_PAD };
    Class class;
    float confidence;       // 0.0–1.0 (from AI inference)
    uint64 timestamp_us;
    uint16 source_drone;    // Origin
}
```

**Validation Rules**:
- `resolution_m` ≤ 0.5 for obstacle avoidance (FR-007, FR-018)
- `semantic_labels.confidence` ≥ 0.7 for action (FR-007)
- `no_fly_zones` signature verified before use (FR-011, Constitution)
- Map fusion: Bayesian update with contributor weighting by sensor quality
- Stale cells (> 30 s) decay to UNKNOWN (FR-017 GPS-denied)

---

### 6. AIInferenceResult

**Description**: Timestamped perception/fault/landing output from onboard AI pipeline.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `timestamp_us` | `uint64` | Monotonic, microseconds | Inference completion time |
| `drone_id` | `uint16` | Source drone | Producer |
| `type` | `enum` | `OBSTACLE`, `LANDING_SITE`, `FAULT`, `ANOMALY` | Inference category |
| `class_id` | `uint16` | Domain-specific | Detected class |
| `class_name` | `string` | Human-readable | e.g., "power_line", "flat_grass", "motor_degradation" |
| `confidence` | `float` | 0.0–1.0 | Model confidence |
| `position_3d` | `Vector3?` | ENU frame | Obstacle/landing position |
| `bounding_box_3d` | `Box3D?` | ENU frame | 3D extent |
| `metadata` | `InferenceMetadata` | Additional context | Latency, model version, etc. |

**InferenceMetadata** (struct):
```c
struct InferenceMetadata {
    uint32 latency_ms;           // End-to-end (FR-010, SC-007)
    string model_name;           // e.g., "yolov8n_obstacle_v3"
    string model_version;        // Semantic version
    string runtime;              // "tensorrt_fp16" / "onnxruntime"
    float preprocessing_ms;
    float inference_ms;
    float postprocessing_ms;
    // Type-specific
    ObstacleMeta? obstacle;
    LandingMeta? landing;
    FaultMeta? fault;
}
```

**ObstacleMeta** (struct):
```c
struct ObstacleMeta {
    enum ThreatLevel { LOW, MEDIUM, HIGH, CRITICAL };
    ThreatLevel threat;
    Vector3 relative_velocity;   // If tracked
    float time_to_collision_s;   // TTC
    bool requires_immediate_avoidance; // TTC < 2.0 s
}
```

**LandingMeta** (struct):
```c
struct LandingMeta {
    float slope_deg;             // < 10° (FR-008)
    float clearance_radius_m;    // > 2.0 m (FR-008)
    enum SurfaceType { GRASS, CONCRETE, DIRT, SAND, WATER, UNKNOWN };
    SurfaceType surface;
    float firmness_score;        // 0.0–1.0
}
```

**FaultMeta** (struct):
```c
struct FaultMeta {
    enum FaultType { IMU_DRIFT, GPS_SPOOF, MOTOR_DEGRADE, BATTERY_IMBALANCE, SENSOR_FAULT, COMM_DEGRADE };
    FaultType type;
    enum Severity { INFO, WARNING, CRITICAL, EMERGENCY };
    Severity severity;
    string affected_component;   // e.g., "motor_3", "gps_1", "imu_0"
    float degradation_pct;       // 0.0–1.0
    bool requires_replan;        // Triggers FR-010
    float recommended_action;    // Enum: CONTINUE, DEGRADED, RTL, EMERGENCY_LAND
}
```

**Validation Rules**:
- `confidence` ≥ 0.7 for autonomous action (FR-007, FR-008, FR-009)
- `latency_ms` < 50 (FR-010, research decision)
- `threat_level == CRITICAL` → immediate avoidance (FR-010, SC-007)
- `severity >= CRITICAL` → emergency stop consideration (FR-013)

---

### 7. CommunicationLink

**Description**: Mesh network topology with per-link QoS and routing.

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `src_id` | `uint16` | Source drone |  |
| `dst_id` | `uint16` | Destination drone |  |
| `link_type` | `enum` | `WIFI_6E`, `FIVE_G_NR`, `LORA`, `MAVLINK_SERIAL` | Physical layer |
| `qos` | `LinkQoS` | Real-time measured | Quality metrics |
| `route` | `uint16[]` | Multi-hop path | Empty if direct |
| `status` | `enum` | `UP`, `DEGRADED`, `DOWN`, `STORE_FORWARD` | Link state |
| `last_heard_us` | `uint64` | Monotonic | Freshness |

**LinkQoS** (struct):
```c
struct LinkQoS {
    float latency_ms;           // p50 / p99
    float packet_loss_pct;      // 0.0–1.0
    float bandwidth_mbps;       // Measured throughput
    float rssi_dbm;             // Signal strength
    float snr_db;               // Signal-to-noise
    uint32 tx_bytes;            // Counters
    uint32 rx_bytes;
    uint32 tx_errors;
    uint32 rx_errors;
}
```

**Validation Rules**:
- `packet_loss_pct > 0.30` → `status = STORE_FORWARD` (FR-019, SC-013)
- `latency_ms > 200` → consensus timeout risk (research decision)
- Mesh consensus requires `≥ 3` disjoint paths for DAL A messages (FR-011, 012, 013)
- Route discovery: BATMAN-adv (Wi-Fi/5G), RPL (LoRa)

---

## Cross-Entity Relationships

```
Mission (1) ──────< (N) Task
    │                    │
    │                    └──< assigned_drone (1) DroneAgent
    │
    └──< assigned_swarm (N) DroneAgent
                            │
                            ├── role: LEADER → Formation (1) ──────< follower_ids (N) DroneAgent
                            │
                            ├── state ←────────────── AgentState (1:1)
                            │
                            ├── capabilities (1:1) Capability
                            │
                            ├── task_assignment (1:1) Task
                            │
                            ├── swarm_map ←──────── SwarmMap (contributors)
                            │
                            ├── ai_inference ──────> AIInferenceResult (N)
                            │
                            └── comm_links ────────> CommunicationLink (N×N)
```

---

## State Machine Summary

| Entity | States | Key Transitions |
|--------|--------|-----------------|
| DroneAgent | IDLE, ARMING, ARMED, TAKEOFF, MISSION_ACTIVE, RTL, LANDING, LANDED, DISARMED, EMERGENCY, EMERGENCY_LAND | Auto transitions on battery, geofence, estop, fault |
| Formation | FORMING, HOLDING, RECONFIGURING, DISSOLVING | Leader fail → bully election; join/leave → slot reassign |
| Mission | PLANNED, UPLOADED, ACTIVE, PAUSED, COMPLETED, FAILED, ABORTED | Task completion → progress; critical fault → FAILED |
| Task | PENDING, ALLOCATED, EXECUTING, COMPLETED, FAILED, REEVALUATING | Drone loss → REEVALUATING → ALLOCATED (CBBA) |
| CommunicationLink | UP, DEGRADED, DOWN, STORE_FORWARD | Loss threshold → DEGRADED → STORE_FORWARD; recovery → UP |

---

## Constitution Alignment

| Principle | Data Model Enforcement |
|-----------|------------------------|
| I. Safety-First | `AgentState.health`, `MissionConstraints.geofence`, `FaultMeta.severity`, `CommunicationLink` redundancy |
| II. Real-Time Determinism | Fixed-size structs, no dynamic allocation in `AgentState`, `FormationTolerances` bounds |
| III. Hardware-Abstracted | `Capability` bitmap, `TaskGeometry` variant, `LinkQoS` per link type |
| IV. Simulation-First | All entities have SITL-compatible serialization (MAVLink/ROS 2/uORB) |
| V. Observability | Every entity has `timestamp_us`, `version`, traceable `source_drone` |
| VI. Versioning | Semantic versioning on `model_version`, `Mission.id` UUID, `SwarmMap.version` vector clock |

---

## Serialization Contracts

| Entity | MAVLink | ROS 2 | uORB |
|--------|---------|-------|------|
| DroneAgent | `HEARTBEAT` + `SYS_STATUS` + custom `SWARM_AGENT_STATE` | `swarm_msgs/msg/AgentState` | `swarm_agent_state` |
| Formation | `SWARM_FORMATION_SET/STATE` | `swarm_msgs/msg/FormationState` | `swarm_formation_state` |
| Mission | `MISSION_ITEM` + custom `SWARM_MISSION` | `swarm_msgs/msg/Mission` | `swarm_mission` |
| Task | Custom `SWARM_TASK_ALLOCATION` | `swarm_msgs/msg/TaskAllocation` | `swarm_task_allocation` |
| SwarmMap | Custom `SWARM_MAP_TILE` (tiled) | `swarm_msgs/msg/SwarmMap` | N/A (companion only) |
| AIInferenceResult | `SWARM_AI_INFERENCE` | `swarm_msgs/msg/AIInference` | `swarm_ai_inference` |
| CommunicationLink | `SWARM_COMM_QOS` | `swarm_msgs/msg/CommQoS` | `swarm_comm_qos` |

*All contracts defined in `contracts/` directory with protobuf/IDL/msg definitions.*