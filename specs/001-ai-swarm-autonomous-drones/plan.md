# Implementation Plan: AI Swarm Autonomous Drone System

**Branch**: `001-ai-swarm-autonomous-drones` | **Date**: 2025-11-19 | **Spec**: [spec.md](spec.md)

**Input**: Feature specification from `/specs/001-ai-swarm-autonomous-drones/spec.md`

## Summary

AI 기반 군집형 자율비행 드론 시스템: 리더-팔로워 편대 비행 + 다중 에이전트 협업 임무 수행(지역 탐색, 경계 감시, 분산 센싱)을 위한 완전 자율 스웜 시스템. 온보드 AI(CNN/Transformer/RL-MPC)로 장애물 회피, 착륙지 평가, 이상 탐지, 적응형 재계획 수행. PX4/ArduPilot FC + Jetson Orin/Snapdragon Flight 컴패니언 컴퓨터 아키텍처. MAVLink/ROS 2/uORB 계약 기반 모듈식 설계. 시뮬레이션 퍼스트(Gazebo SITL → HIL → 필드)로 DO-178C/SC-VTOL 인증 추적성 확보.

## Technical Context

**Language/Version**: 
- FC Firmware: C/C++17 (PX4: NuttX/STM32H7, ArduPilot: ChibiOS/STM32H7)
- Companion Computer: Python 3.11 (ROS 2 Humble), C++20 (real-time nodes), CUDA 12.2 (TensorRT)
- Ground Station/Tools: Python 3.11, TypeScript/React (web dashboard)

**Primary Dependencies**:
- Middleware: ROS 2 Humble (Fast DDS rmw), MAVLink 2.0, uORB (PX4)
- AI/ML: TensorRT 10, ONNX Runtime 1.18, PyTorch 2.3 (training), OpenCV 4.10, Eigen 3.4
- Simulation: Gazebo Harmonic + PX4 SITL, AirSim (Unreal Engine 5), Isaac Sim 4.2
- Communication: Fast DDS (ROS 2), libsodium (crypto), wpa_supplicant (Wi-Fi 6E mesh), ModemManager (5G NR)
- Build: CMake 3.28, colcon, Bazel (AI pipelines), Docker/Podman (CI containers)
- Testing: gtest, pytest, Unity (C), ROS 2 launch testing, mavlink_router

**Storage**: 
- FC: FRAM/EEPROM (params), SD card (ulog)
- Companion: NVMe (rosbag2, AI models, maps), Redis (shared state cache)
- Ground: PostgreSQL (mission DB, flight logs), MinIO (artifact storage)

**Testing**: 
- Unit: gtest (C++), pytest (Python), Unity (C firmware)
- Integration: ROS 2 launch testing, MAVLink mock, contract tests
- SITL: PX4 SITL + Gazebo Harmonic, custom scenario runner (YAML/DSL)
- HIL: Pixhawk 6C / Cube Orange+ + Gazebo bridge (MAVLink over UDP/serial)
- Field: Tethered → free flight progression, automated log analysis (Python/ulog)

**Target Platform**: 
- Flight Controller: Pixhawk 6C (STM32H7), Cube Orange+ (STM32H7), CUAV Nora (STM32H7)
- Companion Computer: NVIDIA Jetson Orin NX 16GB (primary), Qualcomm Snapdragon Flight RB5 (alternative)
- Sensors: IMU (ICM-42688-P dual), GPS (u-blox ZED-F9P dual RTK), Baro (MS5611), Mag (IST8310), Camera (IMX477 global shutter stereo + downward), LiDAR (Velabit/OS0-32 optional), UWB (DWM3000)
- Communication: Wi-Fi 6E (Intel AX210/AX211), 5G NR (Quectel RM520N-GL), LoRa (SX1262 mesh)

**Project Type**: Embedded real-time system + distributed autonomy stack (firmware + companion + ground)

**Performance Goals**:
- Control loop: Attitude 4 kHz, Position 400 Hz, Velocity 200 Hz (WCET < 80% period)
- AI inference: < 50 ms/model (TensorRT FP16 on Orin), end-to-end perception-to-action < 100 ms
- Formation hold: ≤ 0.5 m lateral, ≤ 0.3 m vertical (wind < 5 m/s)
- Mesh consensus: < 200 ms propagation (10 drones, 2 hops)
- Task reallocation: < 5 s (CBBA convergence)
- SITL regression: < 15 min (full mission suite)

**Constraints**:
- Hard real-time: No heap allocation in control loops, priority inheritance (PI) mutex
- Safety: DO-178C Level A traceability for FC critical paths; ISO 21448 (SOTIF) for autonomy
- Security: Signed firmware (ED25519), encrypted MAVLink (TLS 1.3/DTLS 1.3), A/B OTA
- Regulatory: Remote ID (ASTM F3411), Geo-fence (firmware), EASA SC-VTOL / FAA Part 135
- Power: 20–60 min sortie (6S 22 Ah Li-ion), hot-swap support
- SWaP: Companion < 15 W, FC < 5 W, total drone < 2 kg (quad) / 5 kg (VTOL)

**Scale/Scope**:
- Swarm size: 2–50 drones (typical 5–10)
- Mission area: Up to 10 km² (search), 5 km radius (waypoint)
- Codebase: ~150k LOC (FC: 80k, Companion: 50k, Ground: 20k)
- Interfaces: 30+ MAVLink messages, 50+ ROS 2 topics/services/actions, 15+ uORB topics

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Evidence / Justification |
|-----------|--------|--------------------------|
| I. Safety-First Architecture | ✅ PASS | Redundant sensors (dual IMU/GPS), fail-safe modes (RTL, land, terminate), verified safe-state transitions in plan |
| II. Real-Time Determinism | ✅ PASS | WCET analysis planned, no heap in control loops, PI mutex, rate-monotonic scheduling |
| III. Hardware-Abstracted Modularity | ✅ PASS | HAL layer, MAVLink/ROS 2/uORB contracts, portable across Pixhawk/Cube/Jetson/Snapdragon |
| IV. Simulation-Before-Hardware | ✅ PASS | CI pipeline: unit → integration → SITL → HIL → field; no untested code flies |
| V. Observability & Traceability | ✅ PASS | ulog + rosbag2 structured logging, telemetry streaming, automated post-flight analysis |
| VI. Versioning & Airworthiness | ✅ PASS | Semantic versioning + airworthy tags, config as code, breaking changes need regression + board approval |
| Regulatory: Geo-fence (firmware) | ✅ PASS | FR-011 mandates firmware-level hard limits |
| Regulatory: Remote ID | ✅ PASS | FR-012 mandates ASTM F3411/EU DRI broadcast |
| Regulatory: Certification trace | ✅ PASS | DO-178C/DO-254 traceability in safety artifacts |
| Security: Signed firmware, encrypted links, A/B OTA | ✅ PASS | FR-013/015/019 + constitution security requirements |
| Development: Branching, PR gates, testing pyramid | ✅ PASS | Matches constitution workflow exactly |
| Governance: Amendment process | ✅ PASS | Documented in constitution |

**All gates PASS. Proceed to Phase 0.**

## Project Structure

### Documentation (this feature)

```text
specs/001-ai-swarm-autonomous-drones/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   ├── mavlink/
│   │   ├── swarm_formation.proto
│   │   ├── task_allocation.proto
│   │   ├── ai_inference.proto
│   │   └── safety_compliance.proto
│   ├── ros2/
│   │   ├── msg/
│   │   ├── srv/
│   │   └── action/
│   └── uorb/
│       ├── swarm_formation.idl
│       ├── task_allocation.idl
│       └── ai_inference.idl
└── tasks.md             # Phase 2 output (/speckit-tasks command)
```

### Source Code (repository root)

```text
# Drone Swarm Autonomy Stack
src/
├── firmware/                    # FC firmware (PX4/ArduPilot modules)
│   ├── modules/
│   │   ├── swarm_formation/     # Formation control (leader/follower)
│   │   ├── swarm_safety/        # Geo-fence, RID, emergency stop
│   │   ├── swarm_comm/          # MAVLink mesh routing
│   │   └── hal/                 # Hardware abstraction layer
│   └── platforms/
│       ├── pixhawk6c/
│       ├── cubeorange/
│       └── cuav_nora/
├── companion/                   # Companion computer stack (ROS 2)
│   ├── swarm_autonomy/          # Core autonomy nodes
│   │   ├── formation_control/   # Follower position control
│   │   ├── task_allocator/      # CBBA/market-based allocation
│   │   ├── trajectory_planner/  # RRT*/MPC/RL planner
│   │   ├── collision_avoidance/ # ORCA/VO/MPC distributed
│   │   └── state_estimator/     # VIO + UWB fusion
│   ├── ai_perception/           # AI inference pipeline
│   │   ├── obstacle_detector/   # CNN/Transformer detection
│   │   ├── landing_assessor/    # Site classification
│   │   ├── fault_detector/      # Anomaly detection
│   │   └── model_runtime/       # TensorRT/ONNX execution
│   ├── swarm_comm/              # Mesh networking
│   │   ├── mesh_manager/        # Wi-Fi 6E/5G/LoRa management
│   │   ├── consensus/           # State synchronization
│   │   └── store_forward/       # Delay-tolerant messaging
│   ├── swarm_safety/            # Safety monitors
│   │   ├── geofence_monitor/    # Independent RTL trigger
│   │   ├── rid_broadcaster/     # Remote ID compliance
│   │   └── emergency_stop/      # Kill switch handler
│   └── swarm_interface/         # Human-swarm interaction
│       ├── intent_parser/       # Voice/tablet NLP
│       ├── mission_uploader/    # Waypoint/polygon upload
│       └── telemetry_streamer/  # Real-time monitoring
├── ground/                      # Ground station & cloud
│   ├── gcs/                     # Ground control station (QGroundControl fork + custom)
│   │   ├── swarm_mission_planner/
│   │   ├── swarm_monitor/
│   │   └── post_flight_analyzer/
│   ├── cloud/                   # Cloud backend (optional)
│   │   ├── mission_db/
│   │   ├── model_registry/
│   │   └── fleet_manager/
│   └── dashboard/               # Web dashboard (React/TypeScript)
│       ├── live_view/
│       ├── replay/
│       └── analytics/
├── simulation/                  # Simulation & test infrastructure
│   ├── sitl/                    # PX4 SITL + Gazebo configs
│   ├── hil/                     # HIL bridge configurations
│   ├── scenarios/               # Mission scenarios (YAML/DSL)
│   └── test_runner/             # Automated test orchestration
└── tools/                       # Build, deploy, analysis tools
    ├── ota_builder/             # A/B OTA image builder
    ├── log_analyzer/            # ulog/rosbag2 analysis
    └── config_generator/        # Param/launch file generator

tests/
├── unit/
│   ├── firmware/                # gtest, Unity
│   ├── companion/               # pytest, gtest
│   └── ground/                  # pytest, jest
├── integration/
│   ├── contract/                # MAVLink/ROS 2/uORB contract tests
│   ├── sitl/                    # SITL mission tests
│   └── hil/                     # HIL hardware tests
├── simulation/
│   ├── formation/
│   ├── task_allocation/
│   ├── ai_perception/
│   └── safety/
└── field/
    ├── test_cards/              # Flight test cards
    └── automated_analysis/      # Log analysis scripts
```

**Structure Decision**: Monorepo with three top-level domains (firmware, companion, ground) reflecting the three compute tiers. This matches Constitution Principle III (Hardware-Abstracted Modularity) and enables independent CI pipelines per tier while sharing contracts via `contracts/`.

## Complexity Tracking

> No constitution violations requiring justification at this stage. All design choices align with principles.

---

## Phase 0: Research & Clarification

### Technical Context Unknowns → Research Tasks

| Unknown | Research Task | Priority |
|---------|---------------|----------|
| CBBA vs Market-based task allocation for 50-drone scale | Compare convergence time, communication overhead, optimality gap in swarm sim | P1 |
| TensorRT vs ONNX Runtime on Jetson Orin vs Snapdragon Flight | Benchmark inference latency, memory, model compatibility for detection/classification/fault | P1 |
| Wi-Fi 6E mesh vs 5G NR vs LoRa for swarm consensus | Measure latency, packet loss, range, power at 10/20/50 drones | P1 |
| VIO + UWB fusion algorithm (EKF vs Factor Graph) | Evaluate drift, compute load, outlier rejection in GPS-denied | P1 |
| DO-178C Level A certification path for FC swarm modules | Map FR-011/012/013 to DAL A objectives, identify verification gaps | P2 |
| ROS 2 Fast DDS tuning for 50-node mesh | Discovery, reliability QoS, bandwidth optimization | P2 |
| A/B OTA partition scheme for FC + Companion dual image | Rollback criteria, health checks, secure boot integration | P2 |
| Human intent NLP: on-device vs cloud | Latency, privacy, accuracy trade-off for "search area", "follow me" | P3 |

### Research Output: `research.md`

Each research task produces a decision entry:

```markdown
## Decision: [Topic]
**Chosen**: [Technology/Approach]
**Rationale**: [Why, with data]
**Alternatives Considered**: [List with pros/cons]
**Impact**: [Requirements affected, migration effort]
```

All NEEDS CLARIFICATION resolved before Phase 1.

---

## Phase 1: Design & Contracts

### 1. Data Model (`data-model.md`)

Extract entities from spec → formal data model:

| Entity | Key Fields | Relationships | Validation Rules |
|--------|------------|---------------|------------------|
| DroneAgent | id, type, capabilities[], state(pose, vel, battery, mode), role | belongs to Formation, executes Tasks | battery > 20% for mission; pose covariance < threshold |
| Formation | id, geometry(template), tolerances, leader_id, follower_ids[] | has many DroneAgents | follower count 1–49; geometry valid in 3D |
| Mission | id, type(waypoint/search/inspect), constraints, priority, status | decomposes to Tasks | geofence compliant; time/energy budget feasible |
| Task | id, type(cover/inspect/relay/hold), polygon/POI, deps[], assigned_drone | assigned to DroneAgent, part of Mission | no circular deps; coverage non-overlapping |
| SwarmMap | id, occupancy_grid, semantic_labels, no_fly_zones, landmarks | fused from all DroneAgents | update rate ≥ 1 Hz; consistency check |
| AIInferenceResult | timestamp, drone_id, type(obstacle/landing/fault), class, confidence, 3D_pos | generated by DroneAgent | confidence > 0.7 for action; latency < 50 ms |
| CommunicationLink | src_id, dst_id, type(wifi/5g/lora), qos(latency, loss, bw), route[] | mesh topology | loss < 30% for consensus; store-forward buffer |

State machines for: DroneAgent (idle→arming→takeoff→mission→rtl→land), Formation (forming→holding→reconfiguring→dissolving), Task (pending→allocated→executing→completed/failed).

### 2. Interface Contracts (`contracts/`)

#### MAVLink Messages (`contracts/mavlink/*.proto`)
- `SWARM_FORMATION_SET`: Leader → Followers (geometry, offsets, gains)
- `SWARM_FORMATION_STATE`: Follower → Leader (position error, status)
- `TASK_ALLOCATION_BID`: Drone → Swarm (task_id, cost, capability)
- `TASK_ALLOCATION_AWARD`: Swarm → Drone (task_id, assignment)
- `AI_INFERENCE_REPORT`: Drone → Swarm (type, class, confidence, position)
- `SAFETY_EVENT`: Any → All (geofence_breach, emergency_stop, rid_status)
- `HUMAN_INTENT_CMD`: GCS → Swarm (intent_type, parameters, priority)

#### ROS 2 Interfaces (`contracts/ros2/`)
- **Messages**: `FormationState.msg`, `TaskAllocation.msg`, `AIInference.msg`, `SwarmMap.msg`, `CommQoS.msg`
- **Services**: `UploadMission.srv`, `RequestRTL.srv`, `EmergencyStop.srv`, `GetSwarmStatus.srv`
- **Actions**: `ExecuteMission.action`, `SearchArea.action`, `InspectTarget.action`, `FollowMe.action`

#### uORB Topics (`contracts/uorb/*.idl`)
- `swarm_formation_set`, `swarm_formation_state`, `task_allocation_bid`, `task_allocation_award`, `ai_inference_report`, `safety_event`, `human_intent_cmd`

ICD auto-generated from `.proto/.msg/.idl` → Markdown/HTML for review.

### 3. Quickstart Validation Guide (`quickstart.md`)

Runnable validation scenarios proving end-to-end feature:

```markdown
# Quickstart: AI Swarm Autonomous Drone System

## Prerequisites
- Hardware: 3× Pixhawk 6C + Jetson Orin NX (1 leader, 2 followers)
- Software: Docker, PX4 SITL, Gazebo Harmonic, ROS 2 Humble
- Network: Wi-Fi 6E AP (or 5G SIMs)

## Setup
```bash
# 1. Clone & build
git clone <repo> && cd drone-swarm
./tools/build_all.sh --target=sitl

# 2. Start SITL swarm (3 drones)
./simulation/sitl/launch_swarm.sh --count=3 --formation=vee

# 3. Run formation mission
ros2 run swarm_autonomy mission_uploader --mission specs/001/scenarios/formation_10wp.yaml

# 4. Verify
# - Formation error < 0.5 m (ros2 topic echo /swarm/formation_state)
# - All waypoints reached (check ulog)
# - Landing sequenced
```

## Validation Scenarios
| Scenario | Command | Expected Outcome |
|----------|---------|------------------|
| Formation hold | `mission_uploader --formation=vee --waypoints=10` | Position error < 0.5 m |
| Leader obstacle replan | Inject Gazebo obstacle at WP 5 | Followers adjust < 2 s |
| Task reallocation | Kill follower at 50% search | Remaining cover gap < 2% |
| AI obstacle avoid | Spawn power line at 25 m | Avoidance < 200 ms |
| Emergency stop | `ros2 service call /swarm/emergency_stop` | All motors off < 1 s |

## CI Integration
```yaml
# .github/workflows/sitl_regression.yml
- name: SITL Formation Test
  run: ./simulation/test_runner.py --suite=formation --drones=5
- name: SITL Search Test
  run: ./simulation/test_runner.py --suite=search --area=1km2
```

---

## Completion Report

**Branch**: `001-ai-swarm-autonomous-drones`
**Plan Path**: `/my-drone-project/specs/001-ai-swarm-autonomous-drones/plan.md`
**Generated Artifacts** (Phase 1):
- `research.md` — All technical decisions documented
- `data-model.md` — 7 entities, state machines, validation rules
- `contracts/mavlink/` — 7 message definitions (protobuf)
- `contracts/ros2/` — 12 interfaces (msg/srv/action)
- `contracts/uorb/` — 7 topic definitions (IDL)
- `quickstart.md` — 5 validation scenarios, CI integration

**Readiness**: Ready for `/speckit-tasks` → `/speckit-implement`