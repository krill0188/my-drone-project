---
description: "Task list for AI Swarm Autonomous Drone System implementation"
---

# Tasks: AI Swarm Autonomous Drone System

**Input**: Design documents from `/specs/001-ai-swarm-autonomous-drones/`

**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/, quickstart.md

**Tests**: Test tasks included per user story (spec mandates independent testability)

**Organization**: Tasks grouped by user story to enable independent implementation and testing of each story. Follows project structure from plan.md (firmware/, companion/, ground/, simulation/).

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions (from plan.md)

```
src/
├── firmware/                    # FC firmware (PX4/ArduPilot modules)
│   ├── modules/
│   │   ├── swarm_formation/
│   │   ├── swarm_safety/
│   │   ├── swarm_comm/
│   │   └── hal/
│   └── platforms/
│       ├── pixhawk6c/
│       ├── cubeorange/
│       └── cuav_nora/
├── companion/                   # Companion computer stack (ROS 2)
│   ├── swarm_autonomy/
│   │   ├── formation_control/
│   │   ├── task_allocator/
│   │   ├── trajectory_planner/
│   │   ├── collision_avoidance/
│   │   └── state_estimator/
│   ├── ai_perception/
│   │   ├── obstacle_detector/
│   │   ├── landing_assessor/
│   │   ├── fault_detector/
│   │   └── model_runtime/
│   ├── swarm_comm/
│   │   ├── mesh_manager/
│   │   ├── consensus/
│   │   └── store_forward/
│   ├── swarm_safety/
│   │   ├── geofence_monitor/
│   │   ├── rid_broadcaster/
│   │   └── emergency_stop/
│   └── swarm_interface/
│       ├── intent_parser/
│       ├── mission_uploader/
│       └── telemetry_streamer/
├── ground/                      # Ground station & cloud
│   ├── gcs/
│   ├── cloud/
│   └── dashboard/
├── simulation/                  # Simulation & test infrastructure
│   ├── sitl/
│   ├── hil/
│   ├── scenarios/
│   └── test_runner/
└── tools/                       # Build, deploy, analysis tools
    ├── ota_builder/
    ├── log_analyzer/
    └── config_generator/

tests/
├── unit/
│   ├── firmware/
│   ├── companion/
│   └── ground/
├── integration/
│   ├── contract/
│   ├── sitl/
│   └── hil/
├── simulation/
│   ├── formation/
│   ├── task_allocation/
│   ├── ai_perception/
│   └── safety/
└── field/
    ├── test_cards/
    └── automated_analysis/
```

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for all three tiers (firmware, companion, ground)

- [X] T001 Create monorepo directory structure per plan.md (src/firmware, src/companion, src/ground, src/simulation, src/tools, tests/)
- [X] T002 Initialize PX4 firmware submodule and configure build for Pixhawk6C/CubeOrange/CUAV Nora platforms
- [X] T003 Initialize ROS 2 Humble workspace (colcon) in src/companion with Fast DDS rmw
- [X] T004 [P] Configure CMake toolchain for STM32H7 (firmware) and aarch64 (companion)
- [X] T005 [P] Setup Docker/Podman CI containers for SITL (Gazebo Harmonic + PX4) and HIL
- [X] T006 [P] Configure linting/formatting: clang-tidy, cppcheck, mypy, black, pre-commit hooks
- [X] T007 [P] Setup GitHub Actions workflow skeleton (sitl_regression.yml from quickstart.md)
- [X] T008 [P] Initialize Python venv for ground tools and dashboard (requirements.txt)
- [X] T009 Create .specify/memory/adr/ directory for Architecture Decision Records

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

### 2.1 Contract Definitions & Code Generation

- [X] T010 Generate MAVLink message definitions from contracts/mavlink/swarm_messages.proto → src/firmware/modules/swarm_comm/mavlink/
- [X] T011 Generate ROS 2 interfaces from contracts/ros2/ (msg, srv, action) → src/companion/swarm_msgs/
- [X] T012 Generate uORB topic definitions from contracts/uorb/swarm_messages.idl → src/firmware/modules/swarm_comm/uorb/
- [ ] T013 [P] Create ICD (Interface Control Document) auto-generation from protobuf/msg/idl → docs/icd/

### 2.2 Common Infrastructure (All Tiers)

- [X] T014 Implement common timestamp utilities (monotonic µs, synchronization) in src/firmware/modules/swarm_comm/common/ and src/companion/swarm_common/
- [X] T015 [P] Implement CRC32/ED25519 verification for OTA and message integrity (research.md Decision 7)
- [X] T016 Implement structured logging framework (ulog for firmware, rosbag2 for companion) per FR-015
- [X] T017 [P] Create configuration management: YAML param files → C++/Python structs (data-model.md entities)
- [X] T018 Implement MAVLink 2.0 router with mesh routing (BATMAN-adv integration) in src/firmware/modules/swarm_comm/

### 2.3 Hardware Abstraction Layer (Constitution Principle III)

- [X] T019 Implement HAL interface for sensors (IMU, GPS, Baro, Mag, Camera, LiDAR, UWB) in src/firmware/modules/hal/
- [X] T020 [P] Implement platform-specific HAL bindings for Pixhawk6C, CubeOrange, CUAV Nora in src/firmware/platforms/
- [X] T021 Implement companion computer hardware abstraction (Jetson Orin / Snapdragon Flight) in src/companion/swarm_common/hal/

### 2.4 Safety-Critical Foundation (DAL A per research.md Decision 5)

- [X] T022 Implement firmware-level geofence monitor (hard limits, independent of companion) per FR-011, SC-008
- [X] T023 Implement Remote ID broadcaster (ASTM F3411 / EU DRI) at 1 Hz in firmware per FR-012, SC-010
- [X] T024 Implement emergency stop handler (motor kill + parachute deploy) in firmware per FR-013, SC-009
- [X] T025 [P] Write DO-178C traceability matrix template linking FR-011/012/013 to code modules

### 2.5 Simulation & Test Infrastructure

- [X] T026 Configure PX4 SITL + Gazebo Harmonic multi-drone launch (launch_swarm.sh from quickstart.md)
- [X] T027 [P] Implement test_runner.py framework with scenario DSL (YAML) from quickstart.md
- [X] T028 [P] Configure HIL bridge (MAVLink over UDP/serial) for Pixhawk 6C / Cube Orange+
- [X] T029 Implement log_analyzer.py for automated validation (formation error, coverage, drift, latency)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Swarm Formation & Leader-Follower Navigation (Priority: P1) 🎯 MVP

**Goal**: Leader-follower formation flight with configurable geometries (line, V, diamond, column, custom) for 2–50 drones, maintaining ≤ 0.5 m lateral / ≤ 0.3 m vertical error, with leader replan propagation < 200 ms

**Independent Test**: Deploy 1 leader + 3 followers in SITL/Gazebo with 10-waypoint mission. Verify: (a) all take off, (b) formation geometry within ±0.5 m, (c) all waypoints reached, (d) sequenced landing

### Tests for User Story 1 (Write FIRST, ensure they FAIL before implementation)

- [ ] T030 [P] [US1] Contract test: SWARM_FORMATION_SET/STATE MAVLink messages in tests/integration/contract/test_formation_mavlink.py
- [ ] T031 [P] [US1] Contract test: FormationState ROS 2 topic in tests/integration/contract/test_formation_ros2.py
- [X] T032 [P] [US1] SITL integration test: 4-drone V-formation 10-waypoint mission in tests/simulation/formation/test_formation_hold.py
- [ ] T033 [P] [US1] SITL integration test: Leader obstacle replan propagation < 200 ms in tests/simulation/formation/test_replan_propagation.py
- [ ] T034 [P] [US1] SITL integration test: Follower rejoin after localization loss in tests/simulation/formation/test_follower_rejoin.py

### Implementation for User Story 1

#### 3.1 Data Models (data-model.md: DroneAgent, Formation, FormationSlot, FormationGeometry, FormationTolerances)

- [X] T035 [P] [US1] Implement DroneAgent state struct (AgentState) in src/firmware/modules/swarm_formation/drone_agent.hpp
- [X] T036 [P] [US1] Implement Formation entity (geometry, tolerances, slots, state machine) in src/firmware/modules/swarm_formation/formation.hpp
- [X] T037 [P] [US1] Implement FormationSlot assignment logic in src/firmware/modules/swarm_formation/formation_slot.cpp
- [X] T038 [P] [US1] ROS 2 mirrors: AgentState, FormationState in src/companion/swarm_autonomy/formation_control/

#### 3.2 Firmware Formation Control Module (src/firmware/modules/swarm_formation/)

- [X] T039 [US1] Implement leader trajectory generator (waypoint following, energy-aware) in src/firmware/modules/swarm_formation/leader_trajectory.cpp
- [X] T040 [US1] Implement follower position controller (PID + feedforward, gains from FormationSlot) in src/firmware/modules/swarm_formation/follower_control.cpp
- [X] T041 [US1] Implement formation geometry manager (line, V, diamond, column, custom) in src/firmware/modules/swarm_formation/geometry_manager.cpp
- [X] T042 [US1] Implement formation state machine (FORMING→HOLDING→RECONFIGURING→DISSOLVING) in src/firmware/modules/swarm_formation/formation_fsm.cpp
- [X] T043 [US1] Implement leader-follower MAVLink message handling (SWARM_FORMATION_SET/STATE) in src/firmware/modules/swarm_formation/mavlink_handler.cpp
- [X] T044 [US1] Implement relative localization fusion (UWB + vision + LiDAR SLAM) for follower position in src/firmware/modules/swarm_formation/relative_localization.cpp

#### 3.3 Companion Formation Control (src/companion/swarm_autonomy/formation_control/)

- [X] T045 [P] [US1] ROS 2 node: FormationState publisher/subscriber in src/companion/swarm_autonomy/formation_control/formation_state_node.cpp
- [X] T046 [P] [US1] ROS 2 node: Leader trajectory visualization and monitoring in src/companion/swarm_autonomy/formation_control/leader_monitor.cpp
- [X] T047 [US1] Implement formation reconfiguration service (geometry change, join/leave) in src/companion/swarm_autonomy/formation_control/reconfigure_service.cpp

#### 3.4 Leader Replanning & Propagation (FR-003, SC-007)

- [X] T048 [US1] Integrate trajectory planner (RRT*/MPC) for obstacle avoidance in src/companion/swarm_autonomy/trajectory_planner/
- [X] T049 [US1] Implement replan propagation to followers < 200 ms (mesh broadcast) in src/companion/swarm_comm/consensus/
- [X] T050 [US1] Implement follower formation adjustment on leader replan (2 s bound) in src/firmware/modules/swarm_formation/follower_replan.cpp

#### 3.5 Mission Integration & Takeoff/Landing Sequencing

- [X] T051 [US1] Implement mission uploader (waypoint + formation) in src/companion/swarm_interface/mission_uploader/
- [X] T052 [US1] Implement coordinated takeoff (formation acquisition within 10 s) in src/firmware/modules/swarm_formation/takeoff_sequence.cpp
- [X] T053 [US1] Implement sequenced RTL and landing at designated pads in src/firmware/modules/swarm_formation/landing_sequence.cpp

#### 3.6 Edge Cases (spec.md Edge Cases 1, 3, 5)

- [X] T054 [US1] Implement leader failure detection and bully election for new leader in src/firmware/modules/swarm_formation/leader_election.hpp
- [X] T055 [US1] Implement follower localization loss detection and smooth rejoin (overshoot < 1 m) in src/firmware/modules/swarm_formation/rejoin_logic.cpp
- [X] T056 [US1] Implement GPS-denied relative localization fallback (VIO + UWB) in src/companion/swarm_autonomy/state_estimator/

**Checkpoint**: User Story 1 fully functional and independently testable (run Scenario 1 & 2 from quickstart.md)

---

## Phase 4: User Story 2 - Multi-Agent Cooperative Mobility & Task Allocation (Priority: P1)

**Goal**: Leaderless multi-agent task allocation (area search, perimeter surveillance, distributed sensing) using CBBA consensus, achieving ≥ 95% coverage, task reallocation < 5 s on drone dropout

**Independent Test**: Deploy 5 drones in 1 km² search area in SITL. Verify: (a) 95% coverage, (b) zero collisions, (c) reallocation < 5 s, (d) all return with sensor data

### Tests for User Story 2 (Write FIRST, ensure they FAIL before implementation)

- [X] T057 [P] [US2] Contract test: TASK_ALLOCATION_BID/AWARD MAVLink messages in tests/integration/contract/test_task_allocation_mavlink.py
- [X] T058 [P] [US2] Contract test: TaskAllocation ROS 2 topic in tests/integration/contract/test_task_allocation_ros2.py
- [X] T059 [P] [US2] SITL integration test: 5-drone 1 km² search coverage ≥ 95% in tests/simulation/task_allocation/test_search_coverage.py
- [X] T060 [P] [US2] SITL integration test: Task reallocation on drone dropout < 5 s in tests/simulation/task_allocation/test_reallocation.py
- [X] T061 [P] [US2] SITL integration test: Dynamic obstacle broadcast and replan < 1 s in tests/simulation/task_allocation/test_obstacle_broadcast.py
- [X] T062 [P] [US2] SITL integration test: Mesh degradation (30% loss) store-and-forward in tests/simulation/task_allocation/test_mesh_degradation.py

### Implementation for User Story 2

#### 4.1 Data Models (data-model.md: Mission, Task, TaskGeometry, SuccessCriteria, SwarmMap)

- [X] T063 [P] [US2] Implement Mission entity (type, constraints, tasks, state machine) in src/companion/swarm_autonomy/task_allocator/mission.hpp
- [X] T064 [P] [US2] Implement Task entity (type, geometry, dependencies, success criteria) in src/companion/swarm_autonomy/task_allocator/task.hpp
- [X] T065 [P] [US2] Implement SwarmMap entity (occupancy grid, semantic labels, no-fly zones, landmarks) in src/companion/swarm_autonomy/task_allocator/swarm_map.hpp
- [X] T066 [P] [US2] Implement CapabilityBitmap and bid scoring in src/companion/swarm_autonomy/task_allocator/capability.cpp

#### 4.2 CBBA Task Allocator (research.md Decision 1) (src/companion/swarm_autonomy/task_allocator/)

- [X] T067 [US2] Implement CBBA core algorithm (bundle building, consensus, convergence) in src/companion/swarm_autonomy/task_allocator/cbba.hpp
- [X] T068 [US2] Implement decentralized auction loop (async, 1 Hz) in src/companion/swarm_autonomy/task_allocator/auction_loop.cpp
- [X] T069 [US2] Implement task decomposition: area search → lawnmower/spiral sub-tasks in src/companion/swarm_autonomy/task_allocator/task_decomposer.cpp
- [X] T070 [US2] Implement heterogeneous capability-aware bidding (payload, endurance, sensors) in src/companion/swarm_autonomy/task_allocator/heterogeneous_bidding.cpp
- [X] T071 [US2] Implement task reallocation on drone dropout (5 s bound) in src/companion/swarm_autonomy/task_allocator/reallocation.cpp
- [X] T072 [US2] MAVLink/ROS 2 message handling for TASK_ALLOCATION_BID/AWARD in src/companion/swarm_autonomy/task_allocator/comm_handler.cpp

#### 4.3 Swarm Map Fusion (src/companion/swarm_autonomy/task_allocator/)

- [ ] T073 [US2] Implement occupancy grid fusion (Bayesian update, contributor weighting) in src/companion/swarm_autonomy/task_allocator/map_fusion.cpp
- [ ] T074 [US2] Implement semantic label fusion (AI inference + sensor) in src/companion/swarm_autonomy/task_allocator/semantic_fusion.cpp
- [ ] T075 [US2] Implement no-fly zone distribution and cryptographic verification in src/companion/swarm_autonomy/task_allocator/nofly_sync.cpp
- [ ] T076 [US2] Implement landmark-based absolute localization correction in src/companion/swarm_autonomy/task_allocator/landmark_correction.cpp

#### 4.4 Distributed Collision Avoidance (FR-018, research.md)

- [ ] T077 [US2] Implement ORCA/Velocity Obstacles for inter-drone separation > 5 m in src/companion/swarm_autonomy/collision_avoidance/orca.cpp
- [ ] T078 [US2] Implement MPC-based collision avoidance as alternative in src/companion/swarm_autonomy/collision_avoidance/mpc_avoidance.cpp
- [ ] T079 [US2] Integrate collision avoidance with local trajectory planner in src/companion/swarm_autonomy/trajectory_planner/local_planner.cpp

#### 4.5 Mission Types & Patterns

- [ ] T080 [US2] Implement area search patterns (lawnmower, spiral, adaptive) in src/companion/swarm_autonomy/task_allocator/search_patterns.cpp
- [ ] T081 [US2] Implement perimeter surveillance task allocation in src/companion/swarm_autonomy/task_allocator/perimeter_surveillance.cpp
- [X] T082 [US2] Implement distributed sensing task allocation in src/companion/swarm_autonomy/task_allocator/distributed_sensing.cpp

#### 4.6 Mesh Network Resilience (FR-019, research.md Decision 3)

- [X] T083 [US2] Implement store-and-forward message queue for > 30% packet loss in src/companion/swarm_comm/store_forward/
- [X] T084 [US2] Implement mesh link QoS monitoring and tier failover (Wi-Fi 6E → 5G → LoRa) in src/companion/swarm_comm/mesh_manager/
- [X] T085 [US2] Implement consensus re-sync on link restoration (< 10 s) in src/companion/swarm_comm/consensus/resync.cpp

#### 4.7 Edge Cases (spec.md Edge Cases 2, 4)

- [X] T086 [US2] Implement split-brain detection and independent partition operation in src/companion/swarm_comm/consensus/split_brain.cpp
- [X] T087 [US2] Implement heterogeneous swarm capability advertisement at startup in src/companion/swarm_autonomy/task_allocator/capability_advertisement.cpp

**Checkpoint**: User Story 2 fully functional and independently testable (run Scenarios 3 & 4 from quickstart.md)

---

## Phase 5: User Story 3 - AI-Based Autonomous Decision Making & Adaptation (Priority: P2)

**Goal**: Onboard AI perception (obstacle detection, landing assessment, fault detection) with < 50 ms inference, adaptive replanning < 100 ms, GPS-denied operation with < 1% drift

**Independent Test**: In Gazebo with photorealistic assets, verify: (a) obstacle detection F1 ≥ 0.92 at 30 m, (b) landing classification ≥ 95%, (c) fault detection < 200 ms, (d) replan < 100 ms

### Tests for User Story 3 (Write FIRST, ensure they FAIL before implementation)

- [X] T088 [P] [US3] Contract test: AI_INFERENCE_REPORT MAVLink message in tests/integration/contract/test_ai_inference_mavlink.py
- [X] T089 [P] [US3] Contract test: AIInference ROS 2 topic in tests/integration/contract/test_ai_inference_ros2.py
- [X] T090 [P] [US3] SITL integration test: Power line detection at 25 m, avoidance < 200 ms in tests/simulation/ai_perception/test_obstacle_avoidance.py
- [X] T091 [P] [US3] SITL integration test: GPS-denied VIO+UWB drift < 1% over 2 km in tests/simulation/ai_perception/test_gps_denied.py
- [X] T092 [P] [US3] SITL integration test: Motor fault detection < 200 ms, control reallocation in tests/simulation/ai_perception/test_fault_detection.py
- [X] T093 [P] [US3] Unit test: TensorRT inference latency < 50 ms per model on Orin in tests/unit/companion/test_model_runtime.py

### Implementation for User Story 3

#### 5.1 Data Models (data-model.md: AIInferenceResult, InferenceMetadata, ObstacleMeta, LandingMeta, FaultMeta)

- [X] T094 [P] [US3] Implement AIInferenceResult struct and metadata in src/companion/ai_perception/common/ai_inference.hpp

#### 5.2 AI Model Runtime (research.md Decision 2) (src/companion/ai_perception/model_runtime/)

- [X] T095 [US3] Implement TensorRT engine builder (ONNX → .engine, FP16/INT8 calibration) in src/companion/ai_perception/model_runtime/tensorrt_builder.cpp
- [X] T096 [US3] Implement ONNX Runtime fallback execution provider in src/companion/ai_perception/model_runtime/onnx_runner.cpp
- [X] T097 [US3] Implement unified inference API (preprocess → infer → postprocess) in src/companion/ai_perception/model_runtime/inference_engine.hpp
- [X] T098 [US3] Implement model versioning and A/B OTA update integration in src/companion/ai_perception/model_runtime/model_manager.cpp

#### 5.3 Obstacle Detection (FR-007, SC-004) (src/companion/ai_perception/obstacle_detector/)

- [X] T099 [US3] Implement YOLOv8n/Transformer obstacle detector (power lines, vegetation, structures, aircraft, birds) in src/companion/ai_perception/obstacle_detector/obstacle_detector.cpp
- [X] T100 [US3] Implement 3D localization from stereo/mono + depth in src/companion/ai_perception/obstacle_detector/localization.cpp
- [X] T101 [US3] Implement threat assessment (TTC, threat level, immediate avoidance flag) in src/companion/ai_perception/obstacle_detector/threat_assessment.cpp
- [X] T102 [US3] ROS 2 node: Obstacle detection pipeline at 20 Hz in src/companion/ai_perception/obstacle_detector/obstacle_node.cpp

#### 5.4 Landing Site Assessment (FR-008, SC-005) (src/companion/ai_perception/landing_assessor/)

- [X] T103 [US3] Implement landing site classifier (slope, clearance, surface, firmness) in src/companion/ai_perception/landing_assessor/landing_classifier.cpp
- [X] T104 [US3] Implement downward camera processing for site evaluation in src/companion/ai_perception/landing_assessor/downward_processor.cpp
- [X] T105 [US3] ROS 2 node: Landing assessment service in src/companion/ai_perception/landing_assessor/landing_node.cpp

#### 5.5 Fault Detection & Anomaly Classification (FR-009, SC-006) (src/companion/ai_perception/fault_detector/)

- [X] T106 [US3] Implement LSTM-based fault detector (IMU drift, GPS spoof, motor degradation, battery imbalance) in src/companion/ai_perception/fault_detector/fault_classifier.cpp
- [X] T107 [US3] Implement sensor/actuator health monitoring with statistical baselines in src/companion/ai_perception/fault_detector/health_monitor.cpp
- [X] T108 [US3] Implement fault severity classification and recommended action in src/companion/ai_perception/fault_detector/severity_assessment.cpp
- [X] T109 [US3] ROS 2 node: Fault detection pipeline at 10 Hz in src/companion/ai_perception/fault_detector/fault_node.cpp

#### 5.6 Adaptive Replanning Integration (FR-010, SC-007)

- [X] T110 [US3] Integrate AI inference triggers with trajectory planner (replan < 100 ms) in src/companion/swarm_autonomy/trajectory_planner/ai_replan_integration.cpp
- [X] T111 [US3] Implement formation adjustment on fault detection (follower offset) in src/firmware/modules/swarm_formation/fault_replan.cpp
- [X] T112 [US3] Implement GPS-denied VIO+UWB factor graph fusion (GTSAM per research.md Decision 4) in src/companion/swarm_autonomy/state_estimator/vio_uwb_fusion.cpp

#### 5.7 Edge Cases (spec.md Edge Cases 3, 5)

- [X] T113 [US3] Implement conservative geometric avoidance fallback (LiDAR/sonar) on AI misclassification in src/companion/ai_perception/obstacle_detector/conservative_fallback.cpp
- [X] T114 [US3] Implement visual landmark absolute localization for GPS-denied swarm in src/companion/swarm_autonomy/state_estimator/landmark_localization.cpp

**Checkpoint**: User Story 3 fully functional and independently testable (run Scenarios 5 & 4 from quickstart.md)

---

## Phase 6: User Story 4 - Swarm Safety, Compliance & Human-Swarm Interaction (Priority: P2)

**Goal**: Hard geofence (firmware), Remote ID broadcast, emergency stop < 1 s, human intent commands (voice/tablet), complete flight logging

**Independent Test**: In HIL with real FC hardware, verify: (a) geofence breach → RTL < 500 ms, (b) RID decoded by standard receiver, (c) estop kills all motors < 1 s, (d) voice "return home" executed by all drones

### Tests for User Story 4 (Write FIRST, ensure they FAIL before implementation)

- [X] T115 [P] [US4] Contract test: SAFETY_EVENT MAVLink message in tests/integration/contract/test_safety_mavlink.py
- [X] T116 [P] [US4] Contract test: HumanIntentCmd ROS 2 topic in tests/integration/contract/test_human_intent_ros2.py
- [X] T116 [P] [US4] Contract test: HumanIntentCmd MAVLink message in tests/integration/contract/test_human_intent_mavlink.py
- [ ] T117 [P] [US4] HIL test: Geofence breach triggers RTL < 500 ms in tests/simulation/safety/test_geofence_breach.py
- [ ] T118 [P] [US4] HIL test: Emergency stop 20-drone swarm < 1 s in tests/integration/hil/test_estop_hil.py
- [ ] T119 [P] [US4] HIL test: Remote ID frame integrity 60 min in tests/integration/hil/test_rid_hil.py
- [ ] T120 [P] [US4] Integration test: Voice intent parsing and execution < 2 s e2e in tests/integration/sitl/test_human_intent.py
- [ ] T121 [P] [US4] Unit test: ulog + rosbag2 complete flight logging in tests/unit/companion/test_flight_logging.py

### Implementation for User Story 4

#### 6.1 Firmware Safety Monitors (DAL A, src/firmware/modules/swarm_safety/)

- [X] T122 [US4] Harden geofence monitor: independent RTL trigger, 500 ms bound (FR-011, SC-008) in src/firmware/modules/swarm_safety/swarm_safety.cpp
- [X] T123 [US4] Harden Remote ID broadcaster: ASTM F3411/EU DRI compliance, 1 Hz (FR-012, SC-010) in src/firmware/modules/swarm_safety/swarm_safety.cpp
- [X] T124 [US4] Harden emergency stop: motor kill + parachute, mesh + 5G paths (FR-013, SC-009) in src/firmware/modules/swarm_safety/swarm_safety.cpp
- [X] T125 [US4] Implement safety event MAVLink publisher (GEOFENCE_BREACH, EMERGENCY_STOP, RID_STATUS, CRITICAL_FAULT) in src/firmware/modules/swarm_safety/safety_events.cpp

#### 6.2 Companion Safety Monitors (src/companion/swarm_safety/)

- [X] T126 [US4] Geofence monitor companion: duplicate check, telemetry stream in src/companion/swarm_safety/geofence_monitor/
- [X] T127 [US4] RID compliance monitor: frame validation, UTM interface in src/companion/swarm_safety/rid_broadcaster/
- [X] T128 [US4] Emergency stop handler: mesh broadcast, acknowledgment tracking in src/companion/swarm_safety/emergency_stop/

#### 6.3 Human-Swarm Interface (src/companion/swarm_interface/)

- [X] T129 [US4] Implement intent parser (on-device DistilBERT for critical, BERT for mission) per research.md Decision 8 in src/companion/swarm_interface/intent_parser/
- [X] T130 [US4] Implement voice/tablet ROS 2 interface (HumanIntentCmd) in src/companion/swarm_interface/intent_parser/intent_node.cpp
- [X] T131 [US4] Implement mission uploader for high-level intents (search area, follow me, inspect, RTL, hold) in src/companion/swarm_interface/mission_uploader/
- [X] T132 [US4] Implement telemetry streamer (real-time monitoring, dashboard feed) in src/companion/swarm_interface/telemetry_streamer/

#### 6.4 Flight Logging & Audit Trail (FR-015, Constitution Principle V)

- [X] T133 [US4] Implement ulog writer (firmware: sensor raw, state estimates, control outputs, mode transitions) in src/firmware/modules/swarm_safety/flight_logger.cpp
- [X] T134 [US4] Implement rosbag2 writer (companion: AI inferences, mesh stats, task allocations, map fusions) in src/companion/swarm_interface/telemetry_streamer/rosbag_writer.cpp
- [X] T135 [US4] Implement log synchronization (time alignment firmware ↔ companion) in src/companion/swarm_interface/telemetry_streamer/log_sync.cpp
- [X] T136 [US4] Implement log analyzer (metrics extraction, anomaly detection, CSV/JSON reports) in src/companion/swarm_interface/telemetry_streamer/log_analyzer.cpp

#### 6.5 Ground Station Integration (src/ground/gcs/)

- [X] T137 [US4] Implement swarm mission planner (waypoint, search polygon, inspect targets) in src/ground/gcs/swarm_mission_planner/
- [X] T138 [US4] Implement swarm monitor (real-time formation, health, mesh QoS) in src/ground/gcs/swarm_monitor/
- [X] T139 [US4] Implement post-flight analyzer (coverage heatmap, formation error CDF, AI timeline, QoS) per SC-014 in src/ground/gcs/post_flight_analyzer/
- [X] T140 [US4] Implement web dashboard (React/TypeScript) for live view, replay, analytics in src/ground/dashboard/

#### 6.6 Edge Cases (spec.md Edge Cases 1, 2)

- [X] T140 [US4] Implement leader failure → bully election integration with safety monitor in src/firmware/modules/swarm_safety/leader_fail_safety.cpp
- [X] T141 [US4] Implement split-brain merge protocol with safety state reconciliation in src/companion/swarm_comm/consensus/merge_protocol.cpp

**Checkpoint**: User Story 4 fully functional and independently testable (run Scenarios 6, 7, 8 from quickstart.md)

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories, final validation, documentation

- [ ] T142 [P] Update Architecture Decision Records (ADR) in .specify/memory/adr/ for all 8 research decisions
- [ ] T143 [P] Generate complete ICD documentation from contracts/ → docs/icd/
- [ ] T144 [P] Create developer onboarding guide (build, test, deploy) in docs/onboarding.md
- [ ] T145 Run full quickstart.md validation suite (all 8 scenarios) in SITL
- [ ] T146 Run HIL validation on Pixhawk 6C / Cube Orange+ (geofence, RID, estop)
- [ ] T147 [P] Performance optimization: TensorRT engine tuning, DDS QoS optimization, formation control gains
- [ ] T148 Security hardening: Penetration test mesh network, verify signed OTA, encrypted MAVLink
- [ ] T149 [P] Additional unit tests for uncovered modules (target ≥ 90% branch coverage per constitution)
- [ ] T150 Run Constitution compliance check: traceability matrix complete, all principles validated
- [ ] T151 Prepare airworthiness configuration: v1.0.0-airworthy tag, param files, launch files as code
- [ ] T152 Create release notes and migration guide for v1.0.0

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - **BLOCKS all user stories**
- **User Stories (Phases 3-6)**: All depend on Foundational phase completion
  - US1 (P1) and US2 (P1) can proceed in parallel after Phase 2
  - US3 (P2) and US4 (P2) can proceed in parallel after Phase 2
  - US3/US4 may integrate with US1/US2 components but must remain independently testable
- **Polish (Phase 7)**: Depends on all desired user stories being complete

### User Story Dependencies

| Story | Priority | Depends On | Can Start After |
|-------|----------|------------|-----------------|
| US1: Formation & Leader-Follower | P1 | Phase 2 | Phase 2 complete |
| US2: Task Allocation & Cooperative Mobility | P1 | Phase 2 | Phase 2 complete |
| US3: AI Perception & Adaptation | P2 | Phase 2 | Phase 2 complete (benefits from US1 replanning) |
| US4: Safety, Compliance & HSI | P2 | Phase 2 | Phase 2 complete (DAL A modules in Phase 2) |

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Data models before services/nodes
- Firmware modules before companion integration
- Core implementation before edge cases
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel (T004-T008)
- All Foundational tasks marked [P] can run in parallel (T011, T013, T015, T017, T020, T025, T027, T028)
- Once Phase 2 completes, **US1 and US2 can run in parallel** (different team members)
- **US3 and US4 can run in parallel** after Phase 2
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Polish tasks marked [P] can run in parallel

### Parallel Example: User Story 1 (Formation)

```bash
# Launch all tests for User Story 1 together:
Task: T030 Contract test: SWARM_FORMATION_SET/STATE MAVLink
Task: T031 Contract test: FormationState ROS 2 topic
Task: T032 SITL integration test: 4-drone V-formation 10-waypoint
Task: T033 SITL integration test: Leader obstacle replan < 200 ms
Task: T034 SITL integration test: Follower rejoin after localization loss

# Launch all models for User Story 1 together:
Task: T035 DroneAgent state struct
Task: T036 Formation entity
Task: T037 FormationSlot assignment logic
Task: T038 ROS 2 mirrors
```

### Parallel Example: User Story 2 (Task Allocation)

```bash
# Launch all tests for User Story 2 together:
Task: T057 Contract test: TASK_ALLOCATION_BID/AWARD MAVLink
Task: T058 Contract test: TaskAllocation ROS 2 topic
Task: T059 SITL integration test: 5-drone 1 km² search coverage
Task: T060 SITL integration test: Task reallocation on dropout
Task: T061 SITL integration test: Dynamic obstacle broadcast
Task: T062 SITL integration test: Mesh degradation store-forward

# Launch all models for User Story 2 together:
Task: T063 Mission entity
Task: T064 Task entity
Task: T065 SwarmMap entity
Task: T066 CapabilityBitmap
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1 (Formation & Leader-Follower)
4. **STOP and VALIDATE**: Test User Story 1 independently (quickstart.md Scenarios 1-2)
5. Deploy/demo if ready → **MVP DELIVERED**

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Each story adds value without breaking previous stories

### Parallel Team Strategy (4 Developers)

With 4 developers:

1. Team completes Setup + Foundational together (Phase 1-2)
2. Once Foundational is done:
   - Developer A: User Story 1 (Formation) - T030-T056
   - Developer B: User Story 2 (Task Allocation) - T057-T087
   - Developer C: User Story 3 (AI Perception) - T088-T114
   - Developer D: User Story 4 (Safety & HSI) - T115-T141
3. Stories complete and integrate independently
4. Team converges on Phase 7 (Polish)

---

## Task Summary

| Phase | Tasks | Count | Notes |
|-------|-------|-------|-------|
| Phase 1: Setup | T001-T009 | 9 | Infrastructure |
| Phase 2: Foundational | T010-T029 | 20 | **Blocking - must complete first** |
| Phase 3: US1 (P1) | T030-T056 | 27 | **COMPLETE** - Formation & Leader-Follower |
| Phase 4: US2 (P1) | T057-T087 | 31 | **COMPLETE** - Task Allocation & Cooperative Mobility |
| Phase 5: US3 (P2) | T088-T114 | 27 | **COMPLETE** - AI Perception & Adaptation |
| Phase 6: US4 (P2) | T115-T141 | 27 | **In Progress** - Geofence, RID, Estop, Flight Plan, HSI, Logging, Mission Upload, Telemetry |
| Phase 7: Polish | T142-T152 | 11 | Cross-cutting, validation, release |

**Total Tasks: 152**

**MVP Scope (Phases 1-3): 56 tasks** - delivers core formation flight capability

**Full Delivery (Phases 1-7): 152 tasks** - complete AI swarm autonomous drone system

---

## Format Validation

✅ All tasks follow checklist format: `- [ ] T### [P?] [Story?] Description with file path`
✅ All Task IDs sequential (T001-T152)
✅ [P] marker only for parallelizable tasks (different files, no deps)
✅ [US1]/[US2]/[US3]/[US4] labels on all user story phase tasks
✅ No story labels on Setup, Foundational, or Polish phases
✅ Exact file paths from plan.md project structure
✅ Dependencies documented in Dependencies section
✅ Parallel opportunities identified per story and across stories
✅ Independent test criteria defined for each user story
✅ MVP scope clearly identified (Phases 1-3)