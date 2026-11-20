# DO-178C Traceability Matrix Template
# AI Swarm Autonomous Drone System
# Document: TM-SWARM-001
# Version: 1.0
# Date: 2025-11-19

---

## 1. Purpose

This traceability matrix establishes bidirectional traceability between:
- **System Requirements** (from spec.md: FR-001 through FR-020)
- **Software Requirements** (from plan.md, data-model.md, contracts/)
- **Software Architecture** (from plan.md project structure)
- **Source Code** (firmware modules, companion nodes, ground tools)
- **Verification Cases** (from quickstart.md validation scenarios, test files)

Per DO-178C objectives for **DAL A** (swarm_safety: geofence, RID, estop) and **DAL C** (autonomy modules).

---

## 2. Traceability Levels

| Level | Artifact | Document Reference |
|-------|----------|-------------------|
| L0 | System Requirements | spec.md §Requirements (FR-001..FR-020) |
| L1 | High-Level Requirements | plan.md §Technical Context, §Constitution Check |
| L2 | Low-Level Requirements | data-model.md entities, contracts/ interfaces |
| L3 | Architecture | plan.md §Project Structure |
| L4 | Source Code | src/firmware/, src/companion/, src/ground/ |
| L5 | Verification | quickstart.md scenarios, tests/ |

---

## 3. System Requirements → Verification Matrix (DAL A: FR-011, FR-012, FR-013)

| Sys Req ID | Description | DAL | High-Level Req | Low-Level Req | Architecture | Source Code | Verification |
|------------|-------------|-----|----------------|---------------|--------------|-------------|--------------|
| **FR-011** | Hard geofence at FC firmware level (independent of companion) | A | Constitution Principle I, Regulatory §Airspace | data-model.md: GeofenceConfig, GeofenceAction; contracts/mavlink: SAFETY_EVENT GEOFENCE_BREACH | Firmware: swarm_safety/geofence_monitor; Companion: swarm_safety/geofence_monitor | src/firmware/modules/swarm_safety/geofence_monitor.hpp/.cpp<br>src/companion/swarm_safety/geofence_monitor/ | quickstart.md Scenario 6; tests/integration/hil/test_geofence_hil.py (T117); SC-008: RTL < 500ms |
| **FR-012** | Remote ID broadcast ASTM F3411/EU DRI at 1 Hz | A | Constitution Regulatory §Airspace; research.md Decision 5 | data-model.md: RemoteIdConfig, RemoteIdFrame; contracts/mavlink: SAFETY_EVENT RID_STATUS | Firmware: swarm_safety/rid_broadcaster; Companion: swarm_safety/rid_broadcaster | src/firmware/modules/swarm_safety/rid_broadcaster.hpp/.cpp<br>src/companion/swarm_safety/rid_broadcaster/ | quickstart.md Scenario 7; tests/integration/hil/test_rid_hil.py (T119); SC-010: 100% frame integrity 60min |
| **FR-013** | Emergency stop (kill switch) disabling all motors < 1s via GCS and mesh | A | Constitution Principle I, Regulatory §Security; research.md Decision 5 | data-model.md: EmergencyStopConfig; contracts/mavlink: SAFETY_EVENT EMERGENCY_STOP, HUMAN_INTENT_CMD ESTOP | Firmware: swarm_safety/emergency_stop; Companion: swarm_safety/emergency_stop, swarm_comm/mesh_manager | src/firmware/modules/swarm_safety/emergency_stop.hpp/.cpp<br>src/companion/swarm_safety/emergency_stop/<br>src/companion/swarm_comm/mesh_manager/ | quickstart.md Scenario 6; tests/integration/hil/test_estop_hil.py (T118); SC-009: 20-drone swarm < 1s |

---

## 4. System Requirements → Verification Matrix (DAL C: FR-001..FR-010, FR-014..FR-020)

| Sys Req ID | Description | DAL | High-Level Req | Low-Level Req | Architecture | Source Code | Verification |
|------------|-------------|-----|----------------|---------------|--------------|-------------|--------------|
| **FR-001** | Leader-follower formation flight, configurable geometries, 2-50 drones | C | US1 (P1); Constitution III | data-model.md: Formation, FormationGeometry, FormationSlot; contracts: SWARM_FORMATION_SET/STATE | Firmware: swarm_formation; Companion: formation_control | src/firmware/modules/swarm_formation/*<br>src/companion/swarm_autonomy/formation_control/ | quickstart.md Scenario 1 (T032); SC-001 |
| **FR-002** | Formation position error ≤ 0.5m lateral, ≤ 0.3m vertical (wind < 5m/s) | C | US1; Constitution II | data-model.md: FormationTolerances; contracts: FormationState.position_error_flu | Firmware: follower_control; Companion: formation_state_node | src/firmware/modules/swarm_formation/follower_control.cpp<br>src/companion/swarm_autonomy/formation_control/formation_state_node.cpp | quickstart.md Scenario 1; SC-001: mean lat < 0.4m, vert < 0.2m |
| **FR-003** | Leader replan propagation to followers < 200ms | C | US1; Constitution II, IV | data-model.md: FormationTolerances.replan_propagation_ms; contracts: SWARM_FORMATION_SET | Firmware: follower_replan; Companion: trajectory_planner, consensus | src/firmware/modules/swarm_formation/follower_replan.cpp<br>src/companion/swarm_autonomy/trajectory_planner/<br>src/companion/swarm_comm/consensus/ | quickstart.md Scenario 2 (T033); SC-007: replan < 100ms |
| **FR-004** | Leaderless multi-agent task allocation (CBBA) | C | US2 (P1); research.md Decision 1 | data-model.md: Mission, Task, CapabilityBitmap; contracts: TASK_ALLOCATION_BID/AWARD | Companion: task_allocator (CBBA) | src/companion/swarm_autonomy/task_allocator/ | quickstart.md Scenario 3 (T059); SC-002 |
| **FR-005** | ≥ 95% area coverage for search missions | C | US2; Constitution IV | data-model.md: TaskGeometry.area, SuccessCriteria.min_coverage_pct | Companion: task_allocator, search_patterns | src/companion/swarm_autonomy/task_allocator/search_patterns.cpp | quickstart.md Scenario 3 (T059); SC-002: 1km² ≤ 30min, 95% |
| **FR-006** | Task reallocation < 5s on drone dropout | C | US2; research.md Decision 1 | data-model.md: Task.status REEVALUATING; contracts: TASK_ALLOCATION_BID/AWARD | Companion: task_allocator reallocation | src/companion/swarm_autonomy/task_allocator/reallocation.cpp | quickstart.md Scenario 3 (T060); SC-003 |
| **FR-007** | Obstacle detection/classification ≥ 30m, F1 ≥ 0.92 | C | US3 (P2); research.md Decision 2 | data-model.md: AIInferenceResult, ObstacleMeta; contracts: AI_INFERENCE_REPORT | Companion: ai_perception/obstacle_detector | src/companion/ai_perception/obstacle_detector/ | quickstart.md Scenario 4 (T090); SC-004 |
| **FR-008** | Landing site assessment ≥ 95% accuracy | C | US3; research.md Decision 2 | data-model.md: LandingMeta; contracts: AI_INFERENCE_REPORT | Companion: ai_perception/landing_assessor | src/companion/ai_perception/landing_assessor/ | quickstart.md Scenario 4; SC-005 |
| **FR-009** | Sensor/actuator anomaly detection < 200ms | C | US3; research.md Decision 2 | data-model.md: FaultMeta; contracts: AI_INFERENCE_REPORT | Companion: ai_perception/fault_detector | src/companion/ai_perception/fault_detector/ | quickstart.md Scenario 4 (T092); SC-006 |
| **FR-010** | Adaptive replanning < 100ms of detection | C | US3; Constitution II, IV | data-model.md: AIInferenceResult.requires_replan; contracts: trajectory_planner | Companion: trajectory_planner, ai_replan_integration | src/companion/swarm_autonomy/trajectory_planner/ai_replan_integration.cpp | quickstart.md Scenario 4; SC-007 |
| **FR-014** | Human intent commands (voice/tablet): search, follow, inspect, RTL, hold | C | US4 (P2); research.md Decision 8 | data-model.md: HumanIntentCmd; contracts: HUMAN_INTENT_CMD | Companion: intent_parser, mission_uploader | src/companion/swarm_interface/intent_parser/<br>src/companion/swarm_interface/mission_uploader/ | quickstart.md Scenario 8 (T120); SC-011 |
| **FR-015** | Complete flight logging (ulog + rosbag2) | C | Constitution Principle V | data-model.md: all entities timestamp_us; contracts: all messages | Firmware: flight_logger; Companion: telemetry_streamer, rosbag_writer | src/firmware/modules/swarm_safety/flight_logger.cpp<br>src/companion/swarm_interface/telemetry_streamer/ | quickstart.md all scenarios; SC-014 |
| **FR-016** | Heterogeneous drones with capability advertisement | C | US2 Edge Case 4 | data-model.md: CapabilityBitmap; contracts: TASK_ALLOCATION_BID | Companion: capability_advertisement | src/companion/swarm_autonomy/task_allocator/capability_advertisement.cpp | quickstart.md Scenario 3 |
| **FR-017** | GPS-denied operation: VIO + UWB + landmarks, drift < 1% | C | US3; research.md Decision 4 | data-model.md: AgentState.vio_valid, uwb_peers; contracts: state_estimator | Companion: state_estimator (VIO+UWB GTSAM) | src/companion/swarm_autonomy/state_estimator/vio_uwb_fusion.cpp | quickstart.md Scenario 5 (T091); SC-012 |
| **FR-018** | Inter-drone separation > 5m via distributed collision avoidance | C | US2; Constitution I | data-model.md: FormationTolerances, SwarmMap; contracts: collision_avoidance | Companion: collision_avoidance (ORCA/MPC) | src/companion/swarm_autonomy/collision_avoidance/ | quickstart.md Scenario 3; SC-002 (zero collisions) |
| **FR-019** | Mesh networking (Wi-Fi 6E/5G/LoRa) with store-forward at >30% loss | C | US2; research.md Decision 3 | data-model.md: CommunicationLink, LinkQoS; contracts: CommQoS, SWARM_COMM_QOS | Companion: mesh_manager, store_forward, consensus | src/companion/swarm_comm/ | quickstart.md Scenario 3 (T062); SC-013 |
| **FR-020** | Mission replay & post-flight dashboard | C | Constitution Principle V | data-model.md: all logged entities; contracts: telemetry | Ground: post_flight_analyzer, dashboard | src/ground/gcs/post_flight_analyzer/<br>src/ground/dashboard/ | quickstart.md post-flight; SC-014: report < 60s |

---

## 5. Code → Requirements Traceability (Sample)

| Source File | Module | Traced Requirements | DO-178C Objective |
|-------------|--------|---------------------|-------------------|
| src/firmware/modules/swarm_safety/geofence_monitor.cpp | swarm_safety | FR-011, FR-015 | 6.3.1a (HLR), 6.3.2a (LLR), 6.4.2a (code) |
| src/firmware/modules/swarm_safety/rid_broadcaster.cpp | swarm_safety | FR-012, FR-015 | 6.3.1a, 6.3.2a, 6.4.2a |
| src/firmware/modules/swarm_safety/emergency_stop.cpp | swarm_safety | FR-013, FR-015 | 6.3.1a, 6.3.2a, 6.4.2a |
| src/firmware/modules/swarm_formation/leader_trajectory.cpp | swarm_formation | FR-001, FR-003 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_formation/follower_control.cpp | swarm_formation | FR-001, FR-002 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_formation/geometry_manager.cpp | swarm_formation | FR-001 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_formation/formation_fsm.cpp | swarm_formation | FR-001 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_formation/mavlink_handler.cpp | swarm_formation | FR-001, FR-003, FR-015 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_formation/relative_localization.cpp | swarm_formation | FR-001, FR-002, FR-017 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_formation/leader_election.hpp | swarm_formation | FR-001, Edge Case 1 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_comm/mavlink_router.hpp | swarm_comm | FR-003, FR-013, FR-019 | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_comm/common/timestamp.hpp | common | All (timestamp) | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_comm/common/crypto.hpp | common | FR-013, FR-015, Security | 6.3.1a, 6.3.2a |
| src/firmware/modules/swarm_comm/common/config.hpp | common | All (configuration) | 6.3.1a, 6.3.2a |
| src/firmware/platforms/pixhawk6c/hal_pixhawk6c.cpp | hal | FR-011, FR-012, FR-013, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_autonomy/formation_control/formation_state_node.cpp | formation_control | FR-001, FR-002, FR-003, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_autonomy/task_allocator/cbba.hpp | task_allocator | FR-004, FR-005, FR-006 | 6.3.1a, 6.3.2a |
| src/companion/swarm_autonomy/task_allocator/task_allocator_node.cpp | task_allocator | FR-004, FR-005, FR-006, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/ai_perception/obstacle_detector/obstacle_node.cpp | ai_perception | FR-007, FR-010, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/ai_perception/model_runtime/inference_engine.hpp | ai_perception | FR-007, FR-008, FR-009, FR-010 | 6.3.1a, 6.3.2a |
| src/companion/swarm_comm/mesh_manager/ | swarm_comm | FR-019 | 6.3.1a, 6.3.2a |
| src/companion/swarm_safety/geofence_monitor/ | swarm_safety | FR-011, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_safety/rid_broadcaster/ | swarm_safety | FR-012, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_safety/emergency_stop/ | swarm_safety | FR-013, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_interface/intent_parser/ | swarm_interface | FR-014, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_interface/telemetry_streamer/rosbag_writer.cpp | swarm_interface | FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_common/timestamp.py | common | All | 6.3.1a, 6.3.2a |
| src/companion/swarm_common/crypto.py | common | FR-013, FR-015 | 6.3.1a, 6.3.2a |
| src/companion/swarm_common/config.py | common | All | 6.3.1a, 6.3.2a |
| src/companion/swarm_common/hal.py | common | FR-011, FR-012, FR-013 | 6.3.1a, 6.3.2a |

---

## 6. Verification Coverage Summary

| Verification Method | Test Files | Requirements Covered |
|---------------------|------------|---------------------|
| **Unit Test** | tests/unit/companion/test_formation_control.cpp | FR-001, FR-002, FR-004, FR-006, FR-007, FR-009 |
| **Integration Contract Test** | tests/integration/contract/test_formation_mavlink.py, test_task_allocation_mavlink.py, test_ai_inference_mavlink.py, test_safety_mavlink.py, test_human_intent_ros2.py | All FRs (interface compliance) |
| **SITL Integration** | tests/simulation/formation/test_formation_hold.py, test_replan_propagation.py, test_follower_rejoin.py, tests/simulation/task_allocation/test_search_coverage.py, test_reallocation.py, test_obstacle_broadcast.py, test_mesh_degradation.py, tests/simulation/ai_perception/test_obstacle_avoidance.py, test_gps_denied.py, test_fault_detection.py | FR-001..FR-010, FR-014..FR-020 |
| **HIL Test** | tests/integration/hil/test_geofence_hil.py, test_estop_hil.py, test_rid_hil.py | FR-011, FR-012, FR-013 (DAL A) |
| **Quickstart Scenarios** | quickstart.md Scenarios 1-8 | End-to-end FR validation |

---

## 7. DO-178C Objective Satisfaction Checklist

| Objective | Description | Status | Evidence |
|-----------|-------------|--------|----------|
| 6.3.1a | High-level requirements comply with system requirements | ✅ | This matrix L0→L1 |
| 6.3.2a | Low-level requirements comply with high-level requirements | ✅ | This matrix L1→L2 |
| 6.3.3a | Architecture complies with low-level requirements | ✅ | This matrix L2→L3 |
| 6.3.4a | Source code complies with low-level requirements | ✅ | This matrix L2→L4 (§5) |
| 6.3.5a | Source code complies with architecture | ✅ | plan.md structure vs src/ |
| 6.3.6a | Verification cases cover high-level requirements | ✅ | §6 mapping |
| 6.3.6b | Verification cases cover low-level requirements | ✅ | §6 mapping |
| 6.3.6c | Verification procedures correct | ✅ | quickstart.md, test files |
| 6.4.1a | Test coverage of high-level requirements | ✅ | Unit + Integration + SITL + HIL |
| 6.4.1b | Test coverage of low-level requirements | ✅ | Contract tests + unit tests |
| 6.4.2a | Source code verified against low-level requirements | ✅ | Code reviews, static analysis |
| 6.4.2b | Source code verified against architecture | ✅ | Code reviews |
| 6.4.3a | Parameter data item file verified | ✅ | config.py, params YAML |
| 6.4.4a | Tool qualification (if applicable) | ⏳ | clang-tidy, cppcheck, gtest, pytest qualified per project |

---

## 8. Change Impact Analysis Template

When a requirement changes, use this template:

```
CHANGE REQUEST: CR-SWARM-XXX
AFFECTED REQUIREMENT: FR-XXX
CHANGE DESCRIPTION: [What changed]
IMPACT ANALYSIS:
  - High-Level Requirements: [IDs]
  - Low-Level Requirements: [IDs]  
  - Architecture: [Components]
  - Source Code: [Files from §5]
  - Verification: [Test cases from §6]
  - Documentation: [spec.md, plan.md, data-model.md, contracts/]
RISK ASSESSMENT: [Low/Medium/High]
APPROVAL: [DAL A changes require Safety Engineer + Lead Autonomy Engineer]
```

---

## 9. Approval Signatures

| Role | Name | Signature | Date |
|------|------|-----------|------|
| Safety Engineer (DAL A) | | | |
| Lead Autonomy Engineer | | | |
| Software Lead | | | |
| Quality Assurance | | | |

---

*Document controlled per Constitution §Governance. Amendments require written proposal, safety impact analysis, review, migration plan, version bump, and ratification.*