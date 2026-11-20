# Quickstart Validation Guide: AI Swarm Autonomous Drone System

**Generated**: 2025-11-19 | **Phase**: 1 (Design) | **Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

---

## Prerequisites

### Hardware (Minimum for SITL Validation)
- **Host PC**: Ubuntu 22.04/24.04, 32 GB RAM, NVIDIA GPU (RTX 3080+ recommended for Gazebo + AI)
- **Docker**: 24+ with NVIDIA Container Toolkit
- **Network**: Isolated Wi-Fi 6E AP or 5G test core (for mesh validation)

### Software Dependencies
```bash
# Core
sudo apt update && sudo apt install -y \
    python3.11 python3.11-venv python3-pip \
    ros-humble-desktop-full \
    gazebo-harmonic \
    px4-dev-simulation \
    docker.io nvidia-container-toolkit

# Python deps (in venv)
python3.11 -m venv ~/.venv_swarm
source ~/.venv_swarm/bin/activate
pip install -r requirements.txt  # See below
```

### Repository Setup
```bash
git clone <repo-url> drone-swarm
cd drone-swarm
./tools/setup_dev_env.sh --target=sitl
```

---

## Validation Scenarios

Each scenario proves a specific user story from the spec. Run in order.

### Scenario 1: Formation Hold (User Story 1, P1)
**Validates**: FR-001, FR-002, FR-003, SC-001

```bash
# 1. Launch 4-drone SITL swarm (1 leader + 3 followers, V-formation)
cd simulation/sitl
./launch_swarm.sh --count=4 --formation=vee --leader=0

# 2. Upload 10-waypoint mission (5 km total)
ros2 run swarm_autonomy mission_uploader \
    --mission ../scenarios/formation_10wp.yaml \
    --formation=vee

# 3. Monitor formation error (real-time)
ros2 topic echo /swarm/formation_state | grep position_error

# 4. Automated verification (post-flight)
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check formation_hold \
    --max-lateral-error 0.5 \
    --max-vertical-error 0.3
```

**Expected Outcome**:
- All 4 drones take off and form V-formation within 10 s
- Mean lateral error < 0.4 m, vertical < 0.2 m throughout mission
- All 10 waypoints reached
- Sequenced landing at designated pads

---

### Scenario 2: Leader Obstacle Replan (User Story 1, P1)
**Validates**: FR-003, SC-007

```bash
# 1. Start formation mission (Scenario 1 setup)
# 2. Inject dynamic obstacle at waypoint 5 (Gazebo model spawn)
ros2 service call /gazebo/spawn_entity gazebo_msgs/SpawnEntity "
  name: 'power_lines'
  xml: '<sdf version=\"1.6\"><model name=\"power_lines\"><pose>100 50 10 0 0 0</pose>...</model></sdf>'
  robot_namespace: ''
  initial_pose: {position: {x: 100.0, y: 50.0, z: 10.0}}
"

# 3. Verify replan propagation
ros2 topic echo /swarm/formation_state --once | grep replan_propagation_ms

# 4. Check follower adjustment
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check replan_latency \
    --max-ms 200
```

**Expected Outcome**:
- Leader detects obstacle at ≥ 25 m (FR-007)
- New trajectory computed and propagated to followers < 200 ms (FR-003, SC-007)
- Followers adjust formation positions within 2 s without collision (SC-001)

---

### Scenario 3: Task Reallocation on Drone Loss (User Story 2, P1)
**Validates**: FR-004, FR-005, FR-006, SC-002, SC-003

```bash
# 1. Launch 5-drone search swarm (leaderless)
cd simulation/sitl
./launch_swarm.sh --count=5 --formation=none --mode=search

# 2. Upload 1 km² area search mission
ros2 run swarm_autonomy mission_uploader \
    --mission ../scenarios/search_1km2.yaml \
    --type=search

# 3. At 50% coverage, simulate drone 3 battery critical
ros2 service call /drone3/simulate_fault std_srvs/SetBool "data: true"

# 4. Verify reallocation
ros2 topic echo /swarm/task_allocation --once
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check task_reallocation \
    --max-realloc-time 5 \
    --max-coverage-gap 0.02
```

**Expected Outcome**:
- 5 drones achieve ≥ 95% coverage in ≤ 30 min (SC-002)
- Drone 3 announces RTB, remaining 4 reallocate sub-area within 5 s (FR-006, SC-003)
- Coverage gap < 2% of total area (SC-003)
- Zero inter-drone collisions (FR-018)

---

### Scenario 4: AI Obstacle Avoidance (User Story 3, P2)
**Validates**: FR-007, FR-010, SC-004, SC-007

```bash
# 1. Launch single drone with AI perception enabled
cd simulation/sitl
./launch_swarm.sh --count=1 --ai-enabled=true

# 2. Spawn photorealistic power lines at 25 m ahead (domain-randomized texture)
ros2 service call /gazebo/spawn_entity gazebo_msgs/SpawnEntity "
  name: 'power_lines_dr'
  xml: '<sdf version=\"1.6\"><model name=\"power_lines_dr\"><pose>25 0 5 0 0 0</pose>...</model></sdf>'
  initial_pose: {position: {x: 25.0, y: 0.0, z: 5.0}}
"

# 3. Fly toward obstacle at 10 m/s (guided mode)
ros2 run swarm_autonomy guided_flight --target 50 0 5 --speed 10

# 4. Verify detection and avoidance
ros2 topic echo /drone0/ai_inference --once | grep -E "class_name|confidence|threat_level"
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check obstacle_avoidance \
    --min-f1 0.92 \
    --max-avoidance-latency 200
```

**Expected Outcome**:
- Power lines detected at ≥ 25 m, classified as "thin_obstacle" (FR-007)
- Confidence ≥ 0.92, threat_level CRITICAL (SC-004)
- Avoidance maneuver initiated within 200 ms (FR-010, SC-007)
- Mission resumes after clearance

---

### Scenario 5: GPS-Denied Operation (User Story 3, P2)
**Validates**: FR-017, SC-012

```bash
# 1. Launch drone with VIO + UWB (2 drones for ranging)
cd simulation/sitl
./launch_swarm.sh --count=2 --gps-denied=true --uwb-enabled=true

# 2. Fly 2 km trajectory without GPS (Gazebo GPS noise model + jamming)
ros2 run swarm_autonomy mission_uploader \
    --mission ../scenarios/gps_denied_2km.yaml

# 3. Verify position drift
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check gps_denied_drift \
    --max-drift-pct 1.0
```

**Expected Outcome**:
- Position drift < 1% of distance traveled (≤ 20 m over 2 km) (SC-012)
- VIO + UWB fusion maintains localization (FR-017)
- Mission completes or safe RTL

---

### Scenario 6: Emergency Stop (User Story 4, P2)
**Validates**: FR-013, SC-009

```bash
# 1. Launch 10-drone swarm in formation
cd simulation/sitl
./launch_swarm.sh --count=10 --formation=vee

# 2. Start mission
ros2 run swarm_autonomy mission_uploader --mission ../scenarios/formation_10wp.yaml

# 3. Trigger emergency stop from GCS
ros2 service call /swarm/emergency_stop swarm_msgs/srv/EmergencyStop "
  target_id: 0
  deploy_parachute: false
  reason: 'Operator emergency stop test'
"

# 4. Verify all motors stop < 1 s
ros2 topic echo /swarm/safety_event --once | grep EMERGENCY_STOP
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check emergency_stop \
    --max-kill-time 1.0 \
    --drone-count 10
```

**Expected Outcome**:
- All 10 drones receive estop command via mesh within 1 s (SC-009)
- Motors disarm, controlled descent initiated (FR-013)
- Safety event logged with EMERGENCY severity (FR-015)

---

### Scenario 7: Remote ID Broadcast (User Story 4, P2)
**Validates**: FR-012, SC-010

```bash
# 1. Launch any swarm configuration
cd simulation/sitl
./launch_swarm.sh --count=5 --rid-enabled=true

# 2. Run 60-minute mission (accelerated sim time)
ros2 run swarm_autonomy mission_uploader --mission ../scenarios/rid_60min.yaml

# 3. Capture RID frames (simulated UTM receiver)
ros2 topic echo /swarm/rid_broadcast --duration 3600 > rid_frames.json

# 4. Validate RID compliance
python3 tools/rid_validator.py \
    --input rid_frames.json \
    --standard astm_f3411 \
    --duration 3600
```

**Expected Outcome**:
- All drones broadcast valid RID frames at 1 Hz (FR-012)
- 100% frame integrity over 60 min (SC-010)
- Self-ID, Operator-ID, Position, Velocity fields populated per ASTM F3411

---

### Scenario 8: Human Intent Command (User Story 4, P2)
**Validates**: FR-014, SC-011

```bash
# 1. Launch swarm with voice interface
cd simulation/sitl
./launch_swarm.sh --count=3 --voice-enabled=true

# 2. Send voice command via ROS 2 (simulated ASR output)
ros2 topic pub /swarm/human_intent swarm_msgs/msg/HumanIntentCmd "
  timestamp_us: $(date +%s%6N)
  target_id: 0
  type: 3  # RETURN_HOME
  params: []
  priority: 1
"

# 3. Verify execution
ros2 topic echo /swarm/safety_event --once | grep RETURN_HOME
python3 tools/log_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --check intent_latency \
    --max-e2e-latency 2.0
```

**Expected Outcome**:
- Command parsed and executed by all drones within 2 s end-to-end (SC-011)
- All drones transition to RTL mode (FR-014)
- Intent logged in flight data (FR-015)

---

## CI/CD Integration

### GitHub Actions Workflow (`.github/workflows/sitl_regression.yml`)

```yaml
name: SITL Regression Tests

on:
  push:
    branches: [develop, main]
  pull_request:
    branches: [develop, main]

jobs:
  sitl-formation:
    runs-on: ubuntu-22.04-gpu
    timeout-minutes: 30
    steps:
      - uses: actions/checkout@v4
      - name: Build SITL
        run: ./tools/build_all.sh --target=sitl
      - name: Run Formation Test Suite
        run: |
          source ~/.venv_swarm/bin/activate
          python3 simulation/test_runner.py \
            --suite=formation \
            --drones=5 \
            --waypoints=20 \
            --max-lateral-error=0.4 \
            --max-vertical-error=0.2
      - name: Upload Logs
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: formation-logs
          path: ~/.ros/swarm_logs/

  sitl-search:
    runs-on: ubuntu-22.04-gpu
    timeout-minutes: 45
    steps:
      - uses: actions/checkout@v4
      - name: Build SITL
        run: ./tools/build_all.sh --target=sitl
      - name: Run Search Test Suite
        run: |
          source ~/.venv_swarm/bin/activate
          python3 simulation/test_runner.py \
            --suite=search \
            --area=1km2 \
            --drones=5 \
            --min-coverage=0.95 \
            --max-time=1800
      - name: Upload Logs
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: search-logs
          path: ~/.ros/swarm_logs/

  sitl-ai-perception:
    runs-on: ubuntu-22.04-gpu
    timeout-minutes: 30
    steps:
      - uses: actions/checkout@v4
      - name: Build SITL + AI
        run: ./tools/build_all.sh --target=sitl --ai-enabled
      - name: Run AI Perception Tests
        run: |
          source ~/.venv_swarm/bin/activate
          python3 simulation/test_runner.py \
            --suite=ai_perception \
            --models=yolov8n_obstacle,mobilenetv3_landing,lstm_fault \
            --min-f1=0.92 \
            --max-latency=50
      - name: Upload Logs
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: ai-logs
          path: ~/.ros/swarm_logs/
```

---

## Post-Flight Analysis Dashboard

After any field test or SITL run:

```bash
# Generate comprehensive report
python3 tools/post_flight_analyzer.py \
    --logs ~/.ros/swarm_logs/latest/ \
    --output ./flight_report_$(date +%Y%m%d_%H%M%S).html

# Key outputs:
# - Coverage heatmap (search missions)
# - Formation error CDF (formation missions)
# - AI detection timeline (confidence, threat level, position)
# - Network QoS report (latency, loss, bandwidth per link)
# - Safety event timeline
# - Battery/energy consumption per drone
```

**Dashboard validates**: SC-014 (report generation < 60 s)

---

## Next Steps

After all validation scenarios pass:

1. **Run `/speckit-tasks`** → Generate actionable, dependency-ordered `tasks.md`
2. **Run `/speckit-implement`** → Execute implementation per tasks
3. **Progress through Constitution test pyramid**: Unit → Integration → SITL → HIL → Field

---

## Troubleshooting

| Issue | Resolution |
|-------|------------|
| Gazebo fails to spawn models | Check `GAZEBO_MODEL_PATH`, run `gazebo --verbose` for errors |
| ROS 2 discovery fails | Verify `FASTRTPS_DEFAULT_PROFILES_FILE=fastdds.xml`, check firewall |
| AI inference > 50 ms | Verify TensorRT engine built (`trtexec --onnx=model.onnx --saveEngine=model.engine`) |
| Mesh link DOWN | Check Wi-Fi 6E channel (36-48), 5G APN config, LoRa frequency/bandwidth |
| Formation error > 0.5 m | Tune formation control gains (`gain_xy`, `gain_z`), check UWB ranging |
| Drone fails to arm | Check pre-arm checks: GPS fix, battery, RC calibration, geofence load |

---

**Ready for `/speckit-tasks`** when all scenarios pass in SITL.