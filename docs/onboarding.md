# Developer Onboarding Guide

**Version**: 1.0.0  
**Last Updated**: 2025-11-20  
**Target Audience**: New developers joining the AI Swarm Autonomous Drone System project

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [Prerequisites](#prerequisites)
3. [Environment Setup](#environment-setup)
4. [Repository Structure](#repository-structure)
5. [Build & Test](#build--test)
6. [Development Workflow](#development-workflow)
7. [Architecture Overview](#architecture-overview)
8. [Key Components](#key-components)
9. [Testing Strategy](#testing-strategy)
10. [Deployment](#deployment)
11. [Troubleshooting](#troubleshooting)
12. [Resources](#resources)

---

## Project Overview

The **AI Swarm Autonomous Drone System** is a fully autonomous multi-drone swarm system implementing:

| Capability | Status | Priority |
|------------|--------|----------|
| Formation Flight (Leader-Follower) | ✅ 27/27 | P1 |
| Task Allocation (CBBA Consensus) | ✅ 31/31 | P1 |
| AI Perception (Obstacle/Landing/Fault) | ✅ 27/27 | P2 |
| Safety & Compliance (DAL A) | 🚧 22/27 | P2 |
| Ground Station (QGC Plugin + Web) | 🚧 2/4 | P2 |

**Architecture**: PX4 C++17 firmware + ROS 2 Humble C++20/Python companion, tiered mesh (Wi-Fi 6E/5G/LoRa), TensorRT/ONNX AI, GTSAM VIO+UWB, CBBA allocator, ORCA/MPC collision avoidance, CBBA consensus <200ms.

**Repository**: https://github.com/krill0188/my-drone-project

---

## Prerequisites

### Hardware Requirements
| Component | Specification |
|-----------|---------------|
| **Flight Controller** | Pixhawk 6C / Cube Orange+ / CUAV Nora |
| **Companion Computer** | Jetson Orin NX 16GB / Snapdragon Flight RB5 |
| **Sensors** | Dual IMU/GPS, Barometer, Mag, Stereo/Downward Cameras, 2D/3D LiDAR, UWB |
| **Communication** | Wi-Fi 6E, 5G NR, LoRa modules |
| **Development PC** | Ubuntu 22.04+, 32GB RAM, NVIDIA GPU (for TensorRT) |

### Software Requirements
| Tool | Version | Purpose |
|------|---------|---------|
| **Ubuntu** | 22.04 LTS | Base OS |
| **Docker** | 24+ | Containerization |
| **Git** | 2.34+ | Version control |
| **VS Code** | Latest | IDE (recommended) |
| **Python** | 3.11+ | Scripts, backend, tests |
| **CMake** | 3.22+ | Build system |
| **Clang/GCC** | 14+/11+ | C++17/20 compiler |
| **ROS 2** | Humble Hawksbill | Middleware |
| **PX4** | v1.14+ | Firmware |
| **Gazebo** | Harmonic | Simulation |
| **Node.js** | 18+ | Web dashboard |
| **TensorRT** | 8.6+ | AI inference |
| **ONNX Runtime** | 1.16+ | AI inference fallback |

### Accounts & Access
- GitHub: `krill0188/my-drone-project` (request access)
- Docker Hub: `swarm-dashboard` (for images)
- Mapbox: Access token for maps
- NVIDIA NGC: TensorRT containers

---

## Environment Setup

### 1. Clone Repository
```bash
git clone https://github.com/krill0188/my-drone-project.git
cd my-drone-project
git submodule update --init --recursive
```

### 2. Install System Dependencies
```bash
# Ubuntu 22.04
sudo apt update && sudo apt install -y \
    git cmake ninja-build clang-14 lld-14 \
    python3.11 python3.11-venv python3-pip \
    docker.io docker-compose \
    ros-humble-desktop \
    gazebo-harmonic \
    libgtsam-dev libeigen3-dev \
    libopencv-dev libopencv-contrib-dev \
    nlohmann-json3-dev \
    protobuf-compiler libprotobuf-dev \
    libyaml-cpp-dev

# ROS 2 Humble setup
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Python venv
python3.11 -m venv ~/swarm-venv
source ~/swarm-venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
```

### 3. PX4 Firmware Setup
```bash
cd src/firmware
git submodule update --init --recursive
bash ./Tools/setup/ubuntu.sh
# Build firmware
cd /my-drone-project/src/firmware
make px4_sitl gazebo-classic  # For SITL testing
```

### 4. Companion Software Setup
```bash
cd /my-drone-project
# ROS 2 workspace
mkdir -p ~/swarm_ws/src
ln -s /my-drone-project/src/companion ~/swarm_ws/src/
cd ~/swarm_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 5. Web Dashboard Setup
```bash
cd /my-drone-project/src/ground/dashboard
cp .env.example .env
# Edit .env with your Mapbox token
npm install
npm run dev  # Development server at http://localhost:3000
```

### 6. Verification
```bash
# Run tests
cd /my-drone-project
./run_tests.sh  # Runs all unit/integration tests

# Quick health check
python3 -c "import swarm_msgs; print('ROS 2 msgs OK')"
python3 -c "import torch; print('PyTorch OK')"
python3 -c "import tensorrt; print('TensorRT OK')"  # If available
```

---

## Repository Structure

```
my-drone-project/
├── .specify/                 # SpecKit methodology artifacts
│   ├── memory/
│   │   ├── constitution.md   # Project constitution
│   │   ├── adr/              # Architecture Decision Records
│   │   └── research.md       # Research decisions
│   └── specs/
│       └── 001-ai-swarm-autonomous-drones/
│           ├── spec.md       # Requirements spec
│           ├── plan.md       # Implementation plan
│           ├── tasks.md      # Task tracking (152 tasks)
│           └── quickstart.md # Quickstart scenarios
├── src/
│   ├── firmware/             # PX4 C++17 firmware modules
│   │   ├── modules/
│   │   │   ├── swarm_safety/     # Geofence, RID, Estop, LeaderFail
│   │   │   ├── swarm_formation/  # Formation control
│   │   │   ├── swarm_task_alloc/ # CBBA task allocation
│   │   │   └── swarm_comm/       # Mesh comm, consensus
│   │   └── platforms/            # Pixhawk, Cube, CUAV targets
│   ├── companion/          # ROS 2 Humble companion software
│   │   ├── ai_perception/      # YOLOv8n, landing, fault detection
│   │   ├── swarm_formation/    # Formation ROS 2 nodes
│   │   ├── swarm_task_alloc/   # CBBA ROS 2 nodes
│   │   ├── swarm_comm/         # Mesh, consensus, merge protocol
│   │   ├── swarm_safety/       # Companion safety monitors
│   │   ├── swarm_interface/    # Intent parser, mission uploader, telemetry
│   │   └── swarm_hal/          # Hardware abstraction
│   └── ground/             # Ground station software
│       ├── gcs/                # QGC plugins (C++/QML + Python backend)
│       │   ├── swarm_mission_planner/
│       │   ├── swarm_monitor/
│       │   └── post_flight_analyzer/
│       └── dashboard/          # React/TypeScript web dashboard
├── tests/                  # Unit & integration tests
├── contracts/              # MAVLink/ROS 2 message definitions
├── docs/                   # Documentation
│   ├── icd/                  # Interface Control Documents
│   └── onboarding.md         # This file
├── simulation/             # Gazebo SITL, scenarios
├── tools/                  # Build, flash, log analysis tools
├── .github/workflows/      # CI/CD pipelines
├── docker-compose.yml      # Full stack local development
├── Dockerfile              # Production image
├── nginx.conf              # Production reverse proxy
├── Makefile                # Build orchestration
└── README.md               # Project overview
```

---

## Build & Test

### Firmware Build
```bash
# PX4 SITL (Simulation)
cd /my-drone-project/src/firmware
make px4_sitl gazebo-classic

# PX4 Hardware (Pixhawk 6C)
make px4_fmu-v6c_default upload

# PX4 Hardware (Cube Orange+)
make cubepilot_cubeorange_plus upload
```

### Companion Build
```bash
cd ~/swarm_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
# Or with tests
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
ctest --output-on-failure
```

### Web Dashboard Build
```bash
cd /my-drone-project/src/ground/dashboard
npm run build        # Production build to dist/
npm run preview      # Preview production build
```

### Run All Tests
```bash
# Unit tests
cd /my-drone-project
./run_tests.sh --unit

# Integration tests (requires SITL)
./run_tests.sh --integration

# Specific test suites
cd ~/swarm_ws
colcon test --packages-select swarm_safety swarm_formation swarm_task_alloc
```

---

## Development Workflow

### Git Workflow
```bash
# Feature branch
git checkout -b feature/your-feature-name

# Commit with conventional commits
git commit -m "feat: add new safety monitor for battery"

# Push and create PR
git push origin feature/your-feature-name
# Create PR on GitHub
```

### Commit Message Convention
```
<type>(<scope>): <subject>

<body>

<footer>
```

| Type | Description |
|------|-------------|
| feat | New feature |
| fix | Bug fix |
| docs | Documentation only |
| style | Formatting, no code change |
| refactor | Code restructuring |
| test | Adding tests |
| chore | Maintenance |

### Code Review Checklist
- [ ] Tests pass (unit + integration)
- [ ] Code follows style guide (clang-format, black, prettier)
- [ ] No new linting errors
- [ ] Documentation updated
- [ ] ADR updated if architectural change
- [ ] Tasks.md updated

### Continuous Integration
- GitHub Actions: `.github/workflows/ci.yml`
- Runs on: push to main/develop, PRs
- Checks: lint, type-check, unit tests, build
- Deploy: Docker image to GHCR on main branch

---

## Architecture Overview

### System Layers
```
┌─────────────────────────────────────────────────────────────┐
│                    GROUND STATION                           │
│  QGC Plugins (C++/QML) + Web Dashboard (React/TS)          │
└──────────────────────────┬──────────────────────────────────┘
                           │ MAVLink 2.0 + Custom Swarm Msgs
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                   COMPANION COMPUTER                        │
│  ROS 2 Humble (C++20/Python)                               │
│  ┌─────────┐ ┌────────────┐ ┌──────────┐ ┌─────────────┐   │
│  │ AI Perc │ │ Swarm Form │ │Task Alloc│ │ Swarm Comm  │   │
│  └─────────┘ └────────────┘ └──────────┘ └─────────────┘   │
│  ┌─────────┐ ┌────────────┐ ┌──────────┐ ┌─────────────┐   │
│  │Safety   │ │  Mesh Mgr  │ │ Consensus│ │ Store Fwd   │   │
│  └─────────┘ └────────────┘ └──────────┘ └─────────────┘   │
└──────────────────────────┬──────────────────────────────────┘
                           │ MAVLink 2.0 + uORB Bridge
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                    FIRMWARE (PX4)                           │
│  PX4 C++17 on Pixhawk/Cube/CUAV                            │
│  ┌──────────┐ ┌───────────┐ ┌──────────┐ ┌────────────┐    │
│  │Swarm Safe│ │Swarm Form │ │Task Alloc│ │Swarm Comm  │    │
│  └──────────┘ └───────────┘ └──────────┘ └────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Communication Stack
| Layer | Protocol | Frequency | Purpose |
|-------|----------|-----------|---------|
| Physical | Wi-Fi 6E / 5G / LoRa | - | Physical transport |
| Link | MAVLink 2.0 | 10-100Hz | Command/Telemetry |
| Network | DDS/RTPS (ROS 2) | 10-50Hz | Companion internal |
| Application | Custom Swarm Messages | 1-100Hz | Swarm coordination |

### Key Data Flows
1. **Telemetry**: Firmware → Companion → QGC/Web (10-50Hz)
2. **Commands**: GCS → Companion → Firmware (1-10Hz)
3. **Swarm Coordination**: Inter-drone mesh (10-50Hz)
4. **AI Inference**: Companion → Firmware (10-30Hz)
5. **Logging**: Dual ULog + rosbag2 with time sync (ADR-007)

---

## Key Components

### Firmware Modules (`src/firmware/modules/`)
| Module | Purpose | Safety Level |
|--------|---------|--------------|
| `swarm_safety` | Geofence, RID, Estop, LeaderFail | DAL A |
| `swarm_formation` | Leader-follower, formation geometry | DAL B |
| `swarm_task_alloc` | CBBA consensus | DAL C |
| `swarm_comm` | Mesh, MAVLink bridge | DAL B |

### Companion Modules (`src/companion/`)
| Package | Purpose | Language |
|---------|---------|----------|
| `ai_perception` | YOLOv8n, landing, fault detection | C++/Python |
| `swarm_formation` | Formation ROS 2 nodes | C++ |
| `swarm_task_alloc` | CBBA consensus | C++ |
| `swarm_comm/mesh_manager` | Wi-Fi 6E/5G/LoRa management | C++ |
| `swarm_comm/consensus` | CBBA, merge protocol | C++ |
| `swarm_safety` | Companion monitors | C++ |
| `swarm_interface` | Intent, mission, telemetry | C++/Python |

### Ground Station
| Component | Technology | Purpose |
|-----------|------------|---------|
| QGC Plugins | C++/QML + Python | Flight ops, planning, monitoring |
| Web Dashboard | React/TS/Vite | Analytics, replay, mobile access |
| Python Backend | FastAPI + JSON-RPC | Log analysis, scripting |

---

## Testing Strategy

### Test Pyramid
```
                    ▲
                   / \
                  / E2E \        ← 5% (SITL/HIL scenarios)
                 /-------\
                / Integration \    ← 15% (ROS 2 node integration)
               /---------------\
              /    Unit Tests   \    ← 80% (Module-level)
             /___________________\
```

### Test Categories
| Type | Location | Coverage Target |
|------|----------|-----------------|
| Unit | `tests/unit/` | ≥ 90% branch |
| Integration | `tests/integration/` | Key interfaces |
| Contract | `tests/integration/contract/` | MAVLink/ROS 2 |
| SITL | `tests/integration/sitl/` | Scenarios 1-8 |
| HIL | `tests/integration/hil/` | Hardware validation |

### Running Tests
```bash
# All tests
./run_tests.sh

# Specific category
./run_tests.sh --unit
./run_tests.sh --integration
./run_tests.sh --contract
./run_tests.sh --sitl

# Coverage report
./run_tests.sh --coverage
# Open coverage/index.html
```

### Test Data
- Mock data generators: `tests/mocks/`
- Reference logs: `tests/data/reference_logs/`
- SITL scenarios: `simulation/sitl/scenarios/`

---

## Deployment

### Development
```bash
# Full stack with docker-compose
cd /my-drone-project
docker-compose up -d
# Dashboard: http://localhost:3000
# Backend API: http://localhost:8080
```

### Production
```bash
# Build images
docker-compose -f docker-compose.yml -f docker-compose.prod.yml build

# Deploy to edge (Jetson Orin)
scp -r dist/ jetson@192.168.1.100:/opt/swarm/
ssh jetson@192.168.1.100 "cd /opt/swarm && docker-compose up -d"

# Firmware flash
./tools/flash_firmware.sh --target pixhawk6c --port /dev/ttyACM0
```

### Configuration Management
- Parameters: `config/*.yaml` + parameter server
- Secrets: `.env` (never commit!)
- Profiles: `dev`, `staging`, `prod` in docker-compose

---

## Troubleshooting

### Common Issues

| Issue | Symptoms | Solution |
|-------|----------|----------|
| ROS 2 not finding packages | `colcon build` fails | `source /opt/ros/humble/setup.bash && colcon build` |
| PX4 SITL won't start | Gazebo errors | `export GZ_VERSION=harmonic && make clean && make px4_sitl` |
| TensorRT not found | AI inference fails | Install TensorRT 8.6+ or use ONNX fallback |
| Mapbox not loading | Blank map | Check `VITE_MAPBOX_TOKEN` in `.env` |
| ROS 2 domain mismatch | No topics visible | `export ROS_DOMAIN_ID=42` (same on all nodes) |
| MAVLink timeout | No telemetry | Check serial/USB connection, baud rate 921600 |

### Debugging Tips
```bash
# ROS 2 debugging
ros2 topic list
ros2 topic echo /swarm/agent_states
ros2 node info /swarm_formation_node

# PX4 debugging
mavlink_shell.py /dev/ttyACM0
px4_log.py log_file.ulg

# Web dashboard
npm run dev -- --debug
# Check browser dev tools (Network, Console)

# Logs
tail -f ~/swarm_ws/log/latest/*.log
journalctl -u swarm-dashboard -f
```

---

## Resources

### Documentation
- [Project README](../README.md)
- [Implementation Summary](../IMPLEMENTATION_SUMMARY.md)
- [API Documentation](../docs/api/)
- [ICD Documentation](../docs/icd/icd.html)
- [Architecture Decision Records](../.specify/memory/adr/)

### External References
- [PX4 Developer Guide](https://docs.px4.io/)
- [ROS 2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [TensorRT Developer Guide](https://docs.nvidia.com/deeplearning/tensorrt/)
- [ONNX Runtime](https://onnxruntime.ai/)
- [GTSAM Documentation](https://gtsam.org/)
- [MAVLink 2.0 Protocol](https://mavlink.io/en/)
- [ROS 2 DDS/RTPS](https://docs.ros.org/en/humble/Concepts/About-DDS.html)

### Internal Specifications
- [Requirements Spec](.specify/specs/001-ai-swarm-autonomous-drones/spec.md)
- [Implementation Plan](.specify/specs/001-ai-swarm-autonomous-drones/plan.md)
- [Task Tracking](.specify/specs/001-ai-swarm-autonomous-drones/tasks.md)
- [Research Decisions](.specify/memory/research.md)
- [Architecture Decisions](.specify/memory/adr/)

### Community
- GitHub Issues: https://github.com/krill0188/my-drone-project/issues
- Discussions: https://github.com/krill0188/my-drone-project/discussions
- Wiki: https://github.com/krill0188/my-drone-project/wiki

---

## Next Steps for New Developers

1. **Week 1**: Complete environment setup, run all tests, explore codebase
2. **Week 2**: Pick a "good first issue" from GitHub, implement + test
3. **Week 3**: Review architecture docs (ADRs), understand data flows
4. **Week 4**: Contribute to a feature, go through PR process

Welcome to the team! 🚁