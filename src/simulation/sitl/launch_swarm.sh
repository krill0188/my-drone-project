#!/usr/bin/env bash
# Launch SITL swarm with configurable drone count and formation

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"

DRONE_COUNT=4
FORMATION="vee"
LEADER_ID=0
AI_ENABLED=false
GPS_DENIED=false
UWB_ENABLED=false
RID_ENABLED=false
VOICE_ENABLED=false

usage() {
    cat << EOF
Usage: $0 [OPTIONS]
Options:
  --count N          Number of drones (default: 4)
  --formation TYPE   Formation geometry: line, vee, diamond, column, custom, none (default: vee)
  --leader ID        Leader drone sysid (default: 0)
  --ai-enabled       Enable AI perception nodes
  --gps-denied       Enable GPS-denied mode (VIO + UWB)
  --uwb-enabled      Enable UWB ranging
  --rid-enabled      Enable Remote ID broadcast
  --voice-enabled    Enable voice intent interface
  -h, --help         Show this help
EOF
}

while [[ $# -gt 0 ]]; do
    case $1 in
        --count) DRONE_COUNT="$2"; shift 2 ;;
        --formation) FORMATION="$2"; shift 2 ;;
        --leader) LEADER_ID="$2"; shift 2 ;;
        --ai-enabled) AI_ENABLED=true; shift ;;
        --gps-denied) GPS_DENIED=true; shift ;;
        --uwb-enabled) UWB_ENABLED=true; shift ;;
        --rid-enabled) RID_ENABLED=true; shift ;;
        --voice-enabled) VOICE_ENABLED=true; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown option: $1"; usage; exit 1 ;;
    esac
done

# Validate inputs
if [[ $DRONE_COUNT -lt 1 || $DRONE_COUNT -gt 50 ]]; then
    echo "Error: Drone count must be 1-50"
    exit 1
fi

VALID_FORMATIONS=("line" "vee" "diamond" "column" "custom" "none")
if [[ ! " ${VALID_FORMATIONS[@]} " =~ " ${FORMATION} " ]]; then
    echo "Error: Invalid formation. Must be one of: ${VALID_FORMATIONS[*]}"
    exit 1
fi

echo "=== Launching SITL Swarm ==="
echo "Drones: $DRONE_COUNT"
echo "Formation: $FORMATION"
echo "Leader: $LEADER_ID"
echo "AI Enabled: $AI_ENABLED"
echo "GPS Denied: $GPS_DENIED"
echo "UWB Enabled: $UWB_ENABLED"
echo "RID Enabled: $RID_ENABLED"
echo "Voice Enabled: $VOICE_ENABLED"

# Source ROS 2
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/src/companion/install/setup.bash" 2>/dev/null || true

# Set PX4 SITL environment
export PX4_SIM_MODEL=gazebo-classic
export PX4_SIM_SPEED_FACTOR=1
export GAZEBO_MODEL_PATH="$PROJECT_ROOT/src/simulation/sitl/models:$GAZEBO_MODEL_PATH"
export GAZEBO_PLUGIN_PATH="$PROJECT_ROOT/src/simulation/sitl/plugins:$GAZEBO_PLUGIN_PATH"

# Launch PX4 SITL instances
for ((i=0; i<DRONE_COUNT; i++)); do
    SYSID=$((i + 1))
    PORT=$((14540 + i * 10))
    
    echo "Starting PX4 SITL instance $SYSID on port $PORT..."
    
    # Create instance directory
    INSTANCE_DIR="/tmp/px4_sitl_$SYSID"
    mkdir -p "$INSTANCE_DIR"
    
    # Generate PX4 params for this drone
    cat > "$INSTANCE_DIR/params.txt" << PARAMS
SYS_AUTOSTART 4001
SYS_AUTOCONFIG 1
MAV_SYS_ID $SYSID
MAV_COMP_ID 1
MAV_TYPE 2
SIM_SPEED_FACTOR 1
SIM_GPS1_GPSMODE 3
SIM_GPS2_GPSMODE 3
EOF
    
    if [[ $FORMATION != "none" && $i == $LEADER_ID ]]; then
        echo "SWM_FORMATION_LEADER 1" >> "$INSTANCE_DIR/params.txt"
        echo "SWM_FORMATION_TYPE $(case $FORMATION in line) echo 0;; vee) echo 1;; diamond) echo 2;; column) echo 3;; custom) echo 4;; esac)" >> "$INSTANCE_DIR/params.txt"
    elif [[ $FORMATION != "none" ]]; then
        echo "SWM_FORMATION_LEADER 0" >> "$INSTANCE_DIR/params.txt"
        echo "SWM_FORMATION_SLOT $i" >> "$INSTANCE_DIR/params.txt"
    fi
    
    if [[ $RID_ENABLED == true ]]; then
        echo "RID_ENABLE 1" >> "$INSTANCE_DIR/params.txt"
        echo "RID_OPERATOR_ID \"SWARM_OP_001\"" >> "$INSTANCE_DIR/params.txt"
    fi
    
    # Launch PX4 SITL
    cd "$PROJECT_ROOT/Firmware" 2>/dev/null || cd "$HOME/PX4-Autopilot" 2>/dev/null || {
        echo "PX4 Firmware not found. Please set up PX4-Autopilot."
        exit 1
    }
    
    nohup ./build/px4_sitl_default/bin/px4 -i $SYSID -d "$INSTANCE_DIR" > "/tmp/px4_$SYSID.log" 2>&1 &
    PX4_PID=$!
    echo $PX4_PID > "/tmp/px4_$SYSID.pid"
    
    # Wait for MAVLink
    sleep 2
done

# Launch Gazebo
echo "Starting Gazebo Harmonic..."
cd "$PROJECT_ROOT/src/simulation/sitl"
nohup gz sim -v 4 -r swarm_world.sdf > "/tmp/gazebo.log" 2>&1 &
GAZEBO_PID=$!
echo $GAZEBO_PID > "/tmp/gazebo.pid"

# Wait for Gazebo
sleep 5

# Launch ROS 2 nodes
echo "Starting ROS 2 swarm nodes..."

# MAVLink router for multi-drone
ros2 run mavros mavros_node --ros-args -p fcu_url:="udp://:14540@127.0.0.1:14550" -p gcs_url:="" -p tgt_system:=1 -p tgt_component:=1 --remap __ns:=/drone1 &

# Swarm autonomy nodes
if [[ $FORMATION != "none" ]]; then
    ros2 run swarm_autonomy formation_state_node --ros-args -p drone_count:=$DRONE_COUNT -p leader_id:=$LEADER_ID --remap __ns:=/swarm &
    ros2 run swarm_autonomy leader_monitor --ros-args --remap __ns:=/swarm &
fi

if [[ $AI_ENABLED == true ]]; then
    ros2 run ai_perception obstacle_node --ros-args --remap __ns:=/swarm &
    ros2 run ai_perception landing_node --ros-args --remap __ns:=/swarm &
    ros2 run ai_perception fault_node --ros-args --remap __ns:=/swarm &
fi

if [[ $GPS_DENIED == true ]]; then
    ros2 run swarm_autonomy state_estimator_node --ros-args -p vio_enabled:=true -p uwb_enabled:=$UWB_ENABLED --remap __ns:=/swarm &
fi

ros2 run swarm_comm mesh_manager_node --ros-args --remap __ns:=/swarm &
ros2 run swarm_safety geofence_monitor_node --ros-args --remap __ns:=/swarm &
ros2 run swarm_safety rid_broadcaster_node --ros-args --remap __ns:=/swarm &
ros2 run swarm_safety emergency_stop_node --ros-args --remap __ns:=/swarm &
ros2 run swarm_interface telemetry_streamer_node --ros-args --remap __ns:=/swarm &

if [[ $VOICE_ENABLED == true ]]; then
    ros2 run swarm_interface intent_node --ros-args --remap __ns:=/swarm &
fi

echo "=== SITL Swarm Launched ==="
echo "PX4 logs: /tmp/px4_*.log"
echo "Gazebo log: /tmp/gazebo.log"
echo "ROS 2 topics: ros2 topic list"
echo ""
echo "To run a mission:"
echo "  ros2 run swarm_interface mission_uploader_node --mission <mission_file.yaml>"
echo ""
echo "To stop:"
echo "  $SCRIPT_DIR/stop_swarm.sh"

# Save launch config
cat > "/tmp/swarm_launch_config.json" << CONFIG
{
    "drone_count": $DRONE_COUNT,
    "formation": "$FORMATION",
    "leader_id": $LEADER_ID,
    "ai_enabled": $AI_ENABLED,
    "gps_denied": $GPS_DENIED,
    "uwb_enabled": $UWB_ENABLED,
    "rid_enabled": $RID_ENABLED,
    "voice_enabled": $VOICE_ENABLED,
    "timestamp": "$(date -Iseconds)"
}
CONFIG