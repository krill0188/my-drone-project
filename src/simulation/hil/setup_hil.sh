#!/usr/bin/env bash
# HIL Bridge Configuration Script

set -euo pipefail

PROJECT_ROOT="/my-drone-project"
HIL_DIR="$PROJECT_ROOT/src/simulation/hil"

usage() {
    cat << EOF
Usage: $0 [OPTIONS]
Options:
  --setup          Configure HIL environment (udev rules, MAVLink router)
  --connect FC     Connect to flight controller (px4, cube, cuav)
  --bridge         Start MAVLink bridge (FC <-> Companion <-> GCS)
  --test           Run HIL connectivity test
  -h, --help       Show this help
EOF
}

setup_udev() {
    echo "Setting up udev rules for flight controllers..."
    
    cat > /etc/udev/rules.d/99-px4.rules << 'UDEV_EOF'
# Pixhawk 6C / Cube Orange+ / CUAV Nora
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", MODE="0666", GROUP="dialout", SYMLINK+="px4"
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0013", MODE="0666", GROUP="dialout", SYMLINK+="cube"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", MODE="0666", GROUP="dialout", SYMLINK+="cuav"

# Generic USB serial for telemetry
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666", GROUP="dialout", SYMLINK+="telem%n"
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", GROUP="dialout", SYMLINK+="telem%n"

# USB GPS
SUBSYSTEM=="tty", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a7", MODE="0666", GROUP="dialout", SYMLINK+="gps%n"
UDEV_EOF

    udevadm control --reload-rules
    udevadm trigger
    echo "Udev rules installed"
}

setup_mavlink_router() {
    echo "Configuring MAVLink router..."
    
    mkdir -p /etc/mavlink-router
    cat > /etc/mavlink-router/main.conf << 'ROUTER_EOF'
[General]
TcpServerPort=5760
ReportStats=true
MavlinkDialect=auto

[UartEndpoint px4]
Device=/dev/px4
Baud=921600
FlowControl=false

[UartEndpoint cube]
Device=/dev/cube
Baud=921600
FlowControl=false

[UartEndpoint cuav]
Device=/dev/cuav
Baud=921600
FlowControl=false

[UartEndpoint telem]
Device=/dev/telem0
Baud=57600
FlowControl=false

[UdpEndpoint gcs]
Mode=Normal
Address=0.0.0.0
Port=14550

[UdpEndpoint companion]
Mode=Normal
Address=127.0.0.1
Port=14540

[UdpEndpoint simulation]
Mode=Normal
Address=127.0.0.1
Port=14560
ROUTER_EOF

    echo "MAVLink router config written to /etc/mavlink-router/main.conf"
}

connect_fc() {
    local fc_type=$1
    case $fc_type in
        px4)
            echo "Connecting to Pixhawk 6C on /dev/px4..."
            screen /dev/px4 921600
            ;;
        cube)
            echo "Connecting to Cube Orange+ on /dev/cube..."
            screen /dev/cube 921600
            ;;
        cuav)
            echo "Connecting to CUAV Nora on /dev/cuav..."
            screen /dev/cuav 921600
            ;;
        *)
            echo "Unknown FC type: $fc_type"
            echo "Supported: px4, cube, cuav"
            exit 1
            ;;
    esac
}

start_bridge() {
    echo "Starting MAVLink router..."
    mavlink-routerd -c /etc/mavlink-router/main.conf
}

test_hil() {
    echo "Testing HIL connectivity..."
    
    # Check devices
    for dev in /dev/px4 /dev/cube /dev/cuav; do
        if [ -e "$dev" ]; then
            echo "✓ Found $dev"
        else
            echo "✗ Missing $dev"
        fi
    done
    
    # Check MAVLink router
    if command -v mavlink-routerd &> /dev/null; then
        echo "✓ mavlink-routerd installed"
    else
        echo "✗ mavlink-routerd not installed"
    fi
    
    # Check MAVROS
    if ros2 pkg prefix mavros &> /dev/null; then
        echo "✓ MAVROS installed"
    else
        echo "✗ MAVROS not installed"
    fi
    
    # Check PX4 SITL
    if [ -f "$HOME/PX4-Autopilot/build/px4_sitl_default/bin/px4" ]; then
        echo "✓ PX4 SITL built"
    else
        echo "✗ PX4 SITL not built"
    fi
}

main() {
    case ${1:-} in
        --setup)
            setup_udev
            setup_mavlink_router
            ;;
        --connect)
            connect_fc ${2:-px4}
            ;;
        --bridge)
            start_bridge
            ;;
        --test)
            test_hil
            ;;
        -h|--help)
            usage
            ;;
        *)
            usage
            exit 1
            ;;
    esac
}

main "$@"