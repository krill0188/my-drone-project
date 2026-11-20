#!/usr/bin/env python3
# SITL Integration Test: Geofence Breach → RTL < 500ms

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import AgentState, SafetyEvent
from geometry_msgs.msg import PoseStamped
import time
import numpy as np
import sys
import threading

class GeofenceBreachTest(Node):
    def __init__(self, drone_id=1, max_rtl_latency_ms=500, geofence_radius_m=100.0):
        super().__init__('geofence_breach_test')
        
        self._drone_id = drone_id
        self._max_latency_ms = max_rtl_latency_ms
        self._geofence_radius = geofence_radius_m
        
        self._agent_state = None
        self._breach_detected = False
        self._rtl_triggered = False
        self._breach_time = None
        self._rtl_time = None
        self._rtl_latency_ms = None
        self._test_complete = False
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to agent state
        self._agent_state_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStateCallback, qos)
        
        # Subscribe to safety events
        self._safety_event_sub = self.create_subscription(
            SafetyEvent, '/swarm/safety_events', self.safetyEventCallback, qos)
        
        # Timer for test progression
        self._timer = self.create_timer(0.02, self.checkProgress)  # 50 Hz
        
        self.get_logger().info(f'Geofence Breach Test: drone {drone_id}, '
                              f'max_latency={max_rtl_latency_ms}ms, '
                              f'geofence_radius={geofence_radius_m}m')
    
    def agentStateCallback(self, msg):
        if msg.sysid == self._drone_id:
            self._agent_state = msg
    
    def safetyEventCallback(self, msg):
        if msg.source_id == self._drone_id:
            if msg.type == 0:  # GEOFENCE_BREACH
                if not self._breach_detected:
                    self._breach_detected = True
                    self._breach_time = time.time()
                    self.get_logger().info(f'Geofence breach detected at {msg.position_enu[0]:.1f}, {msg.position_enu[1]:.1f}')
            elif msg.type == 1:  # EMERGENCY_STOP (RTL triggered)
                if not self._rtl_triggered:
                    self._rtl_triggered = True
                    self._rtl_time = time.time()
                    if self._breach_time:
                        self._rtl_latency_ms = (self._rtl_time - self._breach_time) * 1000
                        self.get_logger().info(f'RTL triggered, latency: {self._rtl_latency_ms:.1f}ms')
    
    def checkProgress(self):
        if not self._agent_state:
            return
        
        # Get current position
        pos = np.array([self._agent_state.position.x, 
                       self._agent_state.position.y, 
                       self._agent_state.position.z])
        distance_from_origin = np.sqrt(pos[0]**2 + pos[1]**2)
        
        # Start test: fly to geofence boundary + 50m
        if not self._breach_detected and distance_from_origin < self._geofence_radius + 50:
            # Would command drone to fly outward
            # In real test, this would be a trajectory command
            pass
        
        # Check RTL latency
        if self._rtl_triggered and not self._test_complete:
            self.evaluateResults()
            return
        
        # Timeout
        if self._breach_detected and not self._rtl_triggered:
            elapsed = (time.time() - self._breach_time) * 1000
            if elapsed > 5000:  # 5 second timeout
                self.get_logger().error('RTL not triggered within timeout')
                self.evaluateResults()
                return
        
        # Overall timeout
        if time.time() > self._start_time + 60:
            self.get_logger().error('Test timeout')
            self.evaluateResults()
    
    def evaluateResults(self):
        self._test_complete = True
        self.get_logger().info('=== Geofence Breach Test Results ===')
        
        passed = True
        
        # Check breach detected
        if self._breach_detected:
            self.get_logger().info('PASS: Geofence breach detected')
        else:
            self.get_logger().error('FAIL: Geofence breach not detected')
            passed = False
        
        # Check RTL triggered
        if self._rtl_triggered:
            self.get_logger().info('PASS: RTL triggered')
        else:
            self.get_logger().error('FAIL: RTL not triggered')
            passed = False
        
        # Check latency
        if self._rtl_latency_ms is not None:
            self.get_logger().info(f'RTL Latency: {self._rtl_latency_ms:.1f}ms (target: <{self._max_latency_ms}ms)')
            if self._rtl_latency_ms <= self._max_latency_ms:
                self.get_logger().info('PASS: RTL latency within limit')
            else:
                self.get_logger().error(f'FAIL: RTL latency {self._rtl_latency_ms:.1f}ms exceeds {self._max_latency_ms}ms')
                passed = False
        else:
            self.get_logger().error('FAIL: RTL latency not measured')
            passed = False
        
        self.get_logger().info(f'Overall: {"PASS" if passed else "FAIL"}')
        rclpy.shutdown()
        sys.exit(0 if passed else 1)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone-id', type=int, default=1)
    parser.add_argument('--max-latency-ms', type=int, default=500)
    parser.add_argument('--geofence-radius', type=float, default=100.0)
    parsed_args = parser.parse_args()
    
    test = GeofenceBreachTest(
        drone_id=parsed_args.drone_id,
        max_rtl_latency_ms=parsed_args.max_latency_ms,
        geofence_radius_m=parsed_args.geofence_radius
    )
    test._start_time = time.time()
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()