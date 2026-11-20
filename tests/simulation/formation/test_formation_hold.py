#!/usr/bin/env python3
# SITL Integration Test: Formation Hold

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import FormationState, AgentState
import time
import numpy as np
import sys

class FormationHoldTest(Node):
    def __init__(self, drone_count=4, max_lateral_error=0.5, max_vertical_error=0.3):
        super().__init__('formation_hold_test')
        
        self._drone_count = drone_count
        self._max_lateral_error = max_lateral_error
        self._max_vertical_error = max_vertical_error
        
        self._formation_state = None
        self._errors_lateral = []
        self._errors_vertical = []
        self._waypoints_reached = 0
        self._start_time = time.time()
        self._timeout = 300  # 5 minutes
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._formation_sub = self.create_subscription(
            FormationState, '/swarm/formation_state', self.formation_callback, qos)
        
        self._timer = self.create_timer(1.0, self.check_progress)
        
        self.get_logger().info(f'Formation Hold Test started: {drone_count} drones, '
                              f'max_lat_err={max_lateral_error}m, max_vert_err={max_vertical_error}m')
    
    def formation_callback(self, msg):
        self._formation_state = msg
        
        # Collect position errors
        for err in msg.position_errors_flu:
            lateral = np.sqrt(err.x**2 + err.y**2)
            vertical = abs(err.z)
            self._errors_lateral.append(lateral)
            self._errors_vertical.append(vertical)
        
        # Check if waypoint reached (simplified)
        if msg.formation_state == 1:  # HOLDING
            self._waypoints_reached += 1
    
    def check_progress(self):
        if self._formation_state is None:
            return
        
        elapsed = time.time() - self._start_time
        
        if elapsed > self._timeout:
            self.get_logger().error('Test timeout')
            self.evaluate_results()
            rclpy.shutdown()
            return
        
        # Check if all waypoints completed (simplified)
        if self._waypoints_reached >= 10:
            self.get_logger().info('All waypoints reached')
            self.evaluate_results()
            rclpy.shutdown()
    
    def evaluate_results(self):
        if not self._errors_lateral:
            self.get_logger().error('No formation data collected')
            sys.exit(1)
        
        mean_lateral = np.mean(self._errors_lateral)
        max_lateral = np.max(self._errors_lateral)
        mean_vertical = np.mean(self._errors_vertical)
        max_vertical = np.max(self._errors_vertical)
        
        self.get_logger().info(f'Results:')
        self.get_logger().info(f'  Lateral error: mean={mean_lateral:.3f}m, max={max_lateral:.3f}m')
        self.get_logger().info(f'  Vertical error: mean={mean_vertical:.3f}m, max={max_vertical:.3f}m')
        
        passed = True
        if mean_lateral > self._max_lateral_error:
            self.get_logger().error(f'FAIL: Mean lateral error {mean_lateral:.3f}m > {self._max_lateral_error}m')
            passed = False
        if max_lateral > self._max_lateral_error * 2:
            self.get_logger().error(f'FAIL: Max lateral error {max_lateral:.3f}m > {self._max_lateral_error * 2}m')
            passed = False
        if mean_vertical > self._max_vertical_error:
            self.get_logger().error(f'FAIL: Mean vertical error {mean_vertical:.3f}m > {self._max_vertical_error}m')
            passed = False
        
        if passed:
            self.get_logger().info('PASS: Formation hold within tolerances')
            sys.exit(0)
        else:
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    
    # Parse args
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', type=int, default=4)
    parser.add_argument('--max-lateral-error', type=float, default=0.5)
    parser.add_argument('--max-vertical-error', type=float, default=0.3)
    parsed_args = parser.parse_args()
    
    test = FormationHoldTest(
        drone_count=parsed_args.drones,
        max_lateral_error=parsed_args.max_lateral_error,
        max_vertical_error=parsed_args.max_vertical_error
    )
    
    try:
        rclpy.spin(test)
    except KeyboardInterrupt:
        pass
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()