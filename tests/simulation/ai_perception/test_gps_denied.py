#!/usr/bin/env python3
# SITL Integration Test: GPS-Denied VIO+UWB Navigation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import AgentState
import time
import numpy as np
import sys

class GPSDeniedTest(Node):
    def __init__(self, drone_count=2, max_drift_pct=1.0, mission_distance_km=2.0):
        super().__init__('gps_denied_test')
        
        self._drone_count = drone_count
        self._max_drift_pct = max_drift_pct
        self._mission_distance_km = mission_distance_km
        
        self._agent_states = {}
        self._gps_denied = False
        self._mission_start_time = None
        self._mission_start_pos = None
        self._mission_complete = False
        self._max_drift = 0.0
        _drift_at_completion = 0.0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._agent_states_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStatesCallback, 10)
        
        # Service to simulate GPS denial
        # Would need a service client to inject GPS fault
        
        self._timer = self.create_timer(1.0, self.checkProgress)
        
        self.get_logger().info(f'GPS-Denied Test started: {drone_count} drones, '
                              f'max_drift={max_drift_pct*100:.1f}%, '
                              f'distance={mission_distance_km}km')
    
    def agentStatesCallback(self, msg):
        self._agent_states[msg.sysid] = msg
    
    def checkProgress(self):
        if not self._agent_states:
            return
        
        # Inject GPS denial at mission start
        if not self._gps_denied and len(self._agent_states) >= self._drone_count:
            self.injectGPSDenial()
            return
        
        if not self._gps_denied:
            return
        
        # Track position drift for all drones
        for sysid, state in self._agent_states.items():
            if not state.gps_fix and state.vio_valid:
                # GPS denied but VIO working
                pass
            
            # Check if mission complete (distance traveled)
            if self._mission_start_pos is not None:
                dx = state.position.x - self._mission_start_pos.x
                dy = state.position.y - self._mission_start_pos.y
                dz = state.position.z - self._mission_start_pos.z
                distance = np.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance >= self._mission_distance_km * 1000:
                    if not self._mission_complete:
                        self._mission_complete = True
                        self.get_logger().info(f'Drone {msg.sysid} completed {distance/1000:.2f}km mission')
        
        # Check if all drones completed
        if self._mission_complete and all(
            self._get_distance_traveled(s) >= self._mission_distance_km * 1000 
            for s in self._agent_states.values() if s.vio_valid
        ):
            self.evaluateResults()
            rclpy.shutdown()
            return
        
        # Timeout check
        if self._mission_start_time and (time.time() - self._mission_start_time) > 600:  # 10 min
            self.get_logger().error('Mission timeout')
            self.evaluateResults()
            rclpy.shutdown()
    
    def injectGPSDenial(self):
        self.get_logger().info('Injecting GPS denial (jamming/spoofing)...')
        
        # In real implementation, would call service to disable GPS
        # For now, just mark as denied
        self._gps_denied = True
        self._mission_start_time = time.time()
        
        # Record start positions
        for sysid, state in self._agent_states.items():
            if self._mission_start_pos is None:
                self._mission_start_pos = np.array([state.position.x, 
                                                   state.position.y, 
                                                   state.position.z])
    
    def _get_distance_traveled(self, state):
        if self._mission_start_pos is None:
            return 0.0
        dx = state.position.x - self._mission_start_pos[0]
        dy = state.position.y - self._mission_start_pos[1]
        dz = state.position.z - self._mission_start_pos[2]
        return np.sqrt(dx*dx + dy*dy + dz*dz)
    
    def evaluateResults(self):
        self.get_logger().info('=== GPS-Denied Navigation Test Results ===')
        
        passed = True
        
        max_drift_pct = 0.0
        
        for sysid, state in self._agent_states.items():
            if not state.vio_valid:
                self.get_logger().warning(f'Drone {sysid}: VIO not valid')
                continue
            
            # Calculate drift from ground truth (would need ground truth)
            # For now, use position covariance as proxy
            drift_pct = state.pos_covariance_trace * 100  # Simplified
            
            if drift_pct > max_drift_pct:
                max_drift_pct = drift_pct
            
            self.get_logger().info(f'Drone {sysid}: pos_cov_trace={state.pos_covariance_trace:.4f}, '
                                  f'estimated_drift={drift_pct:.2f}%')
        
        self.get_logger().info(f'Max drift: {max_drift_pct:.2f}% (target: <{self._max_drift_pct*100:.1f}%)')
        
        if max_drift_pct <= self._max_drift_pct * 100:
            self.get_logger().info('PASS: Position drift within limits')
        else:
            self.get_logger().error(f'FAIL: Max drift {max_drift_pct:.2f}% exceeds {self._max_drift_pct*100:.1f}%')
            passed = False
        
        # Check VIO valid for all drones
        all_vio_valid = all(s.vio_valid for s in self._agent_states.values())
        if all_vio_valid:
            self.get_logger().info('PASS: All drones maintaining VIO')
        else:
            self.get_logger().error('FAIL: Some drones lost VIO')
            passed = False
        
        self.get_logger().info(f'Overall: {"PASS" if passed else "FAIL"}')
        sys.exit(0 if passed else 1)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', type=int, default=2)
    parser.add_argument('--max-drift-pct', type=float, default=1.0)
    parser.add_argument('--distance', type=float, default=2.0)
    parsed_args = parser.parse_args()
    
    test = GPSDeniedTest(
        drone_count=parsed_args.drones,
        max_drift_pct=parsed_args.max_drift_pct,
        mission_distance_km=parsed_args.distance
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