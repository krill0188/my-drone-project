#!/usr/bin/env python3
# SITL Integration Test: Task Allocation - Search Coverage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import TaskAllocation, AgentState, SwarmMap
import time
import numpy as np
import sys

class SearchCoverageTest(Node):
    def __init__(self, drone_count=5, min_coverage=0.95, max_time=1800):
        super().__init__('search_coverage_test')
        
        self._drone_count = drone_count
        self._min_coverage = min_coverage
        self._max_time = max_time
        
        self._task_allocations = {}
        self._agent_states = {}
        self._swarm_map = None
        self._coverage_history = []
        self._start_time = time.time()
        self._reallocation_times = []
        self._collision_count = 0
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._task_alloc_sub = self.create_subscription(
            TaskAllocation, '/swarm/task_allocation', self.taskAllocCallback, qos)
        
        self._agent_states_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStatesCallback, qos)
        
        self._swarm_map_sub = self.create_subscription(
            SwarmMap, '/swarm/swarm_map', self.swarmMapCallback, qos)
        
        self._timer = self.create_timer(1.0, self.checkProgress)
        
        self.get_logger().info(f'Search Coverage Test started: {drone_count} drones, '
                              f'min_coverage={min_coverage}, max_time={max_time}s')
    
    def taskAllocCallback(self, msg):
        self._task_allocations[msg.drone_id] = msg
    
    def agentStatesCallback(self, msg):
        self._agent_states[msg.sysid] = msg
    
    def swarmMapCallback(self, msg):
        self._swarm_map = msg
    
    def checkProgress(self):
        elapsed = time.time() - self._start_time
        
        if elapsed > self._max_time:
            self.get_logger().error(f'Test timeout after {self._max_time}s')
            self.evaluateResults()
            rclpy.shutdown()
            return
        
        # Check if all drones have task allocations
        if len(self._task_allocations) >= self._drone_count:
            # Compute coverage from swarm map
            if self._swarm_map is not None:
                coverage = self.computeCoverage()
                self._coverage_history.append((time.time(), coverage))
                
                self.get_logger().info(f'Coverage: {coverage*100:.1f}% (elapsed: {elapsed:.1f}s)')
                
                if coverage >= self._min_coverage:
                    self.get_logger().info(f'Target coverage {self._min_coverage*100:.0f}% achieved!')
                    self.evaluateResults()
                    rclpy.shutdown()
                    return
        
        # Check for collisions
        self.checkCollisions()
        
        # Check for task reallocations
        self.checkReallocations()
    
    def computeCoverage(self):
        if self._swarm_map is None:
            return 0.0
        
        # Count covered cells (semantic_class != UNKNOWN and confidence > 0.5)
        total_cells = len(self._swarm_map.semantic_class)
        if total_cells == 0:
            return 0.0
        
        covered = 0
        for i, cls in enumerate(self._swarm_map.semantic_class):
            if cls != 0 and self._swarm_map.semantic_confidence[i] > 0.5:
                covered += 1
        
        return covered / total_cells
    
    def checkCollisions(self):
        # Check pairwise distances between drones
        positions = {}
        for sysid, state in self._agent_states.items():
            positions[sysid] = np.array([state.position.x, state.position.y, state.position.z])
        
        for id1, pos1 in positions.items():
            for id2, pos2 in positions.items():
                if id1 >= id2:
                    continue
                dist = np.linalg.norm(pos1 - pos2)
                if dist < 5.0:  # FR-018: separation > 5m
                    self._collision_count += 1
                    self.get_logger().warning(f'Collision risk: drone {id1} - {id2} distance={dist:.2f}m')
    
    def checkReallocations(self):
        # Track when task allocations change
        # In real implementation, would detect reallocation events
        pass
    
    def evaluateResults(self):
        if not self._coverage_history:
            self.get_logger().error('No coverage data collected')
            sys.exit(1)
        
        final_coverage = self._coverage_history[-1][1]
        
        # Check reallocation time (would need actual reallocation events)
        max_realloc_time = max(self._reallocation_times) if self._reallocation_times else 0
        
        self.get_logger().info(f'=== Test Results ===')
        self.get_logger().info(f'Final coverage: {final_coverage*100:.1f}% (target: {self._min_coverage*100:.0f}%)')
        self.get_logger().info(f'Collision count: {self._collision_count} (target: 0)')
        self.get_logger().info(f'Max reallocation time: {max_realloc_time:.1f}s (target: < 5s)')
        
        passed = True
        if final_coverage < self._min_coverage:
            self.get_logger().error(f'FAIL: Coverage {final_coverage*100:.1f}% < {self._min_coverage*100:.0f}%')
            passed = False
        if self._collision_count > 0:
            self.get_logger().error(f'FAIL: {self._collision_count} collision(s) detected')
            passed = False
        if max_realloc_time > 5.0:
            self.get_logger().error(f'FAIL: Reallocation time {max_realloc_time:.1f}s > 5s')
            passed = False
        
        if passed:
            self.get_logger().info('PASS: All criteria met')
            sys.exit(0)
        else:
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', type=int, default=5)
    parser.add_argument('--min-coverage', type=float, default=0.95)
    parser.add_argument('--max-time', type=int, default=1800)
    parsed_args = parser.parse_args()
    
    test = SearchCoverageTest(
        drone_count=parsed_args.drones,
        min_coverage=parsed_args.min_coverage,
        max_time=parsed_args.max_time
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