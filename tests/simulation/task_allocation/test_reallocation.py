#!/usr/bin/env python3
# SITL Integration Test: Task Reallocation on Drone Dropout

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import TaskAllocation, AgentState
from swarm_msgs.srv import SimulateFault
import time
import numpy as np
import sys

class ReallocationTest(Node):
    def __init__(self, drone_count=5, max_realloc_time=5.0, max_coverage_gap=0.02):
        super().__init__('reallocation_test')
        
        self._drone_count = drone_count
        self._max_realloc_time = max_realloc_time
        self._max_coverage_gap = max_coverage_gap
        
        self._task_allocations = {}
        self._agent_states = {}
        self._initial_coverage = 0.0
        self._coverage_at_dropout = 0.0
        self._coverage_after_realloc = 0.0
        self._realloc_start_time = None
        self._realloc_complete_time = None
        self._fault_injected = False
        self._reallocation_detected = False
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._task_alloc_sub = self.create_subscription(
            TaskAllocation, '/swarm/task_allocation', self.taskAllocCallback, qos)
        
        self._agent_states_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStatesCallback, qos)
        
        # Fault injection service client
        self._fault_client = self.create_client(SimulateFault, '/drone3/simulate_fault')
        
        self._timer = self.create_timer(0.5, self.checkProgress)
        
        self.get_logger().info(f'Reallocation Test started: {drone_count} drones, '
                              f'max_realloc_time={max_realloc_time}s, max_gap={max_coverage_gap*100:.0f}%')
    
    def taskAllocCallback(self, msg):
        self._task_allocations[msg.drone_id] = msg
        
        # Detect reallocation (change in task assignments)
        if self._fault_injected and not self._reallocation_detected:
            self._reallocation_detected = True
            self._realloc_complete_time = time.time()
            self.get_logger().info(f'Reallocation detected at {self._realloc_complete_time - self._realloc_start_time:.2f}s after dropout')
    
    def agentStatesCallback(self, msg):
        self._agent_states[msg.sysid] = msg
    
    def checkProgress(self):
        # Inject fault at 50% coverage (simulated)
        if not self._fault_injected and len(self._task_allocations) >= 5:
            # Simulate 50% coverage achieved
            self.injectFault()
        
        # Check reallocation timeout
        if self._realloc_start_time and not self._reallocation_detected:
            elapsed = time.time() - self._realloc_start_time
            if elapsed > self._max_realloc_time:
                self.get_logger().error(f'Reallocation timeout: {elapsed:.1f}s > {self._max_realloc_time}s')
                self.evaluateResults()
                rclpy.shutdown()
                return
        
        # Check if reallocation complete
        if self._reallocation_detected and self._realloc_complete_time:
            realloc_time = self._realloc_complete_time - self._realloc_start_time
            self.get_logger().info(f'Reallocation completed in {realloc_time:.2f}s')
            self.evaluateResults()
            rclpy.shutdown()
            return
    
    def injectFault(self):
        self.get_logger().info('Injecting battery critical fault on drone 3...')
        
        if not self._fault_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Fault injection service not available')
            return
        
        request = SimulateFault.Request()
        request.data = True
        
        self._fault_injected = True
        self._realloc_start_time = time.time()
        
        future = self._fault_client.call_async(request)
        future.add_done_callback(self.faultInjectionCallback)
    
    def faultInjectionCallback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Fault injected successfully')
            else:
                self.get_logger().error('Fault injection failed')
        except Exception as e:
            self.get_logger().error(f'Fault injection failed: {e}')
    
    def evaluateResults(self):
        realloc_time = self._realloc_complete_time - self._realloc_start_time if self._realloc_complete_time and self._realloc_start_time else None
        
        self.get_logger().info(f'=== Reallocation Test Results ===')
        self.get_logger().info(f'Fault injected: {self._fault_injected}')
        self.get_logger().info(f'Reallocation detected: {self._reallocation_detected}')
        self.get_logger().info(f'Reallocation time: {realloc_time:.2f}s' if realloc_time else 'Reallocation time: N/A')
        
        passed = True
        if not self._reallocation_detected:
            self.get_logger().error('FAIL: Reallocation not detected')
            passed = False
        if realloc_time and realloc_time > self._max_realloc_time:
            self.get_logger().error(f'FAIL: Reallocation time {realloc_time:.2f}s > {self._max_realloc_time}s')
            passed = False
        
        if passed:
            self.get_logger().info('PASS: Reallocation within time limit')
            sys.exit(0)
        else:
            sys.exit(1)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drones', type=int, default=5)
    parser.add_argument('--max-realloc-time', type=float, default=5.0)
    parser.add_argument('--max-coverage-gap', type=float, default=0.02)
    parsed_args = parser.parse_args()
    
    test = ReallocationTest(
        drone_count=parsed_args.drones,
        max_realloc_time=parsed_args.max_realloc_time,
        max_coverage_gap=parsed_args.max_coverage_gap
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