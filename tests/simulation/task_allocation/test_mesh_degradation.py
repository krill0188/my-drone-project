#!/usr/bin/env python3
# SITL Integration Test: Mesh Degradation Store-and-Forward

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import CommQoS, TaskAllocation
import time
import numpy as np
import sys

class MeshDegradationTest(Node):
    def __init__(self, drone_count=5, packet_loss_threshold=0.3, max_store_forward_delay=10.0):
        super().__init__('mesh_degradation_test')
        
        self._drone_count = drone_count
        self._packet_loss_threshold = packet_loss_threshold
        self._max_store_forward_delay = max_store_forward_delay
        
        self._comm_qos = {}
        self._task_allocations = {}
        self._link_degraded = False
        self._degradation_start_time = None
        self._store_forward_messages = []
        self._recovery_time = None
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._comm_qos_sub = self.create_subscription(
            CommQoS, '/swarm/comm_qos', self.commQoSCallback, 10)
        
        self._task_alloc_sub = self.create_subscription(
            TaskAllocation, '/swarm/task_allocation', self.taskAllocCallback, 10)
        
        # Would need a service to inject packet loss
        # self._degrade_client = self.create_client(DegradeLink, '/mesh/degrade_link')
        
        self._timer = self.create_timer(1.0, self.checkProgress)
        
        self.get_logger().info(f'Mesh Degradation Test started: {drone_count} drones, '
                              f'loss_threshold={packet_loss_threshold*100:.0f}%, '
                              f'max_sf_delay={max_store_forward_delay}s')
    
    def commQoSCallback(self, msg):
        key = (msg.src_id, msg.dst_id)
        self._comm_qos[key] = msg
        
        # Check for degradation
        if msg.packet_loss_pct > self._packet_loss_threshold and not self._link_degraded:
            self._link_degraded = True
            self._degradation_start_time = time.time()
            self.get_logger().warning(f'Link degraded: {msg.src_id}->{msg.dst_id} loss={msg.packet_loss_pct*100:.1f}%')
        
        # Check for store-and-forward
        if msg.status == 3:  # STORE_FORWARD
            self._store_forward_messages.append({
                'src': msg.src_id,
                'dst': msg.dst_id,
                'time': time.time(),
                'delayed': False
            })
    
    def taskAllocCallback(self, msg):
        self._task_allocations[msg.drone_id] = msg
        
        # Check if delayed messages are delivered
        for sf_msg in self._store_forward_messages:
            if not sf_msg['delayed'] and msg.drone_id == sf_msg['dst']:
                sf_msg['delayed'] = True
                sf_msg['delivery_time'] = time.time()
                delay = sf_msg['delivery_time'] - sf_msg['time']
                self.get_logger().info(f'Store-forward delivered: {sf_msg["src"]}->{sf_msg["dst"]} delay={delay:.1f}s')
    
    def checkProgress(self):
        # Simulate link degradation after 30 seconds
        if not self._link_degraded and time.time() > 30:
            self.injectDegradation()
        
        # Check recovery
        if self._link_degraded and self._degradation_start_time:
            elapsed = time.time() - self._degradation_start_time
            
            # Check if all store-forward messages delivered
            all_delivered = all(sf['delayed'] for sf in self._store_forward_messages) if self._store_forward_messages else True
            
            if all_delivered and self._recovery_time is None:
                self._recovery_time = time.time()
                self.get_logger().info(f'Mesh recovered after {self._recovery_time - self._degradation_start_time:.1f}s')
                self.evaluateResults()
                rclpy.shutdown()
                return
            
            # Timeout
            if elapsed > 60.0:
                self.get_logger().error('Mesh recovery timeout')
                self.evaluateResults()
                rclpy.shutdown()
                return
    
    def injectDegradation(self):
        self.get_logger().info('Injecting mesh degradation (packet loss > 30%)...')
        # In real implementation, would call mesh degradation service
        # For now, simulate by setting a flag
        self._link_degraded = True
        self._degradation_start_time = time.time()
        
        # Simulate store-forward queue building up
        for i in range(10):
            self._store_forward_messages.append({
                'src': 1,
                'dst': 3,
                'time': time.time() + i * 0.5,
                'delayed': False
            })
    
    def evaluateResults(self):
        recovery_delay = self._recovery_time - self._degradation_start_time if self._recovery_time else None
        
        self.get_logger().info(f'=== Mesh Degradation Test Results ===')
        self.get_logger().info(f'Degradation injected: {self._link_degraded}')
        self.get_logger().info(f'Store-forward messages: {len(self._store_forward_messages)}')
        self.get_logger().info(f'Recovery time: {recovery_delay:.1f}s' if recovery_delay else 'Recovery time: N/A')
        
        passed = True
        if not self._link_degraded:
            self.get_logger().error('FAIL: Degradation not injected')
            passed = False
        if recovery_delay and recovery_delay > 10.0:
            self.get_logger().error(f'FAIL: Recovery delay {recovery_delay:.1f}s > 10s')
            passed = False
        if len(self._store_forward_messages) > 0:
            all_delivered = all(sf.get('delayed', False) for sf in self._store_forward_messages)
            if not all_delivered:
                self.get_logger().error('FAIL: Not all store-forward messages delivered')
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
    parser.add_argument('--packet-loss-threshold', type=float, default=0.3)
    parser.add_argument('--max-store-forward-delay', type=float, default=10.0)
    parsed_args = parser.parse_args()
    
    test = MeshDegradationTest(
        drone_count=parsed_args.drones,
        packet_loss_threshold=parsed_args.packet_loss_threshold,
        max_store_forward_delay=parsed_args.max_store_forward_delay
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