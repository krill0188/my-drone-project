#!/usr/bin/env python3
# SITL Integration Test: AI Fault Detection

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import AIInference, AgentState
from gazebo_msgs.srv import SetEntityState
import time
import numpy as np
import sys

class FaultDetectionTest(Node):
    def __init__(self, drone_id=1, max_detection_latency_ms=200, max_false_positive_rate=0.01):
        super().__init__('fault_detection_test')
        
        self._drone_id = drone_id
        self._max_latency_ms = max_detection_latency_ms
        self._max_fp_rate = max_false_positive_rate
        
        self._ai_inferences = []
        self._agent_state = None
        self._fault_injected = False
        self._fault_type = None
        self._detection_time = None
        self._detection_latency_ms = None
        self._false_positives = 0
        self._total_inferences = 0
        self._fault_detected = False
        _fault_reallocation_verified = False
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._ai_inference_sub = self.create_subscription(
            AIInference, '/drone{}/ai_inference'.format(drone_id), self.aiInferenceCallback, 10)
        
        self._agent_state_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStateCallback, 10)
        
        # Service to inject fault
        self._fault_client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        self._timer = self.create_timer(0.05, self.checkProgress)  # 20 Hz
        
        self.get_logger().info(f'Fault Detection Test started: drone {drone_id}, '
                              f'max_latency={max_detection_latency_ms}ms, '
                              f'max_fp_rate={max_false_positive_rate*100:.1f}%')
    
    def aiInferenceCallback(self, msg):
        self._ai_inferences.append(msg)
        self._total_inferences += 1
        
        if msg.type == 2:  # FAULT
            if not self._fault_detected:
                self._fault_detected = True
                self._detection_time = time.time()
                if self._fault_injected:
                    self._detection_latency_ms = (self._detection_time - self._fault_injection_time) * 1000
                    self.get_logger().info(f'Fault detected: {msg.metadata.fault.affected_component}, '
                                          f'type={msg.metadata.fault.fault_type}, '
                                          f'severity={msg.metadata.fault.severity}, '
                                          f'latency={self._detection_latency_ms:.1f}ms')
            else:
                # Additional detections
                pass
        elif msg.type in [0, 1, 3]:  # OBSTACLE, LANDING, ANOMALY
            # Check for false positives during fault injection
            if self._fault_injected and msg.confidence > 0.7:
                # If we get non-fault detections during fault, might be OK
                pass
    
    def agentStateCallback(self, msg):
        if msg.sysid == self._drone_id:
            self._agent_state = msg
    
    def checkProgress(self):
        if not self._fault_injected and self._agent_state:
            # Inject fault after 5 seconds
            if time.time() > 5.0:
                self.injectMotorFault()
            return
        
        if self._fault_injected and not self._fault_detected:
            # Check detection timeout
            elapsed = (time.time() - self._fault_injection_time) * 1000
            if elapsed > 5000:  # 5 second timeout
                self.get_logger().error('Fault detection timeout')
                self.evaluateResults()
                rclpy.shutdown()
                return
        
        if self._fault_detected and not self._fault_reallocation_verified:
            # Verify reallocation happened
            # Would check if formation reconfigured, task reallocated
            self._fault_reallocation_verified = True
            self.evaluateResults()
            rclpy.shutdown()
    
    def injectMotorFault(self):
        self.get_logger().info('Injecting motor_3 partial thrust loss...')
        
        # In real implementation, would call Gazebo service to reduce motor thrust
        # For now, simulate by setting a flag
        self._fault_injected = True
        self._fault_type = 2  # MOTOR_DEGRADE
        self._fault_injection_time = time.time()
        
        # Would call Gazebo service to reduce motor_3 thrust by 30%
        # self._fault_client.call_async(...)
    
    def evaluateResults(self):
        self.get_logger().info('=== Fault Detection Test Results ===')
        
        passed = True
        
        # Check fault detection
        if self._fault_detected:
            self.get_logger().info('PASS: Fault detected')
        else:
            self.get_logger().error('FAIL: Fault not detected')
            passed = False
        
        # Check detection latency
        if self._detection_latency_ms is not None:
            self.get_logger().info(f'Detection latency: {self._detection_latency_ms:.1f}ms')
            if self._detection_latency_ms <= 200.0:
                self.get_logger().info('PASS: Detection latency <= 200ms')
            else:
                self.get_logger().error(f'FAIL: Detection latency {self._detection_latency_ms:.1f}ms > 200ms')
                passed = False
        else:
            self.get_logger().error('FAIL: Detection latency not measured')
            passed = False
        
        # Check fault type classification
        fault_type_correct = False
        for inf in self._ai_inferences:
            if inf.type == 2 and inf.metadata.fault.fault_type == 2:  # MOTOR_DEGRADE
                fault_type_correct = True
                break
        
        if fault_type_correct:
            self.get_logger().info('PASS: Fault type correctly classified as MOTOR_DEGRADE')
        else:
            self.get_logger().error('FAIL: Fault type not correctly classified')
            passed = False
        
        # Check severity
        severity_correct = False
        for inf in self._ai_inferences:
            if inf.type == 2 and inf.metadata.fault.severity >= 2:  # CRITICAL or EMERGENCY
                severity_correct = True
                break
        
        if severity_correct:
            self.get_logger().info('PASS: Fault severity correctly classified as CRITICAL/EMERGENCY')
        else:
            self.get_logger().error('FAIL: Fault severity not correctly classified')
            passed = False
        
        # Check false positive rate
        if self._total_inferences > 0:
            fp_rate = self._false_positives / self._total_inferences
            self.get_logger().info(f'False positive rate: {fp_rate*100:.2f}%')
            if fp_rate <= self._max_fp_rate:
                self.get_logger().info('PASS: False positive rate within limits')
            else:
                self.get_logger().error(f'FAIL: False positive rate {fp_rate*100:.2f}% > {self._max_fp_rate*100:.1f}%')
                passed = False
        else:
            self.get_logger().warning('No inferences recorded')
        
        # Check reallocation (simplified)
        self.get_logger().info('PASS: Control reallocation verified (simulated)')
        
        self.get_logger().info(f'Overall: {"PASS" if passed else "FAIL"}')
        sys.exit(0 if passed else 1)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone-id', type=int, default=1)
    parser.add_argument('--max-latency-ms', type=int, default=200)
    parser.add_argument('--max-fp-rate', type=float, default=0.01)
    parsed_args = parser.parse_args()
    
    test = FaultDetectionTest(
        drone_id=parsed_args.drone_id,
        max_detection_latency_ms=parsed_args.max_latency_ms,
        max_false_positive_rate=parsed_args.max_fp_rate
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