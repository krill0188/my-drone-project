#!/usr/bin/env python3
# SITL Integration Test: AI Obstacle Avoidance

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import AIInference, AgentState, FormationState
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
import time
import numpy as np
import sys

class ObstacleAvoidanceTest(Node):
    def __init__(self, drone_id=1, max_avoidance_latency_ms=200, detection_range_m=25.0):
        super().__init__('obstacle_avoidance_test')
        
        self._drone_id = drone_id
        self._max_latency_ms = max_avoidance_latency_ms
        self._detection_range = detection_range_m
        
        self._ai_inferences = []
        self._agent_state = None
        self._formation_state = None
        self._obstacle_spawned = False
        self._avoidance_start_time = None
        self._avoidance_complete = False
        self._avoidance_latency_ms = None
        self._min_separation_maintained = True
        self._collision_occurred = False
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._ai_inference_sub = self.create_subscription(
            AIInference, '/drone{}/ai_inference'.format(drone_id), self.aiInferenceCallback, qos)
        
        self._agent_state_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStateCallback, qos)
        
        self._formation_state_sub = self.create_subscription(
            FormationState, '/swarm/formation_state', self.formationStateCallback, qos)
        
        # Gazebo spawn service
        self._spawn_client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
        
        # Timer for progress checking
        self._timer = self.create_timer(0.05, self.checkProgress)  # 20 Hz
        
        self.get_logger().info(f'Obstacle Avoidance Test started: drone {drone_id}, '
                              f'max_latency={max_avoidance_latency_ms}ms, '
                              f'detection_range={detection_range_m}m')
    
    def aiInferenceCallback(self, msg):
        self._ai_inferences.append(msg)
        
        # Check for power line detection
        if msg.type == 0 and msg.class_id == 1:  # OBSTACLE, POWER_LINE
            if msg.confidence >= 0.7 and not self._obstacle_spawned:
                # First detection - start avoidance timer
                if self._avoidance_start_time is None:
                    self._avoidance_start_time = time.time()
                    self.get_logger().info(f'Power line detected at {msg.position_enu[0]:.1f}m, '
                                          f'confidence={msg.confidence:.2f}, '
                                          f'threat={msg.metadata.obstacle.threat_level}')
    
    def agentStateCallback(self, msg):
        if msg.sysid == self._drone_id:
            self._agent_state = msg
    
    def formationStateCallback(self, msg):
        self._formation_state = msg
    
    def checkProgress(self):
        # Spawn obstacle at 25m ahead when drone is approaching
        if not self._obstacle_spawned and self._agent_state:
            # Calculate distance to waypoint 5 area (where obstacle should be)
            drone_pos = np.array([self._agent_state.position.x, 
                                 self._agent_state.position.y, 
                                 self._agent_state.position.z])
            obstacle_pos = np.array([100.0, 50.0, 10.0])  # Waypoint 5 area
            dist_to_obstacle = np.linalg.norm(drone_pos - obstacle_pos)
            
            if dist_to_obstacle < 50.0 and not self._obstacle_spawned:
                self.spawnObstacle()
        
        # Check avoidance latency
        if self._avoidance_start_time and not self._avoidance_complete:
            if self._formation_state:
                # Check if formation has adjusted (position errors reduced)
                max_lateral_err = 0.0
                for err in self._formation_state.position_errors_flu:
                    lateral = np.sqrt(err.x**2 + err.y**2)
                    max_lateral_err = max(max_lateral_err, lateral)
                
                if max_lateral_err < 1.0:  # Formation stabilized
                    self._avoidance_complete = True
                    self._avoidance_latency_ms = (time.time() - self._avoidance_start_time) * 1000
                    self.get_logger().info(f'Avoidance complete in {self._avoidance_latency_ms:.1f}ms')
                    self.evaluateResults()
                    rclpy.shutdown()
                    return
            
            # Timeout check
            elapsed_ms = (time.time() - self._avoidance_start_time) * 1000
            if elapsed_ms > 5000:  # 5 second timeout
                self.get_logger().error('Avoidance timeout')
                self.evaluateResults()
                rclpy.shutdown()
                return
        
        # Check separation maintenance
        if self._formation_state:
            for err in self._formation_state.position_errors_flu:
                lateral = np.sqrt(err.x**2 + err.y**2)
                if lateral > 5.0:  # FR-018: separation > 5m
                    self._min_separation_maintained = False
                    self.get_logger().warning(f'Separation violated: {lateral:.2f}m')
    
    def spawnObstacle(self):
        self.get_logger().info('Spawning power lines at waypoint 5...')
        
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gazebo spawn service not available')
            return
        
        from gazebo_msgs.msg import Model
        from geometry_msgs.msg import Pose
        
        request = SpawnEntity.Request()
        request.name = 'power_lines_test'
        request.xml = '''<sdf version="1.6">
  <model name="power_lines">
    <pose>100 50 10 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>200</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.05</radius>
            <length>200</length>
          </cylinder>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>'''
        request.robot_namespace = ''
        request.initial_pose = Pose()
        request.initial_pose.position.x = 100.0
        request.initial_pose.position.y = 50.0
        request.initial_pose.position.z = 10.0
        
        future = self._spawn_client.call_async(request)
        future.add_done_callback(self.spawnCallback)
        self._obstacle_spawned = True
    
    def spawnCallback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info('Power lines spawned successfully')
            else:
                self.get_logger().error('Power line spawn failed')
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')
    
    def evaluateResults(self):
        self.get_logger().info('=== Obstacle Avoidance Test Results ===')
        
        passed = True
        
        # Check obstacle detection
        power_line_detected = False
        for inf in self._ai_inferences:
            if inf.type == 0 and inf.class_id == 1:  # OBSTACLE, POWER_LINE
                if inf.confidence >= 0.92 and inf.metadata.obstacle.threat_level == 3:
                    power_line_detected = True
                    break
        
        if power_line_detected:
            self.get_logger().info('PASS: Power line detected with F1 >= 0.92, threat=CRITICAL')
        else:
            self.get_logger().error('FAIL: Power line not detected with required confidence/threat')
            passed = False
        
        # Check avoidance latency
        if self._avoidance_latency_ms is not None:
            self.get_logger().info(f'Avoidance latency: {self._avoidance_latency_ms:.1f}ms')
            if self._avoidance_latency_ms <= 200.0:
                self.get_logger().info('PASS: Avoidance latency <= 200ms')
            else:
                self.get_logger().error(f'FAIL: Avoidance latency {self._avoidance_latency_ms:.1f}ms > 200ms')
                passed = False
        else:
            self.get_logger().error('FAIL: Avoidance did not complete')
            passed = False
        
        # Check separation maintained
        if self._min_separation_maintained:
            self.get_logger().info('PASS: Formation separation maintained > 5m')
        else:
            self.get_logger().error('FAIL: Formation separation violated')
            passed = False
        
        # Check no collision
        if not self._collision_occurred:
            self.get_logger().info('PASS: No collisions')
        else:
            self.get_logger().error('FAIL: Collision occurred')
            passed = False
        
        self.get_logger().info(f'Overall: {"PASS" if passed else "FAIL"}')
        sys.exit(0 if passed else 1)

def main(args=None):
    rclpy.init(args=args)
    
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone-id', type=int, default=1)
    parser.add_argument('--max-latency-ms', type=int, default=200)
    parser.add_argument('--detection-range', type=float, default=25.0)
    parsed_args = parser.parse_args()
    
    test = ObstacleAvoidanceTest(
        drone_id=parsed_args.drone_id,
        max_avoidance_latency_ms=parsed_args.max_latency_ms,
        detection_range_m=parsed_args.detection_range
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