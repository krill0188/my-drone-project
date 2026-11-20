#!/usr/bin/env python3
# SITL Integration Test: Dynamic Obstacle Broadcast and Replan

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from swarm_msgs.msg import AIInference, AgentState, FormationState
from gazebo_msgs.srv import SpawnEntity
import time
import numpy as np
import sys

class ObstacleBroadcastTest(Node):
    def __init__(self, drone_count=5, max_replan_time=1.0, min_separation=5.0):
        super().__init__('obstacle_broadcast_test')
        
        self._drone_count = drone_count
        self._max_replan_time = max_replan_time
        self._min_separation = min_separation
        
        self._ai_inferences = []
        self._agent_states = {}
        self._formation_state = None
        self._obstacle_spawned = False
        self._replan_start_time = None
        self._replan_complete = False
        self._replan_time = None
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self._ai_inference_sub = self.create_subscription(
            AIInference, '/swarm/ai_inference', self.aiInferenceCallback, qos)
        
        self._agent_states_sub = self.create_subscription(
            AgentState, '/swarm/agent_states', self.agentStatesCallback, qos)
        
        self._formation_state_sub = self.create_subscription(
            FormationState, '/swarm/formation_state', self.formationStateCallback, qos)
        
        # Gazebo spawn service
        self._spawn_client = self.create_client(SpawnEntity, '/gazebo/spawn_entity')
        
        self._timer = self.create_timer(0.1, self.checkProgress)
        
        self.get_logger().info(f'Obstacle Broadcast Test started: {drone_count} drones, '
                              f'max_replan_time={max_replan_time}s, min_separation={min_separation}m')
    
    def aiInferenceCallback(self, msg):
        if msg.type == 0:  # OBSTACLE
            self._ai_inferences.append(msg)
    
    def agentStatesCallback(self, msg):
        self._agent_states[msg.sysid] = msg
    
    def formationStateCallback(self, msg):
        self._formation_state = msg
    
    def checkProgress(self):
        # Spawn obstacle at waypoint 5 equivalent position
        if not self._obstacle_spawned and len(self._agent_states) >= 5:
            self.spawnObstacle()
        
        # Check replan propagation
        if self._obstacle_spawned and not self._replan_complete:
            if self._replan_start_time is None and len(self._ai_inferences) > 0:
                # First AI inference after obstacle spawn
                self._replan_start_time = time.time()
                self.get_logger().info('First obstacle detection - replan timer started')
            
            if self._replan_start_time:
                # Check if formation has adjusted (formation state shows adjustment)
                if self._formation_state and self._formation_state.formation_state == 1:  # HOLDING
                    # Check if all followers have adjusted
                    all_adjusted = True
                    for err in self._formation_state.position_errors_flu:
                        lateral = np.sqrt(err.x**2 + err.y**2)
                        if lateral > 2.0:  # Still adjusting
                            all_adjusted = False
                            break
                    
                    if all_adjusted:
                        self._replan_complete = True
                        self._replan_time = time.time() - self._replan_start_time
                        self.get_logger().info(f'Formation adjusted in {self._replan_time:.3f}s')
                        self.evaluateResults()
                        rclpy.shutdown()
                        return
                
                # Timeout check
                if self._replan_start_time and (time.time() - self._replan_start_time) > 5.0:
                    self.get_logger().error('Replan timeout')
                    self.evaluateResults()
                    rclpy.shutdown()
                    return
    
    def spawnObstacle(self):
        self.get_logger().info('Spawning dynamic obstacle (bird flock) at waypoint 5...')
        
        if not self._spawn_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Gazebo spawn service not available')
            return
        
        # Spawn a model representing bird flock
        from gazebo_msgs.msg import Model
        from geometry_msgs.msg import Pose
        
        request = SpawnEntity.Request()
        request.name = 'bird_flock_obstacle'
        request.xml = '''<sdf version="1.6">
  <model name="bird_flock">
    <pose>100 50 10 0 0 0</pose>
    <static>false</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>5.0</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>5.0</radius>
          </sphere>
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
                self.get_logger().info('Obstacle spawned successfully')
            else:
                self.get_logger().error('Obstacle spawn failed')
        except Exception as e:
            self.get_logger().error(f'Spawn failed: {e}')
    
    def evaluateResults(self):
        self.get_logger().info(f'=== Obstacle Broadcast Test Results ===')
        self.get_logger().info(f'Obstacle spawned: {self._obstacle_spawned}')
        self.get_logger().info(f'Replan triggered: {self._replan_start_time is not None}')
        self.get_logger().info(f'Replan propagation time: {self._replan_time:.3f}s' if self._replan_time else 'Replan time: N/A')
        
        passed = True
        if not self._obstacle_spawned:
            self.get_logger().error('FAIL: Obstacle not spawned')
            passed = False
        if self._replan_start_time is None:
            self.get_logger().error('FAIL: Replan not triggered')
            passed = False
        if self._replan_time and self._replan_time > 1.0:
            self.get_logger().error(f'FAIL: Replan time {self._replan_time:.3f}s > 1.0s')
            passed = False
        
        # Check separation maintenance
        if self._formation_state:
            for err in self._formation_state.position_errors_flu:
                lateral = np.sqrt(err.x**2 + err.y**2)
                if lateral > self._min_separation:
                    self.get_logger().error(f'FAIL: Separation violated: {lateral:.2f}m > {self._min_separation}m')
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
    parser.add_argument('--max-replan-time', type=float, default=1.0)
    parser.add_argument('--min-separation', type=float, default=5.0)
    parsed_args = parser.parse_args()
    
    test = ObstacleBroadcastTest(
        drone_count=parsed_args.drones,
        max_replan_time=parsed_args.max_replan_time,
        min_separation=parsed_args.min_separation
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