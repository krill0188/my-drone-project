#!/usr/bin/env python3
# Contract Test: AIInference ROS 2 Topic

import unittest
import rclpy
from rclpy.node import Node
from swarm_msgs.msg import AIInference

class TestAIInferenceRos2(unittest.TestCase):
    """Contract tests for AIInference ROS 2 topic"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_ai_inference')
        cls.received_messages = []
        
        cls.sub = cls.node.create_subscription(
            AIInference,
            '/swarm/ai_inference',
            cls.message_callback,
            10
        )
    
    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_subscription(cls.sub)
        cls.node.destroy_node()
        rclpy.shutdown()
    
    @classmethod
    def message_callback(cls, msg):
        cls.received_messages.append(msg)
    
    def setUp(self):
        self.received_messages.clear()
    
    def test_ai_inference_message_fields(self):
        """Test AIInference message has all required fields"""
        msg = AIInference()
        msg.timestamp_us = 1000000
        msg.drone_id = 1
        msg.type = 0  # OBSTACLE
        msg.class_id = 1
        msg.class_name = "power_line"
        msg.confidence = 0.95
        msg.position_enu = [100.0, 10.0, 5.0]
        msg.size = [5.0, 2.0, 3.0]
        msg.orientation = [1.0, 0.0, 0.0, 0.0]
        
        # Metadata
        msg.metadata.latency_ms = 15
        msg.metadata.model_name = "yolov8n_obstacle"
        msg.metadata.model_version = "v1.0"
        msg.metadata.runtime = "tensorrt_fp16"
        
        # ObstacleMeta
        msg.metadata.obstacle.threat_level = 3  # CRITICAL
        msg.metadata.obstacle.relative_velocity = [2.0, -1.0, 0.0]
        msg.metadata.obstacle.time_to_collision_s = 2.5
        msg.metadata.obstacle.requires_immediate_avoidance = True
        
        # LandingMeta (unused for obstacle)
        msg.metadata.landing.slope_deg = 0.0
        msg.metadata.landing.clearance_radius_m = 0.0
        msg.metadata.landing.surface_type = 0
        msg.metadata.landing.firmness_score = 0.0
        
        # FaultMeta (unused for obstacle)
        msg.metadata.fault.fault_type = 0
        msg.metadata.fault.severity = 0
        msg.metadata.fault.affected_component = ""
        msg.metadata.fault.degradation_pct = 0.0
        msg.metadata.fault.requires_replan = False
        msg.metadata.fault.recommended_action = 0
        
        # Verify all fields accessible
        self.assertEqual(msg.drone_id, 1)
        self.assertEqual(msg.type, 0)
        self.assertEqual(msg.class_id, 1)
        self.assertEqual(msg.class_name, "power_line")
        self.assertEqual(msg.confidence, 0.95)
        self.assertEqual(len(msg.position_enu), 3)
        self.assertEqual(len(msg.size), 3)
        self.assertEqual(len(msg.orientation), 4)
        self.assertEqual(msg.metadata.latency_ms, 15)
        self.assertEqual(msg.metadata.model_name, "yolov8n_obstacle")
        self.assertEqual(msg.metadata.obstacle.threat_level, 3)
    
    def test_ai_inference_type_enum(self):
        """Test AIInference type enum values"""
        # 0=OBSTACLE, 1=LANDING_SITE, 2=FAULT, 3=ANOMALY
        msg = AIInference()
        
        # Test all valid types
        for t in [0, 1, 2, 3]:
            msg.type = t
            self.assertEqual(msg.type, t)
    
    def test_ai_inference_obstacle_threat_levels(self):
        """Test obstacle threat level enum"""
        msg = AIInference()
        msg.type = 0  # OBSTACLE
        
        # 0=LOW, 1=MEDIUM, 2=HIGH, 3=CRITICAL
        for threat in [0, 1, 2, 3]:
            msg.metadata.obstacle.threat_level = threat
            self.assertEqual(msg.metadata.obstacle.threat_level, threat)
    
    def test_ai_inference_landing_surface_types(self):
        """Test landing surface type enum"""
        msg = AIInference()
        msg.type = 1  # LANDING_SITE
        
        # 0=GRASS, 1=CONCRETE, 2=DIRT, 3=SAND, 4=WATER, 5=UNKNOWN
        for surface in [0, 1, 2, 3, 4, 5]:
            msg.metadata.landing.surface_type = surface
            self.assertEqual(msg.metadata.landing.surface_type, surface)
    
    def test_ai_inference_fault_types(self):
        """Test fault type enum"""
        msg = AIInference()
        msg.type = 2  # FAULT
        
        # 0=IMU_DRIFT, 1=GPS_SPOOF, 2=MOTOR_DEGRADE, 3=BATTERY_IMBALANCE, 4=SENSOR_FAULT, 5=COMM_DEGRADE
        for fault in [0, 1, 2, 3, 4, 5]:
            msg.metadata.fault.fault_type = fault
            self.assertEqual(msg.metadata.fault.fault_type, fault)
    
    def test_ai_inference_fault_severity(self):
        """Test fault severity enum"""
        msg = AIInference()
        msg.type = 2  # FAULT
        
        # 0=INFO, 1=WARNING, 2=CRITICAL, 3=EMERGENCY
        for sev in [0, 1, 2, 3]:
            msg.metadata.fault.severity = sev
            self.assertEqual(msg.metadata.fault.severity, sev)
    
    def test_ai_inference_recommended_action(self):
        """Test recommended action enum"""
        msg = AIInference()
        msg.type = 2  # FAULT
        
        # 0=CONTINUE, 1=DEGRADED, 2=RTL, 3=EMERGENCY_LAND
        for action in [0, 1, 2, 3]:
            msg.metadata.fault.recommended_action = action
            self.assertEqual(msg.metadata.fault.recommended_action, action)
    
    def test_ai_inference_confidence_bounds(self):
        """Test confidence is in valid range [0, 1]"""
        msg = AIInference()
        
        # Valid confidences
        for conf in [0.0, 0.5, 0.95, 1.0]:
            msg.confidence = conf
            self.assertGreaterEqual(msg.confidence, 0.0)
            self.assertLessEqual(msg.confidence, 1.0)
    
    def test_ai_inference_position_enu(self):
        """Test position_enu is 3D vector"""
        msg = AIInference()
        
        # Valid 3D position
        msg.position_enu = [100.0, 200.0, 50.0]
        self.assertEqual(len(msg.position_enu), 3)
        
        msg.position_enu = [-50.0, -100.0, 10.0]
        self.assertEqual(msg.position_enu[0], -50.0)
        self.assertEqual(msg.position_enu[1], -100.0)
        self.assertEqual(msg.position_enu[2], 10.0)
    
    def test_ai_inference_bounding_box(self):
        """Test 3D bounding box size and orientation"""
        msg = AIInference()
        msg.type = 0  # OBSTACLE
        
        # Size (dx, dy, dz)
        msg.size = [10.0, 5.0, 3.0]
        self.assertEqual(len(msg.size), 3)
        self.assertEqual(msg.size[0], 10.0)
        self.assertEqual(msg.size[1], 5.0)
        self.assertEqual(msg.size[2], 3.0)
        
        # Orientation quaternion (w, x, y, z)
        msg.orientation = [1.0, 0.0, 0.0, 0.0]
        self.assertEqual(len(msg.orientation), 4)
        
        msg.orientation = [0.707, 0.0, 0.707, 0.0]  # 90 deg yaw
        self.assertAlmostEqual(msg.orientation[0]**2 + msg.orientation[1]**2 + 
                              msg.orientation[2]**2 + msg.orientation[3]**2, 1.0, places=3)
    
    def test_ai_inference_obstacle_meta(self):
        """Test obstacle metadata fields"""
        msg = AIInference()
        msg.type = 0  # OBSTACLE
        
        # Threat level
        msg.metadata.obstacle.threat_level = 3  # CRITICAL
        self.assertEqual(msg.metadata.obstacle.threat_level, 3)
        
        # Relative velocity
        msg.metadata.obstacle.relative_velocity = [2.0, -1.0, 0.0]
        self.assertEqual(msg.metadata.obstacle.relative_velocity[0], 2.0)
        
        # Time to collision
        msg.metadata.obstacle.time_to_collision_s = 2.5
        self.assertAlmostEqual(msg.metadata.obstacle.time_to_collision_s, 2.5)
        
        # Immediate avoidance flag
        msg.metadata.obstacle.requires_immediate_avoidance = True
        self.assertTrue(msg.metadata.obstacle.requires_immediate_avoidance)
    
    def test_ai_inference_landing_meta(self):
        """Test landing site metadata"""
        msg = AIInference()
        msg.type = 1  # LANDING_SITE
        
        # Slope
        msg.metadata.landing.slope_deg = 5.0
        self.assertAlmostEqual(msg.metadata.landing.slope_deg, 5.0)
        
        # Clearance radius
        msg.metadata.landing.clearance_radius_m = 3.0
        self.assertAlmostEqual(msg.metadata.landing.clearance_radius_m, 3.0)
        
        # Surface type
        msg.metadata.landing.surface_type = 0  # GRASS
        self.assertEqual(msg.metadata.landing.surface_type, 0)
        
        # Firmness
        msg.metadata.landing.firmness_score = 0.8
        self.assertAlmostEqual(msg.metadata.landing.firmness_score, 0.8)
    
    def test_ai_inference_fault_meta(self):
        """Test fault detection metadata"""
        msg = AIInference()
        msg.type = 2  # FAULT
        
        # Fault type
        msg.metadata.fault.fault_type = 2  # MOTOR_DEGRADE
        self.assertEqual(msg.metadata.fault.fault_type, 2)
        
        # Severity
        msg.metadata.fault.severity = 2  # CRITICAL
        self.assertEqual(msg.metadata.fault.severity, 2)
        
        # Affected component
        msg.metadata.fault.affected_component = "motor_3"
        self.assertEqual(msg.metadata.fault.affected_component, "motor_3")
        
        # Degradation percentage
        msg.metadata.fault.degradation_pct = 0.3
        self.assertAlmostEqual(msg.metadata.fault.degradation_pct, 0.3)
        
        # Requires replan
        msg.metadata.fault.requires_replan = True
        self.assertTrue(msg.metadata.fault.requires_replan)
        
        # Recommended action
        msg.metadata.fault.recommended_action = 2  # RTL
        self.assertEqual(msg.metadata.fault.recommended_action, 2)

if __name__ == '__main__':
    unittest.main()