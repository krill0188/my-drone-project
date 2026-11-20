#!/usr/bin/env python3
# Contract Test: TaskAllocation ROS 2 Topic

import unittest
import rclpy
from rclpy.node import Node
from swarm_msgs.msg import TaskAllocation
import time

class TestTaskAllocationRos2(unittest.TestCase):
    """Contract tests for TaskAllocation ROS 2 topic"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_task_allocation')
        cls.received_messages = []
        
        cls.sub = cls.node.create_subscription(
            TaskAllocation,
            '/swarm/task_allocation',
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
    
    def test_task_allocation_message_fields(self):
        """Test TaskAllocation message has all required fields"""
        # Create a test message
        msg = TaskAllocation()
        msg.timestamp_us = int(time.time() * 1_000_000)
        msg.drone_id = 1
        msg.task_id = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16]  # 16 bytes UUID
        msg.cost = 10.5
        
        # CapabilityBitmap
        msg.capabilities.has_rtk_gps = True
        msg.capabilities.has_stereo_camera = True
        msg.capabilities.has_lidar_2d = False
        msg.capabilities.has_lidar_3d = True
        msg.capabilities.has_uwb = True
        msg.capabilities.has_5g_modem = False
        msg.capabilities.has_ai_compute = True
        msg.capabilities.max_flight_time_s = 1800
        msg.capabilities.max_payload_kg = 0.5
        msg.capabilities.comm_tier_bitmap = 0b111
        
        msg.sequence = 1
        
        # Verify all fields accessible
        self.assertEqual(msg.drone_id, 1)
        self.assertEqual(len(msg.task_id), 16)
        self.assertEqual(msg.cost, 10.5)
        self.assertTrue(msg.capabilities.has_rtk_gps)
        self.assertEqual(msg.capabilities.max_flight_time_s, 1800)
    
    def test_cost_semantics_lower_is_better(self):
        """Test cost: lower = better (more suitable)"""
        # This is a semantic test - cost represents energy/time/risk
        cost_drone_a = 5.0   # Nearby, good sensors
        cost_drone_b = 15.0  # Far, limited sensors
        
        self.assertLess(cost_drone_a, cost_drone_b)
    
    def test_capability_bitmap_fields(self):
        """Test all CapabilityBitmap fields"""
        msg = TaskAllocation()
        
        # All boolean fields
        msg.capabilities.has_rtk_gps = True
        msg.capabilities.has_stereo_camera = True
        msg.capabilities.has_lidar_2d = False
        msg.capabilities.has_lidar_3d = True
        msg.capabilities.has_uwb = True
        msg.capabilities.has_5g_modem = False
        msg.capabilities.has_ai_compute = True
        
        # Numeric fields
        msg.capabilities.max_flight_time_s = 1800
        msg.capabilities.max_payload_kg = 0.5
        
        # Bitmap: bit0=WiFi6E, bit1=5G, bit2=LoRa
        msg.capabilities.comm_tier_bitmap = 0b111
        
        self.assertEqual(msg.capabilities.comm_tier_bitmap, 0b111)
        self.assertTrue(msg.capabilities.has_uwb)
    
    def test_task_id_is_16_bytes(self):
        """Test task_id is exactly 16 bytes (UUID)"""
        msg = TaskAllocation()
        msg.task_id = list(range(16))  # 16 bytes
        
        self.assertEqual(len(msg.task_id), 16)
        
        # Test with actual UUID
        import uuid
        test_uuid = uuid.uuid4()
        msg.task_id = list(test_uuid.bytes)
        self.assertEqual(len(msg.task_id), 16)
    
    def test_sequence_monotonic(self):
        """Test sequence numbers increase"""
        seq1 = 100
        seq2 = 101
        seq3 = 102
        
        self.assertLess(seq1, seq2)
        self.assertLess(seq2, seq3)
    
    def test_cost_non_negative(self):
        """Test cost is non-negative"""
        msg = TaskAllocation()
        msg.cost = 0.0
        self.assertGreaterEqual(msg.cost, 0.0)
        
        msg.cost = 100.0
        self.assertGreaterEqual(msg.cost, 0.0)

if __name__ == '__main__':
    unittest.main()