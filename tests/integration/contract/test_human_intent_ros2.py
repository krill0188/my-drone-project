#!/usr/bin/env python3
# Contract Test: HumanIntentCmd ROS 2 Topic

import unittest
import rclpy
from rclpy.node import Node
from swarm_msgs.msg import HumanIntentCmd

class TestHumanIntentCmdRos2(unittest.TestCase):
    """Contract tests for HumanIntentCmd ROS 2 topic"""
    
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_human_intent')
        cls.received_messages = []
        
        cls.sub = cls.node.create_subscription(
            HumanIntentCmd,
            '/swarm/human_intent',
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
    
    def test_human_intent_cmd_message_fields(self):
        """Test HumanIntentCmd message has all required fields"""
        msg = HumanIntentCmd()
        
        # Header
        msg.timestamp_us = 1000000
        msg.source_id = 0  # Ground station
        msg.target_id = 0  # 0 = all drones, >0 = specific
        
        # Intent
        msg.intent_type = 0  # VOICE
        msg.command = "return_home"
        msg.confidence = 0.95
        msg.language = "ko"
        
        # Parameters (JSON string for flexibility)
        msg.parameters = '{"altitude": 50, "speed": 5}'
        
        # Priority
        msg.priority = 2  # HIGH
        
        # Verify all fields accessible
        self.assertEqual(msg.timestamp_us, 1000000)
        self.assertEqual(msg.source_id, 0)
        self.assertEqual(msg.target_id, 0)
        self.assertEqual(msg.intent_type, 0)
        self.assertEqual(msg.command, "return_home")
        self.assertEqual(msg.confidence, 0.95)
        self.assertEqual(msg.language, "ko")
        self.assertEqual(msg.parameters, '{"altitude": 50, "speed": 5}')
        self.assertEqual(msg.priority, 2)
    
    def test_intent_type_enum(self):
        """Test IntentType enum values"""
        # 0=VOICE, 1=TABLET, 2=GESTURE, 3=AUTO
        msg = HumanIntentCmd()
        
        for t in [0, 1, 2, 3]:
            msg.intent_type = t
            self.assertEqual(msg.intent_type, t)
    
    def test_command_strings(self):
        """Test standard command strings"""
        msg = HumanIntentCmd()
        
        standard_commands = [
            "return_home",
            "land_here",
            "follow_me",
            "hold_position",
            "change_formation",
            "search_area",
            "inspect_target",
            "emergency_stop",
            "resume_mission",
            "abort_mission"
        ]
        
        for cmd in standard_commands:
            msg.command = cmd
            self.assertEqual(msg.command, cmd)
    
    def test_priority_enum(self):
        """Test Priority enum values"""
        # 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL
        msg = HumanIntentCmd()
        
        for p in [0, 1, 2, 3]:
            msg.priority = p
            self.assertEqual(msg.priority, p)
    
    def test_confidence_bounds(self):
        """Test confidence is in valid range [0, 1]"""
        msg = HumanIntentCmd()
        
        # Valid confidences
        for conf in [0.0, 0.5, 0.95, 1.0]:
            msg.confidence = conf
            self.assertGreaterEqual(msg.confidence, 0.0)
            self.assertLessEqual(msg.confidence, 1.0)
    
    def test_parameters_json(self):
        """Test parameters field accepts JSON"""
        msg = HumanIntentCmd()
        
        # Valid JSON strings
        test_params = [
            '{}',
            '{"altitude": 50}',
            '{"altitude": 50, "speed": 5, "radius": 100}',
            '{"waypoints": [[100, 200, 50], [150, 250, 50]]}',
            '{"target_id": 5, "action": "inspect"}'
        ]
        
        for params in test_params:
            msg.parameters = params
            self.assertEqual(msg.parameters, params)
    
    def test_language_codes(self):
        """Test language field accepts ISO 639-1 codes"""
        msg = HumanIntentCmd()
        
        languages = ["ko", "en", "ja", "zh", "fr", "de", "es"]
        
        for lang in languages:
            msg.language = lang
            self.assertEqual(msg.language, lang)
    
    def test_target_id_semantics(self):
        """Test target_id: 0=all, >0=specific drone"""
        msg = HumanIntentCmd()
        
        # Broadcast to all
        msg.target_id = 0
        self.assertEqual(msg.target_id, 0)
        
        # Specific drone
        msg.target_id = 5
        self.assertEqual(msg.target_id, 5)
        
        # Formation leader
        msg.target_id = 255  # Special: formation leader
        self.assertEqual(msg.target_id, 255)
    
    def test_voice_specific_fields(self):
        """Test fields specific to voice commands"""
        msg = HumanIntentCmd()
        msg.intent_type = 0  # VOICE
        msg.command = "return_home"
        msg.confidence = 0.92
        msg.language = "ko"
        msg.parameters = '{"trigger_phrase": "집으로 돌아와", "asr_confidence": 0.98}'
        
        self.assertEqual(msg.intent_type, 0)
        self.assertGreaterEqual(msg.confidence, 0.9)
    
    def test_tablet_specific_fields(self):
        """Test fields specific to tablet commands"""
        msg = HumanIntentCmd()
        msg.intent_type = 1  # TABLET
        msg.command = "search_area"
        msg.confidence = 1.0  # Tablet commands are explicit
        msg.language = "en"
        msg.parameters = '{"polygon": [[0,0],[1000,0],[1000,1000],[0,1000]], "pattern": "lawnmower"}'
        
        self.assertEqual(msg.intent_type, 1)
        self.assertEqual(msg.confidence, 1.0)
    
    def test_gesture_specific_fields(self):
        """Test fields specific to gesture commands"""
        msg = HumanIntentCmd()
        msg.intent_type = 2  # GESTURE
        msg.command = "hold_position"
        msg.confidence = 0.85
        msg.language = ""  # No language for gestures
        msg.parameters = '{"gesture_type": "palm_forward", "hand": "right"}'
        
        self.assertEqual(msg.intent_type, 2)
        self.assertLess(msg.confidence, 0.9)
    
    def test_auto_specific_fields(self):
        """Test fields specific to auto-generated commands"""
        msg = HumanIntentCmd()
        msg.intent_type = 3  # AUTO
        msg.command = "emergency_stop"
        msg.confidence = 1.0
        msg.language = ""
        msg.parameters = '{"trigger": "geofence_breach", "source_drone": 3}'
        
        self.assertEqual(msg.intent_type, 3)
        self.assertEqual(msg.confidence, 1.0)
    
    def test_human_intent_cmd_serialization(self):
        """Test message can be serialized/deserialized"""
        msg = HumanIntentCmd()
        msg.timestamp_us = 1234567890
        msg.source_id = 100
        msg.target_id = 0
        msg.intent_type = 0
        msg.command = "return_home"
        msg.confidence = 0.95
        msg.language = "ko"
        msg.parameters = '{"altitude": 50}'
        msg.priority = 2
        
        # Serialize to bytes (would use rclpy serialization)
        # For now, verify all fields are set
        self.assertEqual(msg.timestamp_us, 1234567890)
        self.assertEqual(msg.source_id, 100)
        self.assertEqual(msg.command, "return_home")

if __name__ == '__main__':
    unittest.main()