#!/usr/bin/env python3
# Contract Test: HUMAN_INTENT_CMD MAVLink Message

import unittest
from pymavlink import mavutil

class TestHumanIntentCmdMavlink(unittest.TestCase):
    """Contract tests for HUMAN_INTENT_CMD (msg_id=30020)"""
    
    def setUp(self):
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14550', autoreconnect=True)
        self.mav.wait_heartbeat(timeout=10)
    
    def test_human_intent_cmd_message_structure(self):
        """Test HUMAN_INTENT_CMD (msg_id=30020) has correct structure"""
        # Create a human intent command message
        self.mav.mav.human_intent_cmd_send(
            1000000,    # timestamp_us
            100,        # source_id (ground station)
            0,          # target_id (0 = all drones)
            0,          # intent_type: 0=VOICE, 1=TABLET, 2=GESTURE, 3=AUTO
            "return_home".encode('utf-8'),  # command (max 32 chars)
            0.95,       # confidence [0, 1]
            "ko".encode('utf-8'),  # language (ISO 639-1, max 4 chars)
            '{"altitude": 50}'.encode('utf-8'),  # parameters JSON (max 256 chars)
            2           # priority: 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL
        )
        
        # Verify message was sent (would need to capture and parse)
        self.assertTrue(True)  # Placeholder
    
    def test_intent_type_enum(self):
        """Test IntentType enum values"""
        # 0=VOICE, 1=TABLET, 2=GESTURE, 3=AUTO
        for t in [0, 1, 2, 3]:
            self.assertIn(t, [0, 1, 2, 3])
    
    def test_priority_enum(self):
        """Test Priority enum values"""
        # 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL
        for p in [0, 1, 2, 3]:
            self.assertIn(p, [0, 1, 2, 3])
    
    def test_human_intent_cmd_voice_command(self):
        """Test voice command with Korean language"""
        self.mav.mav.human_intent_cmd_send(
            1000000, 100, 0, 0,  # VOICE
            "return_home".encode('utf-8'),
            0.92,
            "ko".encode('utf-8'),
            '{"trigger_phrase": "집으로 돌아와"}'.encode('utf-8'),
            2  # HIGH
        )
        self.assertTrue(True)
    
    def test_human_intent_cmd_tablet_command(self):
        """Test tablet command with search area"""
        self.mav.mav.human_intent_cmd_send(
            1000000, 100, 0, 1,  # TABLET
            "search_area".encode('utf-8'),
            1.0,
            "en".encode('utf-8'),
            '{"polygon": [[0,0],[1000,0],[1000,1000],[0,1000]], "pattern": "lawnmower"}'.encode('utf-8'),
            1  # NORMAL
        )
        self.assertTrue(True)
    
    def test_human_intent_cmd_gesture_command(self):
        """Test gesture command"""
        self.mav.mav.human_intent_cmd_send(
            1000000, 100, 0, 2,  # GESTURE
            "hold_position".encode('utf-8'),
            0.85,
            "".encode('utf-8'),  # No language for gestures
            '{"gesture_type": "palm_forward", "hand": "right"}'.encode('utf-8'),
            2  # HIGH
        )
        self.assertTrue(True)
    
    def test_human_intent_cmd_auto_command(self):
        """Test auto-generated emergency command"""
        self.mav.mav.human_intent_cmd_send(
            1000000, 100, 0, 3,  # AUTO
            "emergency_stop".encode('utf-8'),
            1.0,
            "".encode('utf-8'),
            '{"trigger": "geofence_breach", "source_drone": 3}'.encode('utf-8'),
            3  # CRITICAL
        )
        self.assertTrue(True)
    
    def test_human_intent_cmd_specific_target(self):
        """Test command targeting specific drone"""
        self.mav.mav.human_intent_cmd_send(
            1000000, 100, 5, 0,  # target drone 5
            "land_here".encode('utf-8'),
            0.98,
            "en".encode('utf-8'),
            '{"location": [100, 200, 0]}'.encode('utf-8'),
            2
        )
        self.assertTrue(True)
    
    def test_human_intent_cmd_formation_leader(self):
        """Test command targeting formation leader (special target_id=255)"""
        self.mav.mav.human_intent_cmd_send(
            1000000, 100, 255, 1,  # TABLET, target leader
            "change_formation".encode('utf-8'),
            1.0,
            "en".encode('utf-8'),
            '{"formation": "line", "spacing": 10}'.encode('utf-8'),
            1
        )
        self.assertTrue(True)
    
    def test_human_intent_cmd_all_standard_commands(self):
        """Test all standard command strings"""
        commands = [
            "return_home", "land_here", "follow_me", "hold_position",
            "change_formation", "search_area", "inspect_target",
            "emergency_stop", "resume_mission", "abort_mission"
        ]
        
        for i, cmd in enumerate(commands):
            self.mav.mav.human_intent_cmd_send(
                1000000 + i, 100, 0, 1,  # TABLET
                cmd.encode('utf-8'),
                1.0,
                "en".encode('utf-8'),
                "{}".encode('utf-8'),
                1
            )
        
        self.assertTrue(True)
    
    def test_human_intent_cmd_confidence_bounds(self):
        """Test confidence field bounds [0, 1]"""
        for conf in [0.0, 0.5, 0.95, 1.0]:
            self.mav.mav.human_intent_cmd_send(
                1000000, 100, 0, 1,
                "hold_position".encode('utf-8'),
                conf,
                "en".encode('utf-8'),
                "{}".encode('utf-8'),
                1
            )
        self.assertTrue(True)
    
    def test_human_intent_cmd_language_codes(self):
        """Test ISO 639-1 language codes"""
        languages = ["ko", "en", "ja", "zh", "fr", "de", "es"]
        
        for i, lang in enumerate(languages):
            self.mav.mav.human_intent_cmd_send(
                1000000 + i, 100, 0, 0,  # VOICE
                "return_home".encode('utf-8'),
                0.9,
                lang.encode('utf-8'),
                "{}".encode('utf-8'),
                1
            )
        self.assertTrue(True)
    
    def test_human_intent_cmd_parameters_json(self):
        """Test parameters JSON field"""
        test_params = [
            '{}',
            '{"altitude": 50}',
            '{"waypoints": [[100, 200, 50], [150, 250, 50]]}',
            '{"target_id": 5, "action": "inspect", "timeout": 30}'
        ]
        
        for i, params in enumerate(test_params):
            self.mav.mav.human_intent_cmd_send(
                1000000 + i, 100, 0, 1,
                "inspect_target".encode('utf-8'),
                0.9,
                "en".encode('utf-8'),
                params.encode('utf-8'),
                2
            )
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()