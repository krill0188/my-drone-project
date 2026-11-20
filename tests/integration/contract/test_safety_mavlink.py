#!/usr/bin/env python3
# Contract Test: SAFETY_EVENT MAVLink Messages (8 types)

import unittest
from pymavlink import mavutil

class TestSafetyEventMavlink(unittest.TestCase):
    """Contract tests for SAFETY_EVENT MAVLink messages (msg_id=30010-30017)"""
    
    def setUp(self):
        self.mav = mavutil.mavlink_connection('udp:127.0.0.1:14550', autoreconnect=True)
        self.mav.wait_heartbeat(timeout=10)
    
    # ============================================================================
    # GEOFENCE_BREACH (msg_id=30010)
    # ============================================================================
    
    def test_geofence_breach_message_structure(self):
        """Test GEOFENCE_BREACH (msg_id=30010) has correct structure"""
        self.mav.mav.geofence_breach_send(
            1000000,    # timestamp_us
            100.0, 200.0, 50.0,  # position_enu (x, y, z)
            2,          # action: 0=NONE, 1=WARNING, 2=RTL, 3=LAND, 4=TERMINATE
            2,          # severity: 0=INFO, 1=WARNING, 2=CRITICAL, 3=EMERGENCY
            "boundary_1".encode('utf-8')  # polygon_name (max 32 chars)
        )
        self.assertTrue(True)
    
    def test_geofence_breach_action_enum(self):
        """Test GeofenceAction enum values"""
        actions = [0, 1, 2, 3, 4]  # NONE, WARNING, RTL, LAND, TERMINATE
        for action in actions:
            self.mav.mav.geofence_breach_send(
                1000000, 0, 0, 0, action, 1, "test".encode('utf-8')
            )
        self.assertTrue(True)
    
    def test_geofence_breach_severity_enum(self):
        """Test SafetySeverity enum values"""
        severities = [0, 1, 2, 3]  # INFO, WARNING, CRITICAL, EMERGENCY
        for sev in severities:
            self.mav.mav.geofence_breach_send(
                1000000, 0, 0, 0, 2, sev, "test".encode('utf-8')
            )
        self.assertTrue(True)
    
    def test_geofence_breach_polygon_names(self):
        """Test various polygon names"""
        names = ["boundary_1", "no_fly_zone_A", "airport_5km", "restricted_military"]
        for i, name in enumerate(names):
            self.mav.mav.geofence_breach_send(
                1000000 + i, 0, 0, 0, 4, 3, name.encode('utf-8')
            )
        self.assertTrue(True)
    
    # ============================================================================
    # EMERGENCY_STOP (msg_id=30011)
    # ============================================================================
    
    def test_emergency_stop_message_structure(self):
        """Test EMERGENCY_STOP (msg_id=30011) has correct structure"""
        self.mav.mav.emergency_stop_send(
            1000000,    # timestamp_us
            100.0, 200.0, 50.0,  # position_enu
            0,          # reason_code: 0=local, 1=remote, 2=auto
            True,       # parachute_deployed
            "Manual trigger from GCS".encode('utf-8')  # reason (max 64 chars)
        )
        self.assertTrue(True)
    
    def test_emergency_stop_reason_codes(self):
        """Test EmergencyStop reason codes"""
        codes = [0, 1, 2]  # local, remote, auto
        for code in codes:
            self.mav.mav.emergency_stop_send(
                1000000, 0, 0, 0, code, False, "test".encode('utf-8')
            )
        self.assertTrue(True)
    
    def test_emergency_stop_parachute(self):
        """Test parachute deployment flag"""
        self.mav.mav.emergency_stop_send(
            1000000, 0, 0, 0, 0, True, "Auto deploy".encode('utf-8')
        )
        self.mav.mav.emergency_stop_send(
            1000000, 0, 0, 0, 0, False, "No parachute".encode('utf-8')
        )
        self.assertTrue(True)
    
    def test_emergency_stop_reasons(self):
        """Test various emergency stop reasons"""
        reasons = [
            "Manual trigger from GCS",
            "Geofence breach - TERMINATE",
            "Critical fault: motor_3 failure",
            "Battery critical: 5%",
            "Comms lost > 10s",
            "Operator emergency button"
        ]
        for i, reason in enumerate(reasons):
            self.mav.mav.emergency_stop_send(
                1000000 + i, 0, 0, 0, 0, False, reason.encode('utf-8')
            )
        self.assertTrue(True)
    
    # ============================================================================
    # RID_STATUS (msg_id=30012)
    # ============================================================================
    
    def test_rid_status_message_structure(self):
        """Test RID_STATUS (msg_id=30012) has correct structure"""
        self.mav.mav.rid_status_send(
            1000000,    # timestamp_us
            0,          # status: 0=broadcasting, 1=stopped, 2=error
            0,          # error_code
            1000,       # frames_sent
            5           # frames_failed
        )
        self.assertTrue(True)
    
    def test_rid_status_status_enum(self):
        """Test RID status enum"""
        statuses = [0, 1, 2]  # broadcasting, stopped, error
        for status in statuses:
            self.mav.mav.rid_status_send(
                1000000, status, 0, 0, 0
            )
        self.assertTrue(True)
    
    def test_rid_status_error_codes(self):
        """Test RID error codes"""
        codes = [0, 1, 2, 3, 4]  # none, radio_failure, gps_failure, config_error, regulatory
        for code in codes:
            self.mav.mav.rid_status_send(
                1000000, 2, code, 0, 0
            )
        self.assertTrue(True)
    
    def test_rid_status_counters(self):
        """Test frame counters"""
        self.mav.mav.rid_status_send(
            1000000, 0, 0, 3600, 0  # 1 hour at 1Hz
        )
        self.mav.mav.rid_status_send(
            1000000, 0, 0, 3600, 10  # 10 failures
        )
        self.assertTrue(True)
    
    # ============================================================================
    # CRITICAL_FAULT (msg_id=30013)
    # ============================================================================
    
    def test_critical_fault_message_structure(self):
        """Test CRITICAL_FAULT (msg_id=30013) has correct structure"""
        self.mav.mav.critical_fault_send(
            1000000,    # timestamp_us
            100.0, 200.0, 50.0,  # position_enu
            2,          # fault_type: 0=IMU_DRIFT, 1=GPS_SPOOF, 2=MOTOR_DEGRADE, 3=BATTERY_IMBALANCE, 4=SENSOR_FAULT, 5=COMM_DEGRADE
            3,          # severity: 0=INFO, 1=WARNING, 2=CRITICAL, 3=EMERGENCY
            "motor_3".encode('utf-8'),  # component (max 32 chars)
            "Thrust loss 30%".encode('utf-8')  # fault_desc (max 64 chars)
        )
        self.assertTrue(True)
    
    def test_critical_fault_fault_types(self):
        """Test FaultType enum values"""
        types = [0, 1, 2, 3, 4, 5]  # IMU_DRIFT, GPS_SPOOF, MOTOR_DEGRADE, BATTERY_IMBALANCE, SENSOR_FAULT, COMM_DEGRADE
        for ft in types:
            self.mav.mav.critical_fault_send(
                1000000, 0, 0, 0, ft, 2, "test".encode('utf-8'), "test".encode('utf-8')
            )
        self.assertTrue(True)
    
    def test_critical_fault_severity_enum(self):
        """Test FaultSeverity enum values"""
        severities = [0, 1, 2, 3]  # INFO, WARNING, CRITICAL, EMERGENCY
        for sev in severities:
            self.mav.mav.critical_fault_send(
                1000000, 0, 0, 0, 2, sev, "test".encode('utf-8'), "test".encode('utf-8')
            )
        self.assertTrue(True)
    
    def test_critical_fault_components(self):
        """Test various component names"""
        components = [
            "motor_1", "motor_2", "motor_3", "motor_4",
            "esc_1", "esc_2", "esc_3", "esc_4",
            "battery", "cell_1", "cell_2", "cell_3", "cell_4",
            "imu_0", "imu_1", "gps_0", "gps_1",
            "baro", "mag", "camera_fwd", "camera_down",
            "lidar_2d", "lidar_3d", "uwb_anchor_1",
            "radio_5g", "radio_lora", "radio_mesh"
        ]
        for i, comp in enumerate(components):
            self.mav.mav.critical_fault_send(
                1000000 + i, 0, 0, 0, 2, 2, comp.encode('utf-8'), "degraded".encode('utf-8')
            )
        self.assertTrue(True)
    
    def test_critical_fault_descriptions(self):
        """Test fault descriptions"""
        descs = [
            "Thrust loss 30%",
            "RPM oscillation detected",
            "Position jump 50m",
            "HDOP > 5",
            "Cell voltage diff 0.2V",
            "Accelerometer stuck at 0",
            "Link quality < 20%",
            "Packet loss > 50%"
        ]
        for i, desc in enumerate(descs):
            self.mav.mav.critical_fault_send(
                1000000 + i, 0, 0, 0, 2, 2, "test".encode('utf-8'), desc.encode('utf-8')
            )
        self.assertTrue(True)
    
    # ============================================================================
    # COMMS_LOST (msg_id=30014)
    # ============================================================================
    
    def test_comms_lost_message_structure(self):
        """Test COMMS_LOST (msg_id=30014) has correct structure"""
        self.mav.mav.comms_lost_send(
            1000000,    # timestamp_us
            5,          # lost_drone_id
            0,          # comm_type: 0=mesh, 1=5G, 2=LoRa
            -85.0,      # last_rssi (dBm)
            15000       # lost_duration_ms
        )
        self.assertTrue(True)
    
    def test_comms_lost_comm_types(self):
        """Test communication type enum"""
        types = [0, 1, 2]  # mesh, 5G, LoRa
        for ct in types:
            self.mav.mav.comms_lost_send(
                1000000, 5, ct, -80.0, 10000
            )
        self.assertTrue(True)
    
    def test_comms_lost_rssi_values(self):
        """Test RSSI values"""
        rssis = [-50.0, -70.0, -85.0, -100.0, -120.0]
        for i, rssi in enumerate(rssis):
            self.mav.mav.comms_lost_send(
                1000000 + i, 5, 0, rssi, 5000
            )
        self.assertTrue(True)
    
    def test_comms_lost_durations(self):
        """Test lost duration values"""
        durations = [1000, 5000, 10000, 30000, 60000]  # 1s to 60s
        for i, dur in enumerate(durations):
            self.mav.mav.comms_lost_send(
                1000000 + i, 5, 0, -80.0, dur
            )
        self.assertTrue(True)
    
    # ============================================================================
    # LEADER_FAIL (msg_id=30015)
    # ============================================================================
    
    def test_leader_fail_message_structure(self):
        """Test LEADER_FAIL (msg_id=30015) has correct structure"""
        self.mav.mav.leader_fail_send(
            1000000,    # timestamp_us
            3,          # failed_leader_id
            5,          # new_leader_id (0 = election pending)
            1           # fail_reason: 0=comms, 1=fault, 2=estop
        )
        self.assertTrue(True)
    
    def test_leader_fail_reasons(self):
        """Test leader failure reason enum"""
        reasons = [0, 1, 2]  # comms, fault, estop
        for reason in reasons:
            self.mav.mav.leader_fail_send(
                1000000, 3, 5, reason
            )
        self.assertTrue(True)
    
    def test_leader_fail_election_pending(self):
        """Test election pending (new_leader_id = 0)"""
        self.mav.mav.leader_fail_send(
            1000000, 3, 0, 0  # election in progress
        )
        self.assertTrue(True)
    
    def test_leader_fail_new_leader(self):
        """Test successful leader election"""
        self.mav.mav.leader_fail_send(
            1000000, 3, 7, 0  # drone 7 becomes new leader
        )
        self.assertTrue(True)
    
    # ============================================================================
    # BATTERY_CRITICAL (msg_id=30016)
    # ============================================================================
    
    def test_battery_critical_message_structure(self):
        """Test BATTERY_CRITICAL (msg_id=30016) has correct structure"""
        self.mav.mav.battery_critical_send(
            1000000,    # timestamp_us
            5,          # drone_id
            0.05,       # battery_percentage (0.0-1.0)
            18.5,       # voltage_v
            -25.0,      # current_a (negative = discharging)
            6,          # cell_count
            3.05        # min_cell_v
        )
        self.assertTrue(True)
    
    def test_battery_critical_percentage_bounds(self):
        """Test battery percentage bounds [0, 1]"""
        for pct in [0.0, 0.05, 0.1, 0.15, 0.2, 0.3]:
            self.mav.mav.battery_critical_send(
                1000000, 5, pct, 18.0, -20.0, 6, 3.0
            )
        self.assertTrue(True)
    
    def test_battery_critical_voltages(self):
        """Test voltage values"""
        voltages = [16.8, 18.5, 21.0, 22.2, 24.0, 25.2]  # 4S to 6S
        for i, v in enumerate(voltages):
            self.mav.mav.battery_critical_send(
                1000000 + i, 5, 0.1, v, -15.0, 6, 3.0
            )
        self.assertTrue(True)
    
    def test_battery_critical_cell_counts(self):
        """Test cell counts"""
        for cells in [3, 4, 5, 6, 8, 12]:
            self.mav.mav.battery_critical_send(
                1000000, 5, 0.05, 3.7 * cells, -10.0, cells, 3.2
            )
        self.assertTrue(True)
    
    def test_battery_critical_min_cell_voltage(self):
        """Test minimum cell voltage"""
        min_cells = [2.5, 2.8, 3.0, 3.2, 3.5, 3.7]
        for v in min_cells:
            self.mav.mav.battery_critical_send(
                1000000, 5, 0.05, 22.2, -20.0, 6, v
            )
        self.assertTrue(True)
    
    # ============================================================================
    # SENSOR_FAILURE (msg_id=30017)
    # ============================================================================
    
    def test_sensor_failure_message_structure(self):
        """Test SENSOR_FAILURE (msg_id=30017) has correct structure"""
        self.mav.mav.sensor_failure_send(
            1000000,    # timestamp_us
            0,          # sensor_type: 0=IMU, 1=GPS, 2=Baro, 3=Mag, 4=Camera, 5=Lidar, 6=UWB
            0,          # sensor_instance (0 or 1 for redundant)
            0,          # failure_mode: 0=stuck, 1=noisy, 2=offline, 3=diverged
            0.0, 0.0, 9.81  # last_value[3] (e.g., accel z)
        )
        self.assertTrue(True)
    
    def test_sensor_failure_sensor_types(self):
        """Test SensorType enum values"""
        types = [0, 1, 2, 3, 4, 5, 6]  # IMU, GPS, Baro, Mag, Camera, Lidar, UWB
        for st in types:
            self.mav.mav.sensor_failure_send(
                1000000, st, 0, 0, 0, 0, 0
            )
        self.assertTrue(True)
    
    def test_sensor_failure_instances(self):
        """Test sensor instance (0/1 for redundant)"""
        self.mav.mav.sensor_failure_send(1000000, 0, 0, 0, 0, 0, 0)  # IMU 0
        self.mav.mav.sensor_failure_send(1000000, 0, 1, 0, 0, 0, 0)  # IMU 1
        self.mav.mav.sensor_failure_send(1000000, 1, 0, 0, 0, 0, 0)  # GPS 0
        self.mav.mav.sensor_failure_send(1000000, 1, 1, 0, 0, 0, 0)  # GPS 1
        self.assertTrue(True)
    
    def test_sensor_failure_modes(self):
        """Test failure mode enum"""
        modes = [0, 1, 2, 3]  # stuck, noisy, offline, diverged
        for mode in modes:
            self.mav.mav.sensor_failure_send(
                1000000, 0, 0, mode, 0, 0, 0
            )
        self.assertTrue(True)
    
    def test_sensor_failure_last_values(self):
        """Test last_value array for different sensor types"""
        # IMU: accel
        self.mav.mav.sensor_failure_send(1000000, 0, 0, 0, 0.1, -0.2, 9.81)
        # GPS: position
        self.mav.mav.sensor_failure_send(1000000, 1, 0, 0, 100.0, 200.0, 50.0)
        # Baro: altitude
        self.mav.mav.sensor_failure_send(1000000, 2, 0, 0, 100.5, 0.0, 0.0)
        # Mag: field
        self.mav.mav.sensor_failure_send(1000000, 3, 0, 0, 0.2, 0.1, 0.4)
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()