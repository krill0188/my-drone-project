#!/usr/bin/env python3
"""
Unit tests for Swarm Monitor Python Backend
Run with: python3 -m pytest test_python_backend.py -v
"""

import sys
import os
import json
import subprocess

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from swarm_monitor_backend import SwarmMonitorBackend


class TestSwarmMonitorBackend:
    """Test the Python backend methods"""
    
    def setup_method(self):
        self.backend = SwarmMonitorBackend()
    
    def test_health_check(self):
        result = self.backend.handle_request({
            'id': 'test1',
            'method': 'health_check',
            'args': {}
        })
        
        assert result['success'] == True
        assert result['result']['status'] == 'healthy'
        assert 'numpy' in result['result']
        assert 'scipy' in result['result']
        assert 'pandas' in result['result']
    
    def test_analyze_formation_perfect(self):
        result = self.backend.handle_request({
            'id': 'test2',
            'method': 'analyze_formation',
            'args': {
                'vehicles': [
                    {'id': 1, 'lat': 37.7749, 'lon': -122.4194, 'callsign': 'DRONE1'},
                    {'id': 2, 'lat': 37.7759, 'lon': -122.4194, 'callsign': 'DRONE2'},
                ],
                'formation_type': 'line',
                'assigned_positions': {
                    '1': {'lat': 37.7749, 'lon': -122.4194},
                    '2': {'lat': 37.7759, 'lon': -122.4194},
                }
            }
        })
        
        assert result['success'] == True
        assert result['result']['avg_error_m'] < 1.0
        assert result['result']['formation_state'] in ['excellent', 'good']
        assert result['result']['vehicles_in_tolerance'] == 2
    
    def test_analyze_formation_degraded(self):
        result = self.backend.handle_request({
            'id': 'test3',
            'method': 'analyze_formation',
            'args': {
                'vehicles': [
                    {'id': 1, 'lat': 37.7749, 'lon': -122.4194, 'callsign': 'DRONE1'},
                    {'id': 2, 'lat': 37.7759, 'lon': -122.4194, 'callsign': 'DRONE2'},
                    {'id': 3, 'lat': 37.7769, 'lon': -122.4194, 'callsign': 'DRONE3'},
                ],
                'formation_type': 'line',
                'assigned_positions': {
                    '1': {'lat': 37.7749, 'lon': -122.4194},
                    '2': {'lat': 37.7759, 'lon': -122.4194},
                    '3': {'lat': 37.7859, 'lon': -122.4194},  # 1km off!
                }
            }
        })
        
        assert result['success'] == True
        assert result['result']['vehicles_lost'] >= 1
        assert result['result']['formation_state'] == 'lost'
    
    def test_analyze_health_trends(self):
        # Create history with decreasing battery
        history = []
        base_time = 1000000
        for i in range(20):
            history.append({
                'timestamp': base_time + i * 60,  # 1 min intervals
                'vehicle_id': 1,
                'callsign': 'DRONE1',
                'battery_remaining_pct': 100 - i * 2,  # 2% per minute
                'battery_voltage_v': 25.2 - i * 0.1,
                'link_quality': 0.95,
                'rssi_dbm': -50,
                'latency_ms': 10,
                'gps_satellites': 12,
                'ekf_innovation': 0.1,
                'inference_time_ms': 30,
                'formation_error_m': 0.5
            })
        
        result = self.backend.handle_request({
            'id': 'test4',
            'method': 'analyze_health_trends',
            'args': {
                'history': history,
                'window_minutes': 60
            }
        })
        
        assert result['success'] == True
        assert '1' in result['result']
        assert 'battery_depletion_rate_pct_per_min' in result['result']['1']['trends']
        # Should detect ~2%/min depletion
        assert result['result']['1']['trends']['battery_depletion_rate_pct_per_min'] < -1.0
    
    def test_analyze_mesh_qos(self):
        result = self.backend.handle_request({
            'id': 'test5',
            'method': 'analyze_mesh_qos',
            'args': {
                'nodes': [
                    {'node_id': 1, 'link_quality_to': {2: 0.9, 3: 0.7}, 'rssi_dbm_to': {2: -50, 3: -70}, 'latency_ms_to': {2: 10, 3: 25}},
                    {'node_id': 2, 'link_quality_to': {1: 0.9, 3: 0.8}, 'rssi_dbm_to': {1: -50, 3: -60}, 'latency_ms_to': {1: 10, 3: 15}},
                    {'node_id': 3, 'link_quality_to': {1: 0.7, 2: 0.8}, 'rssi_dbm_to': {1: -70, 2: -60}, 'latency_ms_to': {1: 25, 2: 15}},
                ]
            }
        })
        
        assert result['success'] == True
        assert result['result']['total_nodes'] == 3
        assert result['result']['total_links'] == 6
        assert result['result']['link_quality']['mean'] > 0.7
        assert len(result['result']['recommendations']) >= 0
    
    def test_analyze_safety_events(self):
        events = [
            {'timestamp_us': 1000000000, 'event_type': 'GEOFENCE_BREACH', 'severity': 'ERROR', 'source_id': 1},
            {'timestamp_us': 1000000001, 'event_type': 'LOW_BATTERY', 'severity': 'WARN', 'source_id': 2},
            {'timestamp_us': 1000000002, 'event_type': 'LINK_LOSS', 'severity': 'CRITICAL', 'source_id': 1},
            {'timestamp_us': 1000000003, 'event_type': 'ESTOP', 'severity': 'CRITICAL', 'source_id': 2},
        ]
        
        result = self.backend.handle_request({
            'id': 'test6',
            'method': 'analyze_safety_events',
            'args': {
                'events': events,
                'window_hours': 24
            }
        })
        
        assert result['success'] == True
        assert result['result']['total_events'] == 4
        assert result['result']['by_severity']['CRITICAL'] == 2
        assert result['result']['by_severity']['ERROR'] == 1
        assert result['result']['by_severity']['WARN'] == 1
        assert result['result']['critical_count'] == 2
    
    def test_predict_battery_failure(self):
        result = self.backend.handle_request({
            'id': 'test7',
            'method': 'predict_battery_failure',
            'args': {
                'vehicles': [
                    {'id': 1, 'callsign': 'DRONE1', 'battery_remaining_pct': 45, 'battery_current_a': 15, 'battery_capacity_mah': 5000},
                    {'id': 2, 'callsign': 'DRONE2', 'battery_remaining_pct': 15, 'battery_current_a': 20, 'battery_capacity_mah': 5000},
                    {'id': 3, 'callsign': 'DRONE3', 'battery_remaining_pct': 5, 'battery_current_a': 25, 'battery_capacity_mah': 5000},
                ],
                'safety_margin_pct': 20
            }
        })
        
        assert result['success'] == True
        preds = result['result']['predictions']
        
        # DRONE1: 45% - 20% = 25% available, 15A = 250 mAh/min, 5000*0.25/250 = 5 min
        assert preds[0]['risk_level'] in ['LOW', 'MEDIUM']
        
        # DRONE2: 15% < 20% safety margin -> CRITICAL
        assert preds[1]['risk_level'] == 'CRITICAL'
        
        # DRONE3: 5% < 20% -> CRITICAL
        assert preds[2]['risk_level'] == 'CRITICAL'
        
        assert len(result['result']['critical_vehicles']) == 2
    
    def test_predict_link_failure(self):
        result = self.backend.handle_request({
            'id': 'test8',
            'method': 'predict_link_failure',
            'args': {
                'nodes': [
                    {'node_id': 1, 'link_quality_to': {2: 0.9, 3: 0.2}, 'rssi_dbm_to': {2: -50, 3: -85}, 'packets_lost_to': {2: 1, 3: 100}, 'packets_sent_to': {2: 100, 3: 100}},
                    {'node_id': 2, 'link_quality_to': {1: 0.9, 3: 0.8}, 'rssi_dbm_to': {1: -50, 3: -60}, 'packets_lost_to': {1: 1, 3: 5}, 'packets_sent_to': {1: 100, 3: 100}},
                ]
            }
        })
        
        assert result['success'] == True
        preds = result['result']['predictions']
        
        # 1->3 should be CRITICAL (quality 0.2, RSSI -85, 10% loss)
        critical = [p for p in preds if p['risk'] == 'CRITICAL']
        assert len(critical) >= 1
        
        # 1->2 and 2->1 should be LOW
        low = [p for p in preds if p['risk'] == 'LOW']
        assert len(low) >= 2
    
    def test_detect_anomalies(self):
        # Create data with clear anomaly
        data = []
        for i in range(100):
            data.append({
                'vehicle_id': 1,
                'timestamp': 1000 + i,
                'battery_remaining_pct': 100 - i * 0.1,
                'link_quality': 0.95,
            })
        # Add anomaly at index 50
        data[50]['link_quality'] = 0.1  # Sudden drop
        
        result = self.backend.handle_request({
            'id': 'test9',
            'method': 'detect_anomalies',
            'args': {
                'data': data,
                'threshold_std': 3.0
            }
        })
        
        assert result['success'] == True
        anomalies = result['result']['anomalies']
        assert len(anomalies) >= 1
        # Should detect the link_quality anomaly at vehicle 1
        assert any(a['field'] == 'link_quality' for a in anomalies)
    
    def test_correlate_events(self):
        events = [
            {'timestamp_us': 1000000000, 'event_type': 'LOW_BATTERY', 'severity': 'WARN', 'source_id': 1},
        ]
        
        telemetry = [
            {'timestamp': 999, 'vehicle_id': 1, 'battery_remaining_pct': 25, 'link_quality': 0.9},
            {'timestamp': 1000, 'vehicle_id': 1, 'battery_remaining_pct': 20, 'link_quality': 0.9},
            {'timestamp': 1001, 'vehicle_id': 1, 'battery_remaining_pct': 18, 'link_quality': 0.9},
        ]
        
        result = self.backend.handle_request({
            'id': 'test10',
            'method': 'correlate_events',
            'args': {
                'events': events,
                'telemetry': telemetry,
                'window_seconds': 30
            }
        })
        
        assert result['success'] == True
        correlations = result['result']['correlations']
        assert len(correlations) == 1
        assert correlations[0]['telemetry_samples'] == 3
        assert correlations[0]['avg_battery_pct'] == 21.0
    
    def test_generate_health_report(self):
        vehicles = [
            {'id': 1, 'lat': 37.7749, 'lon': -122.4194},
            {'id': 2, 'lat': 37.7759, 'lon': -122.4194},
        ]
        
        result = self.backend.handle_request({
            'id': 'test11',
            'method': 'generate_health_report',
            'args': {
                'vehicles': vehicles,
                'mesh': [],
                'events': [],
                'history': []
            }
        })
        
        assert result['success'] == True
        assert 'overall_status' in result['result']
        assert 'formation_analysis' in result['result']


def run_cli_tests():
    """Test CLI interface"""
    backend_path = os.path.join(os.path.dirname(__file__), '..', 'python', 'swarm_monitor_backend.py')
    
    # Test --test flag
    result = subprocess.run([sys.executable, backend_path, '--test'], 
                          capture_output=True, text=True, timeout=30)
    assert result.returncode == 0
    assert 'All self-tests passed' in result.stdout
    
    # Test single method call
    result = subprocess.run([
        sys.executable, backend_path,
        '--method', 'health_check',
        '--args', '{}'
    ], capture_output=True, text=True, timeout=10)
    assert result.returncode == 0
    response = json.loads(result.stdout)
    assert response['success'] == True
    assert response['result']['status'] == 'healthy'


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])