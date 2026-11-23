#!/usr/bin/env python3
"""
Unit tests for Swarm Mission Planner Python Backend
Run with: python3 -m pytest test_python_backend.py -v
"""

import sys
import os
import json
import subprocess
import tempfile

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from swarm_mission_backend import SwarmMissionBackend


class TestSwarmMissionBackend:
    """Test the Python backend methods"""
    
    def setup_method(self):
        self.backend = SwarmMissionBackend()
    
    def test_health_check(self):
        result = self.backend.handle_request({
            'id': 'test1',
            'method': 'health_check',
            'args': {}
        })
        
        assert result['success'] == True
        assert result['result']['status'] == 'healthy'
        assert 'mavlink' in result['result']
        assert 'numpy' in result['result']
        assert 'scipy' in result['result']
    
    def test_generate_search_pattern_grid(self):
        result = self.backend.handle_request({
            'id': 'test2',
            'method': 'generate_search_pattern',
            'args': {
                'vertices': [
                    {'lat': 37.7749, 'lon': -122.4194},
                    {'lat': 37.7849, 'lon': -122.4194},
                    {'lat': 37.7849, 'lon': -122.4094},
                    {'lat': 37.7749, 'lon': -122.4094}
                ],
                'altitude': 50.0,
                'spacing': 20.0,
                'pattern_type': 'grid'
            }
        })
        
        assert result['success'] == True
        assert 'waypoints' in result['result']
        assert len(result['result']['waypoints']) > 0
        assert result['result']['pattern_type'] == 'grid'
        
        # Check waypoint structure
        wp = result['result']['waypoints'][0]
        assert 'seq' in wp
        assert 'lat' in wp
        assert 'lon' in wp
        assert 'alt' in wp
        assert 'command' in wp
    
    def test_generate_search_pattern_spiral(self):
        result = self.backend.handle_request({
            'id': 'test3',
            'method': 'generate_search_pattern',
            'args': {
                'vertices': [
                    {'lat': 37.7749, 'lon': -122.4194},
                    {'lat': 37.7849, 'lon': -122.4194},
                    {'lat': 37.7849, 'lon': -122.4094},
                    {'lat': 37.7749, 'lon': -122.4094}
                ],
                'altitude': 50.0,
                'spacing': 20.0,
                'pattern_type': 'spiral'
            }
        })
        
        assert result['success'] == True
        assert 'waypoints' in result['result']
    
    def test_generate_search_pattern_invalid_vertices(self):
        result = self.backend.handle_request({
            'id': 'test4',
            'method': 'generate_search_pattern',
            'args': {
                'vertices': [
                    {'lat': 37.7749, 'lon': -122.4194},
                    {'lat': 37.7849, 'lon': -122.4194}
                ],  # Only 2 vertices
                'altitude': 50.0,
                'spacing': 20.0
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
        assert '3 vertices' in result['error']
    
    def test_generate_inspect_waypoints(self):
        result = self.backend.handle_request({
            'id': 'test5',
            'method': 'generate_inspect_waypoints',
            'args': {
                'lat': 37.7749,
                'lon': -122.4194,
                'altitude': 10.0,
                'standoff': 5.0,
                'yaw_angles': [0, 90, 180, 270],
                'gimbal_pitch': -90.0
            }
        })
        
        assert result['success'] == True
        assert 'waypoints' in result['result']
        assert result['result']['inspect_count'] == 4
        assert result['result']['total_waypoints'] == 6  # approach + 4 inspect + depart
        
        # Check structure
        for wp in result['result']['waypoints']:
            assert 'seq' in wp
            assert 'lat' in wp
            assert 'lon' in wp
            assert 'alt' in wp
            assert 'type' in wp
    
    def test_generate_inspect_waypoints_circular(self):
        result = self.backend.handle_request({
            'id': 'test6',
            'method': 'generate_inspect_waypoints',
            'args': {
                'lat': 37.7749,
                'lon': -122.4194,
                'altitude': 10.0,
                'standoff': 5.0,
                'yaw_angles': [0, 90, 180, 270],
                'gimbal_pitch': -90.0,
                'radius': 10.0  # Circular orbit
            }
        })
        
        assert result['success'] == True
        assert 'waypoints' in result['result']
        
        # Check that inspect waypoints form a circle
        inspect_wps = [wp for wp in result['result']['waypoints'] if wp.get('type') == 'inspect']
        assert len(inspect_wps) == 4
    
    def test_cbba_allocate_simple(self):
        result = self.backend.handle_request({
            'id': 'test7',
            'method': 'cbba_allocate',
            'args': {
                'tasks': [
                    {'id': 't1', 'position': [0.0, 0.0], 'priority': 10},
                    {'id': 't2', 'position': [100.0, 0.0], 'priority': 5},
                    {'id': 't3', 'position': [0.0, 100.0], 'priority': 8}
                ],
                'drones': [
                    {'id': 1, 'position': [10.0, 10.0]},
                    {'id': 2, 'position': [90.0, 10.0]}
                ],
                'max_tasks_per_drone': 10
            }
        })
        
        assert result['success'] == True
        assert 'allocations' in result['result']
        assert '1' in result['result']['allocations']
        assert '2' in result['result']['allocations']
        assert result['result']['algorithm'] == 'CBBA-simplified'
        assert result['result']['converged'] == True
        
        # All tasks should be assigned
        total = sum(len(v) for v in result['result']['allocations'].values())
        assert total == 3
    
    def test_cbba_allocate_max_tasks(self):
        result = self.backend.handle_request({
            'id': 'test8',
            'method': 'cbba_allocate',
            'args': {
                'tasks': [
                    {'id': f't{i}', 'position': [float(i * 10), 0.0], 'priority': 10}
                    for i in range(15)
                ],
                'drones': [
                    {'id': 1, 'position': [0.0, 0.0]},
                    {'id': 2, 'position': [100.0, 0.0]}
                ],
                'max_tasks_per_drone': 5
            }
        })
        
        assert result['success'] == True
        
        # Each drone should have at most 5 tasks
        for drone_id, tasks in result['result']['allocations'].items():
            assert len(tasks) <= 5
        
        # Total assigned = 10 (5 per drone)
        total = sum(len(v) for v in result['result']['allocations'].values())
        assert total == 10
    
    def test_cbba_allocate_empty(self):
        result = self.backend.handle_request({
            'id': 'test9',
            'method': 'cbba_allocate',
            'args': {
                'tasks': [],
                'drones': []
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
    
    def test_mission_to_mavlink(self):
        result = self.backend.handle_request({
            'id': 'test10',
            'method': 'mission_to_mavlink',
            'args': {
                'waypoints': [
                    {'seq': 0, 'lat': 37.7749, 'lon': -122.4194, 'alt': 50, 'command': 16},
                    {'seq': 1, 'lat': 37.7849, 'lon': -122.4194, 'alt': 50, 'command': 16},
                ]
            }
        })
        
        assert result['success'] == True
        assert 'items' in result['result']
        assert len(result['result']['items']) == 2
        assert result['result']['count'] == 2
        
        # Check MAVLink format
        item = result['result']['items'][0]
        assert item['frame'] == 3  # MAV_FRAME_GLOBAL_RELATIVE_ALT
        assert item['command'] == 16
        assert item['x'] == int(37.7749 * 1e7)
        assert item['y'] == int(-122.4194 * 1e7)
        assert item['z'] == 50
    
    def test_mavlink_to_mission(self):
        result = self.backend.handle_request({
            'id': 'test11',
            'method': 'mavlink_to_mission',
            'args': {
                'items': [
                    {'seq': 0, 'x': 377749000, 'y': -1224194000, 'z': 50, 'command': 16},
                    {'seq': 1, 'x': 377849000, 'y': -1224194000, 'z': 50, 'command': 16},
                ]
            }
        })
        
        assert result['success'] == True
        assert 'waypoints' in result['result']
        assert len(result['result']['waypoints']) == 2
        
        wp = result['result']['waypoints'][0]
        assert abs(wp['lat'] - 37.7749) < 0.0001
        assert abs(wp['lon'] - (-122.4194)) < 0.0001
        assert wp['alt'] == 50
    
    def test_antenna_tracker_angles(self):
        result = self.backend.handle_request({
            'id': 'test12',
            'method': 'antenna_tracker_angles',
            'args': {
                'gs_lat': 37.7749,
                'gs_lon': -122.4194,
                'gs_alt': 0.0,
                'uav_lat': 37.7849,  # ~1.1km north
                'uav_lon': -122.4194,
                'uav_alt': 100.0
            }
        })
        
        assert result['success'] == True
        assert 'azimuth' in result['result']
        assert 'elevation' in result['result']
        assert 'distance' in result['result']
        
        # UAV is north of GS, so azimuth should be ~0 (or 360)
        azimuth = result['result']['azimuth']
        assert azimuth < 10 or azimuth > 350
        
        # Elevation should be positive (UAV above GS)
        elevation = result['result']['elevation']
        assert elevation > 0
        assert elevation < 90
        
        # Distance should be ~1111m (1/100 degree latitude)
        distance = result['result']['distance']
        assert 1000 < distance < 1300
    
    def test_coverage_analysis(self):
        result = self.backend.handle_request({
            'id': 'test13',
            'method': 'coverage_analysis',
            'args': {
                'waypoints': [
                    {'lat': 37.7749, 'lon': -122.4194, 'alt': 50},
                    {'lat': 37.7849, 'lon': -122.4194, 'alt': 50},  # ~1.1km north
                    {'lat': 37.7849, 'lon': -122.4094, 'alt': 50},  # ~0.9km east
                ],
                'sensor_fov': 60,
                'sensor_range': 100,
                'flight_alt': 50
            }
        })
        
        assert result['success'] == True
        assert 'footprint_radius_m' in result['result']
        assert 'track_spacing_m' in result['result']
        assert 'coverage_efficiency' in result['result']
        assert 'recommendations' in result['result']
        
        # 60 deg FOV at 50m altitude -> footprint radius = 50 * tan(30) = ~28.9m
        footprint = result['result']['footprint_radius_m']
        assert 25 < footprint < 35
        
        # Track spacing for 30% overlap
        spacing = result['result']['track_spacing_m']
        assert 35 < spacing < 45
    
    def test_condition_check(self):
        result = self.backend.handle_request({
            'id': 'test14',
            'method': 'condition_check',
            'args': {
                'log_path': '/fake/path/log.ulg'
            }
        })
        
        assert result['success'] == True
        assert 'overall' in result['result']
        assert 'checks' in result['result']
        assert len(result['result']['checks']) > 0
    
    def test_fft_analysis(self):
        result = self.backend.handle_request({
            'id': 'test15',
            'method': 'fft_analysis',
            'args': {
                'log_path': '/fake/path/log.ulg',
                'field': 'IMU.AccX',
                'sample_rate': 100
            }
        })
        
        # May fail if scipy not available
        if 'error' in result:
            assert 'scipy' in result['error'].lower()
        else:
            assert result['success'] == True
            assert 'dominant_frequency' in result['result']
    
    def test_pid_tuning_suggest(self):
        result = self.backend.handle_request({
            'id': 'test16',
            'method': 'pid_tuning_suggest',
            'args': {
                'log_path': '/fake/path/log.ulg',
                'axis': 'roll'
            }
        })
        
        # May fail if scipy not available
        if 'error' in result:
            assert 'scipy' in result['error'].lower()
        else:
            assert result['success'] == True
            assert 'suggested_pid' in result['result']
            assert 'metrics' in result['result']


def run_cli_tests():
    """Test CLI interface"""
    backend_path = os.path.join(os.path.dirname(__file__), '..', 'python', 'swarm_mission_backend.py')
    
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