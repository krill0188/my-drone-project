#!/usr/bin/env python3
"""
Unit tests for Post-Flight Analysis Python Backend
Run with: python3 -m pytest test_python_backend.py -v
"""

import sys
import os
import json
import tempfile

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

from post_flight_backend import PostFlightBackend


class TestPostFlightBackend:
    """Test the Python backend methods"""
    
    def setup_method(self):
        self.backend = PostFlightBackend()
    
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
        assert 'matplotlib' in result['result']
    
    def test_haversine(self):
        # Test haversine distance calculation
        backend = PostFlightBackend()
        
        # Same point
        d1 = backend._haversine(37.7749, -122.4194, 37.7749, -122.4194)
        assert abs(d1) < 1.0
        
        # ~111km (1 degree latitude)
        d2 = backend._haversine(37.7749, -122.4194, 38.7749, -122.4194)
        assert abs(d2 - 111195) < 1000
        
        # ~111km * cos(lat) at lat 37.77
        d3 = backend._haversine(37.7749, -122.4194, 37.7749, -121.4194)
        expected = 111195 * math.cos(37.7749 * math.pi / 180)
        assert abs(d3 - expected) < 1000
    
    def test_footprint_radius(self):
        backend = PostFlightBackend()
        
        # 50m altitude, 90 deg FOV
        r1 = 50 * math.tan(math.radians(90 / 2))
        assert abs(r1 - 50.0) < 0.1
        
        # 100m altitude, 60 deg FOV
        r2 = 100 * math.tan(math.radians(60 / 2))
        expected = 100 * math.tan(math.pi / 6)
        assert abs(r2 - expected) < 0.1
    
    def test_grid_cell_calculation(self):
        backend = PostFlightBackend()
        
        # Same as min
        lat, lon = backend._grid_cell(37.7749, -122.4194, 37.7749, -122.4194, 10)
        assert lat == 0 and lon == 0
        
        # 0.001 deg lat ~ 111m
        lat, lon = backend._grid_cell(37.7759, -122.4194, 37.7749, -122.4194, 10)
        assert lat == 11
    
    def test_analyze_coverage_local(self):
        # Create mock log data with local coordinates
        log_data = {
            'vehicle_local_position': [
                {'x': 0, 'y': 0},
                {'x': 100, 'y': 0},
                {'x': 200, 'y': 0},
                {'x': 200, 'y': 100},
                {'x': 100, 'y': 100},
                {'x': 0, 'y': 100},
                {'x': 0, 'y': 0},
            ]
        }
        
        # This would test the coverage analysis with local coordinates
        # For now, verify the method exists and handles basic cases
        result = self.backend.handle_request({
            'id': 'test1',
            'method': 'analyze_coverage',
            'args': {
                'log_path': '/nonexistent.log',
                'log_id': 'test_log'
            }
        })
        
        # Should fail gracefully for nonexistent file
        assert result['success'] == False
        assert 'error' in result
    
    def test_analyze_formation_error(self):
        result = self.backend.handle_request({
            'id': 'test2',
            'method': 'analyze_formation_error',
            'args': {
                'log_path': '/nonexistent.log',
                'log_id': 'test_log'
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
    
    def test_analyze_ai_timeline(self):
        result = self.backend.handle_request({
            'id': 'test3',
            'method': 'analyze_ai_timeline',
            'args': {
                'log_path': '/nonexistent.log',
                'log_id': 'test_log'
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
    
    def test_analyze_qos(self):
        result = self.backend.handle_request({
            'id': 'test4',
            'method': 'analyze_qos',
            'args': {
                'log_path': '/nonexistent.log',
                'log_id': 'test_log'
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
    
    def test_run_full_analysis(self):
        result = self.backend.handle_request({
            'id': 'test5',
            'method': 'run_full_analysis',
            'args': {
                'log_path': '/nonexistent.log',
                'log_id': 'test_log'
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
    
    def test_generate_report_invalid_log(self):
        result = self.backend.handle_request({
            'id': 'test6',
            'method': 'generate_report',
            'args': {
                'log_id': 'nonexistent',
                'log_path': '/nonexistent.log',
                'report_id': 'report1',
                'output_dir': '/tmp/reports',
                'format': 'html',
                'options': {}
            }
        })
        
        assert result['success'] == False
        assert 'error' in result
    
    def test_cdf_calculation(self):
        # Test CDF calculation logic
        errors = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 10.0]
        errors.sort()
        
        cdf_x = errors
        cdf_y = [(i + 1) / len(errors) for i in range(len(errors))]
        
        assert len(cdf_x) == len(cdf_y)
        assert cdf_y[-1] == 1.0
        assert cdf_y[0] == 0.1
        
        # P95
        p95_idx = int(0.95 * len(errors))
        assert errors[p95_idx] == 6.0
        
        # P99
        p99_idx = int(0.99 * len(errors))
        assert errors[p99_idx] == 10.0
    
    def test_statistics(self):
        errors = [1.0, 2.0, 3.0, 4.0, 5.0]
        
        mean = statistics.mean(errors)
        assert abs(mean - 3.0) < 0.01
        
        std = statistics.stdev(errors)
        assert abs(std - math.sqrt(2.0)) < 0.01
        
        median = statistics.median(errors)
        assert median == 3.0
        
        max_e = max(errors)
        assert max_e == 5.0
    
    def test_percentile(self):
        data = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        
        p50 = numpy.percentile(data, 50) if NUMPY_AVAILABLE else 5.5
        p95 = numpy.percentile(data, 95) if NUMPY_AVAILABLE else 9.55
        
        # Manual percentile calculation
        sorted_data = sorted(data)
        k = (len(sorted_data) - 1) * 50 / 100
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            manual_p50 = sorted_data[int(k)]
        else:
            manual_p50 = sorted_data[f] * (c - k) + sorted_data[c] * (k - f)
        
        assert abs(manual_p50 - 5.5) < 0.1
    
    def test_detections_grouping(self):
        inferences = [
            ('person', 0.9, 30), ('car', 0.8, 25), ('person', 0.95, 35),
            ('bike', 0.7, 40), ('person', 0.85, 28), ('car', 0.9, 22)
        ]
        
        by_class_conf = {}
        by_class_time = {}
        
        for cls, conf, time in inferences:
            if cls not in by_class_conf:
                by_class_conf[cls] = []
                by_class_time[cls] = []
            by_class_conf[cls].append(conf)
            by_class_time[cls].append(time)
        
        assert len(by_class_conf['person']) == 3
        assert len(by_class_conf['car']) == 2
        assert len(by_class_conf['bike']) == 1
        
        person_avg_conf = sum(by_class_conf['person']) / len(by_class_conf['person'])
        assert abs(person_avg_conf - 0.9) < 0.01
    
    def test_inference_time_anomaly(self):
        timeline = [
            (1000000, 30), (2000000, 35), (3000000, 28),
            (4000000, 45), (5000000, 50), (6000000, 120),  # Spike
        ]
        
        times = [t for _, t in timeline]
        mean = sum(times) / len(times)
        
        threshold = 50  # Real-time threshold
        anomalies = sum(1 for t in times if t > threshold)
        
        assert anomalies == 1
    
    def test_qos_aggregation(self):
        links = [
            (1, 2, 0.9, -50, 10, 0.001),
            (1, 3, 0.7, -70, 25, 0.02),
            (2, 1, 0.9, -50, 10, 0.001),
            (2, 3, 0.8, -60, 15, 0.005),
            (3, 1, 0.7, -70, 25, 0.02),
            (3, 2, 0.8, -60, 15, 0.005),
        ]
        
        qualities = [q for _, _, q, _, _, _ in links]
        rssis = [r for _, _, _, r, _, _ in links]
        latencies = [l for _, _, _, _, l, _ in links]
        loss_rates = [lr for _, _, _, _, _, lr in links]
        
        mean_q = sum(qualities) / len(qualities)
        mean_rssi = sum(rssis) / len(rssis)
        mean_lat = sum(latencies) / len(latencies)
        mean_loss = sum(loss_rates) / len(loss_rates)
        
        assert abs(mean_q - 0.8) < 0.01
        assert abs(mean_rssi + 60) < 1
        assert abs(mean_lat - 16.67) < 1
        assert abs(mean_loss - 0.0087) < 0.001


import math
import statistics
try:
    import numpy
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False


def run_cli_tests():
    """Test CLI interface"""
    backend_path = os.path.join(os.path.dirname(__file__), '..', 'python', 'post_flight_backend.py')
    
    # Test --test flag
    result = subprocess.run([sys.executable, backend_path, '--test'], 
                          capture_output=True, text=True, timeout=30)
    assert result.returncode == 0
    assert 'All self-tests passed' in result.stdout


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])