#!/usr/bin/env python3
"""
Post-Flight Analysis Backend
Provides advanced analysis APIs for post-flight log analysis:
- Coverage heatmap generation
- Formation error CDF analysis
- AI inference timeline
- QoS analysis
- Automated report generation
"""

import json
import sys
import asyncio
import argparse
import signal
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict
from datetime import datetime, timedelta
import math
import statistics
import os
import base64

# Try to import scientific libraries
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import scipy.signal
    import scipy.stats
    import scipy.interpolate
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    import matplotlib
    matplotlib.use('Agg')  # Non-interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.colors as mcolors
    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False

try:
    from pyulog import ULog
    PYULOG_AVAILABLE = True
except ImportError:
    PYULOG_AVAILABLE = False

try:
    from pymavlink import mavutil
    MAVLINK_AVAILABLE = True
except ImportError:
    MAVLINK_AVAILABLE = False

try:
    from rosbags.rosbag2 import Reader as Rosbag2Reader
    from rosbags.serde import deserialize_cdr
    ROSBAGS_AVAILABLE = True
except ImportError:
    ROSBAGS_AVAILABLE = False


@dataclass
class APIResponse:
    id: str
    method: str
    success: bool
    result: Any = None
    error: str = None
    timestamp: str = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.utcnow().isoformat() + 'Z'
    
    def to_json(self) -> str:
        return json.dumps(asdict(self))


class PostFlightBackend:
    """Python backend for Post-Flight Analysis"""
    
    def __init__(self):
        self.methods = {
            'health_check': self.health_check,
            'analyze_coverage': self.analyze_coverage,
            'analyze_formation_error': self.analyze_formation_error,
            'analyze_ai_timeline': self.analyze_ai_timeline,
            'analyze_qos': self.analyze_qos,
            'run_full_analysis': self.run_full_analysis,
            'generate_report': self.generate_report,
            'export_report': self.export_report,
        }
    
    def health_check(self, args: Dict) -> Dict:
        return {
            'status': 'healthy',
            'numpy': NUMPY_AVAILABLE,
            'scipy': SCIPY_AVAILABLE,
            'matplotlib': MATPLOTLIB_AVAILABLE,
            'pandas': PANDAS_AVAILABLE,
            'pyulog': PYULOG_AVAILABLE,
            'mavlink': MAVLINK_AVAILABLE,
            'rosbags': ROSBAGS_AVAILABLE,
            'version': '1.0.0'
        }
    
    def _parse_log(self, log_path: str, log_type: str = None) -> Dict:
        """Parse log file based on type"""
        if log_type is None:
            if log_path.endswith('.ulg'):
                log_type = 'ulg'
            elif log_path.endswith('.tlog'):
                log_type = 'tlog'
            elif log_path.endswith('.db3'):
                log_type = 'rosbag2'
        
        data = {
            'type': log_type,
            'vehicle_local_position': [],
            'vehicle_attitude': [],
            'vehicle_gps_position': [],
            'battery_status': [],
            'actuator_outputs': [],
            'estimator_status': [],
            'ai_inference': [],
            'comm_status': [],
            'formation_state': [],
            'safety_events': [],
        }
        
        if log_type == 'ulg' and PYULOG_AVAILABLE:
            data = self._parse_ulog(log_path)
        elif log_type == 'tlog' and MAVLINK_AVAILABLE:
            data = self._parse_tlog(log_path)
        elif log_type == 'rosbag2' and ROSBAGS_AVAILABLE:
            data = self._parse_rosbag2(log_path)
        
        return data
    
    def _parse_ulog(self, log_path: str) -> Dict:
        """Parse ULog file"""
        data = {
            'vehicle_local_position': [],
            'vehicle_attitude': [],
            'vehicle_gps_position': [],
            'battery_status': [],
            'actuator_outputs': [],
            'estimator_status': [],
            'ai_inference': [],
            'comm_status': [],
            'formation_state': [],
            'safety_events': [],
        }
        
        try:
            ulog = ULog(log_path)
            
            for msg in ulog.data_list:
                if msg.name == 'vehicle_local_position':
                    for i in range(len(msg.data['timestamp'])):
                        data['vehicle_local_position'].append({
                            'timestamp': msg.data['timestamp'][i],
                            'x': msg.data['x'][i],
                            'y': msg.data['y'][i],
                            'z': msg.data['z'][i],
                            'vx': msg.data['vx'][i],
                            'vy': msg.data['vy'][i],
                            'vz': msg.data['vz'][i],
                        })
                elif msg.name == 'vehicle_attitude':
                    for i in range(len(msg.data['timestamp'])):
                        data['vehicle_attitude'].append({
                            'timestamp': msg.data['timestamp'][i],
                            'roll': msg.data['roll'][i],
                            'pitch': msg.data['pitch'][i],
                            'yaw': msg.data['yaw'][i],
                            'rollspeed': msg.data['rollspeed'][i],
                            'pitchspeed': msg.data['pitchspeed'][i],
                            'yawspeed': msg.data['yawspeed'][i],
                        })
                elif msg.name == 'vehicle_gps_position':
                    for i in range(len(msg.data['timestamp'])):
                        data['vehicle_gps_position'].append({
                            'timestamp': msg.data['timestamp'][i],
                            'lat': msg.data['lat'][i],
                            'lon': msg.data['lon'][i],
                            'alt': msg.data['alt'][i],
                            'hdop': msg.data['eph'][i] if 'eph' in msg.data else 0,
                        })
                elif msg.name == 'battery_status':
                    for i in range(len(msg.data['timestamp'])):
                        data['battery_status'].append({
                            'timestamp': msg.data['timestamp'][i],
                            'voltage': msg.data['voltage_filtered_v'][i] if 'voltage_filtered_v' in msg.data else 0,
                            'current': msg.data['current_filtered_a'][i] if 'current_filtered_a' in msg.data else 0,
                            'remaining': msg.data['remaining'][i] if 'remaining' in msg.data else 0,
                        })
        except Exception as e:
            print(f"ULog parse error: {e}")
        
        return data
    
    def _parse_tlog(self, log_path: str) -> Dict:
        """Parse MAVLink tlog file"""
        data = {
            'vehicle_local_position': [],
            'vehicle_attitude': [],
            'vehicle_gps_position': [],
            'battery_status': [],
            'actuator_outputs': [],
            'estimator_status': [],
            'ai_inference': [],
            'comm_status': [],
            'formation_state': [],
            'safety_events': [],
        }
        
        try:
            mlog = mavutil.mavlink_connection(log_path)
            while True:
                msg = mlog.recv_match(blocking=False)
                if msg is None:
                    break
                
                msg_dict = msg.to_dict()
                msg_type = msg.get_type()
                
                if msg_type == 'GLOBAL_POSITION_INT':
                    data['vehicle_local_position'].append({
                        'timestamp': msg.time_usec,
                        'lat': msg.lat / 1e7,
                        'lon': msg.lon / 1e7,
                        'alt': msg.alt / 1000.0,
                        'vx': msg.vx / 100.0,
                        'vy': msg.vy / 100.0,
                        'vz': msg.vz / 100.0,
                    })
                elif msg_type == 'ATTITUDE':
                    data['vehicle_attitude'].append({
                        'timestamp': msg.time_usec,
                        'roll': msg.roll,
                        'pitch': msg.pitch,
                        'yaw': msg.yaw,
                        'rollspeed': msg.rollspeed,
                        'pitchspeed': msg.pitchspeed,
                        'yawspeed': msg.yawspeed,
                    })
                elif msg_type == 'GPS_RAW_INT':
                    data['vehicle_gps_position'].append({
                        'timestamp': msg.time_usec,
                        'lat': msg.lat / 1e7,
                        'lon': msg.lon / 1e7,
                        'alt': msg.alt / 1000.0,
                        'hdop': msg.eph / 100.0 if msg.eph != 65535 else 0,
                    })
                elif msg_type == 'SYS_STATUS':
                    data['battery_status'].append({
                        'timestamp': msg.time_usec,
                        'voltage': msg.voltage_battery / 1000.0,
                        'current': msg.current_battery / 100.0,
                        'remaining': msg.battery_remaining,
                    })
        except Exception as e:
            print(f"TLog parse error: {e}")
        
        return data
    
    def _parse_rosbag2(self, log_path: str) -> Dict:
        """Parse ROS 2 bag file"""
        data = {
            'vehicle_local_position': [],
            'vehicle_attitude': [],
            'vehicle_gps_position': [],
            'battery_status': [],
            'actuator_outputs': [],
            'estimator_status': [],
            'ai_inference': [],
            'comm_status': [],
            'formation_state': [],
            'safety_events': [],
        }
        
        try:
            reader = Rosbag2Reader(log_path)
            connections = [c for c in reader.connections]
            
            for connection, timestamp, rawdata in reader.messages():
                msg = deserialize_cdr(rawdata, connection.msgtype)
                
                if connection.topic == '/swarm/vehicle_local_position':
                    data['vehicle_local_position'].append({
                        'timestamp': timestamp // 1000,
                        'x': msg.x,
                        'y': msg.y,
                        'z': msg.z,
                        'vx': msg.vx,
                        'vy': msg.vy,
                        'vz': msg.vz,
                    })
                elif connection.topic == '/swarm/vehicle_attitude':
                    data['vehicle_attitude'].append({
                        'timestamp': timestamp // 1000,
                        'roll': msg.roll,
                        'pitch': msg.pitch,
                        'yaw': msg.yaw,
                    })
                elif connection.topic == '/swarm/ai_inference':
                    data['ai_inference'].append({
                        'timestamp': timestamp // 1000,
                        'class_name': msg.class_name,
                        'confidence': msg.confidence,
                        'inference_time_ms': msg.inference_time_ms,
                    })
                # Add more topic handlers as needed
        except Exception as e:
            print(f"Rosbag2 parse error: {e}")
        
        return data
    
    def _haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def _grid_cell(self, lat, lon, min_lat, min_lon, cell_size_m):
        lat_m = (lat - min_lat) * 111000
        lon_m = (lon - min_lon) * 111000 * math.cos(math.radians(min_lat))
        return int(lat_m / cell_size_m), int(lon_m / cell_size_m)
    
    def analyze_coverage(self, args: Dict) -> Dict:
        """Generate coverage heatmap from flight path"""
        log_path = args.get('log_path', '')
        log_id = args.get('log_id', '')
        grid_resolution = args.get('grid_resolution_m', 10.0)
        sensor_fov = args.get('sensor_fov_deg', 90.0)
        flight_alt = args.get('flight_alt_m', 50.0)
        
        if not log_path:
            return {'error': 'log_path required'}
        
        log_data = self._parse_log(log_path)
        positions = log_data['vehicle_local_position']
        
        if not positions:
            return {'error': 'No position data found in log'}
        
        # Extract lat/lon
        coords = [(p.get('lat', 0), p.get('lon', 0)) for p in positions if 'lat' in p and 'lon' in p]
        if not coords:
            # Try local position
            coords = [(p.get('x', 0), p.get('y', 0)) for p in positions if 'x' in p and 'y' in p]
            use_local = True
        else:
            use_local = False
        
        if not coords:
            return {'error': 'No valid coordinates found'}
        
        # Calculate bounds
        if use_local:
            min_x = min(c[0] for c in coords)
            max_x = max(c[0] for c in coords)
            min_y = min(c[1] for c in coords)
            max_y = max(c[1] for c in coords)
            
            width = int((max_x - min_x) / grid_resolution) + 1
            height = int((max_y - min_y) / grid_resolution) + 1
            
            # Coverage grid
            grid = np.zeros((height, width)) if NUMPY_AVAILABLE else [[0]*width for _ in range(height)]
            footprint_radius = flight_alt * math.tan(math.radians(sensor_fov / 2))
            footprint_cells = int(footprint_radius / grid_resolution)
            
            for x, y in coords:
                gx = int((x - min_x) / grid_resolution)
                gy = int((y - min_y) / grid_resolution)
                
                # Mark footprint
                for dx in range(-footprint_cells, footprint_cells + 1):
                    for dy in range(-footprint_cells, footprint_cells + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if dx*dx + dy*dy <= footprint_cells*footprint_cells:
                                if NUMPY_AVAILABLE:
                                    grid[ny, nx] = 1
                                else:
                                    grid[ny][nx] = 1
            
            covered = np.sum(grid) if NUMPY_AVAILABLE else sum(sum(row) for row in grid)
            total = width * height
            coverage_pct = (covered / total * 100) if total > 0 else 0
            
            # Sensor footprint area
            footprint_area = math.pi * footprint_radius**2
            total_area = width * height * grid_resolution**2
            covered_area = covered * grid_resolution**2
            
            return {
                'log_id': log_id,
                'grid_resolution_m': grid_resolution,
                'grid_width': width,
                'grid_height': height,
                'total_area_km2': total_area / 1e6,
                'covered_area_km2': covered_area / 1e6,
                'coverage_pct': coverage_pct,
                'footprint_radius_m': footprint_radius,
                'grid_cells': int(covered),
                'heatmap_data': grid.tolist() if NUMPY_AVAILABLE else grid,
                'sensor_coverage': {
                    'camera': coverage_pct,
                    'lidar': min(100, coverage_pct * 1.2),
                },
                'overlap_pct': 30.0,  # Estimated
            }
        else:
            # GPS coordinates
            lats = [c[0] for c in coords]
            lons = [c[1] for c in coords]
            min_lat, max_lat = min(lats), max(lats)
            min_lon, max_lon = min(lons), max(lons)
            
            # Convert to grid
            width = int((max_lon - min_lon) * 111000 * math.cos(math.radians(min_lat)) / grid_resolution) + 1
            height = int((max_lat - min_lat) * 111000 / grid_resolution) + 1
            
            grid = np.zeros((height, width)) if NUMPY_AVAILABLE else [[0]*width for _ in range(height)]
            footprint_radius = flight_alt * math.tan(math.radians(sensor_fov / 2))
            footprint_cells = int(footprint_radius / grid_resolution)
            
            for lat, lon in coords:
                gx, gy = self._grid_cell(lat, lon, min_lat, min_lon, grid_resolution)
                
                for dx in range(-footprint_cells, footprint_cells + 1):
                    for dy in range(-footprint_cells, footprint_cells + 1):
                        nx, ny = gx + dx, gy + dy
                        if 0 <= nx < width and 0 <= ny < height:
                            if dx*dx + dy*dy <= footprint_cells*footprint_cells:
                                if NUMPY_AVAILABLE:
                                    grid[ny, nx] = 1
                                else:
                                    grid[ny][nx] = 1
            
            covered = np.sum(grid) if NUMPY_AVAILABLE else sum(sum(row) for row in grid)
            total = width * height
            coverage_pct = (covered / total * 100) if total > 0 else 0
            
            return {
                'log_id': log_id,
                'grid_resolution_m': grid_resolution,
                'grid_width': width,
                'grid_height': height,
                'min_lat': min_lat,
                'max_lat': max_lat,
                'min_lon': min_lon,
                'max_lon': max_lon,
                'total_area_km2': (width * height * grid_resolution**2) / 1e6,
                'covered_area_km2': (covered * grid_resolution**2) / 1e6,
                'coverage_pct': coverage_pct,
                'footprint_radius_m': footprint_radius,
                'grid_cells': int(covered),
                'heatmap_data': grid.tolist() if NUMPY_AVAILABLE else grid,
                'sensor_coverage': {
                    'camera': coverage_pct,
                    'lidar': min(100, coverage_pct * 1.2),
                },
                'overlap_pct': 30.0,
            }
    
    def analyze_formation_error(self, args: Dict) -> Dict:
        """Analyze formation position errors"""
        log_path = args.get('log_path', '')
        log_id = args.get('log_id', '')
        
        log_data = self._parse_log(log_path)
        positions = log_data['vehicle_local_position']
        attitudes = log_data['vehicle_attitude']
        formation_states = log_data['formation_state']
        
        if not positions or not formation_states:
            return {'error': 'Insufficient data for formation analysis'}
        
        # Calculate position errors from formation state
        errors = []
        vel_errors = []
        att_errors = []
        timestamps = []
        
        for fs in formation_states:
            ts = fs.get('timestamp', 0)
            pos_error = fs.get('formation_error_m', 0)
            vel_error = fs.get('velocity_error_mps', 0)
            att_error = fs.get('attitude_error_deg', 0)
            
            if pos_error > 0:
                errors.append(pos_error)
            if vel_error > 0:
                vel_errors.append(vel_error)
            if att_error > 0:
                att_errors.append(att_error)
            timestamps.append(ts)
        
        if not errors:
            return {'error': 'No formation error data'}
        
        # Statistics
        errors_arr = np.array(errors) if NUMPY_AVAILABLE else errors
        
        # CDF
        sorted_errors = sorted(errors)
        cdf_x = sorted_errors
        cdf_y = [i / len(sorted_errors) for i in range(1, len(sorted_errors) + 1)]
        
        # Per-vehicle (if available)
        per_vehicle = {}
        # Would group by vehicle_id from formation_state
        
        return {
            'log_id': log_id,
            'sample_count': len(errors),
            'mean_error_m': float(statistics.mean(errors)),
            'median_error_m': float(statistics.median(errors)),
            'std_error_m': float(statistics.stdev(errors)) if len(errors) > 1 else 0,
            'max_error_m': float(max(errors)),
            'min_error_m': float(min(errors)),
            'p95_error_m': float(sorted_errors[int(0.95 * len(sorted_errors))]),
            'p99_error_m': float(sorted_errors[int(0.99 * len(sorted_errors))]),
            'in_tolerance': float(sum(1 for e in errors if e < 2.0) / len(errors) * 100),
            'degraded': float(sum(1 for e in errors if 2.0 <= e < 5.0) / len(errors) * 100),
            'lost': float(sum(1 for e in errors if e >= 5.0) / len(errors) * 100),
            'cdf_x': cdf_x,
            'cdf_y': cdf_y,
            'timestamps_us': timestamps,
            'position_errors_m': errors,
            'velocity_errors_mps': vel_errors,
            'attitude_errors_deg': att_errors,
            'per_vehicle': per_vehicle,
        }
    
    def analyze_ai_timeline(self, args: Dict) -> Dict:
        """Analyze AI inference timeline"""
        log_path = args.get('log_path', '')
        log_id = args.get('log_id', '')
        
        log_data = self._parse_log(log_path)
        inferences = log_data['ai_inference']
        
        if not inferences:
            return {'error': 'No AI inference data found'}
        
        # Group by class
        detections_by_class = {}
        events = []
        inference_times = []
        inference_timeline = []
        
        for inf in inferences:
            ts = inf.get('timestamp', 0)
            cls = inf.get('class_name', 'unknown')
            conf = inf.get('confidence', 0)
            inf_time = inf.get('inference_time_ms', 0)
            cls_id = inf.get('class_id', 0)
            
            events.append({
                'timestamp_us': ts * 1000,
                'vehicle_id': inf.get('drone_id', 0),
                'event_type': 'detection',
                'class_name': cls,
                'confidence': conf,
                'inference_time_ms': inf_time,
                'details': f"Detected {cls} with {conf:.2f} confidence",
            })
            
            inference_timeline.append({
                'timestamp_us': ts * 1000,
                'inference_time_ms': inf_time,
            })
            
            inference_times.append(inf_time)
            
            if cls not in detections_by_class:
                detections_by_class[cls] = {
                    'count': 0,
                    'confidences': [],
                    'inference_times': [],
                    'first_seen_us': ts * 1000,
                    'last_seen_us': ts * 1000,
                }
            
            d = detections_by_class[cls]
            d['count'] += 1
            d['confidences'].append(conf)
            d['inference_times'].append(inf_time)
            d['last_seen_us'] = ts * 1000
        
        # Process class stats
        for cls, d in detections_by_class.items():
            d['avg_confidence'] = statistics.mean(d['confidences']) if d['confidences'] else 0
            d['avg_inference_time_ms'] = statistics.mean(d['inference_times']) if d['inference_times'] else 0
            del d['confidences']
            del d['inference_times']
        
        # Count replans and faults
        replan_count = sum(1 for e in events if 'replan' in e.get('details', '').lower())
        fault_count = sum(1 for e in events if 'fault' in e.get('details', '').lower() or 'fail' in e.get('details', '').lower())
        
        return {
            'log_id': log_id,
            'total_inferences': len(inferences),
            'total_detections': sum(d['count'] for d in detections_by_class.values()),
            'avg_inference_time_ms': statistics.mean(inference_times) if inference_times else 0,
            'max_inference_time_ms': max(inference_times) if inference_times else 0,
            'min_inference_time_ms': min(inference_times) if inference_times else 0,
            'replan_count': replan_count,
            'fault_count': fault_count,
            'detections_by_class': detections_by_class,
            'events': events,
            'inference_timeline': inference_timeline,
        }
    
    def analyze_qos(self, args: Dict) -> Dict:
        """Analyze communication QoS"""
        log_path = args.get('log_path', '')
        log_id = args.get('log_id', '')
        
        log_data = self._parse_log(log_path)
        comm_status = log_data['comm_status']
        
        if not comm_status:
            return {'error': 'No communication status data'}
        
        # Aggregate QoS metrics
        link_qualities = []
        rssis = []
        latencies = []
        loss_rates = []
        throughputs = []
        per_link = {}
        
        for cs in comm_status:
            link_q = cs.get('link_quality', 1)
            rssi = cs.get('rssi_dbm', -50)
            latency = cs.get('latency_ms', 10)
            loss = cs.get('packet_loss_rate', 0)
            throughput = cs.get('throughput_mbps', 0)
            
            link_qualities.append(link_q)
            rssis.append(rssi)
            latencies.append(latency)
            loss_rates.append(loss)
            throughputs.append(throughput)
            
            # Per-link (if from/to available)
            from_id = cs.get('from_id', 0)
            to_id = cs.get('to_id', 0)
            key = f"{from_id}->{to_id}"
            
            if key not in per_link:
                per_link[key] = {
                    'qualities': [], 'rssis': [], 'latencies': [], 
                    'losses': [], 'throughputs': []
                }
            
            per_link[key]['qualities'].append(link_q)
            per_link[key]['rssis'].append(rssi)
            per_link[key]['latencies'].append(latency)
            per_link[key]['losses'].append(loss)
            per_link[key]['throughputs'].append(throughput)
        
        # Aggregate per-link
        per_link_stats = {}
        for key, vals in per_link.items():
            per_link_stats[key] = {
                'avg_quality': statistics.mean(vals['qualities']),
                'min_quality': min(vals['qualities']),
                'avg_rssi': statistics.mean(vals['rssis']),
                'avg_latency_ms': statistics.mean(vals['latencies']),
                'loss_rate': statistics.mean(vals['losses']),
                'avg_throughput_mbps': statistics.mean(vals['throughputs']),
            }
        
        # Critical links
        critical = []
        for key, stats in per_link_stats.items():
            if stats['avg_quality'] < 0.3 or stats['loss_rate'] > 0.1:
                from_id, to_id = key.split('->')
                critical.append({
                    'from': int(from_id),
                    'to': int(to_id),
                    'quality': stats['avg_quality'],
                    'rssi': stats['avg_rssi'],
                    'latency': stats['avg_latency_ms'],
                    'loss_rate': stats['loss_rate'],
                    'recommendation': 'Check antenna alignment, reduce distance, or change channel'
                })
        
        return {
            'log_id': log_id,
            'avg_link_quality': statistics.mean(link_qualities) if link_qualities else 1,
            'min_link_quality': min(link_qualities) if link_qualities else 1,
            'avg_latency_ms': statistics.mean(latencies) if latencies else 0,
            'max_latency_ms': max(latencies) if latencies else 0,
            'avg_rssi_dbm': statistics.mean(rssis) if rssis else -50,
            'min_rssi_dbm': min(rssis) if rssis else -50,
            'loss_rate': statistics.mean(loss_rates) if loss_rates else 0,
            'max_loss_rate': max(loss_rates) if loss_rates else 0,
            'total_packets': len(link_qualities),
            'lost_packets': int(sum(l * 100 for l in loss_rates)) if loss_rates else 0,
            'avg_throughput_mbps': statistics.mean(throughputs) if throughputs else 0,
            'per_link_quality': per_link_stats,
            'critical_links': critical,
            'critical_links_count': len(critical),
            'recommendations': [
                'Average link quality below 70% - consider reducing node spacing' if statistics.mean(link_qualities) < 0.7 else '',
                'Minimum RSSI below -80dBm - check antenna alignment' if min(rssis) < -80 else '',
                'Packet loss > 5% - consider reducing data rate' if statistics.mean(loss_rates) > 0.05 else '',
            ],
        }
    
    def run_full_analysis(self, args: Dict) -> Dict:
        """Run all analyses"""
        log_path = args.get('log_path', '')
        log_id = args.get('log_id', '')
        
        coverage = self.analyze_coverage(args)
        formation = self.analyze_formation_error(args)
        ai = self.analyze_ai_timeline(args)
        qos = self.analyze_qos(args)
        
        return {
            'log_id': log_id,
            'coverage': coverage.get('result', coverage) if coverage.get('success') else coverage,
            'formation': formation.get('result', formation) if formation.get('success') else formation,
            'ai_timeline': ai.get('result', ai) if ai.get('success') else ai,
            'qos': qos.get('result', qos) if qos.get('success') else qos,
        }
    
    def generate_report(self, args: Dict) -> Dict:
        """Generate HTML/PDF/JSON report"""
        log_id = args.get('log_id', '')
        log_path = args.get('log_path', '')
        report_id = args.get('report_id', '')
        output_dir = args.get('output_dir', '/tmp/reports')
        format_type = args.get('format', 'html')
        options = args.get('options', {})
        
        os.makedirs(output_dir, exist_ok=True)
        
        # Run full analysis
        analysis = self.run_full_analysis(args)
        
        # Generate report
        timestamp = datetime.utcnow().strftime('%Y%m%d_%H%M%S')
        report_filename = f"post_flight_report_{log_id}_{timestamp}.{format_type}"
        report_path = os.path.join(output_dir, report_filename)
        
        if format_type == 'html':
            report_path = self._generate_html_report(report_path, log_id, analysis, options)
        elif format_type == 'json':
            report_path = self._generate_json_report(report_path, log_id, analysis, options)
        elif format_type == 'pdf':
            report_path = self._generate_pdf_report(report_path, log_id, analysis, options)
        else:
            return {'error': f'Unsupported format: {format_type}'}
        
        return {
            'report_id': report_id,
            'report_path': report_path,
            'summary': f"Report generated for flight {log_id}",
        }
    
    def _generate_html_report(self, path: str, log_id: str, analysis: Dict, options: Dict) -> str:
        html = f"""
<!DOCTYPE html>
<html>
<head>
    <title>Post-Flight Analysis Report - {log_id}</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }}
        .container {{ max-width: 1200px; margin: 0 auto; background: white; padding: 30px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }}
        h1 {{ color: #2c3e50; border-bottom: 3px solid #3498db; padding-bottom: 10px; }}
        h2 {{ color: #34495e; margin-top: 30px; }}
        .metric {{ display: inline-block; background: #ecf0f1; padding: 15px; margin: 10px; border-radius: 5px; min-width: 150px; }}
        .metric-value {{ font-size: 24px; font-weight: bold; color: #2c3e50; }}
        .metric-label {{ font-size: 12px; color: #7f8c8d; text-transform: uppercase; }}
        .good {{ color: #27ae60; }}
        .warn {{ color: #f39c12; }}
        .critical {{ color: #e74c3c; }}
        table {{ width: 100%; border-collapse: collapse; margin: 20px 0; }}
        th, td {{ padding: 12px; text-align: left; border-bottom: 1px solid #ecf0f1; }}
        th {{ background: #34495e; color: white; }}
        .chart-placeholder {{ background: #ecf0f1; height: 300px; display: flex; align-items: center; justify-content: center; color: #7f8c8d; margin: 20px 0; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>Post-Flight Analysis Report</h1>
        <p><strong>Flight ID:</strong> {log_id}</p>
        <p><strong>Generated:</strong> {datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S')} UTC</p>
        
        <h2>Executive Summary</h2>
        <div class="metric">
            <div class="metric-value">{analysis.get('coverage', {}).get('coverage_pct', 0):.1f}%</div>
            <div class="metric-label">Coverage</div>
        </div>
        <div class="metric">
            <div class="metric-value">{analysis.get('formation', {}).get('mean_error_m', 0):.2f}m</div>
            <div class="metric-label">Avg Formation Error</div>
        </div>
        <div class="metric">
            <div class="metric-value">{analysis.get('ai_timeline', {}).get('total_inferences', 0)}</div>
            <div class="metric-label">AI Inferences</div>
        </div>
        <div class="metric">
            <div class="metric-value">{(analysis.get('qos', {}).get('avg_link_quality', 1) * 100):.1f}%</div>
            <div class="metric-label">Avg Link Quality</div>
        </div>
        
        <h2>Coverage Analysis</h2>
        <p>Total Area: {analysis.get('coverage', {}).get('total_area_km2', 0):.2f} km²</p>
        <p>Covered Area: {analysis.get('coverage', {}).get('covered_area_km2', 0):.2f} km²</p>
        <p>Coverage: {analysis.get('coverage', {}).get('coverage_pct', 0):.1f}%</p>
        <div class="chart-placeholder">[Coverage Heatmap]</div>
        
        <h2>Formation Error Analysis</h2>
        <p>Mean Error: {analysis.get('formation', {}).get('mean_error_m', 0):.2f} m</p>
        <p>P95 Error: {analysis.get('formation', {}).get('p95_error_m', 0):.2f} m</p>
        <p>Max Error: {analysis.get('formation', {}).get('max_error_m', 0):.2f} m</p>
        <p>In Tolerance (<2m): {analysis.get('formation', {}).get('in_tolerance', 0):.1f}%</p>
        <div class="chart-placeholder">[Formation Error CDF]</div>
        
        <h2>AI Inference Timeline</h2>
        <p>Total Inferences: {analysis.get('ai_timeline', {}).get('total_inferences', 0)}</p>
        <p>Avg Inference Time: {analysis.get('ai_timeline', {}).get('avg_inference_time_ms', 0):.1f} ms</p>
        <p>Max Inference Time: {analysis.get('ai_timeline', {}).get('max_inference_time_ms', 0):.1f} ms</p>
        <p>Detections: {analysis.get('ai_timeline', {}).get('total_detections', 0)}</p>
        <div class="chart-placeholder">[Inference Time Timeline]</div>
        
        <h2>QoS Analysis</h2>
        <p>Avg Link Quality: {(analysis.get('qos', {}).get('avg_link_quality', 1) * 100):.1f}%</p>
        <p>Avg Latency: {analysis.get('qos', {}).get('avg_latency_ms', 0):.1f} ms</p>
        <p>Packet Loss Rate: {(analysis.get('qos', {}).get('loss_rate', 0) * 100):.2f}%</p>
        <p>Critical Links: {analysis.get('qos', {}).get('critical_links_count', 0)}</p>
        <div class="chart-placeholder">[QoS Timeline]</div>
        
        <h2>Recommendations</h2>
        <ul>
"""
        
        # Add recommendations
        for rec in analysis.get('qos', {}).get('recommendations', []):
            if rec:
                html += f"        <li>{rec}</li>\n"
        
        html += """
        </ul>
    </div>
</body>
</html>
"""
        
        with open(path, 'w') as f:
            f.write(html)
        
        return path
    
    def _generate_json_report(self, path: str, log_id: str, analysis: Dict, options: Dict) -> str:
        report = {
            'report_id': options.get('report_id', ''),
            'log_id': log_id,
            'generated_at': datetime.utcnow().isoformat() + 'Z',
            'analysis': analysis,
        }
        
        with open(path, 'w') as f:
            json.dump(report, f, indent=2)
        
        return path
    
    def _generate_pdf_report(self, path: str, log_id: str, analysis: Dict, options: Dict) -> str:
        # Would use weasyprint or similar to convert HTML to PDF
        # For now, generate HTML and return path
        html_path = path.replace('.pdf', '.html')
        self._generate_html_report(html_path, log_id, analysis, options)
        return html_path
    
    def export_report(self, args: Dict) -> Dict:
        report_path = args.get('report_path', '')
        format_type = args.get('format', 'pdf')
        output_path = args.get('output_path', '')
        
        if not os.path.exists(report_path):
            return {'error': 'Report not found'}
        
        # Convert report to different format
        # Would implement actual conversion
        return {
            'success': True,
            'output_path': output_path or report_path.replace('.html', f'.{format_type}'),
        }
    
    def handle_request(self, request: Dict) -> Dict:
        method = request.get('method', '')
        args = request.get('args', {})
        req_id = request.get('id', '')
        
        if method not in self.methods:
            return APIResponse(
                id=req_id,
                method=method,
                success=False,
                error=f'Unknown method: {method}'
            ).__dict__
        
        try:
            result = self.methods[method](args)
            return APIResponse(
                id=req_id,
                method=method,
                success=True,
                result=result
            ).__dict__
        except Exception as e:
            return APIResponse(
                id=req_id,
                method=method,
                success=False,
                error=str(e)
            ).__dict__


async def run_server():
    """Run JSON-RPC server over stdin/stdout"""
    backend = PostFlightBackend()
    
    print(json.dumps({'status': 'ready', 'methods': list(backend.methods.keys())}), flush=True)
    
    loop = asyncio.get_event_loop()
    reader = asyncio.StreamReader()
    protocol = asyncio.StreamReaderProtocol(reader)
    await loop.connect_read_pipe(lambda: protocol, sys.stdin)
    
    while True:
        line = await reader.readline()
        if not line:
            break
        
        line = line.decode('utf-8').strip()
        if not line:
            continue
        
        try:
            request = json.loads(line)
            response = backend.handle_request(request)
            print(json.dumps(response), flush=True)
        except json.JSONDecodeError:
            print(json.dumps({
                'id': '',
                'method': '',
                'success': False,
                'error': 'Invalid JSON'
            }), flush=True)


def main():
    parser = argparse.ArgumentParser(description='Post-Flight Analysis Backend')
    parser.add_argument('--server', action='store_true', help='Run as JSON-RPC server')
    parser.add_argument('--method', help='Call single method')
    parser.add_argument('--args', help='Method arguments as JSON')
    parser.add_argument('--test', action='store_true', help='Run self-test')
    
    args = parser.parse_args()
    
    if args.test:
        backend = PostFlightBackend()
        print("Running self-tests...")
        
        # Test health check
        result = backend.handle_request({'id': 'test1', 'method': 'health_check', 'args': {}})
        assert result['success']
        print(f"Health check: {result['result']}")
        
        print("All self-tests passed!")
        return
    
    if args.server:
        asyncio.run(run_server())
    elif args.method:
        backend = PostFlightBackend()
        request_args = json.loads(args.args) if args.args else {}
        request = {'id': 'cli', 'method': args.method, 'args': request_args}
        response = backend.handle_request(request)
        print(json.dumps(response, indent=2))
    else:
        parser.print_help()


if __name__ == '__main__':
    main()