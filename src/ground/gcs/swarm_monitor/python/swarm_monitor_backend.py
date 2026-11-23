#!/usr/bin/env python3
"""
Swarm Monitor - Python Backend
Provides advanced analysis APIs for swarm monitoring:
- Real-time formation analysis
- Health trend analysis
- Mesh QoS analysis
- Safety event correlation
- Predictive maintenance
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

# Try to import scientific libraries
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import scipy.signal
    import scipy.stats
    import scipy.fft
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False

try:
    import pandas as pd
    PANDAS_AVAILABLE = True
except ImportError:
    PANDAS_AVAILABLE = False


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


class SwarmMonitorBackend:
    """Python backend for Swarm Monitor - Advanced analysis APIs"""
    
    def __init__(self):
        self.methods = {
            'health_check': self.health_check,
            'analyze_formation': self.analyze_formation,
            'analyze_health_trends': self.analyze_health_trends,
            'analyze_mesh_qos': self.analyze_mesh_qos,
            'analyze_safety_events': self.analyze_safety_events,
            'predict_battery_failure': self.predict_battery_failure,
            'predict_link_failure': self.predict_link_failure,
            'detect_anomalies': self.detect_anomalies,
            'correlate_events': self.correlate_events,
            'generate_health_report': self.generate_health_report,
        }
    
    def health_check(self, args: Dict) -> Dict:
        return {
            'status': 'healthy',
            'numpy': NUMPY_AVAILABLE,
            'scipy': SCIPY_AVAILABLE,
            'pandas': PANDAS_AVAILABLE,
            'version': '1.0.0'
        }
    
    def analyze_formation(self, args: Dict) -> Dict:
        """Analyze formation quality and stability"""
        vehicles = args.get('vehicles', [])
        formation_type = args.get('formation_type', 'line')
        assigned_positions = args.get('assigned_positions', {})
        
        if not vehicles:
            return {'error': 'No vehicle data provided'}
        
        # Calculate position errors
        errors = []
        for v in vehicles:
            vid = str(v.get('id', 0))
            if vid in assigned_positions:
                target = assigned_positions[vid]
                actual_lat = v.get('lat', 0)
                actual_lon = v.get('lon', 0)
                target_lat = target.get('lat', 0)
                target_lon = target.get('lon', 0)
                
                # Haversine distance
                error = self._haversine(actual_lat, actual_lon, target_lat, target_lon)
                errors.append({
                    'vehicle_id': vid,
                    'error_m': error,
                    'callsign': v.get('callsign', '')
                })
        
        if not errors:
            return {'error': 'No assigned positions for vehicles'}
        
        error_values = [e['error_m'] for e in errors]
        
        # Stability metrics
        result = {
            'formation_type': formation_type,
            'vehicle_count': len(vehicles),
            'avg_error_m': statistics.mean(error_values),
            'max_error_m': max(error_values),
            'min_error_m': min(error_values),
            'std_error_m': statistics.stdev(error_values) if len(error_values) > 1 else 0,
            'median_error_m': statistics.median(error_values),
            'vehicles_in_tolerance': sum(1 for e in error_values if e < 2.0),
            'vehicles_degraded': sum(1 for e in error_values if 2.0 <= e < 5.0),
            'vehicles_lost': sum(1 for e in error_values if e >= 5.0),
            'per_vehicle': errors,
            'stability_score': max(0, 100 - statistics.mean(error_values) * 10),
        }
        
        # Formation state classification
        if result['vehicles_lost'] > 0:
            result['formation_state'] = 'lost'
        elif result['vehicles_degraded'] > len(vehicles) * 0.3:
            result['formation_state'] = 'degraded'
        elif result['avg_error_m'] < 1.0:
            result['formation_state'] = 'excellent'
        elif result['avg_error_m'] < 2.0:
            result['formation_state'] = 'good'
        else:
            result['formation_state'] = 'holding'
        
        return result
    
    def analyze_health_trends(self, args: Dict) -> Dict:
        """Analyze health parameter trends over time"""
        history = args.get('history', [])  # List of {timestamp, vehicle_id, params...}
        window_minutes = args.get('window_minutes', 60)
        
        if not history:
            return {'error': 'No history data provided'}
        
        # Group by vehicle
        by_vehicle = {}
        for entry in history:
            vid = entry.get('vehicle_id', 0)
            if vid not in by_vehicle:
                by_vehicle[vid] = []
            by_vehicle[vid].append(entry)
        
        result = {}
        for vid, entries in by_vehicle.items():
            # Sort by timestamp
            entries.sort(key=lambda x: x.get('timestamp', 0))
            
            # Filter to window
            cutoff = time.time() - window_minutes * 60
            entries = [e for e in entries if e.get('timestamp', 0) > cutoff]
            
            if len(entries) < 2:
                continue
            
            # Analyze trends for each parameter
            params = ['battery_remaining_pct', 'battery_voltage_v', 'link_quality', 
                     'rssi_dbm', 'latency_ms', 'gps_satellites', 'ekf_innovation',
                     'inference_time_ms', 'formation_error_m']
            
            trends = {}
            for param in params:
                values = [e.get(param) for e in entries if e.get(param) is not None]
                if len(values) >= 3:
                    trends[param] = self._compute_trend(values)
            
            # Battery depletion rate
            battery_vals = [e.get('battery_remaining_pct') for e in entries if e.get('battery_remaining_pct') is not None]
            if len(battery_vals) >= 3:
                time_span = entries[-1].get('timestamp', 0) - entries[0].get('timestamp', 0)
                if time_span > 0:
                    battery_trend = self._compute_trend(battery_vals)
                    trends['battery_depletion_rate_pct_per_min'] = battery_trend['slope'] * 60 / time_span if time_span > 0 else 0
            
            result[vid] = {
                'callsign': entries[-1].get('callsign', ''),
                'data_points': len(entries),
                'time_span_min': time_span / 60 if time_span > 0 else 0,
                'trends': trends,
                'current': {p: entries[-1].get(p) for p in params if entries[-1].get(p) is not None},
                'alerts': self._generate_health_alerts(entries[-1], trends)
            }
        
        return result
    
    def analyze_mesh_qos(self, args: Dict) -> Dict:
        """Analyze mesh network QoS"""
        nodes = args.get('nodes', [])  # List of node QoS data
        
        if not nodes:
            return {'error': 'No mesh data provided'}
        
        # Aggregate link statistics
        all_links = []
        for node in nodes:
            node_id = node.get('node_id', 0)
            for target_id, quality in node.get('link_quality_to', {}).items():
                rssi = node.get('rssi_dbm_to', {}).get(target_id, -100)
                latency = node.get('latency_ms_to', {}).get(target_id, 0)
                sent = node.get('packets_sent_to', {}).get(target_id, 0)
                recv = node.get('packets_received_from', {}).get(target_id, 0)
                lost = node.get('packets_lost_to', {}).get(target_id, 0)
                
                all_links.append({
                    'from': node_id,
                    'to': target_id,
                    'quality': quality,
                    'rssi': rssi,
                    'latency': latency,
                    'sent': sent,
                    'recv': recv,
                    'lost': lost,
                    'loss_rate': lost / sent if sent > 0 else 0
                })
        
        if not all_links:
            return {'error': 'No link data'}
        
        qualities = [l['quality'] for l in all_links]
        rssis = [l['rssi'] for l in all_links]
        latencies = [l['latency'] for l in all_links]
        loss_rates = [l['loss_rate'] for l in all_links]
        
        return {
            'total_nodes': len(nodes),
            'total_links': len(all_links),
            'link_quality': {
                'mean': statistics.mean(qualities),
                'median': statistics.median(qualities),
                'std': statistics.stdev(qualities) if len(qualities) > 1 else 0,
                'min': min(qualities),
                'max': max(qualities),
                'below_50pct': sum(1 for q in qualities if q < 0.5),
                'below_30pct': sum(1 for q in qualities if q < 0.3),
            },
            'rssi_dbm': {
                'mean': statistics.mean(rssis),
                'median': statistics.median(rssis),
                'min': min(rssis),
                'max': max(rssis),
                'below_85': sum(1 for r in rssis if r < -85),
                'below_70': sum(1 for r in rssis if r < -70),
            },
            'latency_ms': {
                'mean': statistics.mean(latencies),
                'median': statistics.median(latencies),
                'max': max(latencies),
                'p95': self._percentile(latencies, 95),
            },
            'packet_loss': {
                'mean_rate': statistics.mean(loss_rates),
                'max_rate': max(loss_rates),
                'links_above_1pct': sum(1 for lr in loss_rates if lr > 0.01),
                'links_above_5pct': sum(1 for lr in loss_rates if lr > 0.05),
            },
            'critical_links': [l for l in all_links if l['quality'] < 0.3 or l['loss_rate'] > 0.1],
            'recommendations': self._mesh_recommendations(qualities, rssis, latencies, loss_rates)
        }
    
    def analyze_safety_events(self, args: Dict) -> Dict:
        """Analyze safety events for patterns"""
        events = args.get('events', [])
        window_hours = args.get('window_hours', 24)
        
        if not events:
            return {'error': 'No safety events provided'}
        
        cutoff = time.time() - window_hours * 3600
        recent = [e for e in events if e.get('timestamp_us', 0) / 1e6 > cutoff]
        
        by_type = {}
        by_severity = {}
        by_source = {}
        timeline = []
        
        for e in recent:
            etype = e.get('event_type', 'UNKNOWN')
            severity = e.get('severity', 'INFO')
            source = e.get('source_id', 0)
            
            by_type[etype] = by_type.get(etype, 0) + 1
            by_severity[severity] = by_severity.get(severity, 0) + 1
            by_source[source] = by_source.get(source, 0) + 1
            
            timeline.append({
                'time': e.get('timestamp_us', 0) / 1e6,
                'type': etype,
                'severity': severity,
                'source': source,
                'description': e.get('description', '')
            })
        
        timeline.sort(key=lambda x: x['time'])
        
        return {
            'window_hours': window_hours,
            'total_events': len(recent),
            'by_type': by_type,
            'by_severity': by_severity,
            'by_source': by_source,
            'unacknowledged': sum(1 for e in recent if not e.get('acknowledged', False)),
            'critical_count': by_severity.get('CRITICAL', 0),
            'error_count': by_severity.get('ERROR', 0),
            'timeline': timeline,
            'event_rate_per_hour': len(recent) / window_hours,
            'top_types': sorted(by_type.items(), key=lambda x: x[1], reverse=True)[:5],
            'top_sources': sorted(by_source.items(), key=lambda x: x[1], reverse=True)[:5],
        }
    
    def predict_battery_failure(self, args: Dict) -> Dict:
        """Predict battery depletion time"""
        vehicles = args.get('vehicles', [])
        safety_margin_pct = args.get('safety_margin_pct', 20)
        
        predictions = []
        for v in vehicles:
            battery_pct = v.get('battery_remaining_pct', 100)
            current_a = abs(v.get('battery_current_a', 0))
            capacity_mah = v.get('battery_capacity_mah', 5000)
            consumption_rate = v.get('battery_consumption_mah_per_min', 0)
            
            if consumption_rate <= 0 and current_a > 0:
                consumption_rate = current_a * 1000 / 60  # A to mAh/min
            
            if consumption_rate > 0 and battery_pct > safety_margin_pct:
                available_pct = battery_pct - safety_margin_pct
                available_mah = capacity_mah * available_pct / 100
                time_min = available_mah / consumption_rate
            else:
                time_min = float('inf') if battery_pct > safety_margin_pct else 0
            
            predictions.append({
                'vehicle_id': v.get('id', 0),
                'callsign': v.get('callsign', ''),
                'current_pct': battery_pct,
                'safety_margin_pct': safety_margin_pct,
                'consumption_rate_mah_min': consumption_rate,
                'estimated_time_min': time_min if time_min != float('inf') else None,
                'estimated_landing_time': datetime.utcnow() + timedelta(minutes=time_min) if time_min != float('inf') else None,
                'risk_level': 'LOW' if battery_pct > 50 else 'MEDIUM' if battery_pct > 30 else 'HIGH' if battery_pct > safety_margin_pct else 'CRITICAL'
            })
        
        return {
            'predictions': predictions,
            'critical_vehicles': [p for p in predictions if p['risk_level'] == 'CRITICAL'],
            'recommendations': [
                p['callsign'] + ': RTB recommended' 
                for p in predictions if p['risk_level'] in ['HIGH', 'CRITICAL']
            ]
        }
    
    def predict_link_failure(self, args: Dict) -> Dict:
        """Predict communication link degradation"""
        nodes = args.get('nodes', [])
        
        predictions = []
        for node in nodes:
            node_id = node.get('node_id', 0)
            links = node.get('link_quality_to', {})
            
            for target_id, quality in links.items():
                rssi = node.get('rssi_dbm_to', {}).get(target_id, -100)
                latency = node.get('latency_ms_to', {}).get(target_id, 0)
                loss_rate = node.get('packets_lost_to', {}).get(target_id, 0) / max(node.get('packets_sent_to', {}).get(target_id, 1), 1)
                
                # Simple risk assessment
                risk = 'LOW'
                if quality < 0.3 or rssi < -85 or loss_rate > 0.1:
                    risk = 'CRITICAL'
                elif quality < 0.5 or rssi < -75 or loss_rate > 0.05:
                    risk = 'HIGH'
                elif quality < 0.7 or rssi < -70 or loss_rate > 0.01:
                    risk = 'MEDIUM'
                
                predictions.append({
                    'from': node_id,
                    'to': target_id,
                    'quality': quality,
                    'rssi': rssi,
                    'latency': latency,
                    'loss_rate': loss_rate,
                    'risk': risk,
                    'time_to_failure_estimate': 'Imminent' if risk == 'CRITICAL' else 'Soon' if risk == 'HIGH' else 'Stable'
                })
        
        return {
            'predictions': predictions,
            'critical_links': [p for p in predictions if p['risk'] == 'CRITICAL'],
            'degraded_links': [p for p in predictions if p['risk'] in ['HIGH', 'MEDIUM']],
        }
    
    def detect_anomalies(self, args: Dict) -> Dict:
        """Statistical anomaly detection on telemetry"""
        data = args.get('data', [])  # Time series data
        threshold_std = args.get('threshold_std', 3.0)
        
        if not data:
            return {'error': 'No data provided'}
        
        anomalies = []
        for entry in data:
            vid = entry.get('vehicle_id', 0)
            timestamp = entry.get('timestamp', 0)
            
            # Check each numeric field
            for key, value in entry.items():
                if key in ['vehicle_id', 'timestamp', 'callsign']:
                    continue
                if not isinstance(value, (int, float)):
                    continue
                
                # Collect historical values for this vehicle/field
                historical = [d.get(key) for d in data if d.get('vehicle_id') == vid and isinstance(d.get(key), (int, float))]
                if len(historical) < 10:
                    continue
                
                mean = statistics.mean(historical)
                std = statistics.stdev(historical) if len(historical) > 1 else 0
                
                if std > 0 and abs(value - mean) > threshold_std * std:
                    anomalies.append({
                        'vehicle_id': vid,
                        'timestamp': timestamp,
                        'field': key,
                        'value': value,
                        'mean': mean,
                        'std': std,
                        'deviation_std': abs(value - mean) / std,
                        'severity': 'HIGH' if abs(value - mean) > 5 * std else 'MEDIUM'
                    })
        
        return {
            'anomalies': anomalies,
            'count': len(anomalies),
            'by_vehicle': {},
            'by_field': {}
        }
    
    def correlate_events(self, args: Dict) -> Dict:
        """Correlate safety events with telemetry"""
        events = args.get('events', [])
        telemetry = args.get('telemetry', [])  # Time-aligned telemetry
        window_seconds = args.get('window_seconds', 30)
        
        correlations = []
        for event in events:
            event_time = event.get('timestamp_us', 0) / 1e6
            
            # Find telemetry within window
            relevant = [t for t in telemetry 
                       if abs(t.get('timestamp', 0) - event_time) <= window_seconds]
            
            if relevant:
                # Analyze telemetry at event time
                avg_battery = statistics.mean([t.get('battery_remaining_pct', 100) for t in relevant])
                avg_link = statistics.mean([t.get('link_quality', 1) for t in relevant])
                max_formation_err = max([t.get('formation_error_m', 0) for t in relevant])
                
                correlations.append({
                    'event': event,
                    'telemetry_samples': len(relevant),
                    'avg_battery_pct': avg_battery,
                    'avg_link_quality': avg_link,
                    'max_formation_error_m': max_formation_err,
                    'possible_causes': self._identify_causes(event, relevant)
                })
        
        return {'correlations': correlations}
    
    def generate_health_report(self, args: Dict) -> Dict:
        """Generate comprehensive health report"""
        vehicles = args.get('vehicles', [])
        mesh = args.get('mesh', [])
        events = args.get('events', [])
        history = args.get('history', [])
        
        report = {
            'generated_at': datetime.utcnow().isoformat() + 'Z',
            'fleet_size': len(vehicles),
            'formation_analysis': self.analyze_formation({'vehicles': vehicles}) if vehicles else {},
            'health_trends': self.analyze_health_trends({'history': history}) if history else {},
            'mesh_qos': self.analyze_mesh_qos({'nodes': mesh}) if mesh else {},
            'safety_analysis': self.analyze_safety_events({'events': events}) if events else {},
            'battery_predictions': self.predict_battery_failure({'vehicles': vehicles}) if vehicles else {},
            'link_predictions': self.predict_link_failure({'nodes': mesh}) if mesh else {},
            'anomalies': self.detect_anomalies({'data': history}) if history else {},
            'overall_status': 'HEALTHY'
        }
        
        # Determine overall status
        critical_count = report.get('safety_analysis', {}).get('critical_count', 0)
        battery_critical = len(report.get('battery_predictions', {}).get('critical_vehicles', []))
        link_critical = len(report.get('link_predictions', {}).get('critical_links', []))
        
        if critical_count > 0 or battery_critical > 0 or link_critical > 0:
            report['overall_status'] = 'CRITICAL'
        elif report.get('safety_analysis', {}).get('error_count', 0) > 0 or \
             len(report.get('battery_predictions', {}).get('predictions', [])) > 0:
            report['overall_status'] = 'DEGRADED'
        
        return report
    
    # Helper methods
    def _haversine(self, lat1, lon1, lat2, lon2):
        R = 6371000
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat/2)**2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon/2)**2
        return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))
    
    def _compute_trend(self, values):
        if len(values) < 2:
            return {'slope': 0, 'intercept': values[0] if values else 0, 'r2': 0}
        
        n = len(values)
        x = list(range(n))
        x_mean = statistics.mean(x)
        y_mean = statistics.mean(values)
        
        num = sum((x[i] - x_mean) * (values[i] - y_mean) for i in range(n))
        den = sum((x[i] - x_mean)**2 for i in range(n))
        
        if den == 0:
            return {'slope': 0, 'intercept': y_mean, 'r2': 0}
        
        slope = num / den
        intercept = y_mean - slope * x_mean
        
        # R-squared
        y_pred = [intercept + slope * xi for xi in x]
        ss_res = sum((values[i] - y_pred[i])**2 for i in range(n))
        ss_tot = sum((values[i] - y_mean)**2 for i in range(n))
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        
        return {'slope': slope, 'intercept': intercept, 'r2': max(0, r2)}
    
    def _percentile(self, data, p):
        if not data:
            return 0
        sorted_data = sorted(data)
        k = (len(sorted_data) - 1) * p / 100
        f = math.floor(k)
        c = math.ceil(k)
        if f == c:
            return sorted_data[int(k)]
        return sorted_data[f] * (c - k) + sorted_data[c] * (k - f)
    
    def _generate_health_alerts(self, current, trends):
        alerts = []
        if current.get('battery_remaining_pct', 100) < 20:
            alerts.append({'level': 'CRITICAL', 'message': 'Battery critically low'})
        elif current.get('battery_remaining_pct', 100) < 30:
            alerts.append({'level': 'HIGH', 'message': 'Battery low'})
        
        if current.get('link_quality', 1) < 0.3:
            alerts.append({'level': 'CRITICAL', 'message': 'Link quality critical'})
        elif current.get('link_quality', 1) < 0.5:
            alerts.append({'level': 'HIGH', 'message': 'Link quality degraded'})
        
        if current.get('gps_satellites', 20) < 6:
            alerts.append({'level': 'HIGH', 'message': 'GPS satellite count low'})
        
        if current.get('ekf_innovation', 0) > 1.0:
            alerts.append({'level': 'HIGH', 'message': 'EKF innovation high'})
        
        # Trend alerts
        for param, trend in trends.items():
            if 'battery' in param and trend.get('slope', 0) < -0.1:
                alerts.append({'level': 'MEDIUM', 'message': f'Battery depleting rapidly ({trend["slope"]:.2f}%/sample)'})
        
        return alerts
    
    def _mesh_recommendations(self, qualities, rssis, latencies, loss_rates):
        recs = []
        if statistics.mean(qualities) < 0.5:
            recs.append('Average link quality below 50% - consider reducing node spacing or increasing TX power')
        if min(rssis) < -85:
            recs.append('Minimum RSSI below -85dBm - check antenna alignment and obstacles')
        if statistics.mean(latencies) > 100:
            recs.append('Average latency >100ms - check for interference or channel congestion')
        if statistics.mean(loss_rates) > 0.01:
            recs.append('Packet loss >1% - consider reducing data rate or adding relay nodes')
        return recs
    
    def _identify_causes(self, event, telemetry):
        causes = []
        event_type = event.get('event_type', '')
        
        if event_type == 'GEOFENCE_BREACH':
            causes.append('Position drifted outside boundary')
            if any(t.get('formation_error_m', 0) > 5 for t in telemetry):
                causes.append('Formation error contributed to breach')
        
        elif event_type == 'LOW_BATTERY':
            causes.append('Battery below threshold')
            if any(t.get('battery_current_a', 0) > 20 for t in telemetry):
                causes.append('High current draw detected')
        
        elif event_type == 'LINK_LOSS':
            causes.append('Communication link lost')
            if any(t.get('link_quality', 1) < 0.3 for t in telemetry):
                causes.append('Link quality was degrading prior to loss')
        
        elif event_type == 'GPS_LOSS':
            causes.append('GPS fix lost')
            if any(t.get('gps_satellites', 20) < 6 for t in telemetry):
                causes.append('Low satellite count prior to loss')
        
        return causes
    
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
    backend = SwarmMonitorBackend()
    
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
    parser = argparse.ArgumentParser(description='Swarm Monitor Python Backend')
    parser.add_argument('--server', action='store_true', help='Run as JSON-RPC server')
    parser.add_argument('--method', help='Call single method')
    parser.add_argument('--args', help='Method arguments as JSON')
    parser.add_argument('--test', action='store_true', help='Run self-test')
    
    args = parser.parse_args()
    
    if args.test:
        backend = SwarmMonitorBackend()
        print("Running self-tests...")
        
        # Test health check
        result = backend.handle_request({'id': 'test1', 'method': 'health_check', 'args': {}})
        assert result['success']
        print(f"Health check: {result['result']}")
        
        # Test formation analysis
        result = backend.handle_request({
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
        assert result['success']
        print(f"Formation analysis: {result['result']['formation_state']}")
        
        # Test mesh QoS
        result = backend.handle_request({
            'id': 'test3',
            'method': 'analyze_mesh_qos',
            'args': {
                'nodes': [
                    {'node_id': 1, 'link_quality_to': {2: 0.9, 3: 0.7}, 'rssi_dbm_to': {2: -50, 3: -70}, 'latency_ms_to': {2: 10, 3: 25}},
                    {'node_id': 2, 'link_quality_to': {1: 0.9, 3: 0.8}, 'rssi_dbm_to': {1: -50, 3: -60}, 'latency_ms_to': {1: 10, 3: 15}},
                    {'node_id': 3, 'link_quality_to': {1: 0.7, 2: 0.8}, 'rssi_dbm_to': {1: -70, 2: -60}, 'latency_ms_to': {1: 25, 2: 15}},
                ]
            }
        })
        assert result['success']
        print(f"Mesh QoS: {result['result']['total_links']} links, avg quality: {result['result']['link_quality']['mean']:.2f}")
        
        # Test battery prediction
        result = backend.handle_request({
            'id': 'test4',
            'method': 'predict_battery_failure',
            'args': {
                'vehicles': [
                    {'id': 1, 'callsign': 'DRONE1', 'battery_remaining_pct': 45, 'battery_current_a': 15, 'battery_capacity_mah': 5000},
                    {'id': 2, 'callsign': 'DRONE2', 'battery_remaining_pct': 15, 'battery_current_a': 20, 'battery_capacity_mah': 5000},
                ]
            }
        })
        assert result['success']
        print(f"Battery predictions: {len(result['result']['critical_vehicles'])} critical")
        
        print("All self-tests passed!")
        return
    
    if args.server:
        asyncio.run(run_server())
    elif args.method:
        backend = SwarmMonitorBackend()
        request_args = json.loads(args.args) if args.args else {}
        request = {'id': 'cli', 'method': args.method, 'args': request_args}
        response = backend.handle_request(request)
        print(json.dumps(response, indent=2))
    else:
        parser.print_help()


if __name__ == '__main__':
    main()