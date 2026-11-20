#!/usr/bin/env python3
# Log Analyzer for Drone Swarm Validation

import argparse
import sys
import os
import numpy as np
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import json

try:
    import ulog
    ULOG_AVAILABLE = True
except ImportError:
    ULOG_AVAILABLE = False
    print("Warning: pyulog not available, ulog parsing disabled")

try:
    import rosbag2_py
    ROSBAG2_AVAILABLE = True
except ImportError:
    ROSBAG2_AVAILABLE = False
    print("Warning: rosbag2_py not available, rosbag2 parsing disabled")


class LogAnalyzer:
    def __init__(self, log_dir: str):
        self.log_dir = Path(log_dir)
        self.ulog_files = list(self.log_dir.glob("*.ulg"))
        self.rosbag_dirs = [d for d in self.log_dir.iterdir() if d.is_dir() and (d / "metadata.yaml").exists()]
        
        print(f"Found {len(self.ulog_files)} ulog files")
        print(f"Found {len(self.rosbag_dirs)} rosbag2 directories")
    
    def check_formation_hold(self, max_lateral_error: float = 0.5, 
                            max_vertical_error: float = 0.3) -> Dict:
        """Check formation hold performance from logs."""
        results = {
            "check": "formation_hold",
            "passed": False,
            "mean_lateral_error": 0.0,
            "max_lateral_error": 0.0,
            "mean_vertical_error": 0.0,
            "max_vertical_error": 0.0,
            "details": []
        }
        
        if not ULOG_AVAILABLE:
            results["details"].append("pyulog not available")
            return results
        
        all_lateral = []
        all_vertical = []
        
        for ulog_file in self.ulog_files:
            try:
                ulog_data = ulog.ULog(str(ulog_file))
                
                # Look for formation state messages
                for msg in ulog_data.data_list:
                    if msg.name == 'swarm_formation_state':
                        data = msg.data
                        if 'position_error_flu' in data.columns:
                            pos_err = data.data['position_error_flu']
                            for err in pos_err:
                                lateral = np.sqrt(err[0]**2 + err[1]**2)
                                vertical = abs(err[2])
                                all_lateral.append(lateral)
                                all_vertical.append(vertical)
            except Exception as e:
                results["details"].append(f"Error reading {ulog_file}: {e}")
        
        if all_lateral:
            results["mean_lateral_error"] = float(np.mean(all_lateral))
            results["max_lateral_error"] = float(np.max(all_lateral))
            results["mean_vertical_error"] = float(np.mean(all_vertical))
            results["max_vertical_error"] = float(np.max(all_vertical))
            
            results["passed"] = (results["mean_lateral_error"] < max_lateral_error and
                                results["mean_vertical_error"] < max_vertical_error)
            results["details"].append(f"Analyzed {len(all_lateral)} formation samples from {len(self.ulog_files)} flights")
        else:
            results["details"].append("No formation state data found in logs")
        
        return results
    
    def check_replan_latency(self, max_ms: float = 200.0) -> Dict:
        """Check replan propagation latency."""
        results = {
            "check": "replan_latency",
            "passed": False,
            "mean_latency_ms": 0.0,
            "max_latency_ms": 0.0,
            "details": []
        }
        
        if not ROSBAG2_AVAILABLE:
            results["details"].append("rosbag2_py not available")
            return results
        
        # Would parse rosbag2 for replan messages
        results["details"].append("Implementation pending - requires rosbag2 parsing")
        return results
    
    def check_task_reallocation(self, max_realloc_time: float = 5.0,
                               max_coverage_gap: float = 0.02) -> Dict:
        """Check task reallocation performance."""
        results = {
            "check": "task_reallocation",
            "passed": False,
            "realloc_time_s": 0.0,
            "coverage_gap_pct": 0.0,
            "details": []
        }
        
        # Would parse logs for task allocation messages
        results["details"].append("Implementation pending")
        return results
    
    def check_obstacle_avoidance(self, min_f1: float = 0.92,
                                max_avoidance_latency_ms: float = 200.0) -> Dict:
        """Check obstacle avoidance performance."""
        results = {
            "check": "obstacle_avoidance",
            "passed": False,
            "f1_score": 0.0,
            "avoidance_latency_ms": 0.0,
            "details": []
        }
        
        # Would parse AI inference logs
        results["details"].append("Implementation pending")
        return results
    
    def check_gps_denied_drift(self, max_drift_pct: float = 1.0) -> Dict:
        """Check GPS-denied position drift."""
        results = {
            "check": "gps_denied_drift",
            "passed": False,
            "drift_pct": 0.0,
            "details": []
        }
        
        if not ULOG_AVAILABLE:
            results["details"].append("pyulog not available")
            return results
        
        # Compare VIO+UWB position with ground truth
        results["details"].append("Implementation pending")
        return results
    
    def check_emergency_stop(self, max_kill_time_s: float = 1.0, drone_count: int = 10) -> Dict:
        """Check emergency stop propagation."""
        results = {
            "check": "emergency_stop",
            "passed": False,
            "kill_time_s": 0.0,
            "drones_stopped": 0,
            "details": []
        }
        
        # Would parse safety event logs
        results["details"].append("Implementation pending")
        return results
    
    def run_all_checks(self, **kwargs) -> List[Dict]:
        """Run all validation checks."""
        checks = [
            self.check_formation_hold(
                max_lateral_error=kwargs.get('max_lateral_error', 0.5),
                max_vertical_error=kwargs.get('max_vertical_error', 0.3)
            ),
            self.check_replan_latency(max_ms=kwargs.get('max_replan_latency_ms', 200.0)),
            self.check_task_reallocation(
                max_realloc_time=kwargs.get('max_realloc_time', 5.0),
                max_coverage_gap=kwargs.get('max_coverage_gap', 0.02)
            ),
            self.check_obstacle_avoidance(
                min_f1=kwargs.get('min_f1', 0.92),
                max_avoidance_latency_ms=kwargs.get('max_avoidance_latency_ms', 200.0)
            ),
            self.check_gps_denied_drift(max_drift_pct=kwargs.get('max_drift_pct', 1.0)),
            self.check_emergency_stop(
                max_kill_time_s=kwargs.get('max_kill_time_s', 1.0),
                drone_count=kwargs.get('drone_count', 10)
            ),
        ]
        return checks


def main():
    parser = argparse.ArgumentParser(description="Drone Swarm Log Analyzer")
    parser.add_argument('--logs', required=True, help='Log directory path')
    parser.add_argument('--check', choices=['formation_hold', 'replan_latency', 'task_reallocation',
                                           'obstacle_avoidance', 'gps_denied_drift', 'emergency_stop', 'all'],
                       default='all', help='Check to run')
    parser.add_argument('--max-lateral-error', type=float, default=0.5)
    parser.add_argument('--max-vertical-error', type=float, default=0.3)
    parser.add_argument('--max-replan-latency-ms', type=float, default=200.0)
    parser.add_argument('--max-realloc-time', type=float, default=5.0)
    parser.add_argument('--max-coverage-gap', type=float, default=0.02)
    parser.add_argument('--min-f1', type=float, default=0.92)
    parser.add_argument('--max-avoidance-latency-ms', type=float, default=200.0)
    parser.add_argument('--max-drift-pct', type=float, default=1.0)
    parser.add_argument('--max-kill-time-s', type=float, default=1.0)
    parser.add_argument('--drone-count', type=int, default=10)
    parser.add_argument('--output', help='Output JSON file')
    
    args = parser.parse_args()
    
    analyzer = LogAnalyzer(args.logs)
    
    if args.check == 'all':
        results = analyzer.run_all_checks(
            max_lateral_error=args.max_lateral_error,
            max_vertical_error=args.max_vertical_error,
            max_replan_latency_ms=args.max_replan_latency_ms,
            max_realloc_time=args.max_realloc_time,
            max_coverage_gap=args.max_coverage_gap,
            min_f1=args.min_f1,
            max_avoidance_latency_ms=args.max_avoidance_latency_ms,
            max_drift_pct=args.max_drift_pct,
            max_kill_time_s=args.max_kill_time_s,
            drone_count=args.drone_count
        )
    else:
        # Run specific check
        check_method = getattr(analyzer, f'check_{args.check}')
        results = [check_method()]
    
    # Print summary
    print("\n=== Validation Results ===")
    all_passed = True
    for r in results:
        status = "PASS" if r.get("passed", False) else "FAIL"
        print(f"  {r['check']}: {status}")
        for detail in r.get("details", []):
            print(f"    - {detail}")
        if not r.get("passed", False):
            all_passed = False
    
    print(f"\nOverall: {'PASS' if all_passed else 'FAIL'}")
    
    if args.output:
        with open(args.output, 'w') as f:
            json.dump({"results": results, "overall_pass": all_passed}, f, indent=2)
        print(f"Results saved to {args.output}")
    
    sys.exit(0 if all_passed else 1)


if __name__ == '__main__':
    main()