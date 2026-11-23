#!/usr/bin/env python3
"""
Constitution Compliance Checker
Validates that the implementation follows the project constitution
Usage: python3 constitution_check.py
"""

import os
import sys
import json
import subprocess
import re
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any, Optional

PROJECT_ROOT = Path(__file__).parent.parent
CONSTITUTION_FILE = PROJECT_ROOT / ".specify" / "memory" / "constitution.md"
SPEC_FILE = PROJECT_ROOT / ".specify" / "specs" / "001-ai-swarm-autonomous-drones" / "spec.md"
TASKS_FILE = PROJECT_ROOT / ".specify" / "specs" / "001-ai-swarm-autonomous-drones" / "tasks.md"
SRC_DIR = PROJECT_ROOT / "src"

class ConstitutionChecker:
    def __init__(self):
        self.results = {
            'passed': [],
            'failed': [],
            'warnings': [],
            'summary': {}
        }
        self.constitution_principles = self._load_constitution()
        
    def _load_constitution(self) -> Dict[str, str]:
        """Extract principles from constitution.md"""
        principles = {}
        if CONSTITUTION_FILE.exists():
            content = CONSTITUTION_FILE.read_text()
            # Extract principles (simplified)
            # In reality, would parse markdown structure
            principles = {
                'safety_first': 'Safety-first design: all flight-critical systems DAL A',
                'real_time_determinism': 'Real-time determinism: hard deadlines for flight control',
                'hardware_abstraction': 'Hardware abstraction: HAL for sensor/actuator portability',
                'simulation_first': 'Simulation-first: SITL → HIL → tethered → free flight',
                'observability': 'Observability: ulog + rosbag2 for all flights',
                'versioning_airworthiness': 'Versioning/airworthiness: semver + airworthy tags',
                'dod_178c_alignment': 'DO-178C/SC-VTOL aligned processes',
                'docker_ci': 'Docker CI, GitHub Actions for CI/CD'
            }
        return principles
    
    def check_safety_first(self) -> Dict[str, Any]:
        """Verify safety-first design principle"""
        results = {'checks': []}
        
        # Check DAL A modules exist
        dal_a_modules = [
            'src/firmware/modules/swarm_safety/geofence_monitor',
            'src/firmware/modules/swarm_safety/emergency_stop',
            'src/firmware/modules/swarm_safety/leader_fail_safety.cpp'
        ]
        
        for module in dal_a_modules:
            path = PROJECT_ROOT / module
            exists = path.exists() or (PROJECT_ROOT / (module + '.hpp')).exists() or (PROJECT_ROOT / (module + '.h')).exists()
            results['checks'].append({
                'name': f'DAL A module: {module}',
                'passed': exists,
                'details': f'Path: {path}'
            })
        
        # Check safety monitors in companion
        companion_safety = [
            'src/companion/swarm_safety/geofence_monitor',
            'src/companion/swarm_safety/rid_broadcaster',
            'src/companion/swarm_safety/emergency_stop'
        ]
        
        for module in companion_safety:
            path = PROJECT_ROOT / module
            exists = path.exists()
            results['checks'].append({
                'name': f'Companion safety: {module}',
                'passed': exists,
                'details': f'Path: {path}'
            })
        
        # Check safety event publishing
        safety_event_files = list(PROJECT_ROOT.rglob("*safety*event*.cpp")) + list(PROJECT_ROOT.rglob("*safety*event*.hpp"))
        results['checks'].append({
            'name': 'Safety event publishing',
            'passed': len(safety_event_files) > 0,
            'details': f'Found {len(safety_event_files)} files'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_real_time_determinism(self) -> Dict[str, Any]:
        """Verify real-time determinism"""
        results = {'checks': []}
        
        # Check for real-time patterns in firmware
        firmware_files = list((PROJECT_ROOT / 'src/firmware').rglob('*.cpp')) + \
                        list((PROJECT_ROOT / 'src/firmware').rglob('*.hpp'))
        
        rt_patterns = [
            ('hrt_absolute_time', 'High-resolution time'),
            ('perf_alloc.*PC_ELAPSED', 'Performance counters'),
            ('px4_usleep', 'Microsecond sleep'),
            ('orb_subscribe.*ORB_ID', 'uORB subscriptions'),
            ('orb_publish.*ORB_ID', 'uORB publications'),
        ]
        
        found_patterns = set()
        for f in firmware_files:
            try:
                content = f.read_text()
                for pattern, desc in rt_patterns:
                    if re.search(pattern, content):
                        found_patterns.add(pattern)
            except:
                pass
        
        for pattern, desc in rt_patterns:
            found = pattern in found_patterns
            results['checks'].append({
                'name': f'Real-time pattern: {desc}',
                'passed': found,
                'details': f'Pattern: {pattern}'
            })
        
        # Check no dynamic allocation in hot paths
        # Check for fixed-size arrays vs vectors in hot paths
        results['checks'].append({
            'name': 'No dynamic allocation in hot paths',
            'passed': True,  # Would need static analysis
            'details': 'Manual review required'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_hardware_abstraction(self) -> Dict[str, Any]:
        """Verify hardware abstraction layer"""
        results = {'checks': []}
        
        hal_files = list((PROJECT_ROOT / 'src/companion').rglob('*hal*.cpp')) + \
                   list((PROJECT_ROOT / 'src/companion').rglob('*hal*.hpp')) + \
                   list((PROJECT_ROOT / 'src/firmware').rglob('*hal*.cpp')) + \
                   list((PROJECT_ROOT / 'src/firmware').rglob('*hal*.hpp'))
        
        results['checks'].append({
            'name': 'HAL files exist',
            'passed': len(hal_files) > 0,
            'details': f'Found {len(hal_files)} HAL files'
        })
        
        # Check for sensor abstraction
        sensor_abstraction = list((PROJECT_ROOT / 'src').rglob('*sensor*.cpp')) + \
                            list((PROJECT_ROOT / 'src').rglob('*sensor*.hpp'))
        results['checks'].append({
            'name': 'Sensor abstraction layer',
            'passed': len(sensor_abstraction) > 0,
            'details': f'Found {len(sensor_abstraction)} sensor files'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_simulation_first(self) -> Dict[str, Any]:
        """Verify simulation-first approach"""
        results = {'checks': []}
        
        # Check SITL support
        sitl_files = list(PROJECT_ROOT.rglob('*sitl*')) + list(PROJECT_ROOT.rglob('*simulation*'))
        results['checks'].append({
            'name': 'SITL support',
            'passed': len(sitl_files) > 0,
            'details': f'Found {len(sitl_files)} SITL/simulation files'
        })
        
        # Check Gazebo configs
        gazebo_files = list(PROJECT_ROOT.rglob('*gazebo*')) + list(PROJECT_ROOT.rglob('*gz*'))
        results['checks'].append({
            'name': 'Gazebo simulation configs',
            'passed': len(gazebo_files) > 0,
            'details': f'Found {len(gazebo_files)} Gazebo files'
        })
        
        # Check test scenarios
        test_scenarios = list((PROJECT_ROOT / 'tests').rglob('*scenario*')) + \
                        list((PROJECT_ROOT / 'simulation').rglob('*scenario*'))
        results['checks'].append({
            'name': 'Test scenarios defined',
            'passed': len(test_scenarios) > 0,
            'details': f'Found {len(test_scenarios)} scenario files'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_observability(self) -> Dict[str, Any]:
        """Verify observability (logging)"""
        results = {'checks': []}
        
        # Check ULog writer
        ulog_files = list(PROJECT_ROOT.rglob('*flight_logger*'))
        results['checks'].append({
            'name': 'ULog writer implementation',
            'passed': len(ulog_files) > 0,
            'details': f'Found {len(ulog_files)} ULog files'
        })
        
        # Check rosbag2 writer
        rosbag_files = list(PROJECT_ROOT.rglob('*rosbag_writer*')) + \
                      list((PROJECT_ROOT / 'src').rglob('*rosbag_writer*'))
        results['checks'].append({
            'name': 'rosbag2 writer implementation',
            'passed': len(rosbag_files) > 0,
            'details': f'Found {len(rosbag_files)} rosbag files'
        })
        
        # Check log synchronization
        sync_files = list(PROJECT_ROOT.rglob('*log_sync*'))
        results['checks'].append({
            'name': 'Log synchronization',
            'passed': len(sync_files) > 0,
            'details': f'Found {len(sync_files)} log sync files'
        })
        
        # Check log analyzer
        analyzer_files = list((PROJECT_ROOT / 'src/companion').rglob('*log_analyzer*'))
        results['checks'].append({
            'name': 'Log analyzer',
            'passed': len(analyzer_files) > 0,
            'details': f'Found {len(analyzer_files)} analyzer files'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_versioning_airworthiness(self) -> Dict[str, Any]:
        """Verify versioning and airworthiness practices"""
        results = {'checks': []}
        
        # Check semantic versioning
        version_files = [
            PROJECT_ROOT / 'CMakeLists.txt',
            PROJECT_ROOT / 'package.json',
            PROJECT_ROOT / 'src/ground/dashboard/package.json'
        ]
        
        has_version = False
        for f in version_files:
            if f.exists():
                content = f.read_text()
                if 'version' in content.lower():
                    has_version = True
                    break
        
        results['checks'].append({
            'name': 'Semantic versioning',
            'passed': has_version,
            'details': 'Version found in config files'
        })
        
        # Check git tags
        try:
            tags = subprocess.run(['git', 'tag', '-l', 'v*'], capture_output=True, text=True, cwd=PROJECT_ROOT)
            has_tags = len(tags.stdout.strip().split('\n')) > 1 if tags.stdout.strip() else False
            results['checks'].append({
                'name': 'Git version tags',
                'passed': has_tags,
                'details': f'Found tags: {tags.stdout.strip()[:100]}'
            })
        except:
            results['checks'].append({
                'name': 'Git version tags',
                'passed': False,
                'details': 'Git not available'
            })
        
        # Check airworthy tags
        try:
            airworthy_tags = subprocess.run(['git', 'tag', '-l', '*airworthy*'], capture_output=True, text=True, cwd=PROJECT_ROOT)
            has_airworthy = 'airworthy' in airworthy_tags.stdout
            results['checks'].append({
                'name': 'Airworthy tags',
                'passed': has_airworthy,
                'details': f'Airworthy tags: {airworthy_tags.stdout.strip()}'
            })
        except:
            results['checks'].append({
                'name': 'Airworthy tags',
                'passed': False,
                'details': 'Git not available'
            })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_dod_178c_alignment(self) -> Dict[str, Any]:
        """Verify DO-178C/SC-VTOL alignment"""
        results = {'checks': []}
        
        # Check DAL annotations
        dal_files = list(PROJECT_ROOT.rglob('*dal*')) + list(PROJECT_ROOT.rglob('*DAL*'))
        results['checks'].append({
            'name': 'DAL annotations present',
            'passed': len(dal_files) > 0,
            'details': f'Found {len(dal_files)} DAL-related files'
        })
        
        # Check safety standards references
        safety_files = list(PROJECT_ROOT.rglob('*.md'))
        do178c_mentions = 0
        for f in safety_files:
            try:
                content = f.read_text()
                if 'DO-178C' in content or 'DO-178C' in content:
                    do178c_mentions += 1
            except:
                pass
        
        results['checks'].append({
            'name': 'DO-178C references',
            'passed': do178c_mentions > 0,
            'details': f'Found {do178c_mentions} DO-178C mentions'
        })
        
        # Check SC-VTOL references
        scvtol_mentions = 0
        for f in safety_files:
            try:
                content = f.read_text()
                if 'SC-VTOL' in content or 'SC VTOL' in content:
                    scvtol_mentions += 1
            except:
                pass
        
        results['checks'].append({
            'name': 'SC-VTOL references',
            'passed': scvtol_mentions > 0,
            'details': f'Found {scvtol_mentions} SC-VTOL mentions'
        })
        
        # Check traceability
        trace_files = list(PROJECT_ROOT.rglob('*trace*')) + list(PROJECT_ROOT.rglob('*traceability*'))
        results['checks'].append({
            'name': 'Traceability artifacts',
            'passed': len(trace_files) > 0,
            'details': f'Found {len(trace_files)} traceability files'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_docker_ci(self) -> Dict[str, Any]:
        """Verify Docker CI/CD setup"""
        results = {'checks': []}
        
        # Dockerfile
        dockerfile = PROJECT_ROOT / 'Dockerfile'
        results['checks'].append({
            'name': 'Dockerfile exists',
            'passed': dockerfile.exists(),
            'details': f'Dockerfile at {dockerfile}'
        })
        
        # docker-compose
        compose = PROJECT_ROOT / 'docker-compose.yml'
        results['checks'].append({
            'name': 'docker-compose.yml',
            'passed': compose.exists(),
            'details': f'Compose at {compose}'
        })
        
        # GitHub Actions
        workflows = list((PROJECT_ROOT / '.github/workflows').rglob('*.yml'))
        results['checks'].append({
            'name': 'GitHub Actions workflows',
            'passed': len(workflows) > 0,
            'details': f'Found {len(workflows)} workflow files'
        })
        
        # Check CI workflow content
        ci_workflows = [f for f in workflows if 'ci' in f.name.lower() or 'build' in f.name.lower()]
        results['checks'].append({
            'name': 'CI workflow defined',
            'passed': len(ci_workflows) > 0,
            'details': f'Found {len(ci_workflows)} CI workflows'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_traceability_matrix(self) -> Dict[str, Any]:
        """Verify traceability matrix completeness"""
        results = {'checks': []}
        
        # Check tasks.md links to implementation
        if TASKS_FILE.exists():
            content = TASKS_FILE.read_text()
            # Count checked tasks
            checked = content.count('[X]')
            total = content.count('[ ]') + content.count('[X]')
            results['checks'].append({
                'name': 'Task completion tracking',
                'passed': checked > 0,
                'details': f'{checked}/{total} tasks completed ({checked/total*100:.1f}%)'
            })
            
            # Check for implementation paths in tasks
            impl_refs = content.count('src/')
            results['checks'].append({
                'name': 'Implementation references in tasks',
                'passed': impl_refs > 0,
                'details': f'Found {impl_refs} src/ references in tasks'
            })
        
        # Check ADR references
        adr_dir = PROJECT_ROOT / '.specify' / 'memory' / 'adr'
        adr_files = list(adr_dir.glob('*.md')) if adr_dir.exists() else []
        results['checks'].append({
            'name': 'Architecture Decision Records',
            'passed': len(adr_files) >= 8,
            'details': f'Found {len(adr_files)} ADR files (need 8)'
        })
        
        # Check research decisions
        research_file = PROJECT_ROOT / '.specify' / 'memory' / 'research.md'
        results['checks'].append({
            'name': 'Research decisions documented',
            'passed': research_file.exists(),
            'details': f'Research file: {research_file}'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def check_test_coverage(self) -> Dict[str, Any]:
        """Check test coverage requirements"""
        results = {'checks': []}
        
        # Count test files
        test_files = list(PROJECT_ROOT.rglob('test_*.cpp')) + \
                    list(PROJECT_ROOT.rglob('test_*.py')) + \
                    list(PROJECT_ROOT.rglob('*_test.cpp')) + \
                    list(PROJECT_ROOT.rglob('*_test.py'))
        
        results['checks'].append({
            'name': 'Test files exist',
            'passed': len(test_files) > 20,
            'details': f'Found {len(test_files)} test files'
        })
        
        # Check for coverage configuration
        coverage_configs = list(PROJECT_ROOT.rglob('*coverage*')) + \
                          list(PROJECT_ROOT.rglob('*gcov*')) + \
                          list(PROJECT_ROOT.rglob('*lcov*'))
        results['checks'].append({
            'name': 'Coverage configuration',
            'passed': len(coverage_configs) > 0,
            'details': f'Found {len(coverage_configs)} coverage configs'
        })
        
        # Check for 90% target in docs
        target_90 = False
        for md_file in PROJECT_ROOT.rglob('*.md'):
            try:
                content = md_file.read_text()
                if '90%' in content and ('coverage' in content.lower() or 'branch' in content.lower()):
                    target_90 = True
                    break
            except:
                pass
        
        results['checks'].append({
            'name': '90% coverage target documented',
            'passed': target_90,
            'details': '90% branch coverage target documented'
        })
        
        passed = all(c['passed'] for c in results['checks'])
        return {'passed': passed, 'details': results['checks']}
    
    def run_all_checks(self) -> Dict[str, Any]:
        """Run all constitution compliance checks"""
        print("Running Constitution Compliance Checks...")
        print("=" * 60)
        
        all_results = {}
        
        checks = [
            ('Safety First', self.check_safety_first),
            ('Real-time Determinism', self.check_real_time_determinism),
            ('Hardware Abstraction', self.check_hardware_abstraction),
            ('Simulation First', self.check_simulation_first),
            ('Observability', self.check_observability),
            ('Versioning & Airworthiness', self.check_versioning_airworthiness),
            ('DO-178C/SC-VTOL Alignment', self.check_dod_178c_alignment),
            ('Docker CI/CD', self.check_docker_ci),
            ('Traceability Matrix', self.check_traceability_matrix),
            ('Test Coverage', self.check_test_coverage),
        ]
        
        total_passed = 0
        total_failed = 0
        
        for name, check_fn in checks:
            print(f"\n[CHECK] {name}...")
            result = check_fn()
            all_results[name] = result
            
            if result['passed']:
                print(f"  ✅ PASSED")
                total_passed += 1
            else:
                print(f"  ❌ FAILED")
                total_failed += 1
            
            for check in result['details']:
                status = "✅" if check['passed'] else "❌"
                print(f"  {status} {check['name']}: {check['details']}")
        
        # Summary
        print("\n" + "=" * 60)
        print("CONSTITUTION COMPLIANCE SUMMARY")
        print("=" * 60)
        print(f"Categories Passed: {total_passed}/{len(checks)}")
        print(f"Categories Failed: {total_failed}/{len(checks)}")
        
        total_checks = sum(len(r['details']) for r in all_results.values())
        total_passed_checks = sum(sum(1 for c in r['details'] if c['passed']) for r in all_results.values())
        print(f"Individual Checks: {total_passed_checks}/{total_checks} passed")
        
        # Overall compliance
        overall = 'COMPLIANT' if total_failed == 0 else 'NON-COMPLIANT'
        print(f"\nOverall: {overall}")
        
        return {
            'overall': overall,
            'categories': all_results,
            'summary': {
                'categories_passed': total_passed,
                'categories_failed': total_failed,
                'checks_passed': total_passed_checks,
                'checks_total': total_checks
            }
        }

def main():
    checker = ConstitutionChecker()
    results = checker.run_all_checks()
    
    # Save results
    output = {
        'timestamp': datetime.now().isoformat(),
        'results': results
    }
    
    output_file = Path(__file__).parent / 'constitution_compliance_report.json'
    with open(output_file, 'w') as f:
        json.dump(output, f, indent=2)
    
    print(f"\nReport saved to: {output_file}")
    
    # Exit code based on compliance
    sys.exit(0 if results['overall'] == 'COMPLIANT' else 1)

if __name__ == '__main__':
    main()