#!/usr/bin/env python3
# Test runner for SITL simulation scenarios

import argparse
import subprocess
import sys
import time
import yaml
import json
import os
from pathlib import Path
from typing import Dict, List, Any
from dataclasses import dataclass
from enum import Enum

class TestResult(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    ERROR = "ERROR"

@dataclass
class TestCase:
    name: str
    suite: str
    command: List[str]
    timeout: int
    expected_outcome: Dict[str, Any]
    description: str

@dataclass
class TestRunResult:
    test_case: TestCase
    result: TestResult
    duration: float
    output: str
    error: str = ""

class ScenarioRunner:
    def __init__(self, project_root: str):
        self.project_root = Path(project_root)
        self.results: List[TestRunResult] = []
        self.test_cases = self._load_test_cases()
    
    def _load_test_cases(self) -> List[TestCase]:
        """Load test cases from scenario YAML files."""
        cases = []
        scenario_dir = self.project_root / "src" / "simulation" / "scenarios"
        
        if not scenario_dir.exists():
            return cases
        
        for scenario_file in scenario_dir.glob("*.yaml"):
            with open(scenario_file) as f:
                scenario = yaml.safe_load(f)
            
            for test in scenario.get("tests", []):
                cases.append(TestCase(
                    name=test["name"],
                    suite=scenario.get("suite", "unknown"),
                    command=test["command"],
                    timeout=test.get("timeout", 300),
                    expected_outcome=test.get("expected", {}),
                    description=test.get("description", "")
                ))
        
        return cases
    
    def run_suite(self, suite_name: str, **kwargs) -> Dict[str, Any]:
        """Run all tests in a suite."""
        suite_tests = [tc for tc in self.test_cases if tc.suite == suite_name]
        
        if not suite_tests:
            return {"error": f"No tests found for suite: {suite_name}"}
        
        print(f"Running suite: {suite_name} ({len(suite_tests)} tests)")
        
        passed = 0
        failed = 0
        errors = 0
        
        for test_case in suite_tests:
            result = self._run_test(test_case, **kwargs)
            self.results.append(result)
            
            if result.result == TestResult.PASS:
                passed += 1
                print(f"  ✓ {test_case.name} ({result.duration:.1f}s)")
            elif result.result == TestResult.FAIL:
                failed += 1
                print(f"  ✗ {test_case.name} ({result.duration:.1f}s)")
            elif result.result == TestResult.ERROR:
                errors += 1
                print(f"  ⚠ {test_case.name} ({result.duration:.1f}s) - ERROR")
        
        summary = {
            "suite": suite_name,
            "total": len(suite_tests),
            "passed": passed,
            "failed": failed,
            "errors": errors,
            "pass_rate": passed / len(suite_tests) if suite_tests else 0
        }
        
        return summary
    
    def _run_test(self, test_case: TestCase, **kwargs) -> TestRunResult:
        """Run a single test case."""
        start_time = time.time()
        
        # Build command with kwargs
        cmd = []
        for part in test_case.command:
            if isinstance(part, str) and "{" in part:
                cmd.append(part.format(**kwargs))
            else:
                cmd.append(str(part))
        
        try:
            proc = subprocess.run(
                cmd,
                cwd=self.project_root,
                capture_output=True,
                text=True,
                timeout=test_case.timeout
            )
            
            duration = time.time() - start_time
            
            # Check expected outcome
            success = self._check_outcome(proc, test_case.expected_outcome)
            
            return TestRunResult(
                test_case=test_case,
                result=TestResult.PASS if success else TestResult.FAIL,
                duration=duration,
                output=proc.stdout,
                error=proc.stderr if not success else ""
            )
            
        except subprocess.TimeoutExpired:
            duration = time.time() - start_time
            return TestRunResult(
                test_case=test_case,
                result=TestResult.FAIL,
                duration=duration,
                output="",
                error=f"Timeout after {test_case.timeout}s"
            )
        except Exception as e:
            duration = time.time() - start_time
            return TestRunResult(
                test_case=test_case,
                result=TestResult.ERROR,
                duration=duration,
                output="",
                error=str(e)
            )
    
    def _check_outcome(self, proc: subprocess.CompletedProcess, expected: Dict[str, Any]) -> bool:
        """Check if process outcome matches expected."""
        if "returncode" in expected and proc.returncode != expected["returncode"]:
            return False
        
        if "stdout_contains" in expected:
            for pattern in expected["stdout_contains"]:
                if pattern not in proc.stdout:
                    return False
        
        if "stderr_not_contains" in expected:
            for pattern in expected["stderr_not_contains"]:
                if pattern in proc.stderr:
                    return False
        
        return True
    
    def run_all(self, **kwargs) -> Dict[str, Any]:
        """Run all test suites."""
        suites = set(tc.suite for tc in self.test_cases)
        all_summaries = {}
        
        for suite in sorted(suites):
            all_summaries[suite] = self.run_suite(suite, **kwargs)
        
        total_tests = sum(s.get("total", 0) for s in all_summaries.values())
        total_passed = sum(s.get("passed", 0) for s in all_summaries.values())
        total_failed = sum(s.get("failed", 0) for s in all_summaries.values())
        total_errors = sum(s.get("errors", 0) for s in all_summaries.values())
        
        return {
            "suites": all_summaries,
            "total": total_tests,
            "passed": total_passed,
            "failed": total_failed,
            "errors": total_errors,
            "pass_rate": total_passed / total_tests if total_tests else 0
        }
    
    def generate_report(self, output_file: str = None) -> str:
        """Generate test report."""
        report = {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "summary": {
                "total": len(self.results),
                "passed": sum(1 for r in self.results if r.result == TestResult.PASS),
                "failed": sum(1 for r in self.results if r.result == TestResult.FAIL),
                "errors": sum(1 for r in self.results if r.result == TestResult.ERROR)
            },
            "tests": [
                {
                    "name": r.test_case.name,
                    "suite": r.test_case.suite,
                    "result": r.result.value,
                    "duration": r.duration,
                    "error": r.error
                }
                for r in self.results
            ]
        }
        
        report_json = json.dumps(report, indent=2)
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report_json)
        
        return report_json

def main():
    parser = argparse.ArgumentParser(description="SITL Test Runner")
    parser.add_argument("--suite", help="Test suite to run (formation, search, ai_perception, safety)")
    parser.add_argument("--all", action="store_true", help="Run all suites")
    parser.add_argument("--drones", type=int, default=5, help="Number of drones")
    parser.add_argument("--area", default="1km2", help="Search area")
    parser.add_argument("--min-coverage", type=float, default=0.95, help="Minimum coverage")
    parser.add_argument("--max-time", type=int, default=1800, help="Max mission time (seconds)")
    parser.add_argument("--max-lateral-error", type=float, default=0.4, help="Max lateral error (m)")
    parser.add_argument("--max-vertical-error", type=float, default=0.2, help="Max vertical error (m)")
    parser.add_argument("--models", default="yolov8n_obstacle,mobilenetv3_landing,lstm_fault", help="AI models to test")
    parser.add_argument("--min-f1", type=float, default=0.92, help="Minimum F1 score")
    parser.add_argument("--max-latency", type=int, default=50, help="Max inference latency (ms)")
    parser.add_argument("--output", help="Output report file")
    
    args = parser.parse_args()
    
    project_root = os.environ.get("PROJECT_ROOT", "/my-drone-project")
    runner = ScenarioRunner(project_root)
    
    kwargs = vars(args)
    
    if args.all:
        summary = runner.run_all(**kwargs)
    elif args.suite:
        summary = runner.run_suite(args.suite, **kwargs)
    else:
        parser.print_help()
        return 1
    
    print("\n=== Test Summary ===")
    print(f"Total: {summary['total']}")
    print(f"Passed: {summary['passed']}")
    print(f"Failed: {summary['failed']}")
    print(f"Errors: {summary['errors']}")
    print(f"Pass Rate: {summary['pass_rate']:.1%}")
    
    if args.output:
        runner.generate_report(args.output)
        print(f"Report saved to: {args.output}")
    
    return 0 if summary['failed'] == 0 and summary['errors'] == 0 else 1

if __name__ == "__main__":
    sys.exit(main())