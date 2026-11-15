#!/usr/bin/env python3
"""
Standalone Offline Test Runner for Sorting Arm Package.
Executes test functions without requiring pytest or ROS 2 middleware.
"""

import os
import sys
import traceback

script_dir = os.path.dirname(os.path.abspath(__file__))
pkg_dir = os.path.abspath(os.path.join(script_dir, ".."))
project_root = os.path.abspath(os.path.join(pkg_dir, "../../.."))

if pkg_dir not in sys.path:
    sys.path.insert(0, pkg_dir)
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Import test modules
import test_calibration_and_features
import test_camera_alignment
import test_kinematics
import test_vision_pipeline

modules = [
    test_calibration_and_features,
    test_camera_alignment,
    test_kinematics,
    test_vision_pipeline,
]

total_passed = 0
total_failed = 0

print("=" * 70)
print("  RUNNING OFFLINE SIMULATION UNIT TESTS (NO HARDWARE REQUIRED)")
print("=" * 70)

for mod in modules:
    mod_name = mod.__name__
    print(f"\n--- Running tests in {mod_name} ---")
    for attr in dir(mod):
        if attr.startswith("test_") and callable(getattr(mod, attr)):
            test_fn = getattr(mod, attr)
            try:
                test_fn()
                print(f"  [PASS] {attr}")
                total_passed += 1
            except Exception as e:
                print(f"  [FAIL] {attr}: {e}")
                traceback.print_exc()
                total_failed += 1

print("\n" + "=" * 70)
print(f"  TEST RESULTS: {total_passed} PASSED, {total_failed} FAILED")
print("=" * 70)

sys.exit(0 if total_failed == 0 else 1)
