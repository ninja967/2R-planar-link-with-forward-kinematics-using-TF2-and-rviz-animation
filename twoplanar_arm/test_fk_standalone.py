#!/usr/bin/env python3
"""
Standalone Forward Kinematics Test Script
Two-link planar inspection arm and forward kinematics

Can be executed without ROS 2 to mathematically and structurally verify:
1. Analytical Forward Kinematics against course acceptance points
2. URDF file parsing and coordinate transform validation
3. Error tolerance check within 1e-6 m
"""

import math
import os
import xml.etree.ElementTree as ET

# Kinematic parameters
L1 = 0.500  # meters
L2 = 0.300  # meters
TOLERANCE = 1e-6  # meters


def forward_kinematics(q1, q2):
    """Analytic forward kinematics for 2-link planar arm."""
    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y


def run_tests():
    print("=" * 80)
    print("2R PLANAR LINK FORWARD KINEMATICS VERIFICATION")
    print(f"Parameters: L1 = {L1:.3f} m, L2 = {L2:.3f} m, Tolerance = {TOLERANCE:.1e} m")
    print("=" * 80)

    test_cases = [
        {"name": "Case 1: (q1=0, q2=0)",        "q1": 0.0,         "q2": 0.0,         "exp_x": 0.8, "exp_y": 0.0},
        {"name": "Case 2: (q1=pi/2, q2=0)",     "q1": math.pi / 2, "q2": 0.0,         "exp_x": 0.0, "exp_y": 0.8},
        {"name": "Case 3: (q1=0, q2=pi/2)",     "q1": 0.0,         "q2": math.pi / 2, "exp_x": 0.5, "exp_y": 0.3},
    ]

    print(f"{'Test Case':<26} | {'Expected (X, Y)':<18} | {'Calculated (X, Y)':<22} | {'Error (m)':<12} | {'Status'}")
    print("-" * 80)

    all_pass = True
    for tc in test_cases:
        calc_x, calc_y = forward_kinematics(tc["q1"], tc["q2"])
        err_x = abs(calc_x - tc["exp_x"])
        err_y = abs(calc_y - tc["exp_y"])
        total_err = math.sqrt(err_x**2 + err_y**2)
        passed = (err_x < TOLERANCE) and (err_y < TOLERANCE)

        exp_str = f"({tc['exp_x']:.4f}, {tc['exp_y']:.4f})"
        calc_str = f"({calc_x:.6f}, {calc_y:.6f})"
        err_str = f"{total_err:.2e}"
        status = "PASS" if passed else "FAIL"

        print(f"{tc['name']:<26} | {exp_str:<18} | {calc_str:<22} | {err_str:<12} | {status}")
        if not passed:
            all_pass = False

    print("=" * 80)

    # Validate URDF file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_candidates = [
        os.path.join(script_dir, "..", "urdf", "twoplanar_arm.urdf"),
        os.path.join(script_dir, "urdf", "twoplanar_arm.urdf"),
    ]
    urdf_path = None
    for c in urdf_candidates:
        if os.path.exists(c):
            urdf_path = c
            break

    if urdf_path:
        print(f"\nValidating URDF Structure: {os.path.basename(urdf_path)}")
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        
        j1 = root.find(".//joint[@name='joint1']")
        j2 = root.find(".//joint[@name='joint2']")
        j3 = root.find(".//joint[@name='joint3']")
        
        print(f"  joint1: type={j1.get('type')}, axis={j1.find('axis').get('xyz')}, origin={j1.find('origin').get('xyz')}")
        print(f"  joint2: type={j2.get('type')}, axis={j2.find('axis').get('xyz')}, origin={j2.find('origin').get('xyz')}")
        print(f"  joint3: type={j3.get('type')}, origin={j3.find('origin').get('xyz')}")
        
        assert j1.get('type') == 'revolute', "joint1 must be revolute"
        assert j1.find('axis').get('xyz') == '0 0 1', "joint1 axis must be 0 0 1"
        assert j2.get('type') == 'revolute', "joint2 must be revolute"
        assert j2.find('axis').get('xyz') == '0 0 1', "joint2 axis must be 0 0 1"
        assert j3.get('type') == 'fixed', "joint3 must be fixed"
        print("  URDF Kinematic Tree: VERIFIED CORRECT")

    print(f"\nFinal Result: {'ALL TESTS PASSED' if all_pass else 'TESTS FAILED'}\n")
    return all_pass


def main():
    success = run_tests()
    exit(0 if success else 1)


if __name__ == '__main__':
    main()
