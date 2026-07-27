#!/usr/bin/env python3
"""Jog one axis by name - use this to verify the X/Y/Z <-> pin mapping.

Usage (run on the Pi):
    python3 arm_test.py <x|y|z> [steps] [pps]
        steps : signed step count (+ / - sets direction), default 800
        pps   : cruise pulses per second, default 4000

Start small (e.g. `python3 arm_test.py x 400`) and watch WHICH physical joint
moves. If it's not the one you call X, swap the pin rows in arm.py's JOINTS.
"""
import sys
from arm import Arm

axis = sys.argv[1] if len(sys.argv) > 1 else "y"
steps = int(sys.argv[2]) if len(sys.argv) > 2 else 800
pps = int(sys.argv[3]) if len(sys.argv) > 3 else 4000

arm = Arm(max_pps=pps, accel_steps=400)
try:
    print(f"Jogging axis '{axis}' by {steps} steps @ {pps} pps ...")
    arm.move(axis, steps)
    print("done. angles:", arm.angles())
finally:
    arm.close()
