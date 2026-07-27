#!/usr/bin/env python3
"""Multi-axis arm built from Stepper joints sharing one gpiochip handle.

Pin map (BCM), common-cathode direct-to-3.3V wiring:
    base     STEP=GPIO17  DIR=GPIO27
    shoulder STEP=GPIO23  DIR=GPIO24
    elbow    STEP=GPIO5   DIR=GPIO6
All '-' terminals bus to Pi GND; ENA disconnected (drivers always enabled).

If a joint runs the wrong way, set invert=True for it in JOINTS.
"""
import lgpio
from stepper import Stepper

# name -> (step_pin, dir_pin, invert_dir)
JOINTS = {
    "base":     (17, 27, False),
    "shoulder": (23, 24, False),
    "elbow":    (5,  6,  False),
}


class Arm:
    def __init__(self, joints=JOINTS, **stepper_kwargs):
        self.h = lgpio.gpiochip_open(0)
        self.motors = {
            name: Stepper(self.h, step, dir_, invert_dir=inv, **stepper_kwargs)
            for name, (step, dir_, inv) in joints.items()
        }

    def __getitem__(self, name):
        return self.motors[name]

    def move(self, name, steps, max_pps=None):
        self.motors[name].move(steps, max_pps=max_pps)

    def move_degrees(self, name, degrees, max_pps=None):
        self.motors[name].move_degrees(degrees, max_pps=max_pps)

    def home_all(self, max_pps=None):
        """Return every joint to its start position, one at a time."""
        for m in self.motors.values():
            m.go_home(max_pps=max_pps)

    def angles(self):
        return {name: round(m.angle, 1) for name, m in self.motors.items()}

    def close(self):
        for m in self.motors.values():
            m.close()
        lgpio.gpiochip_close(self.h)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
