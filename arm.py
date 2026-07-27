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

# axis -> (step_pin, dir_pin, invert_dir)   BCM numbering; see docs/PINOUT.md
JOINTS = {
    "x": (23, 24, False),   # PUL=GPIO23 (pin16), DIR=GPIO24 (pin18)
    "y": (17, 27, False),   # PUL=GPIO17 (pin11), DIR=GPIO27 (pin13)
    "z": (5,  6,  False),   # PUL=GPIO5  (pin29), DIR=GPIO6  (pin31)
}

# Single shared enable line: every driver's ENA+ ties to this pin.
# Active-low logic: pin LOW = ENA opto off = drivers ENABLED (motors hold);
# pin HIGH = disabled. Owned by the Arm (not the Steppers) because lgpio can't
# let three Steppers each claim the same pin.
ENABLE_PIN = 22


class Arm:
    def __init__(self, joints=JOINTS, enable_pin=ENABLE_PIN, **stepper_kwargs):
        self.h = lgpio.gpiochip_open(0)
        self.enable_pin = enable_pin
        # Claim LOW so all drivers come up enabled/holding.
        lgpio.gpio_claim_output(self.h, enable_pin, 0)
        self._enabled = True
        self.motors = {
            name: Stepper(self.h, step, dir_, invert_dir=inv, **stepper_kwargs)
            for name, (step, dir_, inv) in joints.items()
        }

    # --- shared enable ----------------------------------------------------
    def enable(self):
        """Energize all drivers (motors hold). Always works: no opto current."""
        lgpio.gpio_write(self.h, self.enable_pin, 0)
        self._enabled = True

    def disable(self):
        """Release all drivers (motors go free).

        Caveat: this drives one 3.3V pin into ~3 ENA optos in parallel, which is
        current-marginal at 3.3V -- confirm the motors actually release before
        depending on it. Also: a gravity-loaded joint will sag when disabled.
        """
        lgpio.gpio_write(self.h, self.enable_pin, 1)
        self._enabled = False

    @property
    def enabled(self):
        return self._enabled

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
        lgpio.gpio_free(self.h, self.enable_pin)
        lgpio.gpiochip_close(self.h)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
