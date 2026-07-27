#!/usr/bin/env python3
"""Multi-axis arm built from Stepper joints sharing one gpiochip handle.

Axis map (BCM), common-cathode direct-to-3.3V wiring (see docs/PINOUT.md):
    x = elbow     STEP=GPIO5   DIR=GPIO6
    y = shoulder  STEP=GPIO17  DIR=GPIO27   (home switch at start; homing TODO)
    z = base      STEP=GPIO23  DIR=GPIO24   (continuous rotation)
Shared enable on GPIO26. All '-' terminals bus to Pi GND.

If a joint runs the wrong way, set "invert": True for it in JOINTS.
"""
import json
import os
import time

import lgpio
from stepper import Stepper

# Runtime state file read by dashboard.py (same directory as this module).
STATE_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "state.json")

# axis -> config (BCM pins + calibration). See docs/PINOUT.md.
#   step, dir     : BCM gpio for PUL+ / DIR+
#   invert        : flip if positive moves the "wrong" way
#   steps_per_rev : measured steps for a full 360 deg output turn (folds in
#                   microstepping + gearing) -- makes move_degrees() accurate
#   travel        : usable range in steps for a limited joint (home switch)
#   homed         : has a home/limit switch (homing itself is TODO)
JOINTS = {
    "x": {"step": 5,  "dir": 6,  "invert": False},                            # elbow (uncalibrated)
    "y": {"step": 17, "dir": 27, "invert": False,
          "travel": 33000, "homed": True},                                    # shoulder: ~33k-step range, home switch at start
    "z": {"step": 23, "dir": 24, "invert": False, "steps_per_rev": 162000},   # base: continuous, 90 deg = 40500 steps -> 360 deg = 162000
}

# Single shared enable line: every driver's ENA+ ties to this pin.
# Active-low logic: pin LOW = ENA opto off = drivers ENABLED (motors hold);
# pin HIGH = disabled. Owned by the Arm (not the Steppers) because lgpio can't
# let three Steppers each claim the same pin.
ENABLE_PIN = 26   # BCM GPIO26 = header pin 37


class Arm:
    def __init__(self, joints=JOINTS, enable_pin=ENABLE_PIN, **stepper_kwargs):
        self.h = lgpio.gpiochip_open(0)
        self.enable_pin = enable_pin
        # Claim LOW so all drivers come up enabled/holding.
        lgpio.gpio_claim_output(self.h, enable_pin, 0)
        self._enabled = True
        self.cfg = joints                       # per-axis config (limits, homed, ...)
        self.motors = {}
        for name, c in joints.items():
            kw = dict(stepper_kwargs)
            if "steps_per_rev" in c:
                kw["steps_per_rev"] = c["steps_per_rev"]
                kw.setdefault("microsteps", 1)
            self.motors[name] = Stepper(
                self.h, c["step"], c["dir"],
                invert_dir=c.get("invert", False), **kw)
        self._write_state()

    # --- dashboard state --------------------------------------------------
    def _write_state(self, moving=False):
        """Atomically dump joint angles + status to state.json for the dashboard."""
        state = {
            "joints": {name: round(m.angle, 1) for name, m in self.motors.items()},
            "moving": moving,
            "enabled": self._enabled,
        }
        try:
            tmp = STATE_PATH + ".tmp"
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, STATE_PATH)   # atomic swap; dashboard never sees a partial file
        except OSError:
            pass

    # --- shared enable ----------------------------------------------------
    def enable(self):
        """Energize all drivers (motors hold). Always works: no opto current."""
        lgpio.gpio_write(self.h, self.enable_pin, 0)
        self._enabled = True
        self._write_state()

    def disable(self):
        """Release all drivers (motors go free)."""
        lgpio.gpio_write(self.h, self.enable_pin, 1)
        self._enabled = False
        self._write_state()

    @property
    def enabled(self):
        return self._enabled

    def __getitem__(self, name):
        return self.motors[name]

    def move(self, name, steps, max_pps=None):
        self._write_state(moving=True)
        try:
            self.motors[name].move(steps, max_pps=max_pps)
        finally:
            self._write_state(moving=False)

    def move_degrees(self, name, degrees, max_pps=None):
        self._write_state(moving=True)
        try:
            self.motors[name].move_degrees(degrees, max_pps=max_pps)
        finally:
            self._write_state(moving=False)

    def home_all(self, max_pps=None):
        """Return every joint to its start position, one at a time."""
        self._write_state(moving=True)
        try:
            for m in self.motors.values():
                m.go_home(max_pps=max_pps)
        finally:
            self._write_state(moving=False)

    def move_many(self, moves, max_pps=None):
        """Move several axes at once. moves = {axis: steps}.

        Each STEP pin has its own lgpio tx queue that plays independently, so we
        set every axis's direction, then interleave-feed their trapezoid bursts
        (round-robin as each queue frees room) -- all axes ramp and run together.
        """
        plans = {}
        for name, steps in moves.items():
            steps = int(steps)
            if steps == 0:
                continue
            m = self.motors[name]
            level, segs = m.plan(steps, max_pps)
            m.set_dir(level)
            plans[name] = [m, list(segs), steps]
        if not plans:
            return
        time.sleep(0.001)                      # DIR setup for all axes

        self._write_state(moving=True)
        try:
            pending = {name: p[1] for name, p in plans.items()}
            while pending:
                progressed = False
                for name in list(pending):
                    segs = pending[name]
                    if plans[name][0].try_queue(*segs[0]):
                        segs.pop(0)
                        progressed = True
                        if not segs:
                            del pending[name]
                if not progressed:
                    time.sleep(0.001)          # all queues full; let them drain
            for m, _, _ in plans.values():     # wait for every train to finish
                while m.busy():
                    time.sleep(0.005)
            for m, _, steps in plans.values():
                m.position += steps
        finally:
            self._write_state(moving=False)

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
