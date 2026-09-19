#!/usr/bin/env python3
"""Multi-axis arm built from Stepper joints sharing one gpiochip handle.

Pins and calibration live in `moneysort.config.JOINTS` (see docs/PINOUT.md):
    x = elbow (home switch GPIO7)   y = shoulder (home switch GPIO8)
    z = base (continuous, no switch)
Shared enable on GPIO26 (active-low). All '-' terminals bus to Pi GND.
If a joint runs the wrong way, set "invert": True for it in JOINTS.
"""
import threading
import time

import lgpio

from moneysort.config import ENABLE_PIN, GPIOCHIP, JOINTS
from moneysort.hardware.stepper import Stepper


class Arm:
    def __init__(self, joints=JOINTS, enable_pin=ENABLE_PIN, **stepper_kwargs):
        self.h = lgpio.gpiochip_open(GPIOCHIP)
        self.enable_pin = enable_pin
        # Claim LOW so all drivers come up enabled/holding.
        lgpio.gpio_claim_output(self.h, enable_pin, 0)
        self._enabled = True
        self.cfg = joints                       # per-axis config (limits, homed, ...)
        # Shared e-stop flag: disable() sets it so any in-flight move on any axis
        # bails out at once; enable() clears it. Handed to every Stepper.
        self.abort = threading.Event()
        self.motors = {}
        for name, c in joints.items():
            kw = dict(stepper_kwargs)
            if "steps_per_rev" in c:
                kw["steps_per_rev"] = c["steps_per_rev"]
                kw.setdefault("microsteps", 1)
            self.motors[name] = Stepper(
                self.h, c["step"], c["dir"],
                invert_dir=c.get("invert", False), abort=self.abort, **kw)
        # Claim home-switch pins as pull-up inputs (NC to GND).
        self.home_pins = {}
        for name, c in joints.items():
            if "home_pin" in c:
                lgpio.gpio_claim_input(self.h, c["home_pin"], lgpio.SET_PULL_UP)
                self.home_pins[name] = c["home_pin"]
        # Soft travel limits (enforced only once an axis is homed). Home sits at
        # position 0; the usable range extends opposite the home direction.
        self._limits = {}
        for name, c in joints.items():
            t = c.get("travel")
            if t:
                self._limits[name] = (0, t) if c.get("home_dir", -1) < 0 else (-t, 0)
        self.homed = set()

    # --- shared enable ----------------------------------------------------
    def enable(self):
        """Energize all drivers (motors hold). Always works: no opto current."""
        self.abort.clear()                       # lift the e-stop latch on motion
        lgpio.gpio_write(self.h, self.enable_pin, 0)
        self._enabled = True

    def disable(self):
        """Release all drivers (motors go free) and abort any motion in flight.

        Setting `abort` makes every running move/home loop stop feeding pulses
        and cut its train short, so an emergency disable stops the *task*, not
        just the torque. Positions are now unknown after an abrupt stop, so all
        axes are marked un-homed -- soft limits stay off until you re-home.
        """
        self.abort.set()
        lgpio.gpio_write(self.h, self.enable_pin, 1)
        self._enabled = False
        self.homed.clear()

    @property
    def enabled(self):
        return self._enabled

    def __getitem__(self, name):
        return self.motors[name]

    def _clamp_steps(self, name, steps):
        """Trim `steps` so a homed, travel-limited axis can't overtravel."""
        lim = self._limits.get(name)
        if lim is None or name not in self.homed:
            return steps
        lo, hi = lim
        pos = self.motors[name].position
        return max(lo, min(hi, pos + steps)) - pos

    def move(self, name, steps, max_pps=None):
        steps = self._clamp_steps(name, int(steps))
        self.motors[name].move(steps, max_pps=max_pps)

    def move_degrees(self, name, degrees, max_pps=None):
        m = self.motors[name]
        self.move(name, round(degrees / 360.0 * m.eff_spr), max_pps=max_pps)

    def home_all(self, max_pps=None):
        """Home the whole arm in one call, one axis at a time.

        Axes with a home switch (x, y) seek it via find_home and re-zero on it;
        switch-less axes (z/base) just drive back to their existing zero (no-op
        if already there). Stops early if the e-stop abort fires.
        """
        for name, m in self.motors.items():
            if self.abort.is_set():
                break
            if name in self.home_pins:
                self.find_home(name)
            else:
                m.go_home(max_pps=max_pps)

    def return_zero(self, max_pps=None):
        """Drive every axis back to its zero position, one at a time.

        Unlike home_all this seeks no switch -- it just undoes the tracked net
        motion (a no-op for an axis already at 0), so it relies on the current
        position being trusted (homed or freshly zeroed). This is the everyday
        "go home" move; switch homing is the occasional re-init.
        """
        for m in self.motors.values():
            if self.abort.is_set():
                break
            m.go_home(max_pps=max_pps)

    def move_many(self, moves, max_pps=None):
        """Move several axes at once. moves = {axis: steps}.

        Each STEP pin has its own lgpio tx queue that plays independently, so we
        set every axis's direction, then interleave-feed their trapezoid bursts
        (round-robin as each queue frees room) -- all axes ramp and run together.
        """
        plans = {}
        for name, steps in moves.items():
            steps = self._clamp_steps(name, int(steps))
            if steps == 0:
                continue
            m = self.motors[name]
            level, segs = m.plan(steps, max_pps)
            m.set_dir(level)
            plans[name] = [m, list(segs), steps]
        if not plans:
            return
        time.sleep(0.001)                      # DIR setup for all axes

        pending = {name: p[1] for name, p in plans.items()}
        while pending and not self.abort.is_set():
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
        for m, _, _ in plans.values():     # drain every queue (bursts are
            while m.busy():                # short, so an abort clears fast)
                time.sleep(0.005)
        if not self.abort.is_set():        # positions unknown after an abort
            for m, _, steps in plans.values():
                m.position += steps

    def zero(self, axis=None):
        """Define the current position as 0 (manual home). One axis or all."""
        motors = [self.motors[axis]] if axis else self.motors.values()
        for m in motors:
            m.position = 0

    def at_home(self, axis):
        """True if the axis's home switch is actuated (NC open = HIGH)."""
        pin = self.home_pins.get(axis)
        if pin is None:
            return False
        return lgpio.gpio_read(self.h, pin) == 1

    def _stable_home(self, axis, want, n=4, poll=0.003):
        """True once at_home(axis) equals `want` for `n` consecutive reads.

        Debounces the switch so the fine approach seats firmly past the noisy
        trigger edge instead of stopping on the first flicker -- an axis that
        rests right on the threshold (x) then ends up solidly on the switch,
        exactly like one with margin (y).
        """
        for _ in range(n):
            if self.at_home(axis) != want:
                return False
            time.sleep(poll)
        return True

    def find_home(self, axis, fast_pps=15000, slow_pps=500,
                  backoff=800, fine=8):
        """Seek the home switch and define that point as position 0.

        Fast approach to first touch -> back off until released (+ clearance)
        -> slow fine approach until the switch reads *stably* triggered. Bails
        immediately (without zeroing) if the e-stop abort fires mid-seek.
        """
        c = self.cfg[axis]
        if axis not in self.home_pins:
            raise ValueError(f"axis {axis!r} has no home switch")
        m = self.motors[axis]
        hd = 1 if c.get("home_dir", -1) >= 0 else -1     # +/-1 toward the switch
        home = lambda: self.at_home(axis)

        # 1. fast approach to first touch (skip if already on the switch)
        if not home():
            m.home_seek(hd, fast_pps, home)
        # 2. back off until released, plus a little clearance
        while home() and not self.abort.is_set():
            m.jog(-hd * fine, slow_pps)
        m.jog(-hd * backoff, slow_pps)
        # 3. slow fine approach until the switch is *solidly* pressed
        while not self._stable_home(axis, True) and not self.abort.is_set():
            m.jog(hd * fine, slow_pps)
        if self.abort.is_set():            # e-stopped: don't claim a home
            return
        m.position = 0
        self.homed.add(axis)               # enable soft limits for this axis

    def angles(self):
        return {name: round(m.angle, 1) for name, m in self.motors.items()}

    def close(self):
        for m in self.motors.values():
            m.close()
        for pin in self.home_pins.values():
            lgpio.gpio_free(self.h, pin)
        lgpio.gpio_free(self.h, self.enable_pin)
        lgpio.gpiochip_close(self.h)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
