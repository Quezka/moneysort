#!/usr/bin/env python3
"""Reusable stepper control for STEP/DIR drivers (2DM442), using lgpio.

Why lgpio instead of a Python sleep() loop: hand-timing pulses with time.sleep()
tops out around 1-2 kHz because the OS scheduler can't sleep for less than
~50-100 us. lgpio.tx_pulse() generates the pulse train with hardware-assisted
timing, so we can push tens of kHz and get smooth, fast motion.

Speed control uses a trapezoidal profile: the move is split into a ramp-up, a
cruise, and a ramp-down, each queued as one or more tx_pulse "bursts" that lgpio
plays back-to-back. Ramping avoids stalling the motor by asking it to jump
straight to full speed from a standstill.

Wiring (direct 3.3V common-cathode - no level shifter needed):
    STEP gpio -> PUL+     DIR gpio -> DIR+
    all '-' terminals bussed to Pi GND
"""
import time
import lgpio

TX_PWM = 0  # lgpio kind selector for tx_busy / tx_room


class Stepper:
    def __init__(self, handle, step_pin, dir_pin, en_pin=None,
                 steps_per_rev=200, microsteps=1, invert_dir=False,
                 min_pps=400, max_pps=4000, accel_steps=800):
        """
        handle        : open gpiochip handle (lgpio.gpiochip_open(0))
        step_pin      : BCM gpio wired to PUL+
        dir_pin       : BCM gpio wired to DIR+
        en_pin        : BCM gpio wired to ENA+ (optional)
        steps_per_rev : motor full steps/rev (NEMA 17 = 200)
        microsteps    : driver microstep setting (set by the DIP switches)
        invert_dir    : flip if positive moves the "wrong" way
        min_pps       : starting/ending pulse rate of the ramp
        max_pps       : cruise pulse rate
        accel_steps   : how many steps the ramp-up (and ramp-down) span
        """
        self.h = handle
        self.step_pin = step_pin
        self.dir_pin = dir_pin
        self.en_pin = en_pin
        self.invert_dir = invert_dir
        self.eff_spr = steps_per_rev * microsteps   # effective steps per rev
        self.min_pps = min_pps
        self.max_pps = max_pps
        self.accel_steps = accel_steps
        self.position = 0                           # net steps from home

        lgpio.gpio_claim_output(handle, step_pin, 0)
        lgpio.gpio_claim_output(handle, dir_pin, 0)
        if en_pin is not None:
            lgpio.gpio_claim_output(handle, en_pin, 0)

    # --- internals --------------------------------------------------------
    def _burst(self, pps, cycles):
        """Queue one constant-rate burst of `cycles` pulses at `pps`."""
        if cycles <= 0:
            return
        half = max(int(round(1_000_000 / pps / 2)), 1)   # microseconds, >=1
        while lgpio.tx_room(self.h, self.step_pin, TX_PWM) < 1:
            time.sleep(0.001)                            # wait for queue space
        lgpio.tx_pulse(self.h, self.step_pin, half, half, 0, cycles)

    def _ramp(self, ramp_steps, max_pps, up, k=16):
        """Split a ramp into k bursts of rising (up) or falling speed."""
        if ramp_steps <= 0:
            return
        base, rem = divmod(ramp_steps, k)
        for i in range(k):
            cyc = base + (1 if i < rem else 0)
            frac = (i + 1) / k
            if up:
                pps = self.min_pps + (max_pps - self.min_pps) * frac
            else:
                pps = max_pps - (max_pps - self.min_pps) * frac
            self._burst(max(pps, self.min_pps), cyc)

    # --- public API -------------------------------------------------------
    def move(self, steps, max_pps=None):
        """Move a signed number of steps (+/-) with a trapezoidal ramp."""
        if steps == 0:
            return
        max_pps = max_pps or self.max_pps
        direction = 1 if steps > 0 else 0
        lgpio.gpio_write(self.h, self.dir_pin, direction ^ int(self.invert_dir))
        time.sleep(0.001)                     # DIR setup time

        n = abs(steps)
        ramp = min(self.accel_steps, n // 2)
        cruise = n - 2 * ramp

        self._ramp(ramp, max_pps, up=True)    # accelerate
        self._burst(max_pps, cruise)          # cruise
        self._ramp(ramp, max_pps, up=False)   # decelerate

        while lgpio.tx_busy(self.h, self.step_pin, TX_PWM):
            time.sleep(0.005)                 # block until the train finishes
        self.position += steps

    def move_degrees(self, degrees, max_pps=None):
        self.move(round(degrees / 360.0 * self.eff_spr), max_pps=max_pps)

    def go_home(self, max_pps=None):
        self.move(-self.position, max_pps=max_pps)

    @property
    def angle(self):
        return self.position / self.eff_spr * 360.0

    def close(self):
        lgpio.tx_pulse(self.h, self.step_pin, 0, 0)   # stop any pulses
        lgpio.gpio_free(self.h, self.step_pin)
        lgpio.gpio_free(self.h, self.dir_pin)
        if self.en_pin is not None:
            lgpio.gpio_free(self.h, self.en_pin)
