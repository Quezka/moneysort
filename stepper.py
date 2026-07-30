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
import threading
import time
import lgpio

TX_PWM = 0  # lgpio kind selector for tx_busy / tx_room


class Stepper:
    def __init__(self, handle, step_pin, dir_pin, en_pin=None,
                 steps_per_rev=200, microsteps=1, invert_dir=False,
                 min_pps=400, max_pps=20000, accel_steps=800, abort=None):
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
        abort         : shared threading.Event; when set, any in-flight move
                        bails out and truncates its pulse train (emergency stop)
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
        self.abort = abort if abort is not None else threading.Event()
        self.position = 0                           # net steps from home

        lgpio.gpio_claim_output(handle, step_pin, 0)
        lgpio.gpio_claim_output(handle, dir_pin, 0)
        if en_pin is not None:
            lgpio.gpio_claim_output(handle, en_pin, 0)

    # --- internals --------------------------------------------------------
    def _burst(self, pps, cycles):
        """Queue one constant-rate burst of `cycles` pulses at `pps` (blocking)."""
        if cycles <= 0 or self.abort.is_set():
            return
        half = max(int(round(1_000_000 / pps / 2)), 1)   # microseconds, >=1
        while lgpio.tx_room(self.h, self.step_pin, TX_PWM) < 1:
            if self.abort.is_set():                      # e-stop: stop feeding
                return
            time.sleep(0.001)                            # wait for queue space
        lgpio.tx_pulse(self.h, self.step_pin, half, half, 0, cycles)

    def _halt(self):
        """Truncate whatever is currently transmitting on the step pin.

        A fresh tx_pulse replaces the in-flight train with a single final cycle
        (same idiom home_seek uses to stop its infinite seek burst), so a long
        cruise stops within a pulse instead of playing out to the end.
        """
        half = max(int(round(1_000_000 / self.max_pps / 2)), 1)
        try:
            lgpio.tx_pulse(self.h, self.step_pin, half, half, 0, 1)
        except Exception:
            pass

    def _ramp_segs(self, ramp_steps, max_pps, up, k=16):
        """Return [(pps, cycles), ...] for a rising (up) or falling ramp."""
        segs = []
        if ramp_steps <= 0:
            return segs
        base, rem = divmod(ramp_steps, k)
        for i in range(k):
            cyc = base + (1 if i < rem else 0)
            if cyc <= 0:
                continue
            frac = (i + 1) / k
            if up:
                pps = self.min_pps + (max_pps - self.min_pps) * frac
            else:
                pps = max_pps - (max_pps - self.min_pps) * frac
            segs.append((max(pps, self.min_pps), cyc))
        return segs

    def plan(self, steps, max_pps=None):
        """Build a move: returns (dir_level, [(pps, cycles), ...]).

        Shared by move() and Arm.move_many(); the latter interleaves several
        steppers' segment lists to drive multiple axes at once.
        """
        max_pps = max_pps or self.max_pps
        direction = 1 if steps < 0 else 0
        level = direction ^ int(self.invert_dir)
        n = abs(steps)
        ramp = min(self.accel_steps, n // 2)
        cruise = n - 2 * ramp
        segs = self._ramp_segs(ramp, max_pps, up=True)
        if cruise > 0:
            segs.append((max_pps, cruise))
        segs += self._ramp_segs(ramp, max_pps, up=False)
        return level, segs

    def set_dir(self, level):
        lgpio.gpio_write(self.h, self.dir_pin, level)

    def try_queue(self, pps, cycles):
        """Non-blocking: queue one burst if the tx queue has room, else False."""
        if cycles <= 0:
            return True
        if lgpio.tx_room(self.h, self.step_pin, TX_PWM) < 1:
            return False
        half = max(int(round(1_000_000 / pps / 2)), 1)
        lgpio.tx_pulse(self.h, self.step_pin, half, half, 0, cycles)
        return True

    def busy(self):
        return lgpio.tx_busy(self.h, self.step_pin, TX_PWM)

    # --- public API -------------------------------------------------------
    def move(self, steps, max_pps=None):
        """Move a signed number of steps (+/-) with a trapezoidal ramp (blocking).

        If the abort event fires mid-move it stops feeding bursts, truncates the
        pulse train, and leaves position unchanged (the true stop point is
        unknown -- the caller should treat the axis as no longer homed).
        """
        if steps == 0:
            return
        level, segs = self.plan(steps, max_pps)
        self.set_dir(level)
        time.sleep(0.001)                     # DIR setup time
        for pps, cyc in segs:
            if self.abort.is_set():
                break
            self._burst(pps, cyc)
        while self.busy():
            if self.abort.is_set():
                self._halt()                  # cut the in-flight train short
            time.sleep(0.005)                 # block until the train finishes
        if not self.abort.is_set():
            self.position += steps

    def move_degrees(self, degrees, max_pps=None):
        self.move(round(degrees / 360.0 * self.eff_spr), max_pps=max_pps)

    def go_home(self, max_pps=None):
        self.move(-self.position, max_pps=max_pps)

    def jog(self, steps, pps):
        """Constant-speed move with no accel ramp -- for slow homing chunks."""
        if steps == 0 or self.abort.is_set():
            return
        level = (1 if steps < 0 else 0) ^ int(self.invert_dir)
        self.set_dir(level)
        time.sleep(0.001)
        self._burst(pps, abs(steps))
        while self.busy():
            if self.abort.is_set():
                self._halt()
            time.sleep(0.005)
        if not self.abort.is_set():
            self.position += steps

    def home_seek(self, direction, pps, stop_fn, accel_steps=400,
                  poll=0.0005):
        """Ramp up toward `direction` and run until stop_fn() is True.

        For the fast phase of homing: ramps to `pps` (no stall), cruises
        indefinitely while polling stop_fn(), then halts promptly on contact.
        Position is NOT tracked here -- the caller re-zeroes at the switch.
        """
        level = (1 if direction < 0 else 0) ^ int(self.invert_dir)
        self.set_dir(level)
        time.sleep(0.001)
        stopped = False

        # queue ramp-up bursts then one infinite cruise burst (cyc=0),
        # polling for queue room + trigger so we can bail during the ramp too
        for p, cyc in self._ramp_segs(accel_steps, pps, up=True) + [(pps, 0)]:
            while lgpio.tx_room(self.h, self.step_pin, TX_PWM) < 1:
                if self.abort.is_set():
                    stopped = True
                    break
                time.sleep(poll)
            if stopped:
                break
            ph = max(int(round(1_000_000 / p / 2)), 1)
            lgpio.tx_pulse(self.h, self.step_pin, ph, ph, 0, cyc)

        while not stopped:                       # cruise: poll until trigger
            if stop_fn() or self.abort.is_set():
                break
            time.sleep(poll)

        # halt: replace the (infinite) train with a single final cycle
        half = max(int(round(1_000_000 / pps / 2)), 1)
        lgpio.tx_pulse(self.h, self.step_pin, half, half, 0, 1)
        while self.busy():
            time.sleep(0.001)

    @property
    def angle(self):
        return self.position / self.eff_spr * 360.0

    def close(self):
        # move() already waits for the train to finish, so just park low & free.
        lgpio.gpio_write(self.h, self.step_pin, 0)
        lgpio.gpio_free(self.h, self.step_pin)
        lgpio.gpio_free(self.h, self.dir_pin)
        if self.en_pin is not None:
            lgpio.gpio_free(self.h, self.en_pin)
