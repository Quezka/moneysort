"""Vacuum pickup: diaphragm pump + 3/2 release valve, each switched by a logic-level MOSFET (docs/PUMP.md).

    pump GPIO  -> MOSFET gate (pump runs while HIGH)
    valve GPIO -> MOSFET gate (valve energised while HIGH = cup connected to the pump;
                               de-energised = cup vented to the air, so a dead arm drops the coin)

Both outputs are plain on/off writes on purpose: lgpio's tx_pwm shares the pulse thread with the steppers
(see hardware/stepper.TxAnchor), and a PWM record there would reopen the "TX queue stuck" wedge.

A dead-man timer releases the coin if nobody calls `keepalive()` in time (a lost connection must never leave
the pump running): `grip(ttl)` arms it, `keepalive()` re-arms it, `release()` disarms it.
"""
import logging
import threading

import lgpio

log = logging.getLogger("armd.gripper")


class Gripper:
    def __init__(self, handle, pump_pin, valve_pin):
        self.h = handle
        self.pump_pin = pump_pin
        self.valve_pin = valve_pin
        self.ttl = None
        self._timer = None
        self._lock = threading.Lock()
        self.gripping = False
        # claim with the safe level (pump off, valve venting) already driven
        lgpio.gpio_claim_output(self.h, self.pump_pin, 0)
        lgpio.gpio_claim_output(self.h, self.valve_pin, 0)

    def grip(self, ttl=2.0):
        """Valve to the pump, then the pump on. Releases by itself `ttl` seconds after the last keepalive."""
        with self._lock:
            lgpio.gpio_write(self.h, self.valve_pin, 1)
            lgpio.gpio_write(self.h, self.pump_pin, 1)
            self.gripping = True
            self.ttl = ttl
            self._arm()

    def keepalive(self):
        with self._lock:
            if self.gripping:
                self._arm()

    def release(self):
        """Pump off and the cup vented (valve off), in that order: no pressure spike against a closed line."""
        with self._lock:
            self._disarm()
            lgpio.gpio_write(self.h, self.pump_pin, 0)
            lgpio.gpio_write(self.h, self.valve_pin, 0)
            self.gripping = False

    def close(self):
        self.release()
        lgpio.gpio_free(self.h, self.pump_pin)
        lgpio.gpio_free(self.h, self.valve_pin)

    def status(self):
        return {"gripping": self.gripping, "ttl": self.ttl}

    def _arm(self):
        self._disarm()
        if self.ttl:
            self._timer = threading.Timer(self.ttl, self._expired)
            self._timer.daemon = True
            self._timer.start()

    def _disarm(self):
        if self._timer is not None:
            self._timer.cancel()
            self._timer = None

    def _expired(self):
        log.warning("gripper dead-man expired: releasing")
        self.release()
