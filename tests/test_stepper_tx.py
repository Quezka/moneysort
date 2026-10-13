"""Stepper tx-queue handling against a fake lgpio (no hardware).

Regression: homing's stop used an unchecked tx_pulse; with the queue still full it
raised "TX queue full" and the infinite cruise burst kept running forever.
"""
import sys
import threading
import time
import types
import unittest


class FakeLgpio(types.ModuleType):
    class error(Exception):
        pass

    SET_PULL_UP = 0

    def __init__(self):
        super().__init__("lgpio")
        self.full_for = 0            # tx_room reports 0 for this many calls
        self.raise_for = 0           # tx_pulse raises "TX queue full" this many times
        self.pulses = []             # (cycles) in order
        self.infinite = False        # an infinite burst is the last thing queued
        self.full_until = 0.0        # queue reports full until this monotonic time

    def gpiochip_open(self, n): return 1
    def gpio_claim_output(self, *a): pass
    def gpio_write(self, *a): pass
    def gpio_free(self, *a): pass

    def tx_room(self, h, pin, kind):
        if time.monotonic() < self.full_until:
            return 0
        if self.full_for > 0:
            self.full_for -= 1
            return 0
        return 8

    def tx_pulse(self, h, pin, on, off, off2, cycles):
        if time.monotonic() < self.full_until:
            raise self.error("TX queue full")
        if self.full_for > 0:         # queued while tx_room() still says full -> lgpio refuses
            raise self.error("TX queue full")
        if self.raise_for > 0:
            self.raise_for -= 1
            raise self.error("TX queue full")
        self.pulses.append(cycles)
        self.infinite = cycles == 0   # a later pulse ends an infinite one
        if cycles == 0:               # the queue still holds ramp bursts behind/around it for a while
            self.full_until = time.monotonic() + 0.05

    def tx_busy(self, h, pin, kind):
        return self.infinite


fake = FakeLgpio()
sys.modules["lgpio"] = fake
from moneysort.hardware import stepper as st   # noqa: E402


def make():
    fake.full_for = fake.raise_for = 0
    fake.full_until = 0.0
    fake.pulses.clear()
    fake.infinite = False
    return st.Stepper(1, 5, 6, steps_per_rev=1000)


class TestTx(unittest.TestCase):
    def test_retries_when_queue_full(self):
        m = make()
        fake.raise_for = 3
        self.assertTrue(m._tx(1000, 10))
        self.assertEqual(fake.pulses, [10])

    def test_stuck_queue_raises_instead_of_hanging(self):
        m = make()
        fake.full_for = 10 ** 9
        t0 = time.monotonic()
        with self.assertRaises(RuntimeError):
            m._tx(1000, 10, timeout=0.05)
        self.assertLess(time.monotonic() - t0, 1.0)

    def test_halt_ends_an_infinite_train_even_if_queue_is_full_at_first(self):
        m = make()
        fake.infinite = True
        fake.full_for = 20
        self.assertTrue(m.halt(wait=1.0))
        self.assertFalse(fake.infinite)

    def test_home_seek_stops_even_when_switch_trips_with_full_queue(self):
        m = make()
        fake.full_for = 15                     # queue full while the ramp is being queued
        m.home_seek(+1, 15000, lambda: True, accel_steps=50)   # switch already tripped
        self.assertFalse(fake.infinite, "infinite cruise burst left running")

    def test_home_seek_halts_when_queue_is_full_at_the_moment_of_the_stop(self):
        """The real incident: switch trips while the queue is full -> halt must wait, not raise."""
        m = make()
        m.home_seek(+1, 15000, lambda: True, accel_steps=50)
        self.assertFalse(fake.infinite, "infinite cruise burst left running")

    def test_home_seek_halts_on_exception(self):
        m = make()

        def boom():
            raise ValueError("switch read failed")
        with self.assertRaises(ValueError):
            m.home_seek(+1, 15000, boom, accel_steps=50)
        self.assertFalse(fake.infinite, "infinite cruise burst left running after an exception")

    def test_abort_while_waiting_for_room_still_halts(self):
        m = make()
        fake.full_for = 10 ** 9
        threading.Timer(0.05, m.abort.set).start()
        fake.full_for = 10 ** 9
        # halt ignores abort but the queue never frees -> bounded, returns False
        self.assertFalse(m.halt(wait=0.05))


if __name__ == "__main__":
    unittest.main()
