"""Gripper against a fake lgpio (no hardware): output order and the dead-man timer."""
import sys
import time
import types
import unittest


class FakeLgpio(types.ModuleType):
    class error(Exception):
        pass

    def __init__(self):
        super().__init__("lgpio")
        self.writes = []          # (pin, level) in order, claims included

    def gpio_claim_output(self, h, pin, level=0): self.writes.append((pin, level))
    def gpio_write(self, h, pin, level): self.writes.append((pin, level))
    def gpio_free(self, h, pin): pass


FAKE = FakeLgpio()
sys.modules["lgpio"] = FAKE

from moneysort.hardware.gripper import Gripper  # noqa: E402

PUMP, VALVE = 16, 20


class GripperTest(unittest.TestCase):
    def setUp(self):
        FAKE.writes.clear()
        self.g = Gripper(1, PUMP, VALVE)

    def tearDown(self):
        self.g.release()

    def test_starts_safe(self):
        self.assertEqual(FAKE.writes, [(PUMP, 0), (VALVE, 0)])
        self.assertFalse(self.g.gripping)

    def test_grip_closes_vent_then_runs_pump(self):
        FAKE.writes.clear()
        self.g.grip(ttl=5)
        self.assertEqual(FAKE.writes, [(VALVE, 1), (PUMP, 1)])
        self.assertTrue(self.g.gripping)

    def test_release_stops_pump_before_venting(self):
        self.g.grip(ttl=5)
        FAKE.writes.clear()
        self.g.release()
        self.assertEqual(FAKE.writes, [(PUMP, 0), (VALVE, 0)])
        self.assertFalse(self.g.gripping)

    def test_dead_man_releases(self):
        self.g.grip(ttl=0.05)
        time.sleep(0.2)
        self.assertFalse(self.g.gripping)
        self.assertEqual(FAKE.writes[-2:], [(PUMP, 0), (VALVE, 0)])

    def test_keepalive_defers_release(self):
        self.g.grip(ttl=0.15)
        for _ in range(4):
            time.sleep(0.06)
            self.g.keepalive()
        self.assertTrue(self.g.gripping)


if __name__ == "__main__":
    unittest.main()
