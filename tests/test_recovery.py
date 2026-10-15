"""Wedged pulse queue -> controller restarts the daemon (via os._exit, systemd brings it back)."""
import time
import types
import unittest

try:
    import test_stepper_tx  # noqa: F401  -- installs the shared fake lgpio before the app imports it
except ImportError:
    from tests import test_stepper_tx  # noqa: F401
from moneysort.app import controller as ctl   # noqa: E402


class FakeMotor:
    def __init__(self, busy_after_cancel):
        self.is_busy, self.after, self.cancels = True, busy_after_cancel, 0

    def busy(self):
        return self.is_busy

    def cancel(self):
        self.cancels += 1
        self.is_busy = self.after
        return True


class FakeArm:
    def __init__(self, motors):
        self.motors, self.disabled = motors, 0
        self.abort = types.SimpleNamespace(is_set=lambda: False)

    def disable(self):
        self.disabled += 1


def make(after):
    c = ctl.ArmController.__new__(ctl.ArmController)
    c.arm = FakeArm({"x": FakeMotor(after)})
    c.last_error, c.estopped = None, False
    return c


class TestRecovery(unittest.TestCase):
    def setUp(self):
        self.exits = []
        self._orig = ctl.os._exit
        ctl.os._exit = lambda code: self.exits.append(code)
        self._sleep = ctl.time.sleep
        ctl.time.sleep = lambda s: self._sleep(min(s, 0.02))

    def tearDown(self):
        ctl.os._exit = self._orig
        ctl.time.sleep = self._sleep

    def test_stuck_queue_after_cancel_restarts_the_daemon(self):
        c = make(after=True)                          # cancel does not free it
        c.note_error("move_to", "TX queues stuck: no room for 5.0s (y)")
        self._sleep(1.2)
        self.assertEqual(self.exits, [70])
        self.assertEqual(c.arm.disabled, 0)          # motors stay enabled: the arm must hold its pose

    def test_queue_freed_by_cancel_needs_no_restart(self):
        c = make(after=False)
        c.note_error("jog", "TX queue stuck on GPIO5")
        self._sleep(0.5)
        self.assertEqual(self.exits, [])
        self.assertFalse(c._recovering)

    def test_unrelated_errors_do_not_trigger_it(self):
        c = make(after=True)
        c.note_error("move_to", "joint angles out of range")
        self._sleep(0.5)
        self.assertEqual(self.exits, [])


if __name__ == "__main__":
    unittest.main()
