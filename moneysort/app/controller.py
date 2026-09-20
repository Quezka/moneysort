"""ArmController: owns the Arm, serializes motion, and provides a latched e-stop.

This is the application/use-case layer: it knows nothing about HTTP or the
dashboard. The interface layer drives it; it drives the Arm.
"""
import threading

from moneysort.app.arm import Arm


class ArmController:
    """Owns the Arm, serializes motion, and provides a latched e-stop."""

    def __init__(self):
        self.arm = Arm()
        self._move_lock = threading.Lock()   # one move at a time
        self.moving = False
        self.estopped = False

    def _run(self, fn):
        if self.estopped:
            raise RuntimeError("e-stopped: re-enable before moving")
        with self._move_lock:
            self.moving = True
            try:
                fn()
            finally:
                self.moving = False

    def move(self, axis, steps, pps=None):
        if axis not in self.arm.motors:
            raise KeyError(f"unknown axis {axis!r}")
        self._run(lambda: self.arm.move(axis, int(steps), max_pps=pps))

    def move_many(self, moves, pps=None):
        moves = {a: int(s) for a, s in moves.items() if a in self.arm.motors}
        if not moves:
            raise KeyError("no known axes in move")
        self._run(lambda: self.arm.move_many(moves, max_pps=pps))

    def home(self, pps=None):
        self._run(lambda: self.arm.home_all(max_pps=pps))

    def return_zero(self, pps=None):
        self._run(lambda: self.arm.return_zero(max_pps=pps))

    def move_to(self, point, pps=None):
        self._run(lambda: self.arm.move_to(point, max_pps=pps))

    def disable(self):
        """Latched emergency stop: cut torque now, refuse moves until enabled.

        Safe to call mid-move: it writes the enable pin (which move() never
        touches), so torque drops immediately; the in-flight pulse train just
        finishes harmlessly into a disabled driver.
        """
        self.estopped = True
        self.arm.disable()

    def enable(self):
        self.arm.enable()
        self.estopped = False

    def zero(self):
        self.arm.zero()

    def find_home(self, axis):
        self._run(lambda: self.arm.find_home(axis))

    def status(self):
        return {
            "joints": self.arm.angles(),
            "moving": self.moving,
            "enabled": self.arm.enabled,
            "estopped": self.estopped,
            "home": {ax: self.arm.at_home(ax) for ax in self.arm.home_pins},
            "homed": sorted(self.arm.homed),
        }

    def close(self):
        self.arm.close()
