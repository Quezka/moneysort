"""ArmController: owns the Arm, serializes motion, and provides a latched e-stop.

This is the application/use-case layer: it knows nothing about HTTP or the
dashboard. The interface layer drives it; it drives the Arm.
"""
import threading
import time

from moneysort.app.arm import Arm
from moneysort.domain import jog as jogmath
from moneysort.domain.kinematics import JointAngles

JOG_DT = 0.04            # s per jog slice
JOG_LEAD = 0.012         # queue the next slice this early so the pulse train never starves
JOG_ACCEL = 50000.0      # pps per second (gentle: 0 -> 8000 pps in ~0.16 s). No min-speed
                         # floor: the planner's ramp starts at 400 pps and is 5x harsher than this
JOG_MAX_PPS = 8000.0     # same ceiling the old step-jog used
JOG_TTL = 0.25           # dead-man: motion stops this long after the last /jog


class ArmController:
    """Owns the Arm, serializes motion, and provides a latched e-stop."""

    def __init__(self):
        self.arm = Arm()
        self._move_lock = threading.Lock()   # one move at a time
        self.moving = False
        self.estopped = False
        self._jog_lock = threading.Lock()    # guards the target below
        self._jog_rates = {}                 # joint mode: {axis: deg/s}
        self._jog_cart = None                # cartesian mode: (vx, vy, vz) mm/s
        self._jog_arm_frame = False          # True: cart is (radial, tangential, up), not base X/Y/Z
        self._jog_deadline = 0.0
        self._jog_thread = None

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

    # --- continuous jog (dead-man) -----------------------------------------
    def jog(self, rates=None, cart=None, ttl=None, arm_frame=False):
        """Hold-to-move. Set the target velocity; send again before `ttl` runs out.

        rates: {axis: deg/s} in joint mode, or cart: (vx, vy, vz) mm/s for the tool
        tip (needs x & y homed). With arm_frame the cart vector is (radial, tangential,
        up) relative to where the arm points, so "sideways" always swings the base.
        Empty/zero stops (with a short ramp-down). If no
        new jog arrives within ttl the arm decelerates to rest by itself, so a
        dropped connection or a released key can never leave it running."""
        if self.estopped:
            raise RuntimeError("e-stopped: re-enable before moving")
        rates = {a: float(v) for a, v in (rates or {}).items() if a in self.arm.motors and v}
        cart = tuple(float(c) for c in cart) if cart and any(cart) else None
        if cart and not {"x", "y"} <= self.arm.homed:
            raise RuntimeError("home x & y before cartesian jog (need a position reference)")
        with self._jog_lock:
            self._jog_rates, self._jog_cart, self._jog_arm_frame = rates, cart, bool(arm_frame)
            self._jog_deadline = time.monotonic() + (JOG_TTL if ttl is None else min(float(ttl), 1.0))
            if not (rates or cart):
                self._jog_deadline = 0.0
            running = self._jog_thread is not None and self._jog_thread.is_alive()
            if not running and (rates or cart):
                if not self._move_lock.acquire(blocking=False):
                    raise RuntimeError("busy: another move is running")
                self._jog_thread = threading.Thread(target=self._jog_loop, daemon=True)
                self._jog_thread.start()

    def _jog_targets(self):
        """Target speeds in steps/s per axis for the current jog request."""
        with self._jog_lock:
            rates, cart, live = self._jog_rates, self._jog_cart, time.monotonic() < self._jog_deadline
            arm_frame = self._jog_arm_frame
        if not live:
            return {a: 0.0 for a in self.arm.motors}
        if cart:
            ang = {n: m.angle for n, m in self.arm.motors.items()}   # unrounded
            if arm_frame:
                cart = jogmath.arm_to_base_frame(cart, ang["z"])
            try:
                rates = jogmath.cart_to_joint_rates(JointAngles(**ang), cart)
            except ValueError:                   # singular pose: hold still
                rates = {}
        sps = {a: rates.get(a, 0.0) / 360.0 * m.eff_spr for a, m in self.arm.motors.items()}
        peak = max((abs(v) for v in sps.values()), default=0.0)
        if peak > JOG_MAX_PPS:                   # scale the whole vector: keep the direction
            sps = {a: v * JOG_MAX_PPS / peak for a, v in sps.items()}
        return sps

    def _jog_loop(self):
        """Runs while any axis is moving; holds the move lock for its whole life."""
        self.moving = True
        motors = self.arm.motors
        vel = {a: 0.0 for a in motors}
        carry = {a: 0.0 for a in motors}
        next_t = time.monotonic()
        try:
            while not (self.estopped or self.arm.abort.is_set()):
                target = self._jog_targets()
                if not any(target.values()) and not any(vel.values()):
                    break
                steps = {}
                for a in motors:
                    vel[a] = jogmath.slew(vel[a], target[a], JOG_ACCEL, JOG_DT, 0.0)
                    steps[a], carry[a] = jogmath.take_steps(vel[a], JOG_DT, carry[a])
                done = self.arm.jog_slice({a: n for a, n in steps.items() if n}, JOG_DT)
                for a, n in done.items():
                    if n != steps[a]:            # hit a travel limit: stop that axis
                        vel[a], carry[a] = 0.0, 0.0
                next_t += JOG_DT
                time.sleep(max(0.0, next_t - JOG_LEAD - time.monotonic()))
        finally:
            while any(m.busy() for m in motors.values()) and not self.arm.abort.is_set():
                time.sleep(0.005)
            self.moving = False
            self._move_lock.release()

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
