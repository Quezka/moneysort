"""ArmController: owns the Arm, serializes motion, and provides a latched e-stop.

This is the application/use-case layer: it knows nothing about HTTP or the
dashboard. The interface layer drives it; it drives the Arm.
"""
import logging
import os
import threading
import time

from moneysort.app.arm import Arm
from moneysort.domain import jog as jogmath
from moneysort.domain.kinematics import JointAngles

log = logging.getLogger("armd.ctrl")

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
        self.last_error = None               # (hh:mm:ss, what, message) of the latest failure

    def note_error(self, what, msg):
        """Remember (and log) a rejected/failed request so /status can show why nothing moved."""
        self.last_error = (time.strftime("%H:%M:%S"), what, str(msg))
        log.warning("%s failed: %s", what, msg)
        if "TX queue" in str(msg):
            self._schedule_recovery(str(msg))

    # --- recovery from a wedged lgpio pulse queue ---------------------------
    # lgpio can leave an axis's queue "busy" for good (a spinning timing thread, nothing
    # drains, even a cancel doesn't clear it). Nothing short of dropping the GPIO handle
    # fixes that, so we end the process and systemd (Restart=on-failure) starts a fresh one.
    def _schedule_recovery(self, why):
        if getattr(self, "_recovering", False):
            return
        self._recovering = True

        def check():
            time.sleep(1.5)
            for n, m in self.arm.motors.items():             # one more try at a gentle fix
                if m.busy():
                    m.cancel()
            time.sleep(1.0)
            stuck = [n for n, m in self.arm.motors.items() if m.busy()]
            if stuck:
                self.recover(f"axis {','.join(stuck)} still busy after cancel ({why})")
            else:
                log.info("pulse queue cleared by cancel; no restart needed")
                self._recovering = False
        threading.Thread(target=check, daemon=True).start()

    def recover(self, reason="manual"):
        """Restart the daemon (motors disabled first). The dashboard reconnects by itself;
        axes come back un-homed, which the dashboard already offers to fix."""
        log.error("RECOVERY: restarting armd (%s)", reason)
        self._recovering = True
        self.estopped = True
        try:
            self.arm.disable()
        except Exception:
            log.exception("recover: disable failed")
        threading.Timer(0.6, os._exit, [70]).start()          # non-zero: systemd restarts us

    def _run(self, fn, what="move"):
        if self.estopped:
            raise RuntimeError("e-stopped: re-enable before moving")
        if self._move_lock.locked():
            log.info("%s waiting: another move holds the lock (jog thread alive: %s)",
                     what, bool(self._jog_thread and self._jog_thread.is_alive()))
        with self._move_lock:
            self.moving = True
            t0 = time.monotonic()
            log.info("%s start", what)
            try:
                fn()
            except Exception as e:
                self.note_error(what, e)
                raise
            finally:
                self.moving = False
                log.info("%s end (%.2fs)%s", what, time.monotonic() - t0,
                         " [ABORTED]" if self.arm.abort.is_set() else "")

    def move(self, axis, steps, pps=None):
        if axis not in self.arm.motors:
            raise KeyError(f"unknown axis {axis!r}")
        self._run(lambda: self.arm.move(axis, int(steps), max_pps=pps), f"move {axis} {int(steps):+d}")

    def move_many(self, moves, pps=None):
        moves = {a: int(s) for a, s in moves.items() if a in self.arm.motors}
        if not moves:
            raise KeyError("no known axes in move")
        self._run(lambda: self.arm.move_many(moves, max_pps=pps), f"move_many {moves}")

    def home(self, pps=None):
        self._run(lambda: self.arm.home_all(max_pps=pps), "home")

    def return_zero(self, pps=None):
        self._run(lambda: self.arm.return_zero(max_pps=pps), "return_zero")

    def move_to(self, point, pps=None):
        self._run(lambda: self.arm.move_to(point, max_pps=pps),
                  f"move_to ({point.x:.0f},{point.y:.0f},{point.z:.0f})")

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
                    self.note_error("jog", "busy: another move holds the lock")
                    raise RuntimeError("busy: another move is running")
                log.info("jog start rates=%s cart=%s arm_frame=%s", rates, cart, bool(arm_frame))
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
        clamped = set()
        why = "idle"
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
                        if a not in clamped:
                            clamped.add(a)
                            log.warning("jog: axis %s hit its travel limit at %d steps", a,
                                        motors[a].position)
                    else:
                        clamped.discard(a)
                next_t += JOG_DT
                time.sleep(max(0.0, next_t - JOG_LEAD - time.monotonic()))
        except Exception as e:
            why = f"CRASH {e!r}"
            log.exception("jog loop crashed")
            self.note_error("jog", e)
        else:
            if self.estopped or self.arm.abort.is_set():
                why = "e-stop"
        finally:
            t0 = time.monotonic()
            while (any(m.busy() for m in motors.values()) and not self.arm.abort.is_set()
                   and time.monotonic() - t0 < 5.0):
                time.sleep(0.005)
            self.moving = False
            self._move_lock.release()
            log.info("jog end (%s)", why)

    def debug(self):
        """Why-nothing-moves info for /status: lock, jog thread, abort flag, last failure."""
        return {"lock_held": self._move_lock.locked(),
                "jog_alive": bool(self._jog_thread and self._jog_thread.is_alive()),
                "abort": self.arm.abort.is_set(),
                "tx_busy": [n for n, m in self.arm.motors.items() if m.busy()],
                "positions": {n: m.position for n, m in self.arm.motors.items()},
                "last_error": self.last_error}

    def disable(self):
        """Latched emergency stop: cut torque now, refuse moves until enabled.

        Safe to call mid-move: it writes the enable pin (which move() never
        touches), so torque drops immediately; the in-flight pulse train just
        finishes harmlessly into a disabled driver.
        """
        log.warning("E-STOP (moving=%s)", self.moving)
        self.estopped = True
        self.arm.disable()

    def enable(self):
        log.info("motors re-enabled (axes un-homed: re-home before relying on limits)")
        self.arm.enable()
        self.estopped = False

    def zero(self):
        self.arm.zero()

    def find_home(self, axis):
        self._run(lambda: self.arm.find_home(axis), f"find_home {axis}")

    def status(self):
        return {
            "joints": self.arm.angles(),
            "moving": self.moving,
            "enabled": self.arm.enabled,
            "estopped": self.estopped,
            "home": {ax: self.arm.at_home(ax) for ax in self.arm.home_pins},
            "homed": sorted(self.arm.homed),
            "debug": self.debug(),
        }

    def close(self):
        self.arm.close()
