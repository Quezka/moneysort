"""Pure maths for continuous (velocity) jogging -- no hardware, no I/O.

A held key becomes a target velocity; the jog loop slews the real velocity toward
it with a bounded acceleration and turns it into small step slices. Cartesian
jogging converts a tool-tip velocity (mm/s) into joint rates (deg/s) through the
numeric Jacobian of the forward kinematics.
"""
import math

from moneysort.domain import kinematics
from moneysort.domain.kinematics import JointAngles

# |det J| (mm^3/deg^3) below this = the tip can't move as asked (arm folded straight out/in)
SINGULAR_DET = 1.0


def slew(v, target, accel, dt, vmin):
    """Move speed `v` toward `target` by at most accel*dt (all in steps/s).

    A stepper can't start slower than `vmin`: from rest (or when reversing) it
    jumps straight to +/-vmin, and decelerating below vmin stops it.
    """
    if target == v:
        return v
    dv = accel * dt
    nv = v + max(-dv, min(dv, target - v))
    if abs(nv) < vmin:
        if target == 0 or abs(target) < abs(v):
            return 0.0
        return math.copysign(vmin, target)
    return nv


def take_steps(v, dt, carry):
    """Whole steps to emit for one slice at speed v (steps/s); returns (n, new_carry).

    The fractional remainder is carried so slow speeds don't lose steps."""
    x = v * dt + carry
    n = int(x)                       # truncate toward zero
    return n, x - n


def _solve3(m, b):
    """Solve the 3x3 system m @ x = b by Cramer's rule; ValueError if singular."""
    def det(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det(m)
    if abs(d) < SINGULAR_DET:
        raise ValueError("singular pose")
    out = []
    for c in range(3):
        mc = [[b[r] if k == c else m[r][k] for k in range(3)] for r in range(3)]
        out.append(det(mc) / d)
    return out


def cart_to_joint_rates(angles, vel, eps=0.25):
    """Tool-tip velocity -> joint rates at the pose `angles`.

    angles: JointAngles (deg); vel: (vx, vy, vz) mm/s in the base frame.
    Returns {"x": deg/s, "y": deg/s, "z": deg/s}. ValueError near a singularity.
    """
    base = kinematics.forward(angles)
    cols = []
    for name in ("x", "y", "z"):
        d = {"x": angles.x, "y": angles.y, "z": angles.z}
        d[name] += eps
        p = kinematics.forward(JointAngles(**d))
        cols.append(((p.x - base.x) / eps, (p.y - base.y) / eps, (p.z - base.z) / eps))
    jac = [[cols[c][r] for c in range(3)] for r in range(3)]      # mm per degree
    qx, qy, qz = _solve3(jac, list(vel))
    return {"x": qx, "y": qy, "z": qz}
