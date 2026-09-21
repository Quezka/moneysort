"""Forward / inverse kinematics for the arm -- pure geometry, no hardware.

The arm is a base yaw + a 2-link arm with a LEVELLING WRIST: the forearm tilts
with the elbow, but a passive pivot keeps the tool level, so the tool adds a
CONSTANT horizontal reach rather than rotating with the forearm. Fitted to a 3x3
grid of on-arm tool-tip measurements (2026-09-20), rms ~1.0 cm, max ~1.9 cm:

    r = L1*sin(y) + LF*cos(x) + LT      (horizontal reach from the base axis)
    h = L1*cos(y) + LF*sin(x) + D       (height above the base plate)

    y = shoulder, degrees from vertical (0 = up .. 90 = horizontal), range 0..90
    x = elbow,    degrees, range 0..-90 (forearm angle from vertical is 90 - x)
    L1 = upper-arm length, LF = forearm length, LT = level tool's horizontal
    reach, D = base-pivot height + tool vertical offset (h is from the base plate)

Base frame: X, Y = r*(cos z, sin z) with z the base yaw; Z = h. move_to targets
are in this frame, millimetres, Z measured up from the base plate.

NOTE: this is an empirical fit of a linkage arm, not a from-CAD model, so the
lengths are effective values (they won't match tape measurements). Re-fit from a
fresh grid if the mechanism changes; ~1 cm residual is the linkage vs this form.
"""
import math
from dataclasses import dataclass

# --- fitted geometry (mm) -------------------------------------------------
UPPER_ARM_MM = 161.2   # L1
FOREARM_MM = 210.1     # LF
TOOL_REACH_MM = 131.7  # LT  (level tool's constant horizontal reach)
V_OFFSET_MM = 168.2    # D   (base-plate -> effective vertical zero)

# --- joint limits (degrees) -----------------------------------------------
Y_MIN, Y_MAX = 0.0, 90.0
X_MIN, X_MAX = -90.0, 0.0


class OutOfReach(ValueError):
    """Target cannot be reached within link lengths / joint limits."""


@dataclass(frozen=True)
class JointAngles:
    """Joint angles in degrees, matching the axis names used elsewhere."""
    x: float   # elbow
    y: float   # shoulder
    z: float   # base


@dataclass(frozen=True)
class Point:
    """A point in the arm's base frame, millimetres (Z up from the base plate)."""
    x: float
    y: float
    z: float


def forward(angles):
    """Joint angles -> tool-tip Point in the base frame."""
    y = math.radians(angles.y)
    x = math.radians(angles.x)
    r = UPPER_ARM_MM * math.sin(y) + FOREARM_MM * math.cos(x) + TOOL_REACH_MM
    h = UPPER_ARM_MM * math.cos(y) + FOREARM_MM * math.sin(x) + V_OFFSET_MM
    zr = math.radians(angles.z)
    return Point(r * math.cos(zr), r * math.sin(zr), h)


def inverse(target, check_limits=True):
    """Target Point -> JointAngles that reach it.

    Solves base yaw, then the two-link system
        A = L1 sin y + LF cos x ,  B = L1 cos y + LF sin x
    with A = r - LT, B = h - D. Picks the branch within the joint limits.
    Raises OutOfReach if the point is beyond the links or outside the ranges.
    """
    z = math.degrees(math.atan2(target.y, target.x))
    r = math.hypot(target.x, target.y)
    A = r - TOOL_REACH_MM
    B = target.z - V_OFFSET_MM
    d2 = A * A + B * B
    L1, LF = UPPER_ARM_MM, FOREARM_MM

    denom = 2 * LF * math.sqrt(d2) if d2 > 0 else 0.0
    if denom == 0:
        raise OutOfReach("degenerate target")
    cos_off = (d2 + LF * LF - L1 * L1) / denom
    if abs(cos_off) > 1.0 + 1e-9:
        raise OutOfReach(f"unreachable: |cos|={cos_off:.3f}")
    cos_off = max(-1.0, min(1.0, cos_off))
    off = math.acos(cos_off)
    base = math.atan2(B, A)

    best = None
    for xr in (base - off, base + off):            # elbow angle candidates
        yr = math.atan2(A - LF * math.cos(xr), B - LF * math.sin(xr))
        x, y = math.degrees(xr), math.degrees(yr)
        in_range = (Y_MIN - 1e-4 <= y <= Y_MAX + 1e-4
                    and X_MIN - 1e-4 <= x <= X_MAX + 1e-4)
        if in_range:
            return JointAngles(x=x, y=y, z=z)
        if best is None:
            best = JointAngles(x=x, y=y, z=z)

    if check_limits:
        raise OutOfReach(f"joint angles out of range (nearest {best})")
    return best


def reachable(target):
    """True if inverse(target) succeeds within links and joint limits."""
    try:
        inverse(target)
        return True
    except OutOfReach:
        return False
