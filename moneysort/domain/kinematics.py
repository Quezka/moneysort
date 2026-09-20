"""Forward / inverse kinematics for the arm -- pure geometry, no hardware.

Kinematic model (measured 2026-09-20; all lengths mm, angles degrees):

    z = base      : yaw about the vertical axis (continuous)
    y = shoulder  : upper-arm angle FROM VERTICAL, 0 = straight up,
                    90 = horizontal-forward (range 0..90)
    x = elbow     : range 0..-90; the forearm holds an ABSOLUTE angle from
                    vertical, phi = 90 - x, independent of the shoulder -- i.e.
                    when the shoulder swings, the forearm keeps its orientation
                    in space (verified on the real arm 2026-09-20)

Reference poses (with base z = 0), tool tip in (r, height) mm:
    (y=0,  x=0)   upper arm up, forearm horizontal  -> (r = FOREARM, h = BASE + UPPER)  the "r" shape
    (y=0,  x=-90) forearm folded straight down      -> (r = 0, h = BASE + UPPER - FOREARM)
    (y=90, x=0)   whole arm straight out horizontal -> (r = UPPER + FOREARM, h = BASE)

Everything in the arm's BASE frame:
    r      = horizontal distance from the base (yaw) axis
    height = distance above the base plane
    X, Y   = r * (cos z, sin z)   -- z is measured from the +X base direction
    Z      = height

The shoulder pivot is assumed to sit ON the base yaw axis (r=0) at BASE_HEIGHT.

"""
import math
from dataclasses import dataclass

# --- geometry (mm) --------------------------------------------------------
# Fitted to on-arm tool-TIP measurements 2026-09-20 (least-squares over three
# shoulder poses spanning 0..87 deg, x=0), NOT tape measurements -- joint offsets
# and the tool tip sitting past the last pivot make the effective pivot-to-pivot
# lengths differ (tape read upper 220 / forearm-to-pivot ~305 / base 120).
# Residuals <=6mm. The forearm length is to the TIP; the last pivot is ~50mm short
# of it. (An x-varying tip pose could further check FOREARM, not yet done.)
BASE_HEIGHT_MM = 177.0    # base plane -> shoulder pivot
UPPER_ARM_MM = 149.0      # shoulder pivot -> elbow pivot        (L1)
FOREARM_MM = 356.0        # elbow pivot -> tool TIP              (L2, incl. tool)

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
    """A point in the arm's base frame, millimetres."""
    x: float
    y: float
    z: float


def _forearm_phi(x):
    """Forearm absolute angle from vertical (degrees).

    The elbow is an absolute joint: the forearm keeps its orientation in space
    regardless of the shoulder, so phi depends only on x.
    """
    return 90.0 - x


def forward(angles):
    """Joint angles -> tool-tip Point in the base frame."""
    a = math.radians(angles.y)                 # upper arm from vertical
    phi = math.radians(_forearm_phi(angles.x))
    r = UPPER_ARM_MM * math.sin(a) + FOREARM_MM * math.sin(phi)
    height = BASE_HEIGHT_MM + UPPER_ARM_MM * math.cos(a) + FOREARM_MM * math.cos(phi)
    zr = math.radians(angles.z)
    return Point(r * math.cos(zr), r * math.sin(zr), height)


def inverse(target, check_limits=True):
    """Target Point -> JointAngles that reach it.

    Solves the base yaw, then a 2-link planar arm in that vertical plane. Uses
    the elbow branch that matches the home pose (elbow bend >= 90, i.e. x <= 0).
    Raises OutOfReach if the point is beyond the links or (when check_limits)
    outside the joint ranges.
    """
    z = math.degrees(math.atan2(target.y, target.x))
    r = math.hypot(target.x, target.y)
    a_comp = r                                  # horizontal (sin) component
    b_comp = target.z - BASE_HEIGHT_MM          # vertical (cos) component
    d = math.hypot(a_comp, b_comp)

    L1, L2 = UPPER_ARM_MM, FOREARM_MM
    if d > L1 + L2 + 1e-9 or d < abs(L1 - L2) - 1e-9:
        raise OutOfReach(f"distance {d:.1f}mm outside [{abs(L1 - L2):.0f}, {L1 + L2:.0f}]")

    # elbow: law of cosines. delta = angle between the two link vectors (0..180);
    # the home pose is the positive branch, giving x = 90 - delta in [-90, 90].
    cos_delta = max(-1.0, min(1.0, (d * d - L1 * L1 - L2 * L2) / (2 * L1 * L2)))
    delta = math.acos(cos_delta)                # radians, 0..pi

    psi = math.atan2(a_comp, b_comp)            # target direction from vertical
    alpha = psi - math.atan2(L2 * math.sin(delta), L1 + L2 * math.cos(delta))

    # forearm absolute angle phi = alpha + delta; elbow is absolute so x = 90 - phi
    y = math.degrees(alpha)
    x = 90.0 - math.degrees(alpha + delta)
    angles = JointAngles(x=x, y=y, z=z)

    if check_limits and not (Y_MIN - 1e-6 <= y <= Y_MAX + 1e-6
                             and X_MIN - 1e-6 <= x <= X_MAX + 1e-6):
        raise OutOfReach(f"joint angles out of range: {angles}")
    return angles


def reachable(target):
    """True if inverse(target) succeeds within links and joint limits."""
    try:
        inverse(target)
        return True
    except OutOfReach:
        return False
