"""Forward / inverse kinematics for the arm -- pure geometry, no hardware.

Kinematic model (to be confirmed against the real arm):

    z = base      : yaw about the vertical axis
    y = shoulder  : pitch of the first link, in the vertical plane
    x = elbow     : pitch of the second link, relative to the first

So it's a base rotation plus a 2-link planar arm in the plane picked by the base
angle. Given the link lengths this reduces to:

    base angle  = atan2(target_y, target_x)
    reach r     = hypot(target_x, target_y)
    (r, height) -> shoulder, elbow  via standard 2-link planar IK

Fill in the geometry below once measured, then implement forward()/inverse().
Everything here stays hardware-free so it can be unit-tested off the Pi.
"""
from dataclasses import dataclass

# --- geometry (MEASURE THESE) ---------------------------------------------
# All lengths in millimetres, measured on the real arm.
BASE_HEIGHT_MM = None     # vertical distance from the base plane to the shoulder pivot
UPPER_ARM_MM = None       # shoulder pivot -> elbow pivot
FOREARM_MM = None         # elbow pivot -> tool tip


@dataclass(frozen=True)
class JointAngles:
    """Target joint angles in degrees, matching the axis names elsewhere."""
    x: float   # elbow
    y: float   # shoulder
    z: float   # base


@dataclass(frozen=True)
class Point:
    """A point in the arm's base frame, millimetres."""
    x: float
    y: float
    z: float


def forward(angles):
    """Joint angles -> tool-tip Point in the base frame."""
    raise NotImplementedError("forward kinematics: implement once geometry is measured")


def inverse(target):
    """Target Point -> JointAngles that reach it (raises if out of reach)."""
    raise NotImplementedError("inverse kinematics: implement once geometry is measured")
