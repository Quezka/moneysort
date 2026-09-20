"""Unit tests for the arm kinematics (pure geometry, no hardware).

Run from the repo root:  python3 -m unittest discover tests
"""
import math
import unittest

from moneysort.domain import kinematics as k
from moneysort.domain.kinematics import JointAngles, Point, OutOfReach


class TestForward(unittest.TestCase):
    def test_home_pose_is_the_r_shape(self):
        """y=0,x=0: upper arm up, forearm horizontal -> tool at (r=FOREARM, h=BASE+UPPER)."""
        p = k.forward(JointAngles(x=0, y=0, z=0))
        self.assertAlmostEqual(p.x, k.FOREARM_MM, places=6)   # r along z=0 -> +X
        self.assertAlmostEqual(p.y, 0.0, places=6)
        self.assertAlmostEqual(p.z, k.BASE_HEIGHT_MM + k.UPPER_ARM_MM, places=6)

    def test_elbow_folded_down(self):
        """y=0,x=-90: forearm straight down -> tool on the axis (r=0)."""
        p = k.forward(JointAngles(x=-90, y=0, z=0))
        self.assertAlmostEqual(p.x, 0.0, places=6)
        self.assertAlmostEqual(p.z, k.BASE_HEIGHT_MM + k.UPPER_ARM_MM - k.FOREARM_MM, places=6)

    def test_base_yaw_rotates_into_XY(self):
        p = k.forward(JointAngles(x=0, y=0, z=90))
        self.assertAlmostEqual(p.x, 0.0, places=6)
        self.assertAlmostEqual(p.y, k.FOREARM_MM, places=6)   # r now along +Y
        self.assertAlmostEqual(p.z, k.BASE_HEIGHT_MM + k.UPPER_ARM_MM, places=6)


class TestRoundTrip(unittest.TestCase):
    def test_fk_then_ik_recovers_angles(self):
        """Across the forward workspace, forward()->inverse() round-trips.

        Skip poses whose tool crosses to (or sits on) the base axis: a negative
        signed radius is the same point as a 180 deg base flip, so the yaw
        representation is ambiguous there -- not a forward working pose.
        """
        for y in range(0, 91, 15):
            for x in range(-90, 1, 15):
                r_signed = (k.UPPER_ARM_MM * math.sin(math.radians(y))
                            + k.FOREARM_MM * math.sin(math.radians(k._forearm_phi(x))))
                if r_signed < 5:
                    continue
                for z in range(-150, 151, 60):
                    ang = JointAngles(x=float(x), y=float(y), z=float(z))
                    got = k.inverse(k.forward(ang))
                    self.assertAlmostEqual(got.x, ang.x, places=4, msg=str(ang))
                    self.assertAlmostEqual(got.y, ang.y, places=4, msg=str(ang))
                    self.assertAlmostEqual(got.z, ang.z, places=4, msg=str(ang))

    def test_ik_then_fk_hits_target(self):
        """A target built from valid angles is reached by inverse()->forward()."""
        target = k.forward(JointAngles(x=-20.0, y=40.0, z=25.0))
        p = k.forward(k.inverse(target))
        self.assertAlmostEqual(p.x, target.x, places=3)
        self.assertAlmostEqual(p.y, target.y, places=3)
        self.assertAlmostEqual(p.z, target.z, places=3)


class TestReach(unittest.TestCase):
    def test_too_far_raises(self):
        with self.assertRaises(OutOfReach):
            k.inverse(Point(600.0, 0.0, 340.0))   # beyond 530mm reach

    def test_out_of_joint_range_raises(self):
        # straight up would need the elbow past its limit (x > 0)
        with self.assertRaises(OutOfReach):
            k.inverse(Point(0.0, 0.0, 650.0))

    def test_reachable_helper(self):
        self.assertTrue(k.reachable(k.forward(JointAngles(x=0, y=0, z=0))))   # home tool
        self.assertFalse(k.reachable(Point(0.0, 0.0, 5000.0)))


if __name__ == "__main__":
    unittest.main()
