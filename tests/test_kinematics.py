"""Unit tests for the arm kinematics (pure geometry, no hardware).

Run from the repo root:  python3 -m unittest discover tests
"""
import unittest

from moneysort.domain import kinematics as k
from moneysort.domain.kinematics import JointAngles, Point, OutOfReach


class TestForward(unittest.TestCase):
    def test_home_pose(self):
        """y=0,x=0: upper arm up, forearm horizontal, level tool out front."""
        p = k.forward(JointAngles(x=0, y=0, z=0))
        self.assertAlmostEqual(p.x, k.FOREARM_MM + k.TOOL_REACH_MM, places=6)   # r on +X
        self.assertAlmostEqual(p.y, 0.0, places=6)
        self.assertAlmostEqual(p.z, k.UPPER_ARM_MM + k.V_OFFSET_MM, places=6)

    def test_elbow_folded(self):
        """y=0,x=-90: forearm straight down; level tool still reaches out by LT."""
        p = k.forward(JointAngles(x=-90, y=0, z=0))
        self.assertAlmostEqual(p.x, k.TOOL_REACH_MM, places=6)
        self.assertAlmostEqual(p.z, k.UPPER_ARM_MM - k.FOREARM_MM + k.V_OFFSET_MM, places=6)

    def test_base_yaw_rotates_into_XY(self):
        p = k.forward(JointAngles(x=0, y=0, z=90))
        self.assertAlmostEqual(p.x, 0.0, places=6)
        self.assertAlmostEqual(p.y, k.FOREARM_MM + k.TOOL_REACH_MM, places=6)   # r on +Y


class TestRoundTrip(unittest.TestCase):
    def test_fk_then_ik_recovers_angles(self):
        """forward()->inverse() round-trips across the whole joint range."""
        for y in range(0, 91, 10):
            for x in range(-90, 1, 10):
                for z in range(-150, 151, 60):
                    ang = JointAngles(x=float(x), y=float(y), z=float(z))
                    got = k.inverse(k.forward(ang))
                    self.assertAlmostEqual(got.x, ang.x, places=4, msg=str(ang))
                    self.assertAlmostEqual(got.y, ang.y, places=4, msg=str(ang))
                    self.assertAlmostEqual(got.z, ang.z, places=4, msg=str(ang))

    def test_ik_then_fk_hits_target(self):
        target = k.forward(JointAngles(x=-25.0, y=40.0, z=20.0))
        p = k.forward(k.inverse(target))
        self.assertAlmostEqual(p.x, target.x, places=3)
        self.assertAlmostEqual(p.y, target.y, places=3)
        self.assertAlmostEqual(p.z, target.z, places=3)


class TestGridAccuracy(unittest.TestCase):
    """The fit reproduces the measured 3x3 grid to ~1 cm (rms) / <2 cm (max)."""
    GRID = [  # (y, x, r_mm, h_mm)
        (0, 0, 360, 320), (0, -30, 297, 220), (0, -60, 225, 138),
        (30, 0, 430, 307), (30, -30, 380, 208), (30, -60, 320, 125),
        (60, 0, 485, 253), (60, -30, 445, 156), (60, -60, 395, 70),
    ]

    def test_forward_matches_measurements(self):
        errs = []
        for y, x, r, h in self.GRID:
            p = k.forward(JointAngles(x=x, y=y, z=0))
            errs.append(((p.x - r) ** 2 + (p.z - h) ** 2) ** 0.5)
        self.assertLess(max(errs), 25.0)                 # < 2.5 cm worst case
        self.assertLess(sum(errs) / len(errs), 15.0)     # ~1 cm average


class TestReach(unittest.TestCase):
    def test_too_far_raises(self):
        with self.assertRaises(OutOfReach):
            k.inverse(Point(600.0, 0.0, 340.0))          # beyond ~503mm reach

    def test_reachable_helper(self):
        self.assertTrue(k.reachable(k.forward(JointAngles(x=-20, y=30, z=0))))
        self.assertFalse(k.reachable(Point(0.0, 0.0, 5000.0)))


if __name__ == "__main__":
    unittest.main()
