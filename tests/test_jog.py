"""Unit tests for continuous-jog maths (pure, no hardware)."""
import unittest

from moneysort.domain import jog, kinematics
from moneysort.domain.kinematics import JointAngles


class TestSlew(unittest.TestCase):
    def test_starts_at_vmin_then_ramps(self):
        v = jog.slew(0, 5000, 50000, 0.04, 400)
        self.assertEqual(v, 2000)                       # accel*dt, above vmin
        self.assertEqual(jog.slew(2000, 5000, 50000, 0.04, 400), 4000)
        self.assertEqual(jog.slew(4000, 5000, 50000, 0.04, 400), 5000)   # capped at target

    def test_small_first_step_jumps_to_vmin(self):
        self.assertEqual(jog.slew(0, 5000, 1000, 0.04, 400), 400)

    def test_decel_below_vmin_stops(self):
        self.assertEqual(jog.slew(500, 0, 5000, 0.04, 400), 0.0)

    def test_reverse_passes_through_zero_speed_floor(self):
        v = 600.0
        for _ in range(20):
            v = jog.slew(v, -3000, 5000, 0.04, 400)
        self.assertEqual(v, -3000)

    def test_negative_direction(self):
        self.assertEqual(jog.slew(0, -5000, 50000, 0.04, 400), -2000)


class TestTakeSteps(unittest.TestCase):
    def test_carry_conserves_steps(self):
        carry, total = 0.0, 0
        for _ in range(100):
            n, carry = jog.take_steps(410.0, 0.04, carry)     # 16.4 steps/slice
            total += n
        self.assertAlmostEqual(total + carry, 410.0 * 0.04 * 100, places=6)

    def test_truncates_toward_zero(self):
        self.assertEqual(jog.take_steps(-100.0, 0.04, 0.0)[0], -4)


class TestCartRates(unittest.TestCase):
    def test_moves_tip_as_asked(self):
        a = JointAngles(x=-30, y=40, z=20)
        v = (10.0, -5.0, 8.0)                                  # mm/s
        q = jog.cart_to_joint_rates(a, v)
        dt = 0.01
        p0 = kinematics.forward(a)
        p1 = kinematics.forward(JointAngles(a.x + q["x"] * dt, a.y + q["y"] * dt, a.z + q["z"] * dt))
        self.assertAlmostEqual((p1.x - p0.x) / dt, v[0], delta=0.2)
        self.assertAlmostEqual((p1.y - p0.y) / dt, v[1], delta=0.2)
        self.assertAlmostEqual((p1.z - p0.z) / dt, v[2], delta=0.2)

    def test_singular_pose_raises(self):
        """Upper arm vertical + forearm straight down: elbow and shoulder both only change reach."""
        with self.assertRaises(ValueError):
            jog.cart_to_joint_rates(JointAngles(x=-90, y=0, z=0), (1, 0, 0))


if __name__ == "__main__":
    unittest.main()
