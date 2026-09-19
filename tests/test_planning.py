"""Unit tests for the pure motion planner (no hardware needed).

Run from the repo root:  python3 -m unittest discover tests
"""
import unittest

from moneysort.domain import planning

MIN_PPS, MAX_PPS, ACCEL = 400, 20000, 800


def total_cycles(segs):
    return sum(cyc for _, cyc in segs)


class TestPlanSegments(unittest.TestCase):
    def test_conserves_steps(self):
        """Every plan must emit exactly abs(steps) pulses -- no drift."""
        for steps in (0, 1, 5, 799, 800, 1600, 1601, 33000, 157005):
            segs = planning.plan_segments(steps, MAX_PPS, MIN_PPS, ACCEL)
            self.assertEqual(total_cycles(segs), abs(steps), f"steps={steps}")

    def test_zero_is_empty(self):
        self.assertEqual(planning.plan_segments(0, MAX_PPS, MIN_PPS, ACCEL), [])

    def test_speeds_within_bounds(self):
        segs = planning.plan_segments(157005, MAX_PPS, MIN_PPS, ACCEL)
        for pps, _ in segs:
            self.assertGreaterEqual(pps, MIN_PPS)
            self.assertLessEqual(pps, MAX_PPS)

    def test_reaches_cruise_speed(self):
        segs = planning.plan_segments(50000, MAX_PPS, MIN_PPS, ACCEL)
        self.assertTrue(any(pps == MAX_PPS for pps, _ in segs))

    def test_cruise_is_chunked(self):
        """Cruise bursts are bounded so an e-stop can interrupt them."""
        chunk = max(1, int(MAX_PPS * planning.CRUISE_CHUNK_S))
        segs = planning.plan_segments(157005, MAX_PPS, MIN_PPS, ACCEL)
        cruise = [cyc for pps, cyc in segs if pps == MAX_PPS]
        self.assertTrue(cruise)
        self.assertTrue(all(cyc <= chunk for cyc in cruise))

    def test_short_move_has_no_cruise_plateau(self):
        """Below 2*accel there's no cruise: the ramp only touches max_pps at its
        apex (one segment), never a run of max_pps cruise chunks."""
        segs = planning.plan_segments(200, MAX_PPS, MIN_PPS, ACCEL)
        self.assertEqual(total_cycles(segs), 200)
        self.assertLessEqual(sum(1 for pps, _ in segs if pps == MAX_PPS), 1)

    def test_sign_does_not_change_profile(self):
        """Direction is the caller's job; the magnitude profile is symmetric."""
        pos = planning.plan_segments(12345, MAX_PPS, MIN_PPS, ACCEL)
        neg = planning.plan_segments(-12345, MAX_PPS, MIN_PPS, ACCEL)
        self.assertEqual(pos, neg)


class TestRampSegs(unittest.TestCase):
    def test_sums_to_ramp_steps(self):
        for ramp in (0, 1, 15, 16, 17, 800):
            segs = planning.ramp_segs(ramp, MAX_PPS, MIN_PPS, up=True)
            self.assertEqual(total_cycles(segs), ramp, f"ramp={ramp}")

    def test_up_accelerates_down_decelerates(self):
        up = [pps for pps, _ in planning.ramp_segs(800, MAX_PPS, MIN_PPS, up=True)]
        down = [pps for pps, _ in planning.ramp_segs(800, MAX_PPS, MIN_PPS, up=False)]
        self.assertEqual(up, sorted(up))
        self.assertEqual(down, sorted(down, reverse=True))


if __name__ == "__main__":
    unittest.main()
