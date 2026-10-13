"""Unit tests for visual centering (pure maths, simulated camera)."""
import math
import random
import unittest

from moneysort.domain import centering as c


class SimCamera:
    """Pixel of a fixed coin as the tool moves: pixel = centre + A @ (coin - tool)."""

    def __init__(self, a, coin_mm, tool_mm, centre=(320, 240), noise=0.0, seed=1):
        self.a, self.coin, self.tool, self.centre = a, coin_mm, list(tool_mm), centre
        self.noise, self.rng = noise, random.Random(seed)

    def move(self, dx, dy):
        self.tool[0] += dx
        self.tool[1] += dy

    def coin_px(self):
        rx, ry = self.coin[0] - self.tool[0], self.coin[1] - self.tool[1]
        n = lambda: self.rng.uniform(-self.noise, self.noise)
        return {"px": self.centre[0] + self.a[0][0] * rx + self.a[0][1] * ry + n(),
                "py": self.centre[1] + self.a[1][0] * rx + self.a[1][1] * ry + n()}


def run_centering(cam, tol=8, probe_mm=12.0, iters=15):
    p0 = cam.coin_px()
    cam.move(probe_mm, 0)
    p1 = cam.coin_px()
    cam.move(0, probe_mm)
    p2 = cam.coin_px()
    j = c.solve_image_jacobian([((probe_mm, 0), (p1["px"] - p0["px"], p1["py"] - p0["py"])),
                                ((0, probe_mm), (p2["px"] - p1["px"], p2["py"] - p1["py"]))])
    for i in range(iters):
        err = c.error_px(cam.coin_px(), cam.centre)
        if c.centered(err, tol):
            return i
        cam.move(*c.step_mm(j, err))
    return None


class TestCentering(unittest.TestCase):
    def test_jacobian_recovered(self):
        a = [[0.0, -3.0], [3.0, 0.0]]                      # camera rotated 90 deg, 3 px/mm
        cam = SimCamera(a, (200, 50), (150, 20))
        p0 = cam.coin_px(); cam.move(10, 0); p1 = cam.coin_px(); cam.move(0, 10); p2 = cam.coin_px()
        j = c.solve_image_jacobian([((10, 0), (p1["px"] - p0["px"], p1["py"] - p0["py"])),
                                    ((0, 10), (p2["px"] - p1["px"], p2["py"] - p1["py"]))])
        for r in range(2):
            for k in range(2):
                self.assertAlmostEqual(j[r][k], -a[r][k], places=6)   # tool moves +d -> coin shifts -A d

    def test_converges_for_rotated_mirrored_scaled_cameras(self):
        for a in ([[3, 0], [0, 3]], [[0, -3], [3, 0]], [[-2, 0], [0, 2]],
                  [[2.1, 1.2], [-1.2, 2.1]], [[0, 5], [5, 0]]):
            cam = SimCamera(a, (230, -40), (180, 10), noise=1.0)
            self.assertIsNotNone(run_centering(cam), a)
            self.assertLess(math.hypot(*c.error_px(cam.coin_px(), cam.centre)), 8 + 2)

    def test_step_is_clipped(self):
        j = [[1.0, 0.0], [0.0, 1.0]]
        dx, dy = c.step_mm(j, (1000, 0), gain=1.0, max_mm=20)
        self.assertAlmostEqual(math.hypot(dx, dy), 20.0)

    def test_degenerate_probes_rejected(self):
        with self.assertRaises(ValueError):                 # image did not move
            c.solve_image_jacobian([((10, 0), (0, 0)), ((0, 10), (0, 0))])
        with self.assertRaises(ValueError):                 # collinear moves
            c.solve_image_jacobian([((10, 0), (5, 5)), ((20, 0), (10, 10))])
        with self.assertRaises(ValueError):
            c.solve_image_jacobian([((10, 0), (5, 5))])

    def test_nearest(self):
        coins = [{"px": 10, "py": 10}, {"px": 300, "py": 230}]
        self.assertEqual(c.nearest(coins, (320, 240)), coins[1])
        self.assertIsNone(c.nearest([], (0, 0)))


if __name__ == "__main__":
    unittest.main()
