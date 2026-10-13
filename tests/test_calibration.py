"""Unit tests for the camera->base calibration (pure maths, no hardware)."""
import json
import os
import tempfile
import unittest

from moneysort.domain import calibration as cal

# 90 deg about Z, then translate
CAL = cal.CameraCalib(R=((0, -1, 0), (1, 0, 0), (0, 0, 1)), t=(10, 20, 30),
                      survey={"x": -40.0, "y": 30.0, "z": 5.0})


class TestCalibration(unittest.TestCase):
    def test_to_base(self):
        self.assertEqual(CAL.to_base([1, 2, 3]), [8, 21, 33])

    def test_identity(self):
        c = cal.CameraCalib(R=((1, 0, 0), (0, 1, 0), (0, 0, 1)), t=(0, 0, 0), survey={})
        self.assertEqual(c.to_base([4, 5, 6]), [4, 5, 6])

    def test_at_survey(self):
        self.assertTrue(CAL.at_survey({"x": -39, "y": 31, "z": 5}))
        self.assertFalse(CAL.at_survey({"x": -30, "y": 30, "z": 5}))

    def test_load_roundtrip_and_missing(self):
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "c.json")
            self.assertIsNone(cal.load(p))
            with open(p, "w") as f:
                json.dump({"R": [list(r) for r in CAL.R], "t": list(CAL.t),
                           "survey_pose": CAL.survey, "residual_rms_mm": 3.5}, f)
            c = cal.load(p)
            self.assertEqual(c.to_base([1, 2, 3]), [8, 21, 33])
            self.assertEqual(c.rms_mm, 3.5)
            with open(p, "w") as f:
                f.write("{bad")
            self.assertIsNone(cal.load(p))


if __name__ == "__main__":
    unittest.main()
