"""center_coin settings: parsing/range checks and the saved-file round trip."""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools"))
import center_params as cp   # noqa: E402


class TestParams(unittest.TestCase):
    def test_parse_types_and_ranges(self):
        self.assertEqual(cp.parse("TOL_PX", "12"), 12)
        self.assertIsInstance(cp.parse("TOL_PX", "12.0"), int)
        self.assertAlmostEqual(cp.parse("GAIN", "0.8"), 0.8)
        self.assertTrue(cp.parse("PUMP_FITTED", "si"))
        self.assertFalse(cp.parse("PUMP_FITTED", "no"))
        for key, bad in (("GAIN", "5"), ("TOL_PX", "abc"), ("PPS", "99999"), ("PUMP_FITTED", "boh")):
            with self.assertRaises(ValueError, msg=(key, bad)):
                cp.parse(key, bad)

    def test_values_fall_back_to_defaults_and_saved_wins(self):
        old = cp.PATH
        cp.PATH = os.path.join(tempfile.mkdtemp(), "s.json")
        try:
            self.assertEqual(cp.values()["GAIN"], cp.PARAMS["GAIN"][0])
            cp.save({"params": {"GAIN": 0.7}, "drop": [1, 2, 3]})
            self.assertEqual(cp.values()["GAIN"], 0.7)
            self.assertEqual(cp.values()["PPS"], cp.PARAMS["PPS"][0])
            self.assertEqual(cp.load()["drop"], [1, 2, 3])
        finally:
            cp.PATH = old

    def test_every_default_is_inside_its_own_range(self):
        for key, (default, kind, lo, hi, _) in cp.PARAMS.items():
            if kind is not bool:
                self.assertTrue(lo <= default <= hi, key)
