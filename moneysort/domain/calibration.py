"""Camera -> base transform (pure maths, no numpy, no I/O besides `load`).

`tools/calibrate_camera.py` solves a rigid transform at one fixed survey pose
(the camera is eye-in-hand) and saves it to `camera_calib.json`. This module
applies it: base_mm = R @ cam_mm + t. It is only valid while the arm sits at the
survey pose, so `at_survey` tells callers whether the current joints are close
enough to trust it.
"""
import json
from dataclasses import dataclass

SURVEY_TOL_DEG = 2.0       # joints may differ this much from the survey pose


@dataclass(frozen=True)
class CameraCalib:
    R: tuple                # 3x3, row-major
    t: tuple                # 3
    survey: dict            # {"x":deg,"y":deg,"z":deg}
    rms_mm: float = 0.0

    def to_base(self, cam_mm):
        """Camera-frame point (mm) -> base-frame point (mm)."""
        return [sum(self.R[i][j] * cam_mm[j] for j in range(3)) + self.t[i]
                for i in range(3)]

    def at_survey(self, joints, tol_deg=SURVEY_TOL_DEG):
        """True if every axis in `joints` ({"x":deg,...}) is within tol of the survey pose."""
        return all(abs(joints[a] - self.survey[a]) <= tol_deg for a in ("x", "y", "z"))


def load(path):
    """Read a calibration file written by calibrate_camera.py; None if absent/unreadable."""
    try:
        with open(path) as f:
            d = json.load(f)
        return CameraCalib(R=tuple(tuple(r) for r in d["R"]), t=tuple(d["t"]),
                           survey=dict(d["survey_pose"]),
                           rms_mm=float(d.get("residual_rms_mm", 0.0)))
    except (OSError, ValueError, KeyError, TypeError):
        return None
