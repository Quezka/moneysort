#!/usr/bin/env python3
"""Calibrate the camera->base transform at a fixed survey pose (eye-in-hand).

Run ON THE PI (armd running with the camera, x & y homed):
    python3 ~/projects/moneysort/tools/calibrate_camera.py

armd owns the camera, so this asks it for detections over HTTP (POST /detect)
rather than opening the RealSense itself. Because the camera is on the arm it
can't see its own tool, so we calibrate by correspondence at ONE fixed survey
pose:

  1. Home x & y, park the arm at the survey pose (high, camera looking down).
  2. Place a single coin in view; armd detects it -> p_cam (camera-frame 3D).
  3. Jog the nozzle to TOUCH that coin; read the arm's angles -> FK -> p_base.
  4. It auto-returns to the survey pose; move the coin, repeat (>= 4 spots).
  5. Solve the rigid transform base_mm = R @ cam_mm + t (Kabsch/SVD) and save
     camera_calib.json (valid for that survey pose; re-run if the mount moves).
"""
import json
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import numpy as np
except ImportError:
    sys.exit("needs numpy -- run: sudo apt install -y python3-opencv")

from moneysort.config import JOINTS
from moneysort.domain import kinematics
from moneysort.domain.kinematics import JointAngles

ARMD = "http://localhost:8080"
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "camera_calib.json")


def _get(path):
    with urllib.request.urlopen(ARMD + path, timeout=5) as r:
        return json.load(r)


def _post(path, body):
    req = urllib.request.Request(ARMD + path, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        return json.load(r)


def arm_angles():
    j = _get("/status")["arm"]["joints"]
    return JointAngles(x=j["x"], y=j["y"], z=j["z"])


def goto(target):
    """Drive all axes back to the survey pose (joint angles), coordinated."""
    cur = arm_angles()
    moves = {}
    for a in ("x", "y", "z"):
        spr = JOINTS[a]["steps_per_rev"]
        moves[a] = round(getattr(target, a) / 360 * spr) - round(getattr(cur, a) / 360 * spr)
    if any(moves.values()):
        _post("/move", {"moves": moves})


def tool_base_mm():
    p = kinematics.forward(arm_angles())
    return np.array([p.x, p.y, p.z], float)


def detect_cam_mm():
    """Ask armd to detect coins; return the single coin's camera-frame point (mm)."""
    coins = _post("/detect", {}).get("coins", [])
    if not coins:
        return None
    if len(coins) > 1:
        print(f"  ({len(coins)} coins seen -- using the first; place only ONE for calibration)")
    return np.array(coins[0]["cam_mm"], float)


def rigid_transform(P, Q):
    """R,t with base = R@cam + t (rigid, no scale) via SVD. P,Q: Nx3."""
    P, Q = np.asarray(P, float), np.asarray(Q, float)
    cP, cQ = P.mean(0), Q.mean(0)
    H = (P - cP).T @ (Q - cQ)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1, 1, d]) @ U.T
    return R, cQ - R @ cP


def main():
    try:
        cam = _get("/status").get("camera", {})
    except Exception as e:
        sys.exit(f"can't reach armd at {ARMD} ({e}); is the service up? x & y homed?")
    if not cam.get("ok"):
        sys.exit(f"armd camera not ready: {cam.get('error')}")

    print("\n=== camera->base calibration (via armd /detect) ===")
    input("Home x & y, park the arm at your SURVEY pose (camera looking down). "
          "Enter to lock it in...")
    survey = arm_angles()
    print(f"survey pose: x={survey.x:.1f} y={survey.y:.1f} z={survey.z:.1f}")
    print("It auto-returns here before each capture -- keep a hand near the e-stop.\n")

    cam_pts, base_pts = [], []
    try:
        while True:
            goto(survey)
            input(f"[point {len(cam_pts)+1}] ONE coin in view. "
                  "Enter to capture (Ctrl-C to finish)...")
            c = detect_cam_mm()
            if c is None:
                print("  no coin detected -- adjust and retry.")
                continue
            print(f"  p_cam  = ({c[0]:.0f},{c[1]:.0f},{c[2]:.0f}) mm")
            input("  JOG the nozzle to TOUCH that coin (another terminal: arm_test.py), "
                  "then Enter...")
            b = tool_base_mm()
            print(f"  p_base = ({b[0]:.0f},{b[1]:.0f},{b[2]:.0f}) mm")
            cam_pts.append(c)
            base_pts.append(b)
            print(f"  recorded ({len(cam_pts)} total). Returning to survey pose...\n")
    except KeyboardInterrupt:
        print()

    if len(cam_pts) < 4:
        sys.exit(f"need >= 4 points, got {len(cam_pts)}.")

    R, t = rigid_transform(cam_pts, base_pts)
    resid = [float(np.linalg.norm(base_pts[i] - (R @ cam_pts[i] + t))) for i in range(len(cam_pts))]
    rms = float(np.sqrt(np.mean(np.square(resid))))
    print(f"solved from {len(cam_pts)} points.  residual rms = {rms:.1f} mm (max {max(resid):.1f})")
    with open(OUT, "w") as f:
        json.dump({"R": R.tolist(), "t": t.tolist(),
                   "survey_pose": {"x": survey.x, "y": survey.y, "z": survey.z},
                   "n_points": len(cam_pts), "residual_rms_mm": rms,
                   "note": "base_mm = R @ cam_mm + t ; cam_mm from armd /detect. "
                           "Valid only at survey_pose; re-run if the mount moves."}, f, indent=2)
    print(f"saved -> {OUT}")


if __name__ == "__main__":
    main()
