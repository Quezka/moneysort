#!/usr/bin/env python3
"""Calibrate the camera->base transform at a fixed survey pose (eye-in-hand).

Run ON THE PI (camera mounted on the arm, armd running, x & y homed):
    python3 ~/projects/moneysort/tools/calibrate_camera.py

Because the camera is on the arm it can't see its own tool, so we calibrate by
correspondence at ONE fixed survey pose:

  1. Park the arm at your survey pose (high, camera looking down at the tray).
     Keep it there for every CAPTURE.
  2. Place a single coin in view; the tool captures + detects it -> p_cam
     (3D point in the camera frame).
  3. Jog the nozzle/tool to TOUCH that coin; the tool reads the arm's joint
     angles -> forward kinematics -> p_base (3D in the base frame).
  4. Return to the survey pose, move the coin, repeat (>= 4 spots, spread out).
  5. It solves the rigid transform  base_mm = R @ cam_mm + t  (Umeyama/SVD) and
     saves it to camera_calib.json.

Then: survey -> detect coin -> R@cam+t -> move_to. Re-run this if the camera
mount moves.
"""
import json
import os
import sys
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    import numpy as np
    import cv2
except ImportError:
    sys.exit("needs numpy + opencv -- run: sudo apt install -y python3-opencv")
try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("pyrealsense2 not installed -- run deploy/install_realsense.sh first")

from coin_detect import start, USB2, USB3            # reuse camera pipeline setup
from moneysort.config import JOINTS
from moneysort.domain import kinematics
from moneysort.domain.kinematics import JointAngles

ARMD = "http://localhost:8080"
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                   "camera_calib.json")


def _post(path, body):
    urllib.request.urlopen(
        urllib.request.Request(ARMD + path, data=json.dumps(body).encode(),
                               headers={"Content-Type": "application/json"}),
        timeout=120).read()


def arm_angles():
    """Current joint angles (deg) from armd, as a JointAngles."""
    with urllib.request.urlopen(ARMD + "/status", timeout=5) as r:
        j = json.load(r)["arm"]["joints"]
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
    """Tool-tip position in the base frame (mm) via forward kinematics."""
    p = kinematics.forward(arm_angles())
    return np.array([p.x, p.y, p.z], float)


def detect_one(pipe, align, min_z=0.20, max_z=0.60,
               min_r=18, max_r=70, min_dist=40, param2=32):
    """Capture one frame, return (cam_xyz_mm, pixel) for the single best coin."""
    frames = align.process(pipe.wait_for_frames())
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()
    intr = depth.profile.as_video_stream_profile().intrinsics
    img = np.asanyarray(color.get_data())
    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 5)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=min_dist,
                               param1=100, param2=param2,
                               minRadius=min_r, maxRadius=max_r)
    best = None
    if circles is not None:
        # pick the in-range coin closest to the image centre (least distortion)
        h, w = img.shape[:2]
        for cx, cy, r in np.round(circles[0]).astype(int):
            d = depth.get_distance(int(cx), int(cy))
            if not d or not (min_z <= d <= max_z):
                continue
            score = (cx - w / 2) ** 2 + (cy - h / 2) ** 2
            if best is None or score < best[0]:
                x, y, z = rs.rs2_deproject_pixel_to_point(intr, [int(cx), int(cy)], d)
                best = (score, np.array([x, y, z]) * 1000.0, (int(cx), int(cy)))
    return (best[1], best[2]) if best else (None, None)


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
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        sys.exit("No RealSense device found (check USB/cable).")
    try:
        arm_angles()
    except Exception as e:
        sys.exit(f"can't reach armd at {ARMD} ({e}); is the service up? homed x & y?")

    pipe = start(ctx)
    align = rs.align(rs.stream.color)
    print("\n=== camera->base calibration ===")
    input("Home x & y, park the arm at your SURVEY pose (camera looking down at the "
          "tray). Enter to lock it in...")
    survey = arm_angles()
    print(f"survey pose: x={survey.x:.1f} y={survey.y:.1f} z={survey.z:.1f}")
    print("It auto-returns here before each capture, so keep a hand near the e-stop.\n")

    cam_pts, base_pts = [], []
    try:
        while True:
            goto(survey)                       # every capture from the same pose
            input(f"[point {len(cam_pts)+1}] ONE coin in view. "
                  "Enter to capture (Ctrl-C to finish)...")
            cam, px = detect_one(pipe, align)
            if cam is None:
                print("  no coin detected in range -- adjust and retry.")
                continue
            print(f"  p_cam  = ({cam[0]:.0f},{cam[1]:.0f},{cam[2]:.0f}) mm  px={px}")
            input("  JOG the nozzle to TOUCH that coin (another SSH terminal: "
                  "arm_test.py), then Enter...")
            base = tool_base_mm()
            print(f"  p_base = ({base[0]:.0f},{base[1]:.0f},{base[2]:.0f}) mm")
            cam_pts.append(cam)
            base_pts.append(base)
            print(f"  recorded ({len(cam_pts)} total). Returning to survey pose...\n")
    except KeyboardInterrupt:
        print()
    finally:
        pipe.stop()

    if len(cam_pts) < 4:
        sys.exit(f"need >= 4 points, got {len(cam_pts)} -- not enough to solve.")

    R, t = rigid_transform(cam_pts, base_pts)
    resid = [np.linalg.norm(base_pts[i] - (R @ cam_pts[i] + t)) for i in range(len(cam_pts))]
    print(f"solved from {len(cam_pts)} points.  residual rms = "
          f"{np.sqrt(np.mean(np.square(resid))):.1f} mm  (max {max(resid):.1f} mm)")
    data = {
        "R": R.tolist(), "t": t.tolist(),
        "survey_pose": {"x": survey.x, "y": survey.y, "z": survey.z},
        "n_points": len(cam_pts), "residual_rms_mm": float(np.sqrt(np.mean(np.square(resid)))),
        "note": "base_mm = R @ cam_mm + t ; cam_mm from realsense (metres*1000). "
                "Valid only at the recorded survey_pose. Re-run if the mount moves.",
    }
    with open(OUT, "w") as f:
        json.dump(data, f, indent=2)
    print(f"saved -> {OUT}")


if __name__ == "__main__":
    main()
