"""Coin detection on a colour frame -- OpenCV only, no RealSense import.

Given a BGR image plus callables to look up depth and deproject a pixel to a 3D
point, find coin-like circles (Hough), keep those whose centre depth is in range
(this rejects the background), and return each with its pixel + camera-frame 3D
point (mm). Kept hardware-free (the caller supplies depth_at / deproject) so the
detection logic can be exercised with a plain image + stub callables.
"""
import cv2
import numpy as np

DEFAULTS = dict(min_r=18, max_r=70, min_dist=40, param2=32, min_z=0.20, max_z=0.60)


def detect_coins(bgr, depth_at, deproject, *, square=True, **params):
    """Detect coins. Returns (coins, annotated_bgr).

    depth_at(px, py) -> metres (0 = none); deproject(px, py, d) -> (x,y,z) metres.
    coins: list of {px, py, r, depth_m, cam_mm=[x,y,z]} in FULL-frame pixels.
    """
    p = {**DEFAULTS, **params}
    H, W = bgr.shape[:2]
    if square and W != H:                       # centre-square crop, drop clutter
        s = min(H, W)
        ox, oy = (W - s) // 2, (H - s) // 2
        img = bgr[oy:oy + s, ox:ox + s]
    else:
        ox = oy = 0
        img = bgr

    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 5)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=p["min_dist"],
        param1=100, param2=p["param2"], minRadius=p["min_r"], maxRadius=p["max_r"])

    out = img.copy()
    coins = []
    if circles is not None:
        for cx, cy, r in np.round(circles[0]).astype(int):
            fx, fy = int(cx + ox), int(cy + oy)         # full-frame pixel
            d = depth_at(fx, fy)
            if not d or not (p["min_z"] <= d <= p["max_z"]):
                cv2.circle(out, (cx, cy), r, (60, 60, 60), 1)   # grey = rejected
                continue
            x, y, z = deproject(fx, fy, d)
            coins.append({"px": fx, "py": fy, "r": int(r), "depth_m": float(d),
                          "cam_mm": [x * 1000.0, y * 1000.0, z * 1000.0]})
            cv2.circle(out, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(out, (cx, cy), 2, (0, 0, 255), 3)
            cv2.putText(out, f"{d:.2f}m", (cx - r, cy - r - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return coins, out
