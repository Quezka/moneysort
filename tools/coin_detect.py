#!/usr/bin/env python3
"""Detect coins in a RealSense frame and locate them in 3D (camera frame).

Run ON THE PI (after python3-opencv is installed):
    python3 ~/projects/moneysort/tools/coin_detect.py [out_dir]

Grabs one aligned depth+colour frame, finds circles (coins) in the colour image
with a Hough transform, reads each centre's depth, and deprojects to a 3D point
in the CAMERA frame (metres). Saves an annotated PNG (+ a depth colormap) you can
scp/view, and prints each coin. This is the perception half of the pipeline; the
camera->base transform then turns these points into move_to targets.

Tune detection with the flags if it misses/over-detects -- start from the saved
image. Headless: no GUI window, just files.
"""
import argparse
import os
import sys
import time

try:
    import numpy as np
    import cv2
except ImportError:
    sys.exit("needs numpy + opencv -- run: sudo apt install -y python3-opencv")
try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("pyrealsense2 not installed -- run deploy/install_realsense.sh first")

USB2 = [((480, 270, 15), (424, 240, 15)), ((480, 270, 6), (424, 240, 6))]
USB3 = [((640, 480, 30), (640, 480, 30))]


def start(ctx):
    dev = ctx.query_devices()[0]
    try:
        usb3 = dev.get_info(rs.camera_info.usb_type_descriptor).startswith("3")
    except RuntimeError:
        usb3 = False
    for (dw, dh, dfps), (cw, ch, cfps) in (USB3 if usb3 else USB2):
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
        try:
            pipe.start(cfg)
            for _ in range(10):
                pipe.wait_for_frames(2000)
            return pipe
        except RuntimeError:
            try:
                pipe.stop()
            except RuntimeError:
                pass
            time.sleep(1.0)
    sys.exit("could not start the camera (USB/power).")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir", nargs="?", default=".", help="where to save PNGs")
    ap.add_argument("--min-r", type=int, default=8, help="min coin radius (px)")
    ap.add_argument("--max-r", type=int, default=45, help="max coin radius (px)")
    ap.add_argument("--min-dist", type=int, default=18, help="min gap between coins (px)")
    ap.add_argument("--param2", type=int, default=30, help="Hough accumulator thresh (lower=more)")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        sys.exit("No RealSense device found (check USB/cable; re-plug).")
    pipe = start(ctx)
    align = rs.align(rs.stream.color)
    try:
        frames = align.process(pipe.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        intr = depth.profile.as_video_stream_profile().intrinsics
        img = np.asanyarray(color.get_data())            # BGR
    finally:
        pipe.stop()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=args.min_dist,
        param1=100, param2=args.param2, minRadius=args.min_r, maxRadius=args.max_r)

    out = img.copy()
    coins = []
    if circles is not None:
        for cx, cy, r in np.round(circles[0]).astype(int):
            d = depth.get_distance(int(cx), int(cy))     # metres
            x, y, z = rs.rs2_deproject_pixel_to_point(intr, [int(cx), int(cy)], d)
            coins.append((cx, cy, r, d, x, y, z))
            cv2.circle(out, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(out, (cx, cy), 2, (0, 0, 255), 3)
            label = f"{d:.2f}m" if d else "no-depth"
            cv2.putText(out, label, (cx - r, cy - r - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

    color_path = os.path.join(args.out_dir, "coins_annotated.png")
    raw_path = os.path.join(args.out_dir, "color.png")
    depth_path = os.path.join(args.out_dir, "depth.png")
    cv2.imwrite(raw_path, img)
    cv2.imwrite(color_path, out)
    dimg = np.asanyarray(depth.get_data())
    cv2.imwrite(depth_path, cv2.applyColorMap(
        cv2.convertScaleAbs(dimg, alpha=0.03), cv2.COLORMAP_JET))

    print(f"detected {len(coins)} coin(s):")
    for cx, cy, r, d, x, y, z in coins:
        if d:
            print(f"  px=({cx:3},{cy:3}) r={r:2}  depth={d:.3f}m  "
                  f"cam-frame xyz=({x:+.3f},{y:+.3f},{z:+.3f})m")
        else:
            print(f"  px=({cx:3},{cy:3}) r={r:2}  no depth at centre")
    print(f"\nsaved: {color_path}\n       {raw_path}\n       {depth_path}")
    print("view/scp them; tune with --param2/--min-r/--max-r if detection is off.")


if __name__ == "__main__":
    main()
