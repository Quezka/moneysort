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

# Single-shot capture: high COLOR resolution for detection, LOW fps so it fits
# USB2 bandwidth (we only grab one frame). Depth stays low-res -- it's only read
# at each coin centre and gets aligned/upsampled to the colour frame anyway.
USB3 = [((640, 480, 30), (1280, 720, 30))]
USB2 = [((480, 270, 6), (1280, 720, 6)),
        ((480, 270, 6), (640, 480, 6)),
        ((480, 270, 6), (424, 240, 6))]


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
            for _ in range(30):                  # let exposure + white balance settle
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
    ap.add_argument("--min-r", type=int, default=18, help="min coin radius (px)")
    ap.add_argument("--max-r", type=int, default=70, help="max coin radius (px)")
    ap.add_argument("--min-dist", type=int, default=40, help="min gap between coins (px)")
    ap.add_argument("--param2", type=int, default=32, help="Hough accumulator thresh (lower=more)")
    ap.add_argument("--min-z", type=float, default=0.20, help="keep coins at >= this depth (m)")
    ap.add_argument("--max-z", type=float, default=0.60, help="keep coins at <= this depth (m)")
    ap.add_argument("--no-square", action="store_true", help="don't crop to a centre square")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        sys.exit("No RealSense device found (check USB/cable; re-plug).")
    pipe = start(ctx)
    align = rs.align(rs.stream.color)              # depth -> colour frame + intrinsics
    try:
        frames = align.process(pipe.wait_for_frames())
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        intr = depth.profile.as_video_stream_profile().intrinsics
        full = np.asanyarray(color.get_data())     # BGR, full colour frame
    finally:
        pipe.stop()

    # centre-square crop (keeps pixels on the tray, drops cluttered edges). ox/oy
    # map cropped pixels back to the full frame for depth + deprojection.
    H, W = full.shape[:2]
    if args.no_square:
        ox = oy = 0
        img = full
    else:
        s = min(H, W)
        ox, oy = (W - s) // 2, (H - s) // 2
        img = full[oy:oy + s, ox:ox + s]

    gray = cv2.medianBlur(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 5)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=args.min_dist,
        param1=100, param2=args.param2, minRadius=args.min_r, maxRadius=args.max_r)

    out = img.copy()
    coins, rejected = [], 0
    if circles is not None:
        for cx, cy, r in np.round(circles[0]).astype(int):
            fx, fy = int(cx + ox), int(cy + oy)               # full-frame pixel
            d = depth.get_distance(fx, fy)                     # metres (0 = none)
            keep = d and args.min_z <= d <= args.max_z         # DEPTH GATE
            if not keep:
                rejected += 1
                cv2.circle(out, (cx, cy), r, (60, 60, 60), 1)  # grey = rejected
                continue
            x, y, z = rs.rs2_deproject_pixel_to_point(intr, [fx, fy], d)
            coins.append((cx, cy, r, d, x, y, z))
            cv2.circle(out, (cx, cy), r, (0, 255, 0), 2)
            cv2.circle(out, (cx, cy), 2, (0, 0, 255), 3)
            cv2.putText(out, f"{d:.2f}m", (cx - r, cy - r - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    color_path = os.path.join(args.out_dir, "coins_annotated.png")
    raw_path = os.path.join(args.out_dir, "color.png")
    depth_path = os.path.join(args.out_dir, "depth.png")
    cv2.imwrite(raw_path, img)
    cv2.imwrite(color_path, out)
    dimg = np.asanyarray(depth.get_data())
    cv2.imwrite(depth_path, cv2.applyColorMap(
        cv2.convertScaleAbs(dimg, alpha=0.03), cv2.COLORMAP_JET))

    print(f"detected {len(coins)} coin(s) in range "
          f"[{args.min_z}-{args.max_z} m]; {rejected} rejected by depth gate:")
    for cx, cy, r, d, x, y, z in coins:
        print(f"  px=({cx:4},{cy:4}) r={r:2}  depth={d:.3f}m  "
              f"cam-frame xyz=({x:+.3f},{y:+.3f},{z:+.3f}) m")
    print(f"\nsaved: {color_path}\n       {raw_path}\n       {depth_path}")
    print("tune: --param2 (lower=more circles), --min-r/--max-r (px), "
          "--min-z/--max-z (depth gate).")


if __name__ == "__main__":
    main()
