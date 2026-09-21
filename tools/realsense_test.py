#!/usr/bin/env python3
"""Minimal Intel RealSense D435 smoke test for the Money Sorter.

Run ON THE PI (after deploy/install_realsense.sh):
    python3 ~/projects/moneysort/tools/realsense_test.py

Opens the camera, streams aligned depth + colour, and prints the depth and the
3D point (camera frame) at the centre pixel. This is the primitive the vision
pipeline is built on: pixel -> 3D point via the depth intrinsics.

No numpy needed -- uses the frame accessors directly. Ctrl-C to stop.
"""
import sys

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("pyrealsense2 not installed -- run deploy/install_realsense.sh first")

W, H, FPS = 640, 480, 30


def main():
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        sys.exit("No RealSense device found (check USB3 port + cable).")
    name = ctx.query_devices()[0].get_info(rs.camera_info.name)
    print(f"device: {name}")

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, W, H, rs.format.z16, FPS)
    cfg.enable_stream(rs.stream.color, W, H, rs.format.bgr8, FPS)
    align = rs.align(rs.stream.color)          # align depth into the colour frame
    pipe.start(cfg)
    cx, cy = W // 2, H // 2
    print(f"streaming {W}x{H}@{FPS}; centre pixel ({cx},{cy}). Ctrl-C to stop.\n")
    try:
        # a few frames to let auto-exposure settle
        for _ in range(15):
            pipe.wait_for_frames()
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth = frames.get_depth_frame()
            if not depth:
                continue
            d = depth.get_distance(cx, cy)     # metres (0.0 = no depth)
            intr = depth.profile.as_video_stream_profile().intrinsics
            x, y, z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], d)
            if d == 0:
                print("centre: no depth (too close/reflective/out of range)   ", end="\r")
            else:
                print(f"centre depth {d:5.3f} m   3D cam-frame "
                      f"x={x:+.3f} y={y:+.3f} z={z:+.3f} m   ", end="\r")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        pipe.stop()


if __name__ == "__main__":
    main()
