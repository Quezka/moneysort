#!/usr/bin/env python3
"""Minimal Intel RealSense D435 smoke test for the Money Sorter.

Run ON THE PI (after deploy/install_realsense.sh):
    python3 ~/projects/moneysort/tools/realsense_test.py

Opens the camera, streams aligned depth + colour, and prints the depth and the
3D point (camera frame) at the centre pixel -- the primitive the vision pipeline
is built on: pixel -> 3D point via the depth intrinsics.

Chooses the stream config from the negotiated USB type: full 640x480@30 on USB3,
low-bandwidth configs on USB2 (the D435 can't do depth+colour both at 640@30 on
USB2 -- attempting it knocks the camera off the bus). Each config is validated by
actually pulling frames before it's accepted. No numpy needed. Ctrl-C to stop.
"""
import sys
import time

try:
    import pyrealsense2 as rs
except ImportError:
    sys.exit("pyrealsense2 not installed -- run deploy/install_realsense.sh first")

USB3_CONFIGS = [((640, 480, 30), (640, 480, 30))]
USB2_CONFIGS = [
    ((480, 270, 15), (424, 240, 15)),
    ((480, 270, 6),  (424, 240, 6)),
    ((424, 240, 6),  (424, 240, 6)),
]


def start_streaming(configs):
    """Start + validate (pull frames) the first config that actually holds."""
    for (dw, dh, dfps), (cw, ch, cfps) in configs:
        desc = f"depth {dw}x{dh}@{dfps} + colour {cw}x{ch}@{cfps}"
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
        cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
        try:
            pipe.start(cfg)
            for _ in range(10):                 # must actually STREAM, not just start
                pipe.wait_for_frames(2000)
            return pipe, desc
        except RuntimeError as e:
            print(f"  {desc} didn't hold ({e}); trying lower...")
            try:
                pipe.stop()
            except RuntimeError:
                pass
            time.sleep(1.0)
    return None, None


def main():
    ctx = rs.context()
    if len(ctx.query_devices()) == 0:
        sys.exit("No RealSense device found (check USB port + cable; re-plug it).")
    dev = ctx.query_devices()[0]
    name = dev.get_info(rs.camera_info.name)
    try:
        usb = dev.get_info(rs.camera_info.usb_type_descriptor)
    except RuntimeError:
        usb = "?"
    is_usb3 = usb.startswith("3")
    print(f"device: {name}   USB: {usb}  ({'USB3' if is_usb3 else 'USB2 -- reduced bandwidth'})")

    pipe, desc = start_streaming(USB3_CONFIGS if is_usb3 else USB2_CONFIGS)
    if pipe is None:
        sys.exit("Could not hold any stream config -- re-plug the camera and, for "
                 "full res, use a USB3 cable in a blue port.")
    align = rs.align(rs.stream.color)
    print(f"streaming: {desc}. Ctrl-C to stop.\n")

    try:
        while True:
            frames = align.process(pipe.wait_for_frames())
            depth = frames.get_depth_frame()
            if not depth:
                continue
            w, h = depth.get_width(), depth.get_height()
            cx, cy = w // 2, h // 2
            d = depth.get_distance(cx, cy)      # metres (0.0 = no depth)
            intr = depth.profile.as_video_stream_profile().intrinsics
            x, y, z = rs.rs2_deproject_pixel_to_point(intr, [cx, cy], d)
            if d == 0:
                print("centre: no depth (too close/reflective/out of range)      ", end="\r")
            else:
                print(f"centre depth {d:5.3f} m   3D cam-frame "
                      f"x={x:+.3f} y={y:+.3f} z={z:+.3f} m   ", end="\r")
    except KeyboardInterrupt:
        print("\nstopped.")
    finally:
        try:
            pipe.stop()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
