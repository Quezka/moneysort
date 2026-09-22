"""RealSense D435 owned by the daemon: one process holds the camera and serves
frames to everything else (like armd is the single GPIO owner).

A background thread continuously grabs aligned depth+colour frames and keeps the
latest under a lock. `jpeg()` returns the latest colour as JPEG (for the MJPEG
feed); `detect()` runs coin detection on the latest frame and returns the coins
(camera-frame 3D) plus an annotated JPEG.

All heavy imports are guarded: if pyrealsense2 / OpenCV aren't installed or no
device is present, the Camera comes up `ok=False` with an `error` message and the
daemon keeps running arm-only.
"""
import threading
import time

try:
    import numpy as np
    import cv2
    import pyrealsense2 as rs
    from moneysort import vision
    _LIBS = True
except ImportError as e:                 # noqa: F841 -- reported via Camera.error
    _LIBS = False
    _IMPORT_ERR = str(e)

# (depth w,h,fps), (colour w,h,fps) -- USB3 first, then USB2-friendly for the feed
USB3 = [((640, 480, 30), (640, 480, 30))]
USB2 = [((480, 270, 15), (640, 480, 15)),
        ((480, 270, 15), (424, 240, 15)),
        ((480, 270, 6),  (424, 240, 6))]

JPEG_Q = 70


class Camera:
    def __init__(self):
        self.ok = False
        self.error = None
        self.desc = None
        self._lock = threading.Lock()
        self._color = None
        self._depth = None
        self._intr = None
        self._pipe = None
        self._align = None
        self._run = False
        if not _LIBS:
            self.error = f"camera libs missing ({_IMPORT_ERR})"
            return
        try:
            self._open()
        except Exception as e:
            self.error = f"camera unavailable: {e}"
            return
        self._run = True
        threading.Thread(target=self._loop, daemon=True).start()
        self.ok = True

    def _open(self):
        ctx = rs.context()
        if len(ctx.query_devices()) == 0:
            raise RuntimeError("no RealSense device")
        dev = ctx.query_devices()[0]
        try:
            usb3 = dev.get_info(rs.camera_info.usb_type_descriptor).startswith("3")
        except RuntimeError:
            usb3 = False
        last = None
        for (dw, dh, dfps), (cw, ch, cfps) in (USB3 if usb3 else USB2):
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, dfps)
            cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, cfps)
            try:
                pipe.start(cfg)
                for _ in range(10):
                    pipe.wait_for_frames(2000)
                self._pipe = pipe
                self._align = rs.align(rs.stream.color)
                self.desc = f"colour {cw}x{ch}@{cfps} ({'USB3' if usb3 else 'USB2'})"
                return
            except RuntimeError as e:
                last = e
                try:
                    pipe.stop()
                except RuntimeError:
                    pass
                time.sleep(0.5)
        raise RuntimeError(f"no stream config held ({last})")

    def _loop(self):
        while self._run:
            try:
                frames = self._align.process(self._pipe.wait_for_frames(2000))
                depth = frames.get_depth_frame()
                color = frames.get_color_frame()
                if not depth or not color:
                    continue
                img = np.asanyarray(color.get_data())
                intr = depth.profile.as_video_stream_profile().intrinsics
                with self._lock:
                    self._color, self._depth, self._intr = img, depth, intr
            except RuntimeError:
                time.sleep(0.1)              # transient disconnect / timeout

    def _encode(self, img):
        if img is None:
            return None
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, JPEG_Q])
        return buf.tobytes() if ok else None

    def jpeg(self):
        """Latest colour frame as JPEG bytes, or None if not ready."""
        with self._lock:
            img = None if self._color is None else self._color.copy()
        return self._encode(img)

    def detect(self, **params):
        """Detect coins on the latest frame -> (coins, annotated_jpeg)."""
        with self._lock:
            if self._color is None:
                return [], None
            img = self._color.copy()
            depth, intr = self._depth, self._intr
            coins, annotated = vision.detect_coins(
                img,
                lambda x, y: depth.get_distance(x, y),
                lambda x, y, d: rs.rs2_deproject_pixel_to_point(intr, [x, y], d),
                **params)
        return coins, self._encode(annotated)

    def close(self):
        self._run = False
        if self._pipe is not None:
            try:
                self._pipe.stop()
            except RuntimeError:
                pass
