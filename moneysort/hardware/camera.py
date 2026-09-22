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
        self._depth_scale = 1.0
        self._pipe = None
        self._align = None
        self._run = True
        if not _LIBS:
            self._run = False
            self.error = f"camera libs missing ({_IMPORT_ERR})"
            return
        threading.Thread(target=self._supervise, daemon=True).start()

    def _supervise(self):
        """Keep the camera open and recover automatically from unplug/replug."""
        while self._run:
            try:
                self._open()                 # sets pipe/align/desc, or raises
                self.ok, self.error = True, None
                self._capture()              # runs until a persistent failure
            except Exception as e:
                self.error = f"camera unavailable: {e}"
            self.ok = False
            try:
                if self._pipe is not None:
                    self._pipe.stop()
            except RuntimeError:
                pass
            self._pipe = None
            with self._lock:
                self._color = self._depth = self._intr = None
            if self._run:
                time.sleep(2.0)              # let a re-plug settle, then retry

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
                profile = pipe.start(cfg)
                for _ in range(10):
                    pipe.wait_for_frames(2000)
                self._pipe = pipe
                self._align = rs.align(rs.stream.color)
                self._depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
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

    def _capture(self):
        fails = 0
        while self._run:
            try:
                frames = self._align.process(self._pipe.wait_for_frames(2000))
                depth = frames.get_depth_frame()
                color = frames.get_color_frame()
                if not depth or not color:
                    continue
                # COPY into plain arrays -- holding pyrealsense frames/views starves
                # the frame pool and freezes the stream. intrinsics is a value copy.
                img = np.array(color.get_data())                 # BGR
                dep = np.array(depth.get_data())                 # uint16 HxW
                intr = depth.profile.as_video_stream_profile().intrinsics
                with self._lock:
                    self._color, self._depth, self._intr = img, dep, intr
                fails = 0
            except RuntimeError:
                fails += 1
                if fails > 5:               # persistent disconnect -> reopen device
                    raise
                time.sleep(0.1)

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
        with self._lock:                              # grab copies, then work unlocked
            if self._color is None:
                return [], None
            img = self._color.copy()
            depth, intr, scale = self._depth, self._intr, self._depth_scale

        def depth_at(x, y):
            if 0 <= y < depth.shape[0] and 0 <= x < depth.shape[1]:
                return float(depth[y, x]) * scale     # z16 counts -> metres
            return 0.0

        coins, annotated = vision.detect_coins(
            img, depth_at,
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
