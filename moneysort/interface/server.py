#!/usr/bin/env python3
"""armd - the Money Sorter arm daemon.

A single long-running process that OWNS the arm hardware (all step/dir pins plus
the shared enable) for its entire lifetime. Because it never releases the enable
pin, the emergency stop is LATCHED: once disabled it stays disabled until an
explicit re-enable -- unlike toggling the pin from a short-lived script, which
frees the pin (and thus re-enables) the moment the script exits.

It also serves the dashboard UI and status API (reusing dashboard.py's helpers),
so there is exactly one hardware owner and one web server. Motion is requested
over HTTP instead of by claiming GPIO directly:

    GET  /                 -> dashboard page
    GET  /status           -> system + live arm state (JSON)
    POST /move             -> {"axis":"z","steps":800,"pps":4000}
    POST /detect           -> coins in camera frame (+ base_mm when camera_calib.json exists)
    POST /jog              -> hold-to-move: {"rates":{"x":deg/s,..}} or {"cart":{"x":mm/s,..}} (+"frame":"arm": x=along arm, y=sideways), repeat within ~0.25s
    POST /move_to          -> {"x":..,"y":..,"z":..} tool tip to a point (mm, via IK)
    POST /return_zero      -> drive every axis back to its zero (no switch seek)
    POST /home             -> home all: x/y seek switches, z returns to zero
    POST /disable          -> LATCHED e-stop (cut torque, refuse moves)
    POST /enable           -> clear e-stop, re-energize
    POST /reboot /poweroff -> system control

Runs as a systemd service (see deploy/). Jog with arm_test.py, which is now an
HTTP client rather than a direct-GPIO script.
"""
import json
import math
import os
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from moneysort.config import PORT
from moneysort.app.controller import ArmController
from moneysort.domain import calibration
from moneysort.domain import kinematics
from moneysort.domain.kinematics import Point
from moneysort.hardware.camera import Camera
from moneysort.interface import dashboard   # metric helpers, PAGE, system_action

CALIB_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "camera_calib.json")   # written by tools/calibrate_camera.py

BUILD = str(int(time.time()))               # changes each daemon start -> kiosk auto-reloads


def with_base(coins, joints):
    """Add base_mm to detections when camera_calib.json exists.

    Returns (calib or None, at_survey or None). base_mm is only trustworthy when
    at_survey is true (the camera is eye-in-hand)."""
    calib = calibration.load(CALIB_PATH)               # re-read: no restart after recalibrating
    if not calib:
        return None, None
    for c in coins:
        c["base_mm"] = calib.to_base(c["cam_mm"])
    return calib, calib.at_survey(joints)


def tool_tip(joints):
    """Forward kinematics of the live joint angles -> tip position (mm), or None."""
    try:
        p = kinematics.forward(kinematics.JointAngles(x=joints["x"], y=joints["y"], z=joints["z"]))
    except (KeyError, TypeError):
        return None
    return {"x": round(p.x), "y": round(p.y), "z": round(p.z), "r": round(math.hypot(p.x, p.y))}


def build_status(ctrl, camera):
    mem_used, mem_total = dashboard.mem_pct()
    disk_used, disk_total = dashboard.disk_pct()
    st = ctrl.status()
    return {
        "host": socket.gethostname(),
        "ip": dashboard.ip_addr(),
        "temp": dashboard.cpu_temp_c(),
        "load": dashboard.load_avg(),
        "mem_pct": mem_used, "mem_total": mem_total,
        "disk_pct": disk_used, "disk_total": disk_total,
        "uptime": dashboard.uptime_str(),
        "motors_enabled": st["enabled"],
        "estopped": st["estopped"],
        "arm": {"joints": st["joints"], "moving": st["moving"],
                "homed": st["homed"], "home": st["home"], "_age": 0.0,
                "tip": tool_tip(st["joints"])},
        "camera": {"ok": camera.ok, "desc": camera.desc, "error": camera.error,
                   "calibrated": os.path.exists(CALIB_PATH)},
        "build": BUILD,
        "time": time.strftime("%H:%M:%S"),
    }


def make_handler(ctrl, camera):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype="application/json", code=200, extra=None):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")     # always serve fresh HTML/JSON
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            self.wfile.write(data)

        def _body(self):
            n = int(self.headers.get("Content-Length", 0) or 0)
            if not n:
                return {}
            try:
                return json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return {}

        def do_GET(self):
            if self.path.startswith("/status"):
                self._send(json.dumps(build_status(ctrl, camera)))
            elif self.path.startswith("/camera"):
                self._stream_mjpeg()
            elif self.path.startswith("/snapshot"):
                jpg = camera.jpeg() if camera.ok else None
                if jpg is None:
                    self._send(json.dumps({"error": camera.error or "no frame"}), code=503)
                else:
                    self._send(jpg, "image/jpeg")
            elif self.path.startswith("/detected"):        # annotated frame (JPEG)
                coins, jpg = camera.detect() if camera.ok else ([], None)
                if jpg is None:
                    self._send(json.dumps({"error": camera.error or "no frame"}), code=503)
                else:
                    _, at_survey = with_base(coins, ctrl.status()["joints"])
                    self._send(jpg, "image/jpeg", extra={"X-Coins": json.dumps(
                        {"at_survey": at_survey, "coins": [
                            {"r": c["r"], "cam": c["cam_mm"], "base": c.get("base_mm")}
                            for c in coins]})})
            else:
                self._send(dashboard.PAGE, "text/html; charset=utf-8")

        def _stream_mjpeg(self):
            if not camera.ok:
                self._send(json.dumps({"error": camera.error or "no camera"}), code=503)
                return
            annotate = "detect" in self.path               # /camera?detect=1 -> coins drawn
            period = 0.2 if annotate else 1 / 15.0         # detection is heavier -> ~5fps
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            try:
                while True:
                    jpg = camera.detect()[1] if annotate else camera.jpeg()
                    if jpg:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError):
                pass                                       # client closed the stream

        def do_POST(self):
            try:
                if self.path.startswith("/detect"):
                    coins, _ = camera.detect(**self._body())
                    calib, at_survey = with_base(coins, ctrl.status()["joints"])
                    self._send(json.dumps({"ok": True, "coins": coins,
                                           "calibrated": calib is not None,
                                           "at_survey": at_survey}))
                    return
                if self.path.startswith("/disable"):
                    ctrl.disable()
                elif self.path.startswith("/enable"):
                    ctrl.enable()
                elif self.path.startswith("/zero"):
                    ctrl.zero()
                elif self.path.startswith("/jog"):
                    b = self._body()
                    c = b.get("cart")
                    ctrl.jog(rates=b.get("rates"),
                             cart=(c["x"], c["y"], c["z"]) if c else None, ttl=b.get("ttl"),
                             arm_frame=b.get("frame") == "arm")
                    self._send(json.dumps({"ok": True}))     # hot path: no status
                    return
                elif self.path.startswith("/move_to"):
                    b = self._body()
                    ctrl.move_to(Point(float(b["x"]), float(b["y"]), float(b["z"])),
                                 b.get("pps"))
                elif self.path.startswith("/move"):
                    b = self._body()
                    if isinstance(b.get("moves"), dict):
                        ctrl.move_many(b["moves"], b.get("pps"))
                    else:
                        ctrl.move(b.get("axis"), b.get("steps", 0), b.get("pps"))
                elif self.path.startswith("/find_home"):
                    ctrl.find_home(self._body().get("axis"))
                elif self.path.startswith("/return_zero"):
                    ctrl.return_zero(self._body().get("pps"))
                elif self.path.startswith("/home"):
                    ctrl.home(self._body().get("pps"))
                elif self.path.startswith("/kiosk-exit"):
                    dashboard.exit_kiosk()
                elif self.path.startswith("/reboot"):
                    dashboard.system_action("reboot")
                elif self.path.startswith("/poweroff"):
                    dashboard.system_action("poweroff")
                else:
                    self._send(json.dumps({"error": "not found"}), code=404)
                    return
            except Exception as e:                        # report back to client
                self._send(json.dumps({"ok": False, "error": str(e)}), code=400)
                return
            self._send(json.dumps({"ok": True, **ctrl.status()}))

    return Handler


def main():
    ctrl = ArmController()
    camera = Camera()                            # graceful if no device/libs
    print("camera:", camera.desc if camera.ok else f"off ({camera.error})")
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), make_handler(ctrl, camera))
    srv.daemon_threads = True                    # don't let MJPEG streams block exit

    def shutdown(*_):
        camera.close()                           # release the device promptly on stop
        threading.Thread(target=srv.shutdown, daemon=True).start()   # from ANOTHER thread

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f"armd on http://{dashboard.ip_addr()}:{PORT}  (owns arm + camera, latched e-stop)")
    try:
        srv.serve_forever()
    finally:
        ctrl.close()
        camera.close()


if __name__ == "__main__":
    main()
