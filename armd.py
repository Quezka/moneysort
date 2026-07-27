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
    POST /home             -> all joints back to 0
    POST /disable          -> LATCHED e-stop (cut torque, refuse moves)
    POST /enable           -> clear e-stop, re-energize
    POST /reboot /poweroff -> system control

Runs as a systemd service (see deploy/). Jog with arm_test.py, which is now an
HTTP client rather than a direct-GPIO script.
"""
import json
import signal
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import dashboard            # reuse metric helpers, PAGE, system_action
from arm import Arm

PORT = 8080


class ArmController:
    """Owns the Arm, serializes motion, and provides a latched e-stop."""

    def __init__(self):
        self.arm = Arm()
        self._move_lock = threading.Lock()   # one move at a time
        self.moving = False
        self.estopped = False

    def _run(self, fn):
        if self.estopped:
            raise RuntimeError("e-stopped: re-enable before moving")
        with self._move_lock:
            self.moving = True
            try:
                fn()
            finally:
                self.moving = False

    def move(self, axis, steps, pps=None):
        if axis not in self.arm.motors:
            raise KeyError(f"unknown axis {axis!r}")
        self._run(lambda: self.arm.move(axis, int(steps), max_pps=pps))

    def move_many(self, moves, pps=None):
        moves = {a: int(s) for a, s in moves.items() if a in self.arm.motors}
        if not moves:
            raise KeyError("no known axes in move")
        self._run(lambda: self.arm.move_many(moves, max_pps=pps))

    def home(self, pps=None):
        self._run(lambda: self.arm.home_all(max_pps=pps))

    def disable(self):
        """Latched emergency stop: cut torque now, refuse moves until enabled.

        Safe to call mid-move: it writes the enable pin (which move() never
        touches), so torque drops immediately; the in-flight pulse train just
        finishes harmlessly into a disabled driver.
        """
        self.estopped = True
        self.arm.disable()

    def enable(self):
        self.arm.enable()
        self.estopped = False

    def zero(self):
        self.arm.zero()

    def status(self):
        return {
            "joints": self.arm.angles(),
            "moving": self.moving,
            "enabled": self.arm.enabled,
            "estopped": self.estopped,
        }

    def close(self):
        self.arm.close()


def build_status(ctrl):
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
        "arm": {"joints": st["joints"], "moving": st["moving"], "_age": 0.0},
        "time": time.strftime("%H:%M:%S"),
    }


def make_handler(ctrl):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, body, ctype="application/json", code=200):
            data = body.encode() if isinstance(body, str) else body
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(data)))
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
                self._send(json.dumps(build_status(ctrl)))
            else:
                self._send(dashboard.PAGE, "text/html; charset=utf-8")

        def do_POST(self):
            try:
                if self.path.startswith("/disable"):
                    ctrl.disable()
                elif self.path.startswith("/enable"):
                    ctrl.enable()
                elif self.path.startswith("/zero"):
                    ctrl.zero()
                elif self.path.startswith("/move"):
                    b = self._body()
                    if isinstance(b.get("moves"), dict):
                        ctrl.move_many(b["moves"], b.get("pps"))
                    else:
                        ctrl.move(b.get("axis"), b.get("steps", 0), b.get("pps"))
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
    srv = ThreadingHTTPServer(("0.0.0.0", PORT), make_handler(ctrl))
    signal.signal(signal.SIGTERM, lambda *_: srv.shutdown())
    signal.signal(signal.SIGINT, lambda *_: srv.shutdown())
    print(f"armd on http://{dashboard.ip_addr()}:{PORT}  (owns arm, latched e-stop)")
    try:
        srv.serve_forever()
    finally:
        ctrl.close()


if __name__ == "__main__":
    main()
