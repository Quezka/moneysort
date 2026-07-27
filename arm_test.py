#!/usr/bin/env python3
"""Jog one axis by asking the arm daemon (armd) over HTTP.

The daemon owns the GPIO, so this is now a thin client -- it does NOT touch
hardware directly. armd must be running (systemd service, or `python3 armd.py`).

Usage (run on the Pi):
    python3 arm_test.py <x|y|z> [steps] [pps]
        steps : signed step count (+ / - sets direction), default 800
        pps   : cruise pulses per second, default 4000
"""
import json
import sys
import urllib.request

axis = sys.argv[1] if len(sys.argv) > 1 else "y"
steps = int(sys.argv[2]) if len(sys.argv) > 2 else 800
pps = int(sys.argv[3]) if len(sys.argv) > 3 else 4000

payload = json.dumps({"axis": axis, "steps": steps, "pps": pps}).encode()
req = urllib.request.Request(
    "http://localhost:8080/move", data=payload,
    headers={"Content-Type": "application/json"}, method="POST",
)
print(f"POST /move axis={axis} steps={steps} pps={pps} ...")
try:
    resp = json.load(urllib.request.urlopen(req, timeout=60))
    print("result:", resp)
except urllib.error.HTTPError as e:
    print("rejected:", e.read().decode())
except Exception as e:
    print("error:", e, "(is armd running?)")
