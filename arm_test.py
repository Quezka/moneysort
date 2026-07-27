#!/usr/bin/env python3
"""Jog the arm via the daemon (armd) over HTTP. The daemon owns the GPIO.

Single axis:
    python3 arm_test.py <x|y|z> [steps] [pps]

Multiple axes AT ONCE (any arg containing ':'):
    python3 arm_test.py x:400 y:-800 z:1200 [pps]

    steps : signed step count (+ / - sets direction), default 800
    pps   : cruise pulses per second, default 20000
"""
import json
import sys
import urllib.error
import urllib.request

args = sys.argv[1:]

if args and ":" in args[0]:
    # multi-axis: trailing bare number (no ':') is the shared pps
    pps = 20000
    if len(args) > 1 and ":" not in args[-1]:
        pps = int(args.pop())
    moves = {}
    for a in args:
        ax, st = a.split(":")
        moves[ax] = int(st)
    payload = {"moves": moves, "pps": pps}
    desc = f"move_many {moves} @ {pps}"
else:
    axis = args[0] if args else "y"
    steps = int(args[1]) if len(args) > 1 else 800
    pps = int(args[2]) if len(args) > 2 else 20000
    payload = {"axis": axis, "steps": steps, "pps": pps}
    desc = f"move {axis} {steps} @ {pps}"

req = urllib.request.Request(
    "http://localhost:8080/move", data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json"}, method="POST",
)
print("POST /move:", desc, "...")
try:
    print("result:", json.load(urllib.request.urlopen(req, timeout=120)))
except urllib.error.HTTPError as e:
    print("rejected:", e.read().decode())
except Exception as e:
    print("error:", e, "(is armd running?)")
